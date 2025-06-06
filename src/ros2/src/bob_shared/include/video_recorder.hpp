#pragma once

#include <filesystem>

#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <sensor_msgs/msg/image.hpp>

#include <boblib/api/video/VideoWriter.hpp>
#include <boblib/api/video/ffmpeg_video_writer.hpp>

#include <opencv2/opencv.hpp>
#include <boost/circular_buffer.hpp>

class VideoRecorder final
{
public:
    explicit VideoRecorder(size_t pre_buffer_size, bool use_opencv = false)
        : max_pre_buffer_size_(pre_buffer_size)
        , pre_buffer_image_(pre_buffer_size)
        , use_opencv_(use_opencv)
    {
    }

    // Delete copy operations
    VideoRecorder(const VideoRecorder &) = delete;
    VideoRecorder &operator=(const VideoRecorder &) = delete;

    // Default move operations
    VideoRecorder(VideoRecorder &&) = default;
    VideoRecorder &operator=(VideoRecorder &&) = default;

    void add_to_pre_buffer(const boblib::base::Image & img) noexcept
    {
        if (pre_buffer_image_.full())
        {
            pre_buffer_image_.pop_front();
        }
        pre_buffer_image_.push_back(img.toMat());
    }

    [[nodiscard]] bool open_new_video(const std::string &full_path,
                                      const std::string &codec_str,
                                      double video_fps,
                                      const cv::Size &frame_size) noexcept
    {
        if (video_writer_)
        {
            video_writer_->release();
            video_writer_.reset();
        }
        try
        {
            // Assuming boblib::video::Codec can be constructed from codec_str
            auto codec = boblib::video::VideoWriter::codec_from_string(codec_str);

            video_writer_ = std::make_unique<boblib::video::VideoWriter>(full_path, frame_size, codec, video_fps, use_opencv_, false);
            if (!video_writer_->is_open())
            {
                video_writer_.reset();
                return false;
            }
            write_pre_buffer_to_video();
        }
        catch (const std::exception &e)
        {
            video_writer_.reset();
            return false;
        }

        return true;
    }

    void write_frame(const boblib::base::Image &image) noexcept
    {
        if (video_writer_)
        {
            video_writer_->write(image.toMat());
        }
    }

    void close_video() noexcept
    {
        if (video_writer_)
        {
            video_writer_->release();
            video_writer_.reset();
        }
    }

    bool is_recording() const noexcept
    {
        return video_writer_ != nullptr && video_writer_->is_open();
    }

    std::string get_video_backend() const noexcept
    {
        if (video_writer_)
        {
            return video_writer_->get_backend_name();
        }
        return {};
    }

private:
    void write_pre_buffer_to_video()
    {
        if (!video_writer_)
        {
            return;
        }

        while (!pre_buffer_image_.empty())
        {
            if (video_writer_)
            {
                video_writer_->write(pre_buffer_image_.front());
            }
            pre_buffer_image_.pop_front();
        }
    }

    size_t max_pre_buffer_size_;
    std::unique_ptr<boblib::video::VideoWriter> video_writer_;
    boost::circular_buffer<cv::Mat> pre_buffer_image_;
    bool use_opencv_{false}; // Use OpenCV by default
};
