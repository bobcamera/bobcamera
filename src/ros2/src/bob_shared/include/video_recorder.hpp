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
    explicit VideoRecorder(size_t pre_buffer_size)
        : max_pre_buffer_size_(pre_buffer_size)
        , pre_buffer_image_(pre_buffer_size)
    {
        use_opencv_ = false;
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
        else if (video_writer_ffmpeg_)
        {
            video_writer_ffmpeg_->stop();
            video_writer_ffmpeg_.reset();
        }
        try
        {
            if (use_opencv_)
            {
                // Assuming boblib::video::Codec can be constructed from codec_str
                auto codec = boblib::video::VideoWriter::codec_from_string(codec_str);

                video_writer_ = std::make_unique<boblib::video::VideoWriter>(full_path, frame_size, codec, video_fps, false);
                if (!video_writer_->is_open())
                {
                    video_writer_.reset();
                    return false;
                }
            }
            else
            {
                boblib::video::FFmpegVideoWriter::Options options;
                options.outputPath = full_path;
                options.preset = "fast";
                options.codec = "h264";
                options.fps = video_fps;
                options.width = frame_size.width;
                options.height = frame_size.height;
                options.useHardwareAcceleration = true;
                video_writer_ffmpeg_ = std::make_unique<boblib::video::FFmpegVideoWriter>(options);
                video_writer_ffmpeg_->start();
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
        else if (video_writer_ffmpeg_)
        {
            video_writer_ffmpeg_->write_frame(image.toMat());
        }
    }

    void close_video() noexcept
    {
        if (video_writer_)
        {
            video_writer_->release();
            video_writer_.reset();
        }
        if (video_writer_ffmpeg_)
        {
            video_writer_ffmpeg_->stop();
            video_writer_ffmpeg_.reset();
        }
    }

    bool is_recording() const noexcept
    {
        return video_writer_ || video_writer_ffmpeg_;
    }

    std::string get_video_backend() const noexcept
    {
        if (video_writer_)
        {
            return video_writer_->get_backend_name();
        }
        else if (video_writer_ffmpeg_)
        {
            return video_writer_ffmpeg_->get_codec_name();
        }
        return {};
    }

private:
    void write_pre_buffer_to_video()
    {
        if (!video_writer_ || !video_writer_ffmpeg_)
        {
            return;
        }

        while (!pre_buffer_image_.empty())
        {
            if (video_writer_)
            {
                video_writer_->write(pre_buffer_image_.front());
            }
            else if (video_writer_ffmpeg_)
            {
                video_writer_ffmpeg_->write_frame(pre_buffer_image_.front());
            }
            pre_buffer_image_.pop_front();
        }
    }

    size_t max_pre_buffer_size_;
    std::unique_ptr<boblib::video::VideoWriter> video_writer_;
    boost::circular_buffer<cv::Mat> pre_buffer_image_;
    bool use_opencv_{false}; // Use OpenCV by default

    std::unique_ptr<boblib::video::FFmpegVideoWriter> video_writer_ffmpeg_;
};
