#pragma once

#include <filesystem>

#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <sensor_msgs/msg/image.hpp>

#include <boblib/api/video/VideoWriter.hpp>

#include <deque>
#include <opencv2/opencv.hpp>

#include <deque>
#include <memory>
#include <opencv2/opencv.hpp>

class VideoRecorder final
{
public:
    explicit VideoRecorder(size_t pre_buffer_size)
        : max_pre_buffer_size_(pre_buffer_size) 
    {
    }

    // Delete copy operations
    VideoRecorder(const VideoRecorder &) = delete;
    VideoRecorder &operator=(const VideoRecorder &) = delete;

    // Default move operations
    VideoRecorder(VideoRecorder &&) = default;
    VideoRecorder &operator=(VideoRecorder &&) = default;

    void add_to_pre_buffer(const cv::Mat && img) noexcept
    {
        if (pre_buffer_mat_.size() >= max_pre_buffer_size_)
        {
            pre_buffer_mat_.pop_front();
        }
        pre_buffer_mat_.push_back(std::move(img));
    }

    void add_to_pre_buffer(const boblib::base::Image && img) noexcept
    {
        if (pre_buffer_image_.size() >= max_pre_buffer_size_)
        {
            pre_buffer_image_.pop_front();
        }
        pre_buffer_image_.push_back(std::move(img));
    }

    [[nodiscard]] bool open_new_video(const std::string &full_path,
                                      const std::string &codec_str,
                                      double video_fps,
                                      const cv::Size &frame_size)
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

            video_writer_ = std::make_unique<boblib::video::VideoWriter>(full_path, frame_size, codec, video_fps, false);

            if (!video_writer_->is_open())
            {
                video_writer_.reset();
                return false;
            }
        }
        catch (const std::exception &e)
        {
            video_writer_.reset();
            return false;
        }

        write_pre_buffer_to_video();
        return true;
    }

    void write_frame(const cv::Mat & frame)
    {
        if (video_writer_ && video_writer_->is_open())
        {
            video_writer_->write(frame);
        }
    }

    void write_frame(const boblib::base::Image & image)
    {
        if (video_writer_ && video_writer_->is_open())
        {
            video_writer_->write(image);
        }
    }

    void clear_pre_buffer()
    {
        pre_buffer_mat_.clear();
    }

    void close_video()
    {
        if (video_writer_)
        {
            video_writer_->release();
            video_writer_.reset();
        }
    }

    bool is_recording()
    {
        return video_writer_ != nullptr;
    }

private:
    void write_pre_buffer_to_video()
    {
        if (!video_writer_ || !video_writer_->is_open())
        {
            return;
        }
        for (const auto & img : pre_buffer_mat_)
        {
            video_writer_->write(img);
        }
        pre_buffer_mat_.clear();
    }

    size_t max_pre_buffer_size_;
    std::unique_ptr<boblib::video::VideoWriter> video_writer_;
    std::deque<cv::Mat> pre_buffer_mat_;
    std::deque<boblib::base::Image> pre_buffer_image_;
};

// class VideoRecorder final
// {
// public:
//     VideoRecorder(int pre_buffer_size)
//         : max_pre_buffer_size_(pre_buffer_size)
//     {
//         pre_buffer_ptr_ = std::make_unique<std::deque<std::unique_ptr<cv::Mat>>>();
//     }

//     ~VideoRecorder()
//     {
//     }

//     void add_to_pre_buffer(const cv::Mat & img)
//     {
//         if (pre_buffer_ptr_->size() >= max_pre_buffer_size_)
//         {
//             pre_buffer_ptr_->pop_front();
//         }
//         pre_buffer_ptr_->push_back(std::make_unique<cv::Mat>(img.clone()));
//     }

//     bool open_new_video(const std::string & full_path, const std::string & /*codec_str*/, double video_fps, const cv::Size & frame_size)
//     {
//         //const int codec = cv::VideoWriter::fourcc(codec_str[0], codec_str[1], codec_str[2], codec_str[3]);
//         if (video_writer_ptr_)
//         {
//             video_writer_ptr_->release();
//         }
//         video_writer_ptr_ = std::make_unique<boblib::video::VideoWriter>(full_path, frame_size, boblib::video::Codec::AVC1, video_fps);

//         write_pre_buffer_to_video();

//         return true;
//     }

//     void write_pre_buffer_to_video()
//     {
//         for (const auto& img_ptr : *pre_buffer_ptr_)
//         {
//             video_writer_ptr_->write(*img_ptr);
//         }
//         pre_buffer_ptr_->clear();
//     }

//     void write_frame(const cv::Mat & frame)
//     {
//         if (video_writer_ptr_->is_open())
//         {
//             video_writer_ptr_->write(frame);
//         }
//     }

//     void clear_pre_buffer()
//     {
//         pre_buffer_ptr_->clear();
//     }

//     void close_video()
//     {
//         video_writer_ptr_->release();
//     }

// private:

//     size_t max_pre_buffer_size_;
//     std::unique_ptr<boblib::video::VideoWriter> video_writer_ptr_;

//     std::unique_ptr<std::deque<std::unique_ptr<cv::Mat>>> pre_buffer_ptr_;
// };
