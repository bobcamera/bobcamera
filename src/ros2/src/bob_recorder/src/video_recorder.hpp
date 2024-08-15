#include <filesystem>

#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <sensor_msgs/msg/image.hpp>

#include <boblib/api/video/VideoWriter.hpp>

class VideoRecorder 
{
public:
    VideoRecorder(int pre_buffer_size) : max_pre_buffer_size_(pre_buffer_size)
    {
        pre_buffer_ptr_ = std::make_unique<std::deque<std::unique_ptr<cv::Mat>>>();
    }

    ~VideoRecorder()
    {
        if (video_writer_ptr_)
        {
            video_writer_ptr_->release();
        }
    }

    void add_to_pre_buffer(const cv::Mat& img)
    {
        if (pre_buffer_ptr_->size() >= max_pre_buffer_size_) 
        {
            pre_buffer_ptr_->pop_front();
        }
        pre_buffer_ptr_->push_back(std::make_unique<cv::Mat>(img.clone()));
    }

    bool open_new_video(const std::string& full_path, const std::string& /*codec_str*/, double video_fps, const cv::Size& frame_size)
    {
        //const int codec = cv::VideoWriter::fourcc(codec_str[0], codec_str[1], codec_str[2], codec_str[3]);
        if (video_writer_ptr_)
        {
            video_writer_ptr_->release();
        }
        video_writer_ptr_ = std::make_unique<boblib::video::VideoWriter>(full_path, frame_size, boblib::video::Codec::AVC1, video_fps);

        write_pre_buffer_to_video();
        return true;
    }

    void write_pre_buffer_to_video() 
    {
        for (const auto& img_ptr : *pre_buffer_ptr_) 
        {
            video_writer_ptr_->write(*img_ptr);
        }
        pre_buffer_ptr_->clear();
    }

    void write_frame(const cv::Mat& frame)
    {
        if (video_writer_ptr_->is_open())
        {
            video_writer_ptr_->write(frame);
        }
    }

    void clear_pre_buffer()
    {
        pre_buffer_ptr_->clear();
    }

    void close_video()
    {
        video_writer_ptr_->release();
    }

private:

    size_t max_pre_buffer_size_;
    std::unique_ptr<boblib::video::VideoWriter> video_writer_ptr_;

    std::unique_ptr<std::deque<std::unique_ptr<cv::Mat>>> pre_buffer_ptr_;
};

