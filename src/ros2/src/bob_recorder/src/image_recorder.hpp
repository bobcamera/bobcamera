#include <opencv2/opencv.hpp>

class ImageRecorder {
public:
    ImageRecorder(int pre_buffer_size) : max_pre_buffer_size_(pre_buffer_size)
    {
        pre_buffer_ptr_ = std::make_unique<std::deque<cv::Mat>>();
        heatmap_accumulator_ = cv::Mat();
        frame_for_drawing_ = cv::Mat();
    }

    void accumulate_mask(const cv::Mat& fg_mask) 
    {
        if (heatmap_accumulator_.empty()) 
        {
            heatmap_accumulator_ = cv::Mat::zeros(fg_mask.size(), fg_mask.type());
        }

        cv::add(heatmap_accumulator_, fg_mask, heatmap_accumulator_);
    }

    void reset() 
    {
        heatmap_accumulator_ = cv::Mat::zeros(heatmap_accumulator_.size(), heatmap_accumulator_.type());
    }

    bool write_image(const std::string& full_path)
    {
        cv::Mat converted_heatmap;
        if (heatmap_accumulator_.channels() == 1) 
        {
            cv::cvtColor(heatmap_accumulator_, converted_heatmap, cv::COLOR_GRAY2BGR);
        } 

        cv::Mat overlay;
        cv::addWeighted(frame_for_drawing_, 0.5, converted_heatmap, 0.5, 0, overlay);

        if (cv::imwrite(full_path, overlay)) 
        {
            return true;
        } 
        else 
        {
            return false;
        }
    }

    void update_frame_for_drawing(const cv::Mat& img) 
    {
        frame_for_drawing_ = img.clone();
    }

    void add_to_pre_buffer(const cv::Mat& img)
    {
        if (pre_buffer_ptr_->size() >= max_pre_buffer_size_) 
        {
            pre_buffer_ptr_->pop_front();
        }
        pre_buffer_ptr_->push_back(img.clone());
    }

    void accumulate_pre_buffer_images() 
    {
        for (const auto& img : *pre_buffer_ptr_) 
        {
            accumulate_mask(img);
        }
        pre_buffer_ptr_->clear();
    }

private:
    std::unique_ptr<std::deque<cv::Mat>> pre_buffer_ptr_;
    size_t max_pre_buffer_size_;
    cv::Mat heatmap_accumulator_;
    cv::Mat frame_for_drawing_;
};
