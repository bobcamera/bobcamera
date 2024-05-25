#pragma once
#ifndef __MASK_WORKER_H__
#define __MASK_WORKER_H__

#include <filesystem>

#include <rclcpp/rclcpp.hpp>

#include <opencv2/opencv.hpp>

#include "parameter_node.hpp"

class MaskWorker
{
public:
    enum class MaskCheckType
    {
        Disable,
        Enable,
        NoChange
    };

    explicit MaskWorker(ParameterNode & node
                        , const std::function<void(MaskCheckType result, const cv::Mat &)> & user_callback = nullptr)
        : node_(node)
        , user_callback_(user_callback)
    {
    }

    void init(int mask_timer_seconds, const std::string & mask_filename)
    {
        mask_timer_seconds_ = mask_timer_seconds;
        mask_filename_ = mask_filename;
        
        mask_timer_ = node_.create_wall_timer(std::chrono::seconds(mask_timer_seconds_), [this](){mask_timer_callback();});
        mask_timer_callback(); // Calling it the first time
    }

private:
    int mask_timer_seconds_;
    ParameterNode & node_;
    std::function<void(MaskCheckType result, const cv::Mat &)> user_callback_;

    std::string mask_filename_;
    rclcpp::TimerBase::SharedPtr mask_timer_;
    std::optional<std::filesystem::file_time_type> mask_last_modified_time_;
    cv::Mat image_mask_;

    void mask_timer_callback()
    {
        mask_timer_->cancel();

        auto detection_mask_result = mask_set();

        if (user_callback_)
        {
            user_callback_(detection_mask_result, image_mask_);
        }

        mask_timer_->reset();
    }

    inline MaskCheckType mask_set()
    {
        try
        {
            if (!std::filesystem::exists(mask_filename_))
            {
                mask_last_modified_time_.reset();
                return MaskCheckType::Disable;
            }

            auto current_modified_time = std::filesystem::last_write_time(mask_filename_);
            if (mask_last_modified_time_ == current_modified_time) 
            {
                return MaskCheckType::NoChange;
            }
            mask_last_modified_time_ = current_modified_time;

            image_mask_ = cv::imread(mask_filename_, cv::IMREAD_UNCHANGED);
            if (image_mask_.empty())
            {
                mask_last_modified_time_.reset();
                return MaskCheckType::Disable;
            }

            return MaskCheckType::Enable;
        }
        catch (std::exception &cve)
        {
            RCLCPP_ERROR(node_.get_logger(), "Exception on mask_set: %s", cve.what());
            mask_last_modified_time_.reset();
            image_mask_.release();
            return MaskCheckType::Disable;
        }
    }
};

#endif