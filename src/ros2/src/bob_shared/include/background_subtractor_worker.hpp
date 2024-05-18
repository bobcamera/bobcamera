#pragma once
#ifndef __BACKGROUND_SUBTRACTOR_WORKER_H__
#define __BACKGROUND_SUBTRACTOR_WORKER_H__

#include <filesystem>

#include <cv_bridge/cv_bridge.hpp>

#include "parameter_node.hpp"
#include "image_utils.hpp"
#include "background_subtractor_companion.hpp"

#include <sensor_msgs/msg/region_of_interest.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <bob_interfaces/msg/detector_state.hpp>
#include <bob_interfaces/msg/detector_b_box_array.hpp>

#include <boblib/api/bgs/bgs.hpp>
#include <boblib/api/bgs/WeightedMovingVariance/WeightedMovingVarianceUtils.hpp>
#include <boblib/api/blobs/connectedBlobDetection.hpp>

struct BackgroundSubtractorWorkerParams
{
    enum class BGSType
    {
        Unknown,
        Vibe,
        WMV
    };

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher;
    rclcpp::Publisher<bob_interfaces::msg::DetectorBBoxArray>::SharedPtr detection_publisher;
    rclcpp::Publisher<bob_interfaces::msg::DetectorState>::SharedPtr state_publisher;
    rclcpp::Publisher<sensor_msgs::msg::RegionOfInterest>::SharedPtr roi_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_resized_publisher;

    BGSType bgs_type;
    std::string sensitivity;
    SensitivityConfigCollection sensitivity_collection;
    bool mask_enable_override = true;
    bool mask_enable_roi;
    std::string mask_filename;
    int resize_height;
};

class BackgroundSubtractorWorker
{
public:
    BackgroundSubtractorWorker(ParameterNode & node, BackgroundSubtractorWorkerParams & params)
        : node_(node)
        , params_(params)
    {
    }

    void init()
    {
        mask_timer_ = node_.create_wall_timer(std::chrono::seconds(60), std::bind(&BackgroundSubtractorWorker::mask_timer_callback, this));
        mask_timer_callback(); // Calling it the first time
    }

    void init_bgs(const std::string & bgs)
    {
        ready_ = false;
        if (bgs == "vibe")
        {
            params_.bgs_type = BackgroundSubtractorWorkerParams::BGSType::Vibe;
        }
        else
        {
            params_.bgs_type = BackgroundSubtractorWorkerParams::BGSType::WMV;
        }
        if (!params_.sensitivity.empty())
        {
            bgsPtr = createBGS(params_.bgs_type);
        }
        ready_ = true;
    }

    bool init_sensitivity(const std::string & sensitivity)
    {
        if (sensitivity.empty() || (sensitivity.length() == 0))
        {
            RCLCPP_DEBUG(node_.get_logger(), "Ignoring sensitivity change request, EMPTY VALUE");
            return false;
        }

        if(params_.sensitivity == sensitivity)
        {
            RCLCPP_DEBUG(node_.get_logger(), "Ignoring sensitivity change request, NO CHANGE");
            return false;            
        }
        else
        {
            RCLCPP_INFO(node_.get_logger(), "Performing sensitivity change request, from: %s to: %s", params_.sensitivity.c_str(), sensitivity.c_str());
        }

        if (!params_.sensitivity_collection.configs.contains(sensitivity))
        {
            RCLCPP_ERROR(node_.get_logger(), "Unknown config specified: %s", sensitivity.c_str());
            return false;
        }

        ready_ = false;
        params_.sensitivity = sensitivity;

        SensitivityConfig & config = params_.sensitivity_collection.configs.at(sensitivity);

        wmv_params_ = std::make_unique<boblib::bgs::WMVParams>(config.sensitivity.wmv_enableWeight, config.sensitivity.wmv_enableThreshold, config.sensitivity.wmv_threshold, config.sensitivity.wmv_weight1, config.sensitivity.wmv_weight2, config.sensitivity.wmv_weight3);
        vibe_params_ = std::make_unique<boblib::bgs::VibeParams>(config.sensitivity.vibe_threshold, config.sensitivity.vibe_bgSamples, config.sensitivity.vibe_requiredBGSamples, config.sensitivity.vibe_learningRate);

        bgsPtr = createBGS(params_.bgs_type);

        blob_params_ = std::make_unique<boblib::blobs::ConnectedBlobDetectionParams>(config.sensitivity.blob_sizeThreshold, config.sensitivity.blob_areaThreshold, config.sensitivity.blob_minDistance, config.sensitivity.blob_maxBlobs);
        blob_detector_ptr_ = std::make_unique<boblib::blobs::ConnectedBlobDetection>(*blob_params_);

        median_filter_ = config.sensitivity.median_filter;
        ready_ = true;
        return true;
    }

    void restart()
    {
        bgsPtr->restart();
    }

    void imageCallback(const std_msgs::msg::Header & header, const cv::Mat & img)
    {
        if (ready_)
        {
            try
            {
                cv::Mat gray_img;
                if (img.channels() == 1)
                {
                    gray_img = img;
                }
                else
                {
                    cv::cvtColor(img, gray_img, cv::COLOR_BGR2GRAY);
                }

                if (!ros_cv_foreground_mask_ || (gray_img.size() != ros_cv_foreground_mask_->image_ptr->size()))
                {
                    ros_cv_foreground_mask_ = std::make_unique<RosCvImageMsg>(gray_img, sensor_msgs::image_encodings::MONO8, false);
                }
                ros_cv_foreground_mask_->msg_ptr->header = header;

                bgsPtr->apply(gray_img, *ros_cv_foreground_mask_->image_ptr, (params_.mask_enable_roi && mask_enabled_) ? grey_mask_ : cv::Mat());

                node_.publish_if_subscriber(params_.image_publisher, *ros_cv_foreground_mask_->msg_ptr);

                publish_resized_frame(*ros_cv_foreground_mask_);

                if (median_filter_)
                {
                    cv::medianBlur(*ros_cv_foreground_mask_->image_ptr, *ros_cv_foreground_mask_->image_ptr, 3);
                }

                bob_interfaces::msg::DetectorState state;
                state.sensitivity = params_.sensitivity;
                bob_interfaces::msg::DetectorBBoxArray bbox2D_array;
                bbox2D_array.header = header;
                bbox2D_array.image_width = gray_img.size().width;
                bbox2D_array.image_height = gray_img.size().height;
                std::vector<cv::Rect> bboxes;

                boblib::blobs::DetectionResult det_result = blob_detector_ptr_->detect(*ros_cv_foreground_mask_->image_ptr, bboxes);
                if (det_result == boblib::blobs::DetectionResult::Success)
                {
                    state.max_blobs_reached = false;
                    add_bboxes(bbox2D_array, bboxes);
                }
                else if (det_result == boblib::blobs::DetectionResult::MaxBlobsReached)
                {
                    state.max_blobs_reached = true;
                }
                
                params_.state_publisher->publish(state);
                params_.detection_publisher->publish(bbox2D_array);            
            }
            catch (std::exception &cve)
            {
                RCLCPP_ERROR(node_.get_logger(), "Exception: %s", cve.what());
            }
        }
    }

private:
    enum class MaskCheckType
    {
        Disable,
        Enable,
        NoChange
    };

    ParameterNode & node_;

    BackgroundSubtractorWorkerParams & params_;

    bool ready_;
    bool mask_enabled_;
    bool median_filter_;
    std::unique_ptr<boblib::bgs::CoreBgs> bgsPtr{nullptr};
    std::unique_ptr<boblib::blobs::ConnectedBlobDetection> blob_detector_ptr_{nullptr};
    std::unique_ptr<boblib::bgs::VibeParams> vibe_params_;
    std::unique_ptr<boblib::bgs::WMVParams> wmv_params_;
    std::unique_ptr<boblib::blobs::ConnectedBlobDetectionParams> blob_params_;

    std::unique_ptr<RosCvImageMsg> ros_cv_foreground_mask_;

    rclcpp::TimerBase::SharedPtr mask_timer_;
    std::optional<std::filesystem::file_time_type> mask_last_modified_time_;
    cv::Mat grey_mask_;
    cv::Rect bounding_box_;

    inline void add_bboxes(bob_interfaces::msg::DetectorBBoxArray &bbox2D_array, const std::vector<cv::Rect> &bboxes)
    {
        for (const auto &bbox : bboxes)
        {
            bob_interfaces::msg::DetectorBBox bbox_msg;
            bbox_msg.x = bbox.x;
            bbox_msg.y = bbox.y;
            bbox_msg.width = bbox.width;
            bbox_msg.height = bbox.height;
            bbox2D_array.detections.push_back(bbox_msg);
        }
    }

    std::unique_ptr<boblib::bgs::CoreBgs> createBGS(BackgroundSubtractorWorkerParams::BGSType _type)
    {
        switch (_type)
        {
        case BackgroundSubtractorWorkerParams::BGSType::Vibe:
            return std::make_unique<boblib::bgs::Vibe>(*vibe_params_);
        case BackgroundSubtractorWorkerParams::BGSType::WMV:
            return std::make_unique<boblib::bgs::WeightedMovingVariance>(*wmv_params_);
        default:
            return nullptr;
        }
    }

    inline void roi_calculation()
    {
        if (params_.mask_enable_roi && mask_enabled_)
        {
            sensor_msgs::msg::RegionOfInterest roi_msg;
            if (grey_mask_.empty())
            {
                roi_msg.x_offset = 0;
                roi_msg.y_offset = 0;
                roi_msg.width = 0;
                roi_msg.height = 0;
            }
            else
            {
                bounding_box_ = cv::Rect(grey_mask_.cols, grey_mask_.rows, 0, 0);                                
                for (int y = 0; y < grey_mask_.rows; ++y) 
                {
                    for (int x = 0; x < grey_mask_.cols; ++x) 
                    {
                        if (grey_mask_.at<uchar>(y, x) == 255) 
                        { 
                            bounding_box_.x = std::min(bounding_box_.x, x);
                            bounding_box_.y = std::min(bounding_box_.y, y);
                            bounding_box_.width = std::max(bounding_box_.width, x - bounding_box_.x);
                            bounding_box_.height = std::max(bounding_box_.height, y - bounding_box_.y);
                        }
                    }
                }

                roi_msg.x_offset = 0;//bounding_box_.x;
                roi_msg.y_offset = 0;//bounding_box_.y;
                roi_msg.width = 0;//bounding_box_.width;
                roi_msg.height = 0;//bounding_box_.height;
            }

            params_.roi_publisher->publish(roi_msg);

            RCLCPP_INFO(node_.get_logger(), "Detection frame size determined from mask: %d x %d", bounding_box_.width, bounding_box_.height);
        }
    }

    void mask_timer_callback()
    {
        mask_timer_->cancel();

        auto detection_mask_result = mask_set(params_.mask_filename, mask_last_modified_time_, grey_mask_);

        if (detection_mask_result == MaskCheckType::Enable)
        {
            if (!mask_enabled_)
            {
                mask_enabled_ = true;
                RCLCPP_INFO(node_.get_logger(), "Detection Mask Enabled.");
            }
            else
            {
                RCLCPP_INFO(node_.get_logger(), "Detection Mask Changed.");
            }
            roi_calculation();
        }
        else if ((detection_mask_result == MaskCheckType::Disable) && mask_enabled_)
        {
            RCLCPP_INFO(node_.get_logger(), "Detection Mask Disabled.");
            mask_enabled_ = false;
        }

        mask_timer_->reset();
    }

    inline MaskCheckType mask_set(const std::string & mask_filename, std::optional<std::filesystem::file_time_type> & mask_last_modified_time, cv::Mat & mask)
    {
        try
        {
            if (!std::filesystem::exists(mask_filename))
            {
                mask_last_modified_time.reset();
                return MaskCheckType::Disable;
            }

            auto current_modified_time = std::filesystem::last_write_time(mask_filename);
            if (mask_last_modified_time == current_modified_time) 
            {
                return MaskCheckType::NoChange;
            }
            mask_last_modified_time = current_modified_time;

            mask = cv::imread(mask_filename, cv::IMREAD_UNCHANGED);
            if (mask.empty())
            {
                mask_last_modified_time.reset();
                return MaskCheckType::Disable;
            }

            return MaskCheckType::Enable;
        }
        catch (std::exception &cve)
        {
            RCLCPP_ERROR(node_.get_logger(), "Exception on mask_set: %s", cve.what());
            mask_last_modified_time.reset();
            return MaskCheckType::Disable;
        }
    }

    inline void publish_resized_frame(const RosCvImageMsg & image_msg)
    {
        if (!params_.image_resized_publisher || (node_.count_subscribers(params_.image_resized_publisher->get_topic_name()) <= 0))
        {
            return;
        }
        cv::Mat resized_img;
        if (params_.resize_height > 0)
        {
            const double aspect_ratio = (double)image_msg.image_ptr->size().width / (double)image_msg.image_ptr->size().height;
            const int frame_height = params_.resize_height;
            const auto frame_width = (int)(aspect_ratio * (double)frame_height);
            cv::resize(*image_msg.image_ptr, resized_img, cv::Size(frame_width, frame_height));
        }
        else
        {
            resized_img = *image_msg.image_ptr;
        }

        auto resized_frame_msg = cv_bridge::CvImage(image_msg.msg_ptr->header, image_msg.msg_ptr->encoding, resized_img).toImageMsg();
        params_.image_resized_publisher->publish(*resized_frame_msg);            
    }
};


#endif