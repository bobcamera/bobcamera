#pragma once
#ifndef __BACKGROUND_SUBTRACTOR_WORKER_H__
#define __BACKGROUND_SUBTRACTOR_WORKER_H__

#include <filesystem>

#include <cv_bridge/cv_bridge.hpp>

#include "parameter_lifecycle_node.hpp"
#include "image_utils.hpp"
#include "background_subtractor_companion.hpp"
#include "mask_worker.hpp"

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
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_resized_publisher;
    rclcpp::Publisher<bob_interfaces::msg::DetectorBBoxArray>::SharedPtr detection_publisher;
    rclcpp::Publisher<bob_interfaces::msg::DetectorState>::SharedPtr state_publisher;

    std::string image_publish_topic;
    std::string image_resized_publish_topic;
    std::string detection_publish_topic;
    std::string detection_state_publish_topic;
    BGSType bgs_type;
    std::string sensitivity;
    SensitivityConfigCollection sensitivity_collection;
    bool mask_enable_override = true;
    std::string mask_filename;
    int resize_height;
    int mask_timer_seconds;
};

class BackgroundSubtractorWorker
{
public:
    BackgroundSubtractorWorker(ParameterLifeCycleNode & node, BackgroundSubtractorWorkerParams & params)
        : node_(node)
        , params_(params)
    {
        mask_worker_ptr_ = std::make_unique<MaskWorker>(node_, [this](MaskWorker::MaskCheckType detection_mask_result, const cv::Mat & mask){mask_timer_callback(detection_mask_result, mask);});
    }

    void init()
    {
        mask_worker_ptr_->init(params_.mask_timer_seconds, params_.mask_filename);
    }

    void restart_mask()
    {
        if (mask_worker_ptr_)
        {
            mask_worker_ptr_->init(params_.mask_timer_seconds, params_.mask_filename);
        }
    }

    void init_bgs(const std::string & bgs)
    {
        ready_ = false;

        try
        {
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
        }
        catch (const std::exception & e)
        {
            node_.log_send_error("init_bgs: Exception: %s", e.what());
        }
        catch (...)
        {
            node_.log_send_error("init_bgs: Unknown exception");
        }

        ready_ = true;
        cv.notify_all();
    }

    void restart()
    {
        bgsPtr->restart();
    }

    void init_sensitivity(const std::string & sensitivity)
    {
        std::unique_lock<std::mutex> lock(mutex);
        cv.wait(lock, [this] { return !processing_; });
        
        try
        {
            if (sensitivity.empty() || (sensitivity.length() == 0))
            {
                node_.log_debug("Ignoring sensitivity change request, EMPTY VALUE");
                return;
            }
            if (params_.sensitivity == sensitivity)
            {
                node_.log_info("Ignoring sensitivity change request, NO CHANGE");
                return;
            }

            if (!params_.sensitivity_collection.configs.contains(sensitivity))
            {
                node_.log_error("Unknown config specified: %s", sensitivity.c_str());
                return;
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
        }
        catch (const std::exception & e)
        {
            node_.log_send_error("init_sensitivity: Exception: %s", e.what());
        }
        catch (...)
        {
            node_.log_send_error("init_sensitivity: Unknown exception");
        }
        ready_ = true;
        cv.notify_all();
    }

    void image_callback(const std_msgs::msg::Header & header, const cv::Mat & img)
    {
        std::unique_lock lock(mutex);
        cv.wait(lock, [this] { return ready_; });

        processing_ = true;

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

            if (!ros_cv_foreground_mask_ || (gray_img.size() != ros_cv_foreground_mask_->get_image().size()))
            {
                ros_cv_foreground_mask_ = std::make_unique<RosCvImageMsg>(gray_img, sensor_msgs::image_encodings::MONO8, true);
            }
            ros_cv_foreground_mask_->set_header(header);

            if (mask_enabled_ && (detection_mask_.size() != gray_img.size()))
            {
                cv::resize(detection_mask_, detection_mask_, gray_img.size());
            }

            bgsPtr->apply(gray_img, ros_cv_foreground_mask_->get_image(), mask_enabled_ ? detection_mask_ : cv::Mat());

            node_.publish_if_subscriber(params_.image_publisher, ros_cv_foreground_mask_->get_msg());

            publish_resized_frame(*ros_cv_foreground_mask_);

            if (median_filter_)
            {
                cv::medianBlur(ros_cv_foreground_mask_->get_image(), ros_cv_foreground_mask_->get_image(), 3);
            }

            bob_interfaces::msg::DetectorState state;
            state.sensitivity = params_.sensitivity;
            bob_interfaces::msg::DetectorBBoxArray bbox2D_array;
            bbox2D_array.header = header;
            bbox2D_array.image_width = gray_img.size().width;
            bbox2D_array.image_height = gray_img.size().height;

            std::vector<cv::Rect> bboxes;
            auto det_result = blob_detector_ptr_->detect(ros_cv_foreground_mask_->get_image(), bboxes);
            if (det_result == boblib::blobs::DetectionResult::Success)
            {
                state.max_blobs_reached = false;
                add_bboxes(bbox2D_array, bboxes);
            }
            else if (det_result == boblib::blobs::DetectionResult::MaxBlobsReached)
            {
                state.max_blobs_reached = true;
            }

            node_.publish_if_subscriber(params_.state_publisher, state);
            node_.publish_if_subscriber(params_.detection_publisher, bbox2D_array);            
        }
        catch (const std::exception & e)
        {
            node_.log_send_error("bgs_worker: image_callback: exception: %s", e.what());
        }
        catch (...)
        {
            node_.log_send_error("bgs_worker: image_callback: Unknown Exception");
        }

        processing_ = false;
        cv.notify_all();
    }

private:
    inline void add_bboxes(bob_interfaces::msg::DetectorBBoxArray & bbox2D_array, const std::vector<cv::Rect> & bboxes)
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

    inline void publish_resized_frame(const RosCvImageMsg & image_msg)
    {
        if (!params_.image_resized_publisher 
            || (params_.resize_height <= 0)
            || (node_.count_subscribers(params_.image_resized_publisher->get_topic_name()) <= 0))
        {
            return;
        }
        cv::Mat resized_img;
        if (params_.resize_height > 0)
        {
            const double aspect_ratio = (double)image_msg.get_image().size().width / (double)image_msg.get_image().size().height;
            const int frame_height = params_.resize_height;
            const auto frame_width = (int)(aspect_ratio * (double)frame_height);
            cv::resize(image_msg.get_image(), resized_img, cv::Size(frame_width, frame_height));
        }
        else
        {
            resized_img = image_msg.get_image();
        }

        auto resized_frame_msg = cv_bridge::CvImage(image_msg.get_header(), image_msg.get_msg().encoding, resized_img).toImageMsg();
        params_.image_resized_publisher->publish(*resized_frame_msg);            
    }

    void mask_timer_callback(MaskWorker::MaskCheckType detection_mask_result, const cv::Mat & mask)
    {
        if (detection_mask_result == MaskWorker::MaskCheckType::Enable)
        {
            if (!mask_enabled_)
            {
                mask_enabled_ = true;
                node_.log_send_info("Detection Mask Enabled.");
            }
            else
            {
                node_.log_send_info("Detection Mask Changed.");
            }
            detection_mask_ = mask.clone();
        }
        else if ((detection_mask_result == MaskWorker::MaskCheckType::Disable) && mask_enabled_)
        {
            node_.log_send_info("Detection Mask Disabled.");
            mask_enabled_ = false;
            detection_mask_.release();
        }
    }

    ParameterLifeCycleNode & node_;

    std::unique_ptr<MaskWorker> mask_worker_ptr_;

    BackgroundSubtractorWorkerParams & params_;

    bool mask_enabled_;
    bool median_filter_;
    std::unique_ptr<boblib::bgs::CoreBgs> bgsPtr{nullptr};
    std::unique_ptr<boblib::blobs::ConnectedBlobDetection> blob_detector_ptr_{nullptr};
    std::unique_ptr<boblib::bgs::VibeParams> vibe_params_;
    std::unique_ptr<boblib::bgs::WMVParams> wmv_params_;
    std::unique_ptr<boblib::blobs::ConnectedBlobDetectionParams> blob_params_;

    std::unique_ptr<RosCvImageMsg> ros_cv_foreground_mask_;

    cv::Mat detection_mask_;

    std::condition_variable cv;
    std::mutex mutex;
    bool ready_ = false;
    bool processing_ = false;
};

#endif