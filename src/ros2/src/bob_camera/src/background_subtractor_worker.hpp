#pragma once

#include <filesystem>
#include <mutex>
#include <condition_variable>

#include <boblib/api/bgs/bgs.hpp>
#include <boblib/api/bgs/WeightedMovingVariance/WeightedMovingVarianceUtils.hpp>
#include <boblib/api/blobs/connectedBlobDetection.hpp>
#include <boblib/api/base/Image.hpp>
#include <boblib/api/utils/pubsub/TopicManager.hpp>
#include <boblib/api/utils/profiler.hpp>
#include <boblib/api/utils/fps_tracker.hpp>

#include <bob_interfaces/msg/tracking.hpp>
#include <bob_interfaces/msg/detector_state.hpp>
#include <bob_interfaces/msg/detector_b_box_array.hpp>

#include "parameter_node.hpp"

#include "mask_worker.hpp"
#include "publish_image.hpp"
#include "detection.hpp"
#include "image_recorder.hpp"
#include "json_recorder.hpp"

class BackgroundSubtractorWorker
{
public:
    BackgroundSubtractorWorker(ParameterNode &node
                            , CameraBgsParams &params
                            , const rclcpp::QoS &qos_publish_profile
                            , boblib::utils::pubsub::TopicManager &topic_manager
                            , boblib::utils::Profiler &profiler)
        : node_(node)
        , params_(params)
        , qos_publish_profile_(qos_publish_profile)
        , topic_manager_(topic_manager)
        , profiler_(profiler)
    {
        mask_worker_ptr_ = std::make_unique<MaskWorker>(node_,
                                                        [this](MaskWorker::MaskCheckType detection_mask_result, const cv::Mat &mask)
                                                        { mask_timer_callback(detection_mask_result, mask); });
    }

    ~BackgroundSubtractorWorker()
    {
        node_.log_info("BackgroundSubtractorWorker destructor");
    }

    void init()
    {
        prof_bgs_worker_id_ = profiler_.add_region("BGS Worker");
        prof_convert_color_id_ = profiler_.add_region("Convert Color", prof_bgs_worker_id_);
        prof_bgs_id_ = profiler_.add_region("Background Subtractor", prof_bgs_worker_id_);
        prof_blobs_id_ = profiler_.add_region("Blobs", prof_bgs_worker_id_);

        using_cuda_ = params_.use_cuda ? boblib::base::Utils::has_cuda() : false;

        blank_mask_ptr_ = std::make_unique<boblib::base::Image>(using_cuda_);
        detection_mask_ptr_ = std::make_unique<boblib::base::Image>(using_cuda_);

        mask_worker_ptr_->init(params_.bgs.mask.timer_seconds, params_.bgs.mask.filename);

        image_publisher_ptr_ = node_.create_publisher<sensor_msgs::msg::Image>(params_.topics.bgs_image_publish_topic, qos_publish_profile_);
        image_resized_publisher_ptr_ = node_.create_publisher<sensor_msgs::msg::Image>(params_.topics.bgs_image_publish_topic + "/resized", qos_publish_profile_);
        detection_publisher_ptr_ = node_.create_publisher<bob_interfaces::msg::DetectorBBoxArray>(params_.topics.detection_publish_topic, qos_publish_profile_);
        state_publisher_ptr_ = node_.create_publisher<bob_interfaces::msg::DetectorState>(params_.topics.detection_state_publish_topic, qos_publish_profile_);

        camera_pubsub_ptr_ = topic_manager_.get_topic<PublishImage>(params_.topics.image_publish_topic + "_publish");
        camera_pubsub_ptr_->subscribe<BackgroundSubtractorWorker, &BackgroundSubtractorWorker::camera_image_callback>(this);

        process_pubsub_ptr_ = topic_manager_.get_topic<PublishImage>(params_.topics.image_publish_topic + "_process");
        process_pubsub_ptr_->subscribe<BackgroundSubtractorWorker, &BackgroundSubtractorWorker::publish_bgs_image>(this);

        detector_pubsub_ptr_ = topic_manager_.get_topic<Detection>(params_.topics.detection_publish_topic);
    }

    void restart_mask()
    {
        if (mask_worker_ptr_ && mask_worker_ptr_->is_running())
        {
            mask_worker_ptr_->init(params_.bgs.mask.timer_seconds, params_.bgs.mask.filename);
        }
    }

    void init_bgs(const std::string &bgs) noexcept
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            ready_ = false;
        }

        try
        {
            if (bgs == "vibe")
            {
                params_.bgs.type = CameraBgsParams::BGSType::Vibe;
            }
            else
            {
                params_.bgs.type = CameraBgsParams::BGSType::WMV;
            }
            if (!params_.bgs.sensitivity.empty())
            {
                bgs_ptr_ = create_bgs(params_.bgs.type);
            }
        }
        catch (const std::exception &e)
        {
            node_.log_send_error("init_bgs: Exception: %s", e.what());
            rcutils_reset_error();
        }

        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (bgs_ptr_)
            {
                ready_ = true;
            }
            else
            {
                node_.log_send_error("init_bgs: Failed to initialize background subtractor; bgs_ptr_ is null");
            }
            cv_ready_.notify_all();
        }
    }

    void restart() noexcept
    {
        if (!bgs_ptr_)
        {
            node_.log_send_error("BGSWorker: restart: bgs_ptr_ is not initialized");
            return;
        }
        bgs_ptr_->restart();
    }

    void init_sensitivity(const std::string &sensitivity) noexcept
    {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_processing_.wait(lock, [this]
                              { return !processing_; });

        try
        {
            if (sensitivity.empty())
            {
                node_.log_debug("Ignoring sensitivity change request, EMPTY VALUE");
                cv_processing_.notify_all();
                return;
            }
            if (params_.bgs.sensitivity == sensitivity)
            {
                node_.log_info("Ignoring sensitivity change request, NO CHANGE");
                cv_processing_.notify_all();
                return;
            }

            if (!params_.bgs.sensitivity_collection.get_configs().contains(sensitivity))
            {
                node_.log_error("Unknown config specified: %s", sensitivity.c_str());
                cv_processing_.notify_all();
                return;
            }

            ready_ = false;
            params_.bgs.sensitivity = sensitivity;

            const auto &config = params_.bgs.sensitivity_collection.get_configs().at(sensitivity);

            wmv_params_ = std::make_unique<boblib::bgs::WMVParams>(
                config.sensitivity.wmv_enable_weight, config.sensitivity.wmv_enable_threshold, config.sensitivity.wmv_threshold,
                config.sensitivity.wmv_weight1, config.sensitivity.wmv_weight2, config.sensitivity.wmv_weight3);
            vibe_params_ = std::make_unique<boblib::bgs::VibeParams>(
                config.sensitivity.vibe_threshold, config.sensitivity.vibe_bg_samples, config.sensitivity.vibe_required_bg_samples,
                config.sensitivity.vibe_learning_rate);
            bgs_ptr_ = create_bgs(params_.bgs.type);
            blob_params_ = std::make_unique<boblib::blobs::ConnectedBlobDetectionParams>(false,
                                                                                         config.sensitivity.blob_size_threshold, config.sensitivity.blob_area_threshold, config.sensitivity.blob_min_distance,
                                                                                         config.sensitivity.blob_max_blobs);
            blob_detector_ptr_ = std::make_unique<boblib::blobs::ConnectedBlobDetection>(*blob_params_);

            median_filter_ = config.sensitivity.median_filter;
        }
        catch (const std::exception &e)
        {
            node_.log_send_error("init_sensitivity: Exception: %s", e.what());
            rcutils_reset_error();
        }

        ready_ = true;
        cv_ready_.notify_all();
        cv_processing_.notify_all();
    }

    void camera_image_callback(const std::shared_ptr<PublishImage> &publish_image) noexcept
    {
        try
        {
            std::unique_lock lock(mutex_);
            cv_ready_.wait(lock, [this]
                           { return ready_; });
            processing_ = true;

            if (!bgs_ptr_)
            {
                node_.log_send_error("BGSWorker: process_image: bgs_ptr_ is not initialized");
                processing_ = false;
                cv_processing_.notify_all();
                return;
            }

            profiler_.start(prof_convert_color_id_);
            boblib::base::Image gray_img(using_cuda_);
            publish_image->image_ptr->convertColorTo(gray_img, cv::COLOR_BGR2GRAY);
            profiler_.stop(prof_convert_color_id_);

            // Resize detection mask if needed
            if (mask_enabled_ && params_.bgs.mask.enable_override &&
                (detection_mask_ptr_->size() != gray_img.size()))
            {
                detection_mask_ptr_->resize(gray_img.size());
            }

            // Apply background subtraction
            const auto &mask = (mask_enabled_ && params_.bgs.mask.enable_override)
                                   ? *detection_mask_ptr_
                                   : *blank_mask_ptr_;

            profiler_.start(prof_bgs_id_);
            auto bgs_img_ptr = std::make_shared<boblib::base::Image>(using_cuda_);

            bgs_ptr_->apply(gray_img, *bgs_img_ptr, mask);
            profiler_.stop(prof_bgs_id_);
            profiler_.start(prof_blobs_id_);
            do_detection(publish_image, *bgs_img_ptr, publish_image->camera_info_ptr->fps);
            profiler_.stop(prof_blobs_id_);

            processing_ = false;
            cv_processing_.notify_all();

            process_pubsub_ptr_->publish(PublishImage(publish_image->header_ptr, std::move(bgs_img_ptr), publish_image->camera_info_ptr));
        }
        catch (const std::exception &e)
        {
            node_.log_send_error("bgs_worker: image_callback: exception: %s", e.what());
            rcutils_reset_error();
        }
        catch (...)
        {
            node_.log_send_error("bgs_worker: image_callback: Unknown Exception");
            rcutils_reset_error();
        }
    }

private:
    void publish_bgs_image(const std::shared_ptr<PublishImage> &publish_image) noexcept
    {
        try
        {
            const auto &header = publish_image->header_ptr;
            auto &bgs_img = *publish_image->image_ptr;

            ImageUtils::publish_image(image_publisher_ptr_, *header, bgs_img.toMat());
            publish_resized_frame(header, bgs_img);
        }
        catch (const std::exception &e)
        {
            node_.log_send_error("BGSWorker: process_images: Exception: %s", e.what());
            rcutils_reset_error();
        }
    }

    inline void publish_resized_frame(const std_msgs::msg::Header::SharedPtr &header, const boblib::base::Image &bgs_img) const
    {
        if (!image_resized_publisher_ptr_ 
            || (params_.resize_height <= 0) 
            || (image_resized_publisher_ptr_->get_subscription_count() <= 0))
        {
            return;
        }
        boblib::base::Image resized_img(using_cuda_);
        const auto frame_width = static_cast<int>(bgs_img.size().aspectRatio() * static_cast<double>(params_.resize_height));
        bgs_img.resizeTo(resized_img, cv::Size(frame_width, params_.resize_height));

        ImageUtils::publish_image(image_resized_publisher_ptr_, *header, resized_img.toMat());
    }

    inline void do_detection(const std::shared_ptr<PublishImage> &publish_image, boblib::base::Image &bgs_img, float fps)
    {
        // Apply median filter if enabled to reduce noise
        if (median_filter_)
        {
            bgs_img.medianBlur(3);
        }

        // Perform blob detection
        auto bboxes = std::make_shared<std::vector<cv::Rect>>();
        const auto det_result = blob_detector_ptr_->detect(bgs_img, *bboxes);

        if (state_publisher_ptr_->get_subscription_count() > 0)
        {
            bob_interfaces::msg::DetectorState state;
            state.sensitivity = params_.bgs.sensitivity;
            state.max_blobs_reached = det_result == boblib::blobs::DetectionResult::MaxBlobsReached;
            state_publisher_ptr_->publish(state);
        }

        if (detection_publisher_ptr_->get_subscription_count() > 0)
        {
            bob_interfaces::msg::DetectorBBoxArray bbox2D_array;
            bbox2D_array.header = *publish_image->header_ptr;
            bbox2D_array.image_width = bgs_img.size().width;
            bbox2D_array.image_height = bgs_img.size().height;
            if (det_result != boblib::blobs::DetectionResult::MaxBlobsReached)
            {
                add_bboxes(bbox2D_array, *bboxes);
            }

            detection_publisher_ptr_->publish(bbox2D_array);
        }

        detector_pubsub_ptr_->publish(Detection(publish_image->header_ptr, publish_image->image_ptr, bboxes, fps));
    }

    inline void add_bboxes(bob_interfaces::msg::DetectorBBoxArray &bbox2D_array, const std::vector<cv::Rect> &bboxes) noexcept
    {
        bbox2D_array.detections.reserve(bboxes.size());
        for (const auto &bbox : bboxes)
        {
            bob_interfaces::msg::DetectorBBox detection;
            detection.x = bbox.x;
            detection.y = bbox.y;
            detection.width = bbox.width;
            detection.height = bbox.height;
            bbox2D_array.detections.push_back(detection);
        }
    }

    inline std::unique_ptr<boblib::bgs::CoreBgs> create_bgs(CameraBgsParams::BGSType type)
    {
        switch (type)
        {
        case CameraBgsParams::BGSType::Vibe:
            return std::make_unique<boblib::bgs::Vibe>(*vibe_params_);
        case CameraBgsParams::BGSType::WMV:
            return std::make_unique<boblib::bgs::WeightedMovingVariance>(*wmv_params_);
        default:
            return nullptr;
        }
    }

    void mask_timer_callback(MaskWorker::MaskCheckType detection_mask_result, const cv::Mat &mask)
    {
        std::lock_guard<std::mutex> lock(mutex_);
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
            detection_mask_ptr_->create(mask);
        }
        else if ((detection_mask_result == MaskWorker::MaskCheckType::Disable) && mask_enabled_)
        {
            node_.log_send_info("Detection Mask Disabled.");
            mask_enabled_ = false;
            detection_mask_ptr_->release();
        }
    }

    ParameterNode &node_;
    CameraBgsParams &params_;
    const rclcpp::QoS &qos_publish_profile_;

    std::unique_ptr<MaskWorker> mask_worker_ptr_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_ptr_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_resized_publisher_ptr_;
    rclcpp::Publisher<bob_interfaces::msg::DetectorBBoxArray>::SharedPtr detection_publisher_ptr_;
    rclcpp::Publisher<bob_interfaces::msg::DetectorState>::SharedPtr state_publisher_ptr_;

    bool mask_enabled_{false};
    bool median_filter_{false};
    std::unique_ptr<boblib::bgs::CoreBgs> bgs_ptr_{nullptr};
    std::unique_ptr<boblib::blobs::ConnectedBlobDetection> blob_detector_ptr_{nullptr};
    std::unique_ptr<boblib::bgs::VibeParams> vibe_params_;
    std::unique_ptr<boblib::bgs::WMVParams> wmv_params_;
    std::unique_ptr<boblib::blobs::ConnectedBlobDetectionParams> blob_params_;

    std::unique_ptr<boblib::base::Image> detection_mask_ptr_;

    std::condition_variable cv_ready_;
    std::condition_variable cv_processing_;
    std::mutex mutex_;
    bool ready_{false};
    bool processing_{false};
    bool using_cuda_{false};

    std::unique_ptr<boblib::base::Image> blank_mask_ptr_;
    boblib::utils::pubsub::TopicManager &topic_manager_;
    std::shared_ptr<boblib::utils::pubsub::PubSub<PublishImage>> camera_pubsub_ptr_;
    std::shared_ptr<boblib::utils::pubsub::PubSub<PublishImage>> process_pubsub_ptr_;
    std::shared_ptr<boblib::utils::pubsub::PubSub<Detection>> detector_pubsub_ptr_;

    // profiling fields
    boblib::utils::Profiler &profiler_;

    size_t prof_bgs_worker_id_;
    size_t prof_convert_color_id_;
    size_t prof_bgs_id_;
    size_t prof_blobs_id_;
};
