#pragma once

#include <string>
#include <filesystem>
#include <mutex>
#include <condition_variable>
#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <bob_interfaces/msg/detector_state.hpp>
#include <bob_interfaces/msg/detector_b_box_array.hpp>
#include <boblib/api/bgs/bgs.hpp>
#include <boblib/api/bgs/WeightedMovingVariance/WeightedMovingVarianceUtils.hpp>
#include <boblib/api/blobs/connectedBlobDetection.hpp>
#include "parameter_lifecycle_node.hpp"
#include "image_utils.hpp"
#include "background_subtractor_companion.hpp"
#include "mask_worker.hpp"
//#include "sensitivity_config_collection.hpp"

class BackgroundSubtractorWorkerParams
{
public:
    enum class BGSType
    {
        Unknown,
        Vibe,
        WMV
    };

    // Constructor
    BackgroundSubtractorWorkerParams() = default;

    // Getters
    [[nodiscard]] const auto& get_image_publisher() const { return image_publisher_; }
    [[nodiscard]] const auto& get_image_resized_publisher() const { return image_resized_publisher_; }
    [[nodiscard]] const auto& get_detection_publisher() const { return detection_publisher_; }
    [[nodiscard]] const auto& get_state_publisher() const { return state_publisher_; }

    [[nodiscard]] const auto& get_image_publish_topic() const { return image_publish_topic_; }
    [[nodiscard]] const auto& get_image_resized_publish_topic() const { return image_resized_publish_topic_; }
    [[nodiscard]] const auto& get_detection_publish_topic() const { return detection_publish_topic_; }
    [[nodiscard]] const auto& get_detection_state_publish_topic() const { return detection_state_publish_topic_; }
    [[nodiscard]] BGSType get_bgs_type() const { return bgs_type_; }
    [[nodiscard]] const auto& get_sensitivity() const { return sensitivity_; }
    [[nodiscard]] const auto& get_sensitivity_collection() const { return sensitivity_collection_; }
    [[nodiscard]] bool get_mask_enable_override() const { return mask_enable_override_; }
    [[nodiscard]] const auto& get_mask_filename() const { return mask_filename_; }
    [[nodiscard]] int get_resize_height() const { return resize_height_; }
    [[nodiscard]] int get_mask_timer_seconds() const { return mask_timer_seconds_; }

    // Setters
    void set_image_publisher(const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr& publisher) { image_publisher_ = publisher; }
    void set_image_resized_publisher(const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr& publisher) { image_resized_publisher_ = publisher; }
    void set_detection_publisher(const rclcpp::Publisher<bob_interfaces::msg::DetectorBBoxArray>::SharedPtr& publisher) { detection_publisher_ = publisher; }
    void set_state_publisher(const rclcpp::Publisher<bob_interfaces::msg::DetectorState>::SharedPtr& publisher) { state_publisher_ = publisher; }

    void set_image_publish_topic(const std::string& topic) { image_publish_topic_ = topic; }
    void set_image_resized_publish_topic(const std::string& topic) { image_resized_publish_topic_ = topic; }
    void set_detection_publish_topic(const std::string& topic) { detection_publish_topic_ = topic; }
    void set_detection_state_publish_topic(const std::string& topic) { detection_state_publish_topic_ = topic; }
    void set_bgs_type(BGSType type) { bgs_type_ = type; }
    void set_sensitivity(const std::string& sensitivity) { sensitivity_ = sensitivity; }
    void set_sensitivity_collection(const SensitivityConfigCollection& collection) { sensitivity_collection_ = collection; }
    void set_sensitivity_collection(const std::string& json_collection) { sensitivity_collection_.set_configs(json_collection); }
    void set_mask_enable_override(bool enable) { mask_enable_override_ = enable; }
    void set_mask_filename(const std::string& filename) { mask_filename_ = filename; }
    void set_resize_height(int height) { resize_height_ = height; }
    void set_mask_timer_seconds(int seconds) { mask_timer_seconds_ = seconds; }

private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_resized_publisher_;
    rclcpp::Publisher<bob_interfaces::msg::DetectorBBoxArray>::SharedPtr detection_publisher_;
    rclcpp::Publisher<bob_interfaces::msg::DetectorState>::SharedPtr state_publisher_;

    std::string image_publish_topic_;
    std::string image_resized_publish_topic_;
    std::string detection_publish_topic_;
    std::string detection_state_publish_topic_;
    BGSType bgs_type_{BGSType::Unknown};
    std::string sensitivity_;
    SensitivityConfigCollection sensitivity_collection_;
    bool mask_enable_override_{true};
    std::string mask_filename_;
    int resize_height_{};
    int mask_timer_seconds_{};
};

class BackgroundSubtractorWorker
{
public:
    BackgroundSubtractorWorker(ParameterLifeCycleNode& node, BackgroundSubtractorWorkerParams& params)
        : node_(node)
        , params_(params)
        , mask_worker_ptr_(std::make_unique<MaskWorker>(node_, 
            [this](MaskWorker::MaskCheckType detection_mask_result, const cv::Mat& mask) { mask_timer_callback(detection_mask_result, mask); }))
    {}

    void init()
    {
        mask_worker_ptr_->init(params_.get_mask_timer_seconds(), params_.get_mask_filename());
    }

    void restart_mask()
    {
        if (mask_worker_ptr_)
        {
            mask_worker_ptr_->init(params_.get_mask_timer_seconds(), params_.get_mask_filename());
        }
    }

    void init_bgs(const std::string& bgs)
    {
        ready_ = false;

        try
        {
            if (bgs == "vibe")
            {
                params_.set_bgs_type(BackgroundSubtractorWorkerParams::BGSType::Vibe);
            }
            else
            {
                params_.set_bgs_type(BackgroundSubtractorWorkerParams::BGSType::WMV);
            }
            if (!params_.get_sensitivity().empty())
            {
                bgs_ptr_ = create_bgs(params_.get_bgs_type());
            }
        }
        catch (const std::exception& e)
        {
            node_.log_send_error("init_bgs: Exception: %s", e.what());
        }
        catch (...)
        {
            node_.log_send_error("init_bgs: Unknown exception");
        }

        ready_ = true;
        cv_.notify_all();
    }

    void restart()
    {
        bgs_ptr_->restart();
    }

    void init_sensitivity(const std::string& sensitivity)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [this] { return !processing_; });

        try
        {
            if (sensitivity.empty())
            {
                node_.log_debug("Ignoring sensitivity change request, EMPTY VALUE");
                return;
            }
            if (params_.get_sensitivity() == sensitivity)
            {
                node_.log_info("Ignoring sensitivity change request, NO CHANGE");
                return;
            }

            if (!params_.get_sensitivity_collection().get_configs().contains(sensitivity))
            {
                node_.log_error("Unknown config specified: %s", sensitivity.c_str());
                return;
            }

            ready_ = false;
            params_.set_sensitivity(sensitivity);

            const auto& config = params_.get_sensitivity_collection().get_configs().at(sensitivity);

            wmv_params_ = std::make_unique<boblib::bgs::WMVParams>(
                config.sensitivity.wmv_enable_weight, config.sensitivity.wmv_enable_threshold, config.sensitivity.wmv_threshold, 
                config.sensitivity.wmv_weight1, config.sensitivity.wmv_weight2, config.sensitivity.wmv_weight3);
            vibe_params_ = std::make_unique<boblib::bgs::VibeParams>(
                config.sensitivity.vibe_threshold, config.sensitivity.vibe_bg_samples, config.sensitivity.vibe_required_bg_samples, 
                config.sensitivity.vibe_learning_rate);

            bgs_ptr_ = create_bgs(params_.get_bgs_type());

            blob_params_ = std::make_unique<boblib::blobs::ConnectedBlobDetectionParams>(
                config.sensitivity.blob_size_threshold, config.sensitivity.blob_area_threshold, config.sensitivity.blob_min_distance, 
                config.sensitivity.blob_max_blobs);
            blob_detector_ptr_ = std::make_unique<boblib::blobs::ConnectedBlobDetection>(*blob_params_);

            median_filter_ = config.sensitivity.median_filter;
        }
        catch (const std::exception& e)
        {
            node_.log_send_error("init_sensitivity: Exception: %s", e.what());
        }
        catch (...)
        {
            node_.log_send_error("init_sensitivity: Unknown exception");
        }
        ready_ = true;
        cv_.notify_all();
    }

    void image_callback(const std_msgs::msg::Header& header, const cv::Mat& img)
    {
        std::unique_lock lock(mutex_);
        cv_.wait(lock, [this] { return ready_; });

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

            bgs_ptr_->apply(gray_img, ros_cv_foreground_mask_->get_image(), mask_enabled_ ? detection_mask_ : cv::Mat());

            node_.publish_if_subscriber(params_.get_image_publisher(), ros_cv_foreground_mask_->get_msg());

            publish_resized_frame(*ros_cv_foreground_mask_);

            if (median_filter_)
            {
                cv::medianBlur(ros_cv_foreground_mask_->get_image(), ros_cv_foreground_mask_->get_image(), 3);
            }

            bob_interfaces::msg::DetectorState state;
            state.sensitivity = params_.get_sensitivity();
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

            node_.publish_if_subscriber(params_.get_state_publisher(), state);
            node_.publish_if_subscriber(params_.get_detection_publisher(), bbox2D_array);
        }
        catch (const std::exception& e)
        {
            node_.log_send_error("bgs_worker: image_callback: exception: %s", e.what());
        }
        catch (...)
        {
            node_.log_send_error("bgs_worker: image_callback: Unknown Exception");
        }

        processing_ = false;
        cv_.notify_all();
    }

private:
    void add_bboxes(bob_interfaces::msg::DetectorBBoxArray& bbox2D_array, const std::vector<cv::Rect>& bboxes)
    {
        for (const auto& bbox : bboxes)
        {
            bob_interfaces::msg::DetectorBBox bbox_msg;
            bbox_msg.x = bbox.x;
            bbox_msg.y = bbox.y;
            bbox_msg.width = bbox.width;
            bbox_msg.height = bbox.height;
            bbox2D_array.detections.push_back(bbox_msg);
        }
    }

    std::unique_ptr<boblib::bgs::CoreBgs> create_bgs(BackgroundSubtractorWorkerParams::BGSType type)
    {
        switch (type)
        {
        case BackgroundSubtractorWorkerParams::BGSType::Vibe:
            return std::make_unique<boblib::bgs::Vibe>(*vibe_params_);
        case BackgroundSubtractorWorkerParams::BGSType::WMV:
            return std::make_unique<boblib::bgs::WeightedMovingVariance>(*wmv_params_);
        default:
            return nullptr;
        }
    }

    void publish_resized_frame(const RosCvImageMsg& image_msg)
    {
        if (!params_.get_image_resized_publisher()
            || (params_.get_resize_height() <= 0)
            || (node_.count_subscribers(params_.get_image_resized_publisher()->get_topic_name()) <= 0))
        {
            return;
        }
        cv::Mat resized_img;
        if (params_.get_resize_height() > 0)
        {
            const double aspect_ratio = static_cast<double>(image_msg.get_image().size().width) / image_msg.get_image().size().height;
            const int frame_height = params_.get_resize_height();
            const auto frame_width = static_cast<int>(aspect_ratio * frame_height);
            cv::resize(image_msg.get_image(), resized_img, cv::Size(frame_width, frame_height));
        }
        else
        {
            resized_img = image_msg.get_image();
        }

        auto resized_frame_msg = cv_bridge::CvImage(image_msg.get_header(), image_msg.get_msg().encoding, resized_img).toImageMsg();
        params_.get_image_resized_publisher()->publish(*resized_frame_msg);
    }

    void mask_timer_callback(MaskWorker::MaskCheckType detection_mask_result, const cv::Mat& mask)
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

    ParameterLifeCycleNode& node_;
    BackgroundSubtractorWorkerParams& params_;

    std::unique_ptr<MaskWorker> mask_worker_ptr_;

    bool mask_enabled_{false};
    bool median_filter_{false};
    std::unique_ptr<boblib::bgs::CoreBgs> bgs_ptr_{nullptr};
    std::unique_ptr<boblib::blobs::ConnectedBlobDetection> blob_detector_ptr_{nullptr};
    std::unique_ptr<boblib::bgs::VibeParams> vibe_params_;
    std::unique_ptr<boblib::bgs::WMVParams> wmv_params_;
    std::unique_ptr<boblib::blobs::ConnectedBlobDetectionParams> blob_params_;

    std::unique_ptr<RosCvImageMsg> ros_cv_foreground_mask_;

    cv::Mat detection_mask_;

    std::condition_variable cv_;
    std::mutex mutex_;
    bool ready_{false};
    bool processing_{false};
};
