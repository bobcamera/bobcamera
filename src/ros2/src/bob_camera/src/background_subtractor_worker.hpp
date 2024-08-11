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
#include <boblib/api/base/Image.hpp>

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
    [[nodiscard]] bool get_use_cuda() const { return use_cuda_; }
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
    void set_use_cuda(bool enable) { use_cuda_ = enable; }
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
    bool use_cuda_{true};
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
        using_cuda_ = params_.get_use_cuda() ? cv::cuda::getCudaEnabledDeviceCount() : false;
        detection_mask_ptr_ = std::make_unique<boblib::base::Image>(using_cuda_);
        bgs_img_ptr_ = std::make_unique<boblib::base::Image>(using_cuda_);

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

    void image_callback(const std_msgs::msg::Header& header, const boblib::base::Image & img)
    {
        std::unique_lock lock(mutex_);
        cv_.wait(lock, [this] { return ready_; });

        processing_ = true;

        boblib::base::Image blank_mask(using_cuda_);
        boblib::base::Image gray_img(using_cuda_);
        sensor_msgs::msg::Image bgs_msg;

        try
        {
            bgs_msg.header = header;

            img.convertTo(gray_img, cv::COLOR_BGR2GRAY);

            if (mask_enabled_ && (detection_mask_ptr_->size() != gray_img.size()))
            {
                detection_mask_ptr_->resize(gray_img.size());
            }
            
            bgs_ptr_->apply(gray_img, *bgs_img_ptr_, mask_enabled_ ? *detection_mask_ptr_ : blank_mask);

            fill_header(bgs_msg, *bgs_img_ptr_);

            publish_frame(bgs_msg, *bgs_img_ptr_);

            publish_resized_frame(bgs_msg, *bgs_img_ptr_);

            do_detection(bgs_msg, *bgs_img_ptr_);
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
        cv_.notify_all();
    }

private:
    inline void fill_header(sensor_msgs::msg::Image & bgs_msg, boblib::base::Image & bgs_img)
    {
        bgs_msg.height = bgs_img.size().height;
        bgs_msg.width = bgs_img.size().width;
        bgs_msg.encoding = ImageUtils::type_to_encoding(bgs_img.type());
        bgs_msg.is_bigendian = (rcpputils::endian::native == rcpputils::endian::big);
        bgs_msg.step = bgs_img.size().width * bgs_img.elemSize();
    }

    inline void publish_frame(sensor_msgs::msg::Image & bgs_msg, boblib::base::Image & bgs_img)
    {
        if (node_.count_subscribers(params_.get_image_publisher()->get_topic_name()) <= 0)
        {
            return;
        }
        const size_t totalBytes = bgs_img.total() * bgs_img.elemSize();
        bgs_msg.data.assign(bgs_img.data(), bgs_img.data() + totalBytes);;        

        params_.get_image_publisher()->publish(bgs_msg);
    }

    inline void publish_resized_frame(sensor_msgs::msg::Image & bgs_msg, const boblib::base::Image & bgs_img) const
    {
        if (!params_.get_image_resized_publisher()
            || (params_.get_resize_height() <= 0)
            || (node_.count_subscribers(params_.get_image_resized_publisher()->get_topic_name()) <= 0))
        {
            return;
        }
        boblib::base::Image resized_img(using_cuda_);
        if (params_.get_resize_height() > 0)
        {
            const auto frame_width = static_cast<int>(bgs_img.size().aspectRatio() * static_cast<double>(params_.get_resize_height()));
            bgs_img.resizeTo(resized_img, cv::Size(frame_width, params_.get_resize_height()));
        }

        auto resized_frame_msg = cv_bridge::CvImage(bgs_msg.header, bgs_msg.encoding, resized_img.download()).toImageMsg();
        params_.get_image_resized_publisher()->publish(*resized_frame_msg);            
    }

    inline void do_detection(sensor_msgs::msg::Image & bgs_msg, boblib::base::Image & bgs_img)
    {
        bob_interfaces::msg::DetectorState state;
        state.sensitivity = params_.get_sensitivity();
        bob_interfaces::msg::DetectorBBoxArray bbox2D_array;
        bbox2D_array.header = bgs_msg.header;
        bbox2D_array.image_width = bgs_img_ptr_->size().width;
        bbox2D_array.image_height = bgs_img_ptr_->size().height;

        if (median_filter_)
        {
            bgs_img.medianBlur(3);
        }

        std::vector<cv::Rect> bboxes;
        auto det_result = blob_detector_ptr_->detect(bgs_img, bboxes);
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

    inline void add_bboxes(bob_interfaces::msg::DetectorBBoxArray& bbox2D_array, const std::vector<cv::Rect>& bboxes)
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
            detection_mask_ptr_->create(mask);
        }
        else if ((detection_mask_result == MaskWorker::MaskCheckType::Disable) && mask_enabled_)
        {
            node_.log_send_info("Detection Mask Disabled.");
            mask_enabled_ = false;
            detection_mask_ptr_.release();
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

    std::unique_ptr<boblib::base::Image> bgs_img_ptr_;
    std::unique_ptr<boblib::base::Image> detection_mask_ptr_;

    std::condition_variable cv_;
    std::mutex mutex_;
    bool ready_{false};
    bool processing_{false};

    bool using_cuda_{false};
};
