#pragma once

#include <filesystem>
#include <mutex>
#include <condition_variable>

#include <boblib/api/bgs/bgs.hpp>
#include <boblib/api/bgs/WeightedMovingVariance/WeightedMovingVarianceUtils.hpp>
#include <boblib/api/blobs/connectedBlobDetection.hpp>
#include <boblib/api/base/Image.hpp>

#include <bob_interfaces/msg/tracking.hpp>

#include "parameter_node.hpp"
#include "background_subtractor_worker_params.hpp"

#include "mask_worker.hpp"
#include "publish_image.hpp"
#include "image_recorder.hpp"
#include "json_recorder.hpp"

class BackgroundSubtractorWorker
{
public:
    BackgroundSubtractorWorker(ParameterNode& node, BackgroundSubtractorWorkerParams& params)
        : node_(node)
        , params_(params)
        , mask_worker_ptr_(std::make_unique<MaskWorker>(node_, 
            [this](MaskWorker::MaskCheckType detection_mask_result, const cv::Mat& mask) { mask_timer_callback(detection_mask_result, mask); }))
    {}

    ~BackgroundSubtractorWorker()
    {
        if (process_thread_.joinable())
        {
            process_thread_.join();
        }
    }

    void init()
    {
        using_cuda_ = params_.get_use_cuda() ? boblib::base::Utils::has_cuda() : false;
        detection_mask_ptr_ = std::make_unique<boblib::base::Image>(using_cuda_);

        process_queue_ptr_ = std::make_unique<SynchronizedQueue<PublishImage>>(params_.get_max_queue_process_size());

        mask_worker_ptr_->init(params_.get_mask_timer_seconds(), params_.get_mask_filename());

        process_thread_ = std::jthread(&BackgroundSubtractorWorker::process_images, this);

        rclcpp::QoS sub_qos_profile(4);
        sub_qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        sub_qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);
        sub_qos_profile.history(rclcpp::HistoryPolicy::KeepLast);

        tracking_subscription_ = node_.create_subscription<bob_interfaces::msg::Tracking>(params_.get_tracking_subscriber_topic(), sub_qos_profile,
                                                                                    [this](const bob_interfaces::msg::Tracking::SharedPtr tracking_msg)
                                                                                    { tracking_callback(tracking_msg); });

        camera_info_subscription_ = node_.create_subscription<bob_camera::msg::CameraInfo>(params_.get_camera_info_subscriber_topic(), sub_qos_profile,
                                                                                           [this](const bob_camera::msg::CameraInfo::SharedPtr camera_info_msg)
                                                                                           { camera_info_callback(camera_info_msg); });
    }

    void restart_mask()
    {
        if (mask_worker_ptr_ && mask_worker_ptr_->is_running())
        {
            mask_worker_ptr_->init(params_.get_mask_timer_seconds(), params_.get_mask_filename());
        }
    }

    void init_bgs(const std::string& bgs) noexcept
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

    void restart() noexcept
    {
        bgs_ptr_->restart();
    }

    void init_sensitivity(const std::string &sensitivity) noexcept
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

            blob_params_ = std::make_unique<boblib::blobs::ConnectedBlobDetectionParams>(false,
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

    void image_callback(float fps, const std_msgs::msg::Header &header, const boblib::base::Image &img) noexcept
    {
        try
        {
            if (fps != fps_)
            {
                fps_ = fps;
                auto total_pre_frames = (size_t)(static_cast<int>(std::ceil(fps_)) * params_.get_recording_seconds_save());
                img_recorder_ = std::make_unique<ImageRecorder>(total_pre_frames);
                json_recorder_ = std::make_unique<JsonRecorder>(total_pre_frames);
            }

            boblib::base::Image gray_img(using_cuda_);
            img.convertTo(gray_img, cv::COLOR_BGR2GRAY);

            // sensor_msgs::msg::Image bgs_msg;
            // fill_header(bgs_msg, header, gray_img);

            create_save_heatmap(img);

            process_queue_ptr_->push(PublishImage(header, std::move(gray_img)));
        }
        catch (const std::exception &e)
        {
            node_.log_send_error("bgs_worker: image_callback: exception: %s", e.what());
        }
        catch (...)
        {
            node_.log_send_error("bgs_worker: image_callback: Unknown Exception");
        }
    }

    void recording_event(const bob_interfaces::msg::RecordingEvent &event) noexcept
    {
        last_recording_event_ = event;
    }

    void camera_info_callback(const bob_camera::msg::CameraInfo::SharedPtr camera_info_msg) noexcept
    {
        last_camera_info_ = camera_info_msg;
    }

    void save_json()
    {
        auto json_camera_info = JsonRecorder::build_json_camera_info(last_camera_info_);
        json_recorder_->add_to_buffer(json_camera_info, true);

        auto json_full_path = last_recording_event_.recording_path + "/json/" + last_recording_event_.filename + ".json";
        node_.log_send_info("BGSWorker: save_json: Writing JSON to: %s", json_full_path.c_str());
        json_recorder_->write_buffer_to_file(json_full_path);
    }

private:


    void tracking_callback(const bob_interfaces::msg::Tracking::SharedPtr tracking_msg)
    {
        if (!params_.get_recording_enabled())
        {
            return;
        }

        // Start recording heatmap when recording begins
        if (last_recording_event_.recording && !recording_json_)
        {
            recording_json_ = true;
        }
        else
        // Save heatmap when recording ends
        if (!last_recording_event_.recording && recording_json_)
        {
            recording_json_ = false;
            save_json();
        }

        auto json_data = JsonRecorder::build_json_value(tracking_msg, false);
        if (last_recording_event_.recording)
        {
            for (const auto &detection : tracking_msg->detections)
            {
                if (detection.state == 2) // ActiveTarget
                {
                    const auto &bbox = detection.bbox;
                    const double area = bbox.size_x * bbox.size_y;
                    img_recorder_->store_trajectory_point(detection.id, cv::Point(static_cast<int>(bbox.center.position.x), static_cast<int>(bbox.center.position.y)), area);
                }
            }
            json_recorder_->add_to_buffer(json_data, false);
        }
        else
        {
            json_recorder_->add_to_pre_buffer(json_data, false);
        }
    }

    inline void create_save_heatmap(const boblib::base::Image &img) noexcept
    {
        if (!params_.get_recording_enabled())   
        {
            return;
        }

        try
        {
            // Start recording heatmap when recording begins
            if (last_recording_event_.recording && !recording_heatmap_)
            {
                recording_heatmap_ = true;
                img_recorder_->update_frame_for_drawing(img.toMat());
                return;
            }

            // Save heatmap when recording ends
            if (!last_recording_event_.recording && recording_heatmap_)
            {
                recording_heatmap_ = false;
                
                // Ensure the recording path and filename are valid
                if (last_recording_event_.recording_path.empty() || last_recording_event_.filename.empty())
                {
                    node_.log_send_error("Cannot save heatmap: Invalid recording path or filename");
                    return;
                }
                
                // Create the full path for the heatmap
                std::string full_path = last_recording_event_.recording_path + "/heatmaps/" + last_recording_event_.filename + ".jpg";
                
                // Write the image and reset the recorder
                if (!img_recorder_->write_image(full_path))
                {
                    node_.log_send_error("Failed to write heatmap to: %s", full_path.c_str());
                }
                
                img_recorder_->reset();
            }
        }
        catch (const std::exception& e)
        {
            node_.log_send_error("Error in create_save_heatmap: %s", e.what());
        }
        catch (...)
        {
            node_.log_send_error("Unknown error in create_save_heatmap");
        }
    }

    inline void accumulate_mask(const boblib::base::Image &gray_img)
    {
        if (params_.get_recording_enabled() && last_recording_event_.recording)
        {
            img_recorder_->accumulate_mask(gray_img.toMat(), gray_img.size());
        }
    }

    void process_images()
    {
        boblib::base::Image blank_mask(using_cuda_);
        boblib::base::Image bgs_img(using_cuda_);

        while (rclcpp::ok())
        {
            std::unique_lock lock(mutex_);
            cv_.wait(lock, [this]{ return ready_; });

            processing_ = true;

            auto publish_image = process_queue_ptr_->pop();
            if (!publish_image.has_value())
            {
                processing_ = false;
                cv_.notify_all();
                continue;
            }

            try
            {
                if (process_queue_ptr_->size() > 0)
                {
                    node_.log_info("BGSWorker: Process Queue size: %d", process_queue_ptr_->size());
                }

                auto& header = publish_image.value().Header;
                auto& gray_img = publish_image.value().Image;

                // Resize detection mask if needed
                if (mask_enabled_ && params_.get_mask_enable_override() && 
                    (detection_mask_ptr_->size() != gray_img.size()))
                {
                    detection_mask_ptr_->resize(gray_img.size());
                }

                // Apply background subtraction
                const auto& mask = (mask_enabled_ && params_.get_mask_enable_override()) 
                    ? *detection_mask_ptr_ 
                    : blank_mask;
                bgs_ptr_->apply(gray_img, bgs_img, mask);

                // Process the results
                accumulate_mask(bgs_img);
                do_detection(header, bgs_img);
                publish_frame(header, bgs_img);
                publish_resized_frame(header, bgs_img);
            }
            catch (const std::exception& e)
            {
                node_.log_send_error("BGSWorker: process_images: Exception: %s", e.what());
                rcutils_reset_error();
            }
            catch (...)
            {
                node_.log_send_error("BGSWorker: process_images: Unknown exception");
                rcutils_reset_error();
            }

            processing_ = false;
            cv_.notify_all();
        }

        // Save final heatmap if recording was active
        if (img_recorder_)
        {
            std::string full_path = last_recording_event_.recording_path + 
                "/heatmaps/" + last_recording_event_.filename + ".jpg";
            img_recorder_->write_image(full_path);
        }
        if (json_recorder_)
        {
            save_json();
        }

        node_.log_send_info("BGSWorker: process_images: Leaving process_images");
    }

    inline void fill_header(sensor_msgs::msg::Image &bgs_msg, const std_msgs::msg::Header &header, const boblib::base::Image &bgs_img) noexcept
    {
        bgs_msg.header = header;
        bgs_msg.height = bgs_img.size().height;
        bgs_msg.width = bgs_img.size().width;
        bgs_msg.encoding = ImageUtils::type_to_encoding(bgs_img.type());
        bgs_msg.is_bigendian = (rcpputils::endian::native == rcpputils::endian::big);
        bgs_msg.step = bgs_img.size().width * bgs_img.elemSize();
    }

    inline void publish_frame(const std_msgs::msg::Header &header, boblib::base::Image &bgs_img) noexcept
    {
        if (node_.count_subscribers(params_.get_image_publisher()->get_topic_name()) <= 0)
        {
            return;
        }
        auto bgs_msg = PublishImage::fill_imagemsg_header(header, bgs_img);
        const size_t totalBytes = bgs_img.total() * bgs_img.elemSize();
        bgs_msg.data.assign(bgs_img.data(), bgs_img.data() + totalBytes);

        params_.get_image_publisher()->publish(bgs_msg);
    }

    inline void publish_resized_frame(const std_msgs::msg::Header &header, const boblib::base::Image & bgs_img) const
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

        auto resized_frame_msg = cv_bridge::CvImage(header, ImageUtils::type_to_encoding(resized_img.type()), resized_img.toMat()).toImageMsg();
        params_.get_image_resized_publisher()->publish(*resized_frame_msg);            
    }

    inline void do_detection(const std_msgs::msg::Header &header, boblib::base::Image &bgs_img)
    {
        // Initialize detector state and bounding box array messages
        bob_interfaces::msg::DetectorState state;
        state.sensitivity = params_.get_sensitivity();
        state.max_blobs_reached = false; // Initialize to false by default

        bob_interfaces::msg::DetectorBBoxArray bbox2D_array;
        bbox2D_array.header = header;
        bbox2D_array.image_width = bgs_img.size().width;
        bbox2D_array.image_height = bgs_img.size().height;

        // Apply median filter if enabled to reduce noise
        if (median_filter_)
        {
            bgs_img.medianBlur(3);
        }

        // Perform blob detection
        std::vector<cv::Rect> bboxes;
        const auto det_result = blob_detector_ptr_->detect(bgs_img, bboxes);

        // Handle detection results
        switch (det_result)
        {
            case boblib::blobs::DetectionResult::Success:
                add_bboxes(bbox2D_array, bboxes);
                break;
            
            case boblib::blobs::DetectionResult::MaxBlobsReached:
                state.max_blobs_reached = true;
                break;

            case boblib::blobs::DetectionResult::NoBlobsDetected:
                break;

            default:
                // Log unexpected detection result
                node_.log_send_warn("Unexpected detection result: %d", static_cast<int>(det_result));
                break;
        }

        // Publish results if there are subscribers
        node_.publish_if_subscriber(params_.get_state_publisher(), state);
        node_.publish_if_subscriber(params_.get_detection_publisher(), bbox2D_array);
    }

    inline void add_bboxes(bob_interfaces::msg::DetectorBBoxArray &bbox2D_array, const std::vector<cv::Rect> &bboxes) noexcept
    {
        bbox2D_array.detections.reserve(bboxes.size());
        for (const auto& bbox : bboxes)
        {
            bob_interfaces::msg::DetectorBBox detection;
            detection.x = bbox.x;
            detection.y = bbox.y; 
            detection.width = bbox.width;
            detection.height = bbox.height;
            bbox2D_array.detections.push_back(detection);
        }
    }

    inline std::unique_ptr<boblib::bgs::CoreBgs> create_bgs(BackgroundSubtractorWorkerParams::BGSType type)
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

    ParameterNode& node_;
    BackgroundSubtractorWorkerParams& params_;

    std::unique_ptr<MaskWorker> mask_worker_ptr_;

    bool mask_enabled_{false};
    bool median_filter_{false};
    std::unique_ptr<boblib::bgs::CoreBgs> bgs_ptr_{nullptr};
    std::unique_ptr<boblib::blobs::ConnectedBlobDetection> blob_detector_ptr_{nullptr};
    std::unique_ptr<boblib::bgs::VibeParams> vibe_params_;
    std::unique_ptr<boblib::bgs::WMVParams> wmv_params_;
    std::unique_ptr<boblib::blobs::ConnectedBlobDetectionParams> blob_params_;

    std::unique_ptr<boblib::base::Image> detection_mask_ptr_;

    std::condition_variable cv_;
    std::mutex mutex_;
    bool ready_{false};
    bool processing_{false};
    bool using_cuda_{false};

    bob_interfaces::msg::RecordingEvent last_recording_event_;
    bob_camera::msg::CameraInfo::SharedPtr last_camera_info_;
    std::unique_ptr<SynchronizedQueue<PublishImage>> process_queue_ptr_;
    rclcpp::Subscription<bob_interfaces::msg::Tracking>::SharedPtr tracking_subscription_;
    rclcpp::Subscription<bob_camera::msg::CameraInfo>::SharedPtr camera_info_subscription_;
    std::jthread process_thread_;

    std::unique_ptr<ImageRecorder> img_recorder_;
    std::unique_ptr<JsonRecorder> json_recorder_;
    bool recording_heatmap_{false};
    bool recording_json_{false};
    float fps_{-1.0f};
};
