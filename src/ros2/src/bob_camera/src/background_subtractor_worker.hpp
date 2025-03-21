#pragma once

#include <filesystem>
#include <mutex>
#include <condition_variable>

#include <boblib/api/bgs/bgs.hpp>
#include <boblib/api/bgs/WeightedMovingVariance/WeightedMovingVarianceUtils.hpp>
#include <boblib/api/blobs/connectedBlobDetection.hpp>
#include <boblib/api/base/Image.hpp>

#include "parameter_node.hpp"
#include "background_subtractor_worker_params.hpp"

#include "mask_worker.hpp"
#include "publish_image.hpp"
#include "image_recorder.hpp"

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
                img_recorder_ = std::make_unique<ImageRecorder>((size_t)(static_cast<int>(std::ceil(fps_)) * params_.get_recording_seconds_save()));
            }

            boblib::base::Image gray_img(using_cuda_);
            img.convertTo(gray_img, cv::COLOR_BGR2GRAY);

            sensor_msgs::msg::Image bgs_msg;
            fill_header(bgs_msg, header, gray_img);

            create_save_heatmap(img);

            process_queue_ptr_->push(PublishImage(std::move(bgs_msg), std::move(gray_img)));
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

private:
    inline void create_save_heatmap(const boblib::base::Image &img)
    {
        if (params_.get_recording_enabled())
        {
            if (last_recording_event_.recording && !recording_)
            {
                recording_ = true;
                img_recorder_->update_frame_for_drawing(img.toMat());
                return;
            }

            if (!last_recording_event_.recording && recording_)
            {
                recording_ = false;
                std::string full_path = last_recording_event_.recording_path + "/heatmaps/" + last_recording_event_.filename + ".jpg";
                img_recorder_->write_image(full_path);
                img_recorder_->reset();
            }
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
            if (publish_image.has_value())
            {
                try
                {
                    if (process_queue_ptr_->size() > 0)
                    {
                        node_.log_info("BGSWorker: Process Queue size: %d", process_queue_ptr_->size());
                    }

                    auto & camera_msg = publish_image.value().Image_msg;
                    auto & gray_img = publish_image.value().Image;

                    if (mask_enabled_ && params_.get_mask_enable_override() && (detection_mask_ptr_->size() != gray_img.size()))
                    {
                        detection_mask_ptr_->resize(gray_img.size());
                    }

                    bgs_ptr_->apply(gray_img, bgs_img, (mask_enabled_ && params_.get_mask_enable_override()) ? *detection_mask_ptr_ : blank_mask);

                    accumulate_mask(bgs_img);

                    do_detection(camera_msg.header, bgs_img);

                    publish_frame(camera_msg, bgs_img);

                    publish_resized_frame(camera_msg, bgs_img);
                }
                catch (const std::exception & e)
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
        }
        if (img_recorder_)
        {
            std::string full_path = last_recording_event_.recording_path + "/heatmaps/" + last_recording_event_.filename + ".jpg";
            img_recorder_->write_image(full_path);
        }

        node_.log_send_info("BGSWorker: process_images: Leaving publish_images");
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

    inline void publish_frame(sensor_msgs::msg::Image &bgs_msg, boblib::base::Image &bgs_img) noexcept
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

        auto resized_frame_msg = cv_bridge::CvImage(bgs_msg.header, bgs_msg.encoding, resized_img.toMat()).toImageMsg();
        params_.get_image_resized_publisher()->publish(*resized_frame_msg);            
    }

    inline void do_detection(const std_msgs::msg::Header &header, boblib::base::Image &bgs_img)
    {
        bob_interfaces::msg::DetectorState state;
        state.sensitivity = params_.get_sensitivity();
        bob_interfaces::msg::DetectorBBoxArray bbox2D_array;
        bbox2D_array.header = header;
        bbox2D_array.image_width = bgs_img.size().width;
        bbox2D_array.image_height = bgs_img.size().height;

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

    inline void add_bboxes(bob_interfaces::msg::DetectorBBoxArray &bbox2D_array, const std::vector<cv::Rect> &bboxes) noexcept
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

    std::unique_ptr<SynchronizedQueue<PublishImage>> process_queue_ptr_;
    std::jthread process_thread_;

    std::unique_ptr<ImageRecorder> img_recorder_;
    bool recording_{false};
    float fps_{-1.0f};
};
