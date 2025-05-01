#pragma once

#include <filesystem>
#include <thread>

#include <boblib/api/video/VideoReader.hpp>
#include <boblib/api/base/SynchronizedQueue.hpp>
#include <boblib/api/utils/pubsub/TopicManager.hpp>

#include <mask_worker.hpp>
#include <circuit_breaker.hpp>
#include <video_recorder.hpp>

#include <parameter_node.hpp>

#include "camera_bgs_params.hpp"

#include "publish_image.hpp"
#include "image_recorder.hpp"
#include "json_recorder.hpp"

class CameraSaveWorker final
{
public:
    explicit CameraSaveWorker(ParameterNode &node,
                              CameraBgsParams &params,
                              boblib::utils::pubsub::TopicManager &topic_manager)
        : node_(node),
          params_(params),
          topic_manager_(topic_manager)
    {
    }

    ~CameraSaveWorker()
    {
        node_.log_info("CameraSaveWorker destructor");
        if (video_recorder_ptr_ && video_recorder_ptr_->is_recording())
        {
            node_.log_info("Closing video");
            video_recorder_ptr_->close_video();
        }
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
    }

    void init()
    {
        rclcpp::QoS sub_qos_profile(4);
        sub_qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        sub_qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);
        sub_qos_profile.history(rclcpp::HistoryPolicy::KeepLast);

        recording_event_subscriber_ = node_.create_subscription<bob_interfaces::msg::RecordingEvent>(params_.topics.recording_event_subscriber_topic, sub_qos_profile,
                                                                                     [this](const bob_interfaces::msg::RecordingEvent::SharedPtr event)
                                                                                     { recording_event(*event); });

        tracking_subscription_ = node_.create_subscription<bob_interfaces::msg::Tracking>(params_.topics.tracking_publisher_topic, sub_qos_profile,
                                                                                          [this](const bob_interfaces::msg::Tracking::SharedPtr tracking_msg)
                                                                                          { tracking_callback(tracking_msg); });

        camera_info_subscription_ = node_.create_subscription<bob_camera::msg::CameraInfo>(params_.topics.camera_info_publish_topic, sub_qos_profile,
                                                                                           [this](const bob_camera::msg::CameraInfo::SharedPtr camera_info_msg)
                                                                                           { camera_info_callback(camera_info_msg); });

        image_pubsub_ptr_ = topic_manager_.get_topic<PublishImage>(params_.topics.image_publish_topic + "_publish");
        image_pubsub_ptr_->subscribe<CameraSaveWorker, &CameraSaveWorker::record_image>(this);

        bgs_pubsub_ptr_ = topic_manager_.get_topic<PublishImage>(params_.topics.image_publish_topic + "_process");
        bgs_pubsub_ptr_->subscribe<CameraSaveWorker, &CameraSaveWorker::bgs_image_callback>(this);
    }

    void recording_event(const bob_interfaces::msg::RecordingEvent &event) noexcept
    {
        last_recording_event_ = event;
    }

    void camera_info_callback(const bob_camera::msg::CameraInfo::SharedPtr camera_info_msg) noexcept
    {
        last_camera_info_ = camera_info_msg;
        if (last_camera_info_->fps != fps_)
        {
            fps_ = last_camera_info_->fps;
            auto total_pre_frames = (size_t)(static_cast<int>(std::ceil(fps_)) * params_.recording.seconds_save);
            img_recorder_ = std::make_unique<ImageRecorder>(total_pre_frames);
            json_recorder_ = std::make_unique<JsonRecorder>(total_pre_frames);
            video_recorder_ptr_ = std::make_unique<VideoRecorder>(total_pre_frames);
        }
    }

private:
    void save_json()
    {
        if (!json_recorder_)
        {
            return;
        }
        auto json_camera_info = JsonRecorder::build_json_camera_info(last_camera_info_);
        json_recorder_->add_to_buffer(json_camera_info, true);

        auto json_full_path = last_recording_event_.recording_path + "/json/" + last_recording_event_.filename + ".json";
        node_.log_send_info("CameraSaveWorker: save_json: Writing JSON to: %s", json_full_path.c_str());
        json_recorder_->write_buffer_to_file(json_full_path);
    }

    void bgs_image_callback(const std::shared_ptr<PublishImage> &camera_publish) noexcept
    {
        try
        {
            accumulate_mask(*camera_publish->imagePtr);
        }
        catch (const std::exception &e)
        {
            node_.log_send_error("CameraSaveWorker: process_images: Exception: %s", e.what());
            rcutils_reset_error();
        }
        catch (...)
        {
            node_.log_send_error("CameraSaveWorker: process_images: Unknown exception");
            rcutils_reset_error();
        }
    }

    void record_image(const std::shared_ptr<PublishImage> &camera_publish) noexcept
    {
        try
        {
            if (!params_.recording.enabled)
            {
                return;
            }
            auto &camera_img = *camera_publish->imagePtr;

            create_save_heatmap(camera_img);

            if (!video_recorder_ptr_)
            {
                return;
            }

            if (last_recording_event_.recording)
            {
                if (!video_recorder_ptr_->is_recording())
                {
                    const auto complete_filename = last_recording_event_.recording_path + "/" + last_recording_event_.filename + ".mp4";
                    node_.log_info("Opening new video: %s", complete_filename.c_str());
                    if (!video_recorder_ptr_->open_new_video(complete_filename, params_.recording.codec, fps_, camera_img.size()))
                    {
                        node_.log_info("Could not create new video");
                    }
                }

                if (video_recorder_ptr_->is_recording())
                {
                    video_recorder_ptr_->write_frame(camera_img);
                }
            }
            else
            {
                if (video_recorder_ptr_->is_recording())
                {
                    node_.log_info("Closing video");
                    video_recorder_ptr_->close_video();
                }
                video_recorder_ptr_->add_to_pre_buffer(camera_img);
            }
        }
        catch (const std::exception &e)
        {
            node_.log_send_error("CameraSaveWorker: process_images: Exception: %s", e.what());
            rcutils_reset_error();
        }
        catch (...)
        {
            node_.log_send_error("CameraSaveWorker: process_images: Unknown exception");
            rcutils_reset_error();
        }
    }

    inline void accumulate_mask(const boblib::base::Image &gray_img)
    {
        if (params_.recording.enabled && last_recording_event_.recording && img_recorder_ != nullptr)
        {
            img_recorder_->accumulate_mask(gray_img.toMat());
        }
    }

    void tracking_callback(const bob_interfaces::msg::Tracking::SharedPtr tracking_msg)
    {
        if (!params_.recording.enabled || img_recorder_ == nullptr || json_recorder_ == nullptr)
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
        if (!params_.recording.enabled || img_recorder_ == nullptr)
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
                    node_.log_send_error("CameraSaveWorker: Cannot save heatmap: Invalid recording path or filename");
                    return;
                }

                // Create the full path for the heatmap
                std::string full_path = last_recording_event_.recording_path + "/heatmaps/" + last_recording_event_.filename + ".jpg";

                // Write the image and reset the recorder
                if (!img_recorder_->write_image(full_path))
                {
                    node_.log_send_error("CameraSaveWorker: Failed to write heatmap to: %s", full_path.c_str());
                }

                img_recorder_->reset();
            }
        }
        catch (const std::exception &e)
        {
            node_.log_send_error("CameraSaveWorker: Error in create_save_heatmap: %s", e.what());
        }
        catch (...)
        {
            node_.log_send_error("CameraSaveWorker: Unknown error in create_save_heatmap");
        }
    }

    ParameterNode &node_;
    CameraBgsParams &params_;
    boblib::utils::pubsub::TopicManager &topic_manager_;

    bob_camera::msg::CameraInfo::SharedPtr last_camera_info_;

    rclcpp::Subscription<bob_interfaces::msg::RecordingEvent>::SharedPtr recording_event_subscriber_;
    rclcpp::Subscription<bob_interfaces::msg::Tracking>::SharedPtr tracking_subscription_;
    rclcpp::Subscription<bob_camera::msg::CameraInfo>::SharedPtr camera_info_subscription_;
    std::unique_ptr<ImageRecorder> img_recorder_;
    std::unique_ptr<JsonRecorder> json_recorder_;
    bool recording_heatmap_{false};
    bool recording_json_{false};
    float fps_{-1.0f};

    std::shared_ptr<boblib::utils::pubsub::PubSub<PublishImage>> image_pubsub_ptr_;
    std::shared_ptr<boblib::utils::pubsub::PubSub<PublishImage>> bgs_pubsub_ptr_;

    bob_interfaces::msg::RecordingEvent last_recording_event_;
    std::unique_ptr<VideoRecorder> video_recorder_ptr_;
};