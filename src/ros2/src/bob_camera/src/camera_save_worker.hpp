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

#include "camera_save_worker_params.hpp"
#include "publish_image.hpp"
#include "image_recorder.hpp"
#include "json_recorder.hpp"

class CameraSaveWorker final
{
public:
    explicit CameraSaveWorker(ParameterNode &node,
                              CameraSaveWorkerParams &params,
                              boblib::utils::pubsub::TopicManager &topic_manager)
        : node_(node),
          params_(params),
          topic_manager_(topic_manager)
    {
    }

    ~CameraSaveWorker()
    {
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

        camera_info_subscription_ = node_.create_subscription<bob_camera::msg::CameraInfo>(params_.get_camera_info_subscriber_topic(), sub_qos_profile,
                                                                                           [this](const bob_camera::msg::CameraInfo::SharedPtr camera_info_msg)
                                                                                           { camera_info_callback(camera_info_msg); });

        image_pubsub_ptr_ = topic_manager_.get_topic<boblib::base::Image>(params_.get_image_publish_topic());
        image_pubsub_ptr_->subscribe([this](const boblib::base::Image &image)
                                     { record_image(image); });
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
            auto total_pre_frames = (size_t)(static_cast<int>(std::ceil(fps_)) * params_.get_recording_seconds_save());
            //img_recorder_ = std::make_unique<ImageRecorder>(total_pre_frames);
            //json_recorder_ = std::make_unique<JsonRecorder>(total_pre_frames);
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
        node_.log_send_info("BGSWorker: save_json: Writing JSON to: %s", json_full_path.c_str());
        json_recorder_->write_buffer_to_file(json_full_path);
    }

    void record_image(const boblib::base::Image &camera_img) noexcept
    {
        try
        {
            if (!params_.get_recording_enabled() || !video_recorder_ptr_)
            {
                return;
            }
            if (last_recording_event_.recording)
            {
                if (!video_recorder_ptr_->is_recording())
                {
                    const auto complete_filename = last_recording_event_.recording_path + "/" + last_recording_event_.filename + ".mp4";
                    node_.log_info("Opening new video: %s", complete_filename.c_str());
                    if (!video_recorder_ptr_->open_new_video(complete_filename, params_.get_recording_codec(), fps_, camera_img.size()))
                    {
                        node_.log_info("Could not create new video");
                    }
                }

                if (video_recorder_ptr_->is_recording())
                {
                    video_recorder_ptr_->write_frame(std::move(camera_img));
                }
            }
            else
            {
                if (video_recorder_ptr_->is_recording())
                {
                    node_.log_info("Closing video");
                    video_recorder_ptr_->close_video();
                }
                video_recorder_ptr_->add_to_pre_buffer(std::move(camera_img));
            }
        }
        catch (const std::exception &e)
        {
            node_.log_send_error("CameraWorker: process_images: Exception: %s", e.what());
            rcutils_reset_error();
        }
        catch (...)
        {
            node_.log_send_error("CameraWorker: process_images: Unknown exception");
            rcutils_reset_error();
        }
    }

    ParameterNode & node_;
    CameraSaveWorkerParams & params_;
    boblib::utils::pubsub::TopicManager &topic_manager_;

    bob_camera::msg::CameraInfo::SharedPtr last_camera_info_;

    rclcpp::Subscription<bob_camera::msg::CameraInfo>::SharedPtr camera_info_subscription_;
    std::unique_ptr<ImageRecorder> img_recorder_;
    std::unique_ptr<JsonRecorder> json_recorder_;
    bool recording_heatmap_{false};
    bool recording_json_{false};
    float fps_{-1.0f};

    std::shared_ptr<boblib::utils::pubsub::PubSub<boblib::base::Image>> image_pubsub_ptr_;

    bob_interfaces::msg::RecordingEvent last_recording_event_;
    std::unique_ptr<VideoRecorder> video_recorder_ptr_;
};