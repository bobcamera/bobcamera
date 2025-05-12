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
                              const rclcpp::QoS &sub_qos_profile,
                              boblib::utils::pubsub::TopicManager &topic_manager)
        : node_(node),
          params_(params),
          sub_qos_profile_(sub_qos_profile),
          topic_manager_(topic_manager)
    {
    }

    ~CameraSaveWorker() noexcept
    {
        node_.log_info("CameraSaveWorker destructor");
        close_recorders();
    }

    void init()
    {
        recording_event_pubsub_ptr_ = topic_manager_.get_topic<bob_interfaces::msg::RecordingEvent>(params_.topics.recording_event_publisher_topic);
        recording_event_pubsub_ptr_->subscribe<CameraSaveWorker, &CameraSaveWorker::recording_event>(this);

        tracking_pubsub_ptr_ = topic_manager_.get_topic<bob_interfaces::msg::Tracking>(params_.topics.tracking_publisher_topic);
        tracking_pubsub_ptr_->subscribe<CameraSaveWorker, &CameraSaveWorker::tracking_callback>(this);

        image_pubsub_ptr_ = topic_manager_.get_topic<PublishImage>(params_.topics.image_publish_topic + "_publish");
        image_pubsub_ptr_->subscribe<CameraSaveWorker, &CameraSaveWorker::record_image>(this);

        bgs_pubsub_ptr_ = topic_manager_.get_topic<PublishImage>(params_.topics.image_publish_topic + "_process");
        bgs_pubsub_ptr_->subscribe<CameraSaveWorker, &CameraSaveWorker::bgs_image_callback>(this);
    }

private:
    void close_recorders()
    {
        if (video_recorder_ptr_ && video_recorder_ptr_->is_recording())
        {
            node_.log_info("Closing video");
            video_recorder_ptr_->close_video();
        }
        if (img_recorder_ && !last_recording_event_.recording_path.empty())
        {
            node_.log_info("Saving heatmap image");
            std::string full_path = last_recording_event_.recording_path +
                                    "/heatmaps/" + last_recording_event_.filename + ".jpg";
            img_recorder_->write_image(full_path);
            img_recorder_->reset();
        }
        if (json_recorder_)
        {
            node_.log_info("Saving JSON");
            save_json();
        }
    }

    void recording_event(const bob_interfaces::msg::RecordingEvent::SharedPtr &event) noexcept
    {
        last_recording_event_ = *event;
    }

    void save_json()
    {
        if (!json_recorder_ || last_recording_event_.recording_path.empty())
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
            if (!params_.recording.enabled || !last_recording_event_.recording || !img_recorder_)
            {
                return;
            }
            img_recorder_->accumulate_mask(camera_publish->image_ptr->toMat());
        }
        catch (const std::exception &e)
        {
            node_.log_send_error("CameraSaveWorker: process_images: Exception: %s", e.what());
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
            if (image_pubsub_ptr_->queue_size() > 0)
            {
                node_.log_info("CameraSaveWorker: image_pubsub_ptr_ queue size: %zu", image_pubsub_ptr_->queue_size());
            }

            auto &camera_img = *camera_publish->image_ptr;

            last_camera_info_ = *camera_publish->camera_info_ptr;
            if (last_camera_info_.fps != fps_)
            {
                close_recorders();
                fps_ = last_camera_info_.fps;
                auto total_pre_frames = (size_t)(static_cast<int>(std::ceil(fps_)) * params_.recording.seconds_save);
                img_recorder_ = std::make_unique<ImageRecorder>(total_pre_frames);
                json_recorder_ = std::make_unique<JsonRecorder>(total_pre_frames);
                video_recorder_ptr_ = std::make_unique<VideoRecorder>(total_pre_frames);
            }

            create_save_heatmap(camera_img);

            if (!video_recorder_ptr_ || last_recording_event_.recording_path.empty())
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
                    node_.log_info("Recording video frame");
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
    }

    void tracking_callback(const bob_interfaces::msg::Tracking::SharedPtr &tracking_msg) noexcept
    {
        try
        {
            if (!params_.recording.enabled || img_recorder_ == nullptr || json_recorder_ == nullptr || last_recording_event_.recording_path.empty())
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
                        const double area = bbox.width * bbox.height;
                        img_recorder_->store_trajectory_point(detection.id, cv::Point(static_cast<int>(bbox.x), static_cast<int>(bbox.y)), area);
                    }
                }
                json_recorder_->add_to_buffer(json_data, false);
            }
            else
            {
                json_recorder_->add_to_pre_buffer(json_data, false);
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
    const rclcpp::QoS &sub_qos_profile_;
    boblib::utils::pubsub::TopicManager &topic_manager_;

    bob_camera::msg::CameraInfo last_camera_info_;

    std::unique_ptr<ImageRecorder> img_recorder_;
    std::unique_ptr<JsonRecorder> json_recorder_;
    bool recording_heatmap_{false};
    bool recording_json_{false};
    float fps_{-1.0f};

    std::shared_ptr<boblib::utils::pubsub::PubSub<PublishImage>> image_pubsub_ptr_;
    std::shared_ptr<boblib::utils::pubsub::PubSub<PublishImage>> bgs_pubsub_ptr_;
    std::shared_ptr<boblib::utils::pubsub::PubSub<bob_interfaces::msg::RecordingEvent>> recording_event_pubsub_ptr_;
    std::shared_ptr<boblib::utils::pubsub::PubSub<bob_interfaces::msg::Tracking>> tracking_pubsub_ptr_;

    bob_interfaces::msg::RecordingEvent last_recording_event_;
    std::unique_ptr<VideoRecorder> video_recorder_ptr_;
};