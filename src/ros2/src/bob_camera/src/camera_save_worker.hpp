#pragma once

#include <filesystem>
#include <thread>
#include <mutex>

#include <boblib/api/video/VideoReader.hpp>
#include <boblib/api/base/SynchronizedQueue.hpp>
#include <boblib/api/utils/pubsub/TopicManager.hpp>
#include <boblib/api/utils/fps_tracker.hpp>
#include <boblib/api/utils/profiler.hpp>

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
    explicit CameraSaveWorker(ParameterNode &node
                              , CameraBgsParams &params
                              , const rclcpp::QoS &sub_qos_profile
                              , boblib::utils::pubsub::TopicManager &topic_manager
                              , boblib::utils::Profiler &profiler)
        : node_(node)
        , params_(params)
        , pub_qos_profile_(sub_qos_profile)
        , topic_manager_(topic_manager)
        , profiler_(profiler)
    {
    }

    ~CameraSaveWorker() noexcept
    {
        node_.log_info("CameraSaveWorker destructor");
        recording_event_pubsub_ptr_.reset();
        image_pubsub_ptr_.reset();
        bgs_pubsub_ptr_.reset();
        tracking_pubsub_ptr_.reset();
        close_recorders();
    }

    void init()
    {
        prof_save_worker_id_ = profiler_.add_region("Save Worker");
        prof_image_id_ = profiler_.add_region("Image", prof_save_worker_id_);
        prof_heatmap_id_ = profiler_.add_region("Heatmap", prof_save_worker_id_);
        prof_json_id_ = profiler_.add_region("Json", prof_save_worker_id_);

        recording_event_pubsub_ptr_ = topic_manager_.get_topic<bob_interfaces::msg::RecordingEvent>(params_.topics.recording_event_publisher_topic);
        recording_event_pubsub_ptr_->subscribe<CameraSaveWorker, &CameraSaveWorker::recording_event>(this);

        tracking_pubsub_ptr_ = topic_manager_.get_topic<bob_camera::Tracking>(params_.topics.tracking_publisher_topic);
        tracking_pubsub_ptr_->subscribe<CameraSaveWorker, &CameraSaveWorker::tracking_callback>(this);

        image_pubsub_ptr_ = topic_manager_.get_topic<PublishImage>(params_.topics.image_publish_topic + "_publish");
        image_pubsub_ptr_->subscribe<CameraSaveWorker, &CameraSaveWorker::record_image>(this);

        bgs_pubsub_ptr_ = topic_manager_.get_topic<PublishImage>(params_.topics.image_publish_topic + "_process");
        bgs_pubsub_ptr_->subscribe<CameraSaveWorker, &CameraSaveWorker::bgs_image_callback>(this);
    }

private:
    void close_recorders()
    {
        std::lock_guard<std::recursive_mutex> lk(recording_mutex_);
        if (!recording_)
        {
            return;
        }
        node_.log_info("Closing recorders");
        recording_ = false;

        if (img_recorder_)
        {
            std::string full_path = last_recording_event_.recording_path +
                                    "/heatmaps/" + last_recording_event_.filename + ".jpg";
            node_.log_info("Saving heatmap image to: %s", full_path.c_str());
            img_recorder_->write_image(full_path);
            img_recorder_->reset();
        }
        if (json_recorder_)
        {
            auto json_full_path = last_recording_event_.recording_path + "/json/" + last_recording_event_.filename + ".json";
            node_.log_send_info("Writing JSON to: %s", json_full_path.c_str());

            auto json_camera_info = JsonRecorder::build_json_camera_info(last_camera_info_);
            json_recorder_->add_to_buffer(json_camera_info, true);
            json_recorder_->write_buffer_to_file(json_full_path);
        }
        if (video_recorder_ptr_)
        {
            node_.log_info("Closing video");
            video_recorder_ptr_->close_video();
        }
    }

    void open_recorders()
    {
        std::lock_guard<std::recursive_mutex> lk(recording_mutex_);
        if (recording_ || !last_recording_event_.recording)
        {
            return;
        }

        if (video_recorder_ptr_)
        {
            const auto complete_filename = last_recording_event_.recording_path + "/" + last_recording_event_.filename + ".mp4";
            node_.log_send_info("Opening new video: %s", complete_filename.c_str());
            if (!video_recorder_ptr_->open_new_video(complete_filename, params_.recording.codec, fps_, last_camera_img_.size()))
            {
                node_.log_send_error("Could not create new video");
            }
            node_.log_send_info("Video backend: %s", video_recorder_ptr_->get_video_backend().c_str());
        }

        if (img_recorder_)
        {
            node_.log_send_info("Creating new image recorder");
            img_recorder_->reset();
            img_recorder_->update_frame_for_drawing(last_camera_img_.toMat());
        }

        if (json_recorder_)
        {
            node_.log_send_info("Creating new json recorder");
            json_recorder_->reset();
        }

        recording_ = true;
    }

    void recording_event(const bob_interfaces::msg::RecordingEvent::SharedPtr &event) noexcept
    {
        last_recording_event_ = *event;

        if (!params_.recording.enabled)
        {
            return;
        }

        if (last_recording_event_.recording)
        {
            if (!recording_)
            {
                open_recorders();
            }
        }
        else if (recording_)
        {
            close_recorders();
        }
    }

    void bgs_image_callback(const std::shared_ptr<PublishImage> &camera_publish) noexcept
    {
        std::lock_guard<std::recursive_mutex> lk(recording_mutex_);
        try
        {
            if (!params_.recording.enabled || !recording_)
            {
                return;
            }
            profiler_.start(prof_heatmap_id_);
            img_recorder_->accumulate_mask(camera_publish->image_ptr->toMat());
            profiler_.stop(prof_heatmap_id_);
        }
        catch (const std::exception &e)
        {
            node_.log_send_error("CameraSaveWorker: process_images: Exception: %s", e.what());
            rcutils_reset_error();
        }
    }

    void record_image(const std::shared_ptr<PublishImage> &camera_publish) noexcept
    {
        std::lock_guard<std::recursive_mutex> lk(recording_mutex_);
        try
        {
            if (!params_.recording.enabled)
            {
                return;
            }
            profiler_.start(prof_image_id_);

            auto &camera_img = *camera_publish->image_ptr;

            last_camera_info_ = *camera_publish->camera_info_ptr;
            camera_img.copyTo(last_camera_img_);
            if (last_camera_info_.fps != fps_)
            {
                close_recorders();
                fps_ = last_camera_info_.fps;
                node_.log_send_info("CameraSaveWorker: Creating recorders for fps %f", fps_);
                auto total_pre_frames = (size_t)(static_cast<int>(std::ceil(fps_)) * params_.recording.seconds_save);
                img_recorder_ = std::make_unique<ImageRecorder>(total_pre_frames);
                json_recorder_ = std::make_unique<JsonRecorder>(total_pre_frames);
                video_recorder_ptr_ = std::make_unique<VideoRecorder>(total_pre_frames, params_.camera.use_opencv);
                open_recorders();
            }

            if (recording_)
            {
                video_recorder_ptr_->write_frame(camera_img);
            }
            else
            {
                video_recorder_ptr_->add_to_pre_buffer(camera_img);
            }
            profiler_.stop(prof_image_id_);
        }
        catch (const std::exception &e)
        {
            node_.log_send_error("CameraSaveWorker: process_images: Exception: %s", e.what());
            rcutils_reset_error();
        }
    }

    void tracking_callback(const std::shared_ptr<bob_camera::Tracking> &tracking_msg) noexcept
    {
        std::lock_guard<std::recursive_mutex> lk(recording_mutex_);
        try
        {
            if (!params_.recording.enabled)
            {
                return;
            }

            profiler_.start(prof_json_id_);
            auto json_data = JsonRecorder::build_json_value(tracking_msg, false);
            if (recording_)
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
            profiler_.stop(prof_json_id_);
        }
        catch (const std::exception &e)
        {
            node_.log_send_error("CameraSaveWorker: process_images: Exception: %s", e.what());
            rcutils_reset_error();
        }
    }

    ParameterNode &node_;
    CameraBgsParams &params_;
    const rclcpp::QoS &pub_qos_profile_;
    boblib::utils::pubsub::TopicManager &topic_manager_;
    boblib::utils::Profiler &profiler_;

    std::unique_ptr<ImageRecorder> img_recorder_;
    std::unique_ptr<JsonRecorder> json_recorder_;
    std::unique_ptr<VideoRecorder> video_recorder_ptr_;

    bool recording_{false};
    mutable std::recursive_mutex recording_mutex_;

    float fps_{-1.0f};
    boblib::base::Image last_camera_img_;
    bob_camera::msg::CameraInfo last_camera_info_;
    bob_interfaces::msg::RecordingEvent last_recording_event_;

    std::shared_ptr<boblib::utils::pubsub::PubSub<PublishImage>> image_pubsub_ptr_;
    std::shared_ptr<boblib::utils::pubsub::PubSub<PublishImage>> bgs_pubsub_ptr_;
    std::shared_ptr<boblib::utils::pubsub::PubSub<bob_interfaces::msg::RecordingEvent>> recording_event_pubsub_ptr_;
    std::shared_ptr<boblib::utils::pubsub::PubSub<bob_camera::Tracking>> tracking_pubsub_ptr_;

    size_t prof_save_worker_id_;
    size_t prof_image_id_;
    size_t prof_heatmap_id_;
    size_t prof_json_id_;
};