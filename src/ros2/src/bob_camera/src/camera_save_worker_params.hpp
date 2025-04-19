#pragma once

#include <stddef.h>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <bob_camera/msg/image_info.hpp>
#include <bob_camera/msg/camera_info.hpp>
#include <bob_interfaces/srv/camera_settings.hpp>
#include <bob_interfaces/msg/recording_event.hpp>

class CameraSaveWorkerParams
{
public:
    CameraSaveWorkerParams() = default;

    // Getters
    [[nodiscard]] const auto &get_tracking_subscriber_topic() const { return topics_params_.tracking_subscriber_topic_; }
    [[nodiscard]] const auto &get_camera_info_subscriber_topic() const { return topics_params_.camera_info_subscriber_topic_; }
    [[nodiscard]] const auto &get_image_publish_topic() const { return topics_params_.image_publish_topic_; }
    [[nodiscard]] int get_recording_seconds_save() const { return recording_params_.recording_seconds_save_; }
    [[nodiscard]] bool get_recording_enabled() const { return recording_params_.recording_enabled_; }
    [[nodiscard]] const std::string &get_recording_codec() const { return recording_params_.recording_codec_; }

    // Setters
    void set_tracking_subscriber_topic(const std::string &topic) { topics_params_.tracking_subscriber_topic_ = topic; }
    void set_camera_info_subscriber_topic(const std::string &topic) { topics_params_.camera_info_subscriber_topic_ = topic; }
    void set_image_publish_topic(const std::string &topic) { topics_params_.image_publish_topic_ = topic; }
    void set_recording_seconds_save(int seconds) { recording_params_.recording_seconds_save_ = seconds; }
    void set_recording_enabled(bool enable) { recording_params_.recording_enabled_ = enable; }
    void set_recording_codec(const std::string &codec) { recording_params_.recording_codec_ = codec; }

private:
    struct TopicsParams
    {
        std::string image_publish_topic_;
        std::string camera_info_subscriber_topic_;
        std::string tracking_subscriber_topic_;
    };

    struct RecordingParams
    {
        bool recording_enabled_{false};
        std::string recording_codec_{"avc1"};
        int recording_seconds_save_{2};
    };

    TopicsParams topics_params_;
    RecordingParams recording_params_;
};
