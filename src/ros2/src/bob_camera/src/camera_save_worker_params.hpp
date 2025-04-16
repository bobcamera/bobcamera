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
    [[nodiscard]] size_t get_max_queue_process_size() const { return max_queue_process_size_; }

    [[nodiscard]] const auto &get_image_publisher() const { return publishers_.image_publisher_; }
    [[nodiscard]] const auto &get_image_resized_publisher() const { return publishers_.image_resized_publisher_; }
    [[nodiscard]] const auto &get_image_info_publisher() const { return publishers_.image_info_publisher_; }
    [[nodiscard]] const auto &get_camera_info_publisher() const { return publishers_.camera_info_publisher_; }

    [[nodiscard]] const auto &get_recording_event_subscriber() const { return clients_.recording_event_subscriber_; }
    [[nodiscard]] const auto &get_camera_settings_client() const { return clients_.camera_settings_client_; }

    [[nodiscard]] const auto &get_image_publish_topic() const { return topics_params_.image_publish_topic_; }
    [[nodiscard]] const auto &get_image_info_publish_topic() const { return topics_params_.image_info_publish_topic_; }
    [[nodiscard]] const auto &get_camera_info_publish_topic() const { return topics_params_.camera_info_publish_topic_; }
    [[nodiscard]] const auto &get_recording_event_subscriber_topic() const { return topics_params_.recording_event_subscriber_topic_; }
    [[nodiscard]] const auto &get_image_resized_publish_topic() const { return topics_params_.image_resized_publish_topic_; }
    [[nodiscard]] const auto &get_camera_info_subscriber_topic() const { return topics_params_.camera_info_subscriber_topic_; }

    [[nodiscard]] bool get_recording_enabled() const { return recording_params_.recording_enabled_; }
    [[nodiscard]] const std::string &get_recording_codec() const { return recording_params_.recording_codec_; }
    [[nodiscard]] int get_recording_seconds_save() const { return recording_params_.recording_seconds_save_; }

    // Setters
    void set_max_queue_process_size(size_t size) { max_queue_process_size_ = size; }

    void set_image_publisher(const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr &publisher) { publishers_.image_publisher_ = publisher; }
    void set_image_resized_publisher(const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr &publisher) { publishers_.image_resized_publisher_ = publisher; }
    void set_image_info_publisher(const rclcpp::Publisher<bob_camera::msg::ImageInfo>::SharedPtr &publisher) { publishers_.image_info_publisher_ = publisher; }
    void set_camera_info_publisher(const rclcpp::Publisher<bob_camera::msg::CameraInfo>::SharedPtr &publisher) { publishers_.camera_info_publisher_ = publisher; }

    void set_recording_event_subscriber(const rclcpp::Subscription<bob_interfaces::msg::RecordingEvent>::SharedPtr &subscriber) { clients_.recording_event_subscriber_ = subscriber; }
    void set_camera_settings_client(const rclcpp::Client<bob_interfaces::srv::CameraSettings>::SharedPtr &client) { clients_.camera_settings_client_ = client; }

    void set_image_publish_topic(const std::string &topic) { topics_params_.image_publish_topic_ = topic; }
    void set_recording_event_subscriber_topic(const std::string &topic) { topics_params_.recording_event_subscriber_topic_ = topic; }
    void set_image_info_publish_topic(const std::string &topic) { topics_params_.image_info_publish_topic_ = topic; }
    void set_camera_info_publish_topic(const std::string &topic) { topics_params_.camera_info_publish_topic_ = topic; }
    void set_image_resized_publish_topic(const std::string &topic) { topics_params_.image_resized_publish_topic_ = topic; }
    void set_camera_info_subscriber_topic(const std::string &topic) { topics_params_.camera_info_subscriber_topic_ = topic; }

    void set_pre_recording_seconds(int seconds) { recording_params_.recording_seconds_save_ = seconds; }

    void set_recording_enabled(bool enable) { recording_params_.recording_enabled_ = enable; }
    void set_recording_codec(const std::string &codec) { recording_params_.recording_codec_ = codec; }
    void set_recording_seconds_save(int seconds) { recording_params_.recording_seconds_save_ = seconds; }

private:
    struct Publishers
    {
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_resized_publisher_;
        rclcpp::Publisher<bob_camera::msg::ImageInfo>::SharedPtr image_info_publisher_;
        rclcpp::Publisher<bob_camera::msg::CameraInfo>::SharedPtr camera_info_publisher_;
    };

    struct Clients
    {
        rclcpp::Subscription<bob_interfaces::msg::RecordingEvent>::SharedPtr recording_event_subscriber_;
        rclcpp::Client<bob_interfaces::srv::CameraSettings>::SharedPtr camera_settings_client_;
    };

    struct TopicsParams
    {
        std::string image_publish_topic_;
        std::string image_info_publish_topic_;
        std::string camera_info_publish_topic_;
        std::string recording_event_subscriber_topic_;
        std::string image_resized_publish_topic_;
        std::string camera_info_subscriber_topic_;
    };

    struct RecordingParams
    {
        bool recording_enabled_{false};
        std::string recording_codec_{"avc1"};
        int recording_seconds_save_{2};
    };

    size_t max_queue_process_size_{0};

    TopicsParams topics_params_;
    RecordingParams recording_params_;

    Publishers publishers_;
    Clients clients_;
};
