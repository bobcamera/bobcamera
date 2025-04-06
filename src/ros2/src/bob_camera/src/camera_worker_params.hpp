#pragma once

#include <stddef.h>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <bob_camera/msg/image_info.hpp>
#include <bob_camera/msg/camera_info.hpp>
#include <bob_interfaces/srv/camera_settings.hpp>
#include <bob_interfaces/msg/recording_event.hpp>

class CameraWorkerParams
{
public:
    enum class SourceType
    {
        USB_CAMERA,
        VIDEO_FILE,
        RTSP_STREAM,
        UNKNOWN
    };

    // Constructor
    CameraWorkerParams() = default;

    // Getters
    [[nodiscard]] bool get_use_cuda() const { return use_cuda_; }
    [[nodiscard]] size_t get_max_queue_process_size() const { return max_queue_process_size_; }

    [[nodiscard]] const auto &get_image_publisher() const { return publishers_.image_publisher_; }
    [[nodiscard]] const auto &get_image_resized_publisher() const { return publishers_.image_resized_publisher_; }
    [[nodiscard]] const auto &get_image_info_publisher() const { return publishers_.image_info_publisher_; }
    [[nodiscard]] const auto &get_camera_info_publisher() const { return publishers_.camera_info_publisher_; }

    [[nodiscard]] const auto &get_recording_event_subscriber() const { return clients_.recording_event_subscriber_; }
    [[nodiscard]] const auto &get_camera_settings_client() const { return clients_.camera_settings_client_; }

    [[nodiscard]] SourceType get_source_type() const { return camera_params_.source_type_; }
    [[nodiscard]] int get_camera_id() const { return camera_params_.camera_id_; }
    [[nodiscard]] const auto &get_rtsp_uri() const { return camera_params_.rtsp_uri_; }
    [[nodiscard]] const auto &get_usb_resolution() const { return camera_params_.usb_resolution_; }
    [[nodiscard]] int get_resize_height() const { return camera_params_.resize_height_; }
    [[nodiscard]] const auto &get_videos() const { return camera_params_.videos_; }
    [[nodiscard]] bool get_limit_fps() const { return camera_params_.limit_fps_; }

    [[nodiscard]] const auto &get_image_publish_topic() const { return topics_params_.image_publish_topic_; }
    [[nodiscard]] const auto &get_image_info_publish_topic() const { return topics_params_.image_info_publish_topic_; }
    [[nodiscard]] const auto &get_camera_info_publish_topic() const { return topics_params_.camera_info_publish_topic_; }
    [[nodiscard]] const auto &get_recording_event_subscriber_topic() const { return topics_params_.recording_event_subscriber_topic_; }
    [[nodiscard]] const auto &get_image_resized_publish_topic() const { return topics_params_.image_resized_publish_topic_; }

    [[nodiscard]] const auto &get_onvif_host() const { return onvif_params_.onvif_host_; }
    [[nodiscard]] int get_onvif_port() const { return onvif_params_.onvif_port_; }
    [[nodiscard]] const auto &get_onvif_user() const { return onvif_params_.onvif_user_; }
    [[nodiscard]] const auto &get_onvif_password() const { return onvif_params_.onvif_password_; }

    [[nodiscard]] bool get_mask_enable_override() const { return mask_params_.mask_enable_override_; }
    [[nodiscard]] const auto &get_mask_filename() const { return mask_params_.mask_filename_; }
    [[nodiscard]] int get_mask_timer_seconds() const { return mask_params_.mask_timer_seconds_; }

    [[nodiscard]] int get_simulator_num_objects() const { return simulator_params_.simulator_num_objects_; }
    [[nodiscard]] bool get_simulator_enable() const { return simulator_params_.simulator_enable_; }

    [[nodiscard]] bool get_recording_enabled() const { return recording_params_.recording_enabled_; }
    [[nodiscard]] const std::string &get_recording_codec() const { return recording_params_.recording_codec_; }
    [[nodiscard]] int get_recording_seconds_save() const { return recording_params_.recording_seconds_save_; }

    // Setters
    void set_use_cuda(bool enable)
    {
        use_cuda_ = enable;
    }
    void set_max_queue_process_size(size_t size) { max_queue_process_size_ = size; }

    void set_image_publisher(const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr &publisher) { publishers_.image_publisher_ = publisher; }
    void set_image_resized_publisher(const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr &publisher) { publishers_.image_resized_publisher_ = publisher; }
    void set_image_info_publisher(const rclcpp::Publisher<bob_camera::msg::ImageInfo>::SharedPtr &publisher) { publishers_.image_info_publisher_ = publisher; }
    void set_camera_info_publisher(const rclcpp::Publisher<bob_camera::msg::CameraInfo>::SharedPtr &publisher) { publishers_.camera_info_publisher_ = publisher; }

    void set_recording_event_subscriber(const rclcpp::Subscription<bob_interfaces::msg::RecordingEvent>::SharedPtr &subscriber) { clients_.recording_event_subscriber_ = subscriber; }
    void set_camera_settings_client(const rclcpp::Client<bob_interfaces::srv::CameraSettings>::SharedPtr &client) { clients_.camera_settings_client_ = client; }

    void set_source_type(SourceType type) { camera_params_.source_type_ = type; }
    void set_camera_id(int id) { camera_params_.camera_id_ = id; }
    void set_rtsp_uri(const std::string &uri) { camera_params_.rtsp_uri_ = uri; }
    void set_usb_resolution(const std::vector<long> &resolution) { camera_params_.usb_resolution_ = resolution; }
    void set_resize_height(int height) { camera_params_.resize_height_ = height; }
    void set_videos(const std::vector<std::string> &videos) { camera_params_.videos_ = videos; }
    void set_limit_fps(bool enable) { camera_params_.limit_fps_ = enable; }

    void set_image_publish_topic(const std::string &topic) { topics_params_.image_publish_topic_ = topic; }
    void set_recording_event_subscriber_topic(const std::string &topic) { topics_params_.recording_event_subscriber_topic_ = topic; }
    void set_image_info_publish_topic(const std::string &topic) { topics_params_.image_info_publish_topic_ = topic; }
    void set_camera_info_publish_topic(const std::string &topic) { topics_params_.camera_info_publish_topic_ = topic; }
    void set_image_resized_publish_topic(const std::string &topic) { topics_params_.image_resized_publish_topic_ = topic; }

    void set_onvif_host(const std::string &host) { onvif_params_.onvif_host_ = host; }
    void set_onvif_port(int port) { onvif_params_.onvif_port_ = port; }
    void set_onvif_user(const std::string &user) { onvif_params_.onvif_user_ = user; }
    void set_onvif_password(const std::string &password) { onvif_params_.onvif_password_ = password; }

    void set_mask_enable_override(bool enable) { mask_params_.mask_enable_override_ = enable; }
    void set_mask_filename(const std::string &filename) { mask_params_.mask_filename_ = filename; }
    void set_mask_timer_seconds(int seconds) { mask_params_.mask_timer_seconds_ = seconds; }

    void set_simulator_num_objects(int num_objects) { simulator_params_.simulator_num_objects_ = num_objects; }
    void set_simulator_enable(bool enable) { simulator_params_.simulator_enable_ = enable; }

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

    struct CameraParams
    {
        SourceType source_type_{SourceType::UNKNOWN};
        int camera_id_;
        std::string rtsp_uri_;
        std::vector<long> usb_resolution_;
        int resize_height_{0};
        std::vector<std::string> videos_;
        bool limit_fps_{true};
    };

    struct OnvifParams
    {
        std::string onvif_host_;
        int onvif_port_;
        std::string onvif_user_;
        std::string onvif_password_;
    };

    struct TopicsParams
    {
        std::string image_publish_topic_;
        std::string image_info_publish_topic_;
        std::string camera_info_publish_topic_;
        std::string recording_event_subscriber_topic_;
        std::string image_resized_publish_topic_;
    };

    struct MaskParams
    {
        bool mask_enable_override_{false};
        std::string mask_filename_;
        int mask_timer_seconds_{5};
    };

    struct SimulatorParams
    {
        bool simulator_enable_{false};
        int simulator_num_objects_{0};
    };

    struct RecordingParams
    {
        bool recording_enabled_{false};
        std::string recording_codec_{"avc1"};
        int recording_seconds_save_{2};
    };

    bool use_cuda_{true};
    size_t max_queue_process_size_{0};

    CameraParams camera_params_;
    OnvifParams onvif_params_;
    TopicsParams topics_params_;
    MaskParams mask_params_;
    SimulatorParams simulator_params_;
    RecordingParams recording_params_;

    Publishers publishers_;
    Clients clients_;
};
