#pragma once

#include <stddef.h>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <bob_camera/msg/image_info.hpp>
#include <bob_camera/msg/camera_info.hpp>
#include <bob_interfaces/srv/camera_settings.hpp>
#include <bob_interfaces/msg/recording_event.hpp>

#include "background_subtractor_companion.hpp"

class CameraBgsParams
{
public:
    enum class SourceType
    {
        USB_CAMERA,
        VIDEO_FILE,
        RTSP_STREAM,
        UNKNOWN
    };
    enum class BGSType
    {
        Unknown,
        Vibe,
        WMV
    };

    // Constructor
    CameraBgsParams() = default;

    // Getters
    [[nodiscard]] bool get_use_cuda() const { return use_cuda_; }
    [[nodiscard]] int get_resize_height() const { return resize_height_; }

    [[nodiscard]] auto &get_camera() { return camera_params_; }
    [[nodiscard]] auto &get_topics() { return topics_params_; }
    [[nodiscard]] auto &get_recording() { return recording_params_; }
    [[nodiscard]] auto &get_bgs() { return bgs_params_; }

    // Setters
    void set_use_cuda(bool enable) { use_cuda_ = enable; }
    void set_resize_height(int height) { resize_height_ = height; }

    struct MaskParams
    {
        [[nodiscard]] bool get_mask_enable_override() const { return mask_enable_override_; }
        [[nodiscard]] const auto &get_mask_filename() const { return mask_filename_; }
        [[nodiscard]] int get_mask_timer_seconds() const { return mask_timer_seconds_; }

        void set_mask_enable_override(bool enable) { mask_enable_override_ = enable; }
        void set_mask_filename(const std::string &filename) { mask_filename_ = filename; }
        void set_mask_timer_seconds(int seconds) { mask_timer_seconds_ = seconds; }

    private:
        bool mask_enable_override_{false};
        std::string mask_filename_;
        int mask_timer_seconds_{5};
    };

    struct SimulatorParams
    {
        [[nodiscard]] bool get_simulator_enable() const { return simulator_enable_; }
        [[nodiscard]] int get_simulator_num_objects() const { return simulator_num_objects_; }
        void set_simulator_enable(bool enable) { simulator_enable_ = enable; }
        void set_simulator_num_objects(int num_objects) { simulator_num_objects_ = num_objects; }

    private:
        bool simulator_enable_{false};
        int simulator_num_objects_{0};
    };

    struct CameraParams
    {
        [[nodiscard]] SourceType get_source_type() const { return source_type_; }
        [[nodiscard]] int get_camera_id() const { return camera_id_; }
        [[nodiscard]] const auto &get_rtsp_uri() const { return rtsp_uri_; }
        [[nodiscard]] const auto &get_onvif_uri() const { return onvif_uri_; }
        [[nodiscard]] const auto &get_usb_resolution() const { return usb_resolution_; }
        [[nodiscard]] const auto &get_videos() const { return videos_; }
        [[nodiscard]] bool get_limit_fps() const { return limit_fps_; }
        [[nodiscard]] auto &get_privacy_mask() { return privacy_mask_params_; }
        [[nodiscard]] auto &get_simulator() { return simulator_params_; }

        void set_source_type(SourceType type) { source_type_ = type; }
        void set_camera_id(int id) { camera_id_ = id; }
        void set_rtsp_uri(const std::string &uri) { rtsp_uri_ = uri; }
        void set_onvif_uri(const std::string &uri) { onvif_uri_ = uri; }
        void set_usb_resolution(const std::vector<long> &resolution) { usb_resolution_ = resolution; }
        void set_videos(const std::vector<std::string> &videos) { videos_ = videos; }
        void set_limit_fps(bool enable) { limit_fps_ = enable; }

    private:
        SourceType source_type_{SourceType::UNKNOWN};
        int camera_id_;
        std::string rtsp_uri_;
        std::string onvif_uri_;
        std::vector<long> usb_resolution_;
        std::vector<std::string> videos_;
        bool limit_fps_{true};
        MaskParams privacy_mask_params_;
        SimulatorParams simulator_params_;
    };

    struct TopicsParams
    {
        [[nodiscard]] const auto &get_image_publish_topic() const { return image_publish_topic_; }
        [[nodiscard]] const auto &get_image_info_publish_topic() const { return image_info_publish_topic_; }
        [[nodiscard]] const auto &get_camera_info_publish_topic() const { return camera_info_publish_topic_; }
        [[nodiscard]] const auto &get_recording_event_subscriber_topic() const { return recording_event_subscriber_topic_; }
        [[nodiscard]] const auto &get_image_resized_publish_topic() const { return image_resized_publish_topic_; }
        [[nodiscard]] const auto &get_camera_settings_client_topic() const { return camera_settings_client_topic_; }
        [[nodiscard]] const auto &get_tracking_publisher_topic() const { return tracking_publisher_topic_; }
        [[nodiscard]] const auto &get_detection_publish_topic() const { return detection_publish_topic_; }
        [[nodiscard]] const auto &get_detection_state_publish_topic() const { return detection_state_publish_topic_; }
        [[nodiscard]] const auto &get_bgs_image_publish_topic() const { return bgs_image_publish_topic_; }

        void set_image_publish_topic(const std::string &topic) { image_publish_topic_ = topic; }
        void set_recording_event_subscriber_topic(const std::string &topic) { recording_event_subscriber_topic_ = topic; }
        void set_image_info_publish_topic(const std::string &topic) { image_info_publish_topic_ = topic; }
        void set_camera_info_publish_topic(const std::string &topic) { camera_info_publish_topic_ = topic; }
        void set_image_resized_publish_topic(const std::string &topic) { image_resized_publish_topic_ = topic; }
        void set_camera_settings_client_topic(const std::string &topic) { camera_settings_client_topic_ = topic; }
        void set_tracking_publisher_topic(const std::string &topic) { tracking_publisher_topic_ = topic; }
        void set_detection_publish_topic(const std::string &topic) { detection_publish_topic_ = topic; }
        void set_detection_state_publish_topic(const std::string &topic) { detection_state_publish_topic_ = topic; }
        void set_bgs_image_publish_topic(const std::string &topic) { bgs_image_publish_topic_ = topic; }

    private:
        std::string image_publish_topic_;
        std::string image_info_publish_topic_;
        std::string camera_info_publish_topic_;
        std::string recording_event_subscriber_topic_;
        std::string image_resized_publish_topic_;
        std::string camera_settings_client_topic_;
        std::string tracking_publisher_topic_;
        std::string detection_publish_topic_;
        std::string detection_state_publish_topic_;
        std::string bgs_image_publish_topic_;
    };

    struct RecordingParams
    {
        [[nodiscard]] bool get_recording_enabled() const { return recording_enabled_; }
        [[nodiscard]] const auto &get_recording_codec() const { return recording_codec_; }
        [[nodiscard]] int get_recording_seconds_save() const { return recording_seconds_save_; }

        void set_recording_seconds_save(int seconds) { recording_seconds_save_ = seconds; }
        void set_recording_enabled(bool enable) { recording_enabled_ = enable; }
        void set_recording_codec(const std::string &codec) { recording_codec_ = codec; }

    private:
        bool recording_enabled_{false};
        std::string recording_codec_{"avc1"};
        int recording_seconds_save_{2};
    };

    struct BgsParams
    {
        [[nodiscard]] BGSType get_bgs_type() const { return bgs_type_; }
        [[nodiscard]] const auto &get_sensitivity() const { return sensitivity_; }
        [[nodiscard]] const auto &get_sensitivity_collection() const { return sensitivity_collection_; }
        [[nodiscard]] auto &get_mask() { return mask_params_; }

        void set_bgs_type(BGSType type) { bgs_type_ = type; }
        void set_sensitivity(const std::string &sensitivity) { sensitivity_ = sensitivity; }
        void set_sensitivity_collection(const SensitivityConfigCollection &collection) { sensitivity_collection_ = collection; }
        void set_sensitivity_collection(const std::string &json_collection) { sensitivity_collection_.set_configs(json_collection); }

    private:
        BGSType bgs_type_{BGSType::Unknown};
        std::string sensitivity_;
        SensitivityConfigCollection sensitivity_collection_;
        MaskParams mask_params_;
    };

private:
    bool use_cuda_{true};
    int resize_height_{0};

    CameraParams camera_params_;
    TopicsParams topics_params_;
    RecordingParams recording_params_;
    BgsParams bgs_params_;
};
