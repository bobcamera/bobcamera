#pragma once

#include <stddef.h>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <bob_camera/msg/image_info.hpp>
#include <bob_camera/msg/camera_info.hpp>
#include <bob_interfaces/srv/camera_settings.hpp>
#include <bob_interfaces/msg/recording_event.hpp>

#include "background_subtractor_companion.hpp"

struct CameraBgsParams
{
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

    struct MaskParams
    {
        bool enable_override{false};
        std::string filename;
        int timer_seconds{5};
    };

    struct SimulatorParams
    {
        bool enable{false};
        int num_objects{15};
    };

    struct CameraParams
    {
        SourceType source_type{SourceType::UNKNOWN};
        int camera_id;
        std::string rtsp_uri;
        std::string onvif_uri;
        std::vector<long> usb_resolution;
        std::vector<std::string> videos;
        bool limit_fps{true};

        bool speed_test{false};
        size_t test_frames{0};
        int speed_test_fps{60};

        MaskParams privacy_mask;
        SimulatorParams simulator;
    };

    struct TopicsParams
    {
        std::string image_publish_topic;
        std::string image_info_publish_topic;
        std::string camera_info_publish_topic;
        std::string image_resized_publish_topic;
        std::string camera_settings_client_topic;
        std::string tracking_publisher_topic;
        std::string tracker_state_publisher_topic;
        std::string detection_publish_topic;
        std::string detection_state_publish_topic;
        std::string bgs_image_publish_topic;
        std::string recording_event_publisher_topic;
        std::string recording_request_service_topic;
        std::string recording_state_publisher_topic;
        std::string annotated_frame_publisher_topic;
    };

    struct RecordingParams
    {
        bool enabled{false};
        std::string codec{"avc1"};
        int seconds_save{2};
        std::string recordings_directory{""};
        std::string prefix{""};
    };

    struct AnnotatedFrameParams
    {
        bool enable_tracking_status_message{false};
        std::unordered_map<std::string, std::string> visualiser_settings;
    };

    struct BgsParams
    {
        void set_sensitivity_collection(const std::string &json_collection) { sensitivity_collection.set_configs(json_collection); }

        BGSType type{BGSType::Unknown};
        std::string sensitivity;
        SensitivityConfigCollection sensitivity_collection;
        MaskParams mask;
    };

    bool use_cuda{true};
    int resize_height{0};
    int compression_quality{75};
    bool profiling{false};

    CameraParams camera;
    TopicsParams topics;
    RecordingParams recording;
    BgsParams bgs;
    AnnotatedFrameParams annotated_frame;
};
