#include <chrono>
#include <string>
#include <memory>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <bob_interfaces/srv/mask_override_request.hpp>
#include <bob_interfaces/msg/recording_event.hpp>
#include <parameter_node.hpp>
#include <image_utils.hpp>
#include <visibility_control.h>
#include <boblib/api/utils/pubsub/TopicManager.hpp>

#include "camera_bgs_params.hpp"
#include "camera_worker.hpp"
#include "camera_save_worker.hpp"
#include "background_subtractor_worker.hpp"
#include "track_provider_worker.hpp"
#include "record_manager_worker.hpp"

class CameraBGS : public ParameterNode
{
public:
    COMPOSITION_PUBLIC
    explicit CameraBGS(const rclcpp::NodeOptions & options)
        : ParameterNode("camera_bgs_node", options)
        , qos_profile_(4)
    {
        qos_profile_.reliability(rclcpp::ReliabilityPolicy::Reliable);
        qos_profile_.durability(rclcpp::DurabilityPolicy::Volatile);
        qos_profile_.history(rclcpp::HistoryPolicy::KeepLast);

        topic_manager_ = std::make_unique<boblib::utils::pubsub::TopicManager>(100);

        bgs_worker_ptr_ = std::make_unique<BackgroundSubtractorWorker>(*this, camera_bgs_params_, qos_profile_, *topic_manager_);
        camera_save_worker_ptr_ = std::make_unique<CameraSaveWorker>(*this, camera_bgs_params_, qos_profile_, *topic_manager_);
        track_provider_worker_ptr_ = std::make_unique<TrackProviderWorker>(*this, camera_bgs_params_, qos_profile_, *topic_manager_);
        record_manager_worker_ptr_ = std::make_unique<RecordManagerWorker>(*this, camera_bgs_params_, qos_profile_, *topic_manager_);
        camera_worker_ptr_ = std::make_unique<CameraWorker>(*this, camera_bgs_params_, qos_profile_, *topic_manager_);
    }

    // Ensure child workers are destroyed when node is shut down
    ~CameraBGS() override
    {
        log_info("CameraBGS destructor");
    }

    void on_configure()
    {
        log_info("Configuring");

        init();
    }

private:
    void init()
    {
        log_info("CameraBGS init");
        declare_node_parameters();

        bgs_worker_ptr_->init();
        camera_save_worker_ptr_->init();
        track_provider_worker_ptr_->init();
        record_manager_worker_ptr_->init();
        camera_worker_ptr_->init();
    }

    void reopen_camera()
    {
        // Check if the camera is already open else does nothing
        if ((camera_bgs_params_.camera.source_type != CameraBgsParams::SourceType::UNKNOWN)
            && camera_worker_ptr_->is_open())
        {
            camera_worker_ptr_->open_camera();
        }
    }

    void declare_node_parameters()
    {
        std::vector<ParameterNode::ActionParam> params =
            {
                ParameterNode::ActionParam(
                    rclcpp::Parameter("image_publish_topic", "bob/frames/allsky/original"),
                    [this](const rclcpp::Parameter &param)
                    {
                        camera_bgs_params_.topics.image_publish_topic = param.as_string();
                        camera_bgs_params_.topics.image_resized_publish_topic = param.as_string() + "/resized";
                    }),
                ParameterNode::ActionParam(
                    rclcpp::Parameter("image_info_publish_topic", "bob/camera/all_sky/image_info"),
                    [this](const rclcpp::Parameter &param)
                    {
                        camera_bgs_params_.topics.image_info_publish_topic = param.as_string();
                    }),
                ParameterNode::ActionParam(
                    rclcpp::Parameter("camera_info_publish_topic", "bob/camera/all_sky/camera_info"),
                    [this](const rclcpp::Parameter &param)
                    {
                        camera_bgs_params_.topics.camera_info_publish_topic = param.as_string();
                    }),
                ParameterNode::ActionParam(
                    rclcpp::Parameter("camera_recording_event_publisher_topic", "bob/camera1/recording/event"),
                    [this](const rclcpp::Parameter &param)
                    {
                        camera_bgs_params_.topics.recording_event_publisher_topic = param.as_string();
                    }),
                ParameterNode::ActionParam(
                    rclcpp::Parameter("bgs_publish_topic", "bob/frames/foreground_mask"),
                    [this](const rclcpp::Parameter &param)
                    {
                        camera_bgs_params_.topics.bgs_image_publish_topic = param.as_string();
                    }),
                ParameterNode::ActionParam(
                    rclcpp::Parameter("bgs_detection_topic", "bob/detection/allsky/boundingboxes"),
                    [this](const rclcpp::Parameter &param)
                    {
                        camera_bgs_params_.topics.detection_publish_topic = param.as_string();
                    }),
                ParameterNode::ActionParam(
                    rclcpp::Parameter("bgs_detection_state_topic", "bob/detection/detector_state"),
                    [this](const rclcpp::Parameter &param)
                    {
                        camera_bgs_params_.topics.detection_state_publish_topic = param.as_string();
                    }),
                ParameterNode::ActionParam(
                    rclcpp::Parameter("privacy_mask_override_service_topic", "bob/mask/privacy/override"),
                    [this](const rclcpp::Parameter &param)
                    {
                        privacy_mask_override_service_ = create_service<bob_interfaces::srv::MaskOverrideRequest>(param.as_string(),
                                                                                                                  [this](const std::shared_ptr<bob_interfaces::srv::MaskOverrideRequest::Request> request,
                                                                                                                         std::shared_ptr<bob_interfaces::srv::MaskOverrideRequest::Response> response)
                                                                                                                  { privacy_mask_override_request(request, response); });
                    }),
                ParameterNode::ActionParam(
                    rclcpp::Parameter("bgs_mask_override_service_topic", "bob/mask/detection/override"),
                    [this](const rclcpp::Parameter &param)
                    {
                        bgs_mask_override_service_ = create_service<bob_interfaces::srv::MaskOverrideRequest>(param.as_string(),
                                                                                                              [this](const std::shared_ptr<bob_interfaces::srv::MaskOverrideRequest::Request> request,
                                                                                                                     std::shared_ptr<bob_interfaces::srv::MaskOverrideRequest::Response> response)
                                                                                                              { bgs_mask_override_request(request, response); });
                    }),
                ParameterNode::ActionParam(
                    rclcpp::Parameter("camera_settings_client_topic", "bob/camera/settings"),
                    [this](const rclcpp::Parameter &param)
                    {
                        camera_bgs_params_.topics.camera_settings_client_topic = param.as_string();
                    }),
                ParameterNode::ActionParam(
                    rclcpp::Parameter("tracker_publisher_topic", "bob/tracker/tracking"),
                    [this](const rclcpp::Parameter &param)
                    {
                        camera_bgs_params_.topics.tracking_publisher_topic = param.as_string();
                    }),
                ParameterNode::ActionParam(
                    rclcpp::Parameter("use_cuda", true),
                    [this](const rclcpp::Parameter &param)
                    {
                        camera_bgs_params_.use_cuda = param.as_bool();
                    }),
                ParameterNode::ActionParam(
                    rclcpp::Parameter("mask_timer_seconds", 5),
                    [this](const rclcpp::Parameter &param)
                    {
                        auto mask_timer = static_cast<int>(param.as_int());
                        camera_bgs_params_.camera.privacy_mask.timer_seconds = mask_timer;
                        camera_worker_ptr_->restart_mask();
                        camera_bgs_params_.bgs.mask.timer_seconds = mask_timer;
                        bgs_worker_ptr_->restart_mask();
                    }),
                ParameterNode::ActionParam(
                    rclcpp::Parameter("usb_camera_resolution", std::vector<long>()),
                    [this](const rclcpp::Parameter &param)
                    {
                        camera_bgs_params_.camera.usb_resolution = param.as_integer_array();
                        if (camera_bgs_params_.camera.source_type == CameraBgsParams::SourceType::USB_CAMERA)
                        {
                            reopen_camera();
                        }
                    }),
                ParameterNode::ActionParam(
                    rclcpp::Parameter("source_type", "USB_CAMERA"),
                    [this](const rclcpp::Parameter &param)
                    {
                        using enum CameraBgsParams::SourceType;
                        const auto &type = param.as_string();
                        if (type == "USB_CAMERA")
                        {
                            camera_bgs_params_.camera.source_type = USB_CAMERA;
                        }
                        else if (type == "VIDEO_FILE")
                        {
                            camera_bgs_params_.camera.source_type = VIDEO_FILE;
                        }
                        else if (type == "RTSP_STREAM")
                        {
                            camera_bgs_params_.camera.source_type = RTSP_STREAM;
                        }
                        else
                        {
                            camera_bgs_params_.camera.source_type = UNKNOWN;
                            log_error("Invalid source type: %s", type.c_str());
                        }

                        reopen_camera();
                    }),
                ParameterNode::ActionParam(
                    rclcpp::Parameter("camera_id", 0),
                    [this](const rclcpp::Parameter &param)
                    {
                        camera_bgs_params_.camera.camera_id = static_cast<int>(param.as_int());
                        if (camera_bgs_params_.camera.source_type == CameraBgsParams::SourceType::USB_CAMERA)
                        {
                            reopen_camera();
                        }
                    }),
                ParameterNode::ActionParam(
                    rclcpp::Parameter("videos", std::vector<std::string>()),
                    [this](const rclcpp::Parameter &param)
                    {
                        camera_bgs_params_.camera.videos = param.as_string_array();
                        if (camera_bgs_params_.camera.source_type == CameraBgsParams::SourceType::VIDEO_FILE)
                        {
                            reopen_camera();
                        }
                    }),
                ParameterNode::ActionParam(
                    rclcpp::Parameter("rtsp_uri", ""),
                    [this](const rclcpp::Parameter &param)
                    {
                        camera_bgs_params_.camera.rtsp_uri = param.as_string();
                        if (camera_bgs_params_.camera.source_type == CameraBgsParams::SourceType::RTSP_STREAM)
                        {
                            reopen_camera();
                        }
                    }),
                ParameterNode::ActionParam(
                    rclcpp::Parameter("onvif_uri", "192.168.1.20"),
                    [this](const rclcpp::Parameter &param)
                    {
                        camera_bgs_params_.camera.onvif_uri = param.as_string();
                    }),
                ParameterNode::ActionParam(
                    rclcpp::Parameter("privacy_mask_enable_override", true),
                    [this](const rclcpp::Parameter &param)
                    { 
                        camera_bgs_params_.camera.privacy_mask.enable_override = param.as_bool();
                    }),
                ParameterNode::ActionParam(
                    rclcpp::Parameter("privacy_mask_file", "mask.pgm"),
                    [this](const rclcpp::Parameter &param)
                    {
                        camera_bgs_params_.camera.privacy_mask.filename = param.as_string();
                        camera_worker_ptr_->restart_mask();
                    }),
                ParameterNode::ActionParam(
                    rclcpp::Parameter("bgs_json_params", ""),
                    [this](const rclcpp::Parameter &param)
                    {
                        camera_bgs_params_.bgs.set_sensitivity_collection(param.as_string());
                    }),
                ParameterNode::ActionParam(
                    rclcpp::Parameter("bgs_sensitivity", "medium_c"),
                    [this](const rclcpp::Parameter &param)
                    {
                        log_send_info("Setting Sensitivity: %s", param.as_string().c_str());
                        bgs_worker_ptr_->init_sensitivity(param.as_string());
                    }),
                ParameterNode::ActionParam(
                    rclcpp::Parameter("bgs_type", "vibe"),
                    [this](const rclcpp::Parameter &param)
                    {
                        log_send_info("Setting BGS: %s", param.as_string().c_str());
                        bgs_worker_ptr_->init_bgs(param.as_string());
                    }),
                ParameterNode::ActionParam(
                    rclcpp::Parameter("bgs_mask_enable_override", true),
                    [this](const rclcpp::Parameter &param)
                    {
                        camera_bgs_params_.bgs.mask.enable_override = param.as_bool();
                    }),
                ParameterNode::ActionParam(
                    rclcpp::Parameter("bgs_mask_file", "mask.pgm"),
                    [this](const rclcpp::Parameter &param)
                    {
                        camera_bgs_params_.bgs.mask.filename = param.as_string();
                        bgs_worker_ptr_->restart_mask();
                    }),
                ParameterNode::ActionParam(
                    rclcpp::Parameter("simulator_num_objects", 15),
                    [this](const rclcpp::Parameter &param)
                    {
                        camera_bgs_params_.camera.simulator.num_objects = static_cast<int>(param.as_int());
                    }),
                ParameterNode::ActionParam(
                    rclcpp::Parameter("simulator", false),
                    [this](const rclcpp::Parameter &param)
                    {
                        camera_bgs_params_.camera.simulator.enable = param.as_bool();
                    }),
                ParameterNode::ActionParam(
                    rclcpp::Parameter("resize_height", 960),
                    [this](const rclcpp::Parameter &param)
                    {
                        camera_bgs_params_.resize_height = static_cast<int>(param.as_int());
                    }),
                ParameterNode::ActionParam(
                    rclcpp::Parameter("limit_fps", true),
                    [this](const rclcpp::Parameter &param)
                    {
                        camera_bgs_params_.camera.limit_fps = param.as_bool();
                    }),
                ParameterNode::ActionParam(
                    rclcpp::Parameter("recording_enabled", false),
                    [this](const rclcpp::Parameter &param)
                    {
                        camera_bgs_params_.recording.enabled = param.as_bool();
                    }),
                ParameterNode::ActionParam(
                    rclcpp::Parameter("recording_codec", "avc1"),
                    [this](const rclcpp::Parameter &param)
                    {
                        camera_bgs_params_.recording.codec = param.as_string();
                    }),
                ParameterNode::ActionParam(
                    rclcpp::Parameter("recording_seconds_save", 2),
                    [this](const rclcpp::Parameter &param)
                    {
                        camera_bgs_params_.recording.seconds_save = param.as_int();
                    }),
                ParameterNode::ActionParam(
                    rclcpp::Parameter("recordings_directory", "."),
                    [this](const rclcpp::Parameter &param)
                    {
                        camera_bgs_params_.recording.recordings_directory = param.as_string();
                    }),
                ParameterNode::ActionParam(
                    rclcpp::Parameter("recording_request_service_topic", "bob/recording/update"),
                    [this](const rclcpp::Parameter &param)
                    {
                        camera_bgs_params_.topics.recording_request_service_topic = param.as_string();
                    }),
                ParameterNode::ActionParam(
                    rclcpp::Parameter("recording_state_publisher_topic", "bob/recording/state"),
                    [this](const rclcpp::Parameter &param)
                    {
                        camera_bgs_params_.topics.recording_state_publisher_topic = param.as_string();
                    }),
                ParameterNode::ActionParam(
                    rclcpp::Parameter("recording_prefix", ""),
                    [this](const rclcpp::Parameter &param)
                    {
                        camera_bgs_params_.recording.prefix = param.as_string();
                    }),
                ParameterNode::ActionParam(
                    rclcpp::Parameter("profiling", false),
                    [this](const rclcpp::Parameter &param)
                    {
                        camera_bgs_params_.profiling = param.as_bool();
                    }),
            };
        add_action_parameters(params);
    }

    void privacy_mask_override_request(const std::shared_ptr<bob_interfaces::srv::MaskOverrideRequest::Request> request, 
                                       std::shared_ptr<bob_interfaces::srv::MaskOverrideRequest::Response> response)
    {
        camera_bgs_params_.camera.privacy_mask.enable_override = request->mask_enabled;
        if (request->mask_enabled)
        {
            log_send_debug("Privacy mask Override set to: True");
        }
        else
        {
            log_send_debug("Privacy mask Override set to: False");
        }
        response->success = true;        
    }

    void bgs_mask_override_request(const std::shared_ptr<bob_interfaces::srv::MaskOverrideRequest::Request> request, 
                                   std::shared_ptr<bob_interfaces::srv::MaskOverrideRequest::Response> response)
    {
        camera_bgs_params_.bgs.mask.enable_override = request->mask_enabled;
        if (request->mask_enabled)
        {
            log_send_debug("BGS Mask Override set to: True");
        }
        else
        {
            log_send_debug("BGS mask Override set to: False");
        }
        response->success = true;        
    }

    rclcpp::QoS qos_profile_; 

    std::unique_ptr<CameraWorker> camera_worker_ptr_;
    std::unique_ptr<BackgroundSubtractorWorker> bgs_worker_ptr_;
    std::unique_ptr<CameraSaveWorker> camera_save_worker_ptr_;
    std::unique_ptr<TrackProviderWorker> track_provider_worker_ptr_;
    std::unique_ptr<RecordManagerWorker> record_manager_worker_ptr_;

    std::unique_ptr<boblib::utils::pubsub::TopicManager> topic_manager_;

    rclcpp::Service<bob_interfaces::srv::MaskOverrideRequest>::SharedPtr privacy_mask_override_service_;
    rclcpp::Service<bob_interfaces::srv::MaskOverrideRequest>::SharedPtr bgs_mask_override_service_;

    CameraBgsParams camera_bgs_params_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(CameraBGS)
