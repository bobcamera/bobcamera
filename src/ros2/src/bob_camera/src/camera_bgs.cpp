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
#include "camera_worker.hpp"
#include "background_subtractor_worker.hpp"

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

        camera_params_ptr_ = std::make_unique<CameraWorkerParams>();
        bgs_params_ptr_ = std::make_unique<BackgroundSubtractorWorkerParams>();

        bgs_worker_ptr_ = std::make_unique<BackgroundSubtractorWorker>(*this, *bgs_params_ptr_);
        camera_worker_ptr_ = std::make_unique<CameraWorker>(*this, *camera_params_ptr_,
                                                            [this](float fps, const std_msgs::msg::Header &header, const boblib::base::Image &img)
                                                            {
                                                                bgs_worker_ptr_->image_callback(fps, header, img);
                                                            });
    }

    ~CameraBGS() = default;

    void on_configure()
    {
        log_info("Configuring");

        init();
    }

private:
    void init()
    {
        declare_node_parameters();

        bgs_worker_ptr_->init();
        camera_worker_ptr_->init();
    }

    void reopen_camera()
    {
        // Validate individual types
        if ((camera_params_ptr_->get_source_type() != CameraWorkerParams::SourceType::UNKNOWN)
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
                        camera_params_ptr_->set_image_publish_topic(param.as_string());
                        camera_params_ptr_->set_image_resized_publish_topic(camera_params_ptr_->get_image_publish_topic() + "/resized");
                        camera_params_ptr_->set_image_publisher(create_publisher<sensor_msgs::msg::Image>(camera_params_ptr_->get_image_publish_topic(), qos_profile_));
                    }),
                ParameterNode::ActionParam(
                    rclcpp::Parameter("image_info_publish_topic", "bob/camera/all_sky/image_info"),
                    [this](const rclcpp::Parameter &param)
                    {
                        camera_params_ptr_->set_image_info_publish_topic(param.as_string());
                        camera_params_ptr_->set_image_info_publisher(create_publisher<bob_camera::msg::ImageInfo>(camera_params_ptr_->get_image_info_publish_topic(), qos_profile_));
                    }),
                ParameterNode::ActionParam(
                    rclcpp::Parameter("camera_info_publish_topic", "bob/camera/all_sky/camera_info"),
                    [this](const rclcpp::Parameter &param)
                    {
                        camera_params_ptr_->set_camera_info_publish_topic(param.as_string());
                        camera_params_ptr_->set_camera_info_publisher(create_publisher<bob_camera::msg::CameraInfo>(camera_params_ptr_->get_camera_info_publish_topic(), qos_profile_));
                        bgs_params_ptr_->set_camera_info_subscriber_topic(param.as_string());
                    }),
                ParameterNode::ActionParam(
                    rclcpp::Parameter("camera_recording_event_subscriber_topic", "bob/camera1/recording/event"),
                    [this](const rclcpp::Parameter &param)
                    {
                        camera_params_ptr_->set_recording_event_subscriber_topic(param.as_string());
                        camera_params_ptr_->set_recording_event_subscriber(create_subscription<bob_interfaces::msg::RecordingEvent>(param.as_string(), qos_profile_,
                                                                                                                                    [this](const bob_interfaces::msg::RecordingEvent::SharedPtr event)
                                                                                                                                    { recording_event_callback(event); }));
                    }),
                ParameterNode::ActionParam(
                    rclcpp::Parameter("bgs_publish_topic", "bob/frames/foreground_mask"),
                    [this](const rclcpp::Parameter &param)
                    {
                        bgs_params_ptr_->set_image_publish_topic(param.as_string());
                        bgs_params_ptr_->set_image_resized_publish_topic(bgs_params_ptr_->get_image_publish_topic() + "/resized");
                        bgs_params_ptr_->set_image_publisher(create_publisher<sensor_msgs::msg::Image>(bgs_params_ptr_->get_image_publish_topic(), qos_profile_));
                    }),
                ParameterNode::ActionParam(
                    rclcpp::Parameter("bgs_detection_topic", "bob/detection/allsky/boundingboxes"),
                    [this](const rclcpp::Parameter &param)
                    {
                        bgs_params_ptr_->set_detection_publish_topic(param.as_string());
                        bgs_params_ptr_->set_detection_publisher(create_publisher<bob_interfaces::msg::DetectorBBoxArray>(bgs_params_ptr_->get_detection_publish_topic(), qos_profile_));
                    }),
                ParameterNode::ActionParam(
                    rclcpp::Parameter("bgs_detection_state_topic", "bob/detection/detector_state"),
                    [this](const rclcpp::Parameter &param)
                    {
                        bgs_params_ptr_->set_detection_state_publish_topic(param.as_string());
                        bgs_params_ptr_->set_state_publisher(create_publisher<bob_interfaces::msg::DetectorState>(bgs_params_ptr_->get_detection_state_publish_topic(), qos_profile_));
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
                        camera_params_ptr_->set_camera_settings_client(create_client<bob_interfaces::srv::CameraSettings>(param.as_string()));
                    }),
                ParameterNode::ActionParam(
                    rclcpp::Parameter("tracking_subscriber_topic", "bob/tracker/tracking"),
                    [this](const rclcpp::Parameter &param)
                    {
                        bgs_params_ptr_->set_tracking_subscriber_topic(param.as_string());
                    }),
                ParameterNode::ActionParam(
                    rclcpp::Parameter("use_cuda", true),
                    [this](const rclcpp::Parameter &param)
                    {
                        camera_params_ptr_->set_use_cuda(param.as_bool());
                        bgs_params_ptr_->set_use_cuda(param.as_bool());
                    }),
                ParameterNode::ActionParam(
                    rclcpp::Parameter("mask_timer_seconds", 5),
                    [this](const rclcpp::Parameter &param)
                    {
                        camera_params_ptr_->set_mask_timer_seconds(static_cast<int>(param.as_int()));
                        camera_worker_ptr_->restart_mask();
                        bgs_params_ptr_->set_mask_timer_seconds(static_cast<int>(param.as_int()));
                        bgs_worker_ptr_->restart_mask();
                    }),
                ParameterNode::ActionParam(
                    rclcpp::Parameter("usb_camera_resolution", std::vector<long>()),
                    [this](const rclcpp::Parameter &param)
                    {
                        camera_params_ptr_->set_usb_resolution(param.as_integer_array());
                        if (camera_params_ptr_->get_source_type() == CameraWorkerParams::SourceType::USB_CAMERA)
                        {
                            reopen_camera();
                        }
                    }),
                ParameterNode::ActionParam(
                    rclcpp::Parameter("source_type", "USB_CAMERA"),
                    [this](const rclcpp::Parameter &param)
                    {
                        using enum CameraWorkerParams::SourceType;
                        const auto &type = param.as_string();
                        if (type == "USB_CAMERA")
                        {
                            camera_params_ptr_->set_source_type(USB_CAMERA);
                        }
                        else if (type == "VIDEO_FILE")
                        {
                            camera_params_ptr_->set_source_type(VIDEO_FILE);
                        }
                        else if (type == "RTSP_STREAM")
                        {
                            camera_params_ptr_->set_source_type(RTSP_STREAM);
                        }
                        else
                        {
                            camera_params_ptr_->set_source_type(UNKNOWN);
                            log_error("Invalid source type: %s", type.c_str());
                        }

                        reopen_camera();
                    }),
                ParameterNode::ActionParam(
                    rclcpp::Parameter("camera_id", 0),
                    [this](const rclcpp::Parameter &param)
                    {
                        camera_params_ptr_->set_camera_id(static_cast<int>(param.as_int()));
                        if (camera_params_ptr_->get_source_type() == CameraWorkerParams::SourceType::USB_CAMERA)
                        {
                            reopen_camera();
                        }
                    }),
                ParameterNode::ActionParam(
                    rclcpp::Parameter("videos", std::vector<std::string>()),
                    [this](const rclcpp::Parameter &param)
                    {
                        camera_params_ptr_->set_videos(param.as_string_array());
                        if (camera_params_ptr_->get_source_type() == CameraWorkerParams::SourceType::VIDEO_FILE)
                        {
                            reopen_camera();
                        }
                    }),
                ParameterNode::ActionParam(
                    rclcpp::Parameter("rtsp_uri", ""),
                    [this](const rclcpp::Parameter &param)
                    {
                        camera_params_ptr_->set_rtsp_uri(param.as_string());
                        if (camera_params_ptr_->get_source_type() == CameraWorkerParams::SourceType::RTSP_STREAM)
                        {
                            reopen_camera();
                        }
                    }),
                ParameterNode::ActionParam(
                    rclcpp::Parameter("onvif_host", "192.168.1.20"),
                    [this](const rclcpp::Parameter &param)
                    {
                        camera_params_ptr_->set_onvif_host(param.as_string());
                        if (camera_params_ptr_->get_source_type() == CameraWorkerParams::SourceType::RTSP_STREAM)
                        {
                            reopen_camera();
                        }
                    }),
                ParameterNode::ActionParam(
                    rclcpp::Parameter("onvif_port", 80),
                    [this](const rclcpp::Parameter &param)
                    {
                        camera_params_ptr_->set_onvif_port(static_cast<int>(param.as_int()));
                        if (camera_params_ptr_->get_source_type() == CameraWorkerParams::SourceType::RTSP_STREAM)
                        {
                            reopen_camera();
                        }
                    }),
                ParameterNode::ActionParam(
                    rclcpp::Parameter("onvif_user", "default_user"),
                    [this](const rclcpp::Parameter &param)
                    {
                        camera_params_ptr_->set_onvif_user(param.as_string());
                        if (camera_params_ptr_->get_source_type() == CameraWorkerParams::SourceType::RTSP_STREAM)
                        {
                            reopen_camera();
                        }
                    }),
                ParameterNode::ActionParam(
                    rclcpp::Parameter("onvif_password", "default_password"),
                    [this](const rclcpp::Parameter &param)
                    {
                        camera_params_ptr_->set_onvif_password(param.as_string());
                        if (camera_params_ptr_->get_source_type() == CameraWorkerParams::SourceType::RTSP_STREAM)
                        {
                            reopen_camera();
                        }
                    }),
                ParameterNode::ActionParam(
                    rclcpp::Parameter("privacy_mask_enable_override", true),
                    [this](const rclcpp::Parameter &param)
                    { camera_params_ptr_->set_mask_enable_override(param.as_bool()); }),
                ParameterNode::ActionParam(
                    rclcpp::Parameter("privacy_mask_file", "mask.pgm"),
                    [this](const rclcpp::Parameter &param)
                    {
                        camera_params_ptr_->set_mask_filename(param.as_string());
                        camera_worker_ptr_->restart_mask();
                    }),
                ParameterNode::ActionParam(
                    rclcpp::Parameter("bgs_json_params", ""),
                    [this](const rclcpp::Parameter &param)
                    {
                        bgs_params_ptr_->set_sensitivity_collection(param.as_string());
                    }),
                ParameterNode::ActionParam(
                    rclcpp::Parameter("bgs_sensitivity", "medium_c"),
                    [this](const rclcpp::Parameter &param)
                    {
                        log_info("Setting Sensitivity: %s", param.as_string().c_str());
                        bgs_worker_ptr_->init_sensitivity(param.as_string());
                    }),
                ParameterNode::ActionParam(
                    rclcpp::Parameter("bgs_type", "vibe"),
                    [this](const rclcpp::Parameter &param)
                    {
                        log_info("Setting BGS: %s", param.as_string().c_str());
                        bgs_worker_ptr_->init_bgs(param.as_string());
                    }),
                ParameterNode::ActionParam(
                    rclcpp::Parameter("bgs_mask_enable_override", true),
                    [this](const rclcpp::Parameter &param)
                    { bgs_params_ptr_->set_mask_enable_override(param.as_bool()); }),
                ParameterNode::ActionParam(
                    rclcpp::Parameter("bgs_mask_file", "mask.pgm"),
                    [this](const rclcpp::Parameter &param)
                    {
                        bgs_params_ptr_->set_mask_filename(param.as_string());
                        bgs_worker_ptr_->restart_mask();
                    }),
                ParameterNode::ActionParam(
                    rclcpp::Parameter("simulator_num_objects", 15),
                    [this](const rclcpp::Parameter &param)
                    {
                        camera_params_ptr_->set_simulator_num_objects(static_cast<int>(param.as_int()));
                    }),
                ParameterNode::ActionParam(
                    rclcpp::Parameter("simulator", false),
                    [this](const rclcpp::Parameter &param)
                    {
                        camera_params_ptr_->set_simulator_enable(param.as_bool());
                    }),
                ParameterNode::ActionParam(
                    rclcpp::Parameter("resize_height", 960),
                    [this](const rclcpp::Parameter &param)
                    {
                        camera_params_ptr_->set_resize_height(static_cast<int>(param.as_int()));
                        bgs_params_ptr_->set_resize_height(static_cast<int>(param.as_int()));
                        if (camera_params_ptr_->get_resize_height() > 0)
                        {
                            camera_params_ptr_->set_image_resized_publisher(create_publisher<sensor_msgs::msg::Image>(camera_params_ptr_->get_image_resized_publish_topic(), qos_profile_));
                            bgs_params_ptr_->set_image_resized_publisher(create_publisher<sensor_msgs::msg::Image>(bgs_params_ptr_->get_image_resized_publish_topic(), qos_profile_));
                        }
                        else
                        {
                            camera_params_ptr_->set_image_resized_publisher(nullptr);
                            bgs_params_ptr_->set_image_resized_publisher(nullptr);
                            log_debug("Resizer topics disabled");
                        }
                    }),
                ParameterNode::ActionParam(
                    rclcpp::Parameter("limit_fps", true),
                    [this](const rclcpp::Parameter &param)
                    {
                        camera_params_ptr_->set_limit_fps(param.as_bool());
                    }),
                ParameterNode::ActionParam(
                    rclcpp::Parameter("max_queue_process_size", 0),
                    [this](const rclcpp::Parameter &param)
                    {
                        camera_params_ptr_->set_max_queue_process_size(static_cast<size_t>(param.as_int()));
                        bgs_params_ptr_->set_max_queue_process_size(static_cast<size_t>(param.as_int()));
                    }),
                ParameterNode::ActionParam(
                    rclcpp::Parameter("recording_enabled", false),
                    [this](const rclcpp::Parameter &param)
                    {
                        camera_params_ptr_->set_recording_enabled(param.as_bool());
                        bgs_params_ptr_->set_recording_enabled(param.as_bool());
                    }),
                ParameterNode::ActionParam(
                    rclcpp::Parameter("recording_codec", "avc1"),
                    [this](const rclcpp::Parameter &param)
                    {
                        camera_params_ptr_->set_recording_codec(param.as_string());
                    }),
                ParameterNode::ActionParam(
                    rclcpp::Parameter("recording_seconds_save", 2),
                    [this](const rclcpp::Parameter &param)
                    {
                        camera_params_ptr_->set_recording_seconds_save(param.as_int());
                        bgs_params_ptr_->set_recording_seconds_save(param.as_int());
                    }),
            };
        add_action_parameters(params);
    }

    void recording_event_callback(const bob_interfaces::msg::RecordingEvent::SharedPtr event)
    {
        camera_worker_ptr_->recording_event(*event);
        bgs_worker_ptr_->recording_event(*event);
    }

    void privacy_mask_override_request(const std::shared_ptr<bob_interfaces::srv::MaskOverrideRequest::Request> request, 
                                       std::shared_ptr<bob_interfaces::srv::MaskOverrideRequest::Response> response)
    {
        camera_params_ptr_->set_mask_enable_override(request->mask_enabled);
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
        bgs_params_ptr_->set_mask_enable_override(request->mask_enabled);
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

    std::unique_ptr<CameraWorkerParams> camera_params_ptr_;
    std::unique_ptr<CameraWorker> camera_worker_ptr_;
    std::unique_ptr<BackgroundSubtractorWorkerParams> bgs_params_ptr_;
    std::unique_ptr<BackgroundSubtractorWorker> bgs_worker_ptr_;

    rclcpp::Service<bob_interfaces::srv::MaskOverrideRequest>::SharedPtr privacy_mask_override_service_;
    rclcpp::Service<bob_interfaces::srv::MaskOverrideRequest>::SharedPtr bgs_mask_override_service_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(CameraBGS)
