#include <chrono>
#include <string>

#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <bob_interfaces/srv/mask_override_request.hpp>
#include <bob_interfaces/srv/bgs_reset_request.hpp>
#include <sensor_msgs/msg/region_of_interest.hpp>

#include <parameter_node.hpp>
#include <image_utils.hpp>
#include <visibility_control.h>

#include <camera_worker.hpp>
#include <background_subtractor_worker.hpp>

class CameraBGS
    : public ParameterNode
{
public:
    COMPOSITION_PUBLIC
    explicit CameraBGS(const rclcpp::NodeOptions & options)
        : ParameterNode("camera_bgs_node", options)
    {
        qos_profile_.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        qos_profile_.durability(rclcpp::DurabilityPolicy::Volatile);
        qos_profile_.history(rclcpp::HistoryPolicy::KeepLast);

        one_shot_timer_ = create_wall_timer(std::chrono::seconds(2), [this](){init();});
    }

private:
    CameraWorkerParams camera_params_;
    std::unique_ptr<CameraWorker> camera_worker_ptr_;
    BackgroundSubtractorWorkerParams bgs_params_;
    std::unique_ptr<BackgroundSubtractorWorker> bgs_worker_ptr_;

    rclcpp::QoS qos_profile_{10}; 
    rclcpp::Service<bob_interfaces::srv::MaskOverrideRequest>::SharedPtr privacy_mask_override_service_;
    rclcpp::Service<bob_interfaces::srv::MaskOverrideRequest>::SharedPtr bgs_mask_override_service_;
    rclcpp::Service<bob_interfaces::srv::BGSResetRequest>::SharedPtr bgs_reset_service_;
    rclcpp::TimerBase::SharedPtr one_shot_timer_;

    void init()
    {
        one_shot_timer_.reset();

        bgs_worker_ptr_ = std::make_unique<BackgroundSubtractorWorker>(*this, bgs_params_);
        camera_worker_ptr_ = std::make_unique<CameraWorker>(*this, camera_params_, [this](const std_msgs::msg::Header & header, const cv::Mat & img){bgs_callback(header, img);});

        privacy_mask_override_service_ = create_service<bob_interfaces::srv::MaskOverrideRequest>("bob/mask/privacy/override",
            [this](const std::shared_ptr<bob_interfaces::srv::MaskOverrideRequest::Request> request, 
                    std::shared_ptr<bob_interfaces::srv::MaskOverrideRequest::Response> response) {
                        privacy_mask_override_request(request, response);
                    });
        bgs_mask_override_service_ = create_service<bob_interfaces::srv::MaskOverrideRequest>("bob/mask/detection/override",
            [this](const std::shared_ptr<bob_interfaces::srv::MaskOverrideRequest::Request> request, 
                    std::shared_ptr<bob_interfaces::srv::MaskOverrideRequest::Response> response) {
                        bgs_mask_override_request(request, response);
                    });
        bgs_reset_service_ = create_service<bob_interfaces::srv::BGSResetRequest>("bob/bgs/reset", 
            [this](const std::shared_ptr<bob_interfaces::srv::BGSResetRequest::Request> request, 
                    std::shared_ptr<bob_interfaces::srv::BGSResetRequest::Response> response) {reset_bgs_request(request, response);});

        camera_params_.roi_publisher = create_publisher<sensor_msgs::msg::RegionOfInterest>("bob/mask/roi", qos_profile_);
        bgs_params_.image_publisher = create_publisher<sensor_msgs::msg::Image>("bob/frames/foreground_mask", qos_profile_);
        bgs_params_.detection_publisher = create_publisher<bob_interfaces::msg::DetectorBBoxArray>("bob/detection/allsky/boundingboxes", qos_profile_);        
        bgs_params_.state_publisher = create_publisher<bob_interfaces::msg::DetectorState>("bob/detection/detector_state", qos_profile_);

        declare_node_parameters();
        bgs_worker_ptr_->init();
        camera_worker_ptr_->init();
    }

    void declare_node_parameters()
    {
        std::vector<ParameterNode::ActionParam> params = 
        {
            ParameterNode::ActionParam(
                rclcpp::Parameter("image_publish_topic", "bob/frames/allsky/original"), 
                [this](const rclcpp::Parameter& param) 
                {
                    camera_params_.image_publish_topic = param.as_string(); 
                    camera_params_.image_publisher = create_publisher<sensor_msgs::msg::Image>(camera_params_.image_publish_topic, qos_profile_);
                }
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("image_info_publish_topic", "bob/camera/all_sky/image_info"), 
                [this](const rclcpp::Parameter& param) 
                {
                    camera_params_.image_info_publish_topic = param.as_string(); 
                    camera_params_.image_info_publisher = create_publisher<bob_camera::msg::ImageInfo>(camera_params_.image_info_publish_topic, qos_profile_);
                }
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("camera_info_publish_topic", "bob/camera/all_sky/camera_info"), 
                [this](const rclcpp::Parameter& param) 
                {
                    camera_params_.camera_info_publish_topic = param.as_string(); 
                    camera_params_.camera_info_publisher = create_publisher<bob_camera::msg::CameraInfo>(camera_params_.camera_info_publish_topic, qos_profile_);
                }
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("image_resized_publish_topic", "bob/frames/allsky/original/resized"), 
                [this](const rclcpp::Parameter& param) 
                {
                    camera_params_.image_resized_publish_topic = param.as_string();
                    if (!camera_params_.image_resized_publish_topic.empty())
                    {
                        camera_params_.image_resized_publisher = create_publisher<sensor_msgs::msg::Image>(camera_params_.image_resized_publish_topic, qos_profile_);
                    }
                    else
                    {
                        camera_params_.image_resized_publisher.reset();
                        RCLCPP_INFO(get_logger(), "Resizer topic disabled");
                    }
                }
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("source_type", "USB_CAMERA"), 
                [this](const rclcpp::Parameter& param) {
                    using enum CameraWorkerParams::SourceType;
                    std::string type = param.as_string();
                    if (type == "USB_CAMERA") 
                    {
                        camera_params_.source_type = USB_CAMERA;
                    } 
                    else if (type == "VIDEO_FILE") 
                    {
                        camera_params_.source_type = VIDEO_FILE;
                    } 
                    else if (type == "RTSP_STREAM") 
                    {
                        camera_params_.source_type = RTSP_STREAM;
                    } 
                    else
                    {
                        RCLCPP_ERROR(get_logger(), "Invalid source type: %s", type.c_str());
                    }
                }
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("camera_id", 0), 
                [this](const rclcpp::Parameter& param) {camera_params_.camera_id = static_cast<int>(param.as_int());}
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("videos", std::vector<std::string>({""})), 
                [this](const rclcpp::Parameter& param) {camera_params_.videos = param.as_string_array();}
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("rtsp_uri", ""), 
                [this](const rclcpp::Parameter& param) {camera_params_.rtsp_uri = param.as_string();}
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("onvif_host", "192.168.1.20"),
                [this](const rclcpp::Parameter& param) {camera_params_.onvif_host = param.as_string();}
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("onvif_port", 80),
                [this](const rclcpp::Parameter& param) {camera_params_.onvif_port = static_cast<int>(param.as_int());}
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("onvif_user", "default_user"),
                [this](const rclcpp::Parameter& param) {camera_params_.onvif_user = param.as_string();}
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("onvif_password", "default_password"),
                [this](const rclcpp::Parameter& param) {camera_params_.onvif_password = param.as_string();}
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("resize_height", 960), 
                [this](const rclcpp::Parameter& param) {
                    camera_params_.resize_height = static_cast<int>(param.as_int());
                    bgs_params_.resize_height = static_cast<int>(param.as_int());
                }
            ),
            // MASK parameters
            ParameterNode::ActionParam(
                rclcpp::Parameter("privacy_mask_enable_override", true), 
                [this](const rclcpp::Parameter& param) {camera_params_.mask_enable_override = param.as_bool();}
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("privacy_mask_file", "mask.pgm"), 
                [this](const rclcpp::Parameter& param) {camera_params_.mask_filename = param.as_string();}
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("privacy_mask_enable_offset_correction", false), 
                [this](const rclcpp::Parameter& param) {camera_params_.mask_enable_roi = param.as_bool();}
            ),

            //////// BGS PARAMS
            ParameterNode::ActionParam(
                rclcpp::Parameter("bgs_json_params", ""), 
                [this](const rclcpp::Parameter& param) 
                {
                    try 
                    {
                        bgs_params_.sensitivity_collection = SensitivityConfigCollection::fromJsonString(param.as_string());
                        bgs_worker_ptr_->init_sensitivity(bgs_params_.sensitivity);
                    }
                    catch (const std::exception& e) 
                    {
                        RCLCPP_ERROR(get_logger(), "Failed to parse the JSON data: %s", e.what());
                    }
                }
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("bgs_sensitivity", "medium_c"), 
                [this](const rclcpp::Parameter& param) 
                {
                    RCLCPP_INFO(get_logger(), "Setting Sensitivity: %s", param.as_string().c_str());
                    bgs_worker_ptr_->init_sensitivity(param.as_string());
                }
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("bgs_type", "vibe"), 
                [this](const rclcpp::Parameter& param) 
                {
                    RCLCPP_INFO(get_logger(), "Setting BGS: %s", param.as_string().c_str());
                    bgs_worker_ptr_->init_bgs(param.as_string());
                }
            ),            
            // MASK parameters
            ParameterNode::ActionParam(
                rclcpp::Parameter("bgs_mask_enable_override", true), 
                [this](const rclcpp::Parameter& param) {bgs_params_.mask_enable_override = param.as_bool();}
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("bgs_mask_file", "mask.pgm"), 
                [this](const rclcpp::Parameter& param) 
                {
                    bgs_params_.mask_filename = param.as_string();
                }
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("bgs_mask_enable_offset_correction", true), 
                [this](const rclcpp::Parameter& param) 
                {
                    bgs_params_.mask_enable_roi = param.as_bool();
                }
            ),
            // Image resizing
            ParameterNode::ActionParam(
                rclcpp::Parameter("bgs_image_resized_publish_topic", "bob/frames/foreground_mask/resized"), 
                [this](const rclcpp::Parameter& param) {
                    const auto & image_resized_publish_topic = param.as_string();
                    if (!image_resized_publish_topic.empty())
                    {
                        bgs_params_.image_resized_publisher = create_publisher<sensor_msgs::msg::Image>(image_resized_publish_topic, qos_profile_);
                    }
                    else
                    {
                        bgs_params_.image_resized_publisher.reset();
                        RCLCPP_INFO(get_logger(), "Resizer topic disabled");
                    }
                }
            ),
        };
        add_action_parameters(params);
    }

    void bgs_callback(const std_msgs::msg::Header & header, const cv::Mat & img)
    {
        bgs_worker_ptr_->imageCallback(header, img);
    }

    void reset_bgs_request(const std::shared_ptr<bob_interfaces::srv::BGSResetRequest::Request> request, 
        std::shared_ptr<bob_interfaces::srv::BGSResetRequest::Response> response)
    {
        if(!request->bgs_params.empty() && request->bgs_params.length() > 0) 
        { 
            RCLCPP_INFO(get_logger(), "We have updated bgs params to apply...");
        }
        bgs_worker_ptr_->restart();
        response->success = true;
    }

    void privacy_mask_override_request(const std::shared_ptr<bob_interfaces::srv::MaskOverrideRequest::Request> request, 
        std::shared_ptr<bob_interfaces::srv::MaskOverrideRequest::Response> response)
    {
        camera_params_.mask_enable_override = request->mask_enabled;
        if (request->mask_enabled)
        {
            RCLCPP_DEBUG(get_logger(), "Privacy mask Override set to: True");
        }
        else
        {
            RCLCPP_DEBUG(get_logger(), "Privacy mask Override set to: False");
        }
        response->success = true;        
    }

    void bgs_mask_override_request(const std::shared_ptr<bob_interfaces::srv::MaskOverrideRequest::Request> request, 
        std::shared_ptr<bob_interfaces::srv::MaskOverrideRequest::Response> response)
    {
        bgs_params_.mask_enable_override = request->mask_enabled;
        if (request->mask_enabled)
        {
            RCLCPP_DEBUG(get_logger(), "BGS Mask Override set to: True");
        }
        else
        {
            RCLCPP_DEBUG(get_logger(), "BGS mask Override set to: False");
        }
        response->success = true;        
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::experimental::executors::EventsExecutor executor;
    executor.add_node(std::make_shared<CameraBGS>(rclcpp::NodeOptions()));
    executor.spin();
    rclcpp::shutdown();
    return 0;
}

RCLCPP_COMPONENTS_REGISTER_NODE(CameraBGS)