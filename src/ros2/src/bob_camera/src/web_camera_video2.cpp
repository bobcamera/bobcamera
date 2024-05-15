#include <chrono>
#include <string>

#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <bob_interfaces/srv/mask_override_request.hpp>
#include <sensor_msgs/msg/region_of_interest.hpp>

#include "parameter_node.hpp"
#include "image_utils.hpp"
#include <visibility_control.h>

#include <camera_worker.hpp>

class WebCameraVideo2
    : public ParameterNode
{
public:
    COMPOSITION_PUBLIC
    explicit WebCameraVideo2(const rclcpp::NodeOptions & options)
        : ParameterNode("web_camera_video_node", options)
    {
        qos_profile_.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        qos_profile_.durability(rclcpp::DurabilityPolicy::Volatile);
        qos_profile_.history(rclcpp::HistoryPolicy::KeepLast);

        one_shot_timer_ = create_wall_timer(std::chrono::seconds(2), [this](){init();});
    }

private:
    CameraWorkerParams params_;
    std::unique_ptr<CameraWorker> camera_worker_ptr_;

    rclcpp::QoS qos_profile_{10}; 
    rclcpp::Service<bob_interfaces::srv::MaskOverrideRequest>::SharedPtr mask_override_service_;
    rclcpp::TimerBase::SharedPtr one_shot_timer_;

    void init()
    {
        one_shot_timer_.reset();

        mask_override_service_ = create_service<bob_interfaces::srv::MaskOverrideRequest>("bob/mask/override",
            [this](const std::shared_ptr<bob_interfaces::srv::MaskOverrideRequest::Request> request, 
                    std::shared_ptr<bob_interfaces::srv::MaskOverrideRequest::Response> response) {
                        mask_override_request(request, response);
                    });

        params_.roi_publisher = create_publisher<sensor_msgs::msg::RegionOfInterest>("bob/mask/roi", qos_profile_);

        declare_node_parameters();

        camera_worker_ptr_ = std::make_unique<CameraWorker>(*this, params_);
        camera_worker_ptr_->init();
    }

    void declare_node_parameters()
    {
        std::vector<ParameterNode::ActionParam> params = {
            ParameterNode::ActionParam(
                rclcpp::Parameter("image_publish_topic", "bob/frames/allsky/original"), 
                [this](const rclcpp::Parameter& param) {
                    params_.image_publish_topic = param.as_string(); 
                    params_.image_publisher = create_publisher<sensor_msgs::msg::Image>(params_.image_publish_topic, qos_profile_);
                }
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("image_info_publish_topic", "bob/camera/all_sky/image_info"), 
                [this](const rclcpp::Parameter& param) {
                    params_.image_info_publish_topic = param.as_string(); 
                    params_.image_info_publisher = create_publisher<bob_camera::msg::ImageInfo>(params_.image_info_publish_topic, qos_profile_);
                }
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("camera_info_publish_topic", "bob/camera/all_sky/camera_info"), 
                [this](const rclcpp::Parameter& param) {
                    params_.camera_info_publish_topic = param.as_string(); 
                    params_.camera_info_publisher = create_publisher<bob_camera::msg::CameraInfo>(params_.camera_info_publish_topic, qos_profile_);
                }
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("image_resized_publish_topic", "bob/frames/allsky/original/resized"), 
                [this](const rclcpp::Parameter& param) {
                    params_.image_resized_publish_topic = param.as_string();
                    if (!params_.image_resized_publish_topic.empty())
                    {
                        params_.image_resized_publisher = create_publisher<sensor_msgs::msg::Image>(params_.image_resized_publish_topic, qos_profile_);
                    }
                    else
                    {
                        params_.image_resized_publisher.reset();
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
                        params_.source_type = USB_CAMERA;
                    } 
                    else if (type == "VIDEO_FILE") 
                    {
                        params_.source_type = VIDEO_FILE;
                    } 
                    else if (type == "RTSP_STREAM") 
                    {
                        params_.source_type = RTSP_STREAM;
                    } 
                    else
                    {
                        RCLCPP_ERROR(get_logger(), "Invalid source type: %s", type.c_str());
                    }
                }
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("camera_id", 0), 
                [this](const rclcpp::Parameter& param) {params_.camera_id = static_cast<int>(param.as_int());}
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("videos", std::vector<std::string>({""})), 
                [this](const rclcpp::Parameter& param) {params_.videos = param.as_string_array();}
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("rtsp_uri", ""), 
                [this](const rclcpp::Parameter& param) {params_.rtsp_uri = param.as_string();}
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("onvif_host", "192.168.1.20"),
                [this](const rclcpp::Parameter& param) {params_.onvif_host = param.as_string();}
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("onvif_port", 80),
                [this](const rclcpp::Parameter& param) {params_.onvif_port = static_cast<int>(param.as_int());}
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("onvif_user", "default_user"),
                [this](const rclcpp::Parameter& param) {params_.onvif_user = param.as_string();}
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("onvif_password", "default_password"),
                [this](const rclcpp::Parameter& param) {params_.onvif_password = param.as_string();}
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("resize_height", 960), 
                [this](const rclcpp::Parameter& param) {
                    params_.resize_height = static_cast<int>(param.as_int());
                }
            ),
            // MASK parameters
            ParameterNode::ActionParam(
                rclcpp::Parameter("mask_file", "mask.pgm"), 
                [this](const rclcpp::Parameter& param) {params_.mask_filename = param.as_string();}
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("mask_enable_offset_correction", false), 
                [this](const rclcpp::Parameter& param) {params_.mask_enable_roi = param.as_bool();}
            ),
        };
        add_action_parameters(params);
    }

    void mask_override_request(const std::shared_ptr<bob_interfaces::srv::MaskOverrideRequest::Request> request, 
        std::shared_ptr<bob_interfaces::srv::MaskOverrideRequest::Response> response)
    {
        params_.mask_enable_override = request->mask_enabled;
        if (request->mask_enabled)
        {
            RCLCPP_DEBUG(get_logger(), "Mask Override set to: True");
        }
        else
        {
            RCLCPP_DEBUG(get_logger(), "Mask Override set to: False");
        }
        response->success = true;        
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::experimental::executors::EventsExecutor executor;
    executor.add_node(std::make_shared<WebCameraVideo2>(rclcpp::NodeOptions()));
    executor.spin();
    rclcpp::shutdown();
    return 0;
}

RCLCPP_COMPONENTS_REGISTER_NODE(WebCameraVideo2)