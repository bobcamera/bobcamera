#include <opencv2/opencv.hpp>

#include <rclcpp_components/register_node_macro.hpp>

#include "parameter_node.hpp"
#include "image_utils.hpp"
#include "background_subtractor_worker.hpp"

#include <visibility_control.h>

#include <sensor_msgs/msg/region_of_interest.hpp>
#include <bob_interfaces/srv/bgs_reset_request.hpp>
#include <bob_interfaces/srv/mask_override_request.hpp>
#include <bob_interfaces/msg/detector_state.hpp>
#include <bob_interfaces/msg/detector_b_box_array.hpp>


class BackgroundSubtractor2
    : public ParameterNode
{
public:
    COMPOSITION_PUBLIC
    explicit BackgroundSubtractor2(const rclcpp::NodeOptions & options)
        : ParameterNode("background_subtractor_node2", options)
        , pub_qos_profile_(10)
        , sub_qos_profile_(10)
    {
        init();
    }
    
private:
    BackgroundSubtractorWorkerParams params_;
    std::unique_ptr<BackgroundSubtractorWorker> bgs_worker_ptr_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;

    rclcpp::Service<bob_interfaces::srv::BGSResetRequest>::SharedPtr bgs_reset_service_;
    rclcpp::Service<bob_interfaces::srv::MaskOverrideRequest>::SharedPtr mask_override_service_;

    rclcpp::Client<bob_interfaces::srv::BGSResetRequest>::SharedPtr bgs_reset_client_;

    std::string image_resized_publish_topic_;

    rclcpp::QoS pub_qos_profile_;
    rclcpp::QoS sub_qos_profile_;

    bool mask_enable_override_;
    std::string mask_filename_;
    int resize_height_;

    void init()
    {
        sub_qos_profile_.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        sub_qos_profile_.durability(rclcpp::DurabilityPolicy::Volatile);
        sub_qos_profile_.history(rclcpp::HistoryPolicy::KeepLast);

        pub_qos_profile_.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        pub_qos_profile_.durability(rclcpp::DurabilityPolicy::Volatile);
        pub_qos_profile_.history(rclcpp::HistoryPolicy::KeepLast);

        bgs_worker_ptr_ = std::make_unique<BackgroundSubtractorWorker>(*this, params_);

        bgs_worker_ptr_->init();

        declare_node_parameters();

        image_subscription_ = create_subscription<sensor_msgs::msg::Image>("bob/frames/allsky/original", sub_qos_profile_,
            std::bind(&BackgroundSubtractor2::imageCallback, this, std::placeholders::_1));
        params_.image_publisher = create_publisher<sensor_msgs::msg::Image>("bob/frames/foreground_mask", pub_qos_profile_);
        params_.detection_publisher = create_publisher<bob_interfaces::msg::DetectorBBoxArray>("bob/detection/allsky/boundingboxes", pub_qos_profile_);        
        params_.state_publisher = create_publisher<bob_interfaces::msg::DetectorState>("bob/detection/detector_state", pub_qos_profile_);
    }

    void declare_node_parameters()
    {
        std::vector<ParameterNode::ActionParam> params = {
            ParameterNode::ActionParam(
                rclcpp::Parameter("json_params", ""), 
                [this](const rclcpp::Parameter& param) 
                {
                    try 
                    {
                        params_.sensitivity_collection = SensitivityConfigCollection::fromJsonString(param.as_string());
                    }
                    catch (const std::exception& e) 
                    {
                        RCLCPP_ERROR(get_logger(), "Failed to parse the JSON data: %s", e.what());
                    }
                }
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("sensitivity", "medium_c"), 
                [this](const rclcpp::Parameter& param) 
                {
                    RCLCPP_INFO(get_logger(), "Setting Sensitivity: %s", param.as_string().c_str());
                    bgs_worker_ptr_->init_sensitivity(param.as_string());
                }
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("bgs", "vibe"), 
                [this](const rclcpp::Parameter& param) 
                {
                    RCLCPP_INFO(get_logger(), "Setting BGS: %s", param.as_string().c_str());
                    bgs_worker_ptr_->init_bgs(param.as_string());
                }
            ),            
            // MASK parameters
            ParameterNode::ActionParam(
                rclcpp::Parameter("mask_file", "mask.pgm"), 
                [this](const rclcpp::Parameter& param) 
                {
                    params_.mask_filename = param.as_string();
                }
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("mask_enable_offset_correction", true), 
                [this](const rclcpp::Parameter& param) 
                {
                    params_.mask_enable_roi = param.as_bool();
                }
            ),
            // Image resizing
            ParameterNode::ActionParam(
                rclcpp::Parameter("image_resized_publish_topic", "bob/frames/foreground_mask/resized"), 
                [this](const rclcpp::Parameter& param) {
                    image_resized_publish_topic_ = param.as_string();
                    if (!image_resized_publish_topic_.empty())
                    {
                        params_.image_resized_publisher = create_publisher<sensor_msgs::msg::Image>(image_resized_publish_topic_, pub_qos_profile_);
                    }
                    else
                    {
                        params_.image_resized_publisher.reset();
                        RCLCPP_INFO(get_logger(), "Resizer topic disabled");
                    }
                }
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("resize_height", 960), 
                [this](const rclcpp::Parameter& param) 
                {
                    params_.resize_height = param.as_int();
                }
            ),        
        };
        add_action_parameters(params);
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr img_msg)
    {
        cv::Mat img;
        ImageUtils::convert_image_msg(img_msg, img, false);

        bgs_worker_ptr_->imageCallback(img_msg->header, img);
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

    void mask_override_request(const std::shared_ptr<bob_interfaces::srv::MaskOverrideRequest::Request> request, 
        std::shared_ptr<bob_interfaces::srv::MaskOverrideRequest::Response> response)
    {
        mask_enable_override_ = request->mask_enabled;
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
    executor.add_node(std::make_shared<BackgroundSubtractor2>(rclcpp::NodeOptions()));
    executor.spin();
    rclcpp::shutdown();
    return 0;
}

RCLCPP_COMPONENTS_REGISTER_NODE(BackgroundSubtractor2)