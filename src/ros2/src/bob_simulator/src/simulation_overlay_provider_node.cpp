#include <chrono>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "parameter_node.hpp"
#include <visibility_control.h>

#include "circle_frame_generator.cpp" 
#include "image_utils.hpp"

class SimulationOverlayProviderNode 
    : public ParameterNode 
{
public:
    COMPOSITION_PUBLIC
    explicit SimulationOverlayProviderNode(const rclcpp::NodeOptions & options) 
    : ParameterNode("simulation_overlay_provider_node", options) 
    {
        rclcpp::QoS subscriber_qos_profile(10);
        subscriber_qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        subscriber_qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);
        subscriber_qos_profile.history(rclcpp::HistoryPolicy::KeepLast);

        rclcpp::QoS publisher_qos_profile(10);
        publisher_qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        publisher_qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);
        publisher_qos_profile.history(rclcpp::HistoryPolicy::KeepLast);

        declare_node_parameters();

        std::pair<int, int> default_size_range(2, 10); 
        std::pair<int, int> default_step_range(5, 30);  

        for(int i = 0; i < num_simulated_objects_; i++) {
            moving_circles_.emplace_back(width_, height_, 0.5, default_size_range, default_step_range);
        }

        sub_camera_ = this->create_subscription<sensor_msgs::msg::Image>(
            "bob/simulation/input_frame",
            subscriber_qos_profile,
            std::bind(&SimulationOverlayProviderNode::camera_callback, this, std::placeholders::_1)
        );

        pub_synthetic_frame_ = this->create_publisher<sensor_msgs::msg::Image>("bob/simulation/output_frame", publisher_qos_profile);
    }

    void declare_node_parameters()
    {
        std::vector<ParameterNode::ActionParam> params = {
            ParameterNode::ActionParam(
                rclcpp::Parameter("height", 1080), 
                [this](const rclcpp::Parameter& param) {height_ = param.as_int();}
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("width", 1920), 
                [this](const rclcpp::Parameter& param) {width_ = param.as_int();}
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("num_objects", 5), 
                [this](const rclcpp::Parameter& param) {num_simulated_objects_ = param.as_int();}
            ),
        };
        add_action_parameters(params);
    }

private:
    int height_, width_, num_simulated_objects_;
    std::vector<MovingCircle> moving_circles_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_camera_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_synthetic_frame_;

    void camera_callback(const sensor_msgs::msg::Image::SharedPtr msg_image) {

        if (msg_image->data.empty()) {
            RCLCPP_WARN(get_logger(), "Received an empty image frame. Skipping processing.");
            return;
        }

        try {
            
            cv::Mat input_frame;
            ImageUtils::convert_image_msg(msg_image, input_frame, false);

            for(auto &circle : moving_circles_) {
                circle.move();
                circle.draw(input_frame, cv::Scalar(10, 10, 20));
            }
            auto frame_synthetic_msg = cv_bridge::CvImage(msg_image->header, input_frame.channels() == 1 ? sensor_msgs::image_encodings::MONO8 : sensor_msgs::image_encodings::BGR8, input_frame).toImageMsg();
            pub_synthetic_frame_->publish(*frame_synthetic_msg);


        } catch(const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Exception during frame overlay simulation: %s", e.what());
        }
    }
};

int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimulationOverlayProviderNode>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}


RCLCPP_COMPONENTS_REGISTER_NODE(SimulationOverlayProviderNode)