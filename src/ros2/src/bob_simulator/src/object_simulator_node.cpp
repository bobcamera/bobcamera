#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "parameter_node.hpp"
#include <visibility_control.h>

#include "circle_frame_generator.cpp"


class MovingObjectsSimulation
    : public ParameterNode
{
public:
    COMPOSITION_PUBLIC
    explicit MovingObjectsSimulation(const rclcpp::NodeOptions & options)
        : ParameterNode("moving_objects_simulation_node", options)
    {
        qos_profile_.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        qos_profile_.durability(rclcpp::DurabilityPolicy::Volatile);
        qos_profile_.history(rclcpp::HistoryPolicy::KeepLast);

        declare_node_parameters();

        generator_ = std::make_unique<CircleFrameGenerator>(std::map<std::string, int>{{"height", frame_height_}, {"width", frame_width_}}, num_objects_, std::pair<int, int>{2, 10}, std::pair<int, int>{5, 30}, cv::Scalar(255,255,255));
        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&MovingObjectsSimulation::timer_callback, this));
    }

private:
    rclcpp::QoS qos_profile_{10}; 
    int frame_width_;
    int frame_height_;
    int num_objects_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string image_publish_topic_;
    std::unique_ptr<CircleFrameGenerator> generator_;

    void declare_node_parameters()
    {
        std::vector<ParameterNode::ActionParam> params = {
            ParameterNode::ActionParam(
                rclcpp::Parameter("image_publish_topic", "bob/camera/all_sky/bayer"), 
                [this](const rclcpp::Parameter& param) {
                    image_publish_topic_ = param.as_string(); 
                    image_publisher_ = create_publisher<sensor_msgs::msg::Image>(image_publish_topic_, qos_profile_);
                }
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("height", 1080), 
                [this](const rclcpp::Parameter& param) {frame_height_ = param.as_int();}
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("width", 1920), 
                [this](const rclcpp::Parameter& param) {frame_width_ = param.as_int();}
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("num_objects", 5), 
                [this](const rclcpp::Parameter& param) {num_objects_ = param.as_int();}
            ),
        };
        add_action_parameters(params);
    }

    void timer_callback()
    {
        cv::Mat image = generator_->generate_frame();
        std_msgs::msg::Header header;
        header.stamp = this->now();
        auto image_msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
        image_publisher_->publish(*image_msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MovingObjectsSimulation>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}

RCLCPP_COMPONENTS_REGISTER_NODE(MovingObjectsSimulation)