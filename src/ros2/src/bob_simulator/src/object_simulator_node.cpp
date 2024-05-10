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

        generator_ = std::make_unique<CircleFrameGenerator>(std::map<std::string, int>{{"height", frame_height_}, {"width", frame_width_}}, num_objects_, std::pair<int, int>{3, 10}, std::pair<int, int>{5, 30}, cv::Scalar(255,255,255));
        timer_ = create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000.0 / fps_)), std::bind(&MovingObjectsSimulation::timer_callback, this));
    }

private:
    rclcpp::QoS qos_profile_{10}; 
    int frame_width_;
    int frame_height_;
    int num_objects_;
    int fps_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_resized_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string image_publish_topic_;
    std::unique_ptr<CircleFrameGenerator> generator_;
    std::string image_resized_publish_topic_;
    int resize_height_;

    void declare_node_parameters()
    {
        std::vector<ParameterNode::ActionParam> params = {
            ParameterNode::ActionParam(
                rclcpp::Parameter("image_publish_topic", "bob/simulation/output_frame"), 
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
            ParameterNode::ActionParam(
                rclcpp::Parameter("video_fps", 30.0), 
                [this](const rclcpp::Parameter& param) {fps_ = static_cast<int>(param.as_double());}
            ),
            // Image resizing
            ParameterNode::ActionParam(
                rclcpp::Parameter("image_resized_publish_topic", "bob/simulation/output_frame/resized"), 
                [this](const rclcpp::Parameter& param) {
                    image_resized_publish_topic_ = param.as_string();
                    if (!image_resized_publish_topic_.empty())
                    {
                        image_resized_publisher_ = create_publisher<sensor_msgs::msg::Image>(image_resized_publish_topic_, qos_profile_);
                    }
                    else
                    {
                        image_resized_publisher_.reset();
                        RCLCPP_INFO(get_logger(), "Resizer topic disabled");
                    }
                }
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("resize_height", 960), 
                [this](const rclcpp::Parameter& param) {
                    resize_height_ = param.as_int();
                }
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
        publish_resized_frame(image, header);
    }

    inline void publish_resized_frame(const cv::Mat & image, const std_msgs::msg::Header & header)
    {
        if (!image_resized_publisher_ || (count_subscribers(image_resized_publish_topic_) <= 0))
        {
            return;
        }
        cv::Mat resized_img;
        if (resize_height_ > 0)
        {
            const double aspect_ratio = (double)image.size().width / (double)image.size().height;
            const int frame_height = resize_height_;
            const int frame_width = (int)(aspect_ratio * (double)frame_height);
            cv::resize(image, resized_img, cv::Size(frame_width, frame_height));
        }
        else
        {
            resized_img = image;
        }

        auto resized_frame_msg = cv_bridge::CvImage(header, "bgr8", resized_img).toImageMsg();
        image_resized_publisher_->publish(*resized_frame_msg);
        RCLCPP_INFO(get_logger(), "Resizer topic published to %s", image_resized_publisher_->get_topic_name());
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::StaticSingleThreadedExecutor executor;
    executor.add_node(std::make_shared<MovingObjectsSimulation>(rclcpp::NodeOptions()));
    executor.spin();
    rclcpp::shutdown();
    return 0;
}

RCLCPP_COMPONENTS_REGISTER_NODE(MovingObjectsSimulation)