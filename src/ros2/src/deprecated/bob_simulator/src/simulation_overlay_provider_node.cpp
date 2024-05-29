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
    : ParameterNode("simulation_overlay_provider_node", options),
      default_size_range_{2, 10}, 
      default_step_range_{5, 30} 
    {
        rclcpp::QoS subscriber_qos_profile(10);
        subscriber_qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        subscriber_qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);
        subscriber_qos_profile.history(rclcpp::HistoryPolicy::KeepLast);

        publisher_qos_profile_.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        publisher_qos_profile_.durability(rclcpp::DurabilityPolicy::Volatile);
        publisher_qos_profile_.history(rclcpp::HistoryPolicy::KeepLast);

        declare_node_parameters();

        for(int i = 0; i < num_simulated_objects_; i++) {
            moving_circles_.emplace_back(width_, height_, 0.5, default_size_range_, default_step_range_);
        }

        sub_camera_ = create_subscription<sensor_msgs::msg::Image>(
            "bob/simulation/input_frame",
            subscriber_qos_profile,
            std::bind(&SimulationOverlayProviderNode::camera_callback, this, std::placeholders::_1)
        );

        pub_synthetic_frame_ = create_publisher<sensor_msgs::msg::Image>("bob/simulation/output_frame", publisher_qos_profile_);
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
            // Image resizing
            ParameterNode::ActionParam(
                rclcpp::Parameter("image_resized_publish_topic", "bob/simulation/output_frame/resized"), 
                [this](const rclcpp::Parameter& param) {
                    image_resized_publish_topic_ = param.as_string();
                    if (!image_resized_publish_topic_.empty())
                    {
                        image_resized_publisher_ = create_publisher<sensor_msgs::msg::Image>(image_resized_publish_topic_, publisher_qos_profile_);
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

private:
    int height_, width_, num_simulated_objects_;
    std::vector<MovingCircle> moving_circles_;
    rclcpp::QoS publisher_qos_profile_{10};
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_camera_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_synthetic_frame_;
    std::pair<int, int> default_size_range_; 
    std::pair<int, int> default_step_range_;  
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_resized_publisher_;
    std::string image_resized_publish_topic_;
    int resize_height_;

    void camera_callback(const sensor_msgs::msg::Image::SharedPtr msg_image) 
    {
        if (msg_image->data.empty()) 
        {
            RCLCPP_WARN(get_logger(), "Received an empty image frame. Skipping processing.");
            return;
        }

        try 
        {
            cv::Mat input_frame;
            ImageUtils::convert_image_msg(msg_image, input_frame, false);

            if (input_frame.cols != width_ || input_frame.rows != height_) 
            {
                width_ = input_frame.cols;
                height_ = input_frame.rows;
                moving_circles_.clear();
                for(int i = 0; i < num_simulated_objects_; i++) 
                {
                    moving_circles_.emplace_back(width_, height_, 0.5, default_size_range_, default_step_range_);
                }
            }

            for(auto &circle : moving_circles_) 
            {
                circle.move();
                circle.draw(input_frame, cv::Scalar(10, 10, 20));
            }
            auto frame_synthetic_msg = cv_bridge::CvImage(msg_image->header, input_frame.channels() == 1 ? sensor_msgs::image_encodings::MONO8 : sensor_msgs::image_encodings::BGR8, input_frame).toImageMsg();
            pub_synthetic_frame_->publish(*frame_synthetic_msg);

            publish_resized_frame(input_frame, msg_image->header);
        } 
        catch(const std::exception& e) 
        {
            RCLCPP_ERROR(get_logger(), "Exception during frame overlay simulation: %s", e.what());
        }
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
    }
};

int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);
    rclcpp::executors::StaticSingleThreadedExecutor executor;
    executor.add_node(std::make_shared<SimulationOverlayProviderNode>(rclcpp::NodeOptions()));
    executor.spin();
    rclcpp::shutdown();
    return 0;
}

RCLCPP_COMPONENTS_REGISTER_NODE(SimulationOverlayProviderNode)