#include <vector>

#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <sensor_msgs/msg/image.hpp>

#include <boblib/api/utils/profiler.hpp>

#include "parameter_node.hpp"
#include "image_utils.hpp"

#include <visibility_control.h>

class FrameResizer
    : public ParameterNode
{
public:
    COMPOSITION_PUBLIC
    explicit FrameResizer(const rclcpp::NodeOptions & options) 
        : ParameterNode("frame_resizer_node", options)
    {
        declare_node_parameters();
        init();
    }

private:
    rclcpp::QoS sub_qos_profile_{10};
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_resized_frame_;
    boblib::utils::Profiler profiler_;
    int resize_height_;

    friend std::shared_ptr<FrameResizer> std::make_shared<FrameResizer>();

    void declare_node_parameters()
    {
        std::vector<ParameterNode::ActionParam> params = {
            ParameterNode::ActionParam(
                rclcpp::Parameter("resize_height", 960), 
                [this](const rclcpp::Parameter& param) {resize_height_ = param.as_int();}
            ),
        };
        add_action_parameters(params);
    }
    
    void init()
    {
        rclcpp::QoS sub_qos_profile{10};
        sub_qos_profile_.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        sub_qos_profile_.durability(rclcpp::DurabilityPolicy::Volatile);
        sub_qos_profile_.history(rclcpp::HistoryPolicy::KeepLast);

        rclcpp::QoS pub_qos_profile{10};
        pub_qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        pub_qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);
        pub_qos_profile.history(rclcpp::HistoryPolicy::KeepLast);

        image_subscription_ = create_subscription<sensor_msgs::msg::Image>("bob/resizer/source", sub_qos_profile_,
            std::bind(&FrameResizer::imageCallback, this, std::placeholders::_1));

        pub_resized_frame_ = create_publisher<sensor_msgs::msg::Image>("bob/resizer/target", pub_qos_profile);
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr image_msg)
    {
        try
        {
            cv::Mat image;
            ImageUtils::convert_image_msg(image_msg, image, true);

            if (resize_height_ > 0)
            {
                const double aspect_ratio = (double)image.size().width / (double)image.size().height;
                const int frame_height = resize_height_;
                const int frame_width = (int)(aspect_ratio * (double)frame_height);
                cv::resize(image, image, cv::Size(frame_width, frame_height));
            }

            auto resized_frame_msg = cv_bridge::CvImage(image_msg->header, image_msg->encoding, image).toImageMsg();
            pub_resized_frame_->publish(*resized_frame_msg);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(get_logger(), "CV bridge exception: %s", e.what());
        }
        catch (cv::Exception &cve)
        {
            RCLCPP_ERROR(get_logger(), "Open CV exception: %s", cve.what());
        }        
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::StaticSingleThreadedExecutor executor;
    executor.add_node(std::make_shared<FrameResizer>(rclcpp::NodeOptions()));
    executor.spin();
    rclcpp::shutdown();
    return 0;
}

RCLCPP_COMPONENTS_REGISTER_NODE(FrameResizer)