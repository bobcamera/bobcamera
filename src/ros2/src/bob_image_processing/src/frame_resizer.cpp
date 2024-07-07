#include <vector>

#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <sensor_msgs/msg/image.hpp>

#include "parameter_lifecycle_node.hpp"
#include "image_utils.hpp"

#include <visibility_control.h>

#include <rclcpp/experimental/executors/events_executor/events_executor.hpp>

class FrameResizer
    : public ParameterLifeCycleNode
{
public:
    COMPOSITION_PUBLIC
    explicit FrameResizer(const rclcpp::NodeOptions & options) 
        : ParameterLifeCycleNode("frame_resizer_node", options)
        , pub_qos_profile_(10)
        , sub_qos_profile_(10)
    {
    }

    CallbackReturn on_configure(const rclcpp_lifecycle::State &)
    {
        log_info("Configuring");

        init();

        return CallbackReturn::SUCCESS;
    }

private:
    void init()
    {
        sub_qos_profile_.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        sub_qos_profile_.durability(rclcpp::DurabilityPolicy::Volatile);
        sub_qos_profile_.history(rclcpp::HistoryPolicy::KeepLast);

        pub_qos_profile_.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        pub_qos_profile_.durability(rclcpp::DurabilityPolicy::Volatile);
        pub_qos_profile_.history(rclcpp::HistoryPolicy::KeepLast);

        declare_node_parameters();

        timer_ = create_wall_timer(std::chrono::seconds(2), [this](){check_subscribers();});
    }

    void declare_node_parameters()
    {
        std::vector<ParameterLifeCycleNode::ActionParam> params = {
            ParameterLifeCycleNode::ActionParam(
                rclcpp::Parameter("resized_frame_subscriber_topic", "bob/resizer/source"), 
                [this](const rclcpp::Parameter& param) 
                {
                    resized_frame_subscriber_topic_ = param.as_string();
                    image_subscription_.reset();
                }
            ),
            ParameterLifeCycleNode::ActionParam(
                rclcpp::Parameter("resized_frame_publisher_topic", "bob/resizer/target"), 
                [this](const rclcpp::Parameter& param) 
                {
                    pub_resized_frame_ = create_publisher<sensor_msgs::msg::Image>(param.as_string(), pub_qos_profile_);
                    log_debug("Creating topic %s", pub_resized_frame_->get_topic_name());
                }
            ),
            ParameterLifeCycleNode::ActionParam(
                rclcpp::Parameter("resize_height", 960), 
                [this](const rclcpp::Parameter& param) {resize_height_ = param.as_int();}
            ),
        };
        add_action_parameters(params);
    }

    void check_subscribers() 
    {
        timer_->cancel();
        try
        {
            auto num_subs = count_subscribers(pub_resized_frame_->get_topic_name());
            if ((num_subs > 0) && !image_subscription_)
            {
                image_subscription_ = create_subscription<sensor_msgs::msg::Image>(resized_frame_subscriber_topic_, sub_qos_profile_,
                    [this](const sensor_msgs::msg::Image::SharedPtr image_msg){image_callback(image_msg);});
                log_debug("Subscribing to %s", image_subscription_->get_topic_name());
            } 
            else if ((num_subs <= 0) && image_subscription_) 
            {
                log_debug("Unsubscribing from %s", image_subscription_->get_topic_name());
                image_subscription_.reset();
            }
        }
        catch (...)
        {
            // Ignoring, probably due to application closing
        }
        timer_->reset();
    }
    
    void image_callback(const sensor_msgs::msg::Image::SharedPtr image_msg)
    {
        try
        {
            if (count_subscribers(pub_resized_frame_->get_topic_name()) <= 0)
            {
                return;
            }

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
        catch (const std::exception & e)
        {
            log_send_error("image_callback: exception: %s", e.what());
        }        
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::QoS pub_qos_profile_;
    rclcpp::QoS sub_qos_profile_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_resized_frame_;
    std::string resized_frame_subscriber_topic_;
    int resize_height_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(FrameResizer)