#include <vector>

#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <sensor_msgs/msg/image.hpp>

#include <boblib/api/utils/profiler.hpp>

#include "parameter_lifecycle_node.hpp"
#include "image_utils.hpp"

#include <visibility_control.h>

#include <rclcpp/experimental/executors/events_executor/events_executor.hpp>

class FrameCompressor
    : public ParameterLifeCycleNode
{
public:
    COMPOSITION_PUBLIC
    explicit FrameCompressor(const rclcpp::NodeOptions & options) 
        : ParameterLifeCycleNode("frame_compressor_node", options)
        , sub_qos_profile_(2)
        , pub_qos_profile_(5)
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

        pub_qos_profile_.reliability(rclcpp::ReliabilityPolicy::Reliable);
        pub_qos_profile_.durability(rclcpp::DurabilityPolicy::Volatile);
        pub_qos_profile_.history(rclcpp::HistoryPolicy::KeepLast);

        declare_node_parameters();

        timer_ = create_wall_timer(std::chrono::seconds(2), [this](){check_subscribers();});
    }

    void declare_node_parameters()
    {
        std::vector<ParameterLifeCycleNode::ActionParam> params = {
            ParameterLifeCycleNode::ActionParam(
                rclcpp::Parameter("compressed_frame_subscriber_topic", "bob/compressor/source"), 
                [this](const rclcpp::Parameter& param) 
                {
                    compressed_frame_subscriber_topic_ = param.as_string();
                    image_subscription_.reset();
                    pub_compressed_frame_ = create_publisher<sensor_msgs::msg::CompressedImage>(compressed_frame_subscriber_topic_ + "/compressed", pub_qos_profile_);
                    log_debug("Creating topic %s", pub_compressed_frame_->get_topic_name());
                }
            ),
            ParameterLifeCycleNode::ActionParam(
                rclcpp::Parameter("compression_quality", 75), 
                [this](const rclcpp::Parameter& param) 
                {
                    compression_quality_ = static_cast<int>(param.as_int());
                    compression_params_.clear();
                    compression_params_.push_back(cv::IMWRITE_JPEG_QUALITY);
                    compression_params_.push_back(compression_quality_);
                }
            ),
        };
        add_action_parameters(params);
    }

    void check_subscribers() 
    {
        timer_->cancel();
        try
        {
            auto num_subs = count_subscribers(pub_compressed_frame_->get_topic_name());
            if ((num_subs > 0) && !image_subscription_)
            {
                image_subscription_ = create_subscription<sensor_msgs::msg::Image>(compressed_frame_subscriber_topic_, sub_qos_profile_,
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

    void image_callback(const sensor_msgs::msg::Image::SharedPtr image_msg) const
    {
        try
        {
            cv::Mat image;
            ImageUtils::convert_image_msg(image_msg, image, false);            

            sensor_msgs::msg::CompressedImage compressed_image_msg;
            compressed_image_msg.header = image_msg->header;
            compressed_image_msg.format = "jpeg";
            compressed_image_msg.data.reserve(image.total() * image.elemSize() / 2);

            cv::imencode(".jpg", image, compressed_image_msg.data, compression_params_);            

            pub_compressed_frame_->publish(compressed_image_msg);
        }
        catch (const std::exception & e)
        {
            log_error("image_callback: exception: %s", e.what());
        }
        catch (...)
        {
            log_error("image_callback: unknown exception");
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::QoS sub_qos_profile_;
    rclcpp::QoS pub_qos_profile_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_compressed_frame_;
    std::string compressed_frame_subscriber_topic_;
    int compression_quality_;
    std::vector<int> compression_params_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(FrameCompressor)