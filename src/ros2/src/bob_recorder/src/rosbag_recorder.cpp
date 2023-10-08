#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <sensor_msgs/msg/image.hpp>
#include "bob_interfaces/msg/tracking.hpp"

#include "parameter_node.hpp"
#include "image_utils.hpp"

#include <visibility_control.h>

class RosbagRecorder 
    : public ParameterNode
{
public:
    COMPOSITION_PUBLIC
    explicit RosbagRecorder(const rclcpp::NodeOptions & options) 
        : ParameterNode("rosbag_recorder", options)
    {
        timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&RosbagRecorder::init, this));
    }

private:
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> sub_masked_frame_;
    std::shared_ptr<message_filters::Subscriber<bob_interfaces::msg::Tracking>> sub_tracking_;
    std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, bob_interfaces::msg::Tracking>> time_synchronizer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_annotated_frame_;
    rclcpp::TimerBase::SharedPtr timer_;

    friend std::shared_ptr<RosbagRecorder> std::make_shared<RosbagRecorder>();

    void init()
    {
        RCLCPP_INFO(get_logger(), "Initializing RosbagRecorder");

        timer_->cancel();

        rclcpp::QoS sub_qos_profile{10};
        sub_qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        sub_qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);
        sub_qos_profile.history(rclcpp::HistoryPolicy::KeepLast);
        auto rmw_qos_profile = sub_qos_profile.get_rmw_qos_profile();

        sub_masked_frame_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(shared_from_this(), "bob/camera/all_sky/bayer", rmw_qos_profile);
        sub_tracking_ = std::make_shared<message_filters::Subscriber<bob_interfaces::msg::Tracking>>(shared_from_this(), "bob/tracker/tracking", rmw_qos_profile);

        time_synchronizer_ = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, bob_interfaces::msg::Tracking>>(*sub_masked_frame_, *sub_tracking_, 10);
        time_synchronizer_->registerCallback(&RosbagRecorder::callback, this);
    }

    void callback(const sensor_msgs::msg::Image::SharedPtr& image_msg
                , const bob_interfaces::msg::Tracking::SharedPtr& tracking_msg)
    {
        try
        {
            if (tracking_msg->state.trackable > 0) 
            {
                RCLCPP_INFO(get_logger(), "Recorder Logic goes here --- Recording.....");
            }
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(get_logger(), "CV bridge exception: %s", e.what());
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RosbagRecorder>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}

RCLCPP_COMPONENTS_REGISTER_NODE(RosbagRecorder)