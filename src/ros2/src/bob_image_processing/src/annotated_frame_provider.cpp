#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <sensor_msgs/msg/image.hpp>
#include "bob_interfaces/msg/tracking_state.hpp"
#include "bob_interfaces/msg/track_detection_array.hpp"
#include "bob_interfaces/msg/track_trajectory_array.hpp"
#include <vision_msgs/msg/bounding_box2_d_array.hpp>

#include "annotated_frame/annotated_frame_creator.hpp"

#include "parameter_node.hpp"
#include "image_utils.hpp"

#include <visibility_control.h>

class AnnotatedFrameProvider 
    : public ParameterNode
{
public:
    COMPOSITION_PUBLIC
    explicit AnnotatedFrameProvider(const rclcpp::NodeOptions & options) 
        : ParameterNode("annotated_frame_provider_node", options)
        , annotated_frame_creator_(std::map<std::string, std::string>())
    {
        timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&AnnotatedFrameProvider::init, this));
    }

private:
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> sub_masked_frame_;
    std::shared_ptr<message_filters::Subscriber<bob_interfaces::msg::TrackingState>> sub_tracking_state_;
    std::shared_ptr<message_filters::Subscriber<bob_interfaces::msg::TrackDetectionArray>> sub_tracker_detections_;
    std::shared_ptr<message_filters::Subscriber<bob_interfaces::msg::TrackTrajectoryArray>> sub_tracker_trajectory_;
    std::shared_ptr<message_filters::Subscriber<bob_interfaces::msg::TrackTrajectoryArray>> sub_tracker_prediction_;
    std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, bob_interfaces::msg::TrackingState, 
        bob_interfaces::msg::TrackDetectionArray, bob_interfaces::msg::TrackTrajectoryArray, bob_interfaces::msg::TrackTrajectoryArray>> time_synchronizer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_annotated_frame_;
    AnnotatedFrameCreator annotated_frame_creator_;
    rclcpp::TimerBase::SharedPtr timer_;

    friend std::shared_ptr<AnnotatedFrameProvider> std::make_shared<AnnotatedFrameProvider>();

    void init()
    {
        RCLCPP_INFO(get_logger(), "Initializing AnnotatedFrameProvider");

        timer_->cancel();

        rclcpp::QoS pub_qos_profile{10};
        pub_qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        pub_qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);
        pub_qos_profile.history(rclcpp::HistoryPolicy::KeepLast);

        rclcpp::QoS sub_qos_profile{10};
        sub_qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        sub_qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);
        sub_qos_profile.history(rclcpp::HistoryPolicy::KeepLast);
        auto rmw_qos_profile = sub_qos_profile.get_rmw_qos_profile();

        sub_masked_frame_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(shared_from_this(), "bob/camera/all_sky/bayer", rmw_qos_profile);
        sub_tracking_state_ = std::make_shared<message_filters::Subscriber<bob_interfaces::msg::TrackingState>>(shared_from_this(), "bob/tracker/tracking_state", rmw_qos_profile);
        sub_tracker_detections_ = std::make_shared<message_filters::Subscriber<bob_interfaces::msg::TrackDetectionArray>>(shared_from_this(), "bob/tracker/detections", rmw_qos_profile);
        sub_tracker_trajectory_ = std::make_shared<message_filters::Subscriber<bob_interfaces::msg::TrackTrajectoryArray>>(shared_from_this(), "bob/tracker/trajectory", rmw_qos_profile);
        sub_tracker_prediction_ = std::make_shared<message_filters::Subscriber<bob_interfaces::msg::TrackTrajectoryArray>>(shared_from_this(), "bob/tracker/prediction", rmw_qos_profile);

        time_synchronizer_ = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, bob_interfaces::msg::TrackingState, bob_interfaces::msg::TrackDetectionArray, 
            bob_interfaces::msg::TrackTrajectoryArray, bob_interfaces::msg::TrackTrajectoryArray>>(*sub_masked_frame_, *sub_tracking_state_, *sub_tracker_detections_, *sub_tracker_trajectory_, *sub_tracker_prediction_, 10);
        time_synchronizer_->registerCallback(&AnnotatedFrameProvider::callback, this);

        pub_annotated_frame_ = create_publisher<sensor_msgs::msg::Image>("bob/frames/annotated", pub_qos_profile);
    }

    void callback(const sensor_msgs::msg::Image::SharedPtr& image_msg
                , const bob_interfaces::msg::TrackingState::SharedPtr& tracking_state_msg
                , const bob_interfaces::msg::TrackDetectionArray::SharedPtr& detections_msg
                , const bob_interfaces::msg::TrackTrajectoryArray::SharedPtr& trajectory_msg
                , const bob_interfaces::msg::TrackTrajectoryArray::SharedPtr& prediction_msg)
    {
        try
        {
            cv::Mat img;
            ImageUtils::convert_image_msg(image_msg, img, true);

            auto annotated_frame = annotated_frame_creator_.create_frame(img, *tracking_state_msg, *detections_msg, *trajectory_msg, *prediction_msg);

            auto annotated_frame_msg = cv_bridge::CvImage(image_msg->header, sensor_msgs::image_encodings::BGR8, annotated_frame).toImageMsg();
            pub_annotated_frame_->publish(*annotated_frame_msg);
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
    rclcpp::spin(std::make_shared<AnnotatedFrameProvider>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}

RCLCPP_COMPONENTS_REGISTER_NODE(AnnotatedFrameProvider)