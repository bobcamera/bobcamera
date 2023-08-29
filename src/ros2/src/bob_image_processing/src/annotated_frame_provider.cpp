#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <sensor_msgs/msg/image.hpp>
#include "bob_interfaces/msg/tracking_state.hpp"
#include "bob_interfaces/msg/track_detection_array.hpp"
#include "bob_interfaces/msg/track_trajectory_array.hpp"

#include <boblib/api/utils/profiler.hpp>

#include "annotated_frame/annotated_frame_creator.hpp"

#include "parameter_node.hpp"
#include "image_utils.hpp"

class AnnotatedFrameProvider 
    : public ParameterNode
{
public:
    static std::shared_ptr<AnnotatedFrameProvider> create()
    {
        auto result = std::shared_ptr<AnnotatedFrameProvider>(new AnnotatedFrameProvider());
        result->init();
        return result;
    }

private:
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> sub_masked_frame;
    std::shared_ptr<message_filters::Subscriber<bob_interfaces::msg::TrackingState>> sub_tracking_state;
    std::shared_ptr<message_filters::Subscriber<bob_interfaces::msg::TrackDetectionArray>> sub_tracker_detections;
    std::shared_ptr<message_filters::Subscriber<bob_interfaces::msg::TrackTrajectoryArray>> sub_tracker_trajectory;
    std::shared_ptr<message_filters::Subscriber<bob_interfaces::msg::TrackTrajectoryArray>> sub_tracker_prediction;
    std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, bob_interfaces::msg::TrackingState, 
        bob_interfaces::msg::TrackDetectionArray, bob_interfaces::msg::TrackTrajectoryArray, bob_interfaces::msg::TrackTrajectoryArray>> time_synchronizer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_annotated_frame_;
    AnnotatedFrameCreator annotated_frame_creator_;
    boblib::utils::Profiler profiler_;

    friend std::shared_ptr<AnnotatedFrameProvider> std::make_shared<AnnotatedFrameProvider>();

    AnnotatedFrameProvider() 
        : ParameterNode("annotated_frame_provider_node")
        , annotated_frame_creator_(std::map<std::string, std::string>())
    {
    }

    void init()
    {
        rclcpp::QoS pub_qos_profile{2};
        pub_qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        pub_qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);
        pub_qos_profile.history(rclcpp::HistoryPolicy::KeepLast);

        rclcpp::QoS sub_qos_profile{2};
        sub_qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        sub_qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);
        sub_qos_profile.history(rclcpp::HistoryPolicy::KeepLast);
        auto rmw_qos_profile = sub_qos_profile.get_rmw_qos_profile();

        pub_annotated_frame_ = create_publisher<sensor_msgs::msg::Image>("bob/frames/annotated", pub_qos_profile);

        sub_masked_frame = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(shared_from_this(), "bob/camera/all_sky/bayer", rmw_qos_profile);
        sub_tracking_state = std::make_shared<message_filters::Subscriber<bob_interfaces::msg::TrackingState>>(shared_from_this(), "bob/tracker/tracking_state", rmw_qos_profile);
        sub_tracker_detections = std::make_shared<message_filters::Subscriber<bob_interfaces::msg::TrackDetectionArray>>(shared_from_this(), "bob/tracker/detections", rmw_qos_profile);
        sub_tracker_trajectory = std::make_shared<message_filters::Subscriber<bob_interfaces::msg::TrackTrajectoryArray>>(shared_from_this(), "bob/tracker/trajectory", rmw_qos_profile);
        sub_tracker_prediction = std::make_shared<message_filters::Subscriber<bob_interfaces::msg::TrackTrajectoryArray>>(shared_from_this(), "bob/tracker/prediction", rmw_qos_profile);

        time_synchronizer_ = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, bob_interfaces::msg::TrackingState, bob_interfaces::msg::TrackDetectionArray, bob_interfaces::msg::TrackTrajectoryArray, bob_interfaces::msg::TrackTrajectoryArray>>(
            *sub_masked_frame, *sub_tracking_state, *sub_tracker_detections, *sub_tracker_trajectory, *sub_tracker_prediction, 2);
        time_synchronizer_->registerCallback(&AnnotatedFrameProvider::callback, this);
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
    rclcpp::spin(AnnotatedFrameProvider::create());
    rclcpp::shutdown();
    return 0;
}
