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

#include <rosbag2_cpp/writer.hpp>
#include "rosbag2_cpp/writers/sequential_writer.hpp"

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
        //, annotated_frame_creator_(std::map<std::string, std::string>())
    {
        timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&RosbagRecorder::init, this));
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
    
    rclcpp::TimerBase::SharedPtr timer_;
    rosbag2_cpp::Writer writer_;

    rosbag2_storage::StorageOptions storage_options_;
    rosbag2_storage::TopicMetadata image_topic_info_;
    rosbag2_storage::TopicMetadata tracking_state_topic_info_;
    rosbag2_storage::TopicMetadata detection_topic_info_;
    rosbag2_storage::TopicMetadata trajectory_topic_info_;
    rosbag2_storage::TopicMetadata prediction_topic_info_;

    rclcpp::Serialization<sensor_msgs::msg::Image> image_serialization_;
    rclcpp::Serialization<bob_interfaces::msg::TrackingState> trackingState_serialization_;
    rclcpp::Serialization<bob_interfaces::msg::TrackDetectionArray> trackDetection_serialization_;
    rclcpp::Serialization<bob_interfaces::msg::TrackTrajectoryArray> trackTrajectory_serialization_;
    rclcpp::Serialization<bob_interfaces::msg::TrackTrajectoryArray> trackPrediction_serialization_;

    rclcpp::SerializedMessage image_serialized_msg_;
    rclcpp::SerializedMessage trackingState_serialized_msg_;
    rclcpp::SerializedMessage trackDetection_serialized_msg_;
    rclcpp::SerializedMessage trackTrajectory_serialized_msg_;
    rclcpp::SerializedMessage trackPrediction_serialized_msg_;

    std::shared_ptr<rosbag2_storage::SerializedBagMessage> image_serialized_bag_msg_;
    std::shared_ptr<rosbag2_storage::SerializedBagMessage> trackingState_serialized_bag_msg_;
    std::shared_ptr<rosbag2_storage::SerializedBagMessage> trackDetection_serialized_bag_msg_;
    std::shared_ptr<rosbag2_storage::SerializedBagMessage> trackTrajectory_serialized_bag_msg_;
    std::shared_ptr<rosbag2_storage::SerializedBagMessage> trackPrediction_serialized_bag_msg_;

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

        // MikeG: Trying to follow this as an example
        //https://github.com/ros2/rosbag2/blob/master/rosbag2_tests/test/rosbag2_tests/test_rosbag2_cpp_api.cpp

        //db_uri = f'recordings/rosbags/{datetime.now().strftime("%Y_%m_%d-%H_%M_%S")}'
        //auto db_uri = rcpputils::fs::path("recordings/rosbags/" + std::put_time(rclcpp::Clock().now(), "%d-%m-%Y %H-%M-%S"));
        //auto db_uri = rcpputils::fs::path("recordings/rosbags/" + rclcpp::Clock().now().nanoseconds());
        // MikeG: Get a string representation of the time
        auto db_uri = rcpputils::fs::path("recordings/rosbags/test");

        // MikeG: This is just here while I try and get this thing to work
        // in case the bag was previously not cleaned up

        image_serialized_bag_msg_ = std::make_shared<rosbag2_storage::SerializedBagMessage>();
        trackingState_serialized_bag_msg_ = std::make_shared<rosbag2_storage::SerializedBagMessage>();
        trackDetection_serialized_bag_msg_ = std::make_shared<rosbag2_storage::SerializedBagMessage>();
        trackTrajectory_serialized_bag_msg_ = std::make_shared<rosbag2_storage::SerializedBagMessage>();
        trackPrediction_serialized_bag_msg_ = std::make_shared<rosbag2_storage::SerializedBagMessage>();

        rcpputils::fs::remove_all(db_uri);

        storage_options_.storage_id = "sqlite3";
        storage_options_.uri = db_uri.string();
        writer_.open(storage_options_);

        image_topic_info_.name = "bob/camera/all_sky/bayer";
        image_topic_info_.type = "sensor_msgs/msg/Image";
        image_topic_info_.serialization_format = "cdr";
        writer_.create_topic(image_topic_info_);
        image_serialized_bag_msg_->topic_name = image_topic_info_.name;

        tracking_state_topic_info_.name = "bob/tracker/tracking_state";
        tracking_state_topic_info_.type = "bob_interfaces/msg/TrackingState";
        tracking_state_topic_info_.serialization_format = "cdr";
        writer_.create_topic(tracking_state_topic_info_);
        trackingState_serialized_bag_msg_->topic_name = tracking_state_topic_info_.name;

        detection_topic_info_.name = "bob/tracker/detections";
        detection_topic_info_.type = "bob_interfaces/msg/TrackDetectionArray";
        detection_topic_info_.serialization_format = "cdr";
        writer_.create_topic(detection_topic_info_);
        trackDetection_serialized_bag_msg_->topic_name = detection_topic_info_.name;

        trajectory_topic_info_.name = "bob/tracker/trajectory";
        trajectory_topic_info_.type = "bob_interfaces/msg/TrackTrajectoryArray";
        trajectory_topic_info_.serialization_format = "cdr";
        writer_.create_topic(trajectory_topic_info_);
        trackTrajectory_serialized_bag_msg_->topic_name = trajectory_topic_info_.name;

        prediction_topic_info_.name = "bob/tracker/prediction";
        prediction_topic_info_.type = "bob_interfaces/msg/TrackTrajectoryArray";
        prediction_topic_info_.serialization_format = "cdr";
        writer_.create_topic(prediction_topic_info_);
        trackPrediction_serialized_bag_msg_->topic_name = prediction_topic_info_.name;

        sub_masked_frame_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(shared_from_this(), "bob/camera/all_sky/bayer", rmw_qos_profile);
        sub_tracking_state_ = std::make_shared<message_filters::Subscriber<bob_interfaces::msg::TrackingState>>(shared_from_this(), "bob/tracker/tracking_state", rmw_qos_profile);
        sub_tracker_detections_ = std::make_shared<message_filters::Subscriber<bob_interfaces::msg::TrackDetectionArray>>(shared_from_this(), "bob/tracker/detections", rmw_qos_profile);
        sub_tracker_trajectory_ = std::make_shared<message_filters::Subscriber<bob_interfaces::msg::TrackTrajectoryArray>>(shared_from_this(), "bob/tracker/trajectory", rmw_qos_profile);
        sub_tracker_prediction_ = std::make_shared<message_filters::Subscriber<bob_interfaces::msg::TrackTrajectoryArray>>(shared_from_this(), "bob/tracker/prediction", rmw_qos_profile);

        time_synchronizer_ = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, bob_interfaces::msg::TrackingState, bob_interfaces::msg::TrackDetectionArray, 
            bob_interfaces::msg::TrackTrajectoryArray, bob_interfaces::msg::TrackTrajectoryArray>>(*sub_masked_frame_, *sub_tracking_state_, *sub_tracker_detections_, *sub_tracker_trajectory_, *sub_tracker_prediction_, 10);
        time_synchronizer_->registerCallback(&RosbagRecorder::callback, this);
    }

    void callback(const sensor_msgs::msg::Image::SharedPtr& image_msg
                , const bob_interfaces::msg::TrackingState::SharedPtr& tracking_state_msg
                , const bob_interfaces::msg::TrackDetectionArray::SharedPtr& detections_msg
                , const bob_interfaces::msg::TrackTrajectoryArray::SharedPtr& trajectory_msg
                , const bob_interfaces::msg::TrackTrajectoryArray::SharedPtr& prediction_msg)
    {
        try
        {
            if(tracking_state_msg->trackable > 0) 
            {
                //RCLCPP_INFO(get_logger(), "Recorder Logic goes here --- Recording.....");

                auto header_time = message_filters::message_traits::TimeStamp<sensor_msgs::msg::Image>::value(*image_msg);

                if (false)
                {
                    // Write as is, let writer decide
                    writer_.write(*image_msg, image_topic_info_.name, header_time);
                    writer_.write(*tracking_state_msg, tracking_state_topic_info_.name, header_time);
                    writer_.write(*detections_msg, detection_topic_info_.name, header_time);
                    writer_.write(*trajectory_msg, trajectory_topic_info_.name, header_time);
                    writer_.write(*prediction_msg, prediction_topic_info_.name, header_time);
                }
                else
                {
                    // Do the serialisation
                    image_serialization_.serialize_message(image_msg.get(), &image_serialized_msg_);
                    trackingState_serialization_.serialize_message(tracking_state_msg.get(), &trackingState_serialized_msg_);
                    trackDetection_serialization_.serialize_message(detections_msg.get(), &trackDetection_serialized_msg_);
                    trackTrajectory_serialization_.serialize_message(trajectory_msg.get(), &trackTrajectory_serialized_msg_);
                    trackPrediction_serialization_.serialize_message(prediction_msg.get(), &trackPrediction_serialized_msg_);

                    image_serialized_bag_msg_->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(&image_serialized_msg_.get_rcl_serialized_message(), [](rcutils_uint8_array_t * /* data */) {});
                    trackingState_serialized_bag_msg_->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(&trackingState_serialized_msg_.get_rcl_serialized_message(), [](rcutils_uint8_array_t * /* data */) {});
                    trackDetection_serialized_bag_msg_->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(&trackDetection_serialized_msg_.get_rcl_serialized_message(), [](rcutils_uint8_array_t * /* data */) {});
                    trackTrajectory_serialized_bag_msg_->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(&trackTrajectory_serialized_msg_.get_rcl_serialized_message(), [](rcutils_uint8_array_t * /* data */) {});
                    trackPrediction_serialized_bag_msg_->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(&trackPrediction_serialized_msg_.get_rcl_serialized_message(), [](rcutils_uint8_array_t * /* data */) {});

                    writer_.write(image_serialized_bag_msg_);
                    writer_.write(trackingState_serialized_bag_msg_);
                    writer_.write(trackDetection_serialized_bag_msg_);
                    writer_.write(trackTrajectory_serialized_bag_msg_);
                    writer_.write(trackPrediction_serialized_bag_msg_);
                }
            }
        }
        catch (std::exception &e)
        {
            RCLCPP_ERROR(get_logger(), "exception: %s", e.what());
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