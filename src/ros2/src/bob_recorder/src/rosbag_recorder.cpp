#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <sensor_msgs/msg/image.hpp>
#include "bob_interfaces/msg/tracking.hpp"
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
    std::shared_ptr<message_filters::Subscriber<bob_interfaces::msg::Tracking>> sub_tracking_;
    std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, bob_interfaces::msg::Tracking>> time_synchronizer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_annotated_frame_;
    
    rclcpp::TimerBase::SharedPtr timer_;
    rosbag2_cpp::Writer writer_;

    rosbag2_storage::StorageOptions storage_options_;
    rosbag2_storage::TopicMetadata image_topic_info_;
    rosbag2_storage::TopicMetadata tracking_topic_info_;

    rclcpp::Serialization<sensor_msgs::msg::Image> image_serialization_;
    rclcpp::Serialization<bob_interfaces::msg::Tracking> tracking_serialization_;

    rclcpp::SerializedMessage image_serialized_msg_;
    rclcpp::SerializedMessage tracking_serialized_msg_;

    std::shared_ptr<rosbag2_storage::SerializedBagMessage> image_serialized_bag_msg_;
    std::shared_ptr<rosbag2_storage::SerializedBagMessage> tracking_serialized_bag_msg_;

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
        tracking_serialized_bag_msg_ = std::make_shared<rosbag2_storage::SerializedBagMessage>();

        rcpputils::fs::remove_all(db_uri);

        storage_options_.storage_id = "sqlite3";
        storage_options_.uri = db_uri.string();
        writer_.open(storage_options_);

        image_topic_info_.name = "bob/frames/allsky/original";
        image_topic_info_.type = "sensor_msgs/msg/Image";
        image_topic_info_.serialization_format = "cdr";
        writer_.create_topic(image_topic_info_);
        image_serialized_bag_msg_->topic_name = image_topic_info_.name;

        tracking_topic_info_.name = "bob/tracker/tracking";
        tracking_topic_info_.type = "bob_interfaces/msg/Tracking";
        tracking_topic_info_.serialization_format = "cdr";
        writer_.create_topic(tracking_topic_info_);
        tracking_serialized_bag_msg_->topic_name = tracking_topic_info_.name;

        sub_masked_frame_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(shared_from_this(), "bob/frames/allsky/original", rmw_qos_profile);
        sub_tracking_ = std::make_shared<message_filters::Subscriber<bob_interfaces::msg::Tracking>>(shared_from_this(), "bob/tracker/tracking", rmw_qos_profile);

        time_synchronizer_ = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, bob_interfaces::msg::Tracking>>(*sub_masked_frame_, *sub_tracking_, 10);
        time_synchronizer_->registerCallback(&RosbagRecorder::callback, this);
    }

    void callback(const sensor_msgs::msg::Image::SharedPtr& image_msg
                , const bob_interfaces::msg::Tracking::SharedPtr& tracking_msg
    {
        try
        {
            if(tracking_msg->state.trackable > 0) 
            {
                //RCLCPP_INFO(get_logger(), "Recorder Logic goes here --- Recording.....");

                auto header_time = message_filters::message_traits::TimeStamp<sensor_msgs::msg::Image>::value(*image_msg);

                if (false)
                {
                    // Write as is, let writer decide
                    writer_.write(*image_msg, image_topic_info_.name, header_time);
                    writer_.write(*tracking_msg, tracking_topic_info_.name, header_time);
                }
                else
                {
                    // Do the serialisation
                    image_serialization_.serialize_message(image_msg.get(), &image_serialized_msg_);
                    tracking_serialization_.serialize_message(tracking_msg.get(), &tracking_serialized_msg_);

                    image_serialized_bag_msg_->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(&image_serialized_msg_.get_rcl_serialized_message(), [](rcutils_uint8_array_t * /* data */) {});
                    tracking_serialized_bag_msg_->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(&tracking_serialized_msg_.get_rcl_serialized_message(), [](rcutils_uint8_array_t * /* data */) {});

                    writer_.write(image_serialized_bag_msg_);
                    writer_.write(tracking_serialized_bag_msg_);
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