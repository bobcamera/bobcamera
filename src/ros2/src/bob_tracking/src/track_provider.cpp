#include <opencv2/opencv.hpp>

#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/bounding_box2_d_array.hpp>
#include "bob_interfaces/msg/track_detection_array.hpp"
#include "bob_interfaces/msg/tracking_state.hpp"
#include "bob_interfaces/msg/track_trajectory_array.hpp"

#include "boblib/api/utils/profiler.hpp"
#include "video_tracker.hpp"

#include "parameter_node.hpp"
#include "image_utils.hpp"

class TrackProvider
    : public ParameterNode
{
public:
    static std::shared_ptr<TrackProvider> create()
    {
        auto result = std::shared_ptr<TrackProvider>(new TrackProvider());
        result->init();
        return result;
    }

private:
    using Clock = std::chrono::high_resolution_clock;
    using TimePoint = std::chrono::time_point<Clock>;

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> masked_frame_subscription_;
    std::shared_ptr<message_filters::Subscriber<vision_msgs::msg::BoundingBox2DArray>> detector_bounding_boxes_subscription_;
    std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, vision_msgs::msg::BoundingBox2DArray>> time_synchronizer_;

    rclcpp::Publisher<bob_interfaces::msg::TrackingState>::SharedPtr pub_tracker_tracking_state;
    rclcpp::Publisher<bob_interfaces::msg::TrackDetectionArray>::SharedPtr pub_tracker_detects;
    rclcpp::Publisher<bob_interfaces::msg::TrackTrajectoryArray>::SharedPtr pub_tracker_trajectory;
    rclcpp::Publisher<bob_interfaces::msg::TrackTrajectoryArray>::SharedPtr pub_tracker_prediction;

    boblib::utils::Profiler profiler_;
    VideoTracker video_tracker_;

    friend std::shared_ptr<TrackProvider> std::make_shared<TrackProvider>();

    TrackProvider()
        : ParameterNode("frame_provider_node")
        , video_tracker_(std::map<std::string, std::string>(), get_logger())
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

        masked_frame_subscription_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(shared_from_this(), "bob/camera/all_sky/bayer", rmw_qos_profile);
        detector_bounding_boxes_subscription_ = std::make_shared<message_filters::Subscriber<vision_msgs::msg::BoundingBox2DArray>>(shared_from_this(), "bob/detector/all_sky/bounding_boxes", rmw_qos_profile);

        time_synchronizer_ = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, vision_msgs::msg::BoundingBox2DArray>>(*masked_frame_subscription_, *detector_bounding_boxes_subscription_, 2);
        time_synchronizer_->registerCallback(&TrackProvider::callback, this);

        pub_tracker_tracking_state = create_publisher<bob_interfaces::msg::TrackingState>("bob/tracker/tracking_state", pub_qos_profile);
        pub_tracker_detects = create_publisher<bob_interfaces::msg::TrackDetectionArray>("bob/tracker/detections", pub_qos_profile);
        pub_tracker_trajectory = create_publisher<bob_interfaces::msg::TrackTrajectoryArray>("bob/tracker/trajectory", pub_qos_profile);
        pub_tracker_prediction = create_publisher<bob_interfaces::msg::TrackTrajectoryArray>("bob/tracker/prediction", pub_qos_profile);
    }

    void callback(const sensor_msgs::msg::Image::SharedPtr &image_msg, const vision_msgs::msg::BoundingBox2DArray::SharedPtr &bounding_boxes_msg)
    {
        try
        {
            cv::Mat masked_img_bridge;
            ImageUtils::convert_image_msg(image_msg, masked_img_bridge);

            std::vector<cv::Rect> bboxes;
            for (const auto &bbox2D : bounding_boxes_msg->boxes)
            {
                bboxes.push_back(cv::Rect(bbox2D.center.position.x - bbox2D.size_x / 2, bbox2D.center.position.y - bbox2D.size_y / 2, bbox2D.size_x, bbox2D.size_y));
            }

            video_tracker_.update_trackers(bboxes, masked_img_bridge);

            publish_detect_array(image_msg->header);
            publish_trajectory_array(image_msg->header);
            publish_prediction_array(image_msg->header);
            publish_tracking_state(image_msg->header);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(get_logger(), "CV bridge exception: %s", e.what());
        }
    }

    void publish_detect_array(std_msgs::msg::Header &header)
    {
        bob_interfaces::msg::TrackDetectionArray detection_array_msg;
        detection_array_msg.header = header;
        for (const auto &tracker : video_tracker_.get_live_trackers())
        {
            add_detects_to_msg(tracker, detection_array_msg);
        }
        pub_tracker_detects->publish(detection_array_msg);
    }

    void publish_trajectory_array(std_msgs::msg::Header &header)
    {
        bob_interfaces::msg::TrackTrajectoryArray trajectory_array_msg;
        trajectory_array_msg.header = header;
        for (const auto &tracker : video_tracker_.get_live_trackers())
        {
            add_trajectories_to_msg(tracker, trajectory_array_msg);
        }
        pub_tracker_trajectory->publish(trajectory_array_msg);
    }

    void publish_prediction_array(std_msgs::msg::Header &header)
    {
        bob_interfaces::msg::TrackTrajectoryArray prediction_array_msg;
        prediction_array_msg.header = header;
        for (const auto &tracker : video_tracker_.get_live_trackers())
        {
            add_predictions_to_msg(tracker, prediction_array_msg);
        }
        pub_tracker_prediction->publish(prediction_array_msg);
    }

    void publish_tracking_state(std_msgs::msg::Header &header)
    {
        bob_interfaces::msg::TrackingState tracking_state_msg;
        tracking_state_msg.header = header;
        tracking_state_msg.trackable = video_tracker_.get_total_trackable_trackers();
        tracking_state_msg.alive = video_tracker_.get_total_live_trackers();
        tracking_state_msg.started = video_tracker_.get_total_trackers_started();
        tracking_state_msg.ended = video_tracker_.get_total_trackers_finished();
        pub_tracker_tracking_state->publish(tracking_state_msg);
    }

    void add_detects_to_msg(const Tracker &tracker, bob_interfaces::msg::TrackDetectionArray &detection_2d_array_msg)
    {
        auto bbox = tracker.get_bbox();
        vision_msgs::msg::BoundingBox2D bbox_msg;
        bbox_msg.center.position.x = bbox.x + bbox.width / 2;
        bbox_msg.center.position.y = bbox.y + bbox.height / 2;
        bbox_msg.size_x = bbox.width;
        bbox_msg.size_y = bbox.height;

        bob_interfaces::msg::TrackDetection detect_msg;
        detect_msg.id = tracker.get_id();
        detect_msg.state = (int)tracker.get_tracking_state();
        detect_msg.bbox = bbox_msg;

        detection_2d_array_msg.detections.push_back(detect_msg);
    }

    void add_trajectories_to_msg(const Tracker &tracker, bob_interfaces::msg::TrackTrajectoryArray &trajectory_array_msg)
    {
        bob_interfaces::msg::TrackTrajectory track_msg;
        track_msg.id = std::to_string(tracker.get_id()) + std::string("-") + std::to_string(tracker.get_tracking_state());

        for (const auto &center_point : tracker.get_center_points())
        {
            bob_interfaces::msg::TrackPoint point;
            point.center.x = center_point.first.x;
            point.center.y = center_point.first.y;
            point.tracking_state = (int)center_point.second;
            track_msg.trajectory.push_back(point);
        }

        trajectory_array_msg.trajectories.push_back(track_msg);
    }

    void add_predictions_to_msg(const Tracker &tracker, bob_interfaces::msg::TrackTrajectoryArray &prediction_array_msg)
    {
        bob_interfaces::msg::TrackTrajectory track_msg;
        track_msg.id = std::to_string(tracker.get_id()) + std::string("-") + std::to_string(tracker.get_tracking_state());

        for (const auto &center_point : tracker.get_predictor_center_points())
        {
            bob_interfaces::msg::TrackPoint point;
            point.center.x = center_point.x;
            point.center.y = center_point.y;
            track_msg.trajectory.push_back(point);
        }

        prediction_array_msg.trajectories.push_back(track_msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(TrackProvider::create());
    rclcpp::shutdown();
    return 0;
}
