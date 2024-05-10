#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <bob_interfaces/msg/detector_b_box_array.hpp>
#include <bob_interfaces/msg/tracking.hpp>

#include "sort/include/sort_tracker.h"
#include "parameter_node.hpp"
#include "image_utils.hpp"

#include <visibility_control.h>

class TrackProvider
    : public ParameterNode
{
public:
    COMPOSITION_PUBLIC
    explicit TrackProvider(const rclcpp::NodeOptions & options)
        : ParameterNode("frame_provider_node", options)
        , video_tracker_(get_logger()) 
    {
        one_shot_timer_ = this->create_wall_timer(
            std::chrono::seconds(1), 
            [this]() {
                this->init(); 
                this->one_shot_timer_.reset();  
            }
        );
    }

private:
    rclcpp::Publisher<bob_interfaces::msg::Tracking>::SharedPtr pub_tracker_tracking_;
    rclcpp::Publisher<bob_interfaces::msg::Tracking>::SharedPtr pub_tracker_tracking_resized_;
    rclcpp::Subscription<bob_interfaces::msg::DetectorBBoxArray>::SharedPtr detector_bounding_boxes_subscription_;    
    rclcpp::TimerBase::SharedPtr one_shot_timer_;
    SORT::Tracker video_tracker_;
    int resize_height_;

    friend std::shared_ptr<TrackProvider> std::make_shared<TrackProvider>();

    void init()
    {
        RCLCPP_INFO(get_logger(), "Initializing TrackProvider");

        declare_node_parameters();

        rclcpp::QoS pub_qos_profile{10};
        pub_qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        pub_qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);
        pub_qos_profile.history(rclcpp::HistoryPolicy::KeepLast);
        
        rclcpp::QoS sub_qos_profile{10};
        sub_qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        sub_qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);
        sub_qos_profile.history(rclcpp::HistoryPolicy::KeepLast);

        detector_bounding_boxes_subscription_ = create_subscription<bob_interfaces::msg::DetectorBBoxArray>("bob/detection/allsky/boundingboxes", sub_qos_profile,
            std::bind(&TrackProvider::callback, this, std::placeholders::_1));

        pub_tracker_tracking_ = create_publisher<bob_interfaces::msg::Tracking>("bob/tracker/tracking", pub_qos_profile);
        pub_tracker_tracking_resized_ = create_publisher<bob_interfaces::msg::Tracking>("bob/tracker/tracking/resized", pub_qos_profile);
    }

    void declare_node_parameters() 
    {
        std::vector<ParameterNode::ActionParam> params = {
            ParameterNode::ActionParam(
                rclcpp::Parameter("resize_height", 960), 
                [this](const rclcpp::Parameter& param) {
                    resize_height_ = param.as_int();
                }
            ),        
        };
        add_action_parameters(params);
    }

    void callback(const bob_interfaces::msg::DetectorBBoxArray::SharedPtr bounding_boxes_msg)
    {
        try
        {
            std::vector<cv::Rect> bboxes;
            for (const auto &bbox2D : bounding_boxes_msg->detections)
            {
                bboxes.push_back(cv::Rect(bbox2D.x, bbox2D.y, bbox2D.width, bbox2D.height));
            }

            video_tracker_.update_trackers(bboxes);

            publish_tracking(bounding_boxes_msg->header, bounding_boxes_msg->image_width, bounding_boxes_msg->image_height);
        }
        catch (cv::Exception &cve)
        {
            RCLCPP_ERROR(get_logger(), "Open CV exception: %s", cve.what());
        }        
    }

    inline void publish_tracking(std_msgs::msg::Header &header, int image_width, int image_height)
    {
        bob_interfaces::msg::Tracking tracking_msg;
        tracking_msg.header = header;
        tracking_msg.state.trackable = video_tracker_.get_total_trackable_trackers();
        tracking_msg.state.alive = video_tracker_.get_total_live_trackers();
        tracking_msg.state.started = video_tracker_.get_total_trackers_started();
        tracking_msg.state.ended = video_tracker_.get_total_trackers_finished();

        for (const auto &tracker : video_tracker_.get_live_trackers())
        {
            add_track_detection(tracker, tracking_msg.detections);
            add_trajectory_detection(tracker, tracking_msg.trajectories);
            add_prediction(tracker, tracking_msg.predictions);
        }
        pub_tracker_tracking_->publish(tracking_msg);

        publish_tracking_resized(tracking_msg, image_width, image_height);
    }

    inline void publish_tracking_resized(const bob_interfaces::msg::Tracking & tracking_msg, int image_width, int image_height)
    {
        if (count_subscribers(std::string(pub_tracker_tracking_->get_topic_name()) + "/resized") <= 0)
        {
            return;
        }

        float adjust = (float)resize_height_ / (float)image_height;
        //float adjust_x = (float)image_width * adjust_y;

        bob_interfaces::msg::Tracking tracking_msg_resized(tracking_msg);
        // tracking_msg_resized.header = tracking_msg.header;
        // tracking_msg_resized.state.trackable = tracking_msg.state.trackable;
        // tracking_msg_resized.state.alive = tracking_msg.state.alive;
        // tracking_msg_resized.state.started = tracking_msg.state.started;
        // tracking_msg_resized.state.ended = tracking_msg.state.ended;
        for (auto & detect : tracking_msg_resized.detections)
        {
            detect.bbox.center.position.x *= adjust;
            detect.bbox.center.position.y *= adjust;
            detect.bbox.size_x *= adjust;
            detect.bbox.size_y *= adjust;
        }
        for (auto & trajectories : tracking_msg_resized.trajectories)
        {
            for (auto & trajectory : trajectories.trajectory)
            {
                trajectory.center.x *= adjust;
                trajectory.center.y *= adjust;
            }
        }
        for (auto & predictions : tracking_msg_resized.predictions)
        {
            for (auto & prediction : predictions.trajectory)
            {
                prediction.center.x *= adjust;
                prediction.center.y *= adjust;
            }
        }
        pub_tracker_tracking_resized_->publish(tracking_msg_resized);
    }

    inline void add_track_detection(const auto &tracker, std::vector<bob_interfaces::msg::TrackDetection> &detection_array)
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

        // auto result = tracker.get_ellipse(); 
        // double semi_major_axis = std::get<0>(result);
        // double semi_minor_axis = std::get<1>(result);
        // double orientation = std::get<2>(result);
        // detect_msg.covariance_ellipse_semi_major_axis = semi_major_axis;
        // detect_msg.covariance_ellipse_semi_minor_axis = semi_minor_axis;
        // detect_msg.covariance_ellipse_orientation = orientation;

        detection_array.push_back(detect_msg);
    }

    inline void add_trajectory_detection(const auto &tracker, std::vector<bob_interfaces::msg::TrackTrajectory> &trajectory_array)
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

        trajectory_array.push_back(track_msg);
    }

    inline void add_prediction(const auto &tracker, std::vector<bob_interfaces::msg::TrackTrajectory> &prediction_array)
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

        prediction_array.push_back(track_msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::experimental::executors::EventsExecutor executor;
    executor.add_node(std::make_shared<TrackProvider>(rclcpp::NodeOptions()));
    executor.spin();
    rclcpp::shutdown();
    return 0;
}

RCLCPP_COMPONENTS_REGISTER_NODE(TrackProvider)