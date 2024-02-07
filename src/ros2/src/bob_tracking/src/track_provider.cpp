#include <opencv2/opencv.hpp>

#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/bounding_box2_d_array.hpp>
#include "bob_interfaces/msg/tracking.hpp"

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
    using Clock = std::chrono::high_resolution_clock;
    using TimePoint = std::chrono::time_point<Clock>;

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> masked_frame_subscription_;
    std::shared_ptr<message_filters::Subscriber<vision_msgs::msg::BoundingBox2DArray>> detector_bounding_boxes_subscription_;
    std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, vision_msgs::msg::BoundingBox2DArray>> time_synchronizer_;
    rclcpp::Publisher<bob_interfaces::msg::Tracking>::SharedPtr pub_tracker_tracking_;
    rclcpp::TimerBase::SharedPtr one_shot_timer_;
    SORT::Tracker video_tracker_;
    double fps_;

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
        auto rmw_qos_profile = sub_qos_profile.get_rmw_qos_profile();

        masked_frame_subscription_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(shared_from_this(), "bob/camera/all_sky/bayer", rmw_qos_profile);
        detector_bounding_boxes_subscription_ = std::make_shared<message_filters::Subscriber<vision_msgs::msg::BoundingBox2DArray>>(shared_from_this(), "bob/detector/all_sky/bounding_boxes", rmw_qos_profile);

        time_synchronizer_ = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, vision_msgs::msg::BoundingBox2DArray>>(*masked_frame_subscription_, *detector_bounding_boxes_subscription_, 10);
        time_synchronizer_->registerCallback(&TrackProvider::callback, this);

        pub_tracker_tracking_ = create_publisher<bob_interfaces::msg::Tracking>("bob/tracker/tracking", pub_qos_profile);
    }

    void declare_node_parameters() {
        std::vector<ParameterNode::ActionParam> params = {
            ParameterNode::ActionParam(rclcpp::Parameter("video_fps", 30.0), [this](const rclcpp::Parameter& param) {fps_ = param.as_double();}),
        };
        add_action_parameters(params);
    }

    void callback(const sensor_msgs::msg::Image::SharedPtr &image_msg, const vision_msgs::msg::BoundingBox2DArray::SharedPtr &bounding_boxes_msg)
    {
        try
        {
            cv::Mat img;
            std::vector<cv::Rect> bboxes;
            for (const auto &bbox2D : bounding_boxes_msg->boxes)
            {
                bboxes.push_back(cv::Rect(bbox2D.center.position.x - bbox2D.size_x / 2, bbox2D.center.position.y - bbox2D.size_y / 2, bbox2D.size_x, bbox2D.size_y));
            }

            video_tracker_.update_trackers(bboxes, img);

            publish_tracking(image_msg->header);
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

    void publish_tracking(std_msgs::msg::Header &header)
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
    }

    void add_track_detection(const auto &tracker, std::vector<bob_interfaces::msg::TrackDetection> &detection_array)
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

        auto result = tracker.get_ellipse(); 
        double semi_major_axis = std::get<0>(result);
        double semi_minor_axis = std::get<1>(result);
        double orientation = std::get<2>(result);
        detect_msg.covariance_ellipse_semi_major_axis = semi_major_axis;
        detect_msg.covariance_ellipse_semi_minor_axis = semi_minor_axis;
        detect_msg.covariance_ellipse_orientation = orientation;

        detection_array.push_back(detect_msg);
    }

    void add_trajectory_detection(const auto &tracker, std::vector<bob_interfaces::msg::TrackTrajectory> &trajectory_array)
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

    void add_prediction(const auto &tracker, std::vector<bob_interfaces::msg::TrackTrajectory> &prediction_array)
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
    rclcpp::spin(std::make_shared<TrackProvider>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}

RCLCPP_COMPONENTS_REGISTER_NODE(TrackProvider)