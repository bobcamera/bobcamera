#pragma once

#include <opencv2/opencv.hpp>

#include <boblib/api/utils/pubsub/TopicManager.hpp>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <bob_interfaces/msg/detector_b_box_array.hpp>
#include <bob_interfaces/msg/tracking.hpp>

#include "sort/include/sort_tracker.h"
#include "parameter_node.hpp"
#include "image_utils.hpp"
#include "track_provider_worker_params.hpp"

#include <visibility_control.h>

class TrackProviderWorker final
{
public:
    COMPOSITION_PUBLIC
    explicit TrackProviderWorker(ParameterNode &node,
                                 TrackProviderWorkerParams &params,
                                 boblib::utils::pubsub::TopicManager &topic_manager)
        : node_(node)
        , params_(params)
        , topic_manager_(topic_manager)
        , video_tracker_(node.get_logger())
    {
    }

    void init()
    {
        rclcpp::QoS pub_qos_profile(4);
        pub_qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        pub_qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);
        pub_qos_profile.history(rclcpp::HistoryPolicy::KeepLast);

        pub_tracker_tracking_ = node_.create_publisher<bob_interfaces::msg::Tracking>(params_.get_tracking_publisher_topic(), pub_qos_profile);
        pub_tracker_tracking_resized_ = node_.create_publisher<bob_interfaces::msg::Tracking>(params_.get_tracking_publisher_topic() + "/resized", pub_qos_profile);

        detector_pubsub_ptr_ = topic_manager_.get_topic<bob_interfaces::msg::DetectorBBoxArray>(params_.get_bounding_boxes_subscription_topic());
        detector_pubsub_ptr_->subscribe<TrackProviderWorker, &TrackProviderWorker::callback>(this);
    }

private:
    void callback(const std::shared_ptr<bob_interfaces::msg::DetectorBBoxArray> &bounding_boxes_msg) noexcept
    {
        try
        {
            std::vector<cv::Rect> bboxes;
            bboxes.reserve(bounding_boxes_msg->detections.size());
            for (const auto &bbox2D : bounding_boxes_msg->detections)
            {
                bboxes.emplace_back(bbox2D.x, bbox2D.y, bbox2D.width, bbox2D.height);
            }

            video_tracker_.update_trackers(bboxes);

            publish_tracking(bounding_boxes_msg->header, bounding_boxes_msg->image_height);
        }
        catch (const std::exception &cve)
        {
            node_.log_send_error("callback: exception: %s", cve.what());
        }
    }

    inline void publish_tracking(const std_msgs::msg::Header &header, int image_height)
    {
        bob_interfaces::msg::Tracking tracking_msg;
        tracking_msg.header = header;
        tracking_msg.state.trackable = static_cast<int>(video_tracker_.get_total_trackable_trackers());
        tracking_msg.state.alive = static_cast<int>(video_tracker_.get_total_live_trackers());
        tracking_msg.state.started = video_tracker_.get_total_trackers_started();
        tracking_msg.state.ended = video_tracker_.get_total_trackers_finished();

        for (const auto &tracker : video_tracker_.get_live_trackers())
        {
            add_track_detection(tracker, tracking_msg.detections);
            add_trajectory_detection(tracker, tracking_msg.trajectories);
            add_prediction(tracker, tracking_msg.predictions);
        }
        pub_tracker_tracking_->publish(tracking_msg);

        publish_tracking_resized(tracking_msg, image_height);
    }

    inline void publish_tracking_resized(const bob_interfaces::msg::Tracking &tracking_msg, int image_height) const
    {
        if ((params_.get_resize_height() <= 0) || node_.count_subscribers(std::string(pub_tracker_tracking_->get_topic_name()) + "/resized") <= 0)
        {
            return;
        }

        const double adjust = (double)params_.get_resize_height() / (double)image_height;

        bob_interfaces::msg::Tracking tracking_msg_resized(tracking_msg);
        for (auto &detect : tracking_msg_resized.detections)
        {
            detect.bbox.center.position.x *= adjust;
            detect.bbox.center.position.y *= adjust;
            detect.bbox.size_x *= adjust;
            detect.bbox.size_y *= adjust;
        }
        for (auto &trajectories : tracking_msg_resized.trajectories)
        {
            for (auto &trajectory : trajectories.trajectory)
            {
                trajectory.center.x *= adjust;
                trajectory.center.y *= adjust;
            }
        }
        for (auto &predictions : tracking_msg_resized.predictions)
        {
            for (auto &prediction : predictions.trajectory)
            {
                prediction.center.x *= adjust;
                prediction.center.y *= adjust;
            }
        }
        pub_tracker_tracking_resized_->publish(tracking_msg_resized);
    }

    inline void add_track_detection(const auto &tracker, std::vector<bob_interfaces::msg::TrackDetection> &detection_array) const
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

        detection_array.push_back(detect_msg);
    }

    inline void add_trajectory_detection(const auto &tracker, std::vector<bob_interfaces::msg::TrackTrajectory> &trajectory_array) const
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

    inline void add_prediction(const auto &tracker, std::vector<bob_interfaces::msg::TrackTrajectory> &prediction_array) const
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

    ParameterNode &node_;
    TrackProviderWorkerParams &params_;
    boblib::utils::pubsub::TopicManager &topic_manager_;
    rclcpp::Publisher<bob_interfaces::msg::Tracking>::SharedPtr pub_tracker_tracking_;
    rclcpp::Publisher<bob_interfaces::msg::Tracking>::SharedPtr pub_tracker_tracking_resized_;
    SORT::Tracker video_tracker_;
    std::shared_ptr<boblib::utils::pubsub::PubSub<bob_interfaces::msg::DetectorBBoxArray>> detector_pubsub_ptr_;
};
