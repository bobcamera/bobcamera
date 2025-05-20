#pragma once

#include <opencv2/opencv.hpp>
#include <chrono>

#include <boblib/api/utils/pubsub/TopicManager.hpp>
#include <boblib/api/utils/profiler.hpp>
#include <boblib/api/utils/fps_tracker.hpp>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <bob_interfaces/msg/detector_b_box_array.hpp>
#include <bob_interfaces/msg/tracking.hpp>

#include "sort/include/sort_tracker.h"
#include "parameter_node.hpp"
#include "detection.hpp"
#include "tracking.hpp"
#include "image_utils.hpp"
#include "camera_bgs_params.hpp"

class TrackProviderWorker final
{
public:
    explicit TrackProviderWorker(ParameterNode &node
                                 , CameraBgsParams &params
                                 , const rclcpp::QoS &pub_qos_profile
                                 , boblib::utils::pubsub::TopicManager &topic_manager
                                 , boblib::utils::Profiler &profiler)
        : node_(node)
        , params_(params)
        , pub_qos_profile_(pub_qos_profile)
        , topic_manager_(topic_manager)
        , profiler_(profiler)
        , video_tracker_(node.get_logger())
    {
    }

    void init()
    {
        prof_tracker_id_ = profiler_.add_region("Tracker Worker");
        prof_update_id_ = profiler_.add_region("Update", prof_tracker_id_);
        prof_publish_id_ = profiler_.add_region("Publish", prof_tracker_id_);

        tracking_pubsub_ptr_ = topic_manager_.get_topic<bob_camera::Tracking>(params_.topics.tracking_publisher_topic);

        detector_pubsub_ptr_ = topic_manager_.get_topic<Detection>(params_.topics.detection_publish_topic);
        detector_pubsub_ptr_->subscribe<TrackProviderWorker, &TrackProviderWorker::callback>(this);

        pub_tracker_tracking_ = node_.create_publisher<bob_interfaces::msg::Tracking>(params_.topics.tracking_publisher_topic, pub_qos_profile_);
        pub_tracker_state_ = node_.create_publisher<bob_interfaces::msg::TrackingState>(params_.topics.tracker_state_publisher_topic, pub_qos_profile_);
    }

private:
    void callback(const std::shared_ptr<Detection> &detection) noexcept
    {
        try
        {
            profiler_.start(prof_update_id_);
            video_tracker_.update_trackers(*detection->bbox_ptr);
            profiler_.stop(prof_update_id_);
            profiler_.start(prof_publish_id_);
            publish_tracking(detection);
            profiler_.stop(prof_publish_id_);
        }
        catch (const std::exception &cve)
        {
            node_.log_send_error("callback: exception: %s", cve.what());
        }
    }

    inline void publish_tracking(const std::shared_ptr<Detection> &detection)
    {
        bob_camera::Tracking tracking(detection->header_ptr, detection->image_ptr, detection->fps);
        tracking.state.trackable = static_cast<int>(video_tracker_.get_total_trackable_trackers());
        tracking.state.alive = static_cast<int>(video_tracker_.get_total_live_trackers());
        tracking.state.started = video_tracker_.get_total_trackers_started();
        tracking.state.ended = video_tracker_.get_total_trackers_finished();
        tracking.detections.reserve(detection->bbox_ptr->size());
        tracking.trajectories.reserve(detection->bbox_ptr->size());
        tracking.predictions.reserve(detection->bbox_ptr->size());
        for (const auto &tracker : video_tracker_.get_tracks())
        {
            add_track_detection(tracker.second, tracking.detections);
            add_trajectory_detection(tracker.second, tracking.trajectories);
            add_prediction(tracker.second, tracking.predictions);
        }
        tracking_pubsub_ptr_->publish(std::move(tracking));

        if (pub_tracker_state_->get_subscription_count() > 0)
        {
            bob_interfaces::msg::TrackingState tracking_msg;
            tracking_msg.header = *detection->header_ptr;
            tracking_msg.trackable = static_cast<int>(video_tracker_.get_total_trackable_trackers());
            tracking_msg.alive = static_cast<int>(video_tracker_.get_total_live_trackers());
            tracking_msg.started = video_tracker_.get_total_trackers_started();
            tracking_msg.ended = video_tracker_.get_total_trackers_finished();

            pub_tracker_state_->publish(tracking_msg);
        }

        if (pub_tracker_tracking_->get_subscription_count() > 0)
        {
            bob_interfaces::msg::Tracking tracking_msg;
            tracking_msg.header = *detection->header_ptr;
            tracking_msg.state.trackable = static_cast<int>(video_tracker_.get_total_trackable_trackers());
            tracking_msg.state.alive = static_cast<int>(video_tracker_.get_total_live_trackers());
            tracking_msg.state.started = video_tracker_.get_total_trackers_started();
            tracking_msg.state.ended = video_tracker_.get_total_trackers_finished();

            const auto &lts = video_tracker_.get_tracks();
            const size_t n = lts.size();
            tracking_msg.detections.reserve(n);
            tracking_msg.trajectories.reserve(n);
            tracking_msg.predictions.reserve(n);
            for (const auto &tracker : lts)
            {
                add_track_detection(tracker.second, tracking_msg.detections);
                add_trajectory_detection(tracker.second, tracking_msg.trajectories);
                add_prediction(tracker.second, tracking_msg.predictions);
            }
            pub_tracker_tracking_->publish(tracking_msg);
        }
    }

    inline void add_track_detection(const auto &tracker, std::vector<bob_interfaces::msg::TrackDetection> &detection_array) const
    {
        auto bbox = tracker.get_bbox();

        auto &detect_msg = detection_array.emplace_back();
        detect_msg.id = tracker.get_id();
        detect_msg.state = (int)tracker.get_tracking_state();
        detect_msg.bbox.x = bbox.x;
        detect_msg.bbox.y = bbox.y;
        detect_msg.bbox.width = bbox.width;
        detect_msg.bbox.height = bbox.height;
    }

    inline void add_track_detection(const auto &tracker, std::vector<bob_camera::TrackDetection> &detections) const
    {
        auto bbox = tracker.get_bbox();

        auto &detect = detections.emplace_back();
        detect.id = tracker.get_id();
        detect.state = (int)tracker.get_tracking_state();
        detect.bbox.x = bbox.x;
        detect.bbox.y = bbox.y;
        detect.bbox.width = bbox.width;
        detect.bbox.height = bbox.height;
    }

    inline void add_trajectory_detection(const auto &tracker, std::vector<bob_interfaces::msg::TrackTrajectory> &trajectory_array) const
    {
        auto &center_points = tracker.get_center_points();

        auto &track_msg = trajectory_array.emplace_back();
        track_msg.id = std::to_string(tracker.get_id()) + std::string("-") + std::to_string(tracker.get_tracking_state());
        track_msg.trajectory.reserve(center_points.size());
        for (const auto &center_point : center_points)
        {
            bob_interfaces::msg::TrackPoint &point = track_msg.trajectory.emplace_back();
            point.center.x = center_point.first.x;
            point.center.y = center_point.first.y;
            point.tracking_state = (int)center_point.second;
        }
    }

    inline void add_trajectory_detection(const auto &tracker, std::vector<bob_camera::TrackTrajectory> &trajectory_array) const
    {
        auto &center_points = tracker.get_center_points();

        auto &track_msg = trajectory_array.emplace_back();
        track_msg.id = std::to_string(tracker.get_id()) + std::string("-") + std::to_string(tracker.get_tracking_state());
        track_msg.trajectory.reserve(center_points.size());
        for (const auto &center_point : center_points)
        {
            bob_camera::TrackPoint &point = track_msg.trajectory.emplace_back();
            point.center.x = center_point.first.x;
            point.center.y = center_point.first.y;
            point.tracking_state = (int)center_point.second;
        }
    }

    inline void add_prediction(const auto &tracker, std::vector<bob_interfaces::msg::TrackTrajectory> &prediction_array) const
    {
        auto &predictor_center_points = tracker.get_predictor_center_points();

        auto &track_msg = prediction_array.emplace_back();
        track_msg.id = std::to_string(tracker.get_id()) + std::string("-") + std::to_string(tracker.get_tracking_state());
        track_msg.trajectory.reserve(predictor_center_points.size());
        for (const auto &center_point : predictor_center_points)
        {
            bob_interfaces::msg::TrackPoint point;
            point.center.x = center_point.x;
            point.center.y = center_point.y;
            track_msg.trajectory.push_back(std::move(point));
        }
    }

    inline void add_prediction(const auto &tracker, std::vector<bob_camera::TrackTrajectory> &prediction_array) const
    {
        auto &predictor_center_points = tracker.get_predictor_center_points();

        auto &track_msg = prediction_array.emplace_back();
        track_msg.id = std::to_string(tracker.get_id()) + std::string("-") + std::to_string(tracker.get_tracking_state());
        track_msg.trajectory.reserve(predictor_center_points.size());
        for (const auto &center_point : predictor_center_points)
        {
            bob_camera::TrackPoint &point = track_msg.trajectory.emplace_back();
            point.center.x = center_point.x;
            point.center.y = center_point.y;
        }
    }

    ParameterNode &node_;
    CameraBgsParams &params_;
    const rclcpp::QoS &pub_qos_profile_;
    boblib::utils::pubsub::TopicManager &topic_manager_;
    boblib::utils::Profiler &profiler_;

    SORT::Tracker video_tracker_;

    rclcpp::Publisher<bob_interfaces::msg::Tracking>::SharedPtr pub_tracker_tracking_;
    rclcpp::Publisher<bob_interfaces::msg::TrackingState>::SharedPtr pub_tracker_state_;

    std::shared_ptr<boblib::utils::pubsub::PubSub<bob_camera::Tracking>> tracking_pubsub_ptr_;
    std::shared_ptr<boblib::utils::pubsub::PubSub<Detection>> detector_pubsub_ptr_;

    size_t prof_tracker_id_;
    size_t prof_update_id_;
    size_t prof_publish_id_;
};
