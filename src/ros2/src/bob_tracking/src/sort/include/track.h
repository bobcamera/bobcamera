#pragma once

#include <opencv2/core.hpp>
#include "../include/kalman_filter.h"
#include "../../bob_shared/include/tracking_state.hpp"
#include <rclcpp/rclcpp.hpp>

class Track {
public:
Track();
    Track(rclcpp::Logger logger);
    ~Track() = default;

    void init(const cv::Rect& bbox);
    void predict();
    void update(const cv::Rect& bbox);
    float get_nis() const;
    bool is_active() const;
    bool is_tracking() const;
    int get_id() const;
    int get_coast_cycles() const;
    void set_id(int x);
    void set_min_hits(int min_hits);
    void set_track_stationary_threshold(int thresh);
    cv::Point get_center() const;
    cv::Rect get_bbox() const;
    const std::vector<std::pair<cv::Point, TrackingStateEnum>>& get_center_points() const;
    const std::vector<cv::Point>& get_predictor_center_points() const;
    TrackingStateEnum get_tracking_state() const;
    std::tuple<double, double, double> get_ellipse() const;

private:
    Eigen::VectorXd convert_bbox_to_observation(const cv::Rect& bbox) const;
    cv::Rect convert_state_to_bbox(const Eigen::VectorXd &state) const;
    
    SORT::KalmanFilter kf_;
    TrackingStateEnum tracking_state_;
    std::vector<std::pair<cv::Point, TrackingStateEnum>> center_points_;
    std::vector<cv::Point> predictor_center_points_;
    int track_stationary_threshold_;
    int stationary_track_counter_;
    int coast_cycles_;
    int hit_streak_;
    int id_;
    int min_hits_;
    rclcpp::Logger logger_;
    cv::Rect last_bbox_;
};
