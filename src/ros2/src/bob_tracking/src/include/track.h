#pragma once

#include <opencv2/core.hpp>
#include "../include/kalman_filter.h"
#include "../../bob_shared/include/tracking_state.hpp"
#include "../include/utils.h"


class Track {
public:
    Track();
    ~Track() = default;

    void Init(const cv::Rect& bbox);
    void Predict();
    void Update(const cv::Rect& bbox);
    float GetNIS() const;
    bool isActive() const;
    bool is_tracking() const;
    int get_id() const;
    int get_coast_cycles() const;
    void set_id(int x);
    cv::Point get_center() const;
    cv::Rect get_bbox() const;
    const std::vector<std::pair<cv::Point, TrackingStateEnum>>& get_center_points() const;
    const std::vector<cv::Point>& get_predictor_center_points() const;
    TrackingStateEnum get_tracking_state() const;
    bool bbox_overlap(const cv::Rect &r1, const cv::Rect &r2) const;

private:
    Eigen::VectorXd ConvertBboxToObservation(const cv::Rect& bbox) const;
    cv::Rect ConvertStateToBbox(const Eigen::VectorXd &state) const;
    
    SORT::KalmanFilter kf_;
    TrackingStateEnum tracking_state_;

    std::vector<std::pair<cv::Point, TrackingStateEnum>> center_points_;
    std::vector<cv::Point> predictor_center_points_;

    int track_stationary_threshold_; 
    int stationary_track_counter_; 
    int coast_cycles_;
    int hit_streak_;
    cv::Rect last_bbox_; 

    int id;
};