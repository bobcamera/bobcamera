#pragma once

#include <opencv2/core.hpp>
#include "../include/kalman_filter.h"
#include "../../bob_shared/include/tracking_state.hpp"
#include <rclcpp/rclcpp.hpp>

// Define constants for state and observation dimensions
constexpr int TRACK_STATE_DIM = 8; // x, y, w, h, vx, vy, vw, vh
constexpr int TRACK_OBS_DIM = 4;   // x, y, w, h

class Track
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
    explicit Track(rclcpp::Logger logger = rclcpp::get_logger("track_logger"));
    ~Track() = default;

    void init(const cv::Rect &bbox) noexcept;
    void predict() noexcept;
    void update(const cv::Rect &bbox) noexcept;
    [[nodiscard]] float get_nis() const noexcept;
    [[nodiscard]] constexpr bool is_active() const noexcept { return tracking_state_ == ActiveTarget; }
    [[nodiscard]] bool is_tracking() const noexcept;
    [[nodiscard]] constexpr int get_id() const noexcept { return id_; }
    [[nodiscard]] int get_coast_cycles() const noexcept;
    void set_id(int x) noexcept;
    void set_min_hits(int min_hits) noexcept;
    void set_track_stationary_threshold(int thresh) noexcept;
    [[nodiscard]] cv::Point get_center() const noexcept;
    [[nodiscard]] cv::Rect get_bbox() const noexcept;
    [[nodiscard]] const std::vector<std::pair<cv::Point, TrackingStateEnum>> &get_center_points() const noexcept;
    [[nodiscard]] const std::vector<cv::Point> &get_predictor_center_points() const noexcept;
    [[nodiscard]] constexpr TrackingStateEnum get_tracking_state() const noexcept { return tracking_state_; }
    [[nodiscard]] std::tuple<double, double, double> get_ellipse() const noexcept;

private:
    [[nodiscard]] Eigen::Matrix<double, TRACK_OBS_DIM, 1> convert_bbox_to_observation(const cv::Rect &bbox) const noexcept;
    [[nodiscard]] static cv::Rect convert_state_to_bbox(const Eigen::Matrix<double, TRACK_STATE_DIM, 1> &state) noexcept;
    void assignStaticKF() noexcept;

    SORT::KalmanFilter<TRACK_STATE_DIM, TRACK_OBS_DIM> kf_;
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
