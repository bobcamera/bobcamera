#include "include/track.h"

namespace
{
    // Static KalmanFilter matrices initialized once
    static const Eigen::Matrix<double, TRACK_STATE_DIM, TRACK_STATE_DIM> static_KF_F = []
    {
        Eigen::Matrix<double, TRACK_STATE_DIM, TRACK_STATE_DIM> m;
        m << 1, 0, 0, 0, 1, 0, 0, 0,
            0, 1, 0, 0, 0, 1, 0, 0,
            0, 0, 1, 0, 0, 0, 1, 0,
            0, 0, 0, 1, 0, 0, 0, 1,
            0, 0, 0, 0, 1, 0, 0, 0,
            0, 0, 0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 0, 0, 1;
        return m;
    }();
    static const Eigen::Matrix<double, TRACK_STATE_DIM, TRACK_STATE_DIM> static_KF_P = []
    {
        Eigen::Matrix<double, TRACK_STATE_DIM, TRACK_STATE_DIM> m;
        m << 10, 0, 0, 0, 0, 0, 0, 0,
            0, 10, 0, 0, 0, 0, 0, 0,
            0, 0, 10, 0, 0, 0, 0, 0,
            0, 0, 0, 10, 0, 0, 0, 0,
            0, 0, 0, 0, 10000, 0, 0, 0,
            0, 0, 0, 0, 0, 10000, 0, 0,
            0, 0, 0, 0, 0, 0, 10000, 0,
            0, 0, 0, 0, 0, 0, 0, 10000;
        return m;
    }();
    static const Eigen::Matrix<double, TRACK_OBS_DIM, TRACK_STATE_DIM> static_KF_H = []
    {
        Eigen::Matrix<double, TRACK_OBS_DIM, TRACK_STATE_DIM> m;
        m << 1, 0, 0, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 0, 0, 0,
            0, 0, 0, 1, 0, 0, 0, 0;
        return m;
    }();
    static const Eigen::Matrix<double, TRACK_OBS_DIM, TRACK_OBS_DIM> static_KF_R = []
    {
        Eigen::Matrix<double, TRACK_OBS_DIM, TRACK_OBS_DIM> m;
        m << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
        return m;
    }();
} // anonymous namespace

// Helper to assign static Kalman filter matrices
void Track::assignStaticKF() noexcept
{
    kf_.F_ = static_KF_F;
    kf_.P_ = static_KF_P;
    kf_.H_ = static_KF_H;
    kf_.R_ = static_KF_R;
}

Track::Track(int id)
    : id_(id),
      tracking_state_(ProvisionaryTarget),
      track_stationary_threshold_(25),
      stationary_track_counter_(0),
      coast_cycles_(0),
      hit_streak_(0),
      min_hits_(2)
{
    // Estimate typical size, e.g., 100 points. Adjust if needed.
    constexpr size_t estimated_points = 100; 
    center_points_.reserve(estimated_points);
    predictor_center_points_.reserve(estimated_points); 

    assignStaticKF();

    constexpr float fps = 50.0f; // placeholder
    constexpr float delta_k = 1.0f / fps;

    // Override only the process noise Q based on delta_k
    kf_.Q_ << 1, 0, 0, 0, delta_k * 4, 0, 0, 0,
        0, 1, 0, 0, 0, delta_k * 4, 0, 0,
        0, 0, 1, 0, 0, 0, delta_k * 4, 0,
        0, 0, 0, 1, 0, 0, 0, delta_k * 4,
        delta_k * 4, 0, 0, 0, 0.01, 0, 0, 0,
        0, delta_k * 4, 0, 0, 0, 0.01, 0, 0,
        0, 0, delta_k * 4, 0, 0, 0, 0.01, 0,
        0, 0, 0, delta_k * 4, 0, 0, 0, 0.01;
}

// Get predicted locations from existing trackers
void Track::predict() noexcept
{
    kf_.Predict();

    if (coast_cycles_ > 0)
    {
        hit_streak_ = 0;
        tracking_state_ = LostTarget;
    }
    else
    {
        tracking_state_ = hit_streak_ >= min_hits_ ? ActiveTarget : ProvisionaryTarget;
    }

    auto predicted_bbox = get_bbox();
    predictor_center_points_.emplace_back(predicted_bbox.x + predicted_bbox.width / 2, predicted_bbox.y + predicted_bbox.height / 2);
    coast_cycles_++;
}

// Update matched trackers with assigned detections
void Track::update(const cv::Rect &bbox) noexcept
{
    coast_cycles_ = 0;
    hit_streak_++;

    if (hit_streak_ >= min_hits_)
    {
        tracking_state_ = ActiveTarget;
    }

    // observation - center_x, center_y, area, ratio
    Eigen::Matrix<double, TRACK_OBS_DIM, 1> observation = convert_bbox_to_observation(bbox);
    kf_.Update(observation);

    cv::Point center(bbox.x + bbox.width / 2, bbox.y + bbox.height / 2);
    center_points_.emplace_back(center, tracking_state_);
    predictor_center_points_.clear();
}

// Create and initialize new trackers for unmatched detections, with initial bounding box
void Track::init(const cv::Rect &bbox) noexcept
{
    kf_.x_.head<TRACK_OBS_DIM>() = convert_bbox_to_observation(bbox);
    hit_streak_++;
}

/**
 * Returns the current bounding box estimate
 * @return
 */
cv::Rect Track::get_bbox() const noexcept
{
    return convert_state_to_bbox(kf_.x_);
}

float Track::get_nis() const noexcept
{
    return kf_.NIS_;
}

std::tuple<double, double, double> Track::get_ellipse() const noexcept
{
    return kf_.ellipse_;
}

/**
 * Takes a bounding box in the form [x, y, width, height] and returns z in the form
 * [center_x, center_y, width, height] where center_x, center_y is the centre of the box
 *
 * @param bbox
 * @return
 */
Eigen::Matrix<double, TRACK_OBS_DIM, 1> Track::convert_bbox_to_observation(const cv::Rect &bbox) const noexcept
{
    Eigen::Matrix<double, TRACK_OBS_DIM, 1> observation;
    const double width = static_cast<double>(bbox.width);
    const double height = static_cast<double>(bbox.height);
    const double center_x = bbox.x + width * 0.5;
    const double center_y = bbox.y + height * 0.5;
    observation << center_x, center_y, width, height;
    return observation;
}

/**
 * Takes a state vector in the form [center_x, center_y, width, height, v_cx, v_cy, v_width, v_height] 
 * and returns a bounding box in the form [x, y, width, height] where x,y is the top left corner
 *
 * @param state
 * @return
 */
[[nodiscard]] cv::Rect Track::convert_state_to_bbox(const Eigen::Matrix<double, TRACK_STATE_DIM, 1> &state) noexcept
{
    // state - center_x, center_y, width, height, v_cx, v_cy, v_width, v_height
    const auto width = std::max(0, static_cast<int>(state[2]));
    const auto height = std::max(0, static_cast<int>(state[3]));
    const auto tl_x = static_cast<int>(state[0] - width * 0.5);
    const auto tl_y = static_cast<int>(state[1] - height * 0.5);
    return {tl_x, tl_y, width, height};
}

bool Track::is_tracking() const noexcept
{
    return tracking_state_ == TrackingStateEnum::ActiveTarget;
}

void Track::set_id(int x) noexcept
{
    id_ = x;
}

void Track::set_min_hits(int min_hits) noexcept
{
    min_hits_ = min_hits;
}

void Track::set_track_stationary_threshold(int thresh) noexcept
{
    track_stationary_threshold_ = thresh;
}

cv::Point Track::get_center() const noexcept
{
    return center_points_.back().first;
}

const std::vector<std::pair<cv::Point, TrackingStateEnum>> &Track::get_center_points() const noexcept
{
    return center_points_;
}

const std::vector<cv::Point> &Track::get_predictor_center_points() const noexcept
{
    return predictor_center_points_;
}

int Track::get_coast_cycles() const noexcept
{
    return coast_cycles_;
}
