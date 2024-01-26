#include "include/track.h"

Track::Track() : kf_(8, 4),
                 tracking_state_(ProvisionaryTarget),
                 track_stationary_threshold_(50),
                 stationary_track_counter_(0),
                 coast_cycles_(0),
                 hit_streak_(0),
                 min_hits_(2),
                 logger_(rclcpp::get_logger("track_logger"))  
{
}

Track::Track(rclcpp::Logger logger) 
    : kf_(8, 4),
      tracking_state_(ProvisionaryTarget),
      track_stationary_threshold_(25),
      stationary_track_counter_(0),
      coast_cycles_(0),
      hit_streak_(0),
      min_hits_(2),
      logger_(logger)
{

    float fps = 50.0f; // placeholder
    float delta_k = 1.0f / fps; 

    /*** Define constant velocity model ***/
    // state - center_x, center_y, width, height, v_cx, v_cy, v_width, v_height
    kf_.F_ <<
           1, 0, 0, 0, 1, 0, 0, 0,
            0, 1, 0, 0, 0, 1, 0, 0,
            0, 0, 1, 0, 0, 0, 1, 0,
            0, 0, 0, 1, 0, 0, 0, 1,
            0, 0, 0, 0, 1, 0, 0, 0,
            0, 0, 0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 0, 0, 1;   // time to intercept scaling

    // Give high uncertainty to the unobservable initial velocities
    kf_.P_ <<
           10, 0, 0, 0, 0, 0, 0, 0,
            0, 10, 0, 0, 0, 0, 0, 0,
            0, 0, 10, 0, 0, 0, 0, 0,
            0, 0, 0, 10, 0, 0, 0, 0,
            0, 0, 0, 0, 10000, 0, 0, 0,
            0, 0, 0, 0, 0, 10000, 0, 0,
            0, 0, 0, 0, 0, 0, 10000, 0,
            0, 0, 0, 0, 0, 0, 0, 10000;


    kf_.H_ <<
           1, 0, 0, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 0, 0, 0,
            0, 0, 0, 1, 0, 0, 0, 0;

    // Represents the uncertainty in the process model
    // Larger Q suggests uncertainty in process model leading the filter to put more weight on new measurements
    kf_.Q_ <<
            1, 0, 0, 0, delta_k*4, 0, 0, 0,
            0, 1, 0, 0, 0, delta_k*4, 0, 0,
            0, 0, 1, 0, 0, 0, delta_k*4, 0,
            0, 0, 0, 1, 0, 0, 0, delta_k*4,
            delta_k*4, 0, 0, 0, 0.01, 0, 0, 0,
            0, delta_k*4, 0, 0, 0, 0.01, 0, 0,
            0, 0, 0, delta_k*4, 0, 0, 0.01, 0,
            0, 0, 0, 0, delta_k*4, 0, 0, 0.01;

    // Represents the uncertainty in the measurements
    // Larger R places less trust in measurements 
    kf_.R_ << 
           1, 0, 0,  0,
            0, 1, 0,  0,
            0, 0, 1, 0,
            0, 0, 0,  1;
}

// Get predicted locations from existing trackers
void Track::predict() 
{
    kf_.Predict();
    cv::Rect predicted_bbox = get_bbox();
    cv::Point center(predicted_bbox.x + predicted_bbox.width / 2, predicted_bbox.y + predicted_bbox.height / 2);

    if (coast_cycles_ > 0) 
    {
        hit_streak_ = 0;
        tracking_state_ = LostTarget;
    } 
    else 
    {
        if (hit_streak_ >= min_hits_) 
        {
            tracking_state_ = ActiveTarget;
        } 
        else 
        {
            tracking_state_ = ProvisionaryTarget;
        }
    }

    predictor_center_points_.push_back(center);
    coast_cycles_++;

    // last_bbox_ = predicted_bbox; // Update the last_bbox_
}


// Update matched trackers with assigned detections
void Track::update(const cv::Rect& bbox) 
{
    coast_cycles_ = 0;
    hit_streak_++;

    if (hit_streak_ >= min_hits_)
    {
        tracking_state_ = ActiveTarget;
    }

    // observation - center_x, center_y, area, ratio
    Eigen::VectorXd observation = convert_bbox_to_observation(bbox);
    kf_.Update(observation);

    cv::Point center(bbox.x + bbox.width / 2, bbox.y + bbox.height / 2);
    center_points_.push_back(std::make_pair(center, tracking_state_));
    predictor_center_points_.clear();

}


// Create and initialize new trackers for unmatched detections, with initial bounding box
void Track::init(const cv::Rect &bbox) {
    kf_.x_.head(4) << convert_bbox_to_observation(bbox);
    hit_streak_++;
}


/**
 * Returns the current bounding box estimate
 * @return
 */
cv::Rect Track::get_bbox() const {
    return convert_state_to_bbox(kf_.x_);
}


float Track::get_nis() const {
    return kf_.NIS_;
}

std::tuple<double, double, double> Track::get_ellipse() const {
    return kf_.ellipse_;
}


/**
 * Takes a bounding box in the form [x, y, width, height] and returns z in the form
 * [x, y, s, r] where x,y is the centre of the box and s is the scale/area and r is
 * the aspect ratio
 *
 * @param bbox
 * @return
 */
Eigen::VectorXd Track::convert_bbox_to_observation(const cv::Rect& bbox) const{
    Eigen::VectorXd observation = Eigen::VectorXd::Zero(4);
    auto width = static_cast<float>(bbox.width);
    auto height = static_cast<float>(bbox.height);
    float center_x = bbox.x + width / 2;
    float center_y = bbox.y + height / 2;
    observation << center_x, center_y, width, height;
    return observation;
}


/**
 * Takes a bounding box in the centre form [x,y,s,r] and returns it in the form
 * [x1,y1,x2,y2] where x1,y1 is the top left and x2,y2 is the bottom right
 *
 * @param state
 * @return
 */
cv::Rect Track::convert_state_to_bbox(const Eigen::VectorXd &state) const {
    // state - center_x, center_y, width, height, v_cx, v_cy, v_width, v_height
    auto width = std::max(0, static_cast<int>(state[2]));
    auto height = std::max(0, static_cast<int>(state[3]));
    auto tl_x = static_cast<int>(state[0] - width / 2.0);
    auto tl_y = static_cast<int>(state[1] - height / 2.0);
    cv::Rect rect(cv::Point(tl_x, tl_y), cv::Size(width, height));
    return rect;
}

bool Track::is_active() const {
    return tracking_state_ == ActiveTarget;
}

TrackingStateEnum Track::get_tracking_state() const {
    return tracking_state_;
}

bool Track::is_tracking() const
{
    return tracking_state_ == TrackingStateEnum::ActiveTarget;
}

int Track::get_id() const
{
    return id_;
}

void Track::set_id(int x) 
{
    id_ = x;
}

void Track::set_min_hits(int min_hits)
{
    min_hits_ = min_hits;
}

void Track::set_track_stationary_threshold(int thresh)
{
    track_stationary_threshold_ = thresh;
}

cv::Point Track::get_center() const 
{
    return center_points_.back().first;
}

const std::vector<std::pair<cv::Point, TrackingStateEnum>>& Track::get_center_points() const 
{
    return center_points_;
}

const std::vector<cv::Point>& Track::get_predictor_center_points() const 
{
    return predictor_center_points_;
}

int Track::get_coast_cycles() const 
{
    return coast_cycles_;
}
