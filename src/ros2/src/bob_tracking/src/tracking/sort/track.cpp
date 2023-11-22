#include "include/track.h"


Track::Track() 
    : kf_(8, 4),
      tracking_state_(ProvisionaryTarget),
      track_stationary_threshold_(25),
      stationary_track_counter_(0),
      coast_cycles_(0),
      hit_streak_(0),
      min_hits_(2)
{

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
            0, 0, 0, 0, 0, 0, 0, 1;

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

    kf_.Q_ <<
           1, 0, 0, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 0, 0, 0,
            0, 0, 0, 1, 0, 0, 0, 0,
            0, 0, 0, 0, 0.01, 0, 0, 0,
            0, 0, 0, 0, 0, 0.01, 0, 0,
            0, 0, 0, 0, 0, 0, 0.0001, 0,
            0, 0, 0, 0, 0, 0, 0, 0.0001;

    kf_.R_ <<
           1, 0, 0,  0,
            0, 1, 0,  0,
            0, 0, 10, 0,
            0, 0, 0,  10;
}
// Get predicted locations from existing trackers
// dt is time elapsed between the current and previous measurements
void Track::Predict() 
{
    kf_.Predict();
    
    cv::Rect predicted_bbox = get_bbox();
    cv::Point center(predicted_bbox.x + predicted_bbox.width / 2, predicted_bbox.y + predicted_bbox.height / 2);

    // hit streak count will be reset
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

    // center_points_.push_back(std::make_pair(center, tracking_state_));  // THIS NOT NEEDED?
    predictor_center_points_.push_back(center);

    // accumulate coast cycle count
    coast_cycles_++;

    // Add logic to check if the track is stationary.
    // cv::Rect predicted_bbox = get_bbox();
    if (bbox_overlap(last_bbox_, predicted_bbox) == false)
    {
        stationary_track_counter_++;
    }
    else
    {
        stationary_track_counter_ = 0;
    }

    int stationary_scavanage_threshold = (int)std::floor((double)track_stationary_threshold_ * 1.5);

    // Logic to mark track as lost if stationary for too long.
    if (stationary_track_counter_ >= track_stationary_threshold_ && stationary_track_counter_ < stationary_scavanage_threshold)
    {
        tracking_state_ = LostTarget;
    }

    last_bbox_ = predicted_bbox; // Update the last_bbox_
}


// Update matched trackers with assigned detections
void Track::Update(const cv::Rect& bbox) 
{
    coast_cycles_ = 0;
    hit_streak_++;

    if (hit_streak_ >= min_hits_) 
    {
        tracking_state_ = ActiveTarget;
    }

    // observation - center_x, center_y, area, ratio
    Eigen::VectorXd observation = ConvertBboxToObservation(bbox);
    kf_.Update(observation);

    cv::Point center(bbox.x + bbox.width / 2, bbox.y + bbox.height / 2);
    center_points_.push_back(std::make_pair(center, tracking_state_));
    predictor_center_points_.clear();

    // Reset the stationary track counter since the object is seen moving.
    stationary_track_counter_ = 0;
    last_bbox_ = bbox;

}


// Create and initialize new trackers for unmatched detections, with initial bounding box
void Track::Init(const cv::Rect &bbox) {
    kf_.x_.head(4) << ConvertBboxToObservation(bbox);
    hit_streak_++;
}


/**
 * Returns the current bounding box estimate
 * @return
 */
cv::Rect Track::get_bbox() const {
    return ConvertStateToBbox(kf_.x_);
}


float Track::GetNIS() const {
    return kf_.NIS_;
}


/**
 * Takes a bounding box in the form [x, y, width, height] and returns z in the form
 * [x, y, s, r] where x,y is the centre of the box and s is the scale/area and r is
 * the aspect ratio
 *
 * @param bbox
 * @return
 */
Eigen::VectorXd Track::ConvertBboxToObservation(const cv::Rect& bbox) const{
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
cv::Rect Track::ConvertStateToBbox(const Eigen::VectorXd &state) const {
    // state - center_x, center_y, width, height, v_cx, v_cy, v_width, v_height
    auto width = std::max(0, static_cast<int>(state[2]));
    auto height = std::max(0, static_cast<int>(state[3]));
    auto tl_x = static_cast<int>(state[0] - width / 2.0);
    auto tl_y = static_cast<int>(state[1] - height / 2.0);
    cv::Rect rect(cv::Point(tl_x, tl_y), cv::Size(width, height));
    return rect;
}

bool Track::isActive() const {
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

bool Track::bbox_overlap(const cv::Rect &r1, const cv::Rect &r2) const
{
    if ((r1.width == 0 || r1.height == 0 || r2.width == 0 || r2.height == 0) ||
        (r1.x > (r2.x + r2.width) || r2.x > (r1.x + r1.width)) ||
        (r1.y > (r2.y + r2.height) || r2.y > (r1.y + r1.height)))
    {
        return false;
    }
    return true;
}

int Track::get_coast_cycles() const 
{
    return coast_cycles_;
}
