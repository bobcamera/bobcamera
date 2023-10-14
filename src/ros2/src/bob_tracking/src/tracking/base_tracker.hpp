#pragma once
#ifndef __BASE_TRACKER_HPP__
#define __BASE_TRACKER_HPP__

#include <vector>
#include <thread>
#include <stdexcept>
#include <algorithm>
#include <map>
#include <vector>
#include <queue>

#include <opencv2/opencv.hpp>

#include "tracking_state.hpp"

class BaseTrack
{
public:
    BaseTrack() {}
    virtual ~BaseTrack() {}
    virtual int get_id() const = 0;
    virtual bool is_tracking() const = 0;
    virtual cv::Rect get_bbox() const = 0;
    virtual TrackingStateEnum get_tracking_state() const = 0;
    virtual const std::vector<std::pair<cv::Point, TrackingStateEnum>>& get_center_points() const = 0;
    virtual const std::vector<cv::Point>& get_predictor_center_points() const = 0;
};

class BaseTracker
{
public:
    BaseTracker(const std::map<std::string, std::string> &settings)
        : settings(settings)
    {}

    virtual ~BaseTracker()
    {}

    virtual std::vector<std::shared_ptr<BaseTrack>> get_live_trackers() const = 0;
    virtual size_t get_total_trackers_started() const = 0;
    virtual size_t get_total_trackers_finished() const = 0;
    virtual size_t get_total_live_trackers() const = 0;
    virtual size_t get_total_trackable_trackers() const = 0;
    virtual void update_trackers(const std::vector<cv::Rect> &bboxes, const cv::Mat &frame) = 0;

protected:
    std::map<std::string, std::string> settings;
};

#endif