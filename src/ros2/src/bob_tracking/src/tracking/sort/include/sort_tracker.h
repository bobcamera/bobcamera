#pragma once

#include <map>
#include <thread>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>

#include "../include/track.h"
#include "../include/munkres.h"
#include "utils.h"
#include "../../base_tracker.hpp"

namespace SORT
{
    class Tracker 
        : public BaseTracker
    {
    public:
        Tracker(const std::map<std::string, std::string>& settings);

        std::vector<std::shared_ptr<BaseTrack>> get_live_trackers() const;
        size_t get_total_trackers_started() const;
        size_t get_total_trackers_finished() const;
        size_t get_total_trackable_trackers() const;
        size_t get_total_live_trackers() const;
        void update_trackers(const std::vector<cv::Rect> &detections, const cv::Mat &/*frame*/);

    private:
        std::map<int, std::shared_ptr<Track>> tracks_;
        size_t total_trackers_started_;  
        size_t total_trackers_finished_;
        
        size_t tracker_max_active_trackers_ = 50;

        /**
         * Assigns detections to tracked object (both represented as bounding boxes)
         * Returns 2 lists of matches, unmatched_detections
         * @param detection
         * @param tracks
         * @param matched
         * @param unmatched_det
         * @param iou_threshold
         */
        static void associate_detections_to_trackers(const std::vector<cv::Rect>& detection,
                                        std::map<int, std::shared_ptr<Track>>& tracks,
                                        std::map<int, cv::Rect>& matched,
                                        std::vector<cv::Rect>& unmatched_det,
                                        float iou_threshold = 0.001);

        static float calculate_iou(const cv::Rect& rect1, const cv::Rect& rect2);

        static void hungarian_matching(const std::vector<std::vector<float>>& iou_matrix,
                                    size_t nrows, size_t ncols,
                                    std::vector<std::vector<float>>& association);
    };
}