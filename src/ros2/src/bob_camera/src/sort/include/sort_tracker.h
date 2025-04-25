#pragma once

#include <map>
#include <thread>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>

#include "../include/track.h"
#include "../include/munkres.h"

namespace SORT
{
    class Tracker {
    public:
        Tracker(rclcpp::Logger logger);
        ~Tracker() = default;

        [[nodiscard]] static constexpr float CalculateIou(const cv::Rect &rect1, const cv::Rect &rect2);
        [[nodiscard]] static float CalculateDiou(const cv::Rect &rect1, const cv::Rect &rect2);
        static void HungarianMatching(const std::vector<std::vector<float>>& iou_matrix,
                            size_t nrows, size_t ncols,
                            std::vector<std::vector<float>>& association);

    /**
     * Assigns detections to tracked object (both represented as bounding boxes)
     * Returns 2 lists of matches, unmatched_detections
     * @param detection
     * @param tracks
     * @param matched
     * @param unmatched_det
     * @param iou_threshold
     */
        static void AssociateDetectionsToTrackers(const std::vector<cv::Rect>& detection,
                                        const std::map<int, Track>& tracks,
                                        std::map<int, cv::Rect>& matched,
                                        std::vector<cv::Rect>& unmatched_det,
                                        // float iou_threshold = 0.001); // 0 to 1 for IOU
                                        float iou_threshold = -0.9); // -1 to 1 for DIOU

        const std::vector<Track> get_active_trackers() const;
        const std::vector<Track> get_live_trackers() const;
        void update_trackers(const std::vector<cv::Rect> &detections);
        [[nodiscard]] size_t get_total_trackable_trackers() const;
        [[nodiscard]] size_t get_total_live_trackers() const;
        [[nodiscard]] std::map<int, Track> GetTracks() const;
        [[nodiscard]] int get_total_trackers_started() const;
        [[nodiscard]] int get_total_trackers_finished() const;
        void set_max_coast_cycles(size_t max_coast_cycles);

    private:
        rclcpp::Logger logger_;
        std::map<int, Track> tracks_;
        int total_trackers_started_;  
        int total_trackers_finished_;
        int max_coast_cycles_; // base on fps
        size_t tracker_max_active_trackers_;
    };

}
