#pragma once

#include <map>
#include <thread>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>

#include "../include/track.h"
#include "../include/munkres.h"

namespace SORT
{
    class Tracker
    {
    public:
        Tracker(rclcpp::Logger logger);
        ~Tracker() = default;

        [[nodiscard]] static float CalculateDiou(const cv::Rect &rect1, const cv::Rect &rect2) noexcept;
        static void HungarianMatching(const std::vector<std::vector<float>> &iou_matrix,
                                      size_t nrows, size_t ncols,
                                      std::vector<std::vector<float>> &association) noexcept;

        /**
         * Assigns detections to tracked object (both represented as bounding boxes)
         * Returns 2 lists of matches, unmatched_detections
         * @param detection
         * @param tracks
         * @param matched
         * @param unmatched_det
         * @param iou_threshold
         */
        static void AssociateDetectionsToTrackers(const std::vector<cv::Rect> &detection,
                                                  const std::unordered_map<int, Track> &tracks,
                                                  std::unordered_map<int, cv::Rect> &matched,
                                                  std::vector<cv::Rect> &unmatched_det,
                                                  // float iou_threshold = 0.001); // 0 to 1 for IOU
                                                  float iou_threshold = -0.9) noexcept; // -1 to 1 for DIOU

        void update_trackers(const std::vector<cv::Rect> &detections) noexcept;
        [[nodiscard]] size_t get_total_trackable_trackers() const noexcept;
        [[nodiscard]] size_t get_total_live_trackers() const noexcept;
        [[nodiscard]] const std::unordered_map<int, Track> & get_tracks() const noexcept;
        [[nodiscard]] int get_total_trackers_started() const noexcept;
        [[nodiscard]] int get_total_trackers_finished() const noexcept;
        void set_max_coast_cycles(size_t max_coast_cycles) noexcept;

    private:
        rclcpp::Logger logger_;
        std::unordered_map<int, Track> tracks_;
        int total_trackers_started_;
        int total_trackers_finished_;
        int max_coast_cycles_; // base on fps
        size_t tracker_max_active_trackers_;
    };

}
