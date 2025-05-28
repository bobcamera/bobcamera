#include "include/sort_tracker.h"
#include <ranges>

SORT::Tracker::Tracker(rclcpp::Logger logger)
    : logger_(logger)
    , total_trackers_started_(0)
    , total_trackers_finished_(0)
    , max_coast_cycles_(50)
    , tracker_max_active_trackers_(100)
{
}

[[nodiscard]] float SORT::Tracker::CalculateDiou(const cv::Rect &rect1, const cv::Rect &rect2) noexcept
{
    // Calculate coordinates only once
    const int right1 = rect1.x + rect1.width;
    const int right2 = rect2.x + rect2.width;
    const int bottom1 = rect1.y + rect1.height;
    const int bottom2 = rect2.y + rect2.height;

    // Calculate intersection area efficiently
    const int xA = std::max(rect1.x, rect2.x);
    const int yA = std::max(rect1.y, rect2.y);
    const int xB = std::min(right1, right2);
    const int yB = std::min(bottom1, bottom2);

    const int interWidth = xB - xA;
    const int interHeight = yB - yA;

    // Fast exit if no overlap
    if (interWidth <= 0 || interHeight <= 0)
    {
        return -1.0f; // Minimum DIoU value when no overlap
    }

    const int interArea = interWidth * interHeight;
    const int boxAArea = rect1.width * rect1.height;
    const int boxBArea = rect2.width * rect2.height;

    // Calculate IoU
    const float iou = static_cast<float>(interArea) / (boxAArea + boxBArea - interArea);

    // Calculate center distance - use squares to avoid sqrt
    const float cx1 = rect1.x + rect1.width * 0.5f;
    const float cy1 = rect1.y + rect1.height * 0.5f;
    const float cx2 = rect2.x + rect2.width * 0.5f;
    const float cy2 = rect2.y + rect2.height * 0.5f;
    const float centerDistanceSquared = (cx1 - cx2) * (cx1 - cx2) + (cy1 - cy2) * (cy1 - cy2);

    // Calculate diagonal of enclosing rectangle - use squares to avoid sqrt
    const int enclose_x1 = std::min(rect1.x, rect2.x);
    const int enclose_y1 = std::min(rect1.y, rect2.y);
    const int enclose_x2 = std::max(right1, right2);
    const int enclose_y2 = std::max(bottom1, bottom2);
    const float encloseDiagonalSquared = static_cast<float>((enclose_x2 - enclose_x1) * (enclose_x2 - enclose_x1) +
                                                            (enclose_y2 - enclose_y1) * (enclose_y2 - enclose_y1));

    // Avoid division by zero
    if (encloseDiagonalSquared < 1e-10f)
    {
        return iou;
    }

    // Calculate DIoU
    return iou - (centerDistanceSquared / encloseDiagonalSquared);
}

void SORT::Tracker::HungarianMatching(
    const std::vector<std::vector<float>> &iou_matrix,
    size_t nrows, size_t ncols,
    std::vector<std::vector<float>> &association) noexcept
{
    // Convert input to Eigen matrix
    // Convert DIoU scores to costs (lower is better for Hungarian algorithm)
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> cost(nrows, ncols);

    // Transform DIoU values (higher is better) to costs (lower is better)
    for (size_t i = 0; i < nrows; i++)
    {
        for (size_t j = 0; j < ncols; j++)
        {
            // Convert from maximization to minimization problem
            // Add 1 to handle negative DIoU values (DIoU ranges from -1 to 1)
            cost(i, j) = -iou_matrix[i][j] + 1.0f; // Now costs range from 0 to 2
        }
    }

    // Apply Hungarian algorithm
    Munkres<float> m;
    m.solve(cost);

    // Convert result back to association vector
    association.resize(nrows);
    for (size_t i = 0; i < nrows; i++)
    {
        association[i].resize(ncols);
        for (size_t j = 0; j < ncols; j++)
        {
            association[i][j] = cost(i, j);
        }
    }
}

void SORT::Tracker::AssociateDetectionsToTrackers(const std::vector<cv::Rect> &detection,
                                                  const std::unordered_map<int, Track> &tracks,
                                                  std::unordered_map<int, cv::Rect> &matched,
                                                  std::vector<cv::Rect> &unmatched_det,
                                                  float iou_threshold) noexcept
{
    // Set all detection as unmatched if no tracks existing
    if (tracks.empty())
    {
        // Reserve space for all detections if there are no tracks
        unmatched_det.reserve(detection.size());
        unmatched_det.insert(unmatched_det.end(), detection.begin(), detection.end());
        return;
    }

    // resize IOU matrix based on number of detection and tracks
    std::vector<std::vector<float>> iou_matrix(detection.size(), std::vector<float>(tracks.size()));
    std::vector<std::vector<float>> association(detection.size(), std::vector<float>(tracks.size()));

    for (size_t i = 0; i < detection.size(); ++i)
    {
        for (size_t j = 0; j < tracks.size(); ++j)
        {
            const auto &track_iter = std::next(tracks.begin(), j);
            iou_matrix[i][j] = CalculateDiou(detection[i], track_iter->second.get_bbox());
        }
    }

    // Find association
    HungarianMatching(iou_matrix, detection.size(), tracks.size(), association);

    for (size_t i = 0; i < detection.size(); ++i)
    {
        bool matched_flag = false;
        size_t j = 0;
        for (const auto &trk : tracks)
        {
            // Check for 0s in the Munkres result (optimal assignments)
            if (association[i][j] == 0)
            {
                // Filter out matched with low IOU
                if (iou_matrix[i][j] >= iou_threshold)
                {
                    matched[trk.first] = detection[i];
                    matched_flag = true;
                }
                // It builds 1 to 1 association, so we can break from here
                break;
            }
            ++j;
        }
        // if detection cannot match with any tracks
        if (!matched_flag)
        {
            // Note: push_back might reallocate if capacity is insufficient.
            // Reserving beforehand helps if the number of unmatched detections can be estimated,
            // but it's less predictable here than the initial case.
            unmatched_det.push_back(detection[i]);
        }
    }
}

void SORT::Tracker::update_trackers(const std::vector<cv::Rect> &detections) noexcept
{
    /*** Predict internal tracks from previous frame ***/
    for (auto &track : tracks_)
    {
        track.second.predict();
    }

    // Hash-map between track ID and associated detection bounding box
    std::unordered_map<int, cv::Rect> matched;

    // vector of unassociated detections
    std::vector<cv::Rect> unmatched_det;
    if (!detections.empty())
    {
        AssociateDetectionsToTrackers(detections, tracks_, matched, unmatched_det);
    }

    // Update phase
    for (const auto &match : matched)
    {
        tracks_.at(match.first).update(match.second);
    }

    /*** Create new tracks for unmatched detections ***/
    for (const auto &det : unmatched_det)
    {
        if (tracks_.size() < tracker_max_active_trackers_)
        {
            ++total_trackers_started_;
            // Use emplace instead of operator[] and assignment
            auto [it, inserted] = tracks_.emplace(total_trackers_started_, Track(total_trackers_started_));
            if (inserted)
            {
                it->second.init(det);
            }
        }
        else
        {
            RCLCPP_WARN(logger_, "Reached max number of trackers: %zu", tracker_max_active_trackers_);
            break;
        }
    }

    /*** Delete lose tracked tracks ***/
    for (auto it = tracks_.begin(); it != tracks_.end();)
    {
        if (it->second.get_coast_cycles() > max_coast_cycles_)
        {
            it = tracks_.erase(it);
            ++total_trackers_finished_;
        }
        else
        {
            ++it;
        }
    }
}

[[nodiscard]] const std::unordered_map<int, Track> &SORT::Tracker::get_tracks() const noexcept
{
    return tracks_;
}

[[nodiscard]] size_t SORT::Tracker::get_total_trackable_trackers() const noexcept
{
    return std::ranges::count_if(tracks_, [](const auto &pair)
                                 { return pair.second.is_tracking(); });
}

[[nodiscard]] size_t SORT::Tracker::get_total_live_trackers() const noexcept
{
    return std::ranges::count_if(tracks_, [](const auto &pair)
                                 { return pair.second.is_active(); });
}

[[nodiscard]] int SORT::Tracker::get_total_trackers_started() const noexcept
{
    return total_trackers_started_;
}

[[nodiscard]] int SORT::Tracker::get_total_trackers_finished() const noexcept
{
    return total_trackers_finished_;
}

void SORT::Tracker::set_max_coast_cycles(size_t max_coast_cycles) noexcept
{
    max_coast_cycles_ = max_coast_cycles;
}
