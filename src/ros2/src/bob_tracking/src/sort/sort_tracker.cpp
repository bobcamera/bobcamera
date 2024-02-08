#include "include/sort_tracker.h"


SORT::Tracker::Tracker(rclcpp::Logger logger)
    : logger_(logger), total_trackers_started_(0), total_trackers_finished_(0),
     max_coast_cycles_(25), tracker_max_active_trackers_(100)
{

}

float SORT::Tracker::CalculateIou(const cv::Rect& rect1, const cv::Rect& rect2) 
{
    int xA = std::max(rect1.x, rect2.x);
    int yA = std::max(rect1.y, rect2.y);
    int xB = std::min(rect1.x + rect1.width, rect2.x + rect2.width);
    int yB = std::min(rect1.y + rect1.height, rect2.y + rect2.height);

    int interArea = std::max(0, xB - xA) * std::max(0, yB - yA);
    int boxAArea = rect1.width * rect1.height;
    int boxBArea = rect2.width * rect2.height;

    return (float)interArea / (boxAArea + boxBArea - interArea);
}

// Distance IOU.  Could also try Generalized IOU
float SORT::Tracker::CalculateDiou(const cv::Rect& rect1, const cv::Rect& rect2) 
{
    float iou = CalculateIou(rect1, rect2);

    // Calculate the center distance
    float centerDistance = std::pow((rect1.x + rect1.width / 2.0f) - (rect2.x + rect2.width / 2.0f), 2) +
                           std::pow((rect1.y + rect1.height / 2.0f) - (rect2.y + rect2.height / 2.0f), 2);

    // Calculate the diagonal of the smallest enclosing box
    int enclose_x1 = std::min(rect1.x, rect2.x);
    int enclose_y1 = std::min(rect1.y, rect2.y);
    int enclose_x2 = std::max(rect1.x + rect1.width, rect2.x + rect2.width);
    int enclose_y2 = std::max(rect1.y + rect1.height, rect2.y + rect2.height);
    float encloseDiagonal = std::pow(enclose_x2 - enclose_x1, 2) + std::pow(enclose_y2 - enclose_y1, 2);

    // Calculate DIoU
    float diou = iou - (centerDistance / encloseDiagonal);

    return diou;
}


void SORT::Tracker::HungarianMatching(const std::vector<std::vector<float>>& iou_matrix,
                                size_t nrows, size_t ncols,
                                std::vector<std::vector<float>>& association) 
{

    Matrix<float> matrix(nrows, ncols);
    // Initialize matrix with IOU values
    for (size_t i = 0 ; i < nrows ; i++) {
        for (size_t j = 0 ; j < ncols ; j++) {
            // Multiply by -1 to find max cost
            if (iou_matrix[i][j] != 0) {
                matrix(i, j) = -iou_matrix[i][j];
            }
            else {
                // TODO: figure out why we have to assign value to get correct result
                matrix(i, j) = 1.0f;
            }
        }
    }

    // Apply Kuhn-Munkres algorithm to matrix.
    Munkres<float> m;
    m.solve(matrix);

    for (size_t i = 0 ; i < nrows ; i++) {
        for (size_t j = 0 ; j < ncols ; j++) {
            association[i][j] = matrix(i, j);
        }
    }
}


void SORT::Tracker::AssociateDetectionsToTrackers(const std::vector<cv::Rect>& detection,
                                            std::map<int, Track>& tracks,
                                            std::map<int, cv::Rect>& matched,
                                            std::vector<cv::Rect>& unmatched_det,
                                            float iou_threshold) 
{

    // Set all detection as unmatched if no tracks existing
    if (tracks.empty()) {
        for (const auto& det : detection) {
            unmatched_det.push_back(det);
        }
        return;
    }

    std::vector<std::vector<float>> iou_matrix;
    // resize IOU matrix based on number of detection and tracks
    iou_matrix.resize(detection.size(), std::vector<float>(tracks.size()));

    std::vector<std::vector<float>> association;
    // resize association matrix based on number of detection and tracks
    association.resize(detection.size(), std::vector<float>(tracks.size()));


    // row - detection, column - tracks
    for (size_t i = 0; i < detection.size(); i++) {
        size_t j = 0;
        for (const auto& trk : tracks) {
            iou_matrix[i][j] = CalculateDiou(detection[i], trk.second.get_bbox());
            j++;
        }
    }

    // Find association
    HungarianMatching(iou_matrix, detection.size(), tracks.size(), association);

    for (size_t i = 0; i < detection.size(); i++) {
        bool matched_flag = false;
        size_t j = 0;
        for (const auto& trk : tracks) {
            if (0 == association[i][j]) {
                // Filter out matched with low IOU
                if (iou_matrix[i][j] >= iou_threshold) {
                    matched[trk.first] = detection[i];
                    matched_flag = true;
                }
                // It builds 1 to 1 association, so we can break from here
                break;
            }
            j++;
        }
        // if detection cannot match with any tracks
        if (!matched_flag) {
            unmatched_det.push_back(detection[i]);
        }
    }
}

void SORT::Tracker::update_trackers(const std::vector<cv::Rect> &detections, const cv::Mat &/*frame*/)
{
    /*** Predict internal tracks from previous frame ***/
    std::vector<std::thread> predict_threads;
    for (auto &track : tracks_) 
    {
        predict_threads.emplace_back([&track]() 
        { 
            track.second.predict(); 
        });
    }
    for (auto &t : predict_threads) 
    {
        t.join();
    }

    // Hash-map between track ID and associated detection bounding box
    std::map<int, cv::Rect> matched;

    // vector of unassociated detections
    std::vector<cv::Rect> unmatched_det;
    if (!detections.empty()) 
    {
        AssociateDetectionsToTrackers(detections, tracks_, matched, unmatched_det);
    }

    // Update phase with threading
    std::vector<std::thread> update_threads;
    for (const auto &match : matched) 
    {
        const auto &ID = match.first;
        update_threads.emplace_back([this, &ID, &match]() 
        { 
            tracks_[ID].update(match.second); 
        });
    }
    for (auto &t : update_threads) 
    {
        t.join();
    }

    /*** Create new tracks for unmatched detections ***/
    for (const auto &det : unmatched_det) 
    {
        if (tracks_.size() < tracker_max_active_trackers_) {
            Track tracker(logger_);
            tracker.init(det);
            total_trackers_started_++; 
            tracker.set_id(total_trackers_started_);
            tracks_[total_trackers_started_] = tracker;
        } else {
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
            total_trackers_finished_++; 
        } 
        else 
        {
            it++;
        }
    }
}

const std::vector<Track> SORT::Tracker::get_active_trackers() const 
{
    std::vector<Track> activeTracks;
    for (const auto &pair : tracks_) 
    {
        if (pair.second.is_active()) 
        {  
            activeTracks.push_back(pair.second);
        }
    }
    return activeTracks;
}

const std::vector<Track> SORT::Tracker::get_live_trackers() const
{
    std::vector<Track> live_trackers;
    
    for (const auto& pair : tracks_) {
        live_trackers.push_back(pair.second);
    }

    return live_trackers;
}

std::map<int, Track> SORT::Tracker::GetTracks() const 
{
    return tracks_;
}

size_t SORT::Tracker::get_total_trackable_trackers() const
{
    return std::count_if(tracks_.begin(), tracks_.end(), [](const std::pair<int, Track> &pair)
                  { return pair.second.is_tracking(); });
}
size_t SORT::Tracker::get_total_live_trackers() const
{
    return get_active_trackers().size();
}

int SORT::Tracker::get_total_trackers_started() const
{
    return total_trackers_started_;
}

int SORT::Tracker::get_total_trackers_finished() const
{
    return total_trackers_finished_;
}

void SORT::Tracker::set_max_coast_cycles(size_t max_coast_cycles)
{
    max_coast_cycles_ = max_coast_cycles;
}
