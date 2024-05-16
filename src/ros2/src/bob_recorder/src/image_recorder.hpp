#include <opencv2/opencv.hpp>

#include <array>

struct TrackPoint 
{
    const cv::Point point;
    const double bbox_area;
    const int thickness;
    const int thickness_scaled;

    TrackPoint(const cv::Point & pt, double area) 
        : point(pt)
        , bbox_area(area)
        , thickness(std::min(std::max(1, static_cast<int>(sqrt(bbox_area))), 10))
        , thickness_scaled(std::max(1, static_cast<int>(static_cast<double>(thickness) * 0.40)))
    {
    }
};

class ImageRecorder 
{
public:
    explicit ImageRecorder(int pre_buffer_size) 
        : max_pre_buffer_size_(pre_buffer_size)
    {
        pre_buffer_ptr_ = std::make_unique<std::deque<cv::Mat>>();
        draw_trajectories_enabled_ = true;
    }

    void accumulate_mask(const cv::Mat & fg_mask, const cv::Size & frame_size, int x_offset, int y_offset) 
    {
        x_offset_ = x_offset;
        y_offset_ = y_offset;
        
        if (heatmap_accumulator_.empty()) 
        {
            heatmap_accumulator_ = cv::Mat::zeros(frame_size, fg_mask.type());
        }

        cv::Mat shifted_fg_mask = cv::Mat::zeros(frame_size, fg_mask.type());

        cv::Rect roi_rect(std::max(0, -x_offset), std::max(0, -y_offset), 
                        std::min(frame_size.width - std::max(0, -x_offset), fg_mask.cols),
                        std::min(frame_size.height - std::max(0, -y_offset), fg_mask.rows));

        cv::Rect accumulator_roi_rect(std::max(0, x_offset), std::max(0, y_offset), 
                                    roi_rect.width, roi_rect.height);

        fg_mask(roi_rect).copyTo(shifted_fg_mask(accumulator_roi_rect));
        cv::add(heatmap_accumulator_, shifted_fg_mask, heatmap_accumulator_);
    }

    void reset() 
    {
        heatmap_accumulator_ = cv::Mat();
        track_trajectories_.clear();
    }

    bool write_image(const std::string & full_path)
    {
        draw_trajectories();

        if (!heatmap_accumulator_.empty()) 
        {
            cv::Mat converted_heatmap;
            if (heatmap_accumulator_.channels() == 1) 
            {
                cv::cvtColor(heatmap_accumulator_, converted_heatmap, cv::COLOR_GRAY2BGR);
            } 

            cv::Mat overlay;
            cv::addWeighted(frame_for_drawing_, 0.5, converted_heatmap, 0.5, 0, overlay);
            return cv::imwrite(full_path, overlay);
        }

        return cv::imwrite(full_path, frame_for_drawing_);
    }

    void update_frame_for_drawing(const cv::Mat & img) 
    {
        frame_for_drawing_ = img.clone();
    }

    void add_to_pre_buffer(const cv::Mat& img)
    {
        if (pre_buffer_ptr_->size() >= max_pre_buffer_size_) 
        {
            pre_buffer_ptr_->pop_front();
        }
        pre_buffer_ptr_->push_back(img.clone());
    }

    void accumulate_pre_buffer_images() 
    {
        for (const auto& img : *pre_buffer_ptr_) 
        {
            accumulate_mask(img, img.size(), x_offset_, y_offset_);
        }
        pre_buffer_ptr_->clear();
    }

    void store_trajectory_point(int detection_id, const cv::Point & point, double area)
    {
        track_trajectories_[detection_id].emplace_back(point, area);
    }

    void set_draw_trajectories_enabled(bool enabled) 
    {
        draw_trajectories_enabled_ = enabled;
    }

    bool is_draw_trajectories_enabled() const 
    {
        return draw_trajectories_enabled_;
    }

private:

    void draw_trajectories()
    {
        if (!draw_trajectories_enabled_)
        {
            return;
        }

        for (const auto & [detection_id, track_points] : track_trajectories_)
        {
            const cv::Scalar track_color = get_color_for_track(detection_id); 

            for (size_t i = 1; i < track_points.size(); ++i)
            {
                const cv::Point shifted_start_point = track_points[i - 1].point + cv::Point(x_offset_, y_offset_);
                const cv::Point shifted_end_point = track_points[i].point + cv::Point(x_offset_, y_offset_);
                cv::line(frame_for_drawing_, shifted_start_point, shifted_end_point, track_color, track_points[i].thickness_scaled);

                if (i == 1)
                {
                    // Draw the marker only for the first point of the trajectory
                    cv::drawMarker(frame_for_drawing_, shifted_start_point, track_color, cv::MARKER_DIAMOND, 10, track_points[i - 1].thickness);
                }
            }
        }
    }

    const cv::Scalar & get_color_for_track(int trackID) const
    {
        static const std::array<cv::Scalar, 9> pre_defined_colors = 
        {
            cv::Scalar(255, 0, 0),     // Bright Red
            cv::Scalar(0, 255, 0),     // Lime Green
            cv::Scalar(0, 255, 255),   // Bright Yellow
            cv::Scalar(255, 0, 255),   // Magenta
            cv::Scalar(0, 165, 255),   // Orange
            cv::Scalar(255, 255, 0),   // Bright Cyan
            cv::Scalar(0, 215, 255),   // Gold
            cv::Scalar(238, 130, 238), // Violet
            cv::Scalar(147, 20, 255)   // Deep Pink
        };

        return pre_defined_colors[trackID % pre_defined_colors.size()];
    }

    std::unique_ptr<std::deque<cv::Mat>> pre_buffer_ptr_;
    std::map<int, std::vector<TrackPoint>> track_trajectories_;
    size_t max_pre_buffer_size_;
    cv::Mat heatmap_accumulator_;
    cv::Mat frame_for_drawing_;
    bool draw_trajectories_enabled_;
    int x_offset_;
    int y_offset_;
};
