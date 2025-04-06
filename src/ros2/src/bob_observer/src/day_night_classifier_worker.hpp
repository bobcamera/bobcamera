#pragma once

#include <opencv2/opencv.hpp>
#include <day_night.hpp>

// This classifier is to try and determine if an image is classified as day or night
class DayNightClassifierWorker
{
public:
    explicit DayNightClassifierWorker(ParameterNode & node)
        : node_(node)
    {
    }

    std::pair<DayNightEnum, int> estimate(const cv::Mat & frame, int threshold)
    {
        if (mask_enabled_ && (detection_mask_.size() != frame.size()))
        {
            cv::resize(detection_mask_, detection_mask_, frame.size());
        }

        DayNightEnum result = DayNightEnum::Night;

        cv::Mat hsv_frame;
        cv::cvtColor(frame, hsv_frame, cv::COLOR_BGR2HSV);

        std::vector<cv::Mat> hsv_channels;
        cv::split(hsv_frame, hsv_channels);
        auto & v_channel = hsv_channels[2];

        double sum_brightness = 0.0;
        double num_pixels = 0.0;

        for (int i = 0; i < v_channel.rows; ++i)
        {
            for (int j = 0; j < v_channel.cols; ++j)
            {
                if (!mask_enabled_ || detection_mask_.at<uchar>(i, j) > 0)
                {
                    sum_brightness += v_channel.at<uchar>(i, j);
                    ++num_pixels;
                }
            }
        }

        if (num_pixels == 0) 
        {
            node_.log_send_warn("No valid pixels found for brightness estimation.");
            return {result, 0};
        }

        int avg_brightness = static_cast<int>(sum_brightness / num_pixels);
        node_.log_debug("Pixels used: %.2f%%, Avg. Brightness: %d", (num_pixels * 100.0) / frame.total(), avg_brightness);

        if (avg_brightness > threshold)
        {
            result = DayNightEnum::Day;
        }

        return {result, avg_brightness};
    }

    void set_mask(const cv::Mat & mask)
    {
        detection_mask_ = mask;
        mask_enabled_ = !detection_mask_.empty();
    }    

private:
    ParameterNode &node_;
    bool mask_enabled_;
    cv::Mat detection_mask_;
};
