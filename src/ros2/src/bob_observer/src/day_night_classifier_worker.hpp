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

        cv::Scalar mean_brightness = mask_enabled_
            ? cv::mean(v_channel, detection_mask_)
            : cv::mean(v_channel);

        double num_pixels = mask_enabled_
            ? cv::countNonZero(detection_mask_)
            : static_cast<double>(v_channel.total());

        if (num_pixels == 0)
        {
            node_.log_send_warn("No valid pixels found for brightness estimation.");
            return {result, 0};
        }

        int avg_brightness = static_cast<int>(mean_brightness[0]);
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
    bool mask_enabled_{false};
    cv::Mat detection_mask_;
};
