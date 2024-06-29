#pragma once
#ifndef __DAY_NIGHT_CLASSIFIER_WORKER_H__
#define __DAY_NIGHT_CLASSIFIER_WORKER_H__

#include <opencv2/opencv.hpp>
#include <day_night.hpp>

// This classifier is to try and determine if an image is classified as day or night
class DayNightClassifierWorker
{
public:
    std::pair<DayNightEnum, int> estimate(const cv::Mat & frame, int threshold)
    {
        if (mask_enabled_ && (detection_mask_.size() != frame.size()))
        {
            cv::resize(detection_mask_, detection_mask_, frame.size());
        }

        DayNightEnum result = DayNightEnum::Night;

        cv::Mat hsv_frame;
        cv::cvtColor(frame, hsv_frame, cv::COLOR_BGR2HSV);

        double sum_brightness = 0.0;
        double num_pixels = 0.0;
        for (int i = 0; i < hsv_frame.rows; ++i)
        {
            for (int j = 0; j < hsv_frame.cols; ++j)
            {
                if (!mask_enabled_ || detection_mask_.at<uchar>(i, j) > 0)
                {
                    sum_brightness += (&hsv_frame.at<uchar>(i, j))[2];
                    ++num_pixels;
                }
            }
        }

        // // Add up all the pixel values in the V channel
        // cv::Scalar sum_brightness = cv::sum(hsv_frame);

        // // Extract average brightness feature from an HSV image
        // // Find the average Value or brightness of an image
        // int avg_brightness = static_cast<int>(sum_brightness[2] / frame.size().area());
        int avg_brightness = static_cast<int>(sum_brightness / num_pixels);

        if (avg_brightness > threshold)
        {
            // if the average brightness is above the threshold value, we classify it as "day"
            result = DayNightEnum::Day;
        }

        return std::make_pair(result, avg_brightness);
    }

    void set_mask(const cv::Mat & mask)
    {
        detection_mask_ = mask;
        mask_enabled_ = !detection_mask_.empty();
    }    

private:
    bool mask_enabled_;
    cv::Mat detection_mask_;
};

#endif