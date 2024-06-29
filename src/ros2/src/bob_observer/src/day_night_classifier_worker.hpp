#pragma once
#ifndef __DAY_NIGHT_CLASSIFIER_WORKER_H__
#define __DAY_NIGHT_CLASSIFIER_WORKER_H__

#include <opencv2/opencv.hpp>
#include <day_night.hpp>

// This classifier is to try and determine if an image is classified as day or night
class DayNightClassifierWorker
{
public:
    static std::pair<DayNightEnum, int> estimate(const cv::Mat & frame, int threshold)
    {
        DayNightEnum result = DayNightEnum::Night;

        cv::Mat hsv_frame;
        cv::cvtColor(frame, hsv_frame, cv::COLOR_BGR2HSV);

        // Add up all the pixel values in the V channel
        cv::Scalar sum_brightness = cv::sum(hsv_frame);

        // Extract average brightness feature from an HSV image
        // Find the average Value or brightness of an image
        int avg_brightness = static_cast<int>(sum_brightness[2] / frame.size().area());

        if (avg_brightness > threshold)
        {
            // if the average brightness is above the threshold value, we classify it as "day"
            result = DayNightEnum::Day;
        }

        return std::make_pair(result, avg_brightness);
    }

private:
};

#endif