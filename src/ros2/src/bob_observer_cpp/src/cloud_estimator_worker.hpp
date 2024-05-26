#pragma once
#ifndef __CLOUD_ESTIMATOR_WORKER_H__
#define __CLOUD_ESTIMATOR_WORKER_H__

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <vector>
#include <algorithm>

class CloudEstimatorWorker 
{
public:
    enum class DayNightEnum
    {
        Unknown = 0,
        Day = 1,
        Night = 2
    };

    explicit CloudEstimatorWorker() = default;

    virtual ~CloudEstimatorWorker() = default;

    virtual std::pair<double, bool> estimate(cv::Mat& frame) = 0;

protected:
    double find_threshold_mce(const cv::Mat& img, const cv::Mat& mask) const
    {
        std::vector<float> StArray;
        for (int i = 0; i < mask.rows; ++i) 
        {
            for (int j = 0; j < mask.cols; ++j) 
            {
                if (mask.at<uchar>(i, j) != 0) 
                {
                    StArray.push_back(img.at<float>(i, j));
                }
            }
        }

        int bins = 201;
        std::vector<float> y(bins, 0.0f);
        std::vector<float> x(bins);
        for (int i = 0; i < bins; ++i) 
        {
            x[i] = -1.0f + static_cast<float>(i) * (2.0f / (static_cast<float>(bins) - 1.0f));
        }

        for (const auto& val : StArray) 
        {
            auto bin_idx = static_cast<int>((val + 1.0f) * (static_cast<float>(bins) - 1.0f) / 2.0f);
            bin_idx = std::clamp(bin_idx, 0, bins - 1);
            ++y[bin_idx];
        }

        float MinValue = *std::ranges::min_element(StArray);
        float MaxValue = *std::ranges::max_element(StArray);

        float t_int_decimal = std::midpoint(MinValue, MaxValue);
        float t_int = std::ceil(t_int_decimal * 100) / 100;
        auto index_of_t_int = std::ranges::min_element(x, 
            [&](float a, float b) 
            {
                return std::abs(a - t_int) < std::abs(b - t_int);
            }) - x.begin();

        float m0a = 0;
        float m1a = 0;
        float m0b = 0;
        float m1b = 0;
        for (long i = 0; i < index_of_t_int; ++i) 
        {
            m0a += y[i];
            m1a += x[i] * y[i];
        }
        for (long i = index_of_t_int; i < bins - 1; ++i) 
        {
            m0b += y[i];
            m1b += x[i] * y[i];
        }

        float mu_a = m1a / m0a;
        float mu_b = m1b / m0b;
        mu_a = std::abs(mu_a);

        float diff = 5.0f;
        float t_n = 0.0f;
        float t_n_decimal = (mu_b - mu_a) / (std::log(mu_b) - std::log(mu_a));
        if (!std::isnan(t_n_decimal)) 
        {
            t_n = std::ceil(t_n_decimal * 100.0f) / 100.0f;
        }

        int iter = 1;
        while (true) 
        {
            t_int = t_n;
            index_of_t_int = std::ranges::min_element(x, [&](float a, float b) {
                    return std::abs(a - t_int) < std::abs(b - t_int);
                }) - x.begin();

            m0a = m1a = m0b = m1b = 0;
            for (long i = 0; i < index_of_t_int; ++i) 
            {
                m0a += y[i];
                m1a += x[i] * y[i];
            }
            for (long i = index_of_t_int; i < bins - 1; ++i) 
            {
                m0b += y[i];
                m1b += x[i] * y[i];
            }

            mu_a = m1a / m0a;
            mu_b = m1b / m0b;
            mu_a = std::abs(mu_a);

            float t_nplus1_decimal = (mu_b - mu_a) / (std::log(mu_b) - std::log(mu_a));
            if (!std::isnan(t_nplus1_decimal)) 
            {
                float t_nplus1 = std::ceil(t_nplus1_decimal * 100) / 100;
                diff = std::abs(t_nplus1 - t_n);
                t_n = t_nplus1;
                if (diff == 0) 
                {
                    break;
                }
                iter++;
            }
        }

        return t_n;
    }

    cv::Mat get_mask(const cv::Mat& frame)  const
    {
        cv::Mat gray_frame;
        cv::Mat mask;
        cv::cvtColor(frame, gray_frame, cv::COLOR_BGR2GRAY);
        cv::threshold(gray_frame, mask, 1, 255, cv::THRESH_BINARY);
        return mask;
    }
};

class DayTimeCloudEstimator 
    : public CloudEstimatorWorker 
{
public:
    DayTimeCloudEstimator() 
        : CloudEstimatorWorker() 
    {}

    ~DayTimeCloudEstimator() override = default;

    std::pair<double, bool> estimate(cv::Mat& frame) override 
    {
        auto mask = get_mask(frame);

        std::vector<cv::Mat> bgr_channels;
        cv::split(frame, bgr_channels);
        cv::Mat b = bgr_channels[0];
        cv::Mat r = bgr_channels[2];
        r.setTo(1, r == 0);

        cv::Mat lambda_n = (b - r) / (b + r);
        lambda_n.setTo(0, mask == 0);

        cv::Scalar mean;
        cv::Scalar stddev;
        cv::meanStdDev(lambda_n, mean, stddev, mask);
        double std = stddev[0];

        if (std > 0.03) 
        {
            double threshold = find_threshold_mce(lambda_n, mask);
            cv::Mat ratio_mask;
            cv::threshold(lambda_n, ratio_mask, threshold, 255, cv::THRESH_BINARY);
            int N_Cloud = cv::countNonZero(ratio_mask == 0);
            int N_Sky = cv::countNonZero(ratio_mask == 255);
            double ccr = (N_Cloud / static_cast<double>(N_Cloud + N_Sky)) * 100;
            return {round(ccr * 100.0) / 100.0, false};
        } 
        else 
        {
            cv::Mat ratio_mask;
            cv::threshold(b - r, ratio_mask, 30, 255, cv::THRESH_BINARY);
            int N_Cloud = cv::countNonZero(ratio_mask == 0);
            int N_Sky = cv::countNonZero(ratio_mask == 255);
            double ccr = (N_Cloud / static_cast<double>(N_Cloud + N_Sky)) * 100;
            return {round(ccr * 100.0) / 100.0, true};
        }
    }
};

class NightTimeCloudEstimator 
    : public CloudEstimatorWorker 
{
public:
    NightTimeCloudEstimator() 
        : CloudEstimatorWorker() 
    {}

    ~NightTimeCloudEstimator() override = default;

    cv::Mat generate_cloud_feature_image(const cv::Mat& MI) const
    {
        cv::Mat R = 1.164 * (MI - 16) + 1.596 * (MI - 128);
        cv::Mat G = 1.164 * (MI - 16) - 0.392 * (MI - 128) - 0.813 * (MI - 128);
        cv::Mat B = 1.164 * (MI - 16) + 2.017 * (MI - 128);

        cv::Mat max_val;
        cv::Mat min_val;
        cv::max(R, G, max_val);
        cv::max(max_val, B, max_val);
        cv::min(R, G, min_val);
        cv::min(min_val, B, min_val);

        cv::Mat cloud_feature_image = (max_val - min_val) / max_val;
        cv::Mat cloud_feature_image_uint8;
        cloud_feature_image.convertTo(cloud_feature_image_uint8, CV_8U, 255.0);

        return cloud_feature_image_uint8;
    }

    std::pair<double, bool> estimate(cv::Mat& frame) override 
    {
        auto mask = get_mask(frame);
        cv::Mat frame_gray;
        cv::cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);

        cv::Mat cloud_feature_image = generate_cloud_feature_image(frame_gray);
        cloud_feature_image.setTo(0, mask == 0);

        //double otsu_threshold = cv::threshold(cloud_feature_image, cloud_feature_image, 0, 255, cv::THRESH_BINARY + cv::THRESH_OTSU);
        int N_Cloud = cv::countNonZero(cloud_feature_image == 255);
        int N_Sky = cv::countNonZero(cloud_feature_image == 0);

        double ccr = (N_Cloud / static_cast<double>(N_Cloud + N_Sky)) * 100;
        return {round(ccr * 100.0) / 100.0, true};
    }
};

#endif