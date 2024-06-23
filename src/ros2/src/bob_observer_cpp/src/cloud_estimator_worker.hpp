#pragma once
#ifndef __CLOUD_ESTIMATOR_WORKER_H__
#define __CLOUD_ESTIMATOR_WORKER_H__

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <vector>
#include <algorithm>
#include <ranges>

class CloudEstimatorWorker 
{
public:
    enum class DayNightEnum
    {
        Unknown = 0,
        Day = 1,
        Night = 2
    };

    explicit CloudEstimatorWorker(ParameterLifeCycleNode & node)
        : node_(node)
    {
    }

    virtual ~CloudEstimatorWorker() = default;

    virtual std::pair<double, bool> estimate(cv::Mat& frame) = 0;

protected:
    ParameterLifeCycleNode & node_;

    static double find_threshold_mce(const cv::Mat& img, const cv::Mat& mask)
    {
        //cv::Mat img_double;        
        //_img.convertTo(img_double, CV_64F);

        // Extract the pixel values where mask is true
        std::vector<double> StArray;
        for (int i = 0; i < img.rows; ++i) {
            for (int j = 0; j < img.cols; ++j) 
            {
                if (mask.at<uchar>(i, j)) 
                {
                    StArray.push_back(static_cast<double>(img.at<uchar>(i, j)));
                }
            }
        }

        // Generate histogram bins
        std::vector<double> x(201);
        std::iota(x.begin(), x.end(), -1);
        std::ranges::for_each(x, [](double& d) { d = (d + 1) / 100 - 1; });

        std::vector<int> y(200, 0);
        for (const auto& val : StArray) 
        {
            auto it = std::ranges::lower_bound(x, val);
            if (it != x.end() && it != x.begin()) 
            {
                ++y[std::distance(x.begin(), it) - 1];
            }
        }

        double MinValue = *std::ranges::min_element(StArray);
        double MaxValue = *std::ranges::max_element(StArray);

        // Initial threshold estimation
        double t_int_decimal = MinValue + ((MaxValue - MinValue) / 2);
        double t_int = std::ceil(t_int_decimal * 100) / 100;
        auto it_t_int = std::ranges::min_element(x, [t_int](double a, double b) { return std::abs(a - t_int) < std::abs(b - t_int); });
        int index_of_t_int = std::distance(x.begin(), it_t_int);

        // Calculate initial moments
        double m0a = 0, m1a = 0;
        for (int i = 0; i < index_of_t_int; ++i) 
        {
            m0a += y[i];
            m1a += x[i] * y[i];
        }

        double m0b = 0, m1b = 0;
        for (int i = index_of_t_int; i < 200; ++i) 
        {
            m0b += y[i];
            m1b += x[i] * y[i];
        }

        double mu_a = m1a / m0a;
        double mu_b = m1b / m0b;

        if (mu_a < 0) 
        {
            mu_a = std::abs(mu_a);
        }

        double diff = 5;
        double t_n = 0;

        double t_n_decimal = (mu_b - mu_a) / (std::log(mu_b) - std::log(mu_a));
        if (!std::isnan(t_n_decimal)) 
        {
            t_n = std::ceil(t_n_decimal * 100) / 100;
        }

        int iter = 1;
        while (true) 
        {
            t_int = t_n;

            // Finding index of t_int
            it_t_int = std::ranges::min_element(x, [t_int](double a, double b) { return std::abs(a - t_int) < std::abs(b - t_int); });
            index_of_t_int = std::distance(x.begin(), it_t_int);

            // Calculate moments for the new threshold
            m0a = 0, m1a = 0;
            for (int i = 0; i < index_of_t_int; ++i) 
            {
                m0a += y[i];
                m1a += x[i] * y[i];
            }

            m0b = 0, m1b = 0;
            for (int i = index_of_t_int; i < 200; ++i) 
            {
                m0b += y[i];
                m1b += x[i] * y[i];
            }

            mu_a = m1a / m0a;
            mu_b = m1b / m0b;

            if (mu_a < 0) 
            {
                mu_a = std::abs(mu_a);
            }

            double t_nplus1_decimal = (mu_b - mu_a) / (std::log(mu_b) - std::log(mu_a));
            if (!std::isnan(t_nplus1_decimal)) 
            {
                double t_nplus1 = std::ceil(t_nplus1_decimal * 100) / 100;

                diff = std::abs(t_nplus1 - t_n);
                t_n = t_nplus1;

                if (diff == 0) 
                {
                    break;
                }

                ++iter;
            }
        }

        return t_n;
    }

    static cv::Mat get_mask(const cv::Mat& frame)
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
    DayTimeCloudEstimator(ParameterLifeCycleNode & node) 
        : CloudEstimatorWorker(node) 
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
            RCLCPP_INFO(node_.get_logger(), "estimate 5.1");
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
    NightTimeCloudEstimator(ParameterLifeCycleNode & node) 
        : CloudEstimatorWorker(node) 
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