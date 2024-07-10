#pragma once
#ifndef __CLOUD_ESTIMATOR_WORKER_H__
#define __CLOUD_ESTIMATOR_WORKER_H__

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <vector>
#include <algorithm>
#include <ranges>

#include "day_night.hpp"

class CloudEstimatorWorker 
{
public:
    explicit CloudEstimatorWorker(ParameterLifeCycleNode & node)
        : node_(node)
    {
        x_ = linspace(start_, end_, num_bins_);
    }

    virtual ~CloudEstimatorWorker() = default;

    virtual std::pair<double, bool> estimate(const cv::Mat& frame) = 0;

    void set_mask(const cv::Mat & mask)
    {
        mask_enabled_ = !mask.empty();
        detection_mask_ = mask;
    }

protected:
    ParameterLifeCycleNode & node_;
    const double start_ = -1.0;
    const double end_ = 1.0;
    const int num_bins_ = 201;
    std::vector<double> x_;
    bool mask_enabled_;
    cv::Mat detection_mask_;

    static std::vector<double> linspace(double start, double end, int num) 
    {
        std::vector<double> linspace_vector(num);
        const double step = (end - start) / (num - 1);
        
        std::ranges::generate(linspace_vector, [n = 0, start, step]() mutable {
            return start + n++ * step;
        });
        
        return linspace_vector;
    }

    double find_threshold_mce(const cv::Mat & img)
    {
        auto & x = x_;
        std::vector<int> y(num_bins_ - 1, 0);

        const double one_over_bin_width = 1.0 / ((end_ - start_) / static_cast<double>(num_bins_ - 1));
        double min_value = std::numeric_limits<double>::max();
        double max_value = std::numeric_limits<double>::lowest();

        // Iterate through the image and mask, create the histogram and the max and min values
        for (int i = 0; i < img.rows; ++i) 
        {
            for (int j = 0; j < img.cols; ++j) 
            {
                const double pixel_value = img.at<double>(i, j);
                if (pixel_value >= start_ && pixel_value <= end_) 
                {
                    // Calculate the correct bin index
                    const int bin_index = static_cast<int>((pixel_value - start_) * one_over_bin_width);
                    ++y[bin_index];
                }
                min_value = std::min(min_value, pixel_value);
                max_value = std::max(max_value, pixel_value);
            }
        }

        const double t_int_decimal = min_value + ((max_value - min_value) / 2.0);
        double t_int = std::ceil(t_int_decimal * 100.0) / 100.0;

        int index_of_t_int = static_cast<int>(((num_bins_ - 1) / (end_ - start_)) * (t_int - start_));

        // Calculate initial moments
        double m0a = 0.0;
        double m1a = 0.0;
        for (int i = 0; i < index_of_t_int; ++i) 
        {
            m0a += y[i];
            m1a += x[i] * y[i];
        }

        double m0b = 0.0;
        double m1b = 0.0;
        for (int i = index_of_t_int; i < (num_bins_ - 1); ++i) 
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

        double diff = 5.0;
        double t_n = 0.0;

        double t_n_decimal = (mu_b - mu_a) / (std::log(mu_b) - std::log(mu_a));
        if (!std::isnan(t_n_decimal)) 
        {
            t_n = std::ceil(t_n_decimal * 100.0) / 100.0;
        }

        int max_interactions = 1000;
        int iter = 0;
        while (iter < max_interactions) 
        {
            t_int = t_n;

            // Finding index of t_int
            index_of_t_int = static_cast<int>(((num_bins_ - 1) / (end_ - start_)) * (t_int - start_));

            // Calculate moments for the new threshold
            m0a = 0, m1a = 0;
            for (int i = 0; i < index_of_t_int; ++i) 
            {
                m0a += y[i];
                m1a += x[i] * y[i];
            }

            m0b = 0, m1b = 0;
            for (int i = index_of_t_int; i < (num_bins_ - 1); ++i) 
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
                double t_nplus1 = std::ceil(t_nplus1_decimal * 100.0) / 100.0;

                diff = std::abs(t_nplus1 - t_n);
                t_n = t_nplus1;

                if (diff == 0) 
                {
                    break;
                }
                ++iter;
            }
            else
            {
                break;
            }
        }

        return t_n;
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

    std::pair<double, bool> estimate(const cv::Mat & frame) override 
    {
        if (detection_mask_.empty())
        {
            detection_mask_ = cv::Mat::ones(frame.size(), CV_8U);
        }
        if (detection_mask_.size() != frame.size())
        {
            cv::resize(detection_mask_, detection_mask_, frame.size());
        }

        cv::Mat frame_f64;
        frame.convertTo(frame_f64, CV_64F);

        std::vector<cv::Mat> bgr_channels;
        cv::split(frame_f64, bgr_channels);
        cv::Mat b = bgr_channels[0];
        cv::Mat r = bgr_channels[2];
        r.setTo(1, r == 0); 

        node_.log_debug("Original size: %d", b.rows * b.cols);

        // Find non-zero locations in the detection mask
        std::vector<cv::Point> estimationPoints;
        cv::findNonZero(detection_mask_, estimationPoints);

        // Build 1D matrices for b and r using only valid pixels
        cv::Mat b_est(estimationPoints.size(), 1, b.type());
        cv::Mat r_est(estimationPoints.size(), 1, r.type());

        for (size_t i = 0; i < estimationPoints.size(); ++i) 
        {
            b_est.at<double>(i) = b.at<double>(estimationPoints[i].y, estimationPoints[i].x);
            r_est.at<double>(i) = r.at<double>(estimationPoints[i].y, estimationPoints[i].x);
        }

        node_.log_debug("New size: %d", b_est.rows * b_est.cols);

        // Do cloud estimation using the 1D arrays
        cv::Mat lambda_n = (b_est - r_est) / (b_est + r_est);

        cv::Scalar mean;
        cv::Scalar stddev;
        cv::meanStdDev(lambda_n, mean, stddev);
        const double std = stddev[0];

        const bool is_bimodal = std > 0.03;
        cv::Mat ratio_mask;
        if (std > 0.03) 
        {
            const double threshold = find_threshold_mce(lambda_n);
            cv::threshold(lambda_n, ratio_mask, threshold, 255, cv::THRESH_BINARY);
        } 
        else 
        {
            cv::threshold(b_est - r_est, ratio_mask, 30, 255, cv::THRESH_BINARY);
        }
        const double N_Cloud = static_cast<double>(cv::countNonZero(ratio_mask == 0));
        const double N_Sky = static_cast<double>(cv::countNonZero(ratio_mask == 255));
        const double ccr = (N_Cloud / (N_Cloud + N_Sky)) * 100.0;
        return {round(ccr * 100.0) / 100.0, !is_bimodal};
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

    std::pair<double, bool> estimate(const cv::Mat& frame) override 
    {
        if (detection_mask_.empty())
        {
            detection_mask_ = cv::Mat::ones(frame.size(), CV_8U);
        }
        if (detection_mask_.size() != frame.size())
        {
            cv::resize(detection_mask_, detection_mask_, frame.size());
        }

        cv::Mat frame_gray;
        cv::cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);

        cv::Mat cloud_feature_image = generate_cloud_feature_image(frame_gray);
        if (mask_enabled_)
        {
            cloud_feature_image.setTo(0, detection_mask_ == 0);
        }

        int N_Cloud = cv::countNonZero(cloud_feature_image == 255);
        int N_Sky = cv::countNonZero(cloud_feature_image == 0);

        double ccr = (N_Cloud / static_cast<double>(N_Cloud + N_Sky)) * 100;
        return {round(ccr * 100.0) / 100.0, true};
    }
};

#endif