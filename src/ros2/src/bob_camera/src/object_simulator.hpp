#pragma once

#include <random>
#include <memory>
#include <opencv2/opencv.hpp>

class MovingCircle 
{
public:
    MovingCircle(double path_linearity, std::pair<int, int> size_range, std::pair<int, int> step_range, const cv::Rect& occluder = cv::Rect(0, 0, 0, 0))
        : linearity_(path_linearity)
        , color_(10, 10, 20)
        , size_range_(size_range), step_range_(step_range)
        , occluder_(occluder)
    {}

    void move() noexcept
    {
        static constexpr double deg_to_rad = M_PI / 180.0;

        double angle_dist = 0.0;
        if (rand() / static_cast<double>(RAND_MAX) > linearity_) 
        {
            angle_dist = rand() / static_cast<double>(RAND_MAX) * 17 - 11;
        }

        double step_dist = step_range_.first + rand() % (step_range_.second - step_range_.first + 1);
        angle_ += angle_dist;

        double rad_angle = angle_ * deg_to_rad;
        double new_x = x_ + step_ * std::cos(rad_angle);
        double new_y = y_ + step_ * std::sin(rad_angle);

        double distance = std::sqrt((new_x - center_.first) * (new_x - center_.first) + (new_y - center_.second) * (new_y - center_.second));

        if (distance >= radius_of_mask_) 
        {
            double random_angle = rand() / static_cast<double>(RAND_MAX) * 40 - 20;
            angle_ += 180 + random_angle;
            new_x = x_ + step_ * std::cos(angle_ * M_PI / 180);
            new_y = y_ + step_ * std::sin(angle_ * M_PI / 180);
        }

        x_ = new_x;
        y_ = new_y;

        double size_factor = 1.0 - (distance / radius_of_mask_);
        size_ = static_cast<int>(size_range_.first + size_factor * (size_range_.second - size_range_.first));

        if (rand() % 122 == 0) 
        {
            angle_ = angle_dist;
            step_ = step_dist;
        }
    }

    void draw(cv::Mat& frame, const cv::Scalar& color) noexcept
    {
        cv::Point center(static_cast<int>(x_), static_cast<int>(y_));
        if (!occluder_.contains(center)) 
        {
            cv::circle(frame, center, size_, color, -1);
        }
    }

    void initialize(const boblib::base::Image & img)
    {
        x_ = 0.0;
        y_ = 0.0;
        angle_ = 0.0;
        step_ = 0.0;
        size_ = 0;
        center_ = std::make_pair(img.size().width / 2, img.size().height / 2);
        radius_of_mask_ = std::min(img.size().width, img.size().height) / 1.5;

        bool valid_position = false;
        while (!valid_position) 
        {
            x_ = rand() % img.size().width;
            y_ = rand() % img.size().height;

            double distance = std::sqrt((x_ - center_.first) * (x_ - center_.first) + (y_ - center_.second) * (y_ - center_.second));
            if (distance <= radius_of_mask_) 
            {
                valid_position = true;
            }
        }

        angle_ = ((double)rand() / RAND_MAX) * 359;
        color_ = cv::Scalar(10, 10, 20);
        size_ = size_range_.first + rand() % (size_range_.second - size_range_.first + 1);
        step_ = step_range_.first + rand() % (step_range_.second - step_range_.first + 1);
    }

private:
    double x_ = 0.0;
    double y_ = 0.0;
    double angle_ = 0.0;
    double step_ = 0.0;
    double linearity_;
    double radius_of_mask_ = 0.0;
    int size_ = 0;
    std::pair<int, int> center_;
    cv::Scalar color_;
    std::pair<int, int> size_range_;
    std::pair<int, int> step_range_;
    cv::Rect occluder_;
};

class ObjectSimulator
{
public:
    explicit ObjectSimulator(int num_simulated_objects)
        : num_simulated_objects_(num_simulated_objects)
        , default_size_range_{2, 10}
        , default_step_range_{5, 30}
    {}

    void initialize(const boblib::base::Image & img)
    {
        img_size_ = img.size();
        for (int i = 0; i < num_simulated_objects_; ++i)
        {
            MovingCircle circle(0.5, default_size_range_, default_step_range_);
            circle.initialize(img);
            moving_circles_.emplace_back(std::move(circle));
        }
    }

    void move(boblib::base::Image & img)
    {
        if (moving_circles_.empty() || img.size() != img_size_)
        {
            initialize(img);
        }
        auto & img_mat = img.toMat();
        for (auto& circle : moving_circles_) 
        {
            circle.move();
            circle.draw(img_mat, cv::Scalar(10, 10, 20));
        }
        img.upload();
    }

private:
    int num_simulated_objects_;
    std::vector<MovingCircle> moving_circles_;
    std::pair<int, int> default_size_range_; 
    std::pair<int, int> default_step_range_;
    cv::Size img_size_;
};
