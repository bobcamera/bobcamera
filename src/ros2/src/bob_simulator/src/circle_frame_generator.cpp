#include <random>
#include <memory>

#include <opencv2/opencv.hpp>

class MovingCircle {
public:
    MovingCircle(int frame_width, int frame_height, double path_linearity, std::pair<int, int> size_range, std::pair<int, int> step_range)
        : x_(0), y_(0), angle_(0), step_(0), linearity_(path_linearity), radiusOfMask_(0),
        width_(frame_width), height_(frame_height), size_(0),
        size_range_(size_range), step_range_(step_range)
    {
        center_ = std::make_pair(width_ / 2, height_ / 2);
        radiusOfMask_ = std::min(width_, height_) / 1.5;

        bool valid_position = false;
        while (!valid_position) {
            x_ = rand() % width_;
            y_ = rand() % height_;

            double distance = std::sqrt((x_ - center_.first) * (x_ - center_.first) + (y_ - center_.second) * (y_ - center_.second));
            if (distance <= radiusOfMask_) {
                valid_position = true;
            }
        }

        angle_ = ((double) rand() / RAND_MAX) * 359;
        color_ = cv::Scalar(10, 10, 20);
        size_ = size_range_.first + rand() % (size_range_.second - size_range_.first + 1);
        step_ = step_range_.first + rand() % (step_range_.second - step_range_.first + 1);
    }

    void move()
    {
        double angle_dist = 0;
        if (rand() / static_cast<double>(RAND_MAX) > linearity_) {
            angle_dist = rand() / static_cast<double>(RAND_MAX) * 17 - 11;
        }
        
        double step_dist = step_range_.first + rand() % (step_range_.second - step_range_.first + 1);
        angle_ += angle_dist;
        
        double new_x = x_ + step_ * std::cos(angle_ * M_PI / 180);
        double new_y = y_ + step_ * std::sin(angle_ * M_PI / 180);

        double distance = std::sqrt((new_x - center_.first) * (new_x - center_.first) + (new_y - center_.second) * (new_y - center_.second));

        if (distance >= radiusOfMask_) {
            double random_angle = rand() / static_cast<double>(RAND_MAX) * 40 - 20; 
            angle_ += 180 + random_angle;
            new_x = x_ + step_ * std::cos(angle_ * M_PI / 180);
            new_y = y_ + step_ * std::sin(angle_ * M_PI / 180);
        }

        x_ = new_x;
        y_ = new_y;
        
        double shrink_factor = 0.2;
        size_ = static_cast<int>(4 * (1 - shrink_factor * (distance / radiusOfMask_))) + 1;

        if (rand() % 122 == 0) {
            angle_ = angle_dist;
            step_ = step_dist;
        }
    }

    void draw(cv::Mat &frame, cv::Scalar color)
    {
        cv::circle(frame, cv::Point(static_cast<int>(x_), static_cast<int>(y_)), size_, color, -1);
    }

private:
    double x_, y_, angle_, step_, linearity_, radiusOfMask_;
    int width_, height_, size_;
    std::pair<int, int> center_;
    cv::Scalar color_;
    std::pair<int, int> size_range_, step_range_;
};

// Next, we'll create the CircleFrameGenerator class

class CircleFrameGenerator {
public:
    CircleFrameGenerator(std::map<std::string, int> settings, int num_circles, 
                         std::pair<int, int> size_range, std::pair<int, int> step_range, cv::Scalar background_color)
        : background_color_(background_color)
    {
        // Assuming the map settings has keys "height" and "width"
        frame_size = cv::Size(settings["width"], settings["height"]);

        // Initialize the circles
        for (int i = 0; i < num_circles; ++i) {
            circles.emplace_back(frame_size.width, frame_size.height, 0.7, size_range, step_range);
        }

        circle_color_ = cv::Scalar(10, 10, 20);
    }

    cv::Mat generate_frame()
    {
        cv::Mat frame(frame_size, CV_8UC3, background_color_);
        for (auto &circle : circles) {
            circle.draw(frame, circle_color_);
            circle.move();
        }
        return frame;
    }

private:
    std::vector<MovingCircle> circles;
    cv::Size frame_size;
    cv::Scalar background_color_;
    cv::Scalar circle_color_;  
};