#include <random>
#include <memory>

#include <opencv2/opencv.hpp>

class MovingCircle {
public:
    MovingCircle(int frame_width, int frame_height, double path_linearity, std::pair<int, int> size_range, std::pair<int, int> step_range)
        : x_(0), y_(0), angle_(0), step_(0), linearity_(path_linearity), radiusOfMask_(0),
        width_(frame_width), height_(frame_height), size_(0), 
        center_(frame_width / 2, frame_height / 2),
        color_(10, 10, 20), size_range_(size_range), step_range_(step_range),
        occluder_(cv::Rect(0, 0, 0, 0))
    {
        initialize();
    }

    MovingCircle(int frame_width, int frame_height, double path_linearity, std::pair<int, int> size_range, std::pair<int, int> step_range, cv::Rect occluder)
        : x_(0), y_(0), angle_(0), step_(0), linearity_(path_linearity), radiusOfMask_(0),
        width_(frame_width), height_(frame_height), size_(0),
        center_(frame_width / 2, frame_height / 2),
        color_(10, 10, 20), size_range_(size_range), step_range_(step_range),
        occluder_(occluder)
    {
        initialize();
    }

    void move()
    {
        static const double degToRad = M_PI / 180.0;

        double angle_dist = 0;
        if (rand() / static_cast<double>(RAND_MAX) > linearity_) {
            angle_dist = rand() / static_cast<double>(RAND_MAX) * 17 - 11;
        }
        
        double step_dist = step_range_.first + rand() % (step_range_.second - step_range_.first + 1);
        angle_ += angle_dist;
        
        double radAngle = angle_ * degToRad;
        double new_x = x_ + step_ * std::cos(radAngle);
        double new_y = y_ + step_ * std::sin(radAngle);

        double distance = std::sqrt((new_x - center_.first) * (new_x - center_.first) + (new_y - center_.second) * (new_y - center_.second));

        if (distance >= radiusOfMask_) {
            double random_angle = rand() / static_cast<double>(RAND_MAX) * 40 - 20; 
            angle_ += 180 + random_angle;
            new_x = x_ + step_ * std::cos(angle_ * M_PI / 180);
            new_y = y_ + step_ * std::sin(angle_ * M_PI / 180);
        }

        x_ = new_x;
        y_ = new_y;
        
        double size_factor = 1.0 - (distance / radiusOfMask_);
        size_ = static_cast<int>(size_range_.first + size_factor * (size_range_.second - size_range_.first));


        if (rand() % 122 == 0) {
            angle_ = angle_dist;
            step_ = step_dist;
        }
    }

    void draw(cv::Mat &frame, cv::Scalar color) {
        cv::Point center(static_cast<int>(x_), static_cast<int>(y_));
        if (!occluder_.contains(center)) {
            cv::circle(frame, center, size_, color, -1);
        }
    }

private:

    void initialize() {
        x_ = 0; y_ = 0; angle_ = 0; step_ = 0; radiusOfMask_ = 0; size_ = 0;
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

    double x_, y_, angle_, step_, linearity_, radiusOfMask_;
    int width_, height_, size_;
    std::pair<int, int> center_;
    cv::Scalar color_;
    std::pair<int, int> size_range_, step_range_;
    cv::Rect occluder_;
};


class CircleFrameGenerator {
public:
    CircleFrameGenerator(std::map<std::string, int> settings, int num_circles, 
                         std::pair<int, int> size_range, std::pair<int, int> step_range, cv::Scalar background_color)
        : frame_size_(cv::Size(settings["width"], settings["height"])),
          background_color_(background_color),
          circle_color_(cv::Scalar(10, 10, 20)),
          background_frame_(cv::Mat(frame_size_, CV_8UC3, background_color_))
    {
        // Initialize the occluder
        int occluderWidth = frame_size_.width / 4; 
        int occluderHeight = frame_size_.height / 4; 
        int occluderX = (frame_size_.width - occluderWidth) / 2; 
        int occluderY = (frame_size_.height - occluderHeight) / 2; 
        cv::Rect occluder(occluderX, occluderY, occluderWidth, occluderHeight);

        cv::rectangle(background_frame_, occluder, cv::Scalar(0, 0, 0), cv::FILLED);

        // Initialize the circles
        for (int i = 0; i < num_circles; ++i) {
            circles_.emplace_back(frame_size_.width, frame_size_.height, 0.7, size_range, step_range, occluder);
        }
    }

    cv::Mat generate_frame() {

        cv::Mat frame = background_frame_.clone();
        
        for (auto &circle : circles_) {
            circle.draw(frame, circle_color_);
            circle.move();
        }
        return frame;
    }

private:
    cv::Size frame_size_;
    cv::Scalar background_color_;
    cv::Scalar circle_color_;  
    std::vector<MovingCircle> circles_;
    cv::Mat background_frame_;
};