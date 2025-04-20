#pragma once

#include <stddef.h>
#include <string>

#include <rclcpp/rclcpp.hpp>

class TrackProviderWorkerParams
{
public:
    TrackProviderWorkerParams() = default;

    // Getters
    [[nodiscard]] const auto &get_tracking_publisher_topic() const { return topics_params_.tracking_publisher_topic_; }
    [[nodiscard]] const auto &get_bounding_boxes_subscription_topic() const { return topics_params_.bounding_boxes_subscription_topic_; }
    [[nodiscard]] int get_resize_height() const { return resize_height_; }

    // Setters
    void set_tracking_publisher_topic(const std::string &topic) { topics_params_.tracking_publisher_topic_ = topic; }
    void set_bounding_boxes_subscription_topic(const std::string &topic) { topics_params_.bounding_boxes_subscription_topic_ = topic; }
    void set_resize_height(int height) { resize_height_ = height; }

private:
    struct TopicsParams
    {
        std::string tracking_publisher_topic_;
        std::string bounding_boxes_subscription_topic_;
    };
    int resize_height_{0};

    TopicsParams topics_params_;
};
