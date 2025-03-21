#pragma once

#include <stddef.h>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <bob_interfaces/msg/detector_state.hpp>
#include <bob_interfaces/msg/detector_b_box_array.hpp>

#include "background_subtractor_companion.hpp"

class BackgroundSubtractorWorkerParams
{
public:
    enum class BGSType
    {
        Unknown,
        Vibe,
        WMV
    };

    // Constructor
    BackgroundSubtractorWorkerParams() = default;

    // Getters
    [[nodiscard]] size_t get_max_queue_process_size() const { return max_queue_process_size_; }

    [[nodiscard]] const auto &get_image_publisher() const { return image_publisher_; }
    [[nodiscard]] const auto &get_image_resized_publisher() const { return image_resized_publisher_; }
    [[nodiscard]] const auto &get_detection_publisher() const { return detection_publisher_; }
    [[nodiscard]] const auto &get_state_publisher() const { return state_publisher_; }

    [[nodiscard]] const auto &get_image_publish_topic() const { return image_publish_topic_; }
    [[nodiscard]] const auto &get_image_resized_publish_topic() const { return image_resized_publish_topic_; }
    [[nodiscard]] const auto &get_detection_publish_topic() const { return detection_publish_topic_; }
    [[nodiscard]] const auto &get_detection_state_publish_topic() const { return detection_state_publish_topic_; }
    [[nodiscard]] bool get_use_cuda() const { return use_cuda_; }
    [[nodiscard]] BGSType get_bgs_type() const { return bgs_type_; }
    [[nodiscard]] const auto &get_sensitivity() const { return sensitivity_; }
    [[nodiscard]] const auto &get_sensitivity_collection() const { return sensitivity_collection_; }
    [[nodiscard]] bool get_mask_enable_override() const { return mask_enable_override_; }
    [[nodiscard]] const auto &get_mask_filename() const { return mask_filename_; }
    [[nodiscard]] int get_resize_height() const { return resize_height_; }
    [[nodiscard]] int get_mask_timer_seconds() const { return mask_timer_seconds_; }
    [[nodiscard]] bool get_recording_enabled() const { return recording_enabled_; }
    [[nodiscard]] int get_recording_seconds_save() const { return recording_seconds_save_; }

    // Setters
    void set_max_queue_process_size(size_t size) { max_queue_process_size_ = size; }

    void set_image_publisher(const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr &publisher) { image_publisher_ = publisher; }
    void set_image_resized_publisher(const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr &publisher) { image_resized_publisher_ = publisher; }
    void set_detection_publisher(const rclcpp::Publisher<bob_interfaces::msg::DetectorBBoxArray>::SharedPtr &publisher) { detection_publisher_ = publisher; }
    void set_state_publisher(const rclcpp::Publisher<bob_interfaces::msg::DetectorState>::SharedPtr &publisher) { state_publisher_ = publisher; }

    void set_image_publish_topic(const std::string &topic) { image_publish_topic_ = topic; }
    void set_image_resized_publish_topic(const std::string &topic) { image_resized_publish_topic_ = topic; }
    void set_detection_publish_topic(const std::string &topic) { detection_publish_topic_ = topic; }
    void set_detection_state_publish_topic(const std::string &topic) { detection_state_publish_topic_ = topic; }
    void set_use_cuda(bool enable) { use_cuda_ = enable; }
    void set_bgs_type(BGSType type) { bgs_type_ = type; }
    void set_sensitivity(const std::string &sensitivity) { sensitivity_ = sensitivity; }
    void set_sensitivity_collection(const SensitivityConfigCollection &collection) { sensitivity_collection_ = collection; }
    void set_sensitivity_collection(const std::string &json_collection) { sensitivity_collection_.set_configs(json_collection); }
    void set_mask_enable_override(bool enable) { mask_enable_override_ = enable; }
    void set_mask_filename(const std::string &filename) { mask_filename_ = filename; }
    void set_resize_height(int height) { resize_height_ = height; }
    void set_mask_timer_seconds(int seconds) { mask_timer_seconds_ = seconds; }
    void set_recording_enabled(bool enable) { recording_enabled_ = enable; }
    void set_recording_seconds_save(int seconds) { recording_seconds_save_ = seconds; }

private:
    size_t max_queue_process_size_{0};
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_resized_publisher_;
    rclcpp::Publisher<bob_interfaces::msg::DetectorBBoxArray>::SharedPtr detection_publisher_;
    rclcpp::Publisher<bob_interfaces::msg::DetectorState>::SharedPtr state_publisher_;

    std::string image_publish_topic_;
    std::string image_resized_publish_topic_;
    std::string detection_publish_topic_;
    std::string detection_state_publish_topic_;
    bool use_cuda_{true};
    BGSType bgs_type_{BGSType::Unknown};
    std::string sensitivity_;
    SensitivityConfigCollection sensitivity_collection_;
    bool mask_enable_override_{true};
    std::string mask_filename_;
    int resize_height_{};
    int mask_timer_seconds_{};
    bool recording_enabled_{false};
    int recording_seconds_save_{2};
};
