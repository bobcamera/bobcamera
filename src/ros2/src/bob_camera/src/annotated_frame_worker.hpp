#pragma once

#include <string>
#include <memory>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <sensor_msgs/msg/image.hpp>

#include <json/json.h>

#include <bob_interfaces/msg/tracking.hpp>
#include <parameter_node.hpp>
#include <image_utils.hpp>

#include <boblib/api/utils/profiler.hpp>
#include <boblib/api/utils/jpeg_compressor.hpp>

#include "annotated_frame/annotated_frame_creator.hpp"
#include "camera_bgs_params.hpp"

#include "tracking.hpp"

class AnnotatedFrameWorker final
{
public:
    explicit AnnotatedFrameWorker(ParameterNode &node
                                , CameraBgsParams &params
                                , const rclcpp::QoS &pub_qos_profile
                                , boblib::utils::pubsub::TopicManager &topic_manager
                                , boblib::utils::Profiler &profiler)
        : node_(node)
        , params_(params)
        , pub_qos_profile_(pub_qos_profile)
        , topic_manager_(topic_manager)
        , profiler_(profiler)
    {
    }

    void init()
    {
        prof_annotated_id_ = profiler_.add_region("Annotated Frame Provider");
        prof_create_frame_id_ = profiler_.add_region("Create Frame", prof_annotated_id_);
        prof_resize_id_ = profiler_.add_region("Resizing Frame", prof_annotated_id_);
        prof_publish_id_ = profiler_.add_region("Publish Frame", prof_annotated_id_);

        jpeg_compressor_ptr_ = std::make_unique<boblib::utils::JpegCompressor>(params_.compression_quality);

        pub_annotated_frame_ = node_.create_publisher<sensor_msgs::msg::Image>(params_.topics.annotated_frame_publisher_topic, pub_qos_profile_);
        image_resized_publisher_ = node_.create_publisher<sensor_msgs::msg::Image>(params_.topics.annotated_frame_publisher_topic + "/resized", pub_qos_profile_);
        pub_annotated_compressed_frame_ = node_.create_publisher<sensor_msgs::msg::CompressedImage>(params_.topics.annotated_frame_publisher_topic + "/resized/compressed", pub_qos_profile_);

        annotated_frame_creator_ptr_ = std::make_unique<AnnotatedFrameCreator>(params_.annotated_frame.visualiser_settings);

        tracking_pubsub_ptr_ = topic_manager_.get_topic<bob_camera::Tracking>(params_.topics.tracking_publisher_topic);
        tracking_pubsub_ptr_->subscribe<AnnotatedFrameWorker, &AnnotatedFrameWorker::callback>(this);
    }

private:
    void callback(const std::shared_ptr<bob_camera::Tracking> &tracking_ptr) noexcept
    {
        try
        {
            profiler_.start(prof_create_frame_id_);
            annotated_frame_creator_ptr_->create_frame(tracking_ptr, params_.annotated_frame.enable_tracking_status_message);
            profiler_.stop(prof_create_frame_id_);

            //node_.publish_if_subscriber(pub_annotated_frame_, *image_msg);
            publish(tracking_ptr);
        }
        catch (const std::exception &e)
        {
            node_.log_send_error("callback: exception: %s", e.what());
        }
    }

    void publish(const std::shared_ptr<bob_camera::Tracking> &tracking_ptr)
    {
        try
        {
            profiler_.start(prof_publish_id_);
            auto &img = tracking_ptr->image_ptr->toMat();

            ImageUtils::publish_image(pub_annotated_frame_, *tracking_ptr->header_ptr, img);

            if (params_.resize_height <= 0)
            {
                return;
            }
            if (params_.resize_height != img.size().height)
            {
                cv::Mat resized_img;
                const auto frame_width = static_cast<int>(img.size().aspectRatio() * static_cast<double>(params_.resize_height));
                cv::resize(img, resized_img, cv::Size(frame_width, params_.resize_height), 0, 0, cv::INTER_NEAREST);

                ImageUtils::publish_image(image_resized_publisher_, *tracking_ptr->header_ptr, resized_img);
                ImageUtils::publish_compressed_image(pub_annotated_compressed_frame_, *jpeg_compressor_ptr_, *tracking_ptr->header_ptr, resized_img);
            }
            else
            {
                ImageUtils::publish_image(image_resized_publisher_, *tracking_ptr->header_ptr, img);
                ImageUtils::publish_compressed_image(pub_annotated_compressed_frame_, *jpeg_compressor_ptr_, *tracking_ptr->header_ptr, img);
            }

            profiler_.stop(prof_publish_id_);
        }
        catch (const std::exception &e)
        {
            node_.log_send_error("publish_resized_frame: exception: %s", e.what());
        }
    }

    void publish_resized(const cv::Mat &img, const std_msgs::msg::Header &header) noexcept
    {
        if (image_resized_publisher_->get_subscription_count() > 0)
        {
            auto loaned = image_resized_publisher_->borrow_loaned_message();
            auto &camera_msg = loaned.get();
            ImageUtils::fill_imagemsg_header(camera_msg, header, img);
            const size_t totalBytes = img.total() * img.elemSize();
            camera_msg.data.assign(img.data, img.data + totalBytes);
            image_resized_publisher_->publish(std::move(camera_msg));
        }

        if (pub_annotated_compressed_frame_->get_subscription_count() > 0)
        {
            auto loaned = pub_annotated_compressed_frame_->borrow_loaned_message();
            auto &resized_frame_msg = loaned.get();
            resized_frame_msg.header = header;
            resized_frame_msg.format = "jpeg";
            resized_frame_msg.data.reserve(img.total() * img.elemSize() / 2);
            jpeg_compressor_ptr_->compress(img, resized_frame_msg.data);
            pub_annotated_compressed_frame_->publish(std::move(resized_frame_msg));
        }
    }

    ParameterNode &node_;
    CameraBgsParams &params_;
    const rclcpp::QoS &pub_qos_profile_;
    boblib::utils::pubsub::TopicManager &topic_manager_;
    boblib::utils::Profiler &profiler_;

    std::unique_ptr<AnnotatedFrameCreator> annotated_frame_creator_ptr_;

    std::shared_ptr<boblib::utils::pubsub::PubSub<bob_camera::Tracking>> tracking_pubsub_ptr_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_annotated_frame_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_resized_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_annotated_compressed_frame_;

    std::unique_ptr<boblib::utils::JpegCompressor> jpeg_compressor_ptr_;

    size_t prof_annotated_id_;
    size_t prof_create_frame_id_;
    size_t prof_resize_id_;
    size_t prof_publish_id_;
};
