#pragma once

#include <filesystem>
#include <mutex>
#include <chrono>
#include <ctime>
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visibility_control.h>

#include <boblib/api/utils/pubsub/TopicManager.hpp>

#include <parameter_node.hpp>

#include "bob_camera/msg/camera_info.hpp"
#include "bob_interfaces/msg/tracking.hpp"
#include "bob_interfaces/msg/recording_state.hpp"
#include "bob_interfaces/msg/recording_event.hpp"
#include "bob_interfaces/srv/recording_request.hpp"
#include "image_utils.hpp"
#include "camera_bgs_params.hpp"

class RecordManagerWorker final
{
public:
    explicit RecordManagerWorker(ParameterNode &node
                                 , CameraBgsParams &params
                                 , const rclcpp::QoS &pub_qos_profile
                                 , boblib::utils::pubsub::TopicManager &topic_manager
                                 , boblib::utils::Profiler &profiler)
        : node_(node)
        , params_(params)
        , topic_manager_(topic_manager)
        , pub_qos_profile_(pub_qos_profile)
        , profiler_(profiler)
    {
    }

    enum class RecordingStateEnum
    {
        Disabled,
        BeforeStart,
        BetweenEvents,
        AfterEnd
    };

    void init()
    {
        current_state_ = RecordingStateEnum::BeforeStart;

        recording_request_service_ = node_.create_service<bob_interfaces::srv::RecordingRequest>(params_.topics.recording_request_service_topic,
                                                                                                 [this](const std::shared_ptr<bob_interfaces::srv::RecordingRequest::Request> request, std::shared_ptr<bob_interfaces::srv::RecordingRequest::Response> response)
                                                                                                 { change_recording_enabled_request(request, response); });
        state_publisher_ = node_.create_publisher<bob_interfaces::msg::RecordingState>(params_.topics.recording_state_publisher_topic, pub_qos_profile_);
        event_publisher_ = node_.create_publisher<bob_interfaces::msg::RecordingEvent>(params_.topics.recording_event_publisher_topic, pub_qos_profile_);

        recording_event_pubsub_ptr_ = topic_manager_.get_topic<bob_interfaces::msg::RecordingEvent>(params_.topics.recording_event_publisher_topic);

        tracking_pubsub_ptr_ = topic_manager_.get_topic<bob_interfaces::msg::Tracking>(params_.topics.tracking_publisher_topic);
        tracking_pubsub_ptr_->subscribe<RecordManagerWorker, &RecordManagerWorker::tracking_info_callback>(this);

        camera_info_pubsub_ptr_ = topic_manager_.get_topic<bob_camera::msg::CameraInfo>(params_.topics.camera_info_publish_topic);
        camera_info_pubsub_ptr_->subscribe<RecordManagerWorker, &RecordManagerWorker::camera_info_callback>(this);
    };

private:
    static std::string get_current_date_as_str()
    {
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        std::tm local_tm;

        localtime_r(&time_t, &local_tm);

        std::array<char, 16> buffer;
        strftime(buffer.data(), buffer.size(), "%Y%m%d", &local_tm);

        return std::string(buffer.data());
    }

    bool create_recording_directories(const std::string &path)
    {
        std::filesystem::path dirPath = path;

        dated_directory_ = dirPath / get_current_date_as_str() / params_.recording.prefix;

        if (std::filesystem::create_directories(dated_directory_))
        {
            node_.log_send_info("Dated directory created: %s", dated_directory_.c_str());
            return true;
        }

        node_.log_send_error("Failed to create dated directory: %s", dated_directory_.c_str());
        return false;
    }

    static std::string generate_filename(builtin_interfaces::msg::Time time)
    {
        auto stamp = rclcpp::Time(time);
        std::ostringstream oss;
        oss << unsigned(stamp.seconds());
        return oss.str();
    }

    void camera_info_callback(const bob_camera::msg::CameraInfo::SharedPtr &camera_info_msg) noexcept
    {
        last_camera_info_ = *camera_info_msg;
    }

    void tracking_info_callback(const bob_interfaces::msg::Tracking::SharedPtr &tracking_msg) noexcept
    {
        if (!params_.recording.enabled || last_camera_info_.fps == 0)
        {
            return;
        }
        try
        {
            if ((current_state_ != RecordingStateEnum::BeforeStart) && (prev_frame_size_ != cv::Size(last_camera_info_.frame_width, last_camera_info_.frame_height)))
            {
                node_.log_send_info("Frame dimensions changed.");
                current_state_ = RecordingStateEnum::AfterEnd;
                current_end_frame_ = 0;
            }

            switch (current_state_)
            {
            case RecordingStateEnum::BeforeStart:
                if (tracking_msg->state.trackable > 0)
                {
                    recording_ = true;
                    current_state_ = RecordingStateEnum::BetweenEvents;

                    video_fps_ = static_cast<double>(last_camera_info_.fps);
                    base_filename_ = generate_filename(tracking_msg->header.stamp);
                    prev_frame_size_.width = last_camera_info_.frame_width;
                    prev_frame_size_.height = last_camera_info_.frame_height;

                    if (auto current_date = get_current_date_as_str(); current_date != date_)
                    {
                        create_recording_directories(params_.recording.recordings_directory);
                        date_ = current_date;
                    }

                    node_.log_send_info("Starting track recording into: %s", dated_directory_.c_str());

                    bob_interfaces::msg::RecordingEvent event;
                    event.header = tracking_msg->header;
                    event.recording = recording_;
                    event.recording_path = dated_directory_;
                    event.filename = base_filename_;
                    node_.publish_if_subscriber(event_publisher_, event);
                    recording_event_pubsub_ptr_->publish(std::move(event));
                }
                break;

            case RecordingStateEnum::BetweenEvents:
                if (tracking_msg->state.trackable == 0)
                {
                    current_state_ = RecordingStateEnum::AfterEnd;
                    current_end_frame_ = params_.recording.seconds_save * video_fps_;
                }
                break;

            case RecordingStateEnum::AfterEnd:
                if (current_end_frame_ == 0)
                {
                    recording_ = false;
                    node_.log_send_info("Ending track recording...");
                    current_state_ = RecordingStateEnum::BeforeStart;

                    bob_interfaces::msg::RecordingEvent event;
                    event.header = tracking_msg->header;
                    event.recording = recording_;
                    event.recording_path = dated_directory_;
                    event.filename = base_filename_;
                    node_.publish_if_subscriber(event_publisher_, event);
                    recording_event_pubsub_ptr_->publish(std::move(event));
                }
                else
                {
                    --current_end_frame_;
                    if (tracking_msg->state.trackable > 0)
                    {
                        current_state_ = RecordingStateEnum::BetweenEvents;
                        current_end_frame_ = params_.recording.seconds_save * video_fps_;
                    }
                }
                break;

            case RecordingStateEnum::Disabled:
                if (recording_)
                {
                    recording_ = false;
                    current_state_ = RecordingStateEnum::BeforeStart;
                    node_.log_send_info("Requested: Ending track recording...");

                    bob_interfaces::msg::RecordingEvent event;
                    event.header = tracking_msg->header;
                    event.recording = recording_;
                    event.recording_path = dated_directory_;
                    node_.publish_if_subscriber(event_publisher_, event);
                    recording_event_pubsub_ptr_->publish(std::move(event));
                }
                break;
            }

            bob_interfaces::msg::RecordingState state;
            state.recording = recording_;
            node_.publish_if_subscriber(state_publisher_, state);
        }
        catch (const std::exception &e)
        {
            node_.log_send_error("exception: %s", e.what());
        }
    };

    void change_recording_enabled_request(const std::shared_ptr<bob_interfaces::srv::RecordingRequest::Request> request,
                                          std::shared_ptr<bob_interfaces::srv::RecordingRequest::Response> response)
    {
        current_state_ = request->disable_recording ? RecordingStateEnum::Disabled : RecordingStateEnum::BeforeStart;
        response->success = true;
    }

    ParameterNode &node_;
    CameraBgsParams &params_;
    boblib::utils::pubsub::TopicManager &topic_manager_;

    const rclcpp::QoS &pub_qos_profile_;

    std::shared_ptr<boblib::utils::pubsub::PubSub<bob_interfaces::msg::Tracking>> tracking_pubsub_ptr_;
    std::shared_ptr<boblib::utils::pubsub::PubSub<bob_camera::msg::CameraInfo>> camera_info_pubsub_ptr_;
    std::shared_ptr<boblib::utils::pubsub::PubSub<bob_interfaces::msg::RecordingEvent>> recording_event_pubsub_ptr_;

    RecordingStateEnum current_state_;
    std::string dated_directory_;
    std::string date_;
    std::string base_filename_;
    rclcpp::Service<bob_interfaces::srv::RecordingRequest>::SharedPtr recording_request_service_;
    rclcpp::Publisher<bob_interfaces::msg::RecordingState>::SharedPtr state_publisher_;
    rclcpp::Publisher<bob_interfaces::msg::RecordingEvent>::SharedPtr event_publisher_;

    double video_fps_;
    size_t current_end_frame_;
    cv::Size prev_frame_size_;
    bool recording_;

    bob_camera::msg::CameraInfo last_camera_info_;

    boblib::utils::Profiler &profiler_;
};
