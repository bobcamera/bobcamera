#include <filesystem>
#include <mutex>
#include <chrono>
#include <ctime>
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visibility_control.h>
#include "bob_camera/msg/camera_info.hpp"
#include "bob_interfaces/msg/tracking.hpp"
#include "bob_interfaces/msg/recording_state.hpp"
#include "bob_interfaces/msg/recording_event.hpp"
#include "bob_interfaces/srv/recording_request.hpp"
#include "parameter_node.hpp"
#include "image_utils.hpp"
#include "image_recorder.hpp"
#include "json_recorder.hpp"
#include "video_recorder.hpp"

class RecordManager
    : public ParameterNode 
{
public:
    COMPOSITION_PUBLIC
    explicit RecordManager(const rclcpp::NodeOptions& options)
        : ParameterNode("recorder_manager", options)
        , pub_qos_profile_(4)
        , sub_qos_profile_(10)
    {
    }

    void on_configure() override
    {
        log_info("Configuring");

        init();
    }

private:
    enum class RecordingStateEnum 
    {
        Disabled,
        BeforeStart,
        BetweenEvents,
        AfterEnd
    };

    void init()
    {
        pub_qos_profile_.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        pub_qos_profile_.durability(rclcpp::DurabilityPolicy::Volatile);
        pub_qos_profile_.history(rclcpp::HistoryPolicy::KeepLast);

        sub_qos_profile_.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        sub_qos_profile_.durability(rclcpp::DurabilityPolicy::Volatile);
        sub_qos_profile_.history(rclcpp::HistoryPolicy::KeepLast);

        current_state_ = RecordingStateEnum::BeforeStart;

        declare_node_parameters();

        time_synchronizer_ = std::make_shared<message_filters::TimeSynchronizer<bob_interfaces::msg::Tracking, bob_camera::msg::CameraInfo>>
            (*sub_tracking_, *sub_camera_info_, 10);
        time_synchronizer_->registerCallback(&RecordManager::process_recordings, this);
    };

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

    static bool create_dir(const std::string & path) 
    {
        if (std::filesystem::path dirPath = path; !std::filesystem::exists(dirPath)) 
        {
            return std::filesystem::create_directories(dirPath);
        }
        return true;
    }

    void create_subdirectory(const std::filesystem::path & parent, const std::string & subdirectory) const
    {
        std::filesystem::path subDirPath = parent / subdirectory;
        if (std::filesystem::create_directory(subDirPath)) 
        {
            log_info("Subdirectory created: %s", subDirPath.c_str());
        } 
        else 
        {
            log_error("Failed to create subdirectory: %s", subDirPath.c_str());
        }
    }
    
    bool create_dated_dir(const std::string & path) 
    {
        std::filesystem::path dirPath = path;

        if (std::filesystem::exists(dirPath)) 
        {
            dated_directory_ = dirPath / get_current_date_as_str();

            if (std::filesystem::create_directory(dated_directory_)) 
            {
                log_send_info("Dated directory created: %s", dated_directory_.c_str());
                return true;
            } 

            log_send_error("Failed to create dated directory: %s", dated_directory_.c_str());
            return false;
        } 

        log_send_info("Parent recordings directory doesn't exist: %s", dirPath.c_str());
        return false;
    }

    static std::string generate_filename(builtin_interfaces::msg::Time time)
    {  
        auto stamp = rclcpp::Time(time);
        std::ostringstream oss;
        oss << unsigned(stamp.seconds());
        return oss.str();
    } 

    void declare_node_parameters() 
    {
        std::vector<ParameterNode::ActionParam> params = {
            ParameterNode::ActionParam(
                rclcpp::Parameter("recordings_directory", "."), 
                [this](const rclcpp::Parameter& param) 
                {
                    recordings_directory_ = param.as_string();
                    create_dir(recordings_directory_);
                }
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("recording_request_service_topic", "bob/recording/update"), 
                [this](const rclcpp::Parameter& param) 
                {
                    recording_request_service_ = create_service<bob_interfaces::srv::RecordingRequest>(param.as_string(),
                            [this](const std::shared_ptr<bob_interfaces::srv::RecordingRequest::Request> request, std::shared_ptr<bob_interfaces::srv::RecordingRequest::Response> response)
                                {change_recording_enabled_request(request, response);});
                }
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("state_publisher_topic", "bob/recording/state"), 
                [this](const rclcpp::Parameter& param) 
                {
                    state_publisher_ = create_publisher<bob_interfaces::msg::RecordingState>(param.as_string(), pub_qos_profile_);
                }
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("event_publisher_topic", "bob/recording/event"), 
                [this](const rclcpp::Parameter& param) 
                {
                    event_publisher_ = create_publisher<bob_interfaces::msg::RecordingEvent>(param.as_string(), pub_qos_profile_);
                }
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("tracking_topic", "bob/tracker/tracking"), 
                [this](const rclcpp::Parameter& param) 
                {
                    tracking_topic_ = param.as_string();
                    sub_tracking_ = std::make_shared<message_filters::Subscriber<bob_interfaces::msg::Tracking>>(shared_from_this(), tracking_topic_, sub_qos_profile_.get_rmw_qos_profile());
                }
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("camera_info_topic", "bob/camera/all_sky/camera_info"), 
                [this](const rclcpp::Parameter& param)
                {
                    camera_info_topic_ = param.as_string();
                    sub_camera_info_ = std::make_shared<message_filters::Subscriber<bob_camera::msg::CameraInfo>>(shared_from_this(), camera_info_topic_, sub_qos_profile_.get_rmw_qos_profile());
                }
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("seconds_save", 2), 
                [this](const rclcpp::Parameter& param) 
                {
                    number_seconds_save_ = static_cast<int>(param.as_int());
                }
            ),
        };
        add_action_parameters(params);
    }

    // Callback function
    void process_recordings(const bob_interfaces::msg::Tracking::SharedPtr & tracking_msg,
                            const bob_camera::msg::CameraInfo::SharedPtr & camera_info_msg)
    {
        try
        {
            if ((current_state_ != RecordingStateEnum::BeforeStart) 
                && (prev_frame_size_ != cv::Size(camera_info_msg->frame_width, camera_info_msg->frame_height)))
            {
                log_send_info("Frame dimensions changed.");
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

                    video_fps_ = static_cast<double>(camera_info_msg->fps);
                    base_filename_ = generate_filename(tracking_msg->header.stamp);
                    prev_frame_size_.width = camera_info_msg->frame_width;
                    prev_frame_size_.height = camera_info_msg->frame_height;

                    if (auto current_date = get_current_date_as_str(); current_date != date_)
                    {
                        create_dated_dir(recordings_directory_);
                        date_ = current_date;
                    }

                    // Directory for recordings
                    log_send_info("Starting track recording...");

                    bob_interfaces::msg::RecordingEvent event;
                    event.trigger_header = tracking_msg->header;
                    event.recording = recording_;
                    event.recording_path = dated_directory_;
                    event_publisher_->publish(event);
                } 
                break;

            case RecordingStateEnum::BetweenEvents:
                if (tracking_msg->state.trackable == 0) 
                {
                    current_state_ = RecordingStateEnum::AfterEnd;
                }
                break;

            case RecordingStateEnum::AfterEnd:
                if (current_end_frame_ == 0) 
                {
                    recording_ = false;
                    log_send_info("Ending track recording...");
                    current_state_ = RecordingStateEnum::BeforeStart;

                    bob_interfaces::msg::RecordingEvent event;
                    event.trigger_header = tracking_msg->header;
                    event.recording = recording_;
                    event.recording_path = dated_directory_;
                    event_publisher_->publish(event);
                }
                else 
                {
                    --current_end_frame_;
                    if (tracking_msg->state.trackable > 0) 
                    {
                        current_state_ = RecordingStateEnum::BetweenEvents;
                    }
                }
                break;

            case RecordingStateEnum::Disabled:
                if (recording_) 
                {
                    recording_ = false;
                    current_state_ = RecordingStateEnum::BeforeStart;
                    log_send_info("Requested: Ending track recording...");

                    bob_interfaces::msg::RecordingEvent event;
                    event.trigger_header = tracking_msg->header;
                    event.recording = recording_;
                    event.recording_path = dated_directory_;
                    event_publisher_->publish(event);
                }
                break;
            }

            bob_interfaces::msg::RecordingState state;
            state.recording = recording_;
            state_publisher_->publish(state);
        }
        catch (const std::exception & e)
        {
            log_send_error("exception: %s", e.what());
        }
    };

    void change_recording_enabled_request(const std::shared_ptr<bob_interfaces::srv::RecordingRequest::Request> request, 
        std::shared_ptr<bob_interfaces::srv::RecordingRequest::Response> response)
    {
        current_state_ = request->disable_recording ? RecordingStateEnum::Disabled : RecordingStateEnum::BeforeStart;
        response->success = true;
    }

    rclcpp::QoS pub_qos_profile_;
    rclcpp::QoS sub_qos_profile_;

    rclcpp::TimerBase::SharedPtr timer_;
    RecordingStateEnum current_state_;
    std::string trackingTopic_;
    std::string recordings_directory_;
    std::string dated_directory_;
    std::string date_;
    std::string base_filename_;
    std::string tracking_topic_;
    std::string camera_info_topic_;
    rclcpp::Service<bob_interfaces::srv::RecordingRequest>::SharedPtr recording_request_service_;
    rclcpp::Publisher<bob_interfaces::msg::RecordingState>::SharedPtr state_publisher_;
    rclcpp::Publisher<bob_interfaces::msg::RecordingEvent>::SharedPtr event_publisher_;
    std::shared_ptr<message_filters::Subscriber<bob_interfaces::msg::Tracking>> sub_tracking_;
    std::shared_ptr<message_filters::Subscriber<bob_camera::msg::CameraInfo>> sub_camera_info_;
    std::shared_ptr<message_filters::TimeSynchronizer<bob_interfaces::msg::Tracking, bob_camera::msg::CameraInfo>> time_synchronizer_;

    int number_seconds_save_;
    double video_fps_;
    size_t current_end_frame_;
    cv::Size prev_frame_size_;
    bool recording_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(RecordManager)