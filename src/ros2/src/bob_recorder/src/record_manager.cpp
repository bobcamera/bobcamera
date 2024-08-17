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
#include "bob_interfaces/srv/recording_request.hpp"
#include "parameter_lifecycle_node.hpp"
#include "image_utils.hpp"
#include "image_recorder.hpp"
#include "json_recorder.hpp"
#include "video_recorder.hpp"

class RecordManager 
    : public ParameterLifeCycleNode 
{
public:
    COMPOSITION_PUBLIC
    explicit RecordManager(const rclcpp::NodeOptions& options)
        : ParameterLifeCycleNode("recorder_manager", options)
        , pub_qos_profile_(4)
        , sub_qos_profile_(10)
    {
    }

    CallbackReturn on_configure(const rclcpp_lifecycle::State &)
    {
        log_info("Configuring");

        init();

        return CallbackReturn::SUCCESS;
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
        prev_frame_width_ = 0;
        prev_frame_height_ = 0;

        declare_node_parameters();

        time_synchronizer_ = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::Image, 
            bob_interfaces::msg::Tracking, bob_camera::msg::CameraInfo>>(*sub_frame_, *sub_fg_frame_, *sub_tracking_, *sub_camera_info_, 10);
        time_synchronizer_->registerCallback(&RecordManager::process_recordings, this);

        // TODO: Recreate this when fps or seconds save change
        img_recorder_ = std::make_unique<ImageRecorder>(total_pre_frames_);
        json_recorder_ = std::make_unique<JsonRecorder>(total_pre_frames_);
        video_recorder_ = std::make_unique<VideoRecorder>(total_pre_frames_);
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
                log_info("Dated directory created: %s", dated_directory_.c_str());
                create_subdirectory(dated_directory_, "allsky");
                create_subdirectory(dated_directory_, "heatmaps");
                create_subdirectory(dated_directory_, "json");

                return true;
            } 

            // log_error("Failed to create dated directory: %s", dated_directory_.c_str());
            return false;
        } 

        log_info("Parent recordings directory doesn't exist: %s", dirPath.c_str());
        return false;
    }

    static std::string generate_filename(const sensor_msgs::msg::Image::SharedPtr & image_msg)
    {  
        auto stamp = rclcpp::Time(image_msg->header.stamp);
        std::ostringstream oss;
        oss << unsigned(stamp.seconds());
        return oss.str();
    } 

    void declare_node_parameters() 
    {
        std::vector<ParameterLifeCycleNode::ActionParam> params = {
            ParameterLifeCycleNode::ActionParam(
                rclcpp::Parameter("recordings_directory", "."), 
                [this](const rclcpp::Parameter& param) 
                {
                    recordings_directory_ = param.as_string();
                    create_dir(recordings_directory_);
                }
            ),
            ParameterLifeCycleNode::ActionParam(
                rclcpp::Parameter("recording_request_service_topic", "bob/recording/update"), 
                [this](const rclcpp::Parameter& param) 
                {
                    recording_request_service_ = create_service<bob_interfaces::srv::RecordingRequest>(param.as_string(),
                            [this](const std::shared_ptr<bob_interfaces::srv::RecordingRequest::Request> request, std::shared_ptr<bob_interfaces::srv::RecordingRequest::Response> response)
                                {change_recording_enabled_request(request, response);});
                }
            ),
            ParameterLifeCycleNode::ActionParam(
                rclcpp::Parameter("state_publisher_topic", "bob/recording/state"), 
                [this](const rclcpp::Parameter& param) 
                {
                    state_publisher_ = create_publisher<bob_interfaces::msg::RecordingState>(param.as_string(), pub_qos_profile_);
                }
            ),
            ParameterLifeCycleNode::ActionParam(
                rclcpp::Parameter("img_topic", "bob/frames/allsky/original"), 
                [this](const rclcpp::Parameter& param) 
                {
                    img_topic_ = param.as_string();
                    sub_frame_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image, rclcpp_lifecycle::LifecycleNode>>(shared_from_this(), img_topic_, sub_qos_profile_.get_rmw_qos_profile());
                }
            ),
            ParameterLifeCycleNode::ActionParam(
                rclcpp::Parameter("tracking_topic", "bob/tracker/tracking"), 
                [this](const rclcpp::Parameter& param) 
                {
                    tracking_topic_ = param.as_string();
                    sub_tracking_ = std::make_shared<message_filters::Subscriber<bob_interfaces::msg::Tracking, rclcpp_lifecycle::LifecycleNode>>(shared_from_this(), tracking_topic_, sub_qos_profile_.get_rmw_qos_profile());
                }
            ),
            ParameterLifeCycleNode::ActionParam(
                rclcpp::Parameter("fg_img_topic", "bob/frames/foreground_mask"), 
                [this](const rclcpp::Parameter& param) 
                {
                    fg_img_topic_ = param.as_string();
                    sub_fg_frame_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image, rclcpp_lifecycle::LifecycleNode>>(shared_from_this(), fg_img_topic_, sub_qos_profile_.get_rmw_qos_profile());
                }
            ),
            ParameterLifeCycleNode::ActionParam(
                rclcpp::Parameter("camera_info_topic", "bob/camera/all_sky/camera_info"), 
                [this](const rclcpp::Parameter& param)
                {
                    camera_info_topic_ = param.as_string();
                    sub_camera_info_ = std::make_shared<message_filters::Subscriber<bob_camera::msg::CameraInfo, rclcpp_lifecycle::LifecycleNode>>(shared_from_this(), camera_info_topic_, sub_qos_profile_.get_rmw_qos_profile());
                }
            ),
            ParameterLifeCycleNode::ActionParam(
                rclcpp::Parameter("prefix", "video"), 
                [this](const rclcpp::Parameter& param) 
                {
                    prefix_str_ = param.as_string();
                }
            ),            
            ParameterLifeCycleNode::ActionParam(
                rclcpp::Parameter("codec", "avc1"), 
                [this](const rclcpp::Parameter& param)
                {
                    codec_str_ = param.as_string();
                }
            ),
            ParameterLifeCycleNode::ActionParam(
                rclcpp::Parameter("pipeline", "appsrc ! videoconvert ! x264enc ! mp4mux ! filesink location="), 
                [this](const rclcpp::Parameter& param)
                {
                    pipeline_str_ = param.as_string();
                }
            ),            
            ParameterLifeCycleNode::ActionParam(
                rclcpp::Parameter("video_fps", 30.0), 
                [this](const rclcpp::Parameter& param) 
                {
                    video_fps_ = param.as_double();
                    total_pre_frames_ = (size_t)(number_seconds_save_ * video_fps_);
                }
            ),
            ParameterLifeCycleNode::ActionParam(
                rclcpp::Parameter("seconds_save", 2), 
                [this](const rclcpp::Parameter& param) 
                {
                    number_seconds_save_ = static_cast<int>(param.as_int());
                    total_pre_frames_ = (size_t)(number_seconds_save_ * video_fps_);
                }
            ),
        };
        add_action_parameters(params);
    }

    // Callback function
    void process_recordings(const sensor_msgs::msg::Image::SharedPtr & image_msg,
                            const sensor_msgs::msg::Image::SharedPtr & image_fg_msg,
                            const bob_interfaces::msg::Tracking::SharedPtr & tracking_msg,
                            const bob_camera::msg::CameraInfo::SharedPtr & camera_info_msg)
    {
        try
        {
            cv::Mat img;
            ImageUtils::convert_image_msg(image_msg, img, true);

            if (current_state_ == RecordingStateEnum::BeforeStart && prev_frame_width_ == 0 && prev_frame_height_ == 0)
            {
                // Initialize prev_frame_width_ and prev_frame_height_ with the dimensions of the first received image
                prev_frame_width_ = img.cols;
                prev_frame_height_ = img.rows;
            }
            else if (img.rows != prev_frame_height_ || img.cols != prev_frame_width_ )
            {
                log_info("Frame dimensions changed. ");
                prev_frame_height_ = img.rows;
                prev_frame_width_ = img.cols;
                current_state_ = RecordingStateEnum::AfterEnd;
                current_end_frame_ = 0;
            }

            cv::Mat fg_img;
            ImageUtils::convert_image_msg(image_fg_msg, fg_img, false);
            Json::Value json_data;

            switch (current_state_) 
            {
            case RecordingStateEnum::BeforeStart:
                if (tracking_msg->state.trackable > 0) 
                {
                    recording_ = true;
                    video_fps_ = static_cast<double>(camera_info_msg->fps);
                    log_info("Starting track recording...");
                    current_state_ = RecordingStateEnum::BetweenEvents;
                    base_filename_ = generate_filename(image_msg);

                    if (auto current_date = get_current_date_as_str(); current_date != date_)
                    {
                        create_dated_dir(recordings_directory_);
                        date_ = current_date;
                    }

                    const std::string full_path = dated_directory_ + "/allsky/" + prefix_str_ + base_filename_ + ".mp4";
                    const std::string out_pipeline = pipeline_str_ + full_path;
                    video_recorder_->open_new_video(full_path, codec_str_, video_fps_, img.size());
                    img_recorder_->update_frame_for_drawing(img);
                } 
                else 
                {
                    json_data = JsonRecorder::build_json_value(tracking_msg, false);
                    json_recorder_->add_to_pre_buffer(json_data, false);
                    video_recorder_->add_to_pre_buffer(img);
                }
                break;

            case RecordingStateEnum::BetweenEvents:
                json_data = JsonRecorder::build_json_value(tracking_msg, true);
                json_recorder_->add_to_buffer(json_data, false);
                video_recorder_->write_frame(img);
                img_recorder_->accumulate_mask(fg_img, img.size());

                for (const auto & detection : tracking_msg->detections)
                {
                    if (detection.state == 2) // ActiveTarget
                    {
                        const auto & bbox = detection.bbox;
                        const double area = bbox.size_x * bbox.size_y; 
                        img_recorder_->store_trajectory_point(detection.id, cv::Point(static_cast<int>(bbox.center.position.x), static_cast<int>(bbox.center.position.y)), area);
                    }
                }

                if (tracking_msg->state.trackable == 0) 
                {
                    current_state_ = RecordingStateEnum::AfterEnd;
                    current_end_frame_ = total_pre_frames_;
                }
                break;

            case RecordingStateEnum::AfterEnd:
                if (current_end_frame_ == 0) 
                {
                    recording_ = false;
                    log_info("Ending track recording...");
                    std::string full_path = dated_directory_ + "/heatmaps/" + base_filename_ + ".jpg";
                    img_recorder_->write_image(full_path);

                    Json::Value json_camera_info = JsonRecorder::build_json_camera_info(camera_info_msg);
                    json_recorder_->add_to_buffer(json_camera_info, true);
                    
                    std::string json_full_path = dated_directory_ + "/json/" + base_filename_ + ".json";
                    json_recorder_->write_buffer_to_file(json_full_path);

                    img_recorder_->reset();
                    video_recorder_->close_video();
                    current_state_ = RecordingStateEnum::BeforeStart;
                }
                else 
                {
                    json_data = JsonRecorder::build_json_value(tracking_msg, false);
                    json_recorder_->add_to_buffer(json_data, false);
                    video_recorder_->write_frame(img);
                    img_recorder_->accumulate_mask(fg_img, img.size());

                    --current_end_frame_;
                    if (tracking_msg->state.trackable > 0) 
                    {
                        current_state_ = RecordingStateEnum::BetweenEvents;
                        video_recorder_->clear_pre_buffer();
                    }
                }
                video_recorder_->add_to_pre_buffer(img);
                break;

            case RecordingStateEnum::Disabled:
                if (recording_) 
                {
                    recording_ = false;
                    log_info("Ending track recording...");
                    std::string full_path = dated_directory_ + "/heatmaps/" + base_filename_ + ".jpg";
                    img_recorder_->write_image(full_path);

                    Json::Value json_camera_info = JsonRecorder::build_json_camera_info(camera_info_msg);
                    json_recorder_->add_to_buffer(json_camera_info, true);
                    
                    std::string json_full_path = dated_directory_ + "/json/" + base_filename_ + ".json";
                    json_recorder_->write_buffer_to_file(json_full_path);

                    img_recorder_->reset();
                    video_recorder_->close_video();
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
    std::unique_ptr<ImageRecorder> img_recorder_;
    std::unique_ptr<VideoRecorder> video_recorder_;
    std::unique_ptr<JsonRecorder> json_recorder_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string trackingTopic_;
    RecordingStateEnum current_state_;
    rclcpp::TimerBase::SharedPtr one_shot_timer_;
    std::string recordings_directory_;
    std::string dated_directory_;
    std::string date_;
    std::string base_filename_;
    std::string img_topic_;
    std::string tracking_topic_;
    std::string fg_img_topic_;
    std::string camera_info_topic_;
    std::string codec_str_;
    std::string pipeline_str_;
    std::string prefix_str_;
    rclcpp::Service<bob_interfaces::srv::RecordingRequest>::SharedPtr recording_request_service_;
    rclcpp::Publisher<bob_interfaces::msg::RecordingState>::SharedPtr state_publisher_;    
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image, rclcpp_lifecycle::LifecycleNode>> sub_frame_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image, rclcpp_lifecycle::LifecycleNode>> sub_fg_frame_;
    std::shared_ptr<message_filters::Subscriber<bob_interfaces::msg::Tracking, rclcpp_lifecycle::LifecycleNode>> sub_tracking_;
    std::shared_ptr<message_filters::Subscriber<bob_camera::msg::CameraInfo, rclcpp_lifecycle::LifecycleNode>> sub_camera_info_;
    std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::Image,
    bob_interfaces::msg::Tracking, bob_camera::msg::CameraInfo>> time_synchronizer_;

    double video_fps_;
    size_t current_end_frame_;
    size_t total_pre_frames_;
    int number_seconds_save_;
    int prev_frame_width_;
    int prev_frame_height_;
    bool recording_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(RecordManager)