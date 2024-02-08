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
#include "parameter_node.hpp"
#include "image_utils.hpp"
#include "image_recorder.hpp"
#include "json_recorder.hpp"
#include "video_recorder.hpp"


class RecordManager : public ParameterNode {
public:
    COMPOSITION_PUBLIC
    explicit RecordManager(const rclcpp::NodeOptions& options)
    : ParameterNode("recorder_manager", options)
    , current_state_(RecordingState::BeforeStart)
    {
        one_shot_timer_ = this->create_wall_timer(
            std::chrono::seconds(2), 
            [this]() {
                this->init(); 
                this->one_shot_timer_.reset();  
            }
        );
    }

private:
    enum class RecordingState {
        BeforeStart,
        BetweenEvents,
        AfterEnd
    };

    void init()
    {
        RCLCPP_INFO(get_logger(), "Initializing RecordManager");

        declare_node_parameters();

        rclcpp::QoS sub_qos_profile{10};
        sub_qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        sub_qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);
        sub_qos_profile.history(rclcpp::HistoryPolicy::KeepLast);
        auto rmw_qos_profile = sub_qos_profile.get_rmw_qos_profile();

        sub_frame_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(shared_from_this(), img_topic_, rmw_qos_profile);
        sub_fg_frame_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(shared_from_this(), fg_img_topic_, rmw_qos_profile);
        sub_tracking_ = std::make_shared<message_filters::Subscriber<bob_interfaces::msg::Tracking>>(shared_from_this(), tracking_topic_, rmw_qos_profile);
        sub_camera_info_ = std::make_shared<message_filters::Subscriber<bob_camera::msg::CameraInfo>>(shared_from_this(), camera_info_topic_, rmw_qos_profile);

        time_synchronizer_ = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::Image, 
            bob_interfaces::msg::Tracking, bob_camera::msg::CameraInfo>>(*sub_frame_, *sub_fg_frame_, *sub_tracking_, *sub_camera_info_, 10);

        time_synchronizer_->registerCallback(&RecordManager::process_recordings, this);

        img_recorder_ = std::make_unique<ImageRecorder>(total_pre_frames_);
        json_recorder_ = std::make_unique<JsonRecorder>(total_pre_frames_);
        video_recorder_ = std::make_unique<VideoRecorder>(total_pre_frames_);
    };

    std::string get_current_date() 
    {
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        std::tm local_tm = *std::localtime(&time_t);

        std::ostringstream oss;
        oss << std::put_time(&local_tm, "%Y%m%d");

        return oss.str();
    }

    static bool create_dir(const std::string& path) 
    {
        std::filesystem::path dirPath = path;

        if (!std::filesystem::exists(dirPath)) 
        {
            return std::filesystem::create_directories(dirPath);
        }
        return true;
    }

    void create_subdirectory(const std::filesystem::path& parent, const std::string& subdirectory) 
    {
        std::filesystem::path subDirPath = parent / subdirectory;
        if (std::filesystem::create_directory(subDirPath)) 
        {
            RCLCPP_INFO(get_logger(), "Subdirectory created: %s", subDirPath.c_str());
        } 
        else 
        {
            RCLCPP_ERROR(get_logger(), "Failed to create subdirectory: %s", subDirPath.c_str());
        }
    }
    
    bool create_dated_dir(const std::string& path) 
    {
        std::filesystem::path dirPath = path;

        if (std::filesystem::exists(dirPath)) 
        {
            dated_directory_ = dirPath / get_current_date();

            if (std::filesystem::create_directory(dated_directory_)) 
            {
                RCLCPP_INFO(get_logger(), "Dated directory created: %s", dated_directory_.c_str());
                create_subdirectory(dated_directory_, "allsky");
                create_subdirectory(dated_directory_, "heatmaps");
                create_subdirectory(dated_directory_, "json");

                return true;
            } 
            else 
            {
                // RCLCPP_ERROR(get_logger(), "Failed to create dated directory: %s", dated_directory_.c_str());
                return false;
            }
        } 
        else 
        {
            RCLCPP_INFO(get_logger(), "Parent recordings directory doesn't exist: %s", dirPath.c_str());
            return false;
        }
    }

    static std::string generate_filename(const sensor_msgs::msg::Image::SharedPtr& image_msg)
    {  
        auto stamp = rclcpp::Time(image_msg->header.stamp);
        std::ostringstream oss;
        oss << unsigned(stamp.seconds());
        return oss.str();
    } 

    void declare_node_parameters() {
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
                rclcpp::Parameter("img_topic", "bob/camera/all_sky/bayer"), 
                [this](const rclcpp::Parameter& param) {img_topic_ = param.as_string();}
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("tracking_topic", "bob/tracker/tracking"), 
                [this](const rclcpp::Parameter& param) {tracking_topic_ = param.as_string();}
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("fg_img_topic", "bob/frames/all_sky/foreground_mask"), 
                [this](const rclcpp::Parameter& param) {fg_img_topic_ = param.as_string();}
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("camera_info_topic", "bob/camera/all_sky/camera_info"), 
                [this](const rclcpp::Parameter& param) {camera_info_topic_ = param.as_string();}
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("prefix", "video"), 
                [this](const rclcpp::Parameter& param) {prefix_str_ = param.as_string();}
            ),            
            ParameterNode::ActionParam(
                rclcpp::Parameter("codec", "avc1"), 
                [this](const rclcpp::Parameter& param) {codec_str_ = param.as_string();}
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("video_fps", 30.0), 
                [this](const rclcpp::Parameter& param) 
                {
                    video_fps_ = param.as_double();
                    total_pre_frames_ = (size_t)(number_seconds_save_ * video_fps_);
                }
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("seconds_save", 2), 
                [this](const rclcpp::Parameter& param) 
                {
                    number_seconds_save_ = param.as_int();
                    total_pre_frames_ = (size_t)(number_seconds_save_ * video_fps_);
                }
            ),
        };
        add_action_parameters(params);
    }

    // Callback function
    void process_recordings(const sensor_msgs::msg::Image::SharedPtr& image_msg,
                            const sensor_msgs::msg::Image::SharedPtr& image_fg_msg,
                            const bob_interfaces::msg::Tracking::SharedPtr& tracking_msg,
                            const bob_camera::msg::CameraInfo::SharedPtr& camera_info_msg)
    {
        try
        {          
            std::unique_lock<std::mutex> lock(buffer_mutex_);

            cv::Mat img;
            ImageUtils::convert_image_msg(image_msg, img, true);

            cv::Mat fg_img;
            ImageUtils::convert_image_msg(image_fg_msg, fg_img, false);

            Json::Value json_data;
            switch (current_state_) 
            {
            case RecordingState::BeforeStart:
                if (tracking_msg->state.trackable > 0) 
                {
                    RCLCPP_INFO(get_logger(), "Starting track recording...");
                    current_state_ = RecordingState::BetweenEvents;
                    base_filename_ = generate_filename(image_msg);

                    std::string current_date = get_current_date();
                    if(current_date != date_)
                    {
                        create_dated_dir(recordings_directory_);
                        date_ = current_date;
                    }

                    std::string full_path = dated_directory_ + "/allsky/" + prefix_str_ + base_filename_ + ".mp4";
                    video_recorder_->open_new_video(full_path, codec_str_, video_fps_, img.size(), img.channels() == 3);

                    img_recorder_->update_frame_for_drawing(img);
                    img_recorder_->accumulate_pre_buffer_images();
                } 
                else 
                {
                    json_data = JsonRecorder::build_json_value(image_msg, tracking_msg, false);
                    json_recorder_->add_to_pre_buffer(json_data, false);
                    video_recorder_->add_to_pre_buffer(img);
                    img_recorder_->add_to_pre_buffer(fg_img);
                }
                break;

            case RecordingState::BetweenEvents:
                json_data = JsonRecorder::build_json_value(image_msg, tracking_msg, true);
                json_recorder_->add_to_buffer(json_data, false);
                video_recorder_->write_frame(img);
                img_recorder_->accumulate_mask(fg_img);

                if (tracking_msg->state.trackable == 0) 
                {
                    current_state_ = RecordingState::AfterEnd;
                    current_end_frame_ = total_pre_frames_;
                }
                break;

            case RecordingState::AfterEnd:
                if (current_end_frame_ == 0) 
                {
                    RCLCPP_INFO(get_logger(), "Ending track recording...");
                    std::string full_path = dated_directory_ + "/heatmaps/" + base_filename_ + ".jpg";
                    img_recorder_->write_image(full_path);

                    Json::Value json_camera_info = JsonRecorder::build_json_camera_info(camera_info_msg);
                    json_recorder_->add_to_buffer(json_camera_info, true);
                    
                    std::string json_full_path = dated_directory_ + "/json/" + base_filename_ + ".json";
                    json_recorder_->write_buffer_to_file(json_full_path);

                    img_recorder_->reset();
                    video_recorder_->close_video();
                    current_state_ = RecordingState::BeforeStart;
                }
                else 
                {
                    json_data = JsonRecorder::build_json_value(image_msg, tracking_msg, false);
                    json_recorder_->add_to_buffer(json_data, false);
                    video_recorder_->write_frame(img);
                    img_recorder_->accumulate_mask(fg_img);

                    --current_end_frame_;
                    if (tracking_msg->state.trackable > 0) 
                    {
                        current_state_ = RecordingState::BetweenEvents;
                        video_recorder_->clear_pre_buffer();
                    }
                }
                video_recorder_->add_to_pre_buffer(img);
                img_recorder_->add_to_pre_buffer(fg_img);
                break;
            }
        }
        catch (std::exception &e)
        {
            RCLCPP_ERROR(get_logger(), "exception: %s", e.what());
        }
    };

    std::unique_ptr<ImageRecorder> img_recorder_;
    std::unique_ptr<VideoRecorder> video_recorder_;
    std::unique_ptr<JsonRecorder> json_recorder_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string trackingTopic_;
    RecordingState current_state_;
    rclcpp::TimerBase::SharedPtr one_shot_timer_;
    std::mutex buffer_mutex_;
    std::string recordings_directory_;
    std::string dated_directory_;
    std::string date_;
    std::string base_filename_;
    std::string img_topic_;
    std::string tracking_topic_;
    std::string fg_img_topic_;
    std::string camera_info_topic_;
    std::string codec_str_;
    std::string prefix_str_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> sub_frame_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> sub_fg_frame_;
    std::shared_ptr<message_filters::Subscriber<bob_interfaces::msg::Tracking>> sub_tracking_;
    std::shared_ptr<message_filters::Subscriber<bob_camera::msg::CameraInfo>> sub_camera_info_;
    std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, sensor_msgs::msg::Image,
         bob_interfaces::msg::Tracking, bob_camera::msg::CameraInfo>> time_synchronizer_;

    double video_fps_;
    size_t current_end_frame_;
    size_t total_pre_frames_;
    int number_seconds_save_;
};


int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RecordManager>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}

RCLCPP_COMPONENTS_REGISTER_NODE(RecordManager)