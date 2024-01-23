#include <filesystem>
#include <mutex>
#include <chrono>
#include <fstream>
#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "bob_camera/msg/camera_info.hpp"
#include <json/json.h>

#include "bob_interfaces/msg/tracking.hpp"
#include "image_utils.hpp"
#include "parameter_node.hpp"

#include <visibility_control.h>

class JsonRecorder : public ParameterNode {
public:
    COMPOSITION_PUBLIC
    explicit JsonRecorder(const rclcpp::NodeOptions& options)
        : ParameterNode("json_recorder", options),
          current_state_(RecordingState::BeforeStart),
          is_camera_info_written_(false) {
        timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&JsonRecorder::init, this));
    }

private:
    enum class RecordingState {
        BeforeStart,
        BetweenEvents,
        AfterEnd
    };

    std::string json_directory_;
    std::string base_filename_;
    std::string img_topic_;
    std::string tracking_topic_;
    std::shared_ptr<message_filters::Subscriber<bob_interfaces::msg::Tracking>> sub_tracking_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> sub_masked_frame_;
    std::shared_ptr<message_filters::Subscriber<bob_camera::msg::CameraInfo>> sub_camera_info_;
    std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, bob_interfaces::msg::Tracking, bob_camera::msg::CameraInfo>> time_synchronizer_;
    RecordingState current_state_;
    std::mutex buffer_mutex_;
    std::deque<Json::Value> json_pre_buffer_;
    Json::Value json_camera_info_;
    rclcpp::TimerBase::SharedPtr timer_;
    double video_fps_;
    size_t current_end_frame_;
    size_t total_pre_frames_;
    int number_seconds_save_;
    bool is_camera_info_written_;

    void init() {
        RCLCPP_INFO(get_logger(), "Initializing JsonRecorder");

        timer_->cancel();
        
        declare_node_parameters();

        rclcpp::QoS sub_qos_profile{10};
        sub_qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        sub_qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);
        sub_qos_profile.history(rclcpp::HistoryPolicy::KeepLast);
        auto rmw_qos_profile = sub_qos_profile.get_rmw_qos_profile();

        sub_masked_frame_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(shared_from_this(), img_topic_, rmw_qos_profile);
        sub_tracking_ = std::make_shared<message_filters::Subscriber<bob_interfaces::msg::Tracking>>(shared_from_this(), tracking_topic_, rmw_qos_profile);
        sub_camera_info_ = std::make_shared<message_filters::Subscriber<bob_camera::msg::CameraInfo>>(shared_from_this(), "bob/camera/all_sky/camera_info", rmw_qos_profile);

        time_synchronizer_ = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, bob_interfaces::msg::Tracking, bob_camera::msg::CameraInfo>>(*sub_masked_frame_, *sub_tracking_, *sub_camera_info_, 10);
        time_synchronizer_->registerCallback(&JsonRecorder::callback, this);
    }

    static bool create_dir(const std::string& path) 
    {
        std::filesystem::path dirPath = path;

        if (!std::filesystem::exists(dirPath)) 
        {
            return std::filesystem::create_directory(dirPath);
        }

        return true;
    }

    void declare_node_parameters() {
        std::vector<ParameterNode::ActionParam> params = {
            ParameterNode::ActionParam(
                rclcpp::Parameter("json_directory", "."), 
                [this](const rclcpp::Parameter& param) 
                {
                    json_directory_ = param.as_string();
                    create_dir(json_directory_);
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

    void start_json_recording(const std::string& base_filename)
    {
        if (!is_camera_info_written_) 
        {
            write_json_to_file(json_camera_info_, base_filename);
            is_camera_info_written_ = true;
        }
        for (const auto& json : json_pre_buffer_)
        {
            write_json_to_file(json, base_filename);
        }
        json_pre_buffer_.clear();
    }

    void add_to_json_ring_buffer(const Json::Value& jsonValue)
    {
        if (json_pre_buffer_.size() >= total_pre_frames_) 
        {
            json_pre_buffer_.pop_front();
        }
        json_pre_buffer_.push_back(jsonValue);
    }

    void write_json_to_file(const Json::Value& jsonData, const std::string& base_filename)
    {
        std::string filename = json_directory_ + "/" + base_filename + ".json";
        std::ofstream file(filename, std::ios::app);

        if (!file.is_open())
        {
            RCLCPP_ERROR(get_logger(), "Unable to open JSON file: %s", filename.c_str());
            return;
        }

        bool isEmpty = file.tellp() == 0; // Check if the file is empty
        if (!isEmpty && current_state_ != RecordingState::BeforeStart)
        {
            file << "," << std::endl;
        }

        if (isEmpty)
        {
            file << "[" << std::endl;
        }

        Json::StreamWriterBuilder builder;
        std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());
        writer->write(jsonData, &file);
        file.close();

        // maybe a better way?
        if (current_state_ == RecordingState::AfterEnd && current_end_frame_ == 0)
        {
            std::ofstream closingBracketFile(filename, std::ios::app);
            if (closingBracketFile.is_open())
            {
                closingBracketFile << "]" << std::endl;
                closingBracketFile.close();
            }
            else
            {
                RCLCPP_ERROR(get_logger(), "Unable to open JSON file for closing bracket: %s", filename.c_str());
            }
        }
    }

    static std::string generate_filename(const sensor_msgs::msg::Image::SharedPtr& image_msg)
    {  
        auto stamp = rclcpp::Time(image_msg->header.stamp);
        std::ostringstream oss;
        oss << unsigned(stamp.seconds());
        return oss.str();
    }

    void callback(const sensor_msgs::msg::Image::SharedPtr& image_msg,
                const bob_interfaces::msg::Tracking::SharedPtr& tracking_msg,
                const bob_camera::msg::CameraInfo::SharedPtr& camera_info_msg) 
    {
        try
        {
            std::unique_lock<std::mutex> lock(buffer_mutex_);

            auto time_stamp = rclcpp::Time(image_msg->header.stamp);
            int64_t time_in_nanosecs = time_stamp.nanoseconds();

            Json::Value jsonValue;

            jsonValue["time_ns"] = time_in_nanosecs;
            jsonValue["frame_id"] = image_msg->header.frame_id;
            jsonValue["trackable"] = tracking_msg->state.trackable;

            if (current_state_ == RecordingState::BetweenEvents || current_state_ == RecordingState::AfterEnd) {
                Json::Value jsonDetectionsArray;
                for (const auto& detection : tracking_msg->detections) 
                {
                    Json::Value jsonDetection;
                    jsonDetection["id"] = detection.id;
                    jsonDetection["state"] = detection.state;

                    Json::Value jsonBbox;
                    jsonBbox["x"] = detection.bbox.center.position.x;
                    jsonBbox["y"] = detection.bbox.center.position.y;
                    jsonBbox["width"] = detection.bbox.size_x;
                    jsonBbox["height"] = detection.bbox.size_y;
                    jsonDetection["bbox"] = jsonBbox;

                    jsonDetectionsArray.append(jsonDetection);
                }
                jsonValue["detections"] = jsonDetectionsArray;
            } 

            Json::Value jsonCameraInfo;
            jsonCameraInfo["id"] = camera_info_msg->id;
            jsonCameraInfo["manufacturer"] = camera_info_msg->manufacturer;
            jsonCameraInfo["model"] = camera_info_msg->model;
            jsonCameraInfo["serial_num"] = camera_info_msg->serial_num;
            jsonCameraInfo["firmware_version"] = camera_info_msg->firmware_version;
            jsonCameraInfo["num_configurations"] = camera_info_msg->num_configurations;
            jsonCameraInfo["encoding"] = camera_info_msg->encoding;
            json_camera_info_["camera_info"] = jsonCameraInfo;

            switch (current_state_) 
            {
            case RecordingState::BeforeStart:
                if (tracking_msg->state.trackable > 0) 
                {
                    current_state_ = RecordingState::BetweenEvents;
                    base_filename_ = generate_filename(image_msg);
                    start_json_recording(base_filename_);
                } 
                else 
                {
                    add_to_json_ring_buffer(jsonValue);
                }
                break;

            case RecordingState::BetweenEvents:
                write_json_to_file(jsonValue, base_filename_);
                if (tracking_msg->state.trackable == 0) {
                    current_state_ = RecordingState::AfterEnd;
                    current_end_frame_ = total_pre_frames_;
                }
                break;

            case RecordingState::AfterEnd:
                if (current_end_frame_ == 0) 
                {
                    current_state_ = RecordingState::BeforeStart;
                    is_camera_info_written_ = false;
                }
                else 
                {
                    --current_end_frame_;
                    write_json_to_file(jsonValue, base_filename_);

                    if (tracking_msg->state.trackable > 0) 
                    {
                        current_state_ = RecordingState::BetweenEvents;
                    }
                }
                break;
            }
        }
        catch (std::exception &e)
        {
            RCLCPP_ERROR(get_logger(), "exception: %s", e.what());
        }
    }

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JsonRecorder>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}


RCLCPP_COMPONENTS_REGISTER_NODE(JsonRecorder)
