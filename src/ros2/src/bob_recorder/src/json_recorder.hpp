#include <filesystem>
#include <fstream>
#include <json/json.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "bob_interfaces/msg/tracking.hpp"
#include "bob_camera/msg/camera_info.hpp"

class JsonRecorder
{
public:
    explicit JsonRecorder(int pre_buffer_size) 
        : max_pre_buffer_size_(pre_buffer_size)
    {
    }

    void add_to_pre_buffer(const Json::Value & jsonValue, bool prepend = false) 
    {
        if (prepend) 
        {
            json_buffer_.push_front(jsonValue);
        } 
        else 
        {
            json_buffer_.push_back(jsonValue);
        }

        while (json_buffer_.size() > max_pre_buffer_size_) 
        {
            json_buffer_.pop_front();
        }
    }

    void add_to_buffer(const Json::Value& jsonValue, bool prepend = false) 
    {
        if (prepend) 
        {
            json_buffer_.push_front(jsonValue);
        } 
        else 
        {
            json_buffer_.push_back(jsonValue);
        }
    }

    bool write_buffer_to_file(const std::string & filename) 
    {
        if (!json_buffer_.empty()) 
        {
            std::ofstream file(filename, std::ios::app);

            if (!file.is_open()) 
            {
                return false;
            }

            bool isEmpty = file.tellp() == 0; 
            if (isEmpty) 
            {
                file << "[" << std::endl;
            }

            Json::StreamWriterBuilder builder;
            std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());

            for (size_t i = 0; i < json_buffer_.size(); ++i) 
            {
                if (i > 0 || !isEmpty) 
                {
                    file << "," << std::endl;
                }
                writer->write(json_buffer_[i], &file);
            }
            
            file << "]" << std::endl;
            file.close();
            json_buffer_.clear();
            return true;
        }

        return false;
    }

    static Json::Value build_json_value(const sensor_msgs::msg::Image::SharedPtr & image_msg,
                                        const bob_interfaces::msg::Tracking::SharedPtr & tracking_msg,
                                        bool include_detections,
                                        int x_offset,
                                        int y_offset) 
    {
        Json::Value jsonValue;

        auto time_stamp = rclcpp::Time(image_msg->header.stamp);
        int64_t time_in_nanosecs = time_stamp.nanoseconds();

        jsonValue["time_ns"] = time_in_nanosecs;
        jsonValue["frame_id"] = image_msg->header.frame_id;
        jsonValue["trackable"] = tracking_msg->state.trackable;

        if (include_detections) 
        {
            Json::Value jsonDetectionsArray;
            for (const auto& detection : tracking_msg->detections) 
            {
                Json::Value jsonDetection;
                jsonDetection["id"] = detection.id;
                jsonDetection["state"] = detection.state;

                Json::Value jsonBbox;
                jsonBbox["x"] = detection.bbox.center.position.x + x_offset;
                jsonBbox["y"] = detection.bbox.center.position.y + y_offset;
                jsonBbox["width"] = detection.bbox.size_x;
                jsonBbox["height"] = detection.bbox.size_y;
                jsonDetection["bbox"] = jsonBbox;
                jsonDetectionsArray.append(jsonDetection);
            }
            jsonValue["detections"] = jsonDetectionsArray;
        }
        
        return jsonValue;
    }

    static Json::Value build_json_camera_info(const bob_camera::msg::CameraInfo::SharedPtr& camera_info_msg) 
    { 
        Json::Value jsonValue;
        Json::Value jsonCameraInfo;
        jsonCameraInfo["id"] = camera_info_msg->id;
        jsonCameraInfo["manufacturer"] = camera_info_msg->manufacturer;
        jsonCameraInfo["model"] = camera_info_msg->model;
        jsonCameraInfo["serial_num"] = camera_info_msg->serial_num;
        jsonCameraInfo["firmware_version"] = camera_info_msg->firmware_version;
        jsonCameraInfo["num_configurations"] = camera_info_msg->num_configurations;
        jsonCameraInfo["encoding"] = camera_info_msg->encoding;
        jsonCameraInfo["frame_width"] = camera_info_msg->frame_width;
        jsonCameraInfo["frame_height"] = camera_info_msg->frame_height;
        jsonCameraInfo["fps"] = camera_info_msg->fps;
        jsonValue["camera_info"] = jsonCameraInfo;

        return jsonValue;
    }

private: 
    std::deque<Json::Value> json_buffer_;
    size_t max_pre_buffer_size_;
};
