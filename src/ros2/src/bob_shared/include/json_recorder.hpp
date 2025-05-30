#pragma once

#include <filesystem>
#include <fstream>
#include <json/json.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "bob_interfaces/msg/tracking.hpp"
#include "bob_camera/msg/camera_info.hpp"
#include "../../bob_camera/src/tracking.hpp"

class JsonRecorder
{
public:
    explicit JsonRecorder(int pre_buffer_size, size_t auto_flush_size = 1000)
        : max_pre_buffer_size_(pre_buffer_size)
        , auto_flush_size_(auto_flush_size)
    {
    }

    std::string get_output_file() const noexcept
    {
        return current_filename_;
    }

    void open(const std::string &filename)
    {
        current_filename_ = filename;
        std::filesystem::create_directories(std::filesystem::path(current_filename_).parent_path());
        write_buffer_to_file(json_pre_buffer_);        
        json_buffer_.clear();
    }

    void close() noexcept
    {
        if (!current_filename_.empty())
        {
            write_buffer_to_file(json_buffer_);
            current_filename_.clear();
        }
        json_pre_buffer_.clear();
    }

    void add_to_pre_buffer(const Json::Value &jsonValue, bool prepend = false) noexcept
    {
        if (prepend)
        {
            json_pre_buffer_.push_front(jsonValue);
        }
        else
        {
            json_pre_buffer_.push_back(jsonValue);
        }

        while (json_pre_buffer_.size() > max_pre_buffer_size_)
        {
            json_pre_buffer_.pop_front();
        }
    }

    void add_to_buffer(const Json::Value &jsonValue, bool prepend = false) noexcept
    {
        if (prepend)
        {
            json_buffer_.push_front(jsonValue);
        }
        else
        {
            json_buffer_.push_back(jsonValue);
        }

        // Auto-flush if buffer gets too large
        if (json_buffer_.size() >= auto_flush_size_ && !current_filename_.empty())
        {
            write_buffer_to_file(json_buffer_);
        }
    }

    static Json::Value build_json_value(const std::shared_ptr<bob_camera::Tracking> &tracking_msg,
                                        bool include_detections) noexcept
    {
        Json::Value jsonValue;

        auto time_stamp = rclcpp::Time(tracking_msg->header_ptr->stamp);
        int64_t time_in_nanosecs = time_stamp.nanoseconds();

        jsonValue["time_ns"] = time_in_nanosecs;
        jsonValue["frame_id"] = tracking_msg->header_ptr->frame_id;
        jsonValue["trackable"] = tracking_msg->state.trackable;

        if (include_detections)
        {
            Json::Value jsonDetectionsArray;
            for (const auto &detection : tracking_msg->detections)
            {
                Json::Value jsonDetection;
                jsonDetection["id"] = detection.id;
                jsonDetection["state"] = detection.state;

                Json::Value jsonBbox;
                jsonBbox["x"] = detection.bbox.x;
                jsonBbox["y"] = detection.bbox.y;
                jsonBbox["width"] = detection.bbox.width;
                jsonBbox["height"] = detection.bbox.height;
                jsonDetection["bbox"] = jsonBbox;
                jsonDetectionsArray.append(jsonDetection);
            }
            jsonValue["detections"] = jsonDetectionsArray;
        }

        return jsonValue;
    }

    static Json::Value build_json_camera_info(const bob_camera::msg::CameraInfo &camera_info_msg) noexcept
    {
        Json::Value jsonValue;
        Json::Value jsonCameraInfo;
        jsonCameraInfo["id"] = camera_info_msg.id;
        jsonCameraInfo["manufacturer"] = camera_info_msg.manufacturer;
        jsonCameraInfo["model"] = camera_info_msg.model;
        jsonCameraInfo["serial_num"] = camera_info_msg.serial_num;
        jsonCameraInfo["firmware_version"] = camera_info_msg.firmware_version;
        jsonCameraInfo["num_configurations"] = camera_info_msg.num_configurations;
        jsonCameraInfo["encoding"] = camera_info_msg.encoding;
        jsonCameraInfo["frame_width"] = camera_info_msg.frame_width;
        jsonCameraInfo["frame_height"] = camera_info_msg.frame_height;
        jsonCameraInfo["fps"] = camera_info_msg.fps;
        jsonValue["camera_info"] = jsonCameraInfo;

        return jsonValue;
    }

private:
    std::deque<Json::Value> json_pre_buffer_;
    std::deque<Json::Value> json_buffer_;
    size_t max_pre_buffer_size_;
    size_t auto_flush_size_;
    std::string current_filename_;

    bool write_buffer_to_file(std::deque<Json::Value> &buffer) noexcept
    {
        if (!buffer.empty())
        {
            bool file_exists = std::filesystem::exists(current_filename_);
            bool isEmpty = true;

            if (file_exists)
            {
                std::ifstream check_file(current_filename_);
                check_file.seekg(0, std::ios::end);
                isEmpty = check_file.tellg() == 0;
                check_file.close();

                if (!isEmpty)
                {
                    // Properly remove the closing bracket and trailing whitespace
                    std::ifstream read_file(current_filename_);
                    std::string content((std::istreambuf_iterator<char>(read_file)),
                                      std::istreambuf_iterator<char>());
                    read_file.close();

                    // Find the last ']' and remove it along with trailing whitespace
                    size_t last_bracket = content.find_last_of(']');
                    if (last_bracket != std::string::npos)
                    {
                        content = content.substr(0, last_bracket);
                        // Remove trailing whitespace before the bracket
                        while (!content.empty() && std::isspace(content.back()))
                        {
                            content.pop_back();
                        }
                    }

                    // Write back the content without the closing bracket
                    std::ofstream temp_file(current_filename_);
                    temp_file << content;
                    temp_file.close();
                }
            }

            std::ofstream file(current_filename_, std::ios::app);

            if (!file.is_open())
            {
                return false;
            }

            if (isEmpty)
            {
                file << "[" << std::endl;
            }

            Json::StreamWriterBuilder builder;
            builder["indentation"] = "  "; // Add proper indentation
            std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());

            for (size_t i = 0; i < buffer.size(); ++i)
            {
                if (!isEmpty || i > 0) // Add comma before each entry except the very first
                {
                    file << "," << std::endl;
                }
                writer->write(buffer[i], &file);
            }

            file << std::endl << "]" << std::endl;
            file.close();
            buffer.clear();

            return true;
        }

        return false;
    }
};
