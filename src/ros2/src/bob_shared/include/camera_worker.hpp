#pragma once
#ifndef __CAMERA_WORKER_H_
#define __CAMERA_WORKER_H_

#include <string>
#include <filesystem>

#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/region_of_interest.hpp>

#include <bob_interfaces/srv/camera_settings.hpp>
#include <bob_interfaces/srv/config_entry_update.hpp>

#include <bob_camera/msg/image_info.hpp>
#include <bob_camera/msg/camera_info.hpp>

#include "image_utils.hpp"
#include "parameter_node.hpp"

struct CameraWorkerParams
{
    enum class SourceType 
    {
        USB_CAMERA,
        VIDEO_FILE,
        RTSP_STREAM
    };

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_resized_publisher;
    rclcpp::Publisher<bob_camera::msg::ImageInfo>::SharedPtr image_info_publisher;
    rclcpp::Publisher<bob_camera::msg::CameraInfo>::SharedPtr camera_info_publisher;
    rclcpp::Publisher<sensor_msgs::msg::RegionOfInterest>::SharedPtr roi_publisher;

    int camera_id;
    int resize_height;
    std::vector<std::string> videos;
    std::string image_publish_topic;
    std::string image_info_publish_topic;
    std::string camera_info_publish_topic;
    std::string image_resized_publish_topic;
    SourceType source_type;
    std::string rtsp_uri;
    std::string onvif_host;
    int onvif_port;
    std::string onvif_user;
    std::string onvif_password;
    bool mask_enable_override;
    bool mask_enable_roi;
    std::string mask_filename;
};

class CameraWorker
{
public:
    explicit CameraWorker(ParameterNode & node, CameraWorkerParams & params, const std::function<void(const std_msgs::msg::Header &, const cv::Mat &)> & user_callback = nullptr)
        : node_(node)
        , params_(params)
        , user_callback_(user_callback)
    {
    }

    void init()
    {
        current_video_idx_ = 0;
        run_ = false;
        fps_ = 25.0;
        mask_enabled_ = false;

        camera_settings_client_ = node_.create_client<bob_interfaces::srv::CameraSettings>("bob/camera/settings");
        fps_update_client_ = node_.create_client<bob_interfaces::srv::ConfigEntryUpdate>("bob/config/update/fps");

        mask_timer_ = node_.create_wall_timer(std::chrono::seconds(30), [this](){mask_timer_callback();});
        mask_timer_callback(); // Calling it the first time
        open_camera();
        start_capture();
    }

private:
    ParameterNode & node_;
    CameraWorkerParams & params_;

    std::function<void(const std_msgs::msg::Header &, const cv::Mat &)> user_callback_;
    
    rclcpp::Client<bob_interfaces::srv::CameraSettings>::SharedPtr camera_settings_client_;
    rclcpp::Client<bob_interfaces::srv::ConfigEntryUpdate>::SharedPtr fps_update_client_;

    cv::VideoCapture video_capture_;
    bob_camera::msg::CameraInfo camera_info_msg_;
    rclcpp::TimerBase::SharedPtr mask_timer_;
    std::jthread capture_thread_;
    uint32_t current_video_idx_;
    bool run_;
    double fps_;
    bool mask_enabled_;
    std::optional<std::filesystem::file_time_type> mask_last_modified_time_;
    cv::Mat grey_mask_;
    cv::Rect bounding_box_;

    void captureLoop()
    {
        rclcpp::WallRate loop_rate(fps_);

        std::unique_ptr<RosCvImageMsg> roscv_image_msg_ptr = RosCvImageMsg::create(video_capture_);

        while (run_)
        {
            if (!video_capture_.read(*roscv_image_msg_ptr->image_ptr))
            {
                current_video_idx_ = current_video_idx_ >= (params_.videos.size() - 1) ? 0 : current_video_idx_ + 1;
                open_camera();
                roscv_image_msg_ptr = RosCvImageMsg::create(video_capture_);
                video_capture_.read(*roscv_image_msg_ptr->image_ptr);
            }

            apply_mask(*roscv_image_msg_ptr->image_ptr);

            roscv_image_msg_ptr->get_header().stamp = node_.now();
            roscv_image_msg_ptr->get_header().frame_id = ParameterNode::generate_uuid();

            params_.image_publisher->publish(*roscv_image_msg_ptr->msg_ptr);

            publish_resized_frame(*roscv_image_msg_ptr);

            publish_image_info(roscv_image_msg_ptr->get_header(), *roscv_image_msg_ptr->image_ptr);

            publish_camera_info(roscv_image_msg_ptr->get_header());

            if (user_callback_)
            {
                user_callback_(roscv_image_msg_ptr->get_header(), *roscv_image_msg_ptr->image_ptr);
            }

            loop_rate.sleep();  
        }
    }

    inline void apply_mask(const cv::Mat & img)
    {
        if (!mask_enabled_ || !params_.mask_enable_override)
        {
            return;
        }
        RCLCPP_INFO(node_.get_logger(), "Applying mask");
        if (img.size() != grey_mask_.size())
        {
            RCLCPP_WARN(node_.get_logger(), "Frame and mask dimensions do not match. Attempting resize.");
            RCLCPP_WARN(node_.get_logger(), "Note: Please ensure your mask has not gone stale, you might want to recreate it.");
            cv::resize(grey_mask_, grey_mask_, img.size());
            roi_calculation();
        }
        if (params_.mask_enable_roi)
        {
            cv::Mat image_roi = img(bounding_box_);
            cv::Mat mask_roi = grey_mask_(bounding_box_);
            cv::Mat result_roi;
            cv::bitwise_and(image_roi, image_roi, result_roi, mask_roi);
            result_roi.copyTo(img(bounding_box_));
        }
        else
        {
            cv::bitwise_and(img, img, img, grey_mask_);
        }        
    }

    inline void publish_resized_frame(const RosCvImageMsg & image_msg) const
    {
        if (!params_.image_resized_publisher || (node_.count_subscribers(params_.image_resized_publish_topic) <= 0))
        {
            return;
        }
        cv::Mat resized_img;
        if (params_.resize_height > 0)
        {
            const double aspect_ratio = (double)image_msg.image_ptr->size().width / (double)image_msg.image_ptr->size().height;
            const auto frame_width = (int)(aspect_ratio * (double)params_.resize_height);
            cv::resize(*image_msg.image_ptr, resized_img, cv::Size(frame_width, params_.resize_height));
        }
        else
        {
            resized_img = *image_msg.image_ptr;
        }

        auto resized_frame_msg = cv_bridge::CvImage(image_msg.msg_ptr->header, image_msg.msg_ptr->encoding, resized_img).toImageMsg();
        params_.image_resized_publisher->publish(*resized_frame_msg);            
    }

    void start_capture()
	{
		run_ = true;
		capture_thread_ = std::jthread(&CameraWorker::captureLoop, this);
	}

	void stop_capture()
	{
		run_ = false;
		if (capture_thread_.joinable())
        {
			capture_thread_.join();
        }
	}

    inline void open_camera()
    {
        switch (params_.source_type)
        {
            case CameraWorkerParams::SourceType::USB_CAMERA:
            {
                params_.camera_id = static_cast<int>(node_.get_parameter("camera_id").get_value<rclcpp::ParameterType::PARAMETER_INTEGER>());
                RCLCPP_INFO(node_.get_logger(), "Camera %d opening", params_.camera_id);
                video_capture_.open(params_.camera_id);
                set_highest_resolution();
            }
            break;

            case CameraWorkerParams::SourceType::VIDEO_FILE:
            {
                auto video_path = params_.videos[current_video_idx_];
                RCLCPP_INFO(node_.get_logger(), "Video '%s' opening", video_path.c_str());
                video_capture_.open(video_path);
            }
            break;

            case CameraWorkerParams::SourceType::RTSP_STREAM:
            {
                RCLCPP_INFO(node_.get_logger(), "RTSP Stream '%s' opening", params_.rtsp_uri.c_str());
                video_capture_.open(params_.rtsp_uri);
            }
            break;
        }

        if (video_capture_.isOpened())
        {
            fps_ = video_capture_.get(cv::CAP_PROP_FPS);
            RCLCPP_INFO(node_.get_logger(), "fps: %f", fps_);

            create_camera_info_msg();
        }
    }

    inline bool set_highest_resolution()
    {
        static const std::vector<std::pair<int, int>> resolutions = 
        {
            {3840, 2160},  // 4K UHD
            {2560, 1440}, // QHD, WQHD, 2K
            {1920, 1080}, // Full HD
            {1600, 900},  // HD+
            {1280, 720},  // HD
            {1024, 768},  // XGA
            {800, 600},   // SVGA
            {640, 480},   // VGA
            {320, 240}   // QVGA
        };

        for (const auto & [width, height] : resolutions)
        {
            video_capture_.set(cv::CAP_PROP_FRAME_WIDTH, width);
            video_capture_.set(cv::CAP_PROP_FRAME_HEIGHT, height);

            if ((video_capture_.get(cv::CAP_PROP_FRAME_WIDTH) == width) &&
                (video_capture_.get(cv::CAP_PROP_FRAME_HEIGHT) == height))
            {
                RCLCPP_INFO(node_.get_logger(), "Setting resolution for %d x %d", width, height);
                return true;
            }
        }

        return false;
    }

    inline void publish_image_info(const std_msgs::msg::Header & header, const cv::Mat & image) const
    {
        if (!params_.image_info_publisher || (node_.count_subscribers(params_.image_info_publisher->get_topic_name()) <= 0))
        {
            return;
        }

        bob_camera::msg::ImageInfo image_info_msg;
        image_info_msg.header = header;

        image_info_msg.roi.start_x = 0;
        image_info_msg.roi.start_y = 0;
        image_info_msg.roi.width = image.size().width;
        image_info_msg.roi.height = image.size().height;
        image_info_msg.bpp = image.elemSize1() == 1 ? 8 : 16;
        image_info_msg.exposure = 0;
        image_info_msg.gain = 0;
        image_info_msg.offset = 0;
        image_info_msg.white_balance.r = 0;
        image_info_msg.white_balance.g = 0;
        image_info_msg.white_balance.b = 0;
        image_info_msg.contrast = 0;
        image_info_msg.brightness = 0;
        image_info_msg.gamma = 1.0;
        image_info_msg.channels = image.channels();
        image_info_msg.bin_mode = 1;
        image_info_msg.current_temp = 0;
        image_info_msg.cool_enabled = false;
        image_info_msg.target_temp = 0;
        image_info_msg.auto_exposure = false;

        params_.image_info_publisher->publish(image_info_msg);
    }

    inline void publish_camera_info(const std_msgs::msg::Header & header)
    {
        if (!params_.camera_info_publisher || (node_.count_subscribers(params_.camera_info_publisher->get_topic_name()) <= 0))
        {
            return;
        }

        camera_info_msg_.header = header;
        params_.camera_info_publisher->publish(camera_info_msg_);
    }

    void request_camera_settings(const std::string_view & host, int port, const std::string_view& user, const std::string_view& password,
                                const std::function<void(const bob_interfaces::srv::CameraSettings::Response::SharedPtr&)> & user_callback) const
    {
        auto request = std::make_shared<bob_interfaces::srv::CameraSettings::Request>();
        request->host = host;
        request->port = port;
        request->user = user;
        request->password = password;

        auto response_received_callback = [user_callback](rclcpp::Client<bob_interfaces::srv::CameraSettings>::SharedFuture future) 
        {
            auto response = future.get();
            if (response->success)
                user_callback(response);  
        };

        // Send the request
        auto result = camera_settings_client_->async_send_request(request, response_received_callback);
    }

    inline void create_camera_info_msg()
    {
        if (params_.source_type == CameraWorkerParams::SourceType::RTSP_STREAM)
        {
            auto update_camera_info = [this](const bob_interfaces::srv::CameraSettings::Response::SharedPtr& response) 
            {
                camera_info_msg_.id = response->hardware_id;
                camera_info_msg_.manufacturer = response->manufacturer;
                camera_info_msg_.model = response->model;
                camera_info_msg_.serial_num = response->serial_number;
                camera_info_msg_.firmware_version = response->firmware_version;
                camera_info_msg_.num_configurations = response->num_configurations;
                camera_info_msg_.encoding = response->encoding;
                camera_info_msg_.frame_width = response->frame_width;
                camera_info_msg_.frame_height = response->frame_height;
                camera_info_msg_.fps = response->fps;
                // ... other fields ... 
            };
            request_camera_settings(params_.onvif_host, params_.onvif_port, params_.onvif_user, params_.onvif_password, update_camera_info);
        } 
        else if (params_.source_type == CameraWorkerParams::SourceType::USB_CAMERA) 
        {
            camera_info_msg_.model = "USB Camera";
        } 
        else if (params_.source_type == CameraWorkerParams::SourceType::VIDEO_FILE) 
        {
            camera_info_msg_.model = "Video File";
        }
    }

    inline void roi_calculation()
    {
        if (params_.mask_enable_roi && mask_enabled_)
        {
            sensor_msgs::msg::RegionOfInterest roi_msg;
            if(grey_mask_.empty())
            {
                roi_msg.x_offset = 0;
                roi_msg.y_offset = 0;
                roi_msg.width = grey_mask_.size().width;
                roi_msg.height = grey_mask_.size().height;
            }
            else
            {
                bounding_box_ = cv::Rect(grey_mask_.cols, grey_mask_.rows, 0, 0);                                
                for (int y = 0; y < grey_mask_.rows; ++y) 
                {
                    for (int x = 0; x < grey_mask_.cols; ++x) 
                    {
                        if (grey_mask_.at<uchar>(y, x) == 255) 
                        { 
                            bounding_box_.x = std::min(bounding_box_.x, x);
                            bounding_box_.y = std::min(bounding_box_.y, y);
                            bounding_box_.width = std::max(bounding_box_.width, x - bounding_box_.x);
                            bounding_box_.height = std::max(bounding_box_.height, y - bounding_box_.y);
                        }
                    }
                }

                roi_msg.x_offset = bounding_box_.x;
                roi_msg.y_offset = bounding_box_.y;
                roi_msg.width = bounding_box_.width;
                roi_msg.height = bounding_box_.height;
            }

            node_.publish_if_subscriber(params_.roi_publisher, roi_msg);

            RCLCPP_INFO(node_.get_logger(), "Detection frame size determined from mask: %d x %d", roi_msg.width, roi_msg.height);
        }
    }

    void mask_timer_callback()
    {
        mask_timer_->cancel();
        try
        {
            if (!std::filesystem::exists(params_.mask_filename))
            {
                if (mask_enabled_)
                {
                    RCLCPP_INFO(node_.get_logger(), "Mask Disabled.");
                }
                mask_enabled_ = false;
                mask_timer_->reset();
                return;
            }

            auto current_modified_time = std::filesystem::last_write_time(params_.mask_filename);
            if (mask_last_modified_time_ == current_modified_time) 
            {
                mask_timer_->reset();
                return;
            }
            mask_last_modified_time_ = current_modified_time;

            grey_mask_= cv::imread(params_.mask_filename, cv::IMREAD_UNCHANGED);
            mask_enabled_ = !grey_mask_.empty();
            if (grey_mask_.empty())
            {
                RCLCPP_INFO(node_.get_logger(), "Mask Disabled, mask image was empty");
            }
            else
            {
                RCLCPP_INFO(node_.get_logger(), "Mask Enabled.");
            }
            roi_calculation();
        }
        catch (cv::Exception &cve)
        {
            RCLCPP_ERROR(node_.get_logger(), "Open CV exception on timer callback: %s", cve.what());
        }
        
        mask_timer_->reset();
    }

    void request_update_fps(const float fps, 
            const std::function<void(const bob_interfaces::srv::ConfigEntryUpdate::Response::SharedPtr&)> & user_callback) const
    {
        auto request = std::make_shared<bob_interfaces::srv::ConfigEntryUpdate::Request>();
        request->key = "fps";
        request->type = "double";
        request->value = std::to_string(fps);

        auto response_received_callback = [user_callback](rclcpp::Client<bob_interfaces::srv::ConfigEntryUpdate>::SharedFuture future) 
        {
            auto response = future.get();
            if (response->success)
            {
                user_callback(response);
            }
        };

        // Send the request
        auto result = fps_update_client_->async_send_request(request, response_received_callback);
    }
};

#endif