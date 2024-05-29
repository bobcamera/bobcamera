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

#include <image_utils.hpp>
#include <parameter_node.hpp>
#include <object_simulator.hpp>
#include <mask_worker.hpp>

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
    std::string mask_filename;
    int simulator_num_objects;
    bool simulator_enable;
};

class CameraWorker
{
public:
    explicit CameraWorker(ParameterNode & node
                        , CameraWorkerParams & params
                        , const std::function<void(const std_msgs::msg::Header &, const cv::Mat &)> & user_callback = nullptr)
        : node_(node)
        , params_(params)
        , user_callback_(user_callback)
    {
    }

    ~CameraWorker()
    {
        stop_capture();
    }

    void init()
    {
        try
        {
            current_video_idx_ = 0;
            run_ = false;
            fps_ = 25.0;
            mask_enabled_ = false;

            if (params_.simulator_enable)
            {
                object_simulator_ptr_ = std::make_unique<ObjectSimulator>(params_.simulator_num_objects);
            }

            mask_worker_ptr_ = std::make_unique<MaskWorker>(node_, [this](MaskWorker::MaskCheckType detection_mask_result, const cv::Mat & mask){mask_timer_callback(detection_mask_result, mask);});
            mask_worker_ptr_->init(5, params_.mask_filename);

            camera_settings_client_ = node_.create_client<bob_interfaces::srv::CameraSettings>("bob/camera/settings");
            fps_update_client_ = node_.create_client<bob_interfaces::srv::ConfigEntryUpdate>("bob/config/update/fps");

            open_camera();
            start_capture();
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(node_.get_logger(), "Exception: %s", e.what());
            throw;
        }
        catch (...) 
        {
            RCLCPP_ERROR(node_.get_logger(), "Unknown exception caught");
            throw;
        }
    }

private:
    ParameterNode & node_;
    CameraWorkerParams & params_;

    std::unique_ptr<MaskWorker> mask_worker_ptr_;

    std::function<void(const std_msgs::msg::Header &, const cv::Mat &)> user_callback_;
    
    rclcpp::Client<bob_interfaces::srv::CameraSettings>::SharedPtr camera_settings_client_;
    rclcpp::Client<bob_interfaces::srv::ConfigEntryUpdate>::SharedPtr fps_update_client_;

    cv::VideoCapture video_capture_;
    bob_camera::msg::CameraInfo camera_info_msg_;
    std::jthread capture_thread_;
    uint32_t current_video_idx_;
    bool run_;
    double fps_;
    std::unique_ptr<ObjectSimulator> object_simulator_ptr_;

    bool mask_enabled_;
    cv::Mat privacy_mask_;

    void captureLoop()
    {
        rclcpp::WallRate loop_rate(fps_);

        std::unique_ptr<RosCvImageMsg> roscv_image_msg_ptr = RosCvImageMsg::create(video_capture_);

        int retries = 0;
        while (run_)
        {
            try
            {
                if (!video_capture_.read(roscv_image_msg_ptr->get_image()))
                {
                    if (params_.source_type == CameraWorkerParams::SourceType::VIDEO_FILE)
                    {
                        current_video_idx_ = current_video_idx_ >= (params_.videos.size() - 1) ? 0 : current_video_idx_ + 1;
                        open_camera();
                        roscv_image_msg_ptr = RosCvImageMsg::create(video_capture_);
                        continue;
                    }
                    else
                    {
                        if (++retries > 5)
                        {
                            RCLCPP_ERROR(node_.get_logger(), "Could not open VIDEO SOURCE");
                            run_ = false;
                        }
                        continue;
                    }
                }
                if (roscv_image_msg_ptr->recreate_if_invalid())
                {
                    RCLCPP_INFO(node_.get_logger(), "ImageMsg recreated");
                }

                retries = 0;

                if (params_.simulator_enable)
                {
                    object_simulator_ptr_->move(roscv_image_msg_ptr->get_image());
                }

                apply_mask(roscv_image_msg_ptr->get_image());

                roscv_image_msg_ptr->get_header().stamp = node_.now();
                roscv_image_msg_ptr->get_header().frame_id = ParameterNode::generate_uuid();

                node_.publish_if_subscriber(params_.image_publisher, roscv_image_msg_ptr->get_msg());

                publish_resized_frame(*roscv_image_msg_ptr);

                publish_image_info(roscv_image_msg_ptr->get_header(), roscv_image_msg_ptr->get_image());

                publish_camera_info(roscv_image_msg_ptr->get_header());

                if (user_callback_)
                {
                    user_callback_(roscv_image_msg_ptr->get_header(), roscv_image_msg_ptr->get_image());
                }

                loop_rate.sleep();  
            }
            catch (const std::exception& e)
            {
                RCLCPP_ERROR(node_.get_logger(), "Exception: %s", e.what());
                rcutils_reset_error();
                run_ = false;
            }
            catch (...) 
            {
                RCLCPP_ERROR(node_.get_logger(), "Unknown exception caught");
                run_ = false;
            }
        }
        RCLCPP_DEBUG(node_.get_logger(), "Leaving capture loop");
    }

    inline void apply_mask(cv::Mat & img)
    {
        if (!mask_enabled_ || !params_.mask_enable_override)
        {
            return;
        }
        if (img.size() != privacy_mask_.size())
        {
            cv::resize(privacy_mask_, privacy_mask_, img.size());
        }
        if (img.channels() != privacy_mask_.channels())
        {
            cv::cvtColor(privacy_mask_, privacy_mask_, img.channels() == 3 ? cv::COLOR_GRAY2BGR : cv::COLOR_BGR2GRAY);
        }
        cv::bitwise_and(img, privacy_mask_, img);
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
            const auto frame_width = (int)(image_msg.get_image().size().aspectRatio() * (double)params_.resize_height);
            cv::resize(image_msg.get_image(), resized_img, cv::Size(frame_width, params_.resize_height));
        }
        else
        {
            resized_img = image_msg.get_image();
        }

        auto resized_frame_msg = cv_bridge::CvImage(image_msg.get_header(), image_msg.get_msg().encoding, resized_img).toImageMsg();
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
                const auto & video_path = params_.videos[current_video_idx_];
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
            {
                user_callback(response);  
            }
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

    void mask_timer_callback(MaskWorker::MaskCheckType detection_mask_result, const cv::Mat & mask)
    {
        if (detection_mask_result == MaskWorker::MaskCheckType::Enable)
        {
            if (!mask_enabled_)
            {
                mask_enabled_ = true;
                RCLCPP_INFO(node_.get_logger(), "Privacy Mask Enabled.");
            }
            else
            {
                RCLCPP_INFO(node_.get_logger(), "Privacy Mask Changed.");
            }
            privacy_mask_ = mask.clone();
        }
        else if ((detection_mask_result == MaskWorker::MaskCheckType::Disable) && mask_enabled_)
        {
            RCLCPP_INFO(node_.get_logger(), "Privacy Mask Disabled.");
            mask_enabled_ = false;
            privacy_mask_.release();
        }
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