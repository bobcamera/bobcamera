#pragma once
#ifndef __CAMERA_WORKER_H_
#define __CAMERA_WORKER_H_

#include <string>
#include <filesystem>

#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>

#include <bob_interfaces/srv/camera_settings.hpp>
#include <bob_interfaces/srv/config_entry_update.hpp>

#include <bob_camera/msg/image_info.hpp>
#include <bob_camera/msg/camera_info.hpp>

#include <image_utils.hpp>
#include <parameter_lifecycle_node.hpp>
#include "object_simulator.hpp"
#include <mask_worker.hpp>
#include <circuit_breaker.hpp>

struct CameraWorkerParams
{
    enum class SourceType 
    {
        USB_CAMERA,
        VIDEO_FILE,
        RTSP_STREAM,
        UNKNOWN
    };

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_resized_publisher;
    rclcpp::Publisher<bob_camera::msg::ImageInfo>::SharedPtr image_info_publisher;
    rclcpp::Publisher<bob_camera::msg::CameraInfo>::SharedPtr camera_info_publisher;
    rclcpp::Client<bob_interfaces::srv::CameraSettings>::SharedPtr camera_settings_client;
    rclcpp::Client<bob_interfaces::srv::ConfigEntryUpdate>::SharedPtr fps_update_client;

    int camera_id;
    std::vector<long> usb_resolution;
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
    int mask_timer_seconds;
};

class CameraWorker
{
public:
    explicit CameraWorker(ParameterLifeCycleNode & node
                        , CameraWorkerParams & params
                        , const std::function<void(const std_msgs::msg::Header &, const cv::Mat &)> & user_callback = nullptr)
        : node_(node)
        , params_(params)
        , user_callback_(user_callback)
    {
        mask_worker_ptr_ = std::make_unique<MaskWorker>(node_, 
            [this](MaskWorker::MaskCheckType detection_mask_result, const cv::Mat & mask)
            {
                mask_timer_callback(detection_mask_result, mask);
            });
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
            is_open_ = false;
            is_camera_info_auto_ = false;

            camera_info_msg_.frame_height = 0;
            camera_info_msg_.frame_width = 0;
            camera_info_msg_.fps = 0;

            circuit_breaker_ptr_ = std::make_unique<CircuitBreaker>(1000, 200, 10000);

            if (params_.simulator_enable)
            {
                object_simulator_ptr_ = std::make_unique<ObjectSimulator>(params_.simulator_num_objects);
            }

            mask_worker_ptr_->init(params_.mask_timer_seconds, params_.mask_filename);

            open_camera();
            start_capture();
        }
        catch (const std::exception& e)
        {
            node_.log_error("camera_worker: init: Exception: %s", e.what());
            throw;
        }
        catch (...) 
        {
            node_.log_error("camera_worker: init: Unknown exception");
            throw;
        }
    }

    void restart_mask()
    {
        if (mask_worker_ptr_)
        {
            mask_worker_ptr_->init(params_.mask_timer_seconds, params_.mask_filename);
        }
    }

    bool is_open() const
    {
        return is_open_;
    }

    void open_camera()
    {
        switch (params_.source_type)
        {
            case CameraWorkerParams::SourceType::USB_CAMERA:
            {
                node_.log_send_info("Camera %d opening", params_.camera_id);
                video_capture_.open(params_.camera_id);
                set_camera_resolution();
            }
            break;

            case CameraWorkerParams::SourceType::VIDEO_FILE:
            {
                const auto & video_path = params_.videos[current_video_idx_];
                node_.log_send_info("Video '%s' opening", video_path.c_str());
                video_capture_.open(video_path);
            }
            break;

            case CameraWorkerParams::SourceType::RTSP_STREAM:
            {
                node_.log_send_info("RTSP Stream '%s' opening", params_.rtsp_uri.c_str());
                video_capture_.open(params_.rtsp_uri);
            }
            break;

            default:
                node_.log_send_error("Unknown SOURCE_TYPE");
                return;
        }

        is_open_ = video_capture_.isOpened();
        update_capture_info();
    }    

private:
    void update_capture_info()
    {
        if (is_open_)
        {
            fps_ = video_capture_.get(cv::CAP_PROP_FPS);
            cv_camera_width_ = static_cast<int>(video_capture_.get(cv::CAP_PROP_FRAME_WIDTH));
            cv_camera_height_ = static_cast<int>(video_capture_.get(cv::CAP_PROP_FRAME_HEIGHT));
            node_.log_send_info("Camera capture Info: %dx%d at %.2g FPS", cv_camera_width_, cv_camera_height_, fps_);
            loop_rate_ptr_ = std::make_unique<rclcpp::WallRate>(fps_);

            create_camera_info_msg();
        }
        else
        {
            loop_rate_ptr_ = std::make_unique<rclcpp::WallRate>(UNKNOWN_DEVICE_FPS);
        }        
    }

    void capture_loop()
    {
        std::unique_ptr<RosCvImageMsg> roscv_image_msg_ptr = RosCvImageMsg::create(video_capture_);

        while (run_)
        {
            try
            {
                if (!circuit_breaker_ptr_->allow_request()) 
                {
                    node_.log_send_error("Could not acquire image, Waiting to connect to camera");
                    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                    continue;
                }

                if (!video_capture_.read(roscv_image_msg_ptr->get_image()))
                {
                    if (params_.source_type == CameraWorkerParams::SourceType::VIDEO_FILE)
                    {
                        current_video_idx_ = current_video_idx_ >= (params_.videos.size() - 1) ? 0 : current_video_idx_ + 1;
                        open_camera();
                        roscv_image_msg_ptr = RosCvImageMsg::create(video_capture_);
                    }
                    else
                    {
                        circuit_breaker_ptr_->record_failure();
                        continue;
                    }
                }
                circuit_breaker_ptr_->record_success();
                if (roscv_image_msg_ptr->recreate_if_invalid())
                {
                    node_.log_debug("ImageMsg recreated");
                    update_capture_info();
                }

                // If we did not get the data from ONVIF
                if (!is_camera_info_auto_)
                {
                    camera_info_msg_.fps = static_cast<float>(fps_);
                    camera_info_msg_.frame_width = roscv_image_msg_ptr->get_image().size().width;
                    camera_info_msg_.frame_height = roscv_image_msg_ptr->get_image().size().height;
                    camera_info_msg_.is_color = roscv_image_msg_ptr->get_image().channels() >= 3;
                }

                if (params_.simulator_enable)
                {
                    object_simulator_ptr_->move(roscv_image_msg_ptr->get_image());
                }

                apply_mask(roscv_image_msg_ptr->get_image());

                roscv_image_msg_ptr->get_header().stamp = node_.now();
                roscv_image_msg_ptr->get_header().frame_id = ParameterLifeCycleNode::generate_uuid();

                node_.publish_if_subscriber(params_.image_publisher, roscv_image_msg_ptr->get_msg());

                publish_resized_frame(*roscv_image_msg_ptr);

                publish_image_info(roscv_image_msg_ptr->get_header(), roscv_image_msg_ptr->get_image());

                publish_camera_info(roscv_image_msg_ptr->get_header());

                if (user_callback_)
                {
                    user_callback_(roscv_image_msg_ptr->get_header(), roscv_image_msg_ptr->get_image());
                }

                loop_rate_ptr_->sleep();
            }
            catch (const std::exception & e)
            {
                node_.log_send_error("CameraWorker: capture_loop: Exception: %s", e.what());
                rcutils_reset_error();
                //run_ = false;
            }
            catch (...) 
            {
                node_.log_send_error("CameraWorker: capture_loop: Unknown exception");
                rcutils_reset_error();
                //run_ = false;
            }
        }
        node_.log_send_info("CameraWorker: Leaving capture_loop");
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
        if (!params_.image_resized_publisher 
            || (params_.resize_height <= 0)
            || (node_.count_subscribers(params_.image_resized_publish_topic) <= 0))
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
		capture_thread_ = std::jthread(&CameraWorker::capture_loop, this);
	}

	void stop_capture()
	{
		run_ = false;
		if (capture_thread_.joinable())
        {
			capture_thread_.join();
        }
	}

    inline bool set_camera_resolution()
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

        auto set_check_resolution = [this](long width, long height)
        {
            video_capture_.set(cv::CAP_PROP_FRAME_WIDTH, width);
            video_capture_.set(cv::CAP_PROP_FRAME_HEIGHT, height);

            return ((video_capture_.get(cv::CAP_PROP_FRAME_WIDTH) == width) &&
                    (video_capture_.get(cv::CAP_PROP_FRAME_HEIGHT) == height));
        };

        // If we have the values defined, try setting
        if (params_.usb_resolution.size() == 2)
        {
            auto success = set_check_resolution(params_.usb_resolution[0], params_.usb_resolution[1]);
            if (success)
            {
                return true;
            }
        }
        // If not, we set the highest resolution
        for (const auto & [width, height] : resolutions)
        {
            auto success = set_check_resolution(params_.usb_resolution[0], params_.usb_resolution[1]);
            if (success)
            {
                node_.log_info("Setting resolution for %d x %d", width, height);
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

    void request_camera_settings(const std::string_view & host, int port, const std::string_view & user, const std::string_view & password,
                                const std::function<void(const bob_interfaces::srv::CameraSettings::Response::SharedPtr &)> & user_callback) const
    {
        if (host.empty() || port == 0)
        {
            return;
        }
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

        auto result = params_.camera_settings_client->async_send_request(request, response_received_callback);
    }

    inline void create_camera_info_msg()
    {
        is_camera_info_auto_ = false;
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
                is_camera_info_auto_ = true;
                loop_rate_ptr_ = std::make_unique<rclcpp::WallRate>(response->fps);
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
                node_.log_send_info("CameraWorker: Privacy Mask Enabled.");
            }
            else
            {
                node_.log_send_info("CameraWorker: Privacy Mask Changed.");
            }
            privacy_mask_ = mask.clone();
        }
        else if ((detection_mask_result == MaskWorker::MaskCheckType::Disable) && mask_enabled_)
        {
            node_.log_send_info("CameraWorker: Privacy Mask Disabled.");
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
        auto result = params_.fps_update_client->async_send_request(request, response_received_callback);
    }

    static constexpr double UNKNOWN_DEVICE_FPS = 30.0;
    ParameterLifeCycleNode & node_;
    CameraWorkerParams & params_;

    std::unique_ptr<MaskWorker> mask_worker_ptr_;

    std::function<void(const std_msgs::msg::Header &, const cv::Mat &)> user_callback_;
    
    cv::VideoCapture video_capture_;
    bob_camera::msg::CameraInfo camera_info_msg_;
    std::jthread capture_thread_;
    uint32_t current_video_idx_;
    bool run_;
    std::unique_ptr<rclcpp::WallRate> loop_rate_ptr_;
    float fps_;
    int cv_camera_width_;
    int cv_camera_height_;
    bool is_open_;
    bool is_camera_info_auto_;
    std::unique_ptr<ObjectSimulator> object_simulator_ptr_;

    bool mask_enabled_;
    cv::Mat privacy_mask_;

    std::unique_ptr<CircuitBreaker> circuit_breaker_ptr_;
};

#endif