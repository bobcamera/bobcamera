#include <chrono>
#include <string>

#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "bob_camera/msg/image_info.hpp"
#include "bob_camera/msg/camera_info.hpp"
#include "parameter_node.hpp"
#include "bob_interfaces/srv/camera_settings.hpp"
#include "bob_interfaces/srv/config_entry_update.hpp"
#include <boblib/api/utils/profiler.hpp>
#include <visibility_control.h>

enum class SourceType {
    USB_CAMERA,
    VIDEO_FILE,
    RTSP_STREAM
};


class WebCameraVideo
    : public ParameterNode
{
public:
    COMPOSITION_PUBLIC
    explicit WebCameraVideo(const rclcpp::NodeOptions & options)
        : ParameterNode("web_camera_video_node", options)
        , current_video_idx_(0)
        , run_(false)
        , fps_(25.0)
    {
        qos_profile_.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        qos_profile_.durability(rclcpp::DurabilityPolicy::Volatile);
        qos_profile_.history(rclcpp::HistoryPolicy::KeepLast);

        one_shot_timer_ = this->create_wall_timer(
            std::chrono::seconds(2), 
            [this]() {
                this->init(); 
                this->one_shot_timer_.reset();  
            }
        );
    }

    ~WebCameraVideo()
    {
        stop_capture();
    }

private:
    rclcpp::Client<bob_interfaces::srv::CameraSettings>::SharedPtr camera_settings_client_;
    rclcpp::Client<bob_interfaces::srv::ConfigEntryUpdate>::SharedPtr fps_update_client_;
    rclcpp::QoS qos_profile_{10}; 
    cv::VideoCapture video_capture_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::Publisher<bob_camera::msg::ImageInfo>::SharedPtr image_info_publisher_;
    rclcpp::Publisher<bob_camera::msg::CameraInfo>::SharedPtr camera_info_publisher_;
    bob_camera::msg::CameraInfo camera_info_msg_;
    int camera_id_;
    int resize_height_;
    std::vector<std::string> videos_;
    uint32_t current_video_idx_;
    std::string image_publish_topic_;
    std::string image_info_publish_topic_;
    std::string camera_info_publish_topic_;
    rclcpp::TimerBase::SharedPtr one_shot_timer_;
    SourceType source_type_;
    std::string rtsp_uri_;

    std::string onvif_host_;
    int onvif_port_;
    std::string onvif_user_;
    std::string onvif_password_;
    
    std::thread capture_thread_;
    bool run_;
    double fps_;

    void init()
    {
        camera_settings_client_ = this->create_client<bob_interfaces::srv::CameraSettings>("bob/camera/settings");
        fps_update_client_ = this->create_client<bob_interfaces::srv::ConfigEntryUpdate>("bob/config/update/fps");
        declare_node_parameters();    
        open_camera();
        create_camera_info_msg();
        start_capture();
    }

    void captureLoop()
    {
        rclcpp::WallRate loop_rate(fps_);
        cv::Mat image;

        while (run_)
        {
            if (!video_capture_.read(image))
            {
                current_video_idx_ = current_video_idx_ >= (videos_.size() - 1) ? 0 : current_video_idx_ + 1;
                open_camera();
                video_capture_.read(image);
            }

            if (resize_height_ > 0)
            {
                const double aspect_ratio = (double)image.size().width / (double)image.size().height;
                const int frame_height = resize_height_;
                const int frame_width = static_cast<int>(aspect_ratio * (double)frame_height);
                cv::resize(image, image, cv::Size(frame_width, frame_height));
            }

            std_msgs::msg::Header header;
            header.stamp = now();
            header.frame_id = generate_uuid();

            auto image_msg = cv_bridge::CvImage(header, image.channels() == 1 ? sensor_msgs::image_encodings::MONO8 : sensor_msgs::image_encodings::BGR8, image).toImageMsg();
            image_publisher_->publish(*image_msg);

            auto image_info_msg = generate_image_info(header, image);
            image_info_publisher_->publish(image_info_msg);

            camera_info_msg_.header = header;
            camera_info_publisher_->publish(camera_info_msg_);

            loop_rate.sleep();  
        }
    }

    void start_capture()
	{
		run_ = true;
		capture_thread_ = std::thread(&WebCameraVideo::captureLoop, this);
	}

	void stop_capture()
	{
		run_ = false;
		if (capture_thread_.joinable())
        {
			capture_thread_.join();
        }
	}
    
    void declare_node_parameters()
    {
        std::vector<ParameterNode::ActionParam> params = {
            ParameterNode::ActionParam(
                rclcpp::Parameter("image_publish_topic", "bob/frames/allsky/original"), 
                [this](const rclcpp::Parameter& param) {
                    image_publish_topic_ = param.as_string(); 
                    image_publisher_ = create_publisher<sensor_msgs::msg::Image>(image_publish_topic_, qos_profile_);
                }
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("image_info_publish_topic", "bob/camera/all_sky/image_info"), 
                [this](const rclcpp::Parameter& param) {
                    image_info_publish_topic_ = param.as_string(); 
                    image_info_publisher_ = create_publisher<bob_camera::msg::ImageInfo>(image_info_publish_topic_, qos_profile_);
                }
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("camera_info_publish_topic", "bob/camera/all_sky/camera_info"), 
                [this](const rclcpp::Parameter& param) {
                    camera_info_publish_topic_ = param.as_string(); 
                    camera_info_publisher_ = create_publisher<bob_camera::msg::CameraInfo>(camera_info_publish_topic_, qos_profile_);
                }
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("resize_height", 0), 
                [this](const rclcpp::Parameter& param) {resize_height_ = param.as_int();}
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("source_type", "USB_CAMERA"), 
                [this](const rclcpp::Parameter& param) {
                    std::string type = param.as_string();
                    if (type == "USB_CAMERA") {
                        source_type_ = SourceType::USB_CAMERA;
                    } else if (type == "VIDEO_FILE") {
                        source_type_ = SourceType::VIDEO_FILE;
                    } else if (type == "RTSP_STREAM") {
                        source_type_ = SourceType::RTSP_STREAM;
                    } else {
                        RCLCPP_ERROR(get_logger(), "Invalid source type: %s", type.c_str());
                    }
                }
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("camera_id", 0), 
                [this](const rclcpp::Parameter& param) {camera_id_ = param.as_int();}
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("videos", std::vector<std::string>({""})), 
                [this](const rclcpp::Parameter& param) {videos_ = param.as_string_array();}
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("rtsp_uri", ""), 
                [this](const rclcpp::Parameter& param) { rtsp_uri_ = param.as_string();}
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("onvif_host", "192.168.1.20"),
                [this](const rclcpp::Parameter& param) { onvif_host_ = param.as_string(); }
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("onvif_port", 80),
                [this](const rclcpp::Parameter& param) { onvif_port_ = param.as_int(); }
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("onvif_user", "default_user"),
                [this](const rclcpp::Parameter& param) { onvif_user_ = param.as_string(); }
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("onvif_password", "default_password"),
                [this](const rclcpp::Parameter& param) { onvif_password_ = param.as_string(); }
            ),
        };
        add_action_parameters(params);
    }

    void request_camera_settings(const std::string& host, int port, const std::string& user, const std::string& password,
                                std::function<void(const bob_interfaces::srv::CameraSettings::Response::SharedPtr&)> user_callback)
    {
        auto request = std::make_shared<bob_interfaces::srv::CameraSettings::Request>();
        request->host = host;
        request->port = port;
        request->user = user;
        request->password = password;

        auto response_received_callback = [this, user_callback](rclcpp::Client<bob_interfaces::srv::CameraSettings>::SharedFuture future) {
            auto response = future.get();
            if(response->success)
                user_callback(response);  
        };

        // Send the request
        auto result = camera_settings_client_->async_send_request(request, response_received_callback);
    }

    void request_update_fps(const float fps, std::function<void(const bob_interfaces::srv::ConfigEntryUpdate::Response::SharedPtr&)> user_callback)
    {
        auto request = std::make_shared<bob_interfaces::srv::ConfigEntryUpdate::Request>();
        request->key = "fps";
        request->type = "double";
        request->value = std::to_string(fps);

        auto response_received_callback = [this, user_callback](rclcpp::Client<bob_interfaces::srv::ConfigEntryUpdate>::SharedFuture future) {
            auto response = future.get();
            if(response->success)
                user_callback(response);  
        };

        // Send the request
        auto result = fps_update_client_->async_send_request(request, response_received_callback);
    }

    inline void open_camera()
    {
        switch (source_type_)
        {
            case SourceType::USB_CAMERA:
            {
                camera_id_ = get_parameter("camera_id").get_value<rclcpp::ParameterType::PARAMETER_INTEGER>();
                RCLCPP_INFO(get_logger(), "Camera %d opening", camera_id_);
                video_capture_.open(camera_id_);
                setHighestResolution(video_capture_);
            }
            break;

            case SourceType::VIDEO_FILE:
            {
                auto video_path = videos_[current_video_idx_];
                RCLCPP_INFO(get_logger(), "Video '%s' opening", video_path.c_str());
                video_capture_.open(video_path);
            }
            break;

            case SourceType::RTSP_STREAM:
            {
                RCLCPP_INFO(get_logger(), "RTSP Stream '%s' opening", rtsp_uri_.c_str());
                video_capture_.open(rtsp_uri_);
            }
            break;
        }

        if (video_capture_.isOpened())
        {
            fps_ = video_capture_.get(cv::CAP_PROP_FPS);
            RCLCPP_INFO(get_logger(), "fps: %f", fps_);

            /*auto update_fps = [this](const bob_interfaces::srv::ConfigEntryUpdate::Response::SharedPtr& response) 
            {
                if (response->success)
                {
                    RCLCPP_INFO(get_logger(), "FPS Updated Successfully");
                }
            };

            request_update_fps(fps_, update_fps);*/
        }
    }


    inline bool setHighestResolution(cv::VideoCapture &cap)
    {
        std::vector<std::pair<int, int>> resolutions = {
            //{3840, 2160},  // 4K UHD
            //{2560, 1440}, // QHD, WQHD, 2K
            {1920, 1080}, // Full HD
            {1600, 900},  // HD+
            {1280, 720},  // HD
            {1024, 768},  // XGA
            {800, 600},   // SVGA
            {640, 480},   // VGA
            {320, 240}   // QVGA
        };

        std::vector<std::pair<int, int>> supportedResolutions;
        for (const auto &resolution : resolutions)
        {
            cap.set(cv::CAP_PROP_FRAME_WIDTH, resolution.first);
            cap.set(cv::CAP_PROP_FRAME_HEIGHT, resolution.second);

            if (cap.get(cv::CAP_PROP_FRAME_WIDTH) == resolution.first &&
                cap.get(cv::CAP_PROP_FRAME_HEIGHT) == resolution.second)
            {
                RCLCPP_INFO(get_logger(), "Setting resolution for %d x %d", resolution.first, resolution.second);
                return true;
            }
        }

        return false;
    }

    inline bob_camera::msg::ImageInfo generate_image_info(std_msgs::msg::Header &header, const cv::Mat& image)
    {
        bob_camera::msg::ImageInfo image_info_msg;
        image_info_msg.header = header;

        image_info_msg.roi.start_x = 0;
        image_info_msg.roi.start_y = 0;
        image_info_msg.roi.width = image.size().width;
        image_info_msg.roi.height = image.size().height;
        image_info_msg.bpp = image.elemSize1() == 1 ? 8 : 16;
        // image_info_msg.bayer_format = image.channels() == 1 ? boblib::camera::QhyCamera::BayerFormat::Mono : boblib::camera::QhyCamera::BayerFormat::Color;
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

        return image_info_msg;
    }

    inline void create_camera_info_msg()
    {
        if (source_type_ == SourceType::RTSP_STREAM) {
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
            request_camera_settings(onvif_host_, onvif_port_, onvif_user_, onvif_password_, update_camera_info);
        } else if (source_type_ == SourceType::USB_CAMERA) {
            camera_info_msg_.model = "USB Camera";
        } else if (source_type_ == SourceType::VIDEO_FILE) {
            camera_info_msg_.model = "Video File";
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::StaticSingleThreadedExecutor executor;
    executor.add_node(std::make_shared<WebCameraVideo>(rclcpp::NodeOptions()));
    executor.spin();
    rclcpp::shutdown();
    return 0;
}

RCLCPP_COMPONENTS_REGISTER_NODE(WebCameraVideo)