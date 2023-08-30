#include <chrono>

#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <sensor_msgs/msg/image.hpp>

#include "bob_camera/msg/image_info.hpp"
#include "bob_camera/msg/camera_info.hpp"

#include "parameter_node.hpp"

#include <boblib/api/camera/qhy_camera.hpp>
#include <boblib/api/utils/profiler.hpp>

#include <visibility_control.h>

class WebCameraVideo
    : public ParameterNode
{
public:
    COMPOSITION_PUBLIC
    explicit WebCameraVideo(const rclcpp::NodeOptions & options)
        : ParameterNode("web_camera_video_node", options)
    {
        qos_profile_.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        qos_profile_.durability(rclcpp::DurabilityPolicy::Volatile);
        qos_profile_.history(rclcpp::HistoryPolicy::KeepLast);

        declare_node_parameters();
        current_video_idx_ = 0;

        open_camera();
        create_camera_info_msg();

        timer_ = create_wall_timer(std::chrono::milliseconds(10), std::bind(&WebCameraVideo::timer_callback, this));
    }

    void timer_callback()
    {
        cv::Mat image;
        if (!video_capture_.read(image) && is_video_)
        {
            current_video_idx_ = current_video_idx_ >= (videos_.size() - 1) ? 0 : current_video_idx_ + 1;
            open_camera();
            video_capture_.read(image);
        }

        if (resize_height_ > 0)
        {
            const double aspect_ratio = (double)image.size().width / (double)image.size().height;
            const int frame_height = resize_height_;
            const int frame_width = (int)(aspect_ratio * (double)frame_height);
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
    }

private:
    rclcpp::QoS qos_profile_{10}; // The depth of the publisher queue
    cv::VideoCapture video_capture_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::Publisher<bob_camera::msg::ImageInfo>::SharedPtr image_info_publisher_;
    rclcpp::Publisher<bob_camera::msg::CameraInfo>::SharedPtr camera_info_publisher_;
    bob_camera::msg::CameraInfo camera_info_msg_;
    bool is_video_;
    int camera_id_;
    int resize_height_;
    std::vector<std::string> videos_;
    uint32_t current_video_idx_;
    std::string image_publish_topic_;
    std::string image_info_publish_topic_;
    std::string camera_info_publish_topic_;
    rclcpp::TimerBase::SharedPtr timer_;

    void declare_node_parameters()
    {
        std::vector<ParameterNode::ActionParam> params = {
            ParameterNode::ActionParam(
                rclcpp::Parameter("image_publish_topic", "bob/camera/all_sky/bayer"), 
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
                rclcpp::Parameter("is_video", false), 
                [this](const rclcpp::Parameter& param) {is_video_ = param.as_bool();}
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("camera_id", 0), 
                [this](const rclcpp::Parameter& param) {camera_id_ = param.as_int();}
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("videos", std::vector<std::string>({""})), 
                [this](const rclcpp::Parameter& param) {videos_ = param.as_string_array();}
            ),
        };
        add_action_parameters(params);
    }

    inline void open_camera()
    {
        if (!is_video_)
        {
            camera_id_ = get_parameter("camera_id").get_value<rclcpp::ParameterType::PARAMETER_INTEGER>();
            RCLCPP_INFO(get_logger(), "Camera %d opening", camera_id_);
            video_capture_.open(camera_id_);
            setHighestResolution(video_capture_);
        }
        else
        {
            auto video_path = videos_[current_video_idx_];
            RCLCPP_INFO(get_logger(), "Video '%s' opening", video_path.c_str());
            video_capture_.open(video_path);
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
        image_info_msg.bayer_format = image.channels() == 1 ? boblib::camera::QhyCamera::BayerFormat::Mono : boblib::camera::QhyCamera::BayerFormat::Color;
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
        camera_info_msg_.id = "web_camera_node";
        camera_info_msg_.model = is_video_ ? "video" : "web camera";
        camera_info_msg_.serial_num = is_video_ ? videos_[current_video_idx_] : std::to_string(camera_id_);
        camera_info_msg_.overscan.start_x = 0;
        camera_info_msg_.overscan.start_y = 0;
        camera_info_msg_.overscan.width = 0;
        camera_info_msg_.overscan.height = 0;
        camera_info_msg_.effective.start_x = 0;
        camera_info_msg_.effective.start_y = 0;
        camera_info_msg_.effective.width = (uint32_t)video_capture_.get(cv::CAP_PROP_FRAME_WIDTH);
        camera_info_msg_.effective.height = (uint32_t)video_capture_.get(cv::CAP_PROP_FRAME_HEIGHT);
        camera_info_msg_.chip.width_mm = 0;
        camera_info_msg_.chip.height_mm = 0;
        camera_info_msg_.chip.pixel_width_um = 0;
        camera_info_msg_.chip.pixel_height_um = 0;
        camera_info_msg_.chip.max_image_width = (uint32_t)video_capture_.get(cv::CAP_PROP_FRAME_WIDTH);
        camera_info_msg_.chip.max_image_height = (uint32_t)video_capture_.get(cv::CAP_PROP_FRAME_HEIGHT);
        camera_info_msg_.chip.max_bpp = 8;
        camera_info_msg_.bayer_format = boblib::camera::QhyCamera::BayerFormat::Color;
        camera_info_msg_.is_color = true;
        camera_info_msg_.is_cool = false;
        camera_info_msg_.has_bin1x1_mode = true;
        camera_info_msg_.has_bin2x2_mode = false;
        camera_info_msg_.has_bin3x3_mode = false;
        camera_info_msg_.has_bin4x4_mode = false;
        camera_info_msg_.gain_limits.min = 0;
        camera_info_msg_.gain_limits.max = 0;
        camera_info_msg_.gain_limits.step = 0;
        camera_info_msg_.offset_limits.min = 0;
        camera_info_msg_.offset_limits.max = 0;
        camera_info_msg_.offset_limits.step = 0;
        camera_info_msg_.usb_traffic_limits.min = 0;
        camera_info_msg_.usb_traffic_limits.max = 0;
        camera_info_msg_.usb_traffic_limits.step = 0;
        camera_info_msg_.red_wb_limits.min = 0;
        camera_info_msg_.red_wb_limits.max = 0;
        camera_info_msg_.red_wb_limits.step = 0;
        camera_info_msg_.green_wb_limits.min = 0;
        camera_info_msg_.green_wb_limits.max = 0;
        camera_info_msg_.green_wb_limits.step = 0;
        camera_info_msg_.blue_wb_limits.min = 0;
        camera_info_msg_.blue_wb_limits.max = 0;
        camera_info_msg_.blue_wb_limits.step = 0;
        camera_info_msg_.temperature_limits.min = 0;
        camera_info_msg_.temperature_limits.max = 0;
        camera_info_msg_.temperature_limits.step = 0;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WebCameraVideo>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}

RCLCPP_COMPONENTS_REGISTER_NODE(WebCameraVideo)