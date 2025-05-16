#include <string>
#include <memory>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <bob_interfaces/msg/tracking.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <parameter_node.hpp>
#include <image_utils.hpp>
#include <visibility_control.h>
#include <json/json.h>
#include "annotated_frame/annotated_frame_creator.hpp"
#include <boblib/api/utils/fps_tracker.hpp>
#include <boblib/api/utils/profiler.hpp>
#include <boblib/api/utils/jpeg_compressor.hpp>

class AnnotatedFrameProvider : public ParameterNode
{
public:
    COMPOSITION_PUBLIC
    explicit AnnotatedFrameProvider(const rclcpp::NodeOptions &options)
        : ParameterNode("annotated_frame_provider_node", options), pub_qos_profile_(10), sub_qos_profile_(10)
    {
    }

    void on_configure()
    {
        log_info("Configuring");

        init();
    }

private:
    void init()
    {
        prof_annotated_id_ = profiler_.add_region("Annotated Frame Provider");
        prof_convert_color_id_ = profiler_.add_region("AFP: Convert Color", prof_annotated_id_);
        prof_create_frame_id_ = profiler_.add_region("AFP: Create Frame", prof_annotated_id_);
        prof_publish_id_ = profiler_.add_region("AFP: Publish Frame", prof_annotated_id_);

        pub_qos_profile_.reliability(rclcpp::ReliabilityPolicy::Reliable);
        pub_qos_profile_.durability(rclcpp::DurabilityPolicy::Volatile);
        pub_qos_profile_.history(rclcpp::HistoryPolicy::KeepLast);

        sub_qos_profile_.reliability(rclcpp::ReliabilityPolicy::Reliable);
        sub_qos_profile_.durability(rclcpp::DurabilityPolicy::Volatile);
        sub_qos_profile_.history(rclcpp::HistoryPolicy::KeepLast);

        enable_tracking_status_ = true;

        declare_node_parameters();

        profiler_.set_enabled(true);

        compressed_image_msg_.format = "jpeg";

        fps_tracker_ptr_ = std::make_unique<boblib::utils::FpsTracker>(false, 5);

        annotated_frame_creator_ptr_ = std::make_unique<AnnotatedFrameCreator>(annotated_frame_creator_settings_);

        time_synchronizer_ = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, bob_interfaces::msg::Tracking>>(*sub_masked_frame_, *sub_tracking_, 10);
        time_synchronizer_->registerCallback(&AnnotatedFrameProvider::callback, this);
    }

    void declare_node_parameters()
    {
        std::vector<ParameterNode::ActionParam> params = {
            ParameterNode::ActionParam(
                rclcpp::Parameter("annotated_frame_publisher_topic", "bob/frames/annotated"),
                [this](const rclcpp::Parameter &param)
                {
                    pub_annotated_frame_ = create_publisher<sensor_msgs::msg::Image>(param.as_string(), pub_qos_profile_);
                    image_resized_publish_topic_ = param.as_string() + "/resized";
                }),
            ParameterNode::ActionParam(
                rclcpp::Parameter("camera_frame_topic", "bob/frames/allsky/original/resized"),
                [this](const rclcpp::Parameter &param)
                {
                    sub_masked_frame_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(shared_from_this(), param.as_string(), sub_qos_profile_.get_rmw_qos_profile());
                }),
            ParameterNode::ActionParam(
                rclcpp::Parameter("tracking_topic", "bob/tracker/tracking/resized"),
                [this](const rclcpp::Parameter &param)
                {
                    sub_tracking_ = std::make_shared<message_filters::Subscriber<bob_interfaces::msg::Tracking>>(shared_from_this(), param.as_string(), sub_qos_profile_.get_rmw_qos_profile());
                }),
            ParameterNode::ActionParam(
                rclcpp::Parameter("tracking_status_message", false),
                [this](const rclcpp::Parameter &param)
                {
                    enable_tracking_status_ = param.as_bool();
                }),
            ParameterNode::ActionParam(
                rclcpp::Parameter("visualiser_settings", ""),
                [this](const rclcpp::Parameter &param)
                {
                    annotated_frame_creator_settings_ = parse_json_to_map(param.as_string());
                }),
            ParameterNode::ActionParam(
                rclcpp::Parameter("resize_height", 1024),
                [this](const rclcpp::Parameter &param)
                {
                    resize_height_ = static_cast<int>(param.as_int());
                    if (resize_height_ > 0)
                    {
                        image_resized_publisher_ = create_publisher<sensor_msgs::msg::Image>(image_resized_publish_topic_, pub_qos_profile_);
                        pub_annotated_compressed_frame_ = create_publisher<sensor_msgs::msg::CompressedImage>(image_resized_publish_topic_ + "/compressed", pub_qos_profile_);
                    }
                    else
                    {
                        image_resized_publisher_.reset();
                        log_send_info("Resizer topic disabled");
                    }
                }),
            ParameterNode::ActionParam(
                rclcpp::Parameter("compression_quality", 75),
                [this](const rclcpp::Parameter &param)
                {
                    compression_quality_ = static_cast<int>(param.as_int());
                    jpeg_compressor_ptr_ = std::make_unique<boblib::utils::JpegCompressor>(compression_quality_);
                }),
        };
        add_action_parameters(params);
    }

    void callback(const sensor_msgs::msg::Image::SharedPtr &image_msg,
                  const bob_interfaces::msg::Tracking::SharedPtr &tracking)
    {
        try
        {
            profiler_.start(prof_convert_color_id_);
            cv::Mat img;
            ImageUtils::convert_image_msg(image_msg, img, false);
            profiler_.stop(prof_convert_color_id_);
            profiler_.start(prof_create_frame_id_);

            annotated_frame_creator_ptr_->create_frame(img, *tracking, enable_tracking_status_);
            profiler_.stop(prof_create_frame_id_);
            profiler_.start(prof_publish_id_);
            publish_if_subscriber(pub_annotated_frame_, *image_msg);

            publish_resized_frame(image_msg, img);
            profiler_.stop(prof_publish_id_);

            fps_tracker_ptr_->add_frame();
            double current_fps = 0.0;
            if (fps_tracker_ptr_->get_fps_if_ready(current_fps))
            {
                log_info("annotated: FPS: %g", current_fps);
            }
        }
        catch (const std::exception &e)
        {
            log_send_error("callback: exception: %s", e.what());
        }
    }

    void publish_resized_frame(const sensor_msgs::msg::Image::SharedPtr &annotated_frame_msg, const cv::Mat &img)
    {
        try
        {
            if (resize_height_ <= 0)
            {
                return;
            }
            if (resize_height_ != img.size().height)
            {
                cv::Mat resized_img;
                const auto frame_width = static_cast<int>(img.size().aspectRatio() * static_cast<double>(resize_height_));
                cv::resize(img, resized_img, cv::Size(frame_width, resize_height_));

                if (image_resized_publisher_->get_subscription_count() > 0)
                {
                    auto resized_frame_msg = cv_bridge::CvImage(annotated_frame_msg->header, annotated_frame_msg->encoding, resized_img).toImageMsg();
                    image_resized_publisher_->publish(*resized_frame_msg);
                }

                if (pub_annotated_compressed_frame_->get_subscription_count() > 0)
                {
                    compressed_image_msg_.header = annotated_frame_msg->header;
                    compressed_image_msg_.data.reserve(resized_img.total() * resized_img.elemSize() / 2);

                    jpeg_compressor_ptr_->compress(resized_img, compressed_image_msg_.data);

                    pub_annotated_compressed_frame_->publish(compressed_image_msg_);
                }
            }
            else
            {
                publish_if_subscriber(image_resized_publisher_, *annotated_frame_msg);

                if (pub_annotated_compressed_frame_->get_subscription_count() > 0)
                {
                    compressed_image_msg_.header = annotated_frame_msg->header;

                    jpeg_compressor_ptr_->compress(img, compressed_image_msg_.data);

                    pub_annotated_compressed_frame_->publish(compressed_image_msg_);
                }
            }
        }
        catch (const std::exception &e)
        {
            log_send_error("publish_resized_frame: exception: %s", e.what());
        }
    }

    rclcpp::QoS pub_qos_profile_;
    rclcpp::QoS sub_qos_profile_;

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> sub_masked_frame_;
    std::shared_ptr<message_filters::Subscriber<bob_interfaces::msg::Tracking>> sub_tracking_;
    std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, bob_interfaces::msg::Tracking>> time_synchronizer_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_annotated_frame_;
    std::unique_ptr<AnnotatedFrameCreator> annotated_frame_creator_ptr_;
    std::map<std::string, std::string> annotated_frame_creator_settings_;

    bool enable_tracking_status_ = true;

    int resize_height_ = 960;
    std::string image_resized_publish_topic_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_resized_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_annotated_compressed_frame_;
    std::unique_ptr<boblib::utils::FpsTracker> fps_tracker_ptr_;
    int compression_quality_;

    sensor_msgs::msg::CompressedImage compressed_image_msg_;

    boblib::utils::Profiler profiler_;

    std::unique_ptr<boblib::utils::JpegCompressor> jpeg_compressor_ptr_;

    size_t prof_annotated_id_;
    size_t prof_convert_color_id_;
    size_t prof_create_frame_id_;
    size_t prof_publish_id_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(AnnotatedFrameProvider)
