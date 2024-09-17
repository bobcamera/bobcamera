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
#include <parameter_lifecycle_node.hpp>
#include <image_utils.hpp>
#include <visibility_control.h>
#include <json/json.h>
#include "annotated_frame/annotated_frame_creator.hpp"

class AnnotatedFrameProvider : public ParameterLifeCycleNode
{
public:
    COMPOSITION_PUBLIC
    explicit AnnotatedFrameProvider(const rclcpp::NodeOptions &options)
        : ParameterLifeCycleNode("annotated_frame_provider_node", options), pub_qos_profile_(10), sub_qos_profile_(10)
    {
    }

    CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
    {
        log_info("Configuring");

        init();

        return CallbackReturn::SUCCESS;
    }

private:
    void init()
    {
        pub_qos_profile_.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        pub_qos_profile_.durability(rclcpp::DurabilityPolicy::Volatile);
        pub_qos_profile_.history(rclcpp::HistoryPolicy::KeepLast);

        sub_qos_profile_.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        sub_qos_profile_.durability(rclcpp::DurabilityPolicy::Volatile);
        sub_qos_profile_.history(rclcpp::HistoryPolicy::KeepLast);

        enable_tracking_status_ = true;

        declare_node_parameters();

        annotated_frame_creator_ptr_ = std::make_unique<AnnotatedFrameCreator>(annotated_frame_creator_settings_);

        time_synchronizer_ = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, bob_interfaces::msg::Tracking>>(*sub_masked_frame_, *sub_tracking_, 10);
        time_synchronizer_->registerCallback(&AnnotatedFrameProvider::callback, this);
    }

    void declare_node_parameters()
    {
        std::vector<ParameterLifeCycleNode::ActionParam> params = {
            ParameterLifeCycleNode::ActionParam(
                rclcpp::Parameter("annotated_frame_publisher_topic", "bob/frames/annotated"),
                [this](const rclcpp::Parameter &param)
                {
                    pub_annotated_frame_ = create_publisher<sensor_msgs::msg::Image>(param.as_string(), pub_qos_profile_);
                    image_resized_publish_topic_ = param.as_string() + "/resized";
                }),
            ParameterLifeCycleNode::ActionParam(
                rclcpp::Parameter("camera_frame_topic", "bob/frames/allsky/original/resized"),
                [this](const rclcpp::Parameter &param)
                {
                    sub_masked_frame_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image, rclcpp_lifecycle::LifecycleNode>>(shared_from_this(), param.as_string(), sub_qos_profile_.get_rmw_qos_profile());
                }),
            ParameterLifeCycleNode::ActionParam(
                rclcpp::Parameter("tracking_topic", "bob/tracker/tracking/resized"),
                [this](const rclcpp::Parameter &param)
                {
                    sub_tracking_ = std::make_shared<message_filters::Subscriber<bob_interfaces::msg::Tracking, rclcpp_lifecycle::LifecycleNode>>(shared_from_this(), param.as_string(), sub_qos_profile_.get_rmw_qos_profile());
                }),
            ParameterLifeCycleNode::ActionParam(
                rclcpp::Parameter("tracking_status_message", false),
                [this](const rclcpp::Parameter &param)
                {
                    enable_tracking_status_ = param.as_bool();
                }),
            ParameterLifeCycleNode::ActionParam(
                rclcpp::Parameter("visualiser_settings", ""),
                [this](const rclcpp::Parameter &param)
                {
                    annotated_frame_creator_settings_ = parse_json_to_map(param.as_string());
                }),
            ParameterLifeCycleNode::ActionParam(
                rclcpp::Parameter("resize_height", 960),
                [this](const rclcpp::Parameter &param)
                {
                    resize_height_ = static_cast<int>(param.as_int());
                    if (resize_height_ > 0)
                    {
                        image_resized_publisher_ = create_publisher<sensor_msgs::msg::Image>(image_resized_publish_topic_, pub_qos_profile_);
                    }
                    else
                    {
                        image_resized_publisher_.reset();
                        log_send_info("Resizer topic disabled");
                    }
                }),
        };
        add_action_parameters(params);
    }

    void callback(const sensor_msgs::msg::Image::SharedPtr &image_msg,
                  const bob_interfaces::msg::Tracking::SharedPtr &tracking)
    {
        try
        {
            cv::Mat img;
            ImageUtils::convert_image_msg(image_msg, img, true);

            annotated_frame_creator_ptr_->create_frame(img, *tracking, enable_tracking_status_);

            pub_annotated_frame_->publish(*image_msg);

            publish_resized_frame(image_msg, img);
        }
        catch (const std::exception &e)
        {
            log_send_error("callback: exception: %s", e.what());
        }
    }

    void publish_resized_frame(const sensor_msgs::msg::Image::SharedPtr &annotated_frame_msg, const cv::Mat &img) const
    {
        try
        {
            if (!image_resized_publisher_ || (resize_height_ <= 0) || (count_subscribers(image_resized_publish_topic_) <= 0))
            {
                return;
            }
            if ((resize_height_ > 0) && (resize_height_ != img.size().height))
            {
                cv::Mat resized_img;
                const auto frame_width = static_cast<int>(img.size().aspectRatio() * static_cast<double>(resize_height_));
                cv::resize(img, resized_img, cv::Size(frame_width, resize_height_));

                auto resized_frame_msg = cv_bridge::CvImage(annotated_frame_msg->header, annotated_frame_msg->encoding, resized_img).toImageMsg();
                image_resized_publisher_->publish(*resized_frame_msg);
            }
            else
            {
                image_resized_publisher_->publish(*annotated_frame_msg);
            }
        }
        catch (const std::exception &e)
        {
            log_send_error("publish_resized_frame: exception: %s", e.what());
        }
    }

    rclcpp::QoS pub_qos_profile_;
    rclcpp::QoS sub_qos_profile_;

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image, rclcpp_lifecycle::LifecycleNode>> sub_masked_frame_;
    std::shared_ptr<message_filters::Subscriber<bob_interfaces::msg::Tracking, rclcpp_lifecycle::LifecycleNode>> sub_tracking_;
    std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, bob_interfaces::msg::Tracking>> time_synchronizer_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_annotated_frame_;
    std::unique_ptr<AnnotatedFrameCreator> annotated_frame_creator_ptr_;
    std::map<std::string, std::string> annotated_frame_creator_settings_;

    bool enable_tracking_status_ = true;

    int resize_height_ = 960;
    std::string image_resized_publish_topic_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_resized_publisher_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(AnnotatedFrameProvider)
