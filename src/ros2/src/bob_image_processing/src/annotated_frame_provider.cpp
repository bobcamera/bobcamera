#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/region_of_interest.hpp>
#include <bob_interfaces/msg/tracking.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include "annotated_frame/annotated_frame_creator.hpp"
#include "parameter_node.hpp"
#include "image_utils.hpp"
#include <visibility_control.h>

#include <rclcpp/experimental/executors/events_executor/events_executor.hpp>

class AnnotatedFrameProvider 
    : public ParameterNode
{
public:
    COMPOSITION_PUBLIC
    explicit AnnotatedFrameProvider(const rclcpp::NodeOptions & options)
        : ParameterNode("annotated_frame_provider_node", options)
        ,  annotated_frame_creator_(std::map<std::string, std::string>())
        , x_offset_(0)
        , y_offset_(0)
        , enable_tracking_status_(true)
    {
        timer_ = create_wall_timer(std::chrono::seconds(1), [this](){ init(); });
    }

private:
    rclcpp::QoS pub_qos_profile_{10};

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> sub_masked_frame_;
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::RegionOfInterest>> roi_subscription_;
    std::shared_ptr<message_filters::Subscriber<bob_interfaces::msg::Tracking>> sub_tracking_;
    std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, bob_interfaces::msg::Tracking>> time_synchronizer_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_annotated_frame_;
    AnnotatedFrameCreator annotated_frame_creator_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<RosCvImageMsg> ros_cv_annotated_frame_;

    int x_offset_;
    int y_offset_;
    bool enable_tracking_status_;

    std::unique_ptr<RosCvImageMsg> roscv_image_resize_msg_ptr;
    int resize_height_;
    std::string image_resized_publish_topic_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_resized_publisher_;

    void init()
    {
        RCLCPP_INFO(get_logger(), "Initializing AnnotatedFrameProvider");

        timer_->cancel();

        pub_qos_profile_.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        pub_qos_profile_.durability(rclcpp::DurabilityPolicy::Volatile);
        pub_qos_profile_.history(rclcpp::HistoryPolicy::KeepLast);

        rclcpp::QoS sub_qos_profile{10};
        sub_qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        sub_qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);
        sub_qos_profile.history(rclcpp::HistoryPolicy::KeepLast);
        auto rmw_qos_profile = sub_qos_profile.get_rmw_qos_profile();

        declare_node_parameters();

        sub_masked_frame_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(shared_from_this(), "bob/frames/allsky/original/resized", rmw_qos_profile);
        sub_tracking_ = std::make_shared<message_filters::Subscriber<bob_interfaces::msg::Tracking>>(shared_from_this(), "bob/tracker/tracking/resized", rmw_qos_profile);
        pub_annotated_frame_ = create_publisher<sensor_msgs::msg::Image>("bob/frames/annotated", pub_qos_profile_);
        time_synchronizer_ = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, bob_interfaces::msg::Tracking>>(*sub_masked_frame_, *sub_tracking_, 10);
        time_synchronizer_->registerCallback(&AnnotatedFrameProvider::callback, this);

        roi_subscription_ = create_subscription<sensor_msgs::msg::RegionOfInterest>("bob/mask/roi", sub_qos_profile, [this](const sensor_msgs::msg::RegionOfInterest::SharedPtr roi_msg) { roi_callback(roi_msg); });
    }

    void declare_node_parameters()
    {
        std::vector<ParameterNode::ActionParam> params = {
            ParameterNode::ActionParam(
                rclcpp::Parameter("tracking_status_message", false), 
                [this](const rclcpp::Parameter& param) {enable_tracking_status_ = param.as_bool();}
            ),                     
            ParameterNode::ActionParam(
                rclcpp::Parameter("image_resized_publish_topic", "bob/frames/allsky/original/resized"), 
                [this](const rclcpp::Parameter& param) {
                    image_resized_publish_topic_ = param.as_string();
                    if (!image_resized_publish_topic_.empty())
                    {
                        image_resized_publisher_ = create_publisher<sensor_msgs::msg::Image>(image_resized_publish_topic_, pub_qos_profile_);
                    }
                    else
                    {
                        image_resized_publisher_.reset();
                        RCLCPP_INFO(get_logger(), "Resizer topic disabled");
                    }
                }
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("resize_height", 960), 
                [this](const rclcpp::Parameter& param) 
                {
                    resize_height_ = static_cast<int>(param.as_int());
                }
            ),
        };
        add_action_parameters(params);
    }

    void roi_callback(const sensor_msgs::msg::RegionOfInterest::SharedPtr roi_msg) 
    {
        x_offset_ = roi_msg->x_offset;
        y_offset_ = roi_msg->y_offset;
    }

    void callback(const sensor_msgs::msg::Image::SharedPtr& image_msg,
                  const bob_interfaces::msg::Tracking::SharedPtr& tracking)
    {
        try
        {
            cv::Mat img;
            ImageUtils::convert_image_msg(image_msg, img, true);

            annotated_frame_creator_.create_frame(img, *tracking, x_offset_, y_offset_, enable_tracking_status_);

            pub_annotated_frame_->publish(*image_msg);

            publish_resized_frame(image_msg, img);
        }
        catch (cv::Exception &cve)
        {
            RCLCPP_ERROR(get_logger(), "Open CV exception: %s", cve.what());
        }        
    }

    inline void publish_resized_frame(const std::shared_ptr<sensor_msgs::msg::Image> & annotated_frame_msg, const cv::Mat & img) const
    {
        if (!image_resized_publisher_ || (count_subscribers(image_resized_publish_topic_) <= 0))
        {
            return;
        }
        if ((resize_height_ > 0) && (resize_height_ != img.size().height))
        {
            cv::Mat resized_img;
            const auto frame_width = (int)(img.size().aspectRatio() * (double)resize_height_);
            cv::resize(img, resized_img, cv::Size(frame_width, resize_height_));

            auto resized_frame_msg = cv_bridge::CvImage(annotated_frame_msg->header, annotated_frame_msg->encoding, resized_img).toImageMsg();
            image_resized_publisher_->publish(*resized_frame_msg);
        }
        else
        {
            image_resized_publisher_->publish(*annotated_frame_msg);
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::experimental::executors::EventsExecutor executor;
    executor.add_node(std::make_shared<AnnotatedFrameProvider>(rclcpp::NodeOptions()));
    executor.spin();
    rclcpp::shutdown();
    return 0;
}

RCLCPP_COMPONENTS_REGISTER_NODE(AnnotatedFrameProvider)
