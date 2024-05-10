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

class AnnotatedFrameProvider : public ParameterNode
{
public:
    COMPOSITION_PUBLIC
    explicit AnnotatedFrameProvider(const rclcpp::NodeOptions & options)
        : ParameterNode("annotated_frame_provider_node", options),
          annotated_frame_creator_(std::map<std::string, std::string>()),
        x_offset_(0),
        y_offset_(0),
        enable_tracking_status_(true)
    {
        timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&AnnotatedFrameProvider::init, this));
    }

private:
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> sub_masked_frame_;
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::RegionOfInterest>> roi_subscription_;
    std::shared_ptr<message_filters::Subscriber<bob_interfaces::msg::Tracking>> sub_tracking_;
    std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, bob_interfaces::msg::Tracking>> time_synchronizer_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_annotated_frame_;
    AnnotatedFrameCreator annotated_frame_creator_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_annotated_frame_compressed_;

    std::unique_ptr<RosCvImageMsg> ros_cv_annotated_frame_;

    int x_offset_;
    int y_offset_;
    bool enable_tracking_status_;

    void init()
    {
        RCLCPP_INFO(get_logger(), "Initializing AnnotatedFrameProvider");

        timer_->cancel();

        rclcpp::QoS pub_qos_profile{10};
        pub_qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        pub_qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);
        pub_qos_profile.history(rclcpp::HistoryPolicy::KeepLast);

        rclcpp::QoS sub_qos_profile{10};
        sub_qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        sub_qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);
        sub_qos_profile.history(rclcpp::HistoryPolicy::KeepLast);
        auto rmw_qos_profile = sub_qos_profile.get_rmw_qos_profile();

        declare_node_parameters();

        sub_masked_frame_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(shared_from_this(), "bob/frames/allsky/original/resized", rmw_qos_profile);
        sub_tracking_ = std::make_shared<message_filters::Subscriber<bob_interfaces::msg::Tracking>>(shared_from_this(), "bob/tracker/tracking/resized", rmw_qos_profile);
        pub_annotated_frame_ = create_publisher<sensor_msgs::msg::Image>("bob/frames/annotated", pub_qos_profile);
        time_synchronizer_ = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, bob_interfaces::msg::Tracking>>(*sub_masked_frame_, *sub_tracking_, 10);
        time_synchronizer_->registerCallback(&AnnotatedFrameProvider::callback, this);

        roi_subscription_ = create_subscription<sensor_msgs::msg::RegionOfInterest>(
            "bob/mask/roi", sub_qos_profile,
            std::bind(&AnnotatedFrameProvider::roi_callback, this, std::placeholders::_1));
    }

    void declare_node_parameters()
    {
        std::vector<ParameterNode::ActionParam> params = {
            ParameterNode::ActionParam(
                rclcpp::Parameter("tracking_status_message", false), 
                [this](const rclcpp::Parameter& param) {enable_tracking_status_ = param.as_bool();}
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

            // if (!ros_cv_annotated_frame_ || (img.size() != ros_cv_annotated_frame_->image_ptr->size()))
            // {
            //     ros_cv_annotated_frame_ = std::make_unique<RosCvImageMsg>(img, sensor_msgs::image_encodings::BGR8, false);
            // }

            annotated_frame_creator_.create_frame(img, *tracking, x_offset_, y_offset_, enable_tracking_status_);

            auto annotated_frame_msg = cv_bridge::CvImage(image_msg->header, sensor_msgs::image_encodings::BGR8, img).toImageMsg();
            pub_annotated_frame_->publish(*annotated_frame_msg);            
        }
        catch (cv::Exception &cve)
        {
            RCLCPP_ERROR(get_logger(), "Open CV exception: %s", cve.what());
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
