#include <vector>

#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <sensor_msgs/msg/image.hpp>

#include "parameter_node.hpp"
#include "image_utils.hpp"

#include <visibility_control.h>

#include <rclcpp/experimental/executors/events_executor/events_executor.hpp>

class FrameViewer
    : public ParameterNode
{
public:
    COMPOSITION_PUBLIC
    explicit FrameViewer(const rclcpp::NodeOptions & options) 
        : ParameterNode("frame_viewer_node", options)
        , current_topic_{0}
    {
        declare_node_parameters();
        init();
    }

private:
    rclcpp::QoS sub_qos_profile_{10};
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    std::vector<std::string> topics_;
    int current_topic_;

    friend std::shared_ptr<FrameViewer> std::make_shared<FrameViewer>();

    void declare_node_parameters()
    {
        std::vector<ParameterNode::ActionParam> params = {
            ParameterNode::ActionParam(
                rclcpp::Parameter("topics", std::vector<std::string>({"bob/frames/allsky/masked/detection", "bob/frames/foreground_mask"})), 
                [this](const rclcpp::Parameter& param) {topics_ = param.as_string_array();}
            ),
        };
        add_action_parameters(params);
    }
    
    void init()
    {
        sub_qos_profile_.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        sub_qos_profile_.durability(rclcpp::DurabilityPolicy::Volatile);
        sub_qos_profile_.history(rclcpp::HistoryPolicy::KeepLast);

        image_subscription_ = create_subscription<sensor_msgs::msg::Image>(topics_[current_topic_], sub_qos_profile_,
            std::bind(&FrameViewer::imageCallback, this, std::placeholders::_1));

        cv::namedWindow("Image Viewer", cv::WINDOW_NORMAL);
        cv::displayStatusBar("Image Viewer", topics_[current_topic_], 0);
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr image_msg)
    {
        try
        {
            cv::Mat img;
            ImageUtils::convert_image_msg(image_msg, img, true);

            cv::imshow("Image Viewer", img);
            int key = cv::waitKey(1);
            bool topic_change = false;
            switch (key)
            {
                case 'q': current_topic_--; topic_change = true; break;
                case 81: current_topic_--; topic_change = true; break;
                case 83: current_topic_++; topic_change = true; break;
                case 'w': current_topic_++; topic_change = true; break;
            }
            if (topic_change)
            {
                current_topic_ = current_topic_ < 0 ? topics_.size() - 1 : (current_topic_ >= (int)topics_.size() ? 0 : current_topic_);
                RCLCPP_INFO(get_logger(), "Changing topic to %s", topics_[current_topic_].c_str());
                image_subscription_ = create_subscription<sensor_msgs::msg::Image>(topics_[current_topic_], sub_qos_profile_,
                    std::bind(&FrameViewer::imageCallback, this, std::placeholders::_1));
                cv::displayStatusBar("Image Viewer", topics_[current_topic_], 0);
            }
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(get_logger(), "CV bridge exception: %s", e.what());
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::experimental::executors::EventsExecutor executor;
    executor.add_node(std::make_shared<FrameViewer>(rclcpp::NodeOptions()));
    executor.spin();
    rclcpp::shutdown();
    return 0;
}

RCLCPP_COMPONENTS_REGISTER_NODE(FrameViewer)