#include <vector>

#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/bounding_box2_d_array.hpp>

#include "parameter_node.hpp"
#include "image_utils.hpp"

#include <visibility_control.h>

class FrameBBoxViewer
    : public ParameterNode
{
public:
    COMPOSITION_PUBLIC
    explicit FrameBBoxViewer(const rclcpp::NodeOptions & options)
        : ParameterNode("frame_bbox_viewer_node", options), current_topic_{0}
    {
        declare_node_parameters();
        timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&FrameBBoxViewer::init, this));
    }

private:
    rclcpp::QoS sub_qos_profile_{10};
    message_filters::Subscriber<sensor_msgs::msg::Image> sub_image_;
    message_filters::Subscriber<vision_msgs::msg::BoundingBox2DArray> sub_bbox_;
    std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, vision_msgs::msg::BoundingBox2DArray>> time_synchronizer_;
    std::vector<std::string> topics_;
    int current_topic_;
    rclcpp::TimerBase::SharedPtr timer_;

    friend std::shared_ptr<FrameBBoxViewer> std::make_shared<FrameBBoxViewer>();

    void init()
    {
        timer_->cancel();

        sub_qos_profile_.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        sub_qos_profile_.durability(rclcpp::DurabilityPolicy::Volatile);
        sub_qos_profile_.history(rclcpp::HistoryPolicy::KeepLast);
        auto rmw_qos_profile = sub_qos_profile_.get_rmw_qos_profile();

        sub_image_.subscribe(this, topics_[current_topic_], rmw_qos_profile);
        sub_bbox_.subscribe(this, "bob/detector/all_sky/bounding_boxes", rmw_qos_profile);

        time_synchronizer_ = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, vision_msgs::msg::BoundingBox2DArray>>(sub_image_, sub_bbox_, 2);
        time_synchronizer_->registerCallback(&FrameBBoxViewer::imageCallback, this);

        cv::namedWindow("Image Viewer", cv::WINDOW_NORMAL);
        cv::displayStatusBar("Image Viewer", topics_[current_topic_], 0);
    }

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

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr &image_msg, const vision_msgs::msg::BoundingBox2DArray::SharedPtr &bbox_msg)
    {
        try
        {
            cv::Mat img;
            ImageUtils::convert_image_msg(image_msg, img, true);

            for (const auto &bbox2D : bbox_msg->boxes)
            {
                auto bbox = cv::Rect(bbox2D.center.position.x - bbox2D.size_x / 2, bbox2D.center.position.y - bbox2D.size_y / 2, bbox2D.size_x, bbox2D.size_y);
                cv::rectangle(img, bbox, cv::Scalar(255, 0, 255), 5, 1);
            }

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

                sub_image_.unsubscribe();
                sub_image_.subscribe(this, topics_[current_topic_], sub_qos_profile_.get_rmw_qos_profile());

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
    rclcpp::spin(std::make_shared<FrameBBoxViewer>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}

RCLCPP_COMPONENTS_REGISTER_NODE(FrameBBoxViewer)