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
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_subscription_compressed_;
    std::vector<std::string> topics_;
    int current_topic_;

    friend std::shared_ptr<FrameViewer> std::make_shared<FrameViewer>();

    void declare_node_parameters()
    {
        std::vector<ParameterNode::ActionParam> params = {
            ParameterNode::ActionParam(
                rclcpp::Parameter("topics", std::vector<std::string>({"bob/frames/allsky/original", "bob/frames/foreground_mask"})), 
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

        cv::namedWindow("Image Viewer", cv::WINDOW_NORMAL);
        cv::displayStatusBar("Image Viewer", topics_[current_topic_], 0);

        subscribe_image_topic();
    }

    void check_topics_callback()
    {
        timer_->cancel();
        std::string specific_topic_name = topics_[current_topic_];
        auto topics_and_types = this->get_topic_names_and_types();

        // for (const auto& topic_type : topics_and_types)
        //     for (const auto &type : topic_type.second)
        //         RCLCPP_INFO(this->get_logger(), "Topic: '%s' has type: '%s'", topic_type.first.c_str(), type.c_str());

        auto it = topics_and_types.find(specific_topic_name);
        if (it != topics_and_types.end()) 
        {
            auto &topic_type = it->second[0];
            RCLCPP_INFO(this->get_logger(), "Topic: '%s' has type: '%s'", it->first.c_str(), topic_type.c_str());
            if (topic_type == "sensor_msgs/msg/CompressedImage")
            {
                image_subscription_.reset();
                image_subscription_compressed_ = create_subscription<sensor_msgs::msg::CompressedImage>(topics_[current_topic_], sub_qos_profile_,
                    std::bind(&FrameViewer::image_callback_compressed, this, std::placeholders::_1));
            }
            else if (topic_type == "sensor_msgs/msg/Image")
            {
                image_subscription_compressed_.reset();
                image_subscription_ = create_subscription<sensor_msgs::msg::Image>(topics_[current_topic_], sub_qos_profile_,
                    std::bind(&FrameViewer::image_callback, this, std::placeholders::_1));
            }
            else
            {
                current_topic_ = current_topic_ < 0 ? topics_.size() - 1 : (current_topic_ >= (int)topics_.size() ? 0 : current_topic_);
                RCLCPP_WARN(this->get_logger(), "Topic: '%s' has type: '%s' and is not supported", it->first.c_str(), topic_type.c_str());
                timer_->reset();
            }
            return;
        } 
        RCLCPP_WARN(this->get_logger(), "Topic '%s' not found", specific_topic_name.c_str());
        timer_->reset();
    }

    void subscribe_image_topic()
    {
        timer_ = create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&FrameViewer::check_topics_callback, this));
    }

    void image_callback_compressed(const sensor_msgs::msg::CompressedImage::SharedPtr image_msg)
    {
        if (image_msg->data.empty()) 
        {
            return;
        }

        try 
        {
            cv::Mat img = cv::imdecode(cv::Mat(image_msg->data), cv::IMREAD_COLOR);

            if (img.empty()) 
            {
                RCLCPP_ERROR(this->get_logger(), "Decoding failed.");
                return;
            } 

            display_image(img);
        } 
        catch (const cv::Exception& e) 
        {
            RCLCPP_ERROR(this->get_logger(), "cv::Exception: %s", e.what());
        }
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr image_msg)
    {
        cv::Mat img;
        ImageUtils::convert_image_msg(image_msg, img, true);
        display_image(img);
    }

    void display_image(const cv::Mat & img)
    {
        try
        {
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
                subscribe_image_topic();
                cv::displayStatusBar("Image Viewer", topics_[current_topic_], 0);
            }
        }
        catch (cv::Exception &e)
        {
            RCLCPP_ERROR(get_logger(), "CV exception: %s", e.what());
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