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

class FrameViewer
    : public ParameterNode
{
public:
    COMPOSITION_PUBLIC
    explicit FrameViewer(const rclcpp::NodeOptions &options)
        : ParameterNode("frame_viewer_node", options), sub_qos_profile_(4)
    {
    }

    void on_configure()
    {
        log_info("Configuring");

        init();
    }

private:
    void declare_node_parameters()
    {
        std::vector<ParameterNode::ActionParam> params = {
            ParameterNode::ActionParam(
                rclcpp::Parameter("topics", std::vector<std::string>({"bob/frames/allsky/original", "bob/frames/foreground_mask"})),
                [this](const rclcpp::Parameter &param)
                { topics_ = param.as_string_array(); }),
        };
        add_action_parameters(params);
    }

    void init()
    {
        sub_qos_profile_.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        sub_qos_profile_.durability(rclcpp::DurabilityPolicy::Volatile);
        sub_qos_profile_.history(rclcpp::HistoryPolicy::KeepLast);

        current_topic_ = 0;

        declare_node_parameters();

        cv::namedWindow("Image Viewer", cv::WINDOW_NORMAL);
        cv::displayStatusBar("Image Viewer", topics_[current_topic_], 0);

        subscribe_image_topic();
    }

    void check_topics_callback()
    {
        int retries = 0;
        timer_->cancel();
        std::string specific_topic_name = topics_[current_topic_];
        auto topics_and_types = get_topic_names_and_types();

        auto it = topics_and_types.find(specific_topic_name);
        if (it != topics_and_types.end())
        {
            auto &topic_type = it->second[0];
            log_debug("Topic: '%s' has type: '%s'", it->first.c_str(), topic_type.c_str());
            if (topic_type == "sensor_msgs/msg/CompressedImage")
            {
                if (!image_subscription_compressed_)
                {
                    image_subscription_compressed_ = create_subscription<sensor_msgs::msg::CompressedImage>(topics_[current_topic_], sub_qos_profile_,
                                                                                                            [this](const sensor_msgs::msg::CompressedImage::SharedPtr image_msg)
                                                                                                            { image_callback_compressed(image_msg); });
                }
            }
            else if (topic_type == "sensor_msgs/msg/Image")
            {
                if (!image_subscription_)
                {
                    image_subscription_ = create_subscription<sensor_msgs::msg::Image>(topics_[current_topic_], sub_qos_profile_,
                                                                                       [this](const sensor_msgs::msg::Image::SharedPtr image_msg)
                                                                                       { image_callback(image_msg); });
                }
            }
            else
            {
                current_topic_ = current_topic_ < 0 ? topics_.size() - 1 : (current_topic_ >= (int)topics_.size() ? 0 : current_topic_);
                log_warn("Topic: '%s' has type: '%s' and is not supported", it->first.c_str(), topic_type.c_str());
            }
            timer_->reset();
            return;
        }
        image_subscription_compressed_.reset();
        image_subscription_.reset();
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        log_warn("Topic '%s' not found, retries: %d", specific_topic_name.c_str(), retries);
        if (++retries > max_retries)
        {
            retries = 0;
            current_topic_ = ++current_topic_ >= (int)topics_.size() ? 0 : current_topic_;
            log_info("Switching Topic to '%s'", topics_[current_topic_].c_str());
        }
        timer_->reset();
    }

    void subscribe_image_topic()
    {
        timer_ = create_wall_timer(std::chrono::milliseconds(1000), [this]()
                                   { check_topics_callback(); });
    }

    void image_callback_compressed(const sensor_msgs::msg::CompressedImage::SharedPtr image_msg)
    {
        if (image_msg->data.empty())
        {
            return;
        }

        try
        {
            auto current_time = std::chrono::steady_clock::now();
            frame_count++;

            // Update FPS calculation every N frames for stability
            if (frame_count % fps_update_interval == 0)
            {
                auto time_diff = std::chrono::duration_cast<std::chrono::milliseconds>(
                                     current_time - last_time)
                                     .count() /
                                 1000.0;
                fps = fps_update_interval / time_diff;
                last_time = current_time;
                log_info("FPS: %.2f", fps);
            }

            auto img = cv::imdecode(cv::Mat(image_msg->data), cv::IMREAD_COLOR);
            if (img.empty())
            {
                log_error("Decoding failed.");
                return;
            }

            display_image(img);
        }
        catch (const std::exception &e)
        {
            log_error("image_callback_compressed: exception: %s", e.what());
        }
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr image_msg)
    {
        try
        {
            auto current_time = std::chrono::steady_clock::now();
            frame_count++;

            // Update FPS calculation every N frames for stability
            if (frame_count % fps_update_interval == 0)
            {
                auto time_diff = std::chrono::duration_cast<std::chrono::milliseconds>(
                                     current_time - last_time)
                                     .count() /
                                 1000.0;
                fps = fps_update_interval / time_diff;
                last_time = current_time;
                log_info("FPS: %.2f", fps);
            }
            cv::Mat img;
            ImageUtils::convert_image_msg(image_msg, img, true);
            display_image(img);
        }
        catch (const std::exception &e)
        {
            log_error("image_callback: exception: %s", e.what());
        }
    }

    void display_image(const cv::Mat &img)
    {
        try
        {
            cv::imshow("Image Viewer", img);
            int key = cv::waitKey(1);
            bool topic_change = false;
            switch (key)
            {
            case 'q':
                current_topic_--;
                topic_change = true;
                break;
            case 81:
                current_topic_--;
                topic_change = true;
                break;
            case 83:
                current_topic_++;
                topic_change = true;
                break;
            case 'w':
                current_topic_++;
                topic_change = true;
                break;
            }
            if (topic_change)
            {
                current_topic_ = current_topic_ < 0 ? topics_.size() - 1 : (current_topic_ >= (int)topics_.size() ? 0 : current_topic_);
                log_info("Changing topic to %s", topics_[current_topic_].c_str());
                image_subscription_compressed_.reset();
                image_subscription_.reset();
                subscribe_image_topic();
                cv::displayStatusBar("Image Viewer", topics_[current_topic_], 0);
            }
        }
        catch (const std::exception &e)
        {
            log_error("display_image: exception: %s", e.what());
        }
    }

    rclcpp::QoS sub_qos_profile_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_subscription_compressed_;
    std::vector<std::string> topics_;
    int current_topic_;

    std::chrono::steady_clock::time_point last_time = std::chrono::steady_clock::now();
    double fps = 0.0;
    int frame_count = 0;
    const int fps_update_interval = 60;
    int retries_ = 0;
    const int max_retries{3};
};

RCLCPP_COMPONENTS_REGISTER_NODE(FrameViewer)