#include <vector>

#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <sensor_msgs/msg/image.hpp>
#include <bob_interfaces/msg/detector_b_box_array.hpp>

#include "parameter_node.hpp"
#include "image_utils.hpp"

#include <visibility_control.h>

class FrameViewerBlobs
    : public ParameterNode
{
public:
    COMPOSITION_PUBLIC
    explicit FrameViewerBlobs(const rclcpp::NodeOptions &options)
        : ParameterNode("frame_viewer_blobs_node", options), sub_qos_profile_(4)
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
            ParameterNode::ActionParam(
                rclcpp::Parameter("blob_topic", "bob/detection/allsky/boundingboxes"),
                [this](const rclcpp::Parameter &param)
                { blob_topic_ = param.as_string(); }),
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
                sub_image_ptr_.reset();
                time_synchronizer_.reset();
                sub_image_compressed_ptr_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::CompressedImage>>(shared_from_this(), topics_[current_topic_], sub_qos_profile_.get_rmw_qos_profile());
                sub_bbox_ptr_ = std::make_shared<message_filters::Subscriber<bob_interfaces::msg::DetectorBBoxArray>>(shared_from_this(), blob_topic_, sub_qos_profile_.get_rmw_qos_profile());

                time_synchronizer_compressed_ = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::CompressedImage, bob_interfaces::msg::DetectorBBoxArray>>(*sub_image_compressed_ptr_, *sub_bbox_ptr_, 10);
                time_synchronizer_compressed_->registerCallback(&FrameViewerBlobs::image_callback_compressed, this);
            }
            else if (topic_type == "sensor_msgs/msg/Image")
            {
                sub_image_compressed_ptr_.reset();
                time_synchronizer_compressed_.reset();
                sub_image_ptr_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(shared_from_this(), topics_[current_topic_], sub_qos_profile_.get_rmw_qos_profile());
                sub_bbox_ptr_ = std::make_shared<message_filters::Subscriber<bob_interfaces::msg::DetectorBBoxArray>>(shared_from_this(), blob_topic_, sub_qos_profile_.get_rmw_qos_profile());

                time_synchronizer_ = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, bob_interfaces::msg::DetectorBBoxArray>>(*sub_image_ptr_, *sub_bbox_ptr_, 10);
                time_synchronizer_->registerCallback(&FrameViewerBlobs::image_callback, this);
            }
            else
            {
                current_topic_ = current_topic_ < 0 ? topics_.size() - 1 : (current_topic_ >= (int)topics_.size() ? 0 : current_topic_);
                log_warn("Topic: '%s' has type: '%s' and is not supported", it->first.c_str(), topic_type.c_str());
                timer_->reset();
            }
            return;
        }
        log_warn("Topic '%s' not found", specific_topic_name.c_str());
        timer_->reset();
    }

    void subscribe_image_topic()
    {
        timer_ = create_wall_timer(std::chrono::milliseconds(1000), [this]()
                                   { check_topics_callback(); });
    }

    void image_callback_compressed(const sensor_msgs::msg::CompressedImage::SharedPtr & image_msg,
                                   const bob_interfaces::msg::DetectorBBoxArray::SharedPtr & bounding_boxes_msg)
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
                log_error("Decoding failed.");
                return;
            }

            display_image(img, bounding_boxes_msg->detections);
        }
        catch (const std::exception &e)
        {
            log_error("image_callback_compressed: exception: %s", e.what());
        }
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr & image_msg,
                        const bob_interfaces::msg::DetectorBBoxArray::SharedPtr & bounding_boxes_msg)
    {
        try
        {
            cv::Mat img;
            ImageUtils::convert_image_msg(image_msg, img, true);
            display_image(img, bounding_boxes_msg->detections);
        }
        catch (const std::exception &e)
        {
            log_error("image_callback: exception: %s", e.what());
        }
    }

    void display_image(const cv::Mat & img, const std::vector<bob_interfaces::msg::DetectorBBox> & bboxes_msg)
    {
        try
        {
            for (const auto & bbox_msg : bboxes_msg)
            {
                auto bbox = cv::Rect(bbox_msg.x, bbox_msg.y, bbox_msg.width, bbox_msg.height);
                cv::rectangle(img, bbox, cv::Scalar(255, 0, 255), 2, 1);
            }

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
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> sub_image_ptr_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::CompressedImage>> sub_image_compressed_ptr_;
    std::shared_ptr<message_filters::Subscriber<bob_interfaces::msg::DetectorBBoxArray>> sub_bbox_ptr_;
    std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, bob_interfaces::msg::DetectorBBoxArray>> time_synchronizer_;
    std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::CompressedImage, bob_interfaces::msg::DetectorBBoxArray>> time_synchronizer_compressed_;
    std::vector<std::string> topics_;
    std::string blob_topic_;
    int current_topic_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(FrameViewerBlobs)