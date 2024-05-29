#include <vector>

#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <sensor_msgs/msg/image.hpp>

#include <boblib/api/utils/profiler.hpp>

#include "parameter_node.hpp"
#include "image_utils.hpp"

#include <visibility_control.h>

#include <rclcpp/experimental/executors/events_executor/events_executor.hpp>

class FrameOutput
    : public ParameterNode
{
public:
    COMPOSITION_PUBLIC
    explicit FrameOutput(const rclcpp::NodeOptions & options) 
        : ParameterNode("frame_output_node", options)
        , resize_height_(960)
        , compression_quality_(0)
        , topic_("bob/frames/annotated")
        , running_(true)
    {
        declare_node_parameters();
        init();
    }

private:
    rclcpp::QoS sub_qos_profile_{10};
    rclcpp::QoS pub_qos_profile_{10};
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_output_compressed_frame_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_output_uncompressed_frame_;
    boblib::utils::Profiler profiler_;
    int resize_height_;
    int compression_quality_;
    std::string topic_;
    bool running_;

    friend std::shared_ptr<FrameOutput> std::make_shared<FrameOutput>();

    void create_pub_sub()
    {
        if (compression_quality_ > 0)
        {
            pub_output_compressed_frame_ = create_publisher<sensor_msgs::msg::CompressedImage>("bob/output/target", pub_qos_profile_);
        }
        else
        {
            pub_output_uncompressed_frame_ = create_publisher<sensor_msgs::msg::Image>("bob/output/target", pub_qos_profile_);
        }
        image_subscription_ = create_subscription<sensor_msgs::msg::Image>(topic_, sub_qos_profile_,
                    std::bind(&FrameOutput::imageCallback, this, std::placeholders::_1));        
    }

    void declare_node_parameters()
    {
        std::vector<ParameterNode::ActionParam> params = {
            ParameterNode::ActionParam(
                rclcpp::Parameter("resize_height", resize_height_), 
                [this](const rclcpp::Parameter& param) {resize_height_ = param.as_int();}
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("compression_quality", compression_quality_), 
                [this](const rclcpp::Parameter& param) {
                    compression_quality_ = param.as_int();
                    if (running_)
                    {
                        create_pub_sub();
                    }
                }
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("topic", topic_), 
                [this](const rclcpp::Parameter& param) {
                    topic_ = param.as_string(); 
                    image_subscription_ = create_subscription<sensor_msgs::msg::Image>(topic_, sub_qos_profile_,
                                std::bind(&FrameOutput::imageCallback, this, std::placeholders::_1));
                }
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("on", running_), 
                [this](const rclcpp::Parameter& param) {
                    running_ = param.as_bool();
                    if (running_)
                    {
                        create_pub_sub();
                    }
                    else
                    {
                        pub_output_compressed_frame_ = nullptr;
                        pub_output_uncompressed_frame_ = nullptr;
                        image_subscription_ = nullptr;
                    }
                }
            ),
        };
        add_action_parameters(params);
    }
    
    void init()
    {
        sub_qos_profile_.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        sub_qos_profile_.durability(rclcpp::DurabilityPolicy::Volatile);
        sub_qos_profile_.history(rclcpp::HistoryPolicy::KeepLast);

        pub_qos_profile_.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        pub_qos_profile_.durability(rclcpp::DurabilityPolicy::Volatile);
        pub_qos_profile_.history(rclcpp::HistoryPolicy::KeepLast);
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr image_msg)
    {
        try
        {
            cv::Mat image;
            ImageUtils::convert_image_msg(image_msg, image, true);

            if (resize_height_ > 0)
            {
                const double aspect_ratio = (double)image.size().width / (double)image.size().height;
                const int frame_height = resize_height_;
                const int frame_width = (int)(aspect_ratio * (double)frame_height);
                cv::resize(image, image, cv::Size(frame_width, frame_height));
            }

            if (compression_quality_ > 0)
            {
                std::vector<int> compression_params;
                compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
                compression_params.push_back(compression_quality_);  // Compression quality

                std::vector<uchar> compressed_image;
                cv::imencode(".jpg", image, compressed_image, compression_params);            

                sensor_msgs::msg::CompressedImage compressed_image_msg;
                compressed_image_msg.header = image_msg->header;
                compressed_image_msg.format = "jpeg";
                compressed_image_msg.data = compressed_image;

                pub_output_compressed_frame_->publish(compressed_image_msg);
            }
            else
            {
                auto resized_frame_msg = cv_bridge::CvImage(image_msg->header, image_msg->encoding, image).toImageMsg();
                pub_output_uncompressed_frame_->publish(*resized_frame_msg);                
            }
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(get_logger(), "CV bridge exception: %s", e.what());
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
    executor.add_node(std::make_shared<FrameOutput>(rclcpp::NodeOptions()));
    executor.spin();
    rclcpp::shutdown();
    return 0;
}

RCLCPP_COMPONENTS_REGISTER_NODE(FrameOutput)