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

class FrameCompressor
    : public ParameterNode
{
public:
    COMPOSITION_PUBLIC
    explicit FrameCompressor(const rclcpp::NodeOptions & options) 
        : ParameterNode("frame_compressor_node", options)
    {
        declare_node_parameters();
        init();
    }

private:
    rclcpp::QoS sub_qos_profile_{2};
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_compressed_frame_;
    boblib::utils::Profiler profiler_;
    int compression_quality_;

    friend std::shared_ptr<FrameCompressor> std::make_shared<FrameCompressor>();

    void declare_node_parameters()
    {
        std::vector<ParameterNode::ActionParam> params = {
            ParameterNode::ActionParam(
                rclcpp::Parameter("compression_quality", 75), 
                [this](const rclcpp::Parameter& param) {compression_quality_ = param.as_int();}
            ),
        };
        add_action_parameters(params);
    }
    
    void init()
    {
        rclcpp::QoS sub_qos_profile{10};
        sub_qos_profile_.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        sub_qos_profile_.durability(rclcpp::DurabilityPolicy::Volatile);
        sub_qos_profile_.history(rclcpp::HistoryPolicy::KeepLast);

        rclcpp::QoS pub_qos_profile{10};
        pub_qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        pub_qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);
        pub_qos_profile.history(rclcpp::HistoryPolicy::KeepLast);

        image_subscription_ = create_subscription<sensor_msgs::msg::Image>("bob/compressor/source", sub_qos_profile_,
            std::bind(&FrameCompressor::imageCallback, this, std::placeholders::_1));

        pub_compressed_frame_ = create_publisher<sensor_msgs::msg::CompressedImage>("bob/compressor/target", pub_qos_profile);
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr image_msg)
    {
        try
        {
            cv::Mat image;
            ImageUtils::convert_image_msg(image_msg, image, true);            

            std::vector<int> compression_params;
            compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
            compression_params.push_back(compression_quality_);  // Compression quality

            std::vector<uchar> compressed_image;
            cv::imencode(".jpg", image, compressed_image, compression_params);            

            sensor_msgs::msg::CompressedImage compressed_image_msg;
            compressed_image_msg.header = image_msg->header;
            compressed_image_msg.format = "jpeg";
            compressed_image_msg.data = compressed_image;

            pub_compressed_frame_->publish(compressed_image_msg);
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
    rclcpp::spin(std::make_shared<FrameCompressor>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}

RCLCPP_COMPONENTS_REGISTER_NODE(FrameCompressor)