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
    static std::shared_ptr<FrameCompressor> create()
    {
        auto result = std::shared_ptr<FrameCompressor>(new FrameCompressor());
        result->init();
        return result;
    }

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

    friend std::shared_ptr<FrameCompressor> std::make_shared<FrameCompressor>();

    FrameCompressor() 
        : ParameterNode("frame_compressor")
    {
        declare_node_parameters();
    }

    int resize_height_;

    void declare_node_parameters()
    {
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
            
            // TODO: @Fabio, could you please fix this for me.
            //compressed_msg: CompressedImage = CompressedImage()
            //compressed_msg.header = msg_source.header
            //compressed_msg.format = 'jpeg'
            //compressed_msg.data = cv2.imencode('.jpg', source_frame)[1].tobytes()

            //pub_resized_frame_->publish(*compressed_msg);
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
    rclcpp::spin(FrameCompressor::create());
    rclcpp::shutdown();
    return 0;
}

RCLCPP_COMPONENTS_REGISTER_NODE(FrameCompressor)