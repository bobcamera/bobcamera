#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <json/json.h>

#include <boblib/api/utils/profiler.hpp>

#include "parameter_node.hpp"
#include "image_utils.hpp"

#include <visibility_control.h>

class MaskApplication 
    : public ParameterNode
{
public:
    COMPOSITION_PUBLIC
    explicit MaskApplication(const rclcpp::NodeOptions & options)
        : ParameterNode("mask_application_node", options)
    {
        try {
            init();
        } catch(const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Exception in MaskApplicationNode constructor: %s", e.what());
        }
    }

    void init()
    {
        rclcpp::QoS sub_qos_profile(10);
        sub_qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        sub_qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);
        sub_qos_profile.history(rclcpp::HistoryPolicy::KeepLast);

        rclcpp::QoS pub_qos_profile(10);
        pub_qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        pub_qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);
        pub_qos_profile.history(rclcpp::HistoryPolicy::KeepLast);

        declare_node_parameters();
        
        // Load mask
        cv::Mat mask = cv::imread(mask_filename_, cv::IMREAD_UNCHANGED);

        if (mask.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load mask.");
            return;
        }
        else
        {
            cv::cvtColor(mask, converted_mask_, cv::COLOR_GRAY2BGR);
        }

        image_publisher_ = create_publisher<sensor_msgs::msg::Image>("bob/camera/all_sky/bayer_masked", pub_qos_profile);
        image_subscription_ = create_subscription<sensor_msgs::msg::Image>("bob/camera/all_sky/bayer", sub_qos_profile, 
            std::bind(&MaskApplication::callback, this, std::placeholders::_1));

    }

    void declare_node_parameters()
    {
        std::vector<ParameterNode::ActionParam> params = {
            ParameterNode::ActionParam(
                rclcpp::Parameter("mask_file", "mask.pgm"), 
                [this](const rclcpp::Parameter& param) {mask_filename_ = param.as_string();}
            )
        };
        add_action_parameters(params);
    }

private:
    void callback(const sensor_msgs::msg::Image::SharedPtr img_msg)
    {
        try
        {
            cv::Mat img;
            ImageUtils::convert_image_msg(img_msg, img, false);

            // Print dimensions
            // RCLCPP_INFO(this->get_logger(), "Image dimensions: %d x %d", img.cols, img.rows);
            // RCLCPP_INFO(this->get_logger(), "Mask dimensions: %d x %d", converted_mask_.cols, converted_mask_.rows);

            // Check dimensions
            if(img.rows != converted_mask_.rows || img.cols != converted_mask_.cols)
            {
                RCLCPP_WARN(this->get_logger(), "Frame and mask dimensions do not match. Attempting resize.");
                cv::resize(converted_mask_, converted_mask_, cv::Size(img.cols, img.rows));
            }

            cv::Mat masked_frame = img.mul(converted_mask_/255.0);

            // Convert back to ROS Image message and publish
            auto ros_image = cv_bridge::CvImage(img_msg->header, sensor_msgs::image_encodings::BGR8, masked_frame).toImageMsg();
            image_publisher_->publish(*ros_image);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "CV bridge exception: %s", e.what());
        }
        catch (cv::Exception &cve)
        {
            RCLCPP_ERROR(get_logger(), "Open CV exception: %s", cve.what());
        }        
    }

    cv::Mat converted_mask_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    std::string mask_filename_;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MaskApplication>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}


RCLCPP_COMPONENTS_REGISTER_NODE(MaskApplication)