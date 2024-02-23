#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <json/json.h>

#include <boblib/api/utils/profiler.hpp>

#include "parameter_node.hpp"
#include "image_utils.hpp"

#include <visibility_control.h>

#include <filesystem>

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
        
        timer_callback();

        image_publisher_ = create_publisher<sensor_msgs::msg::Image>("bob/camera/all_sky/bayer_masked", pub_qos_profile);
        image_subscription_ = create_subscription<sensor_msgs::msg::Image>("bob/camera/all_sky/bayer", sub_qos_profile, 
            std::bind(&MaskApplication::callback, this, std::placeholders::_1));

        timer_ = create_wall_timer(std::chrono::seconds(60), std::bind(&MaskApplication::timer_callback, this));
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
            if(mask_enabled_)
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
                    RCLCPP_WARN(this->get_logger(), "Note: Please ensure your mask has not gone stale, you might want to recreate it.");
                    cv::resize(converted_mask_, converted_mask_, cv::Size(img.cols, img.rows));
                }

                cv::Mat masked_frame = img.mul(converted_mask_/255.0);

                // Convert back to ROS Image message and publish
                auto ros_image = cv_bridge::CvImage(img_msg->header, sensor_msgs::image_encodings::BGR8, masked_frame).toImageMsg();            
                image_publisher_->publish(*ros_image);
            }
            else
            {
                image_publisher_->publish(*img_msg);
            }
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

    void timer_callback()
    {
        cv::Mat mask;
        // Load mask
        if (std::filesystem::exists(mask_filename_))
        {
            mask = cv::imread(mask_filename_, cv::IMREAD_UNCHANGED);    
        }

        if (mask.empty())
        {
            if (mask_enabled_)
            {
                RCLCPP_INFO(get_logger(), "Mask Disabled.");
            }
            mask_enabled_ = false;            
        }
        else
        {
            if (!mask_enabled_)
            {
                RCLCPP_INFO(get_logger(), "Mask Enabled.");
            }

            mask_enabled_ = true;
            cv::cvtColor(mask, converted_mask_, cv::COLOR_GRAY2BGR);            
        }        
    }

    cv::Mat converted_mask_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    std::string mask_filename_;
    bool mask_enabled_;
    rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MaskApplication>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}


RCLCPP_COMPONENTS_REGISTER_NODE(MaskApplication)