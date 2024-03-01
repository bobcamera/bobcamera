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

#include "bob_interfaces/srv/bgs_reset_request.hpp"
#include "bob_interfaces/srv/mask_override_request.hpp"

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

        image_publisher_ = create_publisher<sensor_msgs::msg::Image>("bob/mask/target", pub_qos_profile);
        image_subscription_ = create_subscription<sensor_msgs::msg::Image>("bob/mask/source", sub_qos_profile, 
            std::bind(&MaskApplication::callback, this, std::placeholders::_1));

        timer_ = create_wall_timer(std::chrono::seconds(60), std::bind(&MaskApplication::timer_callback, this));

        bgs_reset_client_ = create_client<bob_interfaces::srv::BGSResetRequest>("bob/bgs/reset");

        mask_override_service_ = create_service<bob_interfaces::srv::MaskOverrideRequest>(
            "bob/mask/override", 
            std::bind(&MaskApplication::mask_override_request, 
            this, 
            std::placeholders::_1, 
            std::placeholders::_2));

        timer_callback();
    }

    void declare_node_parameters()
    {
        std::vector<ParameterNode::ActionParam> params = {
            ParameterNode::ActionParam(
                rclcpp::Parameter("mask_file", "mask.pgm"), 
                [this](const rclcpp::Parameter& param) {mask_filename_ = param.as_string();}
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("mask_enable_override", true), 
                [this](const rclcpp::Parameter& param) {mask_enable_override_ = param.as_bool();}
            ),
        };
        add_action_parameters(params);
    }

private:
    void callback(const sensor_msgs::msg::Image::SharedPtr img_msg)
    {
        try
        {
            if (mask_enable_override_ && mask_enabled_)
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
        try
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
                    request_bgs_reset(false);
                }
                mask_enabled_ = false;            
            }
            else
            {
                if (!mask_enabled_)
                {
                    RCLCPP_INFO(get_logger(), "Mask Enabled.");
                    request_bgs_reset(true);
                }

                mask_enabled_ = true;
                
                if (!areImagesEqual(mask, grey_mask_))
                {
                    grey_mask_= mask;
                    cv::cvtColor(mask, converted_mask_, cv::COLOR_GRAY2BGR);
                    request_bgs_reset(true);
                }
            }
        }
        catch (cv::Exception &cve)
        {
            RCLCPP_ERROR(get_logger(), "Open CV exception on timer callback: %s", cve.what());
        }  
    }
  
    void request_bgs_reset(const bool enabled)
    {
        auto request = std::make_shared<bob_interfaces::srv::BGSResetRequest::Request>();
        request->mask_enabled = enabled;
        if (bgs_reset_client_->service_is_ready())
        {
            auto result = bgs_reset_client_->async_send_request(request, std::bind(&MaskApplication::request_bgs_reset_callback, this, std::placeholders::_1));
        }
    }

    void request_bgs_reset_callback(rclcpp::Client<bob_interfaces::srv::BGSResetRequest>::SharedFuture future)
    {
        auto response = future.get();
        if(response->success)
            RCLCPP_INFO(get_logger(), "BGS Reset Successfull");
    }

    void mask_override_request(const std::shared_ptr<bob_interfaces::srv::MaskOverrideRequest::Request> request, 
        std::shared_ptr<bob_interfaces::srv::MaskOverrideRequest::Response> response)
    {
        mask_enable_override_ = request->mask_enabled;
        if (request->mask_enabled)
        {
            RCLCPP_INFO(get_logger(), "Mask Override set to: True");
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Mask Override set to: True");
        }
        response->success = true;        
    }

    bool areImagesEqual(const cv::Mat& image1, const cv::Mat& image2) 
    {
        // Check if dimensions are the same
        if (image1.size() != image2.size()) {
            return false;
        }

        // Compute absolute difference between images
        cv::Mat diff;
        absdiff(image1, image2, diff);

        // Check if the images are identical (all pixels are equal)
        return countNonZero(diff) == 0;
    }

    cv::Mat grey_mask_;
    cv::Mat converted_mask_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    std::string mask_filename_;
    bool mask_enabled_;
    bool mask_enable_override_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<bob_interfaces::srv::BGSResetRequest>::SharedPtr bgs_reset_client_;
    rclcpp::Service<bob_interfaces::srv::MaskOverrideRequest>::SharedPtr mask_override_service_;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MaskApplication>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}


RCLCPP_COMPONENTS_REGISTER_NODE(MaskApplication)