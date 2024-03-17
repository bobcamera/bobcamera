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
#include <sensor_msgs/msg/region_of_interest.hpp>

class MaskApplication 
    : public ParameterNode
{
public:
    COMPOSITION_PUBLIC
    explicit MaskApplication(const rclcpp::NodeOptions & options)
        : ParameterNode("mask_application_node", options), bounding_box_(0, 0, 0, 0) 
    {
        one_shot_timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            [this]() {
                try {
                    init();
                } catch(const std::exception &e) {
                    RCLCPP_ERROR(this->get_logger(), "Exception in MaskApplicationNode constructor: %s", e.what());
                }
                one_shot_timer_.reset();
            }
        );
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

        roi_calc_complete_ = false;

        image_publisher_ = create_publisher<sensor_msgs::msg::Image>("bob/mask/target", pub_qos_profile);
        roi_publisher_ = create_publisher<sensor_msgs::msg::RegionOfInterest>("bob/mask/roi", pub_qos_profile);
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
            ParameterNode::ActionParam(
                rclcpp::Parameter("mask_enable_offset_correction", true), 
                [this](const rclcpp::Parameter& param) {mask_enable_offset_correction_ = param.as_bool();}
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("image_width", 0), 
                [this](const rclcpp::Parameter& param) {image_width_ = param.as_int();}
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("image_height", 0), 
                [this](const rclcpp::Parameter& param) {image_height_ = param.as_int();}
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

                if(img.rows != converted_mask_.rows || img.cols != converted_mask_.cols)
                {
                    RCLCPP_WARN(this->get_logger(), "Frame and mask dimensions do not match. Attempting resize.");
                    RCLCPP_WARN(this->get_logger(), "Note: Please ensure your mask has not gone stale, you might want to recreate it.");
                    cv::resize(converted_mask_, converted_mask_, cv::Size(img.cols, img.rows));
                }

                cv::Mat masked_frame;
                if (mask_enable_offset_correction_)
                {
                    cv::Rect roi(bounding_box_.x, bounding_box_.y, bounding_box_.width, bounding_box_.height);
                    masked_frame = img(roi).mul(converted_mask_(roi)/255.0);
                }
                else
                {
                    masked_frame = img.mul(converted_mask_/255.0);
                }

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
            if (std::filesystem::exists(mask_filename_))
            {
                mask = cv::imread(mask_filename_, cv::IMREAD_UNCHANGED);    
            }

            if (mask.empty())
            {
                if (mask_enabled_)
                {
                    RCLCPP_INFO(get_logger(), "Mask Disabled.");
                    request_bgs_reset();
                    roi_calc_complete_ = false;
                }
                mask_enabled_ = false;                
            }
            else
            {
                if (!mask_enabled_)
                {
                    RCLCPP_INFO(get_logger(), "Mask Enabled.");
                    request_bgs_reset();
                    roi_calc_complete_ = false;
                }

                mask_enabled_ = true;
                
                if (!areImagesEqual(mask, grey_mask_))
                {
                    grey_mask_= mask;
                    cv::cvtColor(mask, converted_mask_, cv::COLOR_GRAY2BGR);
                    request_bgs_reset();
                    roi_calc_complete_ = false;
                }
            }

            if (mask_enable_offset_correction_ && !roi_calc_complete_)
            {
                sensor_msgs::msg::RegionOfInterest roi_msg;
                if(mask.empty())
                {
                    roi_msg.x_offset = 0;
                    roi_msg.y_offset = 0;
                    roi_msg.width = image_width_;
                    roi_msg.height = image_height_;
                }
                else
                {
                    bounding_box_ = cv::Rect(grey_mask_.cols, grey_mask_.rows, 0, 0);                                
                    for (int y = 0; y < grey_mask_.rows; ++y) 
                    {
                        for (int x = 0; x < grey_mask_.cols; ++x) 
                        {
                            if (grey_mask_.at<uchar>(y, x) == 255) 
                            { 
                                bounding_box_.x = std::min(bounding_box_.x, x);
                                bounding_box_.y = std::min(bounding_box_.y, y);
                                bounding_box_.width = std::max(bounding_box_.width, x - bounding_box_.x);
                                bounding_box_.height = std::max(bounding_box_.height, y - bounding_box_.y);
                            }
                        }
                    }

                    roi_msg.x_offset = bounding_box_.x;
                    roi_msg.y_offset = bounding_box_.y;
                    roi_msg.width = bounding_box_.width;
                    roi_msg.height = bounding_box_.height;
                }

                roi_publisher_->publish(roi_msg);
                roi_calc_complete_ = true;

                cv::Size frame_size(roi_msg.width, roi_msg.height);
                RCLCPP_INFO(get_logger(), "Detection frame size determined from mask: %d x %d", frame_size.width, frame_size.height);
            }            
        }
        catch (cv::Exception &cve)
        {
            RCLCPP_ERROR(get_logger(), "Open CV exception on timer callback: %s", cve.what());
        }  
    }

    void request_bgs_reset()
    {
        auto request = std::make_shared<bob_interfaces::srv::BGSResetRequest::Request>();
        request->bgs_params = "";
        if (bgs_reset_client_->service_is_ready())
        {
            auto result = bgs_reset_client_->async_send_request(request, std::bind(&MaskApplication::request_bgs_reset_callback, this, std::placeholders::_1));
        }
    }

    void request_bgs_reset_callback(rclcpp::Client<bob_interfaces::srv::BGSResetRequest>::SharedFuture future)
    {
        auto response = future.get();
        if(response->success)
            RCLCPP_DEBUG(get_logger(), "BGS Reset Successfull");
    }

    void mask_override_request(const std::shared_ptr<bob_interfaces::srv::MaskOverrideRequest::Request> request, 
        std::shared_ptr<bob_interfaces::srv::MaskOverrideRequest::Response> response)
    {
        mask_enable_override_ = request->mask_enabled;
        if (request->mask_enabled)
        {
            RCLCPP_DEBUG(get_logger(), "Mask Override set to: True");
        }
        else
        {
            RCLCPP_DEBUG(get_logger(), "Mask Override set to: False");
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
    cv::Rect bounding_box_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::RegionOfInterest>::SharedPtr roi_publisher_;
    std::string mask_filename_;
    bool mask_enabled_;
    bool mask_enable_override_;
    bool mask_enable_offset_correction_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<bob_interfaces::srv::BGSResetRequest>::SharedPtr bgs_reset_client_;
    rclcpp::Service<bob_interfaces::srv::MaskOverrideRequest>::SharedPtr mask_override_service_;
    rclcpp::TimerBase::SharedPtr one_shot_timer_;
    bool roi_calc_complete_;
    int image_height_;
    int image_width_;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MaskApplication>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}


RCLCPP_COMPONENTS_REGISTER_NODE(MaskApplication)