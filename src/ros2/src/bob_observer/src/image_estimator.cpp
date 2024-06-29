#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <sensor_msgs/msg/image.hpp>

#include <bob_interfaces/msg/observer_cloud_estimation.hpp>
#include <bob_interfaces/msg/observer_day_night.hpp>

#include <parameter_lifecycle_node.hpp>
#include <image_utils.hpp>
#include <day_night.hpp>
#include <mask_worker.hpp>

#include "cloud_estimator_worker.hpp"
#include "day_night_classifier_worker.hpp"

#include <visibility_control.h>

class ImageEstimator
    : public ParameterLifeCycleNode
{
public:
    COMPOSITION_PUBLIC
    explicit ImageEstimator(const rclcpp::NodeOptions & options) 
        : ParameterLifeCycleNode("cloud_estimator_node", options)
    {
    }

    CallbackReturn on_configure(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Configuring");

        init();

        return CallbackReturn::SUCCESS;
    }

private:
    rclcpp::QoS sub_qos_profile_{4};
    rclcpp::QoS pub_qos_profile_{4};
    int timer_interval_;
    cv::Mat image_;
    std::unique_ptr<DayTimeCloudEstimator> day_cloud_estimator_worker_ptr_;
    std::unique_ptr<NightTimeCloudEstimator> night_cloud_estimator_worker_ptr_;
    int observer_day_night_brightness_threshold_;
    DayNightEnum day_night_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<bob_interfaces::msg::ObserverCloudEstimation>::SharedPtr pub_cloud_data_;
    rclcpp::Publisher<bob_interfaces::msg::ObserverDayNight>::SharedPtr pub_day_night_data_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_camera_;
    std::unique_ptr<MaskWorker> mask_worker_ptr_;
    bool mask_enabled_;
    cv::Mat detection_mask_;
    int mask_timer_seconds_;
    std::string mask_filename_;

    void declare_node_parameters()
    {
        std::vector<ParameterLifeCycleNode::ActionParam> params = {
            ParameterLifeCycleNode::ActionParam(
                rclcpp::Parameter("cloud_estimation_publish_topic", "bob/observer/cloud_estimation"), 
                [this](const rclcpp::Parameter& param) 
                {
                    pub_cloud_data_ = create_publisher<bob_interfaces::msg::ObserverCloudEstimation>(param.as_string(), pub_qos_profile_);
                }
            ),
            ParameterLifeCycleNode::ActionParam(
                rclcpp::Parameter("day_night_publish_topic", "bob/observer/day_night_classifier"), 
                [this](const rclcpp::Parameter& param) 
                {
                    pub_day_night_data_ = create_publisher<bob_interfaces::msg::ObserverDayNight>(param.as_string(), pub_qos_profile_);
                }
            ),
            ParameterLifeCycleNode::ActionParam(
                rclcpp::Parameter("camera_subscription_topic", "bob/observer_frame/source"), 
                [this](const rclcpp::Parameter& param) 
                {
                    sub_camera_ = create_subscription<sensor_msgs::msg::Image>(param.as_string(), sub_qos_profile_,
                                    [this](const sensor_msgs::msg::Image::SharedPtr img){camera_callback(img);});
                }
            ),
            ParameterLifeCycleNode::ActionParam(
                rclcpp::Parameter("observer_timer_interval", 30), 
                [this](const rclcpp::Parameter& param) 
                {
                    timer_interval_ = static_cast<int>(param.as_int());
                }
            ),
            ParameterLifeCycleNode::ActionParam(
                rclcpp::Parameter("observer_day_night_brightness_threshold", 95), 
                [this](const rclcpp::Parameter& param) 
                {
                    observer_day_night_brightness_threshold_ = static_cast<int>(param.as_int());
                }
            ),
            ParameterLifeCycleNode::ActionParam(
                rclcpp::Parameter("mask_timer_seconds", 5), 
                [this](const rclcpp::Parameter& param) 
                {
                    mask_timer_seconds_ = static_cast<int>(param.as_int());
                    mask_worker_ptr_->init(mask_timer_seconds_, mask_filename_);
                }
            ),
            ParameterLifeCycleNode::ActionParam(
                rclcpp::Parameter("mask_file", "mask.pgm"), 
                [this](const rclcpp::Parameter& param) 
                {
                    mask_filename_ = param.as_string();
                    mask_worker_ptr_->init(mask_timer_seconds_, mask_filename_);
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

        day_night_ = DayNightEnum::Unknown;
        mask_enabled_ = false;

        day_cloud_estimator_worker_ptr_ = std::make_unique<DayTimeCloudEstimator>(*this);
        night_cloud_estimator_worker_ptr_ = std::make_unique<NightTimeCloudEstimator>(*this);
        mask_worker_ptr_ = std::make_unique<MaskWorker>(*this, [this](MaskWorker::MaskCheckType detection_mask_result, const cv::Mat & mask){mask_timer_callback(detection_mask_result, mask);});

        declare_node_parameters();

        timer_ = create_wall_timer(std::chrono::seconds(timer_interval_), [this](){timer_callback();});
    }

    void mask_timer_callback(MaskWorker::MaskCheckType detection_mask_result, const cv::Mat & mask)
    {
        if (detection_mask_result == MaskWorker::MaskCheckType::Enable)
        {
            if (!mask_enabled_)
            {
                mask_enabled_ = true;
                RCLCPP_INFO(get_logger(), "Detection Mask Enabled.");
            }
            else
            {
                RCLCPP_INFO(get_logger(), "Detection Mask Changed.");
            }
            detection_mask_ = mask.clone();
        }
        else if ((detection_mask_result == MaskWorker::MaskCheckType::Disable) && mask_enabled_)
        {
            RCLCPP_INFO(get_logger(), "Detection Mask Disabled.");
            mask_enabled_ = false;
            detection_mask_.release();
        }
        day_cloud_estimator_worker_ptr_->set_mask(detection_mask_);
        night_cloud_estimator_worker_ptr_->set_mask(detection_mask_);
    }

    void camera_callback(const sensor_msgs::msg::Image::SharedPtr image_msg)
    {
        ImageUtils::convert_image_msg(image_msg, image_, true);
    }

    void timer_callback()
    {
        day_night_classifier();
        cloud_sampler();
    }

    void day_night_classifier()
    {
        auto [result, average_brightness] = DayNightClassifierWorker::estimate(image_, observer_day_night_brightness_threshold_);
        RCLCPP_INFO(get_logger(), "Day/Night classifier --> %d, %d", (int)result, average_brightness);
        day_night_ = result;

        auto day_night_msg = bob_interfaces::msg::ObserverDayNight();
        day_night_msg.day_night_enum = static_cast<int>(result);
        day_night_msg.avg_brightness = average_brightness;
        pub_day_night_data_->publish(day_night_msg);
    }

    void cloud_sampler()
    {
        if (image_.empty())
        {
            return;
        }

        try
        {
            double estimation = std::numeric_limits<double>::quiet_NaN();
            bool distribution;

            switch (day_night_)
            {
                case DayNightEnum::Day:
                {
                    std::tie(estimation, distribution) = day_cloud_estimator_worker_ptr_->estimate(image_);
                    RCLCPP_INFO(get_logger(), "Day time cloud estimation --> %f", estimation);
                }
                break;

                case DayNightEnum::Night:
                {
                    std::tie(estimation, distribution) = night_cloud_estimator_worker_ptr_->estimate(image_);
                    RCLCPP_INFO(get_logger(), "Night time cloud estimation --> %f", estimation);
                }
                break;

                default:
                    RCLCPP_INFO(get_logger(), "Unknown Day/Night classifier, ignore for now");
                    return;
            }

            if (!std::isnan(estimation))
            {
                bob_interfaces::msg::ObserverCloudEstimation cloud_estimation_msg;
                cloud_estimation_msg.percentage_cloud_cover = static_cast<float>(estimation);
                cloud_estimation_msg.unimodal_cloud_cover = distribution;
                pub_cloud_data_->publish(cloud_estimation_msg);
            }
        }
        catch (const std::exception & e)
        {
            RCLCPP_ERROR(get_logger(), "Exception during cloud estimation sampler. Error: %s", e.what());
        }
    }
};

RCLCPP_COMPONENTS_REGISTER_NODE(ImageEstimator)