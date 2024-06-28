#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <sensor_msgs/msg/image.hpp>

#include <bob_interfaces/msg/observer_cloud_estimation.hpp>
#include <bob_interfaces/msg/observer_day_night.hpp>

#include <parameter_lifecycle_node.hpp>
#include <image_utils.hpp>
#include "cloud_estimator_worker.hpp"

#include <visibility_control.h>

class CloudEstimator
    : public ParameterLifeCycleNode
{
public:
    COMPOSITION_PUBLIC
    explicit CloudEstimator(const rclcpp::NodeOptions & options) 
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
    CloudEstimatorWorker::DayNightEnum day_night_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<bob_interfaces::msg::ObserverCloudEstimation>::SharedPtr pub_environment_data_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_camera_;
    rclcpp::Subscription<bob_interfaces::msg::ObserverDayNight>::SharedPtr sub_environment_day_night_;

    void declare_node_parameters()
    {
        std::vector<ParameterLifeCycleNode::ActionParam> params = {
            ParameterLifeCycleNode::ActionParam(
                rclcpp::Parameter("cloud_estimation_publish_topic", "bob/observer/cloud_estimation"), 
                [this](const rclcpp::Parameter& param) 
                {
                    //pub_environment_data_ = create_publisher<bob_interfaces::msg::ObserverCloudEstimation>(param.as_string(), pub_qos_profile_);
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
                rclcpp::Parameter("day_night_subscription_topic", "bob/observer/day_night_classifier"), 
                [this](const rclcpp::Parameter& param) 
                {
                    sub_environment_day_night_ = create_subscription<bob_interfaces::msg::ObserverDayNight>(param.as_string(), sub_qos_profile_,
                                   [this](const bob_interfaces::msg::ObserverDayNight::SharedPtr odn){day_night_callback(odn);});
                }
            ),
            ParameterLifeCycleNode::ActionParam(
                rclcpp::Parameter("observer_timer_interval", 30), 
                [this](const rclcpp::Parameter& param) 
                {
                    timer_interval_ = static_cast<int>(param.as_int());
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

        day_night_ = CloudEstimatorWorker::DayNightEnum::Unknown;
        //day_night_ = CloudEstimatorWorker::DayNightEnum::Day;

        day_cloud_estimator_worker_ptr_ = std::make_unique<DayTimeCloudEstimator>(*this);
        night_cloud_estimator_worker_ptr_ = std::make_unique<NightTimeCloudEstimator>(*this);

        declare_node_parameters();

        timer_ = create_wall_timer(std::chrono::seconds(timer_interval_), [this](){cloud_sampler();});
    }

    void camera_callback(const sensor_msgs::msg::Image::SharedPtr image_msg)
    {
        ImageUtils::convert_image_msg(image_msg, image_, true);
    }

    void day_night_callback(const bob_interfaces::msg::ObserverDayNight::SharedPtr odn)
    {
        day_night_ = CloudEstimatorWorker::DayNightEnum(odn->day_night_enum);
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
                case CloudEstimatorWorker::DayNightEnum::Day:
                {
                    std::tie(estimation, distribution) = day_cloud_estimator_worker_ptr_->estimate(image_);
                    RCLCPP_INFO(get_logger(), "Day time cloud estimation --> %f", estimation);
                }
                break;

                case CloudEstimatorWorker::DayNightEnum::Night:
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
                //pub_environment_data_->publish(cloud_estimation_msg);
            }
        }
        catch (const std::exception & e)
        {
            RCLCPP_ERROR(get_logger(), "Exception during cloud estimation sampler. Error: %s", e.what());
        }
    }
};

RCLCPP_COMPONENTS_REGISTER_NODE(CloudEstimator)