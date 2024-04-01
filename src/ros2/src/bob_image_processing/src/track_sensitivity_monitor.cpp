#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp/experimental/executors/events_executor/events_executor.hpp>

#include "parameter_node.hpp"

#include <visibility_control.h>

#include "bob_interfaces/srv/sensitivity_change_request.hpp"

#include "bob_interfaces/msg/monitoring_status.hpp"

class TrackSensitivityMonitor
    : public ParameterNode
{
public:
    COMPOSITION_PUBLIC
    explicit TrackSensitivityMonitor(const rclcpp::NodeOptions & options)
        : ParameterNode("track_sensitivity_monitor_node", options)
        , pub_qos_profile_(10)
        , sub_qos_profile_(10)
    {
        init();
    }
    
private:
    enum TrackingHintEnum
    {
        NoHint,
        IncreaseSensitivity,
        LowerSensitivity
    };

    rclcpp::Client<bob_interfaces::srv::SensitivityChangeRequest>::SharedPtr sensitivity_change_client_;
    rclcpp::Subscription<bob_interfaces::msg::MonitoringStatus>::SharedPtr monitoring_subscription_;

    std::string sensitivity_;
    TrackingHintEnum tracking_hint_;

    rclcpp::QoS pub_qos_profile_;
    rclcpp::QoS sub_qos_profile_;

    void init()
    {
        sub_qos_profile_.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        sub_qos_profile_.durability(rclcpp::DurabilityPolicy::Volatile);
        sub_qos_profile_.history(rclcpp::HistoryPolicy::KeepLast);

        pub_qos_profile_.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        pub_qos_profile_.durability(rclcpp::DurabilityPolicy::Volatile);
        pub_qos_profile_.history(rclcpp::HistoryPolicy::KeepLast);

        declare_node_parameters();
        tracking_hint_ = NoHint;

        sensitivity_change_client_ = create_client<bob_interfaces::srv::SensitivityChangeRequest>("bob/bgs/sensitivity_update");

        monitoring_subscription_ = create_subscription<bob_interfaces::msg::MonitoringStatus>(
            "bob/monitoring/status", 
            sub_qos_profile_,
            std::bind(&TrackSensitivityMonitor::statusCallback, this, std::placeholders::_1));
    }

    void declare_node_parameters()
    {
        std::vector<ParameterNode::ActionParam> params = {
            ParameterNode::ActionParam(
                rclcpp::Parameter("sensitivity", "medium_c"), 
                [this](const rclcpp::Parameter& param) 
                {
                    RCLCPP_INFO(get_logger(), "Setting Sensitivity: %s", param.as_string().c_str());
                    sensitivity_ = param.as_string();
                }
            ),
        };
        add_action_parameters(params);
    }

    void statusCallback(const bob_interfaces::msg::MonitoringStatus::SharedPtr status_msg)
    {
        if (status_msg->max_blobs_reached)
        {
            tracking_hint_ = LowerSensitivity;
        }
        else
        {
            if (status_msg->unimodal_cloud_cover)
            {
                tracking_hint_ = IncreaseSensitivity;
            }
            else if (!status_msg->unimodal_cloud_cover)
            {
                if (status_msg->percentage_cloud_cover > 10 && status_msg->percentage_cloud_cover < 90)
                {
                    tracking_hint_ = LowerSensitivity;
                }
            }
            else
                tracking_hint_ = NoHint;
        }

        std::string sensitivity = sensitivity_;

        // MWG: This needs way more work
        switch (tracking_hint_)
        {
            case IncreaseSensitivity:

                if (sensitivity_ == "low")
                    sensitivity_ = "medium";
                else if (sensitivity_ == "medium")
                    sensitivity_ = "high";
                else if (sensitivity_ == "low_c")
                    sensitivity_ = "medium_c";
                else if (sensitivity_ == "medium_c")
                    sensitivity_ = "high_c";

                RCLCPP_INFO(get_logger(), "Tracking Hint: Stable conditions - set to medium or high sensitivity");
                break;
            case LowerSensitivity:

                if (sensitivity_ == "medium")
                    sensitivity_ = "low";
                else if (sensitivity_ == "high")
                    sensitivity_ = "medium";
                else if (sensitivity_ == "medium_c")
                    sensitivity_ = "low_c";
                else if (sensitivity_ == "high_c")
                    sensitivity_ = "medium_c";

                RCLCPP_INFO(get_logger(), "Tracking Hint: Unstable conditions - set to low sensitivity");
                break;
            case NoHint:

                RCLCPP_INFO(get_logger(), "Tracking Hint: None");
                break;                
        }

        if (sensitivity != sensitivity_)
        {
            auto sensitivity_change_request = std::make_shared<bob_interfaces::srv::SensitivityChangeRequest::Request>();
            sensitivity_change_request->sensitivity = sensitivity_;
            if (sensitivity_change_client_->service_is_ready())
            {
                auto result = sensitivity_change_client_->async_send_request(sensitivity_change_request, std::bind(&TrackSensitivityMonitor::sensitivity_change_request_callback, this, std::placeholders::_1));
            }    
        }
    }

    void sensitivity_change_request_callback(rclcpp::Client<bob_interfaces::srv::SensitivityChangeRequest>::SharedFuture future)
    {
        auto response = future.get();
        if(response->success)
            RCLCPP_INFO(get_logger(), "Sensitivity change completed, from %s to %s", response->sensitivity_from.c_str(), response->sensitivity_to.c_str());
        else
            RCLCPP_INFO(get_logger(), "Sensitivity change failed");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::experimental::executors::EventsExecutor executor;
    executor.add_node(std::make_shared<TrackSensitivityMonitor>(rclcpp::NodeOptions()));
    executor.spin();
    rclcpp::shutdown();
    return 0;
}

RCLCPP_COMPONENTS_REGISTER_NODE(TrackSensitivityMonitor)