#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp/experimental/executors/events_executor/events_executor.hpp>

#include "parameter_node.hpp"

#include <visibility_control.h>

#include "bob_interfaces/srv/sensitivity_change_request.hpp"

#include "bob_interfaces/msg/detector_state.hpp"

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

    rclcpp::Publisher<bob_interfaces::msg::DetectorState>::SharedPtr state_publisher_;
    rclcpp::Service<bob_interfaces::srv::SensitivityChangeRequest>::SharedPtr sensitivity_change_service_;

    std::string sensitivity_;

    // MWG Test Stuff
    rclcpp::TimerBase::SharedPtr sensitivity_test_timer_;
    rclcpp::Client<bob_interfaces::srv::SensitivityChangeRequest>::SharedPtr sensitivity_change_client_;

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

        sensitivity_test_timer_ = create_wall_timer(std::chrono::seconds(10), std::bind(&TrackSensitivityMonitor::sensitivity_test_timer_callback, this));
        sensitivity_change_client_ = create_client<bob_interfaces::srv::SensitivityChangeRequest>("bob/bgs/sensitivity_update");
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

    void sensitivity_test_timer_callback()
    {
        if (sensitivity_ == "low")
            sensitivity_ = "medium";
        else if (sensitivity_ == "medium")
            sensitivity_ = "high";
        else if (sensitivity_ == "high")
            sensitivity_ = "low_c";
        else if (sensitivity_ == "low_c")
            sensitivity_ = "medium_c";
        else if (sensitivity_ == "medium_c")
            sensitivity_ = "high_c";
        else if (sensitivity_ == "high_c")
            sensitivity_ = "low";

        auto sensitivity_change_request = std::make_shared<bob_interfaces::srv::SensitivityChangeRequest::Request>();
        sensitivity_change_request->sensitivity = sensitivity_;
        if (sensitivity_change_client_->service_is_ready())
        {
            auto result = sensitivity_change_client_->async_send_request(sensitivity_change_request, std::bind(&TrackSensitivityMonitor::sensitivity_change_request_callback, this, std::placeholders::_1));
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