#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "parameter_lifecycle_node.hpp"

#include <visibility_control.h>

#include "bob_interfaces/srv/recording_request.hpp"
#include "bob_interfaces/msg/monitoring_status.hpp"

class RecordingMonitor
    : public ParameterLifeCycleNode
{
public:
    COMPOSITION_PUBLIC
    explicit RecordingMonitor(const rclcpp::NodeOptions & options)
        : ParameterLifeCycleNode("recording_monitor_node", options)
        , pub_qos_profile_(10)
        , sub_qos_profile_(10)
    {
        init();
    }
    
private:
    void init()
    {
        sub_qos_profile_.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        sub_qos_profile_.durability(rclcpp::DurabilityPolicy::Volatile);
        sub_qos_profile_.history(rclcpp::HistoryPolicy::KeepLast);

        pub_qos_profile_.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        pub_qos_profile_.durability(rclcpp::DurabilityPolicy::Volatile);
        pub_qos_profile_.history(rclcpp::HistoryPolicy::KeepLast);

        declare_node_parameters();

        recording_enabled_ = true;

        interval_check_timer_ = create_wall_timer(std::chrono::seconds(check_interval_), 
            std::bind(&RecordingMonitor::interval_check_timer_callback, this));

        recording_change_client_ = create_client<bob_interfaces::srv::RecordingRequest>("bob/recording/update");

        monitoring_subscription_ = create_subscription<bob_interfaces::msg::MonitoringStatus>(
            "bob/monitoring/status", 
            sub_qos_profile_,
            std::bind(&RecordingMonitor::status_callback, this, std::placeholders::_1));
    }

    void declare_node_parameters()
    {
        std::vector<ParameterLifeCycleNode::ActionParam> params = {
            ParameterLifeCycleNode::ActionParam(
                rclcpp::Parameter("check_interval", 5), 
                [this](const rclcpp::Parameter& param) { check_interval_ = param.as_int(); }
            ),
        };
        add_action_parameters(params);
    }

    void status_callback(const bob_interfaces::msg::MonitoringStatus::SharedPtr status_msg)
    {
        status_msg_ = status_msg;
    }

    void interval_check_timer_callback()
    {
        if (status_msg_)
        {
            // Testing the service call
            /*auto recording_change_request = std::make_shared<bob_interfaces::srv::RecordingRequest::Request>();
            recording_change_request->disable_recording = recording_enabled_;
            if (recording_change_client_->service_is_ready())
            {
                auto result = recording_change_client_->async_send_request(recording_change_request, 
                    std::bind(&RecordingMonitor::recording_change_request_callback, 
                    this, 
                    std::placeholders::_1));
            }*/

            if (status_msg_->max_blobs_reached)
            {
                if (recording_enabled_)
                {
                    auto recording_change_request = std::make_shared<bob_interfaces::srv::RecordingRequest::Request>();
                    recording_change_request->disable_recording = true;
                    if (recording_change_client_->service_is_ready())
                    {
                        auto result = recording_change_client_->async_send_request(recording_change_request, 
                            std::bind(&RecordingMonitor::recording_change_request_callback, 
                            this, 
                            std::placeholders::_1));
                    }
                }    
            }
            else
            {
                if (!recording_enabled_)
                {
                    auto recording_change_request = std::make_shared<bob_interfaces::srv::RecordingRequest::Request>();
                    recording_change_request->disable_recording = false;
                    if (recording_change_client_->service_is_ready())
                    {
                        auto result = recording_change_client_->async_send_request(recording_change_request, 
                            std::bind(&RecordingMonitor::recording_change_request_callback, 
                            this, 
                            std::placeholders::_1));
                    }
                }
            }
        }
    }

    void recording_change_request_callback(rclcpp::Client<bob_interfaces::srv::RecordingRequest>::SharedFuture future)
    {
        auto response = future.get();
        if(response->success)
        {
            if (recording_enabled_)
            {
                log_info("Successfully disabled recording");
                recording_enabled_ = false;
            }
            else
            {
                log_info("Successfully enabled recording");
                recording_enabled_ = true;
            }
        }
        else
        {
            log_warn("Recording change failed");
        }
    }

    rclcpp::TimerBase::SharedPtr interval_check_timer_;
    rclcpp::Client<bob_interfaces::srv::RecordingRequest>::SharedPtr recording_change_client_;
    rclcpp::Subscription<bob_interfaces::msg::MonitoringStatus>::SharedPtr monitoring_subscription_;

    bool recording_enabled_;
    int check_interval_;
    bob_interfaces::msg::MonitoringStatus::SharedPtr status_msg_;

    rclcpp::QoS pub_qos_profile_;
    rclcpp::QoS sub_qos_profile_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(RecordingMonitor)