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
    enum SensitivityChangeActionEnum
    {
        Ignore,
        IncreaseSensitivity,
        LowerSensitivity
    };

    rclcpp::TimerBase::SharedPtr interval_check_timer_;
    rclcpp::Client<bob_interfaces::srv::SensitivityChangeRequest>::SharedPtr sensitivity_change_client_;
    rclcpp::Subscription<bob_interfaces::msg::MonitoringStatus>::SharedPtr monitoring_subscription_;

    bob_interfaces::msg::MonitoringStatus::SharedPtr status_msg_;

    std::string sensitivity_;
    int check_interval_;
    SensitivityChangeActionEnum sensitivity_change_action_;
    bool star_mask_enabled_;

    int sensitivity_increase_count_threshold_;
    int sensitivity_increase_check_counter_;
    std::string change_reason_;

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

        sensitivity_change_action_ = Ignore;

        interval_check_timer_ = create_wall_timer(std::chrono::seconds(check_interval_), 
            std::bind(&TrackSensitivityMonitor::interval_check_timer_callback, this));

        sensitivity_change_client_ = create_client<bob_interfaces::srv::SensitivityChangeRequest>("bob/bgs/sensitivity_update");

        monitoring_subscription_ = create_subscription<bob_interfaces::msg::MonitoringStatus>(
            "bob/monitoring/status", 
            sub_qos_profile_,
            std::bind(&TrackSensitivityMonitor::status_callback, this, std::placeholders::_1));
    }

    void declare_node_parameters()
    {
        std::vector<ParameterNode::ActionParam> params = {
            ParameterNode::ActionParam(
                rclcpp::Parameter("sensitivity", "medium_c"), 
                [this](const rclcpp::Parameter& param) { sensitivity_ = param.as_string(); }
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("check_interval", 30), 
                [this](const rclcpp::Parameter& param) { check_interval_ = param.as_int(); }
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("sensitivity_increase_count_threshold", 5), 
                [this](const rclcpp::Parameter& param) 
                { 
                    sensitivity_increase_count_threshold_ = param.as_int();
                    sensitivity_increase_check_counter_ = 0;
                }
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("star_mask_enabled", true), 
                [this](const rclcpp::Parameter& param) { star_mask_enabled_ = param.as_bool(); }
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
        // Current Auto Tuning Rules
        // --------------------------------
        // 1. Wait for a day/night determination before we do anything
        // 2. Lower sensitivity immediately if max blobs
        // 3. Increase sensitivity past medium only during the day, if set to high and its night, lower to medium. 
        //    If we have a star mask enabled then night sensitivity can be set to high or high_c as well.
        // 4. Lower sensitivity immediately if we have bimodal cloud cover and cloud cover is between 10% and 90%
        // 5. Increase sensitivity after a number of iterations, currently set to 5
        // ----------------------------------------------------------------
        // Look into a rules engine e.g.: https://www.clipsrules.net/ 
        // ----------------------------------------------------------------

        if (status_msg_)
        {
            change_reason_ = "";            

            // Rule 1
            if (status_msg_->day_night_enum > 0)
            {
                // Rule 2
                if (status_msg_->max_blobs_reached)
                {
                    change_reason_ = "Max Blobs";
                    sensitivity_change_action_ = LowerSensitivity;
                }
                // Rule 3
                else if (!star_mask_enabled_ && status_msg_->day_night_enum == 2 && (sensitivity_ == "high" || sensitivity_ == "high_c"))
                {
                    change_reason_ = "Day 2 Night";
                    sensitivity_change_action_ = LowerSensitivity;
                }
                else
                {
                    if (status_msg_->unimodal_cloud_cover)
                    {
                        change_reason_ = "Unimodal";
                        sensitivity_change_action_ = IncreaseSensitivity;
                    }
                    // Rule 4
                    else if (!status_msg_->unimodal_cloud_cover)
                    {
                        if (status_msg_->percentage_cloud_cover > 10 && status_msg_->percentage_cloud_cover < 90)
                        {
                            change_reason_ = "Bimodal";
                            sensitivity_change_action_ = LowerSensitivity;
                        }
                    }
                    else
                        sensitivity_change_action_ = Ignore;
                }

                // Rule 5
                if (sensitivity_change_action_ == IncreaseSensitivity)
                {
                    sensitivity_increase_check_counter_++;                        
                    if(sensitivity_increase_check_counter_ < sensitivity_increase_count_threshold_)
                    {
                        sensitivity_change_action_ = Ignore;

                        RCLCPP_DEBUG(get_logger(), "Tracking Auto Tune: IncreaseSensitivity triggered, counter %d of %d", 
                            sensitivity_increase_check_counter_, sensitivity_increase_count_threshold_);
                    }
                }

                bool updating = false;
                switch (sensitivity_change_action_)
                {                
                    case IncreaseSensitivity:

                        // reset the counter
                        sensitivity_increase_check_counter_ = 0;                       

                        if (sensitivity_ == "low")
                        {
                            sensitivity_ = "medium";
                            updating = true;
                        }
                        else if (sensitivity_ == "medium")
                        {        
                            // Rule 3
                            if (status_msg_->day_night_enum == 1 || (star_mask_enabled_ && status_msg_->day_night_enum == 2))
                            {
                                sensitivity_ = "high";
                                updating = true;
                            }
                        }
                        else if (sensitivity_ == "low_c")
                        {
                            sensitivity_ = "medium_c";
                            updating = true;
                        }
                        else if (sensitivity_ == "medium_c")
                        {
                            // Rule 3
                            if (status_msg_->day_night_enum == 1 || (star_mask_enabled_ && status_msg_->day_night_enum == 2))
                            {
                                sensitivity_ = "high_c";
                                updating = true;
                            }
                        }

                        if (updating)
                            RCLCPP_DEBUG(get_logger(), "Tracking Auto Tune: Stable conditions - increasing sensitivity");

                        break;

                    case LowerSensitivity:

                        // reset the counter
                        sensitivity_increase_check_counter_ = 0;

                        if (sensitivity_ == "medium")
                        {
                            sensitivity_ = "low";
                            updating = true;
                        }
                        else if (sensitivity_ == "high")
                        {
                            sensitivity_ = "medium";
                            updating = true;
                        }
                        else if (sensitivity_ == "medium_c")
                        {
                            sensitivity_ = "low_c";
                            updating = true;
                        }
                        else if (sensitivity_ == "high_c")
                        {
                            sensitivity_ = "medium_c";
                            updating = true;
                        }     

                        if (updating)
                            RCLCPP_DEBUG(get_logger(), "Tracking Auto Tune: Unstable conditions - lowering sensitivity");

                        break;

                    case Ignore:
                        //RCLCPP_INFO(get_logger(), "Tracking Action: Ignore");
                        break;
                }

                if (updating)
                {
                    auto sensitivity_change_request = std::make_shared<bob_interfaces::srv::SensitivityChangeRequest::Request>();
                    sensitivity_change_request->sensitivity = sensitivity_;
                    if (sensitivity_change_client_->service_is_ready())
                    {
                        auto result = sensitivity_change_client_->async_send_request(sensitivity_change_request, 
                            std::bind(&TrackSensitivityMonitor::sensitivity_change_request_callback, 
                            this, 
                            std::placeholders::_1));
                    }
                }
            }
        }
    }

    void sensitivity_change_request_callback(rclcpp::Client<bob_interfaces::srv::SensitivityChangeRequest>::SharedFuture future)
    {
        auto response = future.get();
        if(response->success)
        {            
            RCLCPP_INFO(get_logger(), "Sensitivity change completed, from %s to %s - reason: %s", 
                response->sensitivity_from.c_str(), response->sensitivity_to.c_str(), change_reason_.c_str());
            
            // reset the counter
            sensitivity_increase_check_counter_ = 0;
            sensitivity_change_action_ = Ignore;
        }
        else
            RCLCPP_WARN(get_logger(), "Sensitivity change failed");
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