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
        , pub_qos_profile_(4)
        , sub_qos_profile_(4)
    {
    }

    void on_configure()
    {
        log_info("Configuring");

        init();
    }

private:
    enum SensitivityChangeActionEnum
    {
        Ignore,
        IncreaseSensitivity,
        LowerSensitivity
    };

    void init()
    {
        sub_qos_profile_.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        sub_qos_profile_.durability(rclcpp::DurabilityPolicy::Volatile);
        sub_qos_profile_.history(rclcpp::HistoryPolicy::KeepLast);

        pub_qos_profile_.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        pub_qos_profile_.durability(rclcpp::DurabilityPolicy::Volatile);
        pub_qos_profile_.history(rclcpp::HistoryPolicy::KeepLast);

        sensitivity_change_action_ = Ignore;

        declare_node_parameters();

        interval_check_timer_ = create_wall_timer(std::chrono::seconds(check_interval_),
            [this](){interval_check_timer_callback();});
    }

    void declare_node_parameters()
    {
        std::vector<ParameterNode::ActionParam> params = {
            ParameterNode::ActionParam(
                rclcpp::Parameter("monitoring_subscription_topic", "bob/monitoring/status"),
                [this](const rclcpp::Parameter& param)
                {
                    monitoring_subscription_ = create_subscription<bob_interfaces::msg::MonitoringStatus>(param.as_string(), sub_qos_profile_,
                            [this](const bob_interfaces::msg::MonitoringStatus::SharedPtr status_msg){status_callback(status_msg);});
                }
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("bgs_node", "rtsp_camera_node"),
                [this](const rclcpp::Parameter& param)
                {
                    sensitivity_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, param.as_string());
                }
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("sensitivity", "medium_c"),
                [this](const rclcpp::Parameter& param)
                {
                    sensitivity_ = param.as_string();
                    proposed_sensitivity_ = param.as_string();
                }
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

                        log_send_debug("Tracking Auto Tune: IncreaseSensitivity triggered, counter %d of %d",
                            sensitivity_increase_check_counter_, sensitivity_increase_count_threshold_);
                    }
                }

                apply_sensitivity_change();
            }
        }
    }

    void apply_sensitivity_change()
    {
        static const std::map<std::string, std::pair<std::string, std::string>> sensitivity_change_map =
        {
            {"low", {"low", "medium"}},
            {"medium", {"low", "high"}},
            {"high", {"medium", "high"}},
            {"low_c", {"low_c", "medium_c"}},
            {"medium_c", {"low_c", "high_c"}},
            {"high_c", {"medium_c", "high_c"}},
        };

        const auto &[lower_sensitivity, higher_sensitivity] = sensitivity_change_map.at(sensitivity_);

        switch (sensitivity_change_action_)
        {
            case IncreaseSensitivity:
                sensitivity_increase_check_counter_ = 0;
                if ((sensitivity_ == "medium") || (sensitivity_ == "medium_c")) 
                {
                    if (status_msg_->day_night_enum == 1 || (star_mask_enabled_ && status_msg_->day_night_enum == 2))
                    {
                        proposed_sensitivity_ = higher_sensitivity;
                    }
                }
                else
                {
                    proposed_sensitivity_ = higher_sensitivity;
                }

                if (sensitivity_ != proposed_sensitivity_)
                {
                    log_send_info("Tracking Auto Tune: Stable conditions - increasing sensitivity");
                }
                break;

            case LowerSensitivity:
                sensitivity_increase_check_counter_ = 0;
                proposed_sensitivity_ = lower_sensitivity;
                if (sensitivity_ != proposed_sensitivity_)
                {
                    log_send_info("Tracking Auto Tune: Unstable conditions - lowering sensitivity");
                }
                break;

            case Ignore:
                break;
        }

        if (sensitivity_ != proposed_sensitivity_)
        {
            change_parameter_async("bgs_sensitivity", proposed_sensitivity_);
        }
    }

    void change_parameter_async(const std::string & param_name, const std::string & param_value)
    {
        // TODO: Move this into the base class so all param change requests can use the same code
        if (!sensitivity_param_client_->wait_for_service(std::chrono::seconds(1)))
        {
            log_warn("Sensitivity parameter service not ready.");
            return;
        }

        auto parameters = std::vector<rclcpp::Parameter>{rclcpp::Parameter(param_name, param_value)};
        auto result_future = sensitivity_param_client_->set_parameters(parameters,
            std::bind(&TrackSensitivityMonitor::async_sensitivity_change_request_callback, this, std::placeholders::_1));
    }

    void async_sensitivity_change_request_callback(std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future)
    {
        bool param_result = true;
        for (const auto& result : future.get())
        {
            if (!result.successful)
            {
                param_result = false;
                break;
            }
        }
        if (param_result)
        {
            log_info("Sensitivity change from: %s to %s completed - reason: %s",
                sensitivity_.c_str(), proposed_sensitivity_.c_str(), change_reason_.c_str());

            // reset sensitivity tracking state
            sensitivity_increase_check_counter_ = 0;
            sensitivity_change_action_ = Ignore;
            sensitivity_ = proposed_sensitivity_;
        }
        else
        {
            log_warn("Sensitivity change failed");
        }
    }

    rclcpp::TimerBase::SharedPtr interval_check_timer_;
    rclcpp::Subscription<bob_interfaces::msg::MonitoringStatus>::SharedPtr monitoring_subscription_;

    rclcpp::AsyncParametersClient::SharedPtr sensitivity_param_client_;

    bob_interfaces::msg::MonitoringStatus::SharedPtr status_msg_;

    std::string sensitivity_;
    std::string proposed_sensitivity_;
    int check_interval_;
    SensitivityChangeActionEnum sensitivity_change_action_;
    bool star_mask_enabled_;

    int sensitivity_increase_count_threshold_;
    int sensitivity_increase_check_counter_;
    std::string change_reason_;

    rclcpp::QoS pub_qos_profile_;
    rclcpp::QoS sub_qos_profile_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(TrackSensitivityMonitor)