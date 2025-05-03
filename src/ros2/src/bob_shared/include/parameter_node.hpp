#pragma once

#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <bob_interfaces/msg/log_message.hpp>

class ParameterNode
    : public rclcpp::Node
{
public:
    struct ActionParam 
    {
        rclcpp::Parameter parameter;
        std::function<void(const rclcpp::Parameter&)> action;

        ActionParam(const rclcpp::Parameter & _param,
                    const std::function<void(const rclcpp::Parameter &)> & _action) 
            : parameter(_param)
            , action(_action) 
        {}
    };

    explicit ParameterNode(const std::string & node_name)
        : ParameterNode(node_name, *default_options())
    {
    }

    explicit ParameterNode(const std::string &node_name, const rclcpp::NodeOptions &options)
        : rclcpp::Node(node_name, options), timer_seconds_(5)
    {
        parameters_callback_handle_ = add_on_set_parameters_callback(std::bind(&ParameterNode::param_change_callback_method, this, std::placeholders::_1));
        rclcpp::QoS qos_profile{4};
        qos_profile.reliability(rclcpp::ReliabilityPolicy::Reliable);
        qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);
        qos_profile.history(rclcpp::HistoryPolicy::KeepLast);
        log_publisher_ = create_publisher<bob_interfaces::msg::LogMessage>("bob/log", qos_profile);

        timer_init_ = create_wall_timer(std::chrono::seconds(timer_seconds_), [this](){ init(); });
    }

    static std::string generate_uuid()
    {
        boost::uuids::random_generator uuid_generator;
        return boost::uuids::to_string(uuid_generator());
    }

    template <class T>
    inline void publish_if_subscriber(rclcpp::Publisher<T>::SharedPtr publisher, T message) const
    {
        if (!publisher || publisher->get_subscription_count() <= 0)
        {
            return;
        }
        publisher->publish(message);
    }

    template <typename... Args>
    void log_error(const std::string &format, Args... args) const
    {
        if (!rclcpp::ok())
        {
            return;
        }
        RCLCPP_ERROR(get_logger(), format.c_str(), args...);
    }

    template <typename... Args>
    void log_warn(const std::string &format, Args... args) const
    {
        RCLCPP_WARN(get_logger(), format.c_str(), args...);
    }

    template <typename... Args>
    void log_info(const std::string &format, Args... args) const
    {
        RCLCPP_INFO(get_logger(), format.c_str(), args...);
    }

    template <typename... Args>
    void log_debug(const std::string &format, Args... args) const
    {
        RCLCPP_DEBUG(get_logger(), format.c_str(), args...);
    }

    template <typename... Args>
    void log_send_error(const std::string &format, Args... args) const
    {
        if (!rclcpp::ok())
        {
            return;
        }
        RCLCPP_ERROR(get_logger(), format.c_str(), args...);
        send_log_message(rclcpp::Logger::Level::Error, format, args...);
    }

    template <typename... Args>
    void log_send_warn(const std::string &format, Args... args) const
    {
        RCLCPP_WARN(get_logger(), format.c_str(), args...);
        send_log_message(rclcpp::Logger::Level::Warn, format, args...);
    }

    template <typename... Args>
    void log_send_info(const std::string &format, Args... args) const
    {
        RCLCPP_INFO(get_logger(), format.c_str(), args...);
        send_log_message(rclcpp::Logger::Level::Info, format, args...);
    }

    template <typename... Args>
    void log_send_debug(const std::string &format, Args... args) const
    {
        RCLCPP_DEBUG(get_logger(), format.c_str(), args...);
        send_log_message(rclcpp::Logger::Level::Debug, format, args...);
    }

    template <typename... Args>
    void send_log_message(rclcpp::Logger::Level severity, const std::string &str_format, Args... args) const
    {
        if (severity >= get_logger().get_effective_level())
        {
            static uint64_t log_index_ = 0;
            bob_interfaces::msg::LogMessage log_msg;
            log_msg.header.frame_id = std::to_string(++log_index_);
            log_msg.header.stamp = now();
            log_msg.node = get_name();
            log_msg.severity = g_rcutils_log_severity_names[(int)severity];
            log_msg.message = format_c(str_format, args...);
            log_publisher_->publish(log_msg);
        }
    }

protected:
    void add_action_parameters(const std::vector<ActionParam> & action_params)
    {
        for (auto & action_param : action_params)
        {
            parameters_map_.insert_or_assign(action_param.parameter.get_name(), action_param);
            declare_parameter(action_param.parameter.get_name(), action_param.parameter.get_parameter_value());
        }
    }

    void update_action_param(const rclcpp::Parameter &_param)
    {
        try
        {
            const auto & it = parameters_map_.find(_param.get_name());
            if (it != parameters_map_.end() && it->second.action != nullptr)
            {
                it->second.action(_param);
            }
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(get_logger(), e.what());
        }
    }

    virtual void on_configure() = 0;

private:

    void init()
    {
        timer_init_.reset();

        on_configure();
    }

    rcl_interfaces::msg::SetParametersResult param_change_callback_method(const std::vector<rclcpp::Parameter> & parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (auto & param : parameters)
        {
            //RCLCPP_INFO(get_logger(), "Updating parameter: %s", param.get_name().c_str());
            update_action_param(param);
        }
        return result;
    }

    static std::shared_ptr<rclcpp::NodeOptions> default_options()
    {
        auto options = std::make_shared<rclcpp::NodeOptions>();
        options->use_intra_process_comms(true);

        return options;
    }

    template <typename... Args>
    static std::string format_c(const std::string &format, Args... args)
    {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-security"
        size_t size = std::snprintf(nullptr, 0, format.c_str(), args...) + 1; // Extra space for '\0'
        if (size <= 0)
        {
            throw std::runtime_error("Error during formatting.");
        }

        std::string buf(size, '\0');
        std::snprintf(&buf[0], size, format.c_str(), args...);
#pragma GCC diagnostic pop
        return buf;
    }

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameters_callback_handle_;
    std::map<std::string, ActionParam> parameters_map_;
    rclcpp::Publisher<bob_interfaces::msg::LogMessage>::SharedPtr log_publisher_;
    rclcpp::TimerBase::SharedPtr timer_init_;
    int timer_seconds_;
};
