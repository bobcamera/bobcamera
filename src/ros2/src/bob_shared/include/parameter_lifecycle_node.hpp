#pragma once
#ifndef __PARAMETER_LIFECYCLE_NODE_H__
#define __PARAMETER_LIFECYCLE_NODE_H__

#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp_lifecycle/transition.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

class ParameterLifeCycleNode
    : public rclcpp_lifecycle::LifecycleNode
{
public:
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

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

    explicit ParameterLifeCycleNode(const std::string & node_name)
        : ParameterLifeCycleNode(node_name, *default_options())
    {
    }

    explicit ParameterLifeCycleNode(const std::string &node_name, const rclcpp::NodeOptions & options)
        : rclcpp_lifecycle::LifecycleNode(node_name, options)
    {
        parameters_callback_handle_ = add_on_set_parameters_callback(std::bind(&ParameterLifeCycleNode::param_change_callback_method, this, std::placeholders::_1));
        // register_on_configure(std::bind(&ParameterNode::on_configure, this, std::placeholders::_1));
        // register_on_activate(std::bind(&ParameterNode::on_activate, this, std::placeholders::_1));
        // register_on_deactivate(std::bind(&ParameterNode::on_deactivate, this, std::placeholders::_1));
        // register_on_cleanup(std::bind(&ParameterNode::on_cleanup, this, std::placeholders::_1));
        // register_on_shutdown(std::bind(&ParameterNode::on_shutdown, this, std::placeholders::_1));
        // register_on_error(std::bind(&ParameterNode::on_error, this, std::placeholders::_1));
        // register_on_configure([this](const rclcpp_lifecycle::State & state){return on_configure(state);});

        //this->trigger_transition(rclcpp_lifecycle::Transition(1));
        RCLCPP_INFO(get_logger(), "########## ParameterNode constructor");
    }

    static std::string generate_uuid()
    {
        boost::uuids::random_generator uuid_generator;
        return boost::uuids::to_string(uuid_generator());
    }

    template <class T>
    inline void publish_if_subscriber(rclcpp::Publisher<T>::SharedPtr publisher, T message) const
    {
        if (!publisher || (count_subscribers(publisher->get_topic_name()) <= 0))
        {
            return;
        }
        publisher->publish(message);
    }

    CallbackReturn on_configure(const rclcpp_lifecycle::State &) 
    {
        RCLCPP_INFO(get_logger(), "$$$$$$$$$$$$$$$$$$$$$$$$ Configuring");
        sleep(3);

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State &) 
    {
        RCLCPP_INFO(get_logger(), "Activating");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) 
    {
        RCLCPP_INFO(get_logger(), "Deactivating");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) 
    {
        RCLCPP_INFO(get_logger(), "Cleaning up");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) 
    {
        RCLCPP_INFO(get_logger(), "Shutting down");
        // Shutdown node
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_error(const rclcpp_lifecycle::State &) 
    {
        RCLCPP_ERROR(get_logger(), "Error handling");
        // Handle error
        return CallbackReturn::SUCCESS;
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

private:
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameters_callback_handle_;
    std::map<std::string, ActionParam> parameters_map_;

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
};

#endif // __PARAMETER_LIFECYCLE_NODE_H__