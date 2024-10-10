#include <chrono>
// #include <future>
// #include <memory>
#include <string>
#include <thread>

#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <visibility_control.h>

#include <boblib/api/base/Utils.hpp>

#include <parameter_node.hpp>

class LifecycleManager 
    : public ParameterNode
{
public:
    COMPOSITION_PUBLIC
    explicit LifecycleManager(const rclcpp::NodeOptions & options)
        : ParameterNode("lifecycle_service_client_node", options)
        , timer_seconds_(5)
    {
        timer_nodes_ = create_wall_timer(std::chrono::seconds(timer_seconds_), [this](){init();});
    }

    void init()
    {
        timer_nodes_->cancel();
        log_common_information();

        declare_node_parameters();
        timer_nodes_ = create_wall_timer(std::chrono::seconds(timer_seconds_), [this](){timer_callback();});
    }

private:
    void log_common_information()
    {
        RCLCPP_INFO(get_logger(), "LifecycleManager Starting, will start nodes in %d seconds", timer_seconds_);
        RCLCPP_INFO(get_logger(), "Basic Information:");
        RCLCPP_INFO(get_logger(), "              CPU: %s", boblib::base::Utils::get_cpu_name().c_str());
        RCLCPP_INFO(get_logger(), "Available Threads: %ld", boblib::base::Utils::get_available_threads());
        RCLCPP_INFO(get_logger(), "     Total Memory: %ld GB", boblib::base::Utils::get_memory_total() / (1024 * 1024 * 1024));
        RCLCPP_INFO(get_logger(), "         Has Cuda: %s", boblib::base::Utils::has_cuda() ? "True" : "False");
        auto has_opencl = boblib::base::Utils::has_opencl();
        RCLCPP_INFO(get_logger(), "       Has OpenCL: %s", has_opencl ? "True" : "False");
        if (has_opencl)
        {
            RCLCPP_INFO(get_logger(), "%s", boblib::base::Utils::get_ocl_info().c_str());
        }
    }

    class NodeStatusManager
    {
    public:
        NodeStatusManager(const std::string & name, LifecycleManager::SharedPtr base_node, int prune_time_seconds)
            : name_(name)
        {
            node_ = base_node->create_sub_node(name + "_manager");
            prune_time_seconds_ = prune_time_seconds;
            state_.id = lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
            state_.label = "unknown";
            client_change_state_ = node_->create_client<lifecycle_msgs::srv::ChangeState>("/" + name + "/change_state");
            client_get_state_ = node_->create_client<lifecycle_msgs::srv::GetState>("/" + name + "/get_state");
            get_request_ = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
            change_request_ = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
        }

        std::string get_name() const
        {
            return name_;
        }

        lifecycle_msgs::msg::State get_state() const
        {
            return state_;
        }

        void update_state(std::chrono::seconds time_out = std::chrono::seconds(5))
        {
            if (!client_get_state_->wait_for_service(time_out)) 
            {
                RCLCPP_ERROR(node_->get_logger(), "get_state(%s): Service %s is not available.", name_.c_str(), client_get_state_->get_service_name());
                
                state_.id = lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
                state_.label = "unknown";
            }

            auto futureAndRequestId = client_get_state_->async_send_request(get_request_, [this](rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedFuture future) 
            {
                auto response = future.get();
                state_.id = response->current_state.id;
                state_.label = response->current_state.label;
                RCLCPP_DEBUG(node_->get_logger(), "get_state(%s) = %d:%s", name_.c_str(), static_cast<int>(state_.id), state_.label.c_str());
                check_change_status();
            });
        }

        void change_state(std::uint8_t transition, std::chrono::seconds time_out = std::chrono::seconds(5))
        {
            change_request_->transition.id = transition;

            if (!client_change_state_->wait_for_service(time_out)) 
            {
                RCLCPP_ERROR(node_->get_logger(),"Service %s is not available.", client_change_state_->get_service_name());
                return;
            }

            auto future_result = client_change_state_->async_send_request(change_request_, [this, transition](rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedFuture future) 
            {
                auto response = future.get();
                if (response->success)
                {
                    RCLCPP_DEBUG(node_->get_logger(), "change_state(%s) -> Transition %d successfully triggered.", name_.c_str(), static_cast<int>(transition));
                    update_state();
                }
                else
                {
                    RCLCPP_DEBUG(node_->get_logger(), "change_state(%s) -> Transition NOT triggered.", name_.c_str());
                }
            });
        }

        void check_change_status()
        {
            // TODO: implement a state machine, right now we only send a configure state
            switch (state_.id)
            {
                case lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED:
                    change_state(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
                    break;

                default:
                    break;
            }
        }

        void prune_requests()
        {
            std::vector<int64_t> pruned_requests;
            size_t n_pruned_change = client_change_state_->prune_requests_older_than(std::chrono::system_clock::now() - std::chrono::seconds(prune_time_seconds_), &pruned_requests);
            size_t n_pruned_get = client_change_state_->prune_requests_older_than(std::chrono::system_clock::now() - std::chrono::seconds(prune_time_seconds_), &pruned_requests);
            if (n_pruned_change || n_pruned_get) 
            {
                RCLCPP_INFO(node_->get_logger(), "The node '%s' didn't reply for more than %ds, %zu requests were discarded", 
                    name_.c_str(), prune_time_seconds_, n_pruned_change + n_pruned_get);
            }
        }

    private:
        const std::string name_;
        lifecycle_msgs::msg::State state_;
        rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr client_change_state_;
        rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr client_get_state_;
        LifecycleManager::SharedPtr node_;
        std::shared_ptr<lifecycle_msgs::srv::GetState_Request> get_request_;
        std::shared_ptr<lifecycle_msgs::srv::ChangeState_Request> change_request_;
        int prune_time_seconds_;
    };

    void timer_callback()
    {
        timer_nodes_->cancel();

        try
        {
            add_remove_nodes();
            do_state_machine();
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(get_logger(), "timer_callback: exception: %s", e.what());
        }
        catch (...)
        {
            RCLCPP_ERROR(get_logger(), "timer_callback: unknown exception");
        }
        
        timer_nodes_->reset();
    }

    void do_state_machine()
    {
        for (auto & [name, status_manager] : lifecycle_nodes_)
        {
            try
            {
                status_manager.prune_requests();
                status_manager.update_state();
            }
            catch(const std::exception& e)
            {
                RCLCPP_ERROR(get_logger(), "do_state_machine: node: %s, exception: %s", name.c_str(), e.what());
            }
            catch (...)
            {
                RCLCPP_ERROR(get_logger(), "do_state_machine: node: %s, unknown exception", name.c_str());
            }
        }
    }

    void add_remove_nodes()
    {
        auto current_running_nodes = get_node_names();

        std::ranges::transform(current_running_nodes, current_running_nodes.begin(), [](const std::string& str) {return (!str.empty() && str.front() == '/') ? str.substr(1) : str;});

        std::set<std::string> new_values;
        std::set<std::string> deleted_values;
        std::ranges::sort(current_running_nodes);
        std::ranges::set_difference(current_running_nodes, running_nodes_, std::inserter(new_values, new_values.end()));
        std::ranges::set_difference(running_nodes_, current_running_nodes, std::inserter(deleted_values, deleted_values.end()));

        std::erase_if(lifecycle_nodes_, [&deleted_values](const auto& pair) {return deleted_values.contains(pair.first);});

        RCLCPP_DEBUG(get_logger(), "Running Nodes: %ld, New Nodes: %ld", current_running_nodes.size(), new_values.size());
        for (const auto & node_name : new_values)
        {
            auto services_and_types = get_service_names_and_types_by_node(node_name, "/");
            //for (auto service : services_and_types) RCLCPP_INFO(get_logger(), "Node %s : service: %s", node_name.c_str(), service.first.c_str());
            if (!services_and_types.empty() && services_and_types.contains("/" + node_name + "/change_state"))
            {
                RCLCPP_INFO(get_logger(), "Node %s is lifecycle", node_name.c_str());
                *lifecycle_nodes_.emplace(node_name, NodeStatusManager(node_name, shared_from_this(), prune_time_seconds_)).first;
            }
            else
            {
                RCLCPP_INFO(get_logger(), "Node %s is NOT lifecycle", node_name.c_str());
            }
        }
        running_nodes_ = current_running_nodes;
    }

    void declare_node_parameters()
    {
        std::vector<ParameterNode::ActionParam> params = {
            ParameterNode::ActionParam(
                rclcpp::Parameter("prune_time_seconds", 5), 
                [this](const rclcpp::Parameter& param) 
                {
                    prune_time_seconds_ = static_cast<int>(param.as_int());
                }
            ),
        };
        add_action_parameters(params);
    }

    int timer_seconds_;
    int prune_time_seconds_;
    rclcpp::TimerBase::SharedPtr timer_nodes_;
    std::map<std::string, NodeStatusManager> lifecycle_nodes_;
    std::vector<std::string> running_nodes_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(LifecycleManager)