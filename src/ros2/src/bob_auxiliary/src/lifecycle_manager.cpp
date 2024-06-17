#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <thread>

#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <visibility_control.h>

#include <parameter_node.hpp>

class LifecycleManager 
    : public ParameterNode
{
public:
    COMPOSITION_PUBLIC
    explicit LifecycleManager(const rclcpp::NodeOptions & options)
        : ParameterNode("lifecycle_service_client_node", options)
    {
        declare_node_parameters();
        init();
    }

    void init()
    {
        timer_nodes_ = create_wall_timer(std::chrono::seconds(5), [this](){timer_callback();});
    }

private:
    class NodeStatusManager
    {
    public:
        NodeStatusManager(const std::string & name, LifecycleManager::SharedPtr base_node)
            : name_(name)
        {
            node_ = base_node->create_sub_node(name + "_manager");
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
            size_t n_pruned_change = client_change_state_->prune_requests_older_than(std::chrono::system_clock::now() - std::chrono::seconds(5), &pruned_requests);
            size_t n_pruned_get = client_change_state_->prune_requests_older_than(std::chrono::system_clock::now() - std::chrono::seconds(5), &pruned_requests);
            if (n_pruned_change || n_pruned_get) 
            {
                RCLCPP_INFO(node_->get_logger(), "The server hasn't replied for more than 5s, %zu requests were discarded, the discarded requests numbers are:", n_pruned_change + n_pruned_get);
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
    };

    rclcpp::TimerBase::SharedPtr timer_nodes_;
    std::map<std::string, NodeStatusManager> lifecycle_nodes_;
    // std::set<std::string> non_lifecycle_running_nodes_;
    std::vector<std::string> running_nodes_;

    void timer_callback()
    {
        timer_nodes_->cancel();

        add_remove_nodes();
        do_state_machine();

        timer_nodes_->reset();
    }

    void do_state_machine()
    {
        for (auto & node : lifecycle_nodes_)
        {
            node.second.prune_requests();
            node.second.update_state();
        }
    }

    void add_remove_nodes()
    {
        auto current_running_nodes = get_node_names();

        std::ranges::transform(current_running_nodes, current_running_nodes.begin(), [](const std::string& str) {return (!str.empty() && str.front() == '/') ? str.substr(1) : str;});

        std::set<std::string> new_values;
        std::set<std::string> deleted_values;
        // Creating the sets with new and deleted values
        std::ranges::sort(current_running_nodes);
        std::ranges::set_difference(current_running_nodes, running_nodes_, std::inserter(new_values, new_values.end()));
        std::ranges::set_difference(running_nodes_, current_running_nodes, std::inserter(deleted_values, deleted_values.end()));

        // Removing from our node map and set
        std::erase_if(lifecycle_nodes_, [&deleted_values](const auto& pair) {return deleted_values.contains(pair.first);});
        // std::erase_if(non_lifecycle_running_nodes_, [&deleted_values](const auto& node_name) {return deleted_values.contains(node_name);});

        for (const auto & node_name : new_values)
        {
            auto services_and_types = get_service_names_and_types_by_node(node_name, "/");
            //for (auto service : services_and_types) RCLCPP_INFO(get_logger(), "Node %s : service: %s", node_name.c_str(), service.first.c_str());
            if (!services_and_types.empty() && services_and_types.contains("/" + node_name + "/change_state"))
            {
                RCLCPP_INFO(get_logger(), "Node %s is lifecycle", node_name.c_str());
                *lifecycle_nodes_.emplace(node_name, NodeStatusManager(node_name, shared_from_this())).first;
            }
            else
            {
                RCLCPP_INFO(get_logger(), "Node %s is NOT lifecycle", node_name.c_str());
                // non_lifecycle_running_nodes_.emplace(node_name);
            }
        }
        running_nodes_ = current_running_nodes;
    }

    void declare_node_parameters()
    {
        std::vector<ParameterNode::ActionParam> params = {
            ParameterNode::ActionParam(
                rclcpp::Parameter("lifecycle_nodes", std::vector<std::string>()), 
                [this](const rclcpp::Parameter& param) 
                {
                    auto lifecycle_nodes = param.as_string_array();
                }
            ),
        };
        add_action_parameters(params);
    }
};

RCLCPP_COMPONENTS_REGISTER_NODE(LifecycleManager)