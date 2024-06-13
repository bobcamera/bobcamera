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

class LifecycleServiceClient 
    : public ParameterNode
{
public:
    COMPOSITION_PUBLIC
    explicit LifecycleServiceClient(const rclcpp::NodeOptions & options)
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
    class NodeStatus
    {
    public:
        NodeStatus(const std::string & _name, LifecycleServiceClient::SharedPtr node)
            : name(_name)
        {
            node_ = node->create_sub_node(_name + "_manager");
            state.id = lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
            state.label = "unknown";
            client_change_state_ = node_->create_client<lifecycle_msgs::srv::ChangeState>("/" + _name + "/change_state");
            client_get_state_ = node_->create_client<lifecycle_msgs::srv::GetState>("/" + _name + "/get_state");
            get_request_ = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
            change_request_ = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
        }

        std::string get_name() const
        {
            return name;
        }

        lifecycle_msgs::msg::State get_state() const
        {
            return state;
        }

        void update_state(std::chrono::seconds time_out = std::chrono::seconds(3))
        {
            if (!client_get_state_->wait_for_service(time_out)) 
            {
                RCLCPP_ERROR(node_->get_logger(), "Service %s is not available.", client_get_state_->get_service_name());
                
                state.id = lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
                state.label = "unknown";
            }

            auto future = client_get_state_->async_send_request(get_request_, [this](rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedFuture future) 
            {
                auto response = future.get();
                state.id = response->current_state.id;
                state.label = response->current_state.label;
                RCLCPP_DEBUG(node_->get_logger(), "get_state(%s) = %d:%s", name.c_str(), static_cast<int>(state.id), state.label.c_str());
            });
        }

        void change_state(std::uint8_t transition, std::chrono::seconds time_out = std::chrono::seconds(3))
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
                    RCLCPP_DEBUG(node_->get_logger(), "change_state(%s) -> Transition %d successfully triggered.", name.c_str(), static_cast<int>(transition));
                    update_state();
                }
                else
                {
                    RCLCPP_DEBUG(node_->get_logger(), "change_state(%s) -> Transition NOT triggered.", name.c_str());
                }
            });
        }

    private:
        const std::string name;
        lifecycle_msgs::msg::State state;
        rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr client_change_state_;
        rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr client_get_state_;
        LifecycleServiceClient::SharedPtr node_;
        std::shared_ptr<lifecycle_msgs::srv::GetState_Request> get_request_;
        std::shared_ptr<lifecycle_msgs::srv::ChangeState_Request> change_request_;
    };

    rclcpp::TimerBase::SharedPtr timer_nodes_;
    std::map<std::string, NodeStatus> lifecycle_nodes_;
    std::set<std::string> non_lifecycle_running_nodes_;
    std::set<std::string> running_nodes_;

    void timer_callback()
    {
        timer_nodes_->cancel();

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
        std::erase_if(non_lifecycle_running_nodes_, [&deleted_values](const auto& node_name) {return deleted_values.contains(node_name);});

        for (const auto & node_name : new_values)
        {
            auto services_and_types = get_service_names_and_types_by_node(node_name, "/");
            //for (auto service : services_and_types) RCLCPP_INFO(get_logger(), "Node %s : service: %s", node_name.c_str(), service.first.c_str());
            if (!services_and_types.empty() && services_and_types.contains("/" + node_name + "/change_state"))
            {
                RCLCPP_INFO(get_logger(), "Node %s is lifecycle", node_name.c_str());
                lifecycle_nodes_.emplace(node_name, NodeStatus(node_name, shared_from_this()));
                check_change_status(node_name);
            }
            else
            {
                RCLCPP_INFO(get_logger(), "Node %s is NOT lifecycle", node_name.c_str());
                non_lifecycle_running_nodes_.emplace(node_name);
            }
        }
        running_nodes_.clear();
        running_nodes_.insert(current_running_nodes.begin(), current_running_nodes.end());

        timer_nodes_->reset();
    }

    void check_change_status(const std::string & node_name)
    {
        // TODO: implement a state machine, right now we only send a configure state
        lifecycle_nodes_.at(node_name).change_state(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
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

    template<typename FutureT, typename WaitTimeT> 
    static std::future_status wait_for_result(FutureT & future, WaitTimeT time_to_wait)
    {
        auto end = std::chrono::steady_clock::now() + time_to_wait;
        std::chrono::milliseconds wait_period(100);
        std::future_status status = std::future_status::timeout;
        do 
        {
            auto now = std::chrono::steady_clock::now();
            auto time_left = end - now;
            if (time_left <= std::chrono::seconds(0)) 
            {
                break;
            }
            status = future.wait_for((time_left < wait_period) ? time_left : wait_period);
        } while (rclcpp::ok() && (status != std::future_status::ready));

        return status;
    }
};

RCLCPP_COMPONENTS_REGISTER_NODE(LifecycleServiceClient)

int main(int argc, char ** argv)
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);

    auto lc_client = std::make_shared<LifecycleServiceClient>(rclcpp::NodeOptions());
    lc_client->init();

    rclcpp::spin(lc_client);

    rclcpp::shutdown();

    return 0;
}