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
#include <visibility_control.h>

#include <parameter_node.hpp>

using namespace std::chrono_literals;

class LifecycleServiceClient 
    : public ParameterNode
{
public:
    COMPOSITION_PUBLIC
    explicit LifecycleServiceClient(const std::string & node_name)
        : ParameterNode(node_name)
    {
        declare_node_parameters();
    }

    bool change_states(std::uint8_t transition, std::chrono::seconds time_out = 3s) 
    {
        bool success = true;
        for (const auto & node_name : lifecycle_nodes_)
        {
            success &= change_state(node_name, transition, time_out);
        }
        return success;
    }

    unsigned int get_state(const std::string & node, std::chrono::seconds time_out = 3s)
    {
        auto client_get_state_ = create_client<lifecycle_msgs::srv::GetState>(node + "/get_state");
        auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();

        if (!client_get_state_->wait_for_service(time_out)) 
        {
            RCLCPP_ERROR(get_logger(), "Service %s is not available.", client_get_state_->get_service_name());
            return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
        }

        auto future_result = client_get_state_->async_send_request(request).future.share();

        auto future_status = wait_for_result(future_result, time_out);

        if (future_status != std::future_status::ready) 
        {
            RCLCPP_ERROR(get_logger(), "Server time out while getting current state for node %s", node.c_str());
            return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
        }

        if (future_result.get()) 
        {
            RCLCPP_INFO(get_logger(), "Node %s has current state %s.", node.c_str(), future_result.get()->current_state.label.c_str());
            return future_result.get()->current_state.id;
        } 
        else 
        {
            RCLCPP_ERROR(get_logger(), "Failed to get current state for node %s", node.c_str());
            return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
        }
    }

    bool change_state(const std::string & node, std::uint8_t transition, std::chrono::seconds time_out = 3s)
    {
        auto client_change_state_ = create_client<lifecycle_msgs::srv::ChangeState>(node + "/change_state");
        auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
        request->transition.id = transition;

        if (!client_change_state_->wait_for_service(time_out)) 
        {
            RCLCPP_ERROR(get_logger(),"Service %s is not available.", client_change_state_->get_service_name());
            return false;
        }

        auto future_result = client_change_state_->async_send_request(request).future.share();

        auto future_status = wait_for_result(future_result, time_out);

        if (future_status != std::future_status::ready) 
        {
            RCLCPP_ERROR(get_logger(), "Server time out while getting current state for node %s", node.c_str());
            return false;
        }

        if (future_result.get()->success) 
        {
            RCLCPP_INFO(get_logger(), "Transition %d successfully triggered.", static_cast<int>(transition));
            return true;
        } 
        else 
        {
            RCLCPP_WARN(get_logger(), "Failed to trigger transition %u", static_cast<unsigned int>(transition));
            return false;
        }
    }

private:
    std::vector<std::string> lifecycle_nodes_;

    void declare_node_parameters()
    {
        std::vector<ParameterNode::ActionParam> params = {
            ParameterNode::ActionParam(
                rclcpp::Parameter("lifecycle_nodes", std::vector<std::string>()), 
                [this](const rclcpp::Parameter& param) 
                {
                    lifecycle_nodes_ = param.as_string_array();
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

void callee_script(std::shared_ptr<LifecycleServiceClient> lc_client)
{
    using Transition = lifecycle_msgs::msg::Transition;

    // configure
    {
        if (!lc_client->change_states(Transition::TRANSITION_CONFIGURE)) 
        {
            return;
        }
    }
}

void wake_executor(std::shared_future<void> future, rclcpp::executors::SingleThreadedExecutor & exec)
{
    future.wait();
    exec.cancel();
}

int main(int argc, char ** argv)
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);

    auto lc_client = std::make_shared<LifecycleServiceClient>("lc_client");

    rclcpp::executors::SingleThreadedExecutor exe;
    exe.add_node(lc_client);

    std::shared_future<void> script = std::async(std::launch::async, callee_script, lc_client);

    auto wake_exec = std::async(std::launch::async, wake_executor, script,std::ref(exe));

    exe.spin_until_future_complete(script);

    rclcpp::shutdown();

    return 0;
}