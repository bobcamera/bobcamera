#include <chrono>
#include <string>
#include <thread>

#include <rclcpp_components/register_node_macro.hpp>
#include <visibility_control.h>

#include <boblib/api/base/Utils.hpp>

#include <parameter_node.hpp>

class Information 
    : public ParameterNode
{
public:
    COMPOSITION_PUBLIC
    explicit Information(const rclcpp::NodeOptions & options)
        : ParameterNode("information_node", options)
    {
    }

    void on_configure() override
    {
        log_info("Configuring");

        log_common_information();
    }

private:
    void log_common_information()
    {
        auto has_opencl = boblib::base::Utils::has_opencl();
        log_info("Basic Information:");
        log_info("              CPU: %s", boblib::base::Utils::get_cpu_name().c_str());
        log_info("Available Threads: %ld", boblib::base::Utils::get_available_threads());
        log_info("     Total Memory: %.2f GB", static_cast<double>(boblib::base::Utils::get_memory_total()) / (1024.0 * 1024.0 * 1024.0));
        log_info("         Has Cuda: %s", boblib::base::Utils::has_cuda() ? "True" : "False");
        log_info("       Has OpenCL: %s", has_opencl ? "True" : "False");
        if (has_opencl)
        {
            log_info("%s", boblib::base::Utils::get_ocl_info().c_str());
        }

        auto rmw_identifier = rmw_get_implementation_identifier();
        if (rmw_identifier)
        {
            log_info("Using RMW implementation: %s", rmw_identifier);
        }
        else
        {
            log_info("Could not retrieve RMW implementation identifier.");
        }
    }
};

RCLCPP_COMPONENTS_REGISTER_NODE(Information)