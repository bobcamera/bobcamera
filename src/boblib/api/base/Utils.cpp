#include "Utils.hpp"

#include <iostream>
#include <sstream>
#include <fstream>
#include <sys/sysinfo.h>
#include <thread>

#include <opencv2/opencv.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/core/ocl.hpp>

using namespace boblib::base;

bool Utils::has_cuda()
{
    return cv::cuda::getCudaEnabledDeviceCount() > 0;
}

void Utils::reset_cuda()
{
    if (has_cuda())
    {
        cv::cuda::resetDevice();
    }
}

bool Utils::has_opencl()
{
    return cv::ocl::haveOpenCL();
}

std::string Utils::get_ocl_info()
{
    std::ostringstream oss;

    if (cv::ocl::haveOpenCL())
    {
        cv::ocl::setUseOpenCL(true);
        if (cv::ocl::useOpenCL())
        {
            // grab the thread-local default context (auto-initialises if needed)
            cv::ocl::Context context = cv::ocl::Context::getDefault(true);
            size_t numDevices = context.ndevices();
            oss << "Number of OpenCL devices: " << numDevices << "\n";
            for (size_t i = 0; i < numDevices; ++i)
            {
                cv::ocl::Device dev = context.device(i);
                oss << "* Device name: " << dev.name() << "\n" 
                    << "       Vendor: " 
                    << dev.vendorName() << "\n" 
                    << "      version: " 
                    << dev.OpenCL_C_Version() << "\n";
                
            }
        }
        else
        {
            return "OpenCL is available but not being used by OpenCV.";
        }
    }
    else
    {
        return "OpenCL is not available on this system.";
    }

    return oss.str();
}

std::string Utils::get_cpu_name()
{
    std::ifstream cpuinfo("/proc/cpuinfo");
    std::string line;
    while (std::getline(cpuinfo, line))
    {
        if (line.find("model name") != std::string::npos)
        {
            return line.substr(line.find(":") + 2); // Skip "model name" and ": "
        }
    }
    return "Unknown CPU";
}

unsigned long Utils::get_memory_total()
{
    struct sysinfo info;
    sysinfo(&info);

    return info.totalram;
}

size_t Utils::get_available_threads()
{
    return static_cast<size_t>(std::max(1U, std::thread::hardware_concurrency()));
}