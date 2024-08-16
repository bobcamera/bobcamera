#include "Utils.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/core/cuda.hpp>

using namespace boblib::base;

bool Utils::HasCuda()
{
    return cv::cuda::getCudaEnabledDeviceCount() > 0;
}

void Utils::ResetCuda()
{
    if (HasCuda())
    {
        cv::cuda::resetDevice();
    }
}
