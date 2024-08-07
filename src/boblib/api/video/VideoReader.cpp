
#include "VideoReader.hpp"

using namespace boblib::video;

VideoReader::VideoReader(int usb_camera_id, bool use_cuda, const std::vector<int> & params)
: is_usb_(true)
, usb_camera_id_(usb_camera_id)
, params_(params)
{
    using_cuda_ = use_cuda ? cv::cuda::getCudaEnabledDeviceCount() : false;
    create_video_capture();
}

VideoReader::VideoReader(const std::string & camera_uri, bool use_cuda, const std::vector<int> & params)
: is_usb_(false)
, camera_uri_(camera_uri)
, params_(params)
{
    using_cuda_ = use_cuda ? cv::cuda::getCudaEnabledDeviceCount() : false;
    create_video_capture();
}

bool VideoReader::read(cv::Mat & image) const
{
    if (using_cuda_)
    {
        cv::cuda::GpuMat gpu_frame;
        if (!cuda_video_reader_ptr_->nextFrame(gpu_frame)) 
        {
            return false;
        }
        gpu_frame.download(image);
        return true;
    }

    return video_capture_ptr_->read(image);
}

bool VideoReader::set(int parameter_id, double value)
{
    if (using_cuda_)
    {
        return false;
    }
    
    return video_capture_ptr_->set(parameter_id, value);
}

bool VideoReader::get(int parameter_id, double & value) const
{
    if (using_cuda_)
    {
        return cuda_video_reader_ptr_->get(parameter_id, value);
    }
    value = video_capture_ptr_->get(parameter_id);
    return true;
}

bool VideoReader::using_cuda() const
{
    return using_cuda_;
}

bool VideoReader::is_open() const
{
    return using_cuda_ ? true : video_capture_ptr_->isOpened();
}

inline void VideoReader::create_video_capture()
{
    static const std::vector<int> default_params_normal = {cv::CAP_PROP_HW_ACCELERATION, cv::VIDEO_ACCELERATION_ANY};
    static const std::vector<int> default_params_cuda = {cv::CAP_PROP_OPEN_TIMEOUT_MSEC, 10000, cv::CAP_PROP_CONVERT_RGB, 1};

    if (is_usb_)
    {
        video_capture_ptr_ = std::make_unique<cv::VideoCapture>(usb_camera_id_, cv::CAP_ANY, params_.empty() ? default_params_normal : params_);
        cuda_video_reader_ptr_.reset();
        return;
    }

    if (using_cuda_)
    {
        cuda_video_reader_ptr_ = cv::cudacodec::createVideoReader(camera_uri_, params_.empty() ? default_params_cuda : params_);
        cuda_video_reader_ptr_->set(cv::cudacodec::ColorFormat::BGR);
        video_capture_ptr_.reset();
        return;
    }

    cuda_video_reader_ptr_.reset();
    video_capture_ptr_ = std::make_unique<cv::VideoCapture>(camera_uri_, cv::CAP_ANY, params_.empty() ? default_params_normal : params_);
}
