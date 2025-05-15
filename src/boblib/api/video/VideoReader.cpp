#include "VideoReader.hpp"

using namespace boblib::video;

VideoReader::VideoReader(int usb_camera_id, const std::vector<int> & params)
: using_cuda_(false)
, is_usb_(true)
, usb_camera_id_(usb_camera_id)
, params_(params)
{
    create_video_capture();
}

VideoReader::VideoReader(const std::string & camera_uri, bool use_cuda, const std::vector<int> & params)
: using_cuda_(use_cuda ? boblib::base::Utils::has_cuda() : false)
, is_usb_(false)
, camera_uri_(camera_uri)
, params_(params)
{
    create_video_capture();
}

VideoReader::~VideoReader() noexcept
{
    release();
}

void VideoReader::release() noexcept
{
#ifdef HAVE_CUDA
    if (using_cuda_)
    {
        cuda_video_reader_ptr_.reset();
    }
    else
#endif
    {
        if (ffmpeg_video_reader_ptr_)
        {
            ffmpeg_video_reader_ptr_->close();
            ffmpeg_video_reader_ptr_.reset();
        }
        video_capture_ptr_->release();
    }
}

bool VideoReader::read(boblib::base::Image &image) noexcept
{
    try
    {
#ifdef HAVE_CUDA
        if (using_cuda_)
        {
            if (!cuda_video_reader_ptr_->nextFrame(image.toCudaMat())) 
            {
                return false;
            }
            if (!image.get_using_cuda())
            {
                image.download();
            }
            return true;
        }
#endif
        if (!use_opencv_)
        {
            return ffmpeg_video_reader_ptr_->read(image.toMat());
        }
        return video_capture_ptr_->read(image.toMat());
    }
    catch (const std::exception & e)
    {
        std::cerr << e.what() << std::endl;
    }
    return false;
}

bool VideoReader::set(int parameter_id, double value) noexcept
{
    if (using_cuda_)
    {
        return false;
    }
    
    return video_capture_ptr_->set(parameter_id, value);
}

bool VideoReader::get(int parameter_id, double &value) const noexcept
{
#ifdef HAVE_CUDA
    if (using_cuda_)
    {
        return cuda_video_reader_ptr_->get(parameter_id, value);
    }
#endif
    if (!use_opencv_)
    {
        switch (parameter_id)
        {
            case cv::CAP_PROP_FPS:
                value = ffmpeg_video_reader_ptr_->get_fps();
                break;
            case cv::CAP_PROP_FRAME_WIDTH:
                value = ffmpeg_video_reader_ptr_->get_width();
                break;
            case cv::CAP_PROP_FRAME_HEIGHT:
                value = ffmpeg_video_reader_ptr_->get_height();
                break;
            default:
                return false;
        }
        return true;
    }

    value = video_capture_ptr_->get(parameter_id);
    return true;
}

bool VideoReader::using_cuda() const noexcept
{
    return using_cuda_;
}

bool VideoReader::is_open() const noexcept
{
    if (!use_opencv_)
    {
        return ffmpeg_video_reader_ptr_ ? ffmpeg_video_reader_ptr_->is_opened() : false;
    }
    return using_cuda_ ? true : video_capture_ptr_->isOpened();
}

inline void VideoReader::create_video_capture() noexcept
{
    static const std::vector<int> default_params_normal = {cv::CAP_PROP_HW_ACCELERATION, cv::VIDEO_ACCELERATION_ANY};
    static const std::vector<int> default_params_cuda = {cv::CAP_PROP_OPEN_TIMEOUT_MSEC, 10000, cv::CAP_PROP_CONVERT_RGB, 1};

    if (!use_opencv_)
    {
        ffmpeg_video_reader_ptr_ = std::make_unique<FFmpegVideoReader>(camera_uri_);
        return;
    }

    if (is_usb_)
    {
        video_capture_ptr_ = std::make_unique<cv::VideoCapture>(usb_camera_id_, cv::CAP_ANY, params_.empty() ? default_params_normal : params_);
#ifdef HAVE_CUDA
        cuda_video_reader_ptr_.reset();
#endif
        return;
    }

#ifdef HAVE_CUDA
    if (using_cuda_)
    {
        cuda_video_reader_ptr_ = cv::cudacodec::createVideoReader(camera_uri_, params_.empty() ? default_params_cuda : params_);
        cuda_video_reader_ptr_->set(cv::cudacodec::ColorFormat::BGR);
        video_capture_ptr_.reset();
        return;
    }
    cuda_video_reader_ptr_.reset();
#endif
    video_capture_ptr_ = std::make_unique<cv::VideoCapture>(camera_uri_, cv::CAP_ANY, params_.empty() ? default_params_normal : params_);
}

std::string VideoReader::get_codec_name() const noexcept
{
    if (ffmpeg_video_reader_ptr_)
    {
        return ffmpeg_video_reader_ptr_->get_codec_name();
    }
    return {};
}

std::string VideoReader::get_decoder_name() const noexcept
{
    if (ffmpeg_video_reader_ptr_)
    {
        return ffmpeg_video_reader_ptr_->get_decoder_name();
    }
    return {};
}

std::string VideoReader::get_pixel_format_name() const noexcept
{
    if (ffmpeg_video_reader_ptr_)
    {
        return ffmpeg_video_reader_ptr_->get_pixel_format_name();
    }
    return {};
}
