#include "VideoWriter.hpp"
#include "../base/Utils.hpp"

using namespace boblib::video;

VideoWriter::VideoWriter(const std::string &fileName, const cv::Size &frame_size, boblib::video::Codec codec, double fps, bool use_cuda) noexcept
    : using_cuda_(use_cuda ? boblib::base::Utils::has_cuda() : false)
    , fileName_(fileName)
    , codec_(codec)
    , fps_(fps)
    , frame_size_(frame_size)
{
    create_video_writer();
}

VideoWriter::~VideoWriter() noexcept
{
    release();
}

void VideoWriter::write(const cv::Mat &image) noexcept
{
#ifdef HAVE_CUDA
    if (using_cuda_)
    {
        cv::cuda::GpuMat gpu_frame;
        gpu_frame.upload(image);
        cuda_video_writer_ptr_->write(gpu_frame);
    }
    else
#endif
    {
        video_writer_ptr_->write(image);
    }
}

void VideoWriter::write(const boblib::base::Image &image) noexcept
{
#ifdef HAVE_CUDA
    if (using_cuda_)
    {
        cuda_video_writer_ptr_->write(image.toCudaMat());
    }
    else
#endif
    {
        video_writer_ptr_->write(image.toMat());
    }
}

void VideoWriter::release() noexcept
{
#ifdef HAVE_CUDA
    if (using_cuda_)
    {
        cuda_video_writer_ptr_->release();
    }
    else
#endif
    {
        video_writer_ptr_->release();
    }
}

bool VideoWriter::is_open() const noexcept
{
    return using_cuda_ ? true : video_writer_ptr_->isOpened();
}

bool VideoWriter::using_cuda() const noexcept
{
    return using_cuda_;
}

inline void VideoWriter::create_video_writer() noexcept
{
#ifdef HAVE_CUDA
    if (using_cuda_)
    {
        std::cout << "Using cuda" << std::endl;
        std::function<cv::cudacodec::Codec(boblib::video::Codec)> cuda_codec = [](boblib::video::Codec codec)
        {
            switch (codec)
            {
            case boblib::video::Codec::H264:
                return cv::cudacodec::Codec::H264;
            case boblib::video::Codec::HEVC:
                return cv::cudacodec::Codec::HEVC;
            default:
                return cv::cudacodec::Codec::H264;
            }
        };

        try
        {
            cuda_video_writer_ptr_ = cv::cudacodec::createVideoWriter(fileName_, frame_size_, cuda_codec(codec_), fps_);
            std::cout << "cuda_video_writer_ptr_: " << cuda_video_writer_ptr_ << std::endl;
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
        
        return;
    }
#endif
    
    std::function<int(boblib::video::Codec)> fourcc = [](boblib::video::Codec codec)
    {
        switch (codec)
        {
        case boblib::video::Codec::H264:
            return cv::VideoWriter::fourcc('H', '2', '6', '4');
        case boblib::video::Codec::HEVC:
            return cv::VideoWriter::fourcc('H', 'E', 'V', 'C');
        case boblib::video::Codec::AVC1:
            return cv::VideoWriter::fourcc('a', 'v', 'c', '1');
        default:
            return cv::VideoWriter::fourcc('a', 'v', 'c', '1');
        }
    };

    video_writer_ptr_ = std::make_unique<cv::VideoWriter>(fileName_, cv::CAP_FFMPEG, fourcc(codec_), fps_, frame_size_);
}

std::string VideoWriter::get_backend_name() const noexcept
{
    return video_writer_ptr_->getBackendName();
}

[[nodiscard]] boblib::video::Codec VideoWriter::codec_from_string(std::string_view codec_str) noexcept
{
    using namespace std::literals;

    // Case-insensitive comparison
    if (codec_str == "H264"sv || codec_str == "h264"sv)
    {
        return Codec::H264;
    }
    else if (codec_str == "HEVC"sv || codec_str == "hevc"sv)
    {
        return Codec::HEVC;
    }
    else if (codec_str == "AVC1"sv || codec_str == "avc1"sv)
    {
        return Codec::AVC1;
    }

    return Codec::AVC1;
}