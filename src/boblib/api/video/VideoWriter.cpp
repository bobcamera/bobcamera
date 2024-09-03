#include "VideoWriter.hpp"
#include "../base/Utils.hpp"

using namespace boblib::video;

VideoWriter::VideoWriter(const std::string & fileName, const cv::Size & frame_size, boblib::video::Codec codec, double fps, bool use_cuda)
    : using_cuda_(use_cuda ? boblib::base::Utils::HasCuda() : false)
    , fileName_(fileName)
    , codec_(codec)
    , fps_(fps)
    , frame_size_(frame_size)
{
    create_video_writer();
}

VideoWriter::~VideoWriter()
{
    release();
}

void VideoWriter::write(const cv::Mat & image)
{
    if (using_cuda_)
    {
        cv::cuda::GpuMat gpu_frame;
        gpu_frame.upload(image);
        cuda_video_writer_ptr_->write(gpu_frame);
    }
    else
    {
        video_writer_ptr_->write(image);
    }
}

void VideoWriter::release()
{
    if (using_cuda_)
    {
        cuda_video_writer_ptr_->release();
    }
    else 
    {
        video_writer_ptr_->release();
    } 
}

bool VideoWriter::is_open() const
{
    return using_cuda_ ? true : video_writer_ptr_->isOpened();
}

bool VideoWriter::using_cuda() const
{
    return using_cuda_;
}

inline void VideoWriter::create_video_writer()
{
    if (using_cuda_)
    {
        std::function<cv::cudacodec::Codec(boblib::video::Codec)> cuda_codec = [](boblib::video::Codec codec) {
            switch (codec) {
                case boblib::video::Codec::H264: return cv::cudacodec::Codec::H264;
                case boblib::video::Codec::HEVC: return cv::cudacodec::Codec::HEVC;
                default: return cv::cudacodec::Codec::H264;
            }
        };

        cuda_video_writer_ptr_ = cv::cudacodec::createVideoWriter(fileName_, frame_size_, cuda_codec(codec_), fps_);
        return;
    }

    std::function<int(boblib::video::Codec)> fourcc = [](boblib::video::Codec codec) {
        switch (codec) {
            case boblib::video::Codec::H264: return cv::VideoWriter::fourcc('H', '2', '6', '4');
            case boblib::video::Codec::HEVC: return cv::VideoWriter::fourcc('H', 'E', 'V', 'C');
            case boblib::video::Codec::AVC1: return cv::VideoWriter::fourcc('a', 'v', 'c', '1');
            default: return cv::VideoWriter::fourcc('a', 'v', 'c', '1');
        }
    };

    video_writer_ptr_ = std::make_unique<cv::VideoWriter>(fileName_, fourcc(codec_), fps_, frame_size_);
}

