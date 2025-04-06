#pragma once

#include <string>

#include <opencv2/opencv.hpp>

#ifdef HAVE_CUDA
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudacodec.hpp>
#endif

#include "../base/Image.hpp"


namespace boblib::video
{
    enum class Codec
    {
        H264,
        HEVC,
        AVC1 // Does not work with Cuda, will switch to H264
    };

    class VideoWriter
    {
    public:
        VideoWriter(const std::string & fileName, const cv::Size & frame_size, boblib::video::Codec codec, double fps, bool use_cuda = true);

        ~VideoWriter();

        void write(const cv::Mat & image);

        void write(const boblib::base::Image & image);

        void release();

        bool is_open() const;

        bool using_cuda() const;

        [[nodiscard]] static boblib::video::Codec codec_from_string(std::string_view codec_str) noexcept;

    private:
        inline void create_video_writer();

        const bool using_cuda_;
        const std::string &fileName_;
        const boblib::video::Codec codec_;
        const double fps_;
        const cv::Size frame_size_;
#ifdef HAVE_CUDA
        std::unique_ptr<cv::cudacodec::VideoWriter> cuda_video_writer_ptr_;
#endif
        std::unique_ptr<cv::VideoWriter> video_writer_ptr_;
    };
}
