#pragma once

#include <string>

#include <opencv2/opencv.hpp>

#ifdef HAVE_CUDA
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudacodec.hpp>
#endif

#include "../base/Image.hpp"
#include "ffmpeg_video_writer.hpp"

namespace boblib::video
{
    enum class Codec
    {
        H264,
        HEVC,
        AVC1 // Does not work with Cuda, will switch to H264
    };

    class VideoWriter final
    {
    public:
        VideoWriter(const std::string &fileName, const cv::Size &frame_size, boblib::video::Codec codec, double fps, bool use_opencv = false, bool use_cuda = true) noexcept;

        ~VideoWriter() noexcept;

        void write(const cv::Mat &image) noexcept;

        void write(const boblib::base::Image &image) noexcept;

        void release() noexcept;

        bool is_open() const noexcept;

        bool using_cuda() const noexcept;

        std::string get_backend_name() const noexcept;

        [[nodiscard]] static boblib::video::Codec codec_from_string(std::string_view codec_str) noexcept;

    private:
        inline void create_video_writer() noexcept;

        bool use_opencv_{false};
        const bool using_cuda_;
        const std::string &fileName_;
        const boblib::video::Codec codec_;
        const double fps_;
        const cv::Size frame_size_;
#ifdef HAVE_CUDA
        cv::Ptr<cv::cudacodec::VideoWriter> cuda_video_writer_ptr_;
#endif
        std::unique_ptr<cv::VideoWriter> video_writer_ptr_;
        std::unique_ptr<FFmpegVideoWriter> ffmpeg_video_writer_ptr_;
    };
}
