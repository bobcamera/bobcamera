#pragma once

#include <vector>
#include <string>

#include <opencv2/opencv.hpp>
#ifdef HAVE_CUDA
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudacodec.hpp>
#endif

#include "../base/Image.hpp"
#include "ffmpeg_video_reader.hpp"

namespace boblib::video
{
    class VideoReader final
    {
    public:
        VideoReader(int usb_camera_id, bool use_opencv = false, const std::vector<int> & params = {});

        VideoReader(const std::string &camera_uri, bool use_opencv = false, bool use_cuda = true, const std::vector<int> &params = {});

        ~VideoReader() noexcept;

        bool read(boblib::base::Image &image) noexcept;

        int get_width() const noexcept;
        int get_height() const noexcept;
        double get_fps() const noexcept;
        bool set_resolution(int width, int height) noexcept;
        std::vector<std::pair<int, int>> list_camera_resolutions() const noexcept;

        bool is_open() const noexcept;

        bool using_cuda() const noexcept;

        void release() noexcept;

        std::string get_codec_name() const noexcept;
        std::string get_decoder_name() const noexcept;
        std::string get_pixel_format_name() const noexcept;

    private:
        inline void create_video_capture() noexcept;

        bool use_opencv_{false};
        bool using_cuda_;
        bool is_usb_;
        int usb_camera_id_{-1};
        const std::string camera_uri_;
        const std::vector<int> & params_;
        std::unique_ptr<cv::VideoCapture> video_capture_ptr_;
#ifdef HAVE_CUDA
        cv::Ptr<cv::cudacodec::VideoReader> cuda_video_reader_ptr_;
#endif
        std::unique_ptr<FFmpegVideoReader> ffmpeg_video_reader_ptr_;
    };
}
