#pragma once

#include <vector>
#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudacodec.hpp>

#include "../base/Image.hpp"

namespace boblib::video
{
    class VideoReader
    {
    public:
        VideoReader(int usb_camera_id, const std::vector<int> & params = {});

        VideoReader(const std::string & camera_uri, bool use_cuda = true, const std::vector<int> & params = {});

        ~VideoReader();

        bool read(boblib::base::Image & image);

        bool set(int parameter_id, double value);

        bool get(int parameter_id, double & value) const;

        bool is_open() const;

        bool using_cuda() const;

        void release();

    private:

        inline void create_video_capture();

        bool using_cuda_;
        bool is_usb_;
        int usb_camera_id_{-1};
        const std::string camera_uri_;
        const std::vector<int> & params_;
        std::unique_ptr<cv::VideoCapture> video_capture_ptr_;
        cv::Ptr<cv::cudacodec::VideoReader> cuda_video_reader_ptr_;
    };
}
