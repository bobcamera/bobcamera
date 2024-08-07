#pragma once

#include <vector>
#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudacodec.hpp>

namespace boblib::video
{
    class VideoReader
    {
    public:
        VideoReader(int usb_camera_id, bool use_cuda = true, const std::vector<int> & params = {});

        VideoReader(const std::string & camera_uri, bool use_cuda = true, const std::vector<int> & params = {});

        bool read(cv::Mat & image) const;

        bool set(int parameter_id, double value);

        bool get(int parameter_id, double & value) const;

        bool is_open() const;

        bool using_cuda() const;

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
