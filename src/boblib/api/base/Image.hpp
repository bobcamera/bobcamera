#pragma once

#include <vector>
#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudacodec.hpp>

namespace boblib::base
{
    class Image
    {
    public:
        Image(bool use_cuda = true);

        int channels() const;

        Image clone() const;

        void create(int rows, int cols, int type);

        void create(cv::Size size, int type);

        size_t elemSize() const;

        size_t elemSize1() const;

        bool empty() const;

        void release();

        cv::Size size() const;

        size_t step1() const;

        int type() const;

        void apply_mask(Image & mask);

    private:
        void mask(cv::Mat & mask);
        void mask_cuda(cv::cuda::GpuMat & mask_);

        bool using_cuda_;
        cv::cuda::GpuMat gpu_mat_;
        cv::Mat mat_;
    };
}
