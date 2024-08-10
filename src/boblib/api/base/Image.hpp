#pragma once

#include <vector>
#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudacodec.hpp>
#include <opencv2/cudafilters.hpp>

namespace boblib::base
{
    class Image
    {
    public:
        Image(bool use_cuda = true);

        ~Image();

        int channels() const;

        Image clone() const;

        void create(int rows, int cols, int type);

        void create(cv::Size size, int type);

        void create(const cv::Mat & image);

        size_t elemSize() const;

        size_t elemSize1() const;

        bool empty() const;

        void release();

        cv::Size size() const;

        size_t step1() const;

        size_t total() const;

        int type() const;

        void apply_mask(Image & mask);

        uint8_t* data() const;

        void copyTo(Image & copy) const;

        void resizeTo(Image & resized, const cv::Size & size) const;

        void resize(const cv::Size & size);

        void convertTo(Image & converted, int type) const;

        void convert(int type);

        void medianBlurTo(Image & converted, int size) const;

        void medianBlur(int size);

        bool get_using_cuda() const;

        cv::cuda::GpuMat & get_cuda_mat();

        cv::Mat & get_mat();

        const cv::cuda::GpuMat & get_cuda_mat() const;

        const cv::Mat & get_mat() const;

        cv::Mat & download();

        cv::cuda::GpuMat & upload();

        const cv::Mat toMat() const;

        const cv::cuda::GpuMat toCudaMat() const;

    private:
        void mask(cv::Mat & mask);
        void mask_cuda(cv::cuda::GpuMat & mask_);

        bool using_cuda_;
        mutable cv::cuda::GpuMat gpu_mat_;
        mutable cv::Mat mat_;

        cv::Ptr<cv::cuda::Filter> box_filter_;
        int box_filter_size_{-1};
    };
}
