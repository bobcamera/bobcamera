#pragma once

#include <mutex>
#include <vector>
#include <string>

#include <opencv2/opencv.hpp>
#ifdef HAVE_CUDA
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudacodec.hpp>
#include <opencv2/cudafilters.hpp>
#endif

#include "Utils.hpp"

namespace boblib::base
{
    class Image final
    {
    public:
        Image(bool use_cuda = true) noexcept;

        Image(const Image &img);

        Image(Image &&img) noexcept;

        explicit Image(const cv::Mat &image);

        ~Image() noexcept;

        void reset() noexcept;

        Image &operator=(const Image &img);

        Image & operator=(Image && img) noexcept;

        int channels() const noexcept;

        Image clone() const;

        Image &create(int rows, int cols, int type, void *data = nullptr);

        Image &create(cv::Size size, int type);

        Image &create(const cv::Mat &image);

        size_t elemSize() const noexcept;

        size_t elemSize1() const noexcept;

        bool empty() const noexcept;

        void release() noexcept;

        cv::Size size() const noexcept;

        size_t step1() const noexcept;

        size_t total() const noexcept;

        int type() const noexcept;

        void apply_mask(Image &mask);

        uint8_t *data() const;

        void copyTo(Image &copy) const;

        void resizeTo(Image &resized, const cv::Size &size) const;

        void resize(const cv::Size &size);

        void convertColorTo(Image &converted, int type) const;

        void convertColor(int type);

        void medianBlurTo(Image &converted, int size) const;

        void medianBlur(int size);

        bool get_using_cuda() const noexcept;

        void download();

        void upload();

        const cv::Mat &toMat() const;

        const cv::cuda::GpuMat &toCudaMat() const;

        cv::Mat &toMat();

        cv::cuda::GpuMat &toCudaMat();

    private:
        void mask(cv::Mat &mask);

        bool using_cuda_;
        mutable std::unique_ptr<cv::cuda::GpuMat> gpu_mat_ptr_;
        mutable cv::Mat mat_;
        mutable std::mutex transfer_mutex_; // protects GPU↔CPU transfers when using_cuda_

#ifdef HAVE_CUDA
        void mask_cuda(cv::cuda::GpuMat &mask_);

        cv::Ptr<cv::cuda::Filter> median_filter_;
#endif
        int median_filter_size_{-1};
    };
}
