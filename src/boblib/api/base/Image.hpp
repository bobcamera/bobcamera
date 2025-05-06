#pragma once

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

        Image(const Image & img) noexcept;

        Image(Image&& img) noexcept;

        Image(const cv::Mat &image) noexcept;

        ~Image() noexcept;

        void reset() noexcept;

        Image &operator=(const Image &img) noexcept;

        Image & operator=(Image && img) noexcept;

        int channels() const noexcept;

        Image clone() const noexcept;

        Image &create(int rows, int cols, int type, void *data = nullptr) noexcept;

        Image &create(cv::Size size, int type) noexcept;

        Image &create(const cv::Mat &image) noexcept;

        size_t elemSize() const noexcept;

        size_t elemSize1() const noexcept;

        bool empty() const noexcept;

        void release() noexcept;

        cv::Size size() const noexcept;

        size_t step1() const noexcept;

        size_t total() const noexcept;

        int type() const noexcept;

        void apply_mask(Image &mask) noexcept;

        uint8_t *data() const noexcept;

        void copyTo(Image &copy) const noexcept;

        void resizeTo(Image &resized, const cv::Size &size) const noexcept;

        void resize(const cv::Size &size) noexcept;

        void convertColorTo(Image &converted, int type) const noexcept;

        void convertColor(int type) noexcept;

        void medianBlurTo(Image &converted, int size) const noexcept;

        void medianBlur(int size) noexcept;

        bool get_using_cuda() const noexcept;

        void download() noexcept;

        void upload() noexcept;

        const cv::Mat &toMat() const noexcept;

        const cv::cuda::GpuMat &toCudaMat() const noexcept;

        cv::Mat &toMat() noexcept;

        cv::cuda::GpuMat &toCudaMat() noexcept;

    private:
        void mask(cv::Mat &mask) noexcept;

        bool using_cuda_;
        mutable std::unique_ptr<cv::cuda::GpuMat> gpu_mat_ptr_;
        mutable std::unique_ptr<cv::Mat> mat_ptr_;

#ifdef HAVE_CUDA
        void mask_cuda(cv::cuda::GpuMat &mask_) noexcept;

        cv::Ptr<cv::cuda::Filter> median_filter_;
#endif
        int median_filter_size_{-1};
    };
}
