#include "Image.hpp"

#ifdef HAVE_CUDA
#include <opencv2/cudawarping.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudaarithm.hpp>
#endif

using namespace boblib::base;

Image::Image(bool use_cuda) noexcept
    : using_cuda_(use_cuda ? Utils::has_cuda() : false)
{
    reset();
}

Image::Image(const Image &img)
    : using_cuda_(img.using_cuda_),
      median_filter_size_(img.median_filter_size_)
{
    if (using_cuda_)
    {
#ifdef HAVE_CUDA
        median_filter_ = img.median_filter_;
#endif
        gpu_mat_ptr_ = std::make_unique<cv::cuda::GpuMat>();
        img.gpu_mat_ptr_->copyTo(*gpu_mat_ptr_);
    }
    else
    {
        mat_ = img.mat_.clone();
    }
}

Image::Image(Image &&img) noexcept
    : using_cuda_(img.using_cuda_),
      gpu_mat_ptr_(std::move(img.gpu_mat_ptr_)),
      mat_(std::move(img.mat_)),
      median_filter_size_(std::exchange(img.median_filter_size_, -1))
{
#ifdef HAVE_CUDA
    median_filter_ = std::move(img.median_filter_);
#endif
    // Reseting the Moved img
    img.reset();
}

Image::Image(const cv::Mat &image)
    : using_cuda_(Utils::has_cuda())
{
    reset();
    create(image);
}

Image::~Image() noexcept
{
}

void Image::reset() noexcept
{
    if (using_cuda_)
    {
        gpu_mat_ptr_ = std::make_unique<cv::cuda::GpuMat>();
    }
    mat_ = cv::Mat();
#ifdef HAVE_CUDA
    median_filter_.reset();
#endif
    median_filter_size_ = -1;
}

Image &Image::operator=(const Image &img)
{
    if (this == &img)
    {
        return *this;
    }

    using_cuda_ = img.using_cuda_;
    median_filter_size_ = img.median_filter_size_;
#ifdef HAVE_CUDA
    median_filter_ = img.median_filter_;
#endif

    if (using_cuda_)
    {
        if (!gpu_mat_ptr_)
        {
            gpu_mat_ptr_ = std::make_unique<cv::cuda::GpuMat>();
        }
        img.gpu_mat_ptr_->copyTo(*gpu_mat_ptr_);
        mat_ = cv::Mat();
    }
    else
    {
        img.mat_.copyTo(mat_);
        gpu_mat_ptr_.reset();
    }

    return *this;
}

Image& Image::operator=(Image && img) noexcept
{
    if (this == &img)
    {
        return *this;
    }

    using_cuda_ = img.using_cuda_;
    gpu_mat_ptr_ = std::move(img.gpu_mat_ptr_);
    mat_ = std::move(img.mat_);
#ifdef HAVE_CUDA
    median_filter_ = std::move(img.median_filter_);
#endif
    median_filter_size_ = std::exchange(img.median_filter_size_, -1);

    img.reset();

    return *this;
}

int Image::channels() const noexcept
{
    return using_cuda_ ? gpu_mat_ptr_->channels() : mat_.channels();
}

Image Image::clone() const
{
    Image copy(using_cuda_);
    if (using_cuda_)
    {
        gpu_mat_ptr_->copyTo(*copy.gpu_mat_ptr_);
    }
    else
    {
        mat_.copyTo(copy.mat_);
    }
    return copy;
}

Image &Image::create(int rows, int cols, int type, void *data)
{
    if (using_cuda_)
    {
        gpu_mat_ptr_ = data != nullptr ? std::make_unique<cv::cuda::GpuMat>(rows, cols, type, data) : std::make_unique<cv::cuda::GpuMat>(rows, cols, type);
        mat_ = cv::Mat();
        return *this;
    }
    size_t elemSize = CV_ELEM_SIZE(type);
    size_t alignedStep = cv::alignSize(cols * elemSize, 64); // Align to 64-byte boundary
    mat_ = data != nullptr ? cv::Mat(rows, cols, type, data) : cv::Mat(rows, cols, type, nullptr, alignedStep);
    gpu_mat_ptr_.reset();

    return *this;
}

Image &Image::create(cv::Size size, int type)
{
    mat_.release();
    if (using_cuda_)
    {
        gpu_mat_ptr_->release();
        gpu_mat_ptr_->create(size, type);
        return *this;
    }
    mat_.create(size, type);

    return *this;
}

Image &Image::create(const cv::Mat &image)
{
    if (using_cuda_)
    {
        gpu_mat_ptr_->upload(image);
        return *this;
    }

    mat_ = image.clone();

    return *this;
}

size_t Image::elemSize() const noexcept
{
    return using_cuda_ ? gpu_mat_ptr_->elemSize() : mat_.elemSize();
}

size_t Image::elemSize1() const noexcept
{
    return using_cuda_ ? gpu_mat_ptr_->elemSize1() : mat_.elemSize1();
}

bool Image::empty() const noexcept
{
    return using_cuda_ ? gpu_mat_ptr_->empty() : mat_.empty();
}

void Image::release() noexcept
{
    if (using_cuda_)
    {
        gpu_mat_ptr_->release();
    }
    mat_.release();
}

cv::Size Image::size() const noexcept
{
    return using_cuda_ ? gpu_mat_ptr_->size() : mat_.size();
}

size_t Image::step1() const noexcept
{
    return using_cuda_ ? gpu_mat_ptr_->step1() : mat_.step1();
}

size_t Image::total() const noexcept
{
    return using_cuda_ ? gpu_mat_ptr_->size().area() : mat_.total();
}

int Image::type() const noexcept
{
    return using_cuda_ ? gpu_mat_ptr_->type() : mat_.type();
}

void Image::apply_mask(Image &_mask)
{
#ifdef HAVE_CUDA
    if (using_cuda_)
    {
        mask_cuda(*_mask.gpu_mat_ptr_);
        return;
    }
#endif

    mask(_mask.mat_);
}

uint8_t *Image::data() const
{
    if (using_cuda_)
    {
        std::lock_guard<std::mutex> lock(transfer_mutex_);
        gpu_mat_ptr_->download(mat_);
    }

    return mat_.data;
}

void Image::copyTo(Image &copy) const
{
    if (this == &copy)
    {
        return;
    }

    if (using_cuda_)
    {
        if (copy.using_cuda_)
        {
            gpu_mat_ptr_->copyTo(*copy.gpu_mat_ptr_);
            return;
        }

        {
            std::lock_guard<std::mutex> lock(transfer_mutex_);
            gpu_mat_ptr_->download(mat_);
        }
        mat_.copyTo(copy.mat_);
        return;
    }

    if (copy.using_cuda_)
    {
        std::lock_guard<std::mutex> lock(copy.transfer_mutex_);
        copy.gpu_mat_ptr_->upload(mat_);
        return;
    }

    mat_.copyTo(copy.mat_);
}

void Image::resizeTo(Image &resized, const cv::Size &size) const
{
#ifdef HAVE_CUDA
    if (using_cuda_)
    {
        if (resized.using_cuda_)
        {
            cv::cuda::resize(*gpu_mat_ptr_, *resized.gpu_mat_ptr_, size);
            return;
        }
        {
            std::lock_guard<std::mutex> lock(transfer_mutex_);
            gpu_mat_ptr_->download(mat_);
        }
        cv::resize(mat_, resized.mat_, size);
        return;
    }

    if (resized.using_cuda_)
    {
        cv::resize(mat_, resized.mat_, size);
        resized.upload();
        return;
    }
#endif

    cv::resize(mat_, resized.mat_, size);
}

void Image::resize(const cv::Size &size)
{
#ifdef HAVE_CUDA
    if (using_cuda_)
    {
        cv::cuda::resize(*gpu_mat_ptr_, *gpu_mat_ptr_, size);
    }
    else
#endif
    {
        cv::resize(mat_, mat_, size);
    }
}

void Image::medianBlurTo(Image &blured, int size) const
{
#ifdef HAVE_CUDA
    if (using_cuda_)
    {
        if (blured.using_cuda_)
        {
            if (!median_filter_ || median_filter_size_ != size)
            {
                median_filter_ = cv::cuda::createMedianFilter(gpu_mat_ptr_->type(), gpu_mat_ptr_->type(), cv::Size(size, size));
                median_filter_size_ = size;
            }
            median_filter_->apply(*gpu_mat_ptr_, *blured.gpu_mat_ptr_);
            return;
        }

        {
            std::lock_guard<std::mutex> lock(transfer_mutex_);
            gpu_mat_ptr_->download(mat_);
        }
        cv::medianBlur(mat_, blured.mat_, size);
        return;
    }

    if (blured.using_cuda_)
    {
        cv::medianBlur(mat_, blured.mat_, size);
        blured.upload();
        return;
    }
#endif
    cv::medianBlur(mat_, blured.mat_, size);
}

void Image::medianBlur(int size)
{
#ifdef HAVE_CUDA
    if (using_cuda_)
    {
        if (!median_filter_ || median_filter_size_ != size)
        {
            median_filter_ = cv::cuda::createMedianFilter(gpu_mat_ptr_->type(), gpu_mat_ptr_->type(), cv::Size(size, size));
            median_filter_size_ = size;
        }
        median_filter_->apply(*gpu_mat_ptr_, *gpu_mat_ptr_);
    }
    else
#endif
    {
        cv::medianBlur(mat_, mat_, size);
    }
}

void Image::convertColorTo(Image &converted, int type) const
{
#ifdef HAVE_CUDA
    if (using_cuda_)
    {
        if (converted.using_cuda_)
        {
            cv::cuda::cvtColor(*gpu_mat_ptr_, *converted.gpu_mat_ptr_, type);
            return;
        }

        {
            std::lock_guard<std::mutex> lock(transfer_mutex_);
            gpu_mat_ptr_->download(mat_);
        }
        cv::cvtColor(mat_, converted.mat_, type);
        return;
    }

    if (converted.using_cuda_)
    {
        cv::cvtColor(mat_, converted.mat_, type);
        converted.upload();
        return;
    }
#endif

    cv::cvtColor(mat_, converted.mat_, type);
}

void Image::convertColor(int type)
{
#ifdef HAVE_CUDA
    if (using_cuda_)
    {
        cv::cuda::cvtColor(*gpu_mat_ptr_, *gpu_mat_ptr_, type);
    }
    else
#endif
    {
        cv::cvtColor(mat_, mat_, type);
    }
}

void Image::mask(cv::Mat &mask)
{
    if (mat_.size() != mask.size())
    {
        cv::resize(mask, mask, mat_.size());
    }

    if (mat_.channels() != mask.channels())
    {
        cv::cvtColor(mask, mask, mat_.channels() == 3 ? cv::COLOR_GRAY2BGR : cv::COLOR_BGR2GRAY);
    }

    cv::bitwise_and(mat_, mask, mat_);
}

#ifdef HAVE_CUDA
void Image::mask_cuda(cv::cuda::GpuMat &mask)
{
    if (gpu_mat_ptr_->size() != mask.size())
    {
        cv::cuda::resize(mask, mask, gpu_mat_ptr_->size());
    }

    if (gpu_mat_ptr_->channels() != mask.channels())
    {
        cv::cuda::cvtColor(mask, mask, gpu_mat_ptr_->channels() == 3 ? cv::COLOR_GRAY2BGR : cv::COLOR_BGR2GRAY);
    }

    cv::cuda::bitwise_and(*gpu_mat_ptr_, mask, *gpu_mat_ptr_);
}
#endif

bool Image::get_using_cuda() const noexcept
{
    return using_cuda_;
}

const cv::Mat &Image::toMat() const
{
    if (using_cuda_)
    {
        std::lock_guard<std::mutex> lock(transfer_mutex_);
        gpu_mat_ptr_->download(mat_);
    }

    return mat_;
}

const cv::cuda::GpuMat &Image::toCudaMat() const
{
    if (!gpu_mat_ptr_)
        gpu_mat_ptr_ = std::make_unique<cv::cuda::GpuMat>();

    if (!using_cuda_)
    {
        std::lock_guard<std::mutex> lock(transfer_mutex_);
        gpu_mat_ptr_->upload(mat_);
    }

    return *gpu_mat_ptr_;
}

cv::Mat &Image::toMat()
{
    if (using_cuda_)
    {
        std::lock_guard<std::mutex> lock(transfer_mutex_);
        gpu_mat_ptr_->download(mat_);
    }

    return mat_;
}

cv::cuda::GpuMat &Image::toCudaMat()
{
    if (!gpu_mat_ptr_)
        gpu_mat_ptr_ = std::make_unique<cv::cuda::GpuMat>();

    if (!using_cuda_)
    {
        std::lock_guard<std::mutex> lock(transfer_mutex_);
        gpu_mat_ptr_->upload(mat_);
    }

    return *gpu_mat_ptr_;
}

void Image::download()
{
    if (using_cuda_)
    {
        std::lock_guard<std::mutex> lock(transfer_mutex_);
        gpu_mat_ptr_->download(mat_);
    }
}

void Image::upload()
{
    if (using_cuda_)
    {
        std::lock_guard<std::mutex> lock(transfer_mutex_);
        gpu_mat_ptr_->upload(mat_);
    }
}
