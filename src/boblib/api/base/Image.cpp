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

Image::Image(const Image &img) noexcept
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
        mat_ptr_ = std::make_unique<cv::Mat>(img.mat_ptr_->clone());
    }
}

Image::Image(Image &&img) noexcept
    : using_cuda_(img.using_cuda_),
      gpu_mat_ptr_(std::move(img.gpu_mat_ptr_)),
      mat_ptr_(std::move(img.mat_ptr_)),
      median_filter_size_(std::exchange(img.median_filter_size_, -1))
{
#ifdef HAVE_CUDA
    median_filter_ = std::move(img.median_filter_);
#endif
    // Reseting the Moved img
    img.reset();
}

Image::Image(const cv::Mat &image) noexcept
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
    mat_ptr_ = std::make_unique<cv::Mat>();
#ifdef HAVE_CUDA
    median_filter_.reset();
#endif
    median_filter_size_ = -1;
}

Image &Image::operator=(const Image &img) noexcept
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
        mat_ptr_ = std::make_unique<cv::Mat>();
    } 
    else
    {
        if (!mat_ptr_) 
        {
            mat_ptr_ = std::make_unique<cv::Mat>();
        }
        img.mat_ptr_->copyTo(*mat_ptr_);
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
    mat_ptr_ = std::move(img.mat_ptr_);
#ifdef HAVE_CUDA
    median_filter_ = std::move(img.median_filter_);
#endif
    median_filter_size_ = std::exchange(img.median_filter_size_, -1);

    img.reset();

    return *this;
}

int Image::channels() const noexcept
{
    return using_cuda_ ? gpu_mat_ptr_->channels() : mat_ptr_->channels();
}

Image Image::clone() const noexcept
{
    Image copy(using_cuda_);
    if (using_cuda_)
    {
        gpu_mat_ptr_->copyTo(*copy.gpu_mat_ptr_);
    }
    else
    {
        mat_ptr_->copyTo(*copy.mat_ptr_);
    }
    return copy;
}

Image &Image::create(int rows, int cols, int type, void *data) noexcept
{
    if (using_cuda_)
    {
        gpu_mat_ptr_ = data != nullptr ? std::make_unique<cv::cuda::GpuMat>(rows, cols, type, data) : std::make_unique<cv::cuda::GpuMat>(rows, cols, type);
        mat_ptr_ = std::make_unique<cv::Mat>();    
        return *this;
    }
    size_t elemSize = CV_ELEM_SIZE(type);
    size_t alignedStep = cv::alignSize(cols * elemSize, 64); // Align to 64-byte boundary
    mat_ptr_ = data != nullptr ? std::make_unique<cv::Mat>(rows, cols, type, data) : std::make_unique<cv::Mat>(rows, cols, type, nullptr, alignedStep);
    gpu_mat_ptr_.reset();

    return *this;
}

Image &Image::create(cv::Size size, int type) noexcept
{
    mat_ptr_->release();
    if (using_cuda_)
    {
        gpu_mat_ptr_->release();
        gpu_mat_ptr_->create(size, type);
        return *this;
    }
    mat_ptr_->create(size, type);

    return *this;
}

Image &Image::create(const cv::Mat &image) noexcept
{
    if (using_cuda_)
    {
        gpu_mat_ptr_->upload(image);
        return *this;
    }

    *mat_ptr_ = image.clone();

    return *this;
}

size_t Image::elemSize() const noexcept
{
    return using_cuda_ ? gpu_mat_ptr_->elemSize() : mat_ptr_->elemSize();
}

size_t Image::elemSize1() const noexcept
{
    return using_cuda_ ? gpu_mat_ptr_->elemSize1() : mat_ptr_->elemSize1();
}

bool Image::empty() const noexcept
{
    return using_cuda_ ? gpu_mat_ptr_->empty() : mat_ptr_->empty();
}

void Image::release() noexcept
{
    if (using_cuda_)
    {
        gpu_mat_ptr_->release();
    }
    mat_ptr_->release();
}

cv::Size Image::size() const noexcept
{
    return using_cuda_ ? gpu_mat_ptr_->size() : mat_ptr_->size();
}

size_t Image::step1() const noexcept
{
    return using_cuda_ ? gpu_mat_ptr_->step1() : mat_ptr_->step1();
}

size_t Image::total() const noexcept
{
    return using_cuda_ ? gpu_mat_ptr_->size().area() : mat_ptr_->total();
}

int Image::type() const noexcept
{
    return using_cuda_ ? gpu_mat_ptr_->type() : mat_ptr_->type();
}

void Image::apply_mask(Image &_mask) noexcept
{
#ifdef HAVE_CUDA
    if (using_cuda_)
    {
        mask_cuda(*_mask.gpu_mat_ptr_);
        return;
    }
#endif

    mask(*_mask.mat_ptr_);
}

uint8_t *Image::data() const noexcept
{
    if (using_cuda_)
    {
        gpu_mat_ptr_->download(*mat_ptr_);
    }

    return mat_ptr_->data;
}

void Image::copyTo(Image &copy) const noexcept
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

        gpu_mat_ptr_->download(*mat_ptr_);
        mat_ptr_->copyTo(*copy.mat_ptr_);
        return;
    }
    
    if (copy.using_cuda_)
    {
        copy.gpu_mat_ptr_->upload(*mat_ptr_);
        return;
    }

    mat_ptr_->copyTo(*copy.mat_ptr_);
}

void Image::resizeTo(Image &resized, const cv::Size &size) const noexcept
{
#ifdef HAVE_CUDA
    if (using_cuda_)
    {
        if (resized.using_cuda_)
        {
            cv::cuda::resize(*gpu_mat_ptr_, *resized.gpu_mat_ptr_, size);
            return;
        }
        gpu_mat_ptr_->download(*mat_ptr_);
        cv::resize(*mat_ptr_, *resized.mat_ptr_, size);
        return;
    }
    
    if (resized.using_cuda_)
    {
        cv::resize(*mat_ptr_, *resized.mat_ptr_, size);
        resized.upload();
        return;
    }
#endif

    cv::resize(*mat_ptr_, *resized.mat_ptr_, size);
}

void Image::resize(const cv::Size &size) noexcept
{
#ifdef HAVE_CUDA
    if (using_cuda_)
    {
        cv::cuda::resize(*gpu_mat_ptr_, *gpu_mat_ptr_, size);
    }
    else
#endif
    {
        cv::resize(*mat_ptr_, *mat_ptr_, size);
    }
}

void Image::medianBlurTo(Image &blured, int size) const noexcept
{
#ifdef HAVE_CUDA
    if (using_cuda_)
    {
        if (blured.using_cuda_)
        {
            cv::Ptr<cv::cuda::Filter> medianFilter = cv::cuda::createMedianFilter(gpu_mat_ptr_->type(), gpu_mat_ptr_->type(), cv::Size(size, size));
            medianFilter->apply(*gpu_mat_ptr_, *blured.gpu_mat_ptr_);
            return;
        }

        gpu_mat_ptr_->download(*mat_ptr_);
        cv::medianBlur(*mat_ptr_, *blured.mat_ptr_, size);
        return;
    }

    if (blured.using_cuda_)
    {
        cv::medianBlur(*mat_ptr_, *blured.mat_ptr_, size);
        blured.upload();
        return;
    }
#endif
    cv::medianBlur(*mat_ptr_, *blured.mat_ptr_, size);
}

void Image::medianBlur(int size) noexcept
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
        cv::medianBlur(*mat_ptr_, *mat_ptr_, size);
    }
}

void Image::convertColorTo(Image &converted, int type) const noexcept
{
#ifdef HAVE_CUDA
    if (using_cuda_)    
    {
        if (converted.using_cuda_)
        {
            cv::cuda::cvtColor(*gpu_mat_ptr_, *converted.gpu_mat_ptr_, type);
            return;
        }

        gpu_mat_ptr_->download(*mat_ptr_);
        cv::cvtColor(*mat_ptr_, *converted.mat_ptr_, type);
        return;
    }

    if (converted.using_cuda_)
    {
        cv::cvtColor(*mat_ptr_, *converted.mat_ptr_, type);
        converted.upload();
        return;
    }
#endif

    cv::cvtColor(*mat_ptr_, *converted.mat_ptr_, type);
}

void Image::convertColor(int type) noexcept
{
#ifdef HAVE_CUDA
    if (using_cuda_)    
    {
        if (gpu_mat_ptr_->type() != type)
        {
            cv::cuda::cvtColor(*gpu_mat_ptr_, *gpu_mat_ptr_, type);
        }
    }
    else
#endif
    {
        if (mat_ptr_->type() != type)
        {
            cv::cvtColor(*mat_ptr_, *mat_ptr_, type);
        }
    }
}

void Image::mask(cv::Mat &mask) noexcept
{
    if (mat_ptr_->size() != mask.size())
    {
        cv::resize(mask, mask, mat_ptr_->size());
    }

    if (mat_ptr_->channels() != mask.channels())
    {
        cv::cvtColor(mask, mask, mat_ptr_->channels() == 3 ? cv::COLOR_GRAY2BGR : cv::COLOR_BGR2GRAY);
    }

    cv::bitwise_and(*mat_ptr_, mask, *mat_ptr_);
}

#ifdef HAVE_CUDA
void Image::mask_cuda(cv::cuda::GpuMat &mask) noexcept
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

const cv::Mat &Image::toMat() const noexcept
{
    if (using_cuda_)
    {
        gpu_mat_ptr_->download(*mat_ptr_);
    }

    return *mat_ptr_;
}

const cv::cuda::GpuMat &Image::toCudaMat() const noexcept
{
    if (!using_cuda_)
    {
        gpu_mat_ptr_->upload(*mat_ptr_);
    }

    return *gpu_mat_ptr_;
}

cv::Mat &Image::toMat() noexcept
{
    if (using_cuda_)
    {
        gpu_mat_ptr_->download(*mat_ptr_);
    }

    return *mat_ptr_;
}

cv::cuda::GpuMat &Image::toCudaMat() noexcept
{
    if (!using_cuda_)
    {
        gpu_mat_ptr_->upload(*mat_ptr_);
    }

    return *gpu_mat_ptr_;
}

void Image::download() noexcept
{
    if (using_cuda_)
    {
        gpu_mat_ptr_->download(*mat_ptr_);
    }
}

void Image::upload() noexcept
{
    if (using_cuda_)
    {
        gpu_mat_ptr_->upload(*mat_ptr_);
    }
}
