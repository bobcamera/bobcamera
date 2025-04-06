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


Image::Image(const Image & img) noexcept
    : using_cuda_(img.using_cuda_),
      box_filter_size_(img.box_filter_size_)
{
    if (using_cuda_) 
    {
#ifdef HAVE_CUDA
        box_filter_ = img.box_filter_;
#endif
        gpu_mat_ptr_ = std::make_unique<cv::cuda::GpuMat>();
        img.gpu_mat_ptr_->copyTo(*gpu_mat_ptr_);
    } 
    else
    {
        mat_ptr_ = std::make_unique<cv::Mat>(img.mat_ptr_->clone());
    }
}

Image::Image(Image && img) noexcept
    : using_cuda_(img.using_cuda_),
      gpu_mat_ptr_(std::move(img.gpu_mat_ptr_)),
      mat_ptr_(std::move(img.mat_ptr_)),
      box_filter_size_(std::exchange(img.box_filter_size_, -1))
{
#ifdef HAVE_CUDA
    box_filter_ = std::move(img.box_filter_);
#endif
    // Reseting the Moved img
    img.reset();
}

Image::~Image()
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
    box_filter_.reset();
#endif
    box_filter_size_ = -1;
}

Image & Image::operator=(const Image & img)
{
    if (this == &img) 
    {
        return *this;
    }

    using_cuda_ = img.using_cuda_;
    box_filter_size_ = img.box_filter_size_;
#ifdef HAVE_CUDA
    box_filter_ = img.box_filter_; 
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
    box_filter_ = std::move(img.box_filter_);
#endif
    box_filter_size_ = std::exchange(img.box_filter_size_, -1);

    img.reset();

    return *this;
}

int Image::channels() const noexcept
{
    return using_cuda_ ? gpu_mat_ptr_->channels() : mat_ptr_->channels();
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
        mat_ptr_->copyTo(*copy.mat_ptr_);
    }
    return copy;
}

Image & Image::create(int rows, int cols, int type, void* data)
{
    if (using_cuda_)
    {
        gpu_mat_ptr_ = data != nullptr ? std::make_unique<cv::cuda::GpuMat>(rows, cols, type, data) : std::make_unique<cv::cuda::GpuMat>(rows, cols, type);
        mat_ptr_ = std::make_unique<cv::Mat>();    
        return *this;
    }
    mat_ptr_ = data != nullptr ? std::make_unique<cv::Mat>(rows, cols, type, data) : std::make_unique<cv::Mat>(rows, cols, type);
    gpu_mat_ptr_.reset();

    return *this;
}

Image & Image::create(cv::Size size, int type)
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

Image & Image::create(const cv::Mat & image)
{
    if (using_cuda_)
    {
        gpu_mat_ptr_->upload(image);
        return *this;
    }

    *mat_ptr_ = image.clone();

    return *this;
}

size_t Image::elemSize() const
{
    return using_cuda_ ? gpu_mat_ptr_->elemSize() : mat_ptr_->elemSize();
}

size_t Image::elemSize1() const
{
    return using_cuda_ ? gpu_mat_ptr_->elemSize1() : mat_ptr_->elemSize1();
}

bool Image::empty() const
{
    return using_cuda_ ? gpu_mat_ptr_->empty() : mat_ptr_->empty();
}

void Image::release()
{
    if (using_cuda_)
    {
        gpu_mat_ptr_->release();
    }
    mat_ptr_->release();
}

cv::Size Image::size() const
{
    return using_cuda_ ? gpu_mat_ptr_->size() : mat_ptr_->size();
}

size_t Image::step1() const
{
    return using_cuda_ ? gpu_mat_ptr_->step1() : mat_ptr_->step1();
}

size_t Image::total() const
{
    return using_cuda_ ? gpu_mat_ptr_->size().area() : mat_ptr_->total();
}

int Image::type() const
{
    return using_cuda_ ? gpu_mat_ptr_->type() : mat_ptr_->type();
}

void Image::apply_mask(Image & _mask)
{
    if (using_cuda_)
    {
        mask_cuda(*_mask.gpu_mat_ptr_);
        return;
    }

    mask(*_mask.mat_ptr_);
}

uint8_t* Image::data() const
{
    if (using_cuda_)
    {
        gpu_mat_ptr_->download(*mat_ptr_);
    }

    return mat_ptr_->data;
}

void Image::copyTo(Image & copy) const
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

void Image::resizeTo(Image & resized, const cv::Size & size) const
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

void Image::resize(const cv::Size & size)
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

void Image::medianBlurTo(Image & blured, int size) const
{
#ifdef HAVE_CUDA
    if (using_cuda_)
    {
        if (blured.using_cuda_)
        {
            cv::Ptr<cv::cuda::Filter> boxFilter = cv::cuda::createBoxFilter(gpu_mat_ptr_->type(), gpu_mat_ptr_->type(), cv::Size(size, size));
            boxFilter->apply(*gpu_mat_ptr_, *blured.gpu_mat_ptr_);
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

void Image::medianBlur(int size)
{
#ifdef HAVE_CUDA
    if (using_cuda_)
    {
        if (!box_filter_ || box_filter_size_ != size)
        {
            box_filter_ = cv::cuda::createBoxFilter(gpu_mat_ptr_->type(), gpu_mat_ptr_->type(), cv::Size(size, size));
            box_filter_size_ = size;
        }
        box_filter_->apply(*gpu_mat_ptr_, *gpu_mat_ptr_);
    }
    else
#endif
    {
        cv::medianBlur(*mat_ptr_, *mat_ptr_, size);
    }
}

void Image::convertTo(Image & converted, int type) const
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

void Image::convert(int type)
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

void Image::mask(cv::Mat & mask)
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

void Image::mask_cuda(cv::cuda::GpuMat & mask)
{
#ifdef HAVE_CUDA
    if (gpu_mat_ptr_->size() != mask.size())
    {
        cv::cuda::resize(mask, mask, gpu_mat_ptr_->size());
    }

    if (gpu_mat_ptr_->channels() != mask.channels())
    {
        cv::cuda::cvtColor(mask, mask, gpu_mat_ptr_->channels() == 3 ? cv::COLOR_GRAY2BGR : cv::COLOR_BGR2GRAY);
    }

    cv::cuda::bitwise_and(*gpu_mat_ptr_, mask, *gpu_mat_ptr_);
#endif
}

bool Image::get_using_cuda() const
{
    return using_cuda_;
}

const cv::Mat & Image::toMat() const
{
    if (using_cuda_)
    {
        gpu_mat_ptr_->download(*mat_ptr_);
    }

    return *mat_ptr_;
}

const cv::cuda::GpuMat & Image::toCudaMat() const
{
    if (!using_cuda_)
    {
        gpu_mat_ptr_->upload(*mat_ptr_);
    }

    return *gpu_mat_ptr_;
}

cv::Mat & Image::toMat()
{
    if (using_cuda_)
    {
        gpu_mat_ptr_->download(*mat_ptr_);
    }

    return *mat_ptr_;
}

cv::cuda::GpuMat & Image::toCudaMat()
{
    if (!using_cuda_)
    {
        gpu_mat_ptr_->upload(*mat_ptr_);
    }

    return *gpu_mat_ptr_;
}

void Image::download()
{
    if (using_cuda_)
    {
        gpu_mat_ptr_->download(*mat_ptr_);
    }
}

void Image::upload()
{
    if (using_cuda_)
    {
        gpu_mat_ptr_->upload(*mat_ptr_);
    }
}
