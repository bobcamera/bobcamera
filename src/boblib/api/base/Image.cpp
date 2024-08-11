#include "Image.hpp"

#include <opencv2/cudawarping.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudaarithm.hpp>

using namespace boblib::base;

Image::Image(bool use_cuda)
: using_cuda_(use_cuda ? cv::cuda::getCudaEnabledDeviceCount() : false)
{
}

Image::~Image()
{
    gpu_mat_.release();
    mat_.release();
}

int Image::channels() const
{
    return using_cuda_ ? gpu_mat_.channels() : mat_.channels();
}

Image Image::clone() const
{
    Image copy(using_cuda_);
    if (using_cuda_)
    {
        gpu_mat_.copyTo(copy.gpu_mat_);
    }
    else
    {
        mat_.copyTo(copy.mat_);
    }
    return copy;
}

void Image::create(int rows, int cols, int type)
{
    if (using_cuda_)
    {
        gpu_mat_.create(rows, cols, type);
        return;
    }
    mat_.create(rows, cols, type);
}

void Image::create(cv::Size size, int type)
{
    if (using_cuda_)
    {
        gpu_mat_.create(size, type);
        return;
    }
    mat_.create(size, type);
}

void Image::create(const cv::Mat & image)
{
    if (using_cuda_)
    {
        gpu_mat_.upload(image);
        return;
    }

    mat_ = image.clone();
}

size_t Image::elemSize() const
{
    return using_cuda_ ? gpu_mat_.elemSize() : mat_.elemSize();
}

size_t Image::elemSize1() const
{
    return using_cuda_ ? gpu_mat_.elemSize1() : mat_.elemSize1();
}

bool Image::empty() const
{
    return using_cuda_ ? gpu_mat_.empty() : mat_.empty();
}

void Image::release()
{
    gpu_mat_.release();
    mat_.release();
}

cv::Size Image::size() const
{
    return using_cuda_ ? gpu_mat_.size() : mat_.size();
}

size_t Image::step1() const
{
    return using_cuda_ ? gpu_mat_.step1() : mat_.step1();
}

size_t Image::total() const
{
    return using_cuda_ ? gpu_mat_.size().area() : mat_.total();
}

int Image::type() const
{
    return using_cuda_ ? gpu_mat_.type() : mat_.type();
}

void Image::apply_mask(Image & _mask)
{
    if (using_cuda_)
    {
        mask_cuda(_mask.gpu_mat_);
        return;
    }

    mask(_mask.mat_);
}

uint8_t* Image::data() const
{
    if (using_cuda_)
    {
        gpu_mat_.download(mat_);
    }

    return mat_.data;
}

void Image::copyTo(Image & copy) const
{
    copy.using_cuda_ = using_cuda_;
    if (using_cuda_)
    {
        gpu_mat_.copyTo(copy.gpu_mat_);
    }
    else
    {
        mat_.copyTo(copy.mat_);
    }
}

void Image::resizeTo(Image & resized, const cv::Size & size) const
{
    resized.using_cuda_ = using_cuda_;
    if (using_cuda_)
    {
        cv::cuda::resize(gpu_mat_, resized.gpu_mat_, size);
    }
    else
    {
        cv::resize(mat_, resized.mat_, size);
    }
}

void Image::resize(const cv::Size & size)
{
    if (using_cuda_)
    {
        cv::cuda::resize(gpu_mat_, gpu_mat_, size);
    }
    else
    {
        cv::resize(mat_, mat_, size);
    }
}

void Image::medianBlurTo(Image & blured, int size) const
{
    blured.using_cuda_ = using_cuda_;
    if (using_cuda_)
    {
        cv::Ptr<cv::cuda::Filter> boxFilter = cv::cuda::createBoxFilter(gpu_mat_.type(), gpu_mat_.type(), cv::Size(size, size));
        boxFilter->apply(gpu_mat_, blured.gpu_mat_);
    }
    else
    {
        cv::medianBlur(mat_, blured.mat_, size);
    }
}

void Image::medianBlur(int size)
{
    if (using_cuda_)
    {
        if (!box_filter_ || box_filter_size_ != size)
        {
            box_filter_ = cv::cuda::createBoxFilter(gpu_mat_.type(), gpu_mat_.type(), cv::Size(size, size));
            box_filter_size_ = size;
        }
        box_filter_->apply(gpu_mat_, gpu_mat_);
    }
    else
    {
        cv::medianBlur(mat_, mat_, size);
    }
}

void Image::convertTo(Image & converted, int type) const
{
    converted.using_cuda_ = using_cuda_;
    if (using_cuda_)    
    {
        if (gpu_mat_.type() != type)
        {
            cv::cuda::cvtColor(gpu_mat_, converted.gpu_mat_, type);
        }
        else
        {
            gpu_mat_.copyTo(converted.gpu_mat_);
        }
    }
    else
    {
        if (mat_.type() != type)
        {
            cv::cvtColor(mat_, converted.mat_, type);
        }
        else
        {
            mat_.copyTo(converted.mat_);
        }
    }
}

void Image::convert(int type)
{
    if (using_cuda_)    
    {
        if (gpu_mat_.type() != type)
        {
            cv::cuda::cvtColor(gpu_mat_, gpu_mat_, type);
        }
    }
    else
    {
        if (mat_.type() != type)
        {
            cv::cvtColor(mat_, mat_, type);
        }
    }
}

void Image::mask(cv::Mat & mask)
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

void Image::mask_cuda(cv::cuda::GpuMat & mask)
{
    if (gpu_mat_.size() != mask.size())
    {
        cv::cuda::resize(mask, mask, gpu_mat_.size());
    }

    if (gpu_mat_.channels() != mask.channels())
    {
        cv::cuda::cvtColor(mask, mask, gpu_mat_.channels() == 3 ? cv::COLOR_GRAY2BGR : cv::COLOR_BGR2GRAY);
    }

    cv::cuda::bitwise_and(gpu_mat_, mask, gpu_mat_);
}

bool Image::get_using_cuda() const
{
    return using_cuda_;
}

cv::cuda::GpuMat & Image::get_cuda_mat()
{
    return gpu_mat_;
}

cv::Mat & Image::get_mat()
{
    return mat_;
}

const cv::cuda::GpuMat & Image::get_cuda_mat() const
{
    return gpu_mat_;
}

const cv::Mat & Image::get_mat() const
{
    return mat_;
}

const cv::Mat & Image::toMat() const
{
    if (using_cuda_)
    {
        gpu_mat_.download(mat_);
    }

    return mat_;
}

const cv::cuda::GpuMat & Image::toCudaMat() const
{
    if (using_cuda_)
    {
        gpu_mat_.upload(mat_);
    }

    return gpu_mat_;
}

cv::Mat & Image::download()
{
    if (using_cuda_)
    {
        gpu_mat_.download(mat_);
    }

    return mat_;
}

cv::cuda::GpuMat & Image::upload()
{
    if (using_cuda_)
    {
        gpu_mat_.upload(mat_);
    }

    return gpu_mat_;
}
