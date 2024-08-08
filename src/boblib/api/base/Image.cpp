#include "Image.hpp"

#include <opencv2/cudawarping.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudaarithm.hpp>

using namespace boblib::base;

Image::Image(bool use_cuda)
: using_cuda_(use_cuda ? cv::cuda::getCudaEnabledDeviceCount() : false)
{
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
    if (using_cuda_)
    {
        gpu_mat_.release();
        return;
    }
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

void Image::mask(cv::Mat & mask)
{
    // Resize the mask if sizes do not match
    if (mat_.size() != mask.size())
    {
        cv::resize(mask, mask, mat_.size());
    }

    // Convert channels if they do not match
    if (mat_.channels() != mask.channels())
    {
        if (mat_.channels() != mask.channels())
        {
            cv::cvtColor(mask, mask, mat_.channels() == 3 ? cv::COLOR_GRAY2BGR : cv::COLOR_BGR2GRAY);
        }
    }

    // Perform bitwise AND operation
    cv::cuda::bitwise_and(mat_, mask, mat_);
}

void Image::mask_cuda(cv::cuda::GpuMat & mask)
{
    if (gpu_mat_.size() != mask.size())
    {
        cv::cuda::resize(mask, mask, gpu_mat_.size());
    }

    if (gpu_mat_.channels() != mask.channels())
    {
        if (gpu_mat_.channels() != mask.channels())
        {
            cv::cuda::cvtColor(mask, mask, gpu_mat_.channels() == 3 ? cv::COLOR_GRAY2BGR : cv::COLOR_BGR2GRAY);
        }
    }

    cv::cuda::bitwise_and(gpu_mat_, mask, gpu_mat_);
}