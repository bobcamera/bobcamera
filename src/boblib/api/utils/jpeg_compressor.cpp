//// filepath: jpeg_compressor.cpp
#include "jpeg_compressor.hpp"

namespace boblib::utils
{

    JpegCompressor::JpegCompressor(int quality, TJSAMP subsampling) noexcept
        : handle_(tjInitCompress()), quality_(quality), subsampling_(subsampling), jpeg_max_size_(0)
    {
    }

    JpegCompressor::~JpegCompressor() noexcept
    {
        if (handle_)
        {
            tjDestroy(handle_);
        }
    }

    bool JpegCompressor::compress(const cv::Mat &img,
                                  std::vector<uint8_t> &out_buffer)
    {
        if (!handle_ || img.empty())
        {
            return false;
        }

        // Detect pixel format
        int pf;
        switch (img.channels())
        {
        case 1:
            pf = TJPF_GRAY;
            break;
        case 3:
            pf = TJPF_BGR;
            break;
        case 4:
            pf = TJPF_BGRA;
            break;
        default:
            return false;
        }

        // Calculate the maximum buffer size needed
        unsigned long max_size = tjBufSize(img.cols, img.rows, subsampling_);
        if (max_size > jpeg_max_size_)
        {
            jpeg_buffer_ = std::make_unique<uint8_t[]>(max_size);
            jpeg_max_size_ = max_size;
        }

        unsigned long jpeg_size = max_size;
        unsigned char *jpeg_buf = jpeg_buffer_.get();

        // Use TJFLAG_NOREALLOC since we've pre-allocated the buffer
        int ret = tjCompress2(
            handle_,
            img.data,
            img.cols,
            0,
            img.rows,
            pf,
            &jpeg_buf,
            &jpeg_size,
            subsampling_,
            quality_,
            TJFLAG_FASTDCT | TJFLAG_NOREALLOC);

        if (ret != 0)
        {
            return false;
        }

        out_buffer.resize(jpeg_size);
        std::memcpy(out_buffer.data(), jpeg_buffer_.get(), jpeg_size);

        return true;
    }
} // namespace boblib::utils