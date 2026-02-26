//// filepath: jpeg_compressor.hpp
#pragma once

#include <opencv2/opencv.hpp>
#include <turbojpeg.h>
#include <vector>

namespace boblib::utils
{
    class JpegCompressor final
    {
    public:
        JpegCompressor(int quality = 75, TJSAMP subsampling = TJSAMP_420) noexcept;
        ~JpegCompressor() noexcept;

        JpegCompressor(const JpegCompressor&) = delete;
        JpegCompressor& operator=(const JpegCompressor&) = delete;
        JpegCompressor(JpegCompressor&&) = delete;
        JpegCompressor& operator=(JpegCompressor&&) = delete;

        /// encode `img` into `out_buffer` as JPEG
        /// returns true on success, false on failure
        bool compress(const cv::Mat &img, std::vector<uint8_t> &out_buffer) noexcept;

    private:
        tjhandle handle_;
        int quality_;
        TJSAMP subsampling_;

        std::unique_ptr<uint8_t[]> jpeg_buffer_;
        unsigned long jpeg_max_size_;
    };
}