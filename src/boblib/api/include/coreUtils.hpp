#pragma once

#include <opencv2/core.hpp>

#include <cstddef>
#include <stdint.h>
#include <memory>
#include <cstring>
#include <thread>

#include "../base/Image.hpp"

namespace boblib
{
    struct ImgSize
    {
        ImgSize(const ImgSize &_img_size)
            : ImgSize(_img_size.width, _img_size.height, _img_size.num_channels, _img_size.bytes_per_channel, _img_size.original_pixel_pos)
        {
        }

        ImgSize(int _width, int _height, int _num_channels, int _bytes_per_pixel, size_t _original_pixel_pos)
            : width(_width)
            , height(_height)
            , num_channels(_num_channels)
            , bytes_per_channel(_bytes_per_pixel)
            , num_pixels(_width * _height)
            , size_in_bytes(_width * _height * _num_channels * _bytes_per_pixel)
            , original_pixel_pos{_original_pixel_pos}
        {
        }

        ImgSize(const boblib::base::Image & img)
            : ImgSize(img.size().width, img.size().height, img.channels(), img.elemSize1(), 0)
        {
        }

        ImgSize(const cv::Mat & _cv_mat)
            : ImgSize(_cv_mat.size().width, _cv_mat.size().height, _cv_mat.channels(), _cv_mat.elemSize1(), 0)
        {
        }

        static std::unique_ptr<ImgSize> create(int _width, int _height, int _num_channels, int _bytes_per_pixel, size_t _original_pixel_pos)
        {
            return std::make_unique<ImgSize>(_width, _height, _num_channels, _bytes_per_pixel, _original_pixel_pos);
        }

        static std::unique_ptr<ImgSize> create(const cv::Mat & cv_mat)
        {
            return std::make_unique<ImgSize>(cv_mat);
        }

        static std::unique_ptr<ImgSize> create(const boblib::base::Image & img)
        {
            return std::make_unique<ImgSize>(img);
        }

        bool operator==(const ImgSize &rhs) const
        {
            return width == rhs.width && height == rhs.height && num_channels == rhs.num_channels && bytes_per_channel == rhs.bytes_per_channel && original_pixel_pos == rhs.original_pixel_pos;
        }

        bool operator!=(const ImgSize &rhs) const
        {
            return !(*this == rhs);
        }

        bool empty() const
        {
            return width == 0 || height == 0;
        }

        const int width;
        const int height;
        const int num_channels;
        const int bytes_per_channel;
        const size_t num_pixels;
        const size_t size_in_bytes;

        const size_t original_pixel_pos;
    };

    struct Img
    {
        Img(uint8_t *_data, const ImgSize &_img_size, std::unique_ptr<uint8_t[]> _data_ptr = nullptr)
            : data{_data}, size{_img_size}, data_ptr{std::move(_data_ptr)}
        {
        }

        static std::unique_ptr<Img> create(const ImgSize &_img_size, bool _clear = false)
        {
            auto data = std::make_unique_for_overwrite<uint8_t[]>(_img_size.size_in_bytes);
            if (_clear)
            {
                memset(data.get(), 0, _img_size.size_in_bytes);
            }

            return std::make_unique<Img>(data.get(), _img_size, std::move(data));
        }

        inline void clear()
        {
            memset(data, 0, size.size_in_bytes);
        }

        inline bool empty() const
        {
            return size.empty();
        }

        uint8_t *const data;
        const ImgSize size;

        template <class T>
        inline T *ptr() { return (T *)data; }
        template <class T>
        inline const T *ptr() const { return (T *)data; }

        std::unique_ptr<uint8_t[]> data_ptr;
    };
}