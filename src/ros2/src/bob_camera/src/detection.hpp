#pragma once

#include <memory>
#include <rcpputils/endian.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <boblib/api/base/Image.hpp>

struct Detection
{
    using HeaderPtr = std::shared_ptr<std_msgs::msg::Header>;
    using BboxPtr = std::shared_ptr<std::vector<cv::Rect>>;

    Detection() = default;

    Detection(HeaderPtr header,
              BboxPtr bbox,
              int32_t image_width,
              int32_t image_height,
              float fps)
        : header_ptr(std::move(header))
        , bbox_ptr(std::move(bbox))
        , image_width(image_width)
        , image_height(image_height)
        , fps(fps)
    {
    }

    HeaderPtr header_ptr;
    BboxPtr bbox_ptr;
    int32_t image_width;
    int32_t image_height;
    float fps;
};
