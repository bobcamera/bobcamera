#pragma once

#include <memory>
#include <rcpputils/endian.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <boblib/api/base/Image.hpp>

struct Detection
{
    using ImagePtr = std::shared_ptr<boblib::base::Image>;
    using HeaderPtr = std::shared_ptr<std_msgs::msg::Header>;
    using BboxPtr = std::shared_ptr<std::vector<cv::Rect>>;

    Detection() = default;

    Detection(HeaderPtr header,
              ImagePtr image,
              BboxPtr bbox,
              float fps)
        : header_ptr(std::move(header))
        , image_ptr(std::move(image))
        , bbox_ptr(std::move(bbox))
        , fps(fps)
    {
    }

    HeaderPtr header_ptr;
    ImagePtr image_ptr;
    BboxPtr bbox_ptr;
    float fps;
};
