#pragma once

#include <memory>
#include <rcpputils/endian.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <boblib/api/base/Image.hpp>

struct PublishImage
{
    using ImagePtr = std::shared_ptr<boblib::base::Image>;
    using HeaderPtr = std::shared_ptr<std_msgs::msg::Header>;
    using CameraInfoPtr = std::shared_ptr<bob_camera::msg::CameraInfo>;

    PublishImage() = default;

    PublishImage(HeaderPtr header,
                 ImagePtr image,
                 CameraInfoPtr camera_info)
        : header_ptr(std::move(header))
        , image_ptr(std::move(image))
        , camera_info_ptr(std::move(camera_info))
    {
    }

    HeaderPtr header_ptr;
    ImagePtr image_ptr;
    CameraInfoPtr camera_info_ptr;
};
