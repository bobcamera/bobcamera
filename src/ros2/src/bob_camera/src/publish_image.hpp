#pragma once

#include <memory>
#include <rcpputils/endian.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <boblib/api/base/Image.hpp>

struct PublishImage
{
    using ImagePtr = std::shared_ptr<boblib::base::Image>;
    using HeaderPtr = std::shared_ptr<const std_msgs::msg::Header>;

    PublishImage() = default;

    PublishImage(HeaderPtr header,
                 ImagePtr image)
        : headerPtr(std::move(header))
        , imagePtr(std::move(image))
    {
    }

    HeaderPtr headerPtr;
    ImagePtr imagePtr;

    // ------------------------------------------------------------------
    //  Helper to fill a ROS Image message header from our Image
    // ------------------------------------------------------------------
    static sensor_msgs::msg::Image fill_imagemsg_header(
        const std_msgs::msg::Header &header,
        const boblib::base::Image &camera_img)
    {
        sensor_msgs::msg::Image msg;
        msg.header = header;
        msg.height = camera_img.size().height;
        msg.width = camera_img.size().width;
        msg.encoding = ImageUtils::type_to_encoding(camera_img.type());
        msg.is_bigendian = (rcpputils::endian::native == rcpputils::endian::big);
        msg.step = camera_img.size().width * camera_img.elemSize();
        return msg;
    }
};
