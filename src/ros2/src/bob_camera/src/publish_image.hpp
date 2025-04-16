#pragma once

#include <boblib/api/base/Image.hpp>
#include <sensor_msgs/msg/image.hpp>

struct PublishImage
{
    PublishImage(const std_msgs::msg::Header & header, const boblib::base::Image && image)
        : Header(header), Image(std::move(image))
    {
    }

    const std_msgs::msg::Header Header;
    const boblib::base::Image Image;

    static sensor_msgs::msg::Image fill_imagemsg_header(const std_msgs::msg::Header &header, const boblib::base::Image &camera_img)
    {
        sensor_msgs::msg::Image camera_msg;
        camera_msg.header = header;
        camera_msg.height = camera_img.size().height;
        camera_msg.width = camera_img.size().width;
        camera_msg.encoding = ImageUtils::type_to_encoding(camera_img.type());
        camera_msg.is_bigendian = (rcpputils::endian::native == rcpputils::endian::big);
        camera_msg.step = camera_img.size().width * camera_img.elemSize();

        return camera_msg;
    }
};
