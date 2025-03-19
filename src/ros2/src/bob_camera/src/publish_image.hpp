#pragma once

#include <boblib/api/base/Image.hpp>
#include <sensor_msgs/msg/image.hpp>

struct PublishImage
{
    PublishImage(sensor_msgs::msg::Image &&_Image_msg, boblib::base::Image &&_Image)
        : Image_msg(std::move(_Image_msg)), Image(std::move(_Image))
    {
    }

    sensor_msgs::msg::Image Image_msg;
    boblib::base::Image Image;
};
