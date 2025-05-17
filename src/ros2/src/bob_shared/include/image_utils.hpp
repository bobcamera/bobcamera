#pragma once

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <rcpputils/endian.hpp>
#include <rcpputils/endian.hpp>
#include <rclcpp/rclcpp.hpp>

#include "../base/Image.hpp"

class ImageUtils 
{
public:
    static void convert_image_msg(const sensor_msgs::msg::Image::SharedPtr image_msg, cv::Mat &_image_out, bool debayer, bool copy = false)
    {
        if (!copy)
        {
            debayer_image(cv_bridge::toCvShare(image_msg)->image, _image_out, image_msg->encoding, debayer);
        }
        else
        {
            debayer_image(cv_bridge::toCvCopy(image_msg)->image, _image_out, image_msg->encoding, debayer);
        }
    }

    static inline int convert_bayer_pattern(const std::string& _bayerFormat)
    {
        if (_bayerFormat == sensor_msgs::image_encodings::BAYER_GBRG8)
        {
            return cv::COLOR_BayerGR2BGR; //!< equivalent to GBRG Bayer pattern
        }
        else if (_bayerFormat == sensor_msgs::image_encodings::BAYER_GRBG8)
        {
            return cv::COLOR_BayerGB2BGR; //!< equivalent to GRBG Bayer pattern
        }
        else if (_bayerFormat == sensor_msgs::image_encodings::BAYER_BGGR8)
        {
            return cv::COLOR_BayerRG2BGR; //!< equivalent to BGGR Bayer pattern
        }
        else if (_bayerFormat == sensor_msgs::image_encodings::BAYER_RGGB8)
        {
            return cv::COLOR_BayerBG2BGR; //!< equivalent to RGGB Bayer pattern
        }
        return cv::COLOR_BayerGR2BGR;
    }

    static void debayer_image(const cv::Mat & _image_in, cv::Mat & _image_out, const std::string & _bayerFormatStr, bool debayer)
    {
        if (debayer 
            && _bayerFormatStr != sensor_msgs::image_encodings::BGR8 
            && _bayerFormatStr != sensor_msgs::image_encodings::MONO8)
        {
            cv::cvtColor(_image_in, _image_out, convert_bayer_pattern(_bayerFormatStr));
        }
        else
        {
            _image_out = _image_in;
        }
    }

    static const std::string type_to_encoding(int type)
    {
        switch (type) 
        {
            case CV_8UC1:
                return sensor_msgs::image_encodings::MONO8;
            case CV_8UC3:
                return sensor_msgs::image_encodings::BGR8;
            case CV_8UC4:
                return sensor_msgs::image_encodings::BGRA8;
            case CV_16UC1:
                return sensor_msgs::image_encodings::MONO16;
            case CV_16UC3:
                return sensor_msgs::image_encodings::BGR16;
            case CV_16UC4:
                return sensor_msgs::image_encodings::BGRA16;
            // Bayer patterns
            case CV_BayerGR2BGR:
                return sensor_msgs::image_encodings::BAYER_GBRG8;
            case CV_BayerGB2BGR:
                return sensor_msgs::image_encodings::BAYER_GRBG8;
            case CV_BayerRG2BGR:
                return sensor_msgs::image_encodings::BAYER_BGGR8;
            case CV_BayerBG2BGR:
                return sensor_msgs::image_encodings::BAYER_RGGB8;
            default:
                throw std::runtime_error("Unsupported image format");
        }
    }

    // ------------------------------------------------------------------
    //  Helper to fill a ROS Image message header from our Size and type
    // ------------------------------------------------------------------
    static void fill_imagemsg_header(
        sensor_msgs::msg::Image &msg, const std_msgs::msg::Header &header, const cv::Size &img_size, const int type)
    {
        msg.header = header;
        msg.height = img_size.height;
        msg.width = img_size.width;
        msg.encoding = ImageUtils::type_to_encoding(type);
        msg.is_bigendian = (rcpputils::endian::native == rcpputils::endian::big);
        msg.step = img_size.width * CV_ELEM_SIZE(type);
    }

    // ------------------------------------------------------------------
    //  Helper to fill a ROS Image message header from our Image
    // ------------------------------------------------------------------
    static void fill_imagemsg_header(
        sensor_msgs::msg::Image &msg,
        const std_msgs::msg::Header &header,
        const boblib::base::Image &camera_img)
    {
        msg.header = header;
        msg.height = camera_img.size().height;
        msg.width = camera_img.size().width;
        msg.encoding = ImageUtils::type_to_encoding(camera_img.type());
        msg.is_bigendian = (rcpputils::endian::native == rcpputils::endian::big);
        msg.step = camera_img.size().width * camera_img.elemSize();
    }
};
