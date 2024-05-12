#pragma once
#ifndef __IMAGE_UTILS_H__
#define __IMAGE_UTILS_H__

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <rcpputils/endian.hpp>

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

    static void debayer_image(const cv::Mat &_image_in, cv::Mat &_image_out, const std::string& _bayerFormatStr, bool debayer)
    {
        if (debayer 
            &&_bayerFormatStr != sensor_msgs::image_encodings::BGR8 
            && _bayerFormatStr != sensor_msgs::image_encodings::MONO8)
        {
            cv::cvtColor(_image_in, _image_out, convert_bayer_pattern(_bayerFormatStr));
        }
        else
        {
            _image_out = _image_in;
        }
    }
};

class RosCvImageMsg
{
public:
    sensor_msgs::msg::Image::SharedPtr msg_ptr;
    std::unique_ptr<cv::Mat> image_ptr;

    RosCvImageMsg(const cv::Mat & image, const std::string & encoding, bool copy_img)
    {
        CreateImageMsg(image, encoding, copy_img);
    }

    std_msgs::msg::Header & get_header() const
    {
        return msg_ptr->header;
    }

    static std::unique_ptr<RosCvImageMsg> create(cv::VideoCapture & video_capture)
    {
        auto image_temp_ptr = std::make_unique<cv::Mat>();
        video_capture.read(*image_temp_ptr);
        std::unique_ptr<RosCvImageMsg> roscv_image_msg_ptr = std::make_unique<RosCvImageMsg>(*image_temp_ptr, image_temp_ptr->channels() == 1 ? sensor_msgs::image_encodings::MONO8 : sensor_msgs::image_encodings::BGR8, true);
        image_temp_ptr = nullptr;

        return roscv_image_msg_ptr;
    }

private:
    void CreateImageMsg(const cv::Mat & image, const std::string & encoding, bool copy_img)
    {
        msg_ptr = std::make_shared<sensor_msgs::msg::Image>();

        msg_ptr->height = image.size().height;
        msg_ptr->width = image.size().width;
        msg_ptr->encoding = encoding;
        msg_ptr->is_bigendian = (rcpputils::endian::native == rcpputils::endian::big);
        msg_ptr->step = image.size().width * image.elemSize();
        const size_t size = msg_ptr->step * image.size().height;
        msg_ptr->data.resize(size);
        // TODO: image.type() convert it to take into account the encoding we desire
        image_ptr = std::make_unique<cv::Mat>(image.size(), image.type(), reinterpret_cast<void *>(&msg_ptr->data[0]));
        if (copy_img)
        {
            image.copyTo(*image_ptr);
        }
    }

};

#endif