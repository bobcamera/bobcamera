#pragma once
#ifndef __IMAGE_UTILS_H__
#define __IMAGE_UTILS_H__

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <rcpputils/endian.hpp>
#include <rcpputils/endian.hpp>
#include <rclcpp/rclcpp.hpp>

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
    RosCvImageMsg(const cv::Mat & image, const std::string & encoding, bool copy_img)
    {
        CreateImageMsg(image, encoding, copy_img);
    }

    std_msgs::msg::Header & get_header() const
    {
        return msg_ptr_->header;
    }

    void set_header(const std_msgs::msg::Header & header)
    {
        msg_ptr_->header = header;
    }

    void set_header(const rclcpp::Time & time, const std::string & id)
    {
        msg_ptr_->header.stamp = time;
        msg_ptr_->header.frame_id = id;
    }
    
    cv::Mat & get_image() const
    {
        return *image_ptr_;
    }

    sensor_msgs::msg::Image & get_msg() const
    {
        return *msg_ptr_;
    }

    bool recreate_if_invalid()
    {
        if (image_msg_size_ != image_ptr_->size())
        {
            auto matCopy = image_ptr_->clone();
            CreateImageMsg(matCopy, msg_ptr_->encoding, true);
            return true;
        }

        return false;
    }

    static std::unique_ptr<RosCvImageMsg> create(cv::VideoCapture & video_capture)
    {
        auto image_temp_ptr = std::make_unique<cv::Mat>();
        video_capture.read(*image_temp_ptr);
        auto roscv_image_msg_ptr = std::make_unique<RosCvImageMsg>(*image_temp_ptr, image_temp_ptr->channels() == 1 ? sensor_msgs::image_encodings::MONO8 : sensor_msgs::image_encodings::BGR8, true);
        image_temp_ptr = nullptr;

        return roscv_image_msg_ptr;
    }

private:
    sensor_msgs::msg::Image::SharedPtr msg_ptr_;
    std::unique_ptr<cv::Mat> image_ptr_;
    cv::Size image_msg_size_;

    inline void CreateImageMsg(const cv::Mat & image, const std::string & encoding, bool copy_img)
    {
        image_msg_size_ = image.size();
        msg_ptr_ = std::make_shared<sensor_msgs::msg::Image>();

        msg_ptr_->height = image.size().height;
        msg_ptr_->width = image.size().width;
        msg_ptr_->encoding = encoding;
        msg_ptr_->is_bigendian = (rcpputils::endian::native == rcpputils::endian::big);
        msg_ptr_->step = image.size().width * image.elemSize();
        const size_t size = msg_ptr_->step * image.size().height;
        msg_ptr_->data.resize(size);
        // TODO: image.type() convert it to take into account the encoding we desire
        image_ptr_ = std::make_unique<cv::Mat>(image.size(), image.type(), reinterpret_cast<void *>(&msg_ptr_->data[0]));
        if (copy_img)
        {
            image.copyTo(*image_ptr_);
        }
    }

};

#endif