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
};

class RosCvImageMsg
{
public:
    RosCvImageMsg(const boblib::base::Image & image, bool copy_img)
    {
        create_image_msg(image, copy_img);
    }

    RosCvImageMsg(int width, int height, int type, bool use_cuda)
    {
        create_image_msg(width, height, type, use_cuda);
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
    
    boblib::base::Image & get_image() const
    {
        return *image_ptr_;
    }

    const sensor_msgs::msg::Image & get_msg()
    {
        if ((image_msg_size_ != image_ptr_->size()) || (image_type_ != image_ptr_->type()))
        {
            image_msg_size_ = image_ptr_->size();
            image_type_ = image_ptr_->type();
            msg_ptr_->height = image_ptr_->size().height;
            msg_ptr_->width = image_ptr_->size().width;
            msg_ptr_->encoding = ImageUtils::type_to_encoding(image_type_);
            msg_ptr_->is_bigendian = (rcpputils::endian::native == rcpputils::endian::big);
            msg_ptr_->step = image_ptr_->size().width * image_ptr_->elemSize();
        }
        const size_t totalBytes = image_ptr_->total() * image_ptr_->elemSize();
        msg_ptr_->data.assign(image_ptr_->data(), image_ptr_->data() + totalBytes);
        return *msg_ptr_;
    }

    static std::unique_ptr<RosCvImageMsg> create(int width, int height, int type, bool use_cuda)
    {
        return std::make_unique<RosCvImageMsg>(width, height, type, use_cuda);
    }

    static std::unique_ptr<RosCvImageMsg> create(const cv::Mat & img, bool copy_img, bool use_cuda)
    {
        auto img_msg = std::make_unique<RosCvImageMsg>(img.size().width, img.size().height, img.type(), use_cuda);
        if (copy_img)
        {
            img_msg->get_image().create(img);
        }
        return img_msg;
    }

private:
    sensor_msgs::msg::Image::SharedPtr msg_ptr_;
    std::unique_ptr<boblib::base::Image> image_ptr_;
    cv::Size image_msg_size_;
    int image_type_;

    inline void create_image_msg(const boblib::base::Image & image, bool copy_img)
    {
        image_msg_size_ = image.size();
        image_type_ = image.type();
        msg_ptr_ = std::make_shared<sensor_msgs::msg::Image>();

        image_ptr_ = std::make_unique<boblib::base::Image>(image.get_using_cuda());
        image_ptr_->create(image_msg_size_, image_type_);
        msg_ptr_->height = image.size().height;
        msg_ptr_->width = image.size().width;
        msg_ptr_->encoding = ImageUtils::type_to_encoding(image_type_);
        msg_ptr_->is_bigendian = (rcpputils::endian::native == rcpputils::endian::big);
        msg_ptr_->step = image.size().width * image.elemSize();
        if (copy_img)
        {
            image.copyTo(*image_ptr_);
        }
    }

    inline void create_image_msg(int width, int height, int type, bool use_cuda)
    {
        image_msg_size_ = cv::Size(width, height);
        image_type_ = type;
        msg_ptr_ = std::make_shared<sensor_msgs::msg::Image>();

        image_ptr_ = std::make_unique<boblib::base::Image>(use_cuda);
        image_ptr_->create(image_msg_size_, image_type_);
        msg_ptr_->height = height;
        msg_ptr_->width = width;
        msg_ptr_->encoding = ImageUtils::type_to_encoding(type);
        msg_ptr_->is_bigendian = (rcpputils::endian::native == rcpputils::endian::big);
        msg_ptr_->step = width * image_ptr_->elemSize();
    }
};
