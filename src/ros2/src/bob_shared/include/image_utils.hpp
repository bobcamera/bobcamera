#pragma once

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
};

class RosCvImageMsg
{
public:
    RosCvImageMsg(const cv::Mat & image, bool copy_img)
    {
        create_image_msg(image, copy_img);
    }

    RosCvImageMsg(int width, int height, int type)
    {
        create_image_msg(width, height, type);
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
        const size_t totalBytes = image_ptr_->total() * image_ptr_->elemSize();
        msg_ptr_->data.assign(image_ptr_->data, image_ptr_->data + totalBytes);;
        return *msg_ptr_;
    }

    bool recreate_if_invalid()
    {
        if ((image_msg_size_ != image_ptr_->size()) || (image_type_ != image_ptr_->type()))
        {
            auto matCopy = image_ptr_->clone();
            create_image_msg(matCopy, true);
            return true;
        }

        return false;
    }

    static std::unique_ptr<RosCvImageMsg> create(cv::VideoCapture & video_capture)
    {
        auto image_temp_ptr = std::make_unique<cv::Mat>();
        video_capture.read(*image_temp_ptr);
        auto roscv_image_msg_ptr = std::make_unique<RosCvImageMsg>(*image_temp_ptr, true);
        image_temp_ptr = nullptr;

        return roscv_image_msg_ptr;
    }

    static std::unique_ptr<RosCvImageMsg> create(int width, int height, int type)
    {
        return std::make_unique<RosCvImageMsg>(width, height, type);
    }

private:
    sensor_msgs::msg::Image::SharedPtr msg_ptr_;
    std::unique_ptr<cv::Mat> image_ptr_;
    cv::Size image_msg_size_;
    int image_type_;

    inline void create_image_msg(const cv::Mat & image, bool copy_img)
    {
        image_msg_size_ = image.size();
        image_type_ = image.type();
        msg_ptr_ = std::make_shared<sensor_msgs::msg::Image>();

        msg_ptr_->height = image.size().height;
        msg_ptr_->width = image.size().width;
        msg_ptr_->encoding = type_to_encoding(image_type_);
        msg_ptr_->is_bigendian = (rcpputils::endian::native == rcpputils::endian::big);
        msg_ptr_->step = image.size().width * image.elemSize();
        image_ptr_ = std::make_unique<cv::Mat>(image_msg_size_, image_type_);
        if (copy_img)
        {
            image.copyTo(*image_ptr_);
        }
    }

    inline void create_image_msg(int width, int height, int type)
    {
        image_msg_size_ = cv::Size(width, height);
        image_type_ = type;
        msg_ptr_ = std::make_shared<sensor_msgs::msg::Image>();

        msg_ptr_->height = height;
        msg_ptr_->width = width;
        msg_ptr_->encoding = type_to_encoding(type);
        msg_ptr_->is_bigendian = (rcpputils::endian::native == rcpputils::endian::big);
        msg_ptr_->step = width * calculate_elem_size1(type) * get_number_channels(type);
        image_ptr_ = std::make_unique<cv::Mat>(image_msg_size_, type);
    }
    
    static size_t calculate_elem_size1(int type) 
    {
        const int depth = type & CV_MAT_DEPTH_MASK; // Extract depth (first 3 bits)
        switch (depth) 
        {
            case CV_8U:
            case CV_8S:
                return 1;  // 8 bits / 8 = 1 byte
            case CV_16U:
            case CV_16S:
                return 2;  // 16 bits / 8 = 2 bytes
            case CV_32S:
            case CV_32F:
                return 4;  // 32 bits / 8 = 4 bytes
            case CV_64F:
                return 8;  // 64 bits / 8 = 8 bytes
            default:
                throw std::invalid_argument("Unknown depth value");
        }
    }

    static bool get_number_channels(int type) 
    {
        return 1 + (type >> CV_CN_SHIFT);
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
