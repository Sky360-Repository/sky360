#pragma once
#ifndef __IMAGE_UTILS_H__
#define __IMAGE_UTILS_H__

#include <opencv2/opencv.hpp>

#include <sensor_msgs/msg/image.hpp>

class ImageUtils 
{
public:
    static void convert_image_msg(const sensor_msgs::msg::Image::SharedPtr image_msg, cv::Mat &_image_out)
    {
        auto pattern = image_msg->encoding;
        image_msg->encoding = image_msg->encoding != sensor_msgs::image_encodings::BGR8 ? sensor_msgs::image_encodings::MONO8 : sensor_msgs::image_encodings::BGR8;
        auto bayer_img = cv_bridge::toCvShare(image_msg)->image;
        image_msg->encoding = pattern;
        debayer_image(bayer_img, _image_out, pattern);
    }

    static inline int convert_bayer_pattern(const std::string& _bayerFormat)
    {
        if (_bayerFormat == "BAYER_GB")
        {
            return cv::COLOR_BayerGR2BGR; //!< equivalent to GBRG Bayer pattern
        }
        else if (_bayerFormat == "BAYER_GR")
        {
            return cv::COLOR_BayerGB2BGR; //!< equivalent to GRBG Bayer pattern
        }
        else if (_bayerFormat == "BAYER_BG")
        {
            return cv::COLOR_BayerRG2BGR; //!< equivalent to GRBG Bayer pattern
        }
        else if (_bayerFormat == "BAYER_RG")
        {
            return cv::COLOR_BayerBG2BGR; //!< equivalent to GRBG Bayer pattern
        }
        return cv::COLOR_BayerGR2BGR;
    }

    static void debayer_image(const cv::Mat &_image_in, cv::Mat &_image_out, const std::string& _bayerFormatStr)
    {
        if (_bayerFormatStr != sensor_msgs::image_encodings::BGR8 
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

#endif