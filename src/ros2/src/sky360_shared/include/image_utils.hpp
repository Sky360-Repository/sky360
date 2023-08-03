#pragma once
#ifndef __IMAGE_UTILS_H__
#define __IMAGE_UTILS_H__

#include <opencv2/opencv.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>

class ImageUtils 
{
public:
    static void convert_image_msg(const sensor_msgs::msg::Image::SharedPtr image_msg, cv::Mat &_image_out)
    {
        auto bayer_img = cv_bridge::toCvShare(image_msg)->image;
        debayer_image(bayer_img, _image_out, image_msg->encoding);
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