#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

namespace sky360lib::utils
{
    class BrightnessEstimator 
    {
    public:
        BrightnessEstimator(){}

        double estimateCurrentBrightness(const cv::Mat& samples_mat)
        {
            cv::Scalar result = cv::mean(samples_mat); 
            return result[0] * (samples_mat.elemSize1() == 1 ? MULT_8_BITS : MULT_16_BITS);
        }

    private:
        const double MULT_8_BITS = 1.0 / 255.0;
        const double MULT_16_BITS = 1.0 / 65535.0;
    };
}

