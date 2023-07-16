#pragma once

#include <opencv2/opencv.hpp>

namespace sky360lib::utils
{
    class EntropyEstimator
    {
    public:
        float estimate_entropy(const cv::Mat& subSampled)
        {
            cv::Mat hist;
            const int hist_size = 256;

            // Compute the histograms:
            const float range[] = {0, subSampled.elemSize1() == 1 ? 255.0f : 65535.0f};
            const float* hist_range = {range};
            cv::calcHist(&subSampled, 1, 0, cv::Mat(), hist, 1, &hist_size, &hist_range, true, false);

            // Compute entropy
            double entropy_value = 0;
            const double total_size = subSampled.rows * subSampled.cols; 

            float* sym_occur = hist.ptr<float>(0); 
            for (int i = 0; i < hist_size; ++i)
            {
                if (sym_occur[i] > 0) // Log of zero goes to infinity
                {
                    entropy_value += ((double)sym_occur[i] / total_size) * (std::log2(total_size / (double)sym_occur[i]));
                }
            }

            entropy_value /= 8.0; // The max entropy for an 8-bit grayscale image is 8, so needs to be adjusted for 16

            hist.release();

            return entropy_value;
        }
    };
}
