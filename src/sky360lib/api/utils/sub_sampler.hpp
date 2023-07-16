#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

namespace sky360lib::utils
{
    class SubSampler 
    {
    public:
        SubSampler(int n, int m)
            : m_n(n), m_m(m) {}

        template <typename T>
        void subSampleImpl(const cv::Mat& input, cv::Mat& output)
        {
            for (int i = 0; i < output.rows; ++i)
            {
                for (int j = 0; j < output.cols; ++j)
                {
                    const int row = i * m_m;
                    const int col = j * m_n;
                    output.at<T>(i, j) = input.at<T>(row, col);
                }
            }
        }

        cv::Mat subSample(const cv::Mat& image)
        {
            const int rows = image.rows;
            const int cols = image.cols;

            cv::Mat subSampled(rows / m_m, cols / m_n, image.type());

            if (image.channels() == 1) // Grayscale image
            {
                switch (image.depth())
                {
                case CV_8U:
                    subSampleImpl<uchar>(image, subSampled);
                    break;

                case CV_16U:
                    subSampleImpl<ushort>(image, subSampled);
                    break;

                default:
                    // Handle unsupported image type/error condition
                    break;
                }
            }
            else if (image.channels() == 3) // Color image
            {
                switch (image.depth())
                {
                case CV_8U:
                    subSampleImpl<cv::Vec3b>(image, subSampled);
                    break;

                case CV_16U:
                    subSampleImpl<cv::Vec3w>(image, subSampled);
                    break;

                default:
                    // Handle unsupported image type/error condition
                    break;
                }
            }
            else
            {
                // Handle other number of channels
            }

            return subSampled;
        }

    private:
        int m_n;
        int m_m;
    };
}