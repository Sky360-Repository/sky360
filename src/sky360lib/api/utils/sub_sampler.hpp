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
            const double step_row = static_cast<double>(input.rows) / output.rows;
            const double step_col = static_cast<double>(input.cols) / output.cols;

            for (int i = 0; i < output.rows; ++i)
            {
                for (int j = 0; j < output.cols; ++j)
                {
                    const int row = static_cast<int>(i * step_row);
                    const int col = static_cast<int>(j * step_col);
                    output.at<T>(i, j) = input.at<T>(row, col);
                }
            }
        }

        template <typename T>
        void subSampleImplRand(const cv::Mat& input, cv::Mat& output)
        {
            const int rows = input.rows;
            const int cols = input.cols;
            for (int i = 0; i < output.rows; ++i)
            {
                for (int j = 0; j < output.cols; ++j)
                {
                    const int row = m_random.uniform(0, rows);
                    const int col = m_random.uniform(0, cols);
                    output.at<T>(i, j) = input.at<T>(row, col);
                }
            }
        }

        cv::Mat subSample(const cv::Mat& image)
        {
            cv::Mat subSampled(m_m, m_n, image.type());

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
        cv::RNG m_random;
        int m_n;
        int m_m;
    };
}