#pragma once

#include <sstream>
#include <iomanip>

#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

#include "ringbuf.h"
#include "textWriter.hpp"

namespace sky360lib::utils
{
    class Utils
    {
    public:
        static cv::Mat equalize_image(const cv::Mat &image_in, cv::Mat &image_out, double clip_limit, const cv::Size& grid_size)
        {
            if (image_in.channels() > 1)
            {
                cv::Mat lab_image;

                cv::cvtColor(image_in, lab_image, cv::COLOR_BGR2YCrCb);

                std::vector<cv::Mat> lab_channels(3);
                cv::split(lab_image, lab_channels);

                cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
                clahe->setClipLimit(clip_limit);
                clahe->setTilesGridSize(grid_size);
                cv::Mat equalized_l;
                clahe->apply(lab_channels[0], equalized_l);

                lab_channels[0] = equalized_l;
                cv::merge(lab_channels, lab_image);

                cv::cvtColor(lab_image, image_out, cv::COLOR_YCrCb2BGR);

                return equalized_l;
            }
            else
            {
                cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
                clahe->setClipLimit(clip_limit);
                clahe->setTilesGridSize(grid_size);
                clahe->apply(image_in, image_out);
                return image_out;
            }
        }

        // https://stackoverflow.com/questions/6123443/calculating-image-acutance/6129542#6129542
        static double estimate_sharpness(const cv::Mat &img)
        {
            if (img.empty())
            {
                return 0.0;
            }

            cv::Mat sharpness_img;
            if (img.channels() == 3)
            {
                cv::cvtColor(img, sharpness_img, cv::COLOR_BGR2GRAY);
            }
            else
            {
                sharpness_img = img;
            }

            // Calculate gradients in x and y directions
            cv::Mat grad_x, grad_y;
            cv::Sobel(sharpness_img, grad_x, CV_64F, 1, 0, 3);
            cv::Sobel(sharpness_img, grad_y, CV_64F, 0, 1, 3);

            // Calculate gradient magnitude
            cv::Mat grad_mag;
            cv::magnitude(grad_x, grad_y, grad_mag);

            // Calculate mean of gradient magnitude
            cv::Scalar mean = cv::mean(grad_mag);

            return mean[0];
        }

        // Based on: https://www.sciencedirect.com/science/article/abs/pii/S1077314296900600
        static double estimate_noise(const cv::Mat &img)
        {
            if (img.empty())
            {
                return 0.0;
            }

            cv::Mat noise_img;
            if (img.channels() == 3)
            {
                cv::cvtColor(img, noise_img, cv::COLOR_BGR2GRAY);
            }
            else
            {
                noise_img = img;
            }

            const cv::Mat laplacian_mask = (cv::Mat_<double>(3, 3) << 1, -2, 1, -2, 4, -2, 1, -2, 1);

            cv::Mat laplacian_image;
            cv::filter2D(noise_img, laplacian_image, -1, laplacian_mask, cv::Point(-1, -1), 0, cv::BORDER_DEFAULT);

            double noise_height = noise_img.rows - 2;
            double noise_width = noise_img.cols - 2;
            double sigma = cv::sum(cv::abs(laplacian_image))[0] * std::sqrt(0.5 * M_PI) / (6.0 * noise_width * noise_height);

            return sigma;
        }

        // Based on "Noise Aware Image Assessment metric based Auto Exposure Control" by "Uk Cheol Shin, KAIST RCV LAB"
        // Can be used to quantify the amount of information, or "texture", in an image.
        // Normalised here so 1 represents maximum entropy (an image with a perfectly uniform histogram, meaning each gray level is equally probable)
        // and 0 represents minimum entropy (an image where every pixel has the same color).
        static float estimate_entropy(const cv::Mat &img)
        {
            if (img.empty())
            {
                return 0.0;
            }

            cv::Mat entropy_img;
            if (img.channels() == 3)
            {
                cv::cvtColor(img, entropy_img, cv::COLOR_BGR2GRAY);
            }
            else
            {
                entropy_img = img;
            }

            cv::Mat hist;
            const int hist_size = 256;

            // Compute the histograms:
            const float range[] = {0, hist_size};
            const float *hist_range = {range};

            // images, number of images, channels, mask, hist, dim, histsize, ranges,uniform, accumulate
            cv::calcHist(&entropy_img, 1, 0, cv::Mat(), hist, 1, &hist_size, &hist_range, true, false);

            // compute entropy
            double entropy_value = 0;
            const double total_size = entropy_img.rows * entropy_img.cols; // total size of all symbols in an image

            float *sym_occur = hist.ptr<float>(0); // the number of times a sybmol has occured
            for (int i = 0; i < hist_size; ++i)
            {
                if (sym_occur[i] > 0) // log of zero goes to infinity
                {
                    entropy_value += ((double)sym_occur[i] / total_size) * (std::log2(total_size / (double)sym_occur[i]));
                }
            }

            entropy_value /= 8.0; // the max entropy for an 8-bit grayscale image is 8, so needs to be adjusted for 16

            hist.release();

            return entropy_value;
        }

        static cv::Mat create_histogram(const cv::Mat &img, int hist_w = 512, int hist_ht = 400)
        {
            const int hist_size = 256;
            const float range[] = {0, img.elemSize1() == 1 ? 255.0f : 65535.0f};
            const float *hist_range = {range};
            const bool uniform = true;
            const bool accumulate = false;
            int hist_h = hist_ht - 30;

            std::vector<cv::Mat> bgr_planes;
            cv::split(img, bgr_planes);

            cv::Mat b_hist, g_hist, r_hist;
            cv::calcHist(&bgr_planes[0], 1, 0, cv::Mat(), b_hist, 1, &hist_size, &hist_range, uniform, accumulate);

            if (img.channels() > 1)
            {
                cv::calcHist(&bgr_planes[1], 1, 0, cv::Mat(), g_hist, 1, &hist_size, &hist_range, uniform, accumulate);
                cv::calcHist(&bgr_planes[2], 1, 0, cv::Mat(), r_hist, 1, &hist_size, &hist_range, uniform, accumulate);
            }

            int bin_w = cvRound(static_cast<double>(hist_w) / hist_size);
            cv::Mat hist_img(hist_ht, hist_w, CV_8UC3, cv::Scalar(0, 0, 0));

            cv::normalize(b_hist, b_hist, 0, hist_img.rows, cv::NORM_MINMAX, -1, cv::Mat());

            for (int i = 0; i <= hist_w; i += hist_w / 10)
            {
                std::ostringstream str;
                str << std::fixed << std::setprecision(1) << static_cast<float>(i) / hist_w;
                cv::putText(hist_img, str.str(), cv::Point(i, hist_ht - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1, 8);
            }

            for (int i = 1; i < hist_size; ++i)
            {
                cv::line(hist_img,
                         cv::Point(bin_w * (i - 1), hist_h - cvRound(b_hist.at<float>(i - 1))),
                         cv::Point(bin_w * i, hist_h - cvRound(b_hist.at<float>(i))),
                         img.channels() == 1 ? cv::Scalar(255, 255, 255) : cv::Scalar(255, 0, 0),
                         2,
                         8,
                         0);
            }

            if (img.channels() > 1)
            {
                cv::normalize(g_hist, g_hist, 0, hist_img.rows, cv::NORM_MINMAX, -1, cv::Mat());
                cv::normalize(r_hist, r_hist, 0, hist_img.rows, cv::NORM_MINMAX, -1, cv::Mat());

                for (int i = 1; i < hist_size; ++i)
                {
                    cv::line(hist_img,
                             cv::Point(bin_w * (i - 1), hist_h - cvRound(g_hist.at<float>(i - 1))),
                             cv::Point(bin_w * i, hist_h - cvRound(g_hist.at<float>(i))),
                             cv::Scalar(0, 255, 0),
                             2,
                             8,
                             0);
                    cv::line(hist_img,
                             cv::Point(bin_w * (i - 1), hist_h - cvRound(r_hist.at<float>(i - 1))),
                             cv::Point(bin_w * i, hist_h - cvRound(r_hist.at<float>(i))),
                             cv::Scalar(0, 0, 255),
                             2,
                             8,
                             0);
                }
            }

            return hist_img;
        }

        template <typename Container>
        static cv::Mat draw_graph(const std::string &_name, const Container &_data, const cv::Size &_graphSize, int _type, const cv::Scalar& _lineColor, const cv::Scalar& _rectColor, int thickness = 1)
        {
            cv::Mat graph(_graphSize, _type);
            cv::Scalar lineColor{_lineColor};
            cv::Scalar rectColor{_rectColor};
            if (graph.channels() == 1)
            {
                lineColor = cv::Scalar{255, 255, 255, 0};
                rectColor = cv::Scalar{128, 128, 128, 0};
            }
            if (graph.elemSize1() > 1)
            {
                lineColor = cv::Scalar{lineColor[0] * 255, lineColor[1] * 255, lineColor[2] * 255, lineColor[3] * 255};
                rectColor = cv::Scalar{rectColor[0] * 255, rectColor[1] * 255, rectColor[2] * 255, rectColor[3] * 255};
            }
            const TextWriter text_writter(cv::Scalar{255, 255, 255, 0}, 9, 2.5);
            const TextWriter text_writter_name(lineColor, 6, 3.5);

            cv::rectangle(graph, cv::Rect(0, 0, _graphSize.width, _graphSize.height), rectColor, cv::FILLED);

            const double minVal = *std::min_element(_data.begin(), _data.end());
            const double maxVal = *std::max_element(_data.begin(), _data.end());
            const double min_graph = minVal * 0.85;
            const double max_graph = maxVal * 1.15;
            const double normalization_mult = 1.0 / (max_graph - min_graph) * _graphSize.height;
            const double scalingFactor = static_cast<double>(_graphSize.width) / static_cast<double>(_data.size());

            for (size_t i = 1; i < _data.size(); ++i)
            {
                const double val0{(_data[i - 1] - min_graph) * normalization_mult};
                const double val1{(_data[i] - min_graph) * normalization_mult};
                const cv::Point point0((i - 1) * scalingFactor, _graphSize.height - val0);
                const cv::Point point1(i * scalingFactor, _graphSize.height - val1);
                cv::line(graph, point0, point1, lineColor, thickness);
            }

            text_writter_name.write_text(graph, _name, 1, false);
            text_writter.write_text(graph, format_double(maxVal, 3), 1, true);
            text_writter.write_text(graph, format_double(minVal, 3), 8, true);
            text_writter_name.write_text(graph, format_double(_data.back(), 3), 6, false);

            return graph;
        }

        static void overlay_image(cv::Mat &dst, const cv::Mat &src, cv::Point location, double alpha)
        {
            cv::Mat overlay;
            dst.copyTo(overlay);

            cv::Rect roi(location.x, location.y, src.cols, src.rows);
            cv::Mat subImage = overlay(roi);

            src.copyTo(subImage);

            cv::addWeighted(overlay, alpha, dst, 1 - alpha, 0.0, dst);
        }

        static std::string format_double(double value, int decimal_places = 2)
        {
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(decimal_places) << value;
            return oss.str();
        }
    };
}