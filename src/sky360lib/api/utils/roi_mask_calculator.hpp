#pragma once

#include <opencv2/opencv.hpp>

namespace sky360lib::utils
{
    class RoiMaskCalculator
    {
    public:
        struct RoiMask
        {
            uint32_t x;
            uint32_t y;
            uint32_t width;
            uint32_t height;
            cv::Mat mask;
        };

        static RoiMask calc_roi_mask(const cv::Mat& image, cv::Point2d circle_init, cv::Point2d circle_end, double fov, double max_fov)
        {
            const cv::Point circle_center(std::abs((circle_init.x + circle_end.x) / 2),  std::abs((circle_init.y + circle_end.y) / 2));
            const double circle_radius = std::sqrt((circle_center.x - circle_init.x) * (circle_center.x - circle_init.x) + (circle_center.y - circle_init.y) * (circle_center.y - circle_init.y));    

            const double fov_radius = circle_radius * (fov / max_fov);

            RoiMask roi_mask;
            roi_mask.width = fov_radius * 2;
            roi_mask.height = image.size().height;
            roi_mask.x = (uint32_t)(circle_center.x - fov_radius) & ~0x1;
            roi_mask.y = 0;
            //roi_mask.mask = cv::Mat::zeros(cv::Size(roi_mask.width, roi_mask.height), CV_8UC1);
            // cv::circle(roi_mask.mask, cv::Point(roi_mask.width / 2, roi_mask.height / 2), fov_radius, cv::Scalar(255), cv::FILLED);
            roi_mask.mask = cv::Mat::zeros(image.size(), CV_8UC1);
            cv::circle(roi_mask.mask, circle_center, fov_radius, cv::Scalar(255), cv::FILLED);

            return roi_mask;
        }
    };
}