#pragma once

#include <vector>

#include <opencv2/core.hpp>

namespace sky360lib::tracking
{
    class DSST
    {
    public:
        DSST(){};
        DSST(const Mat &image, Rect2f bounding_box, Size2f template_size, int numberOfScales,
             float scaleStep, float maxModelArea, float sigmaFactor, float scaleLearnRate);
        ~DSST();
        void update(const Mat &image, const Point2f objectCenter);
        float getScale(const Mat &image, const Point2f objecCenter);

    private:
        Mat get_scale_features(Mat img, Point2f pos, Size2f base_target_sz, float current_scale,
                               std::vector<float> &scale_factors, Mat scale_window, Size scale_model_sz);

        Size scale_model_sz;
        Mat ys;
        Mat ysf;
        Mat scale_window;
        std::vector<float> scale_factors;
        Mat sf_num;
        Mat sf_den;
        float scale_sigma;
        float min_scale_factor;
        float max_scale_factor;
        float current_scale_factor;
        int scales_count;
        float scale_step;
        float max_model_area;
        float sigma_factor;
        float learn_rate;

        Size original_targ_sz;
    };
}
