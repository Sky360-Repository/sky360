#pragma once
#ifndef __TRACK_PREDICTION_HPP__
#define __TRACK_PREDICTION_HPP__

#include <opencv2/opencv.hpp>

class TrackPrediction
{
public:
    TrackPrediction(int id, const cv::Rect2d& bbox) 
        : id(id)
    {
        double x = bbox.x, 
               y = bbox.y, 
               w = bbox.width, 
               h = bbox.height;

        // Initialize the Kalman filter.
        kalman.init(4, 2, 0);
        kalman.measurementMatrix = (cv::Mat_<float>(2, 4) << 1, 0, 0, 0,
                                    0, 1, 0, 0);
        kalman.transitionMatrix = (cv::Mat_<float>(4, 4) << 1, 0, 1, 0,
                                   0, 1, 0, 1,
                                   0, 0, 1, 0,
                                   0, 0, 0, 1);
        setIdentity(kalman.processNoiseCov, cv::Scalar::all(0.03));

        double cx = x + w / 2;
        double cy = y + h / 2;
        kalman.statePre.at<float>(0) = cx;
        kalman.statePre.at<float>(1) = cy;
        kalman.statePre.at<float>(2) = 0;
        kalman.statePre.at<float>(3) = 0;

        kalman.statePost.at<float>(0) = cx;
        kalman.statePost.at<float>(1) = cy;
        kalman.statePost.at<float>(2) = 0;
        kalman.statePost.at<float>(3) = 0;
    }

    cv::Point update(const cv::Rect2d& bbox)
    {
        double x = bbox.x, y = bbox.y, w = bbox.width, h = bbox.height;
        cv::Mat center = (cv::Mat_<float>(2, 1) << x + w / 2, y + h / 2);
        kalman.correct(center);
        cv::Mat predicted = kalman.predict();

        int px = static_cast<int>(predicted.at<float>(0));
        int py = static_cast<int>(predicted.at<float>(1));
        return {px, py};
    }

private:
    int id;
    cv::KalmanFilter kalman;
};

#endif