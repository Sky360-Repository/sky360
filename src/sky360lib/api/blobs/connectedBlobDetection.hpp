#pragma once

#include "../include/coreUtils.hpp"

#include <opencv2/core.hpp>

namespace sky360lib::blobs
{
    struct ConnectedBlobDetectionParams final
    {
        static const int DEFAULT_SIZE_THRESHOLD = 5;
        static const int DEFAULT_AREA_THRESHOLD = 25;
        static const int DEFAULT_MIN_DISTANCE = 25;
        static const int DEFAULT_MAX_BLOBS = 100;

        ConnectedBlobDetectionParams()
            : ConnectedBlobDetectionParams(DEFAULT_SIZE_THRESHOLD, DEFAULT_AREA_THRESHOLD, DEFAULT_MIN_DISTANCE, DEFAULT_MAX_BLOBS)
        {
        }

        ConnectedBlobDetectionParams(int _sizeThreshold, int _areaThreshold, int _minDistance, int _maxBlobs)
            : size_threshold{_sizeThreshold}, area_threshold{_areaThreshold}, min_mistance{_minDistance}, min_distance_squared{_minDistance * _minDistance}, max_blobs{_maxBlobs}
        {
        }

        inline void set_size_threshold(int _threshold) { size_threshold = std::max(_threshold, 2); }
        inline void set_area_threshold(int _threshold) { area_threshold = std::max(_threshold, size_threshold * size_threshold); }
        inline void setMinDistance(int _min_distance)
        {
            min_mistance = std::max(_min_distance, 2);
            min_distance_squared = min_mistance * min_mistance;
        }

        int size_threshold;
        int area_threshold;
        int min_mistance;
        int min_distance_squared;
        int max_blobs;
    };

    class ConnectedBlobDetection final
    {
    public:
        /// Detects the number of available threads to use
        static const size_t DETECT_NUMBER_OF_THREADS{0};

        ConnectedBlobDetection(const ConnectedBlobDetectionParams &_params = ConnectedBlobDetectionParams(),
                               size_t _numProcessesParallel = DETECT_NUMBER_OF_THREADS);

        inline void set_size_threshold(int _threshold) { m_params.set_size_threshold(_threshold); }
        inline void set_area_threshold(int _threshold) { m_params.set_size_threshold(_threshold); }
        inline void set_min_distance(int _distance) { m_params.setMinDistance(_distance); }

        // Finds the connected components in the image and returns a list of bounding boxes
        bool detect(const cv::Mat &_image, std::vector<cv::Rect> &_bboxes);

        // Finds the connected components in the image and returns a list of keypoints
        // This function uses detect and converts from Rect to KeyPoints using a fixed scale
        std::vector<cv::KeyPoint> detect_kp(const cv::Mat &_image);

        // Finds the connected components in the image and returns a list of bounding boxes
        std::vector<cv::Rect> detect_ret(const cv::Mat &_image);

    private:
        ConnectedBlobDetectionParams m_params;
        size_t m_num_processes_parallel;
        bool m_initialized;
        cv::Mat m_labels;
        std::vector<size_t> m_process_seq;
        std::vector<std::unique_ptr<ImgSize>> m_img_sizes_parallel;
        std::vector<std::vector<cv::Rect>> m_bboxes_parallel;
        std::unique_ptr<ImgSize> m_original_img_size;

        void prepare_parallel(const cv::Mat &_image);
        static void apply_detect_bboxes(const cv::Mat &_labels, std::vector<cv::Rect> &_bboxes);
        inline void pos_process_bboxes(std::vector<cv::Rect> &_bboxes);
    };
}