#include "connectedBlobDetection.hpp"

#include <opencv2/imgproc.hpp>

#include <iostream>
#include <execution>
#include <algorithm>

namespace sky360lib::blobs
{
    ConnectedBlobDetection::ConnectedBlobDetection(const ConnectedBlobDetectionParams &_params, size_t _num_processes_parallel)
        : m_params{_params}, m_num_processes_parallel{_num_processes_parallel}, m_initialized{false}
    {
        if (m_num_processes_parallel == DETECT_NUMBER_OF_THREADS)
        {
            m_num_processes_parallel = calc_available_threads();
        }
    }

    static inline cv::KeyPoint convertFromRect(const cv::Rect &rect)
    {
        static const float scale = 6.0f;
        const float size = (float)std::max(rect.width, rect.height) / scale;
        return cv::KeyPoint(rect.x + scale * size / 2.0f, rect.y + scale * size / 2.0f, size);
    }

    std::vector<cv::KeyPoint> ConnectedBlobDetection::detect_kp(const cv::Mat &_image)
    {
        std::vector<cv::Rect> bboxes;
        detect(_image, bboxes);
        std::vector<cv::KeyPoint> kps;
        std::transform(bboxes.begin(),
                       bboxes.end(),
                       std::back_inserter(kps),
                       [](const cv::Rect &r) -> cv::KeyPoint
                       { return convertFromRect(r); });
        return kps;
    }

    std::vector<cv::Rect> ConnectedBlobDetection::detect_ret(const cv::Mat &_image)
    {
        std::vector<cv::Rect> bboxes;
        detect(_image, bboxes);
        return bboxes;
    }

    inline bool rects_overlap(const cv::Rect &r1, const cv::Rect &r2)
    {
        if ((r1.width == 0 || r1.height == 0 || r2.width == 0 || r2.height == 0) ||
            (r1.x > (r2.x + r2.width) || r2.x > (r1.x + r1.width)) ||
            (r1.y > (r2.y + r2.height) || r2.y > (r1.y + r1.height)))
        {
            return false;
        }

        return true;
    }

    inline float rects_distance_squared(const cv::Rect &r1, const cv::Rect &r2)
    {
        if (rects_overlap(r1, r2))
            return 0;

        const int x_distance = std::max(0, std::max(r1.x, r2.x) - std::min(r1.x + r1.width, r2.x + r2.width));
        const int y_distance = std::max(0, std::max(r1.y, r2.y) - std::min(r1.y + r1.height, r2.y + r2.height));

        return (x_distance * x_distance) + (y_distance * y_distance);
    }

    // Joining bboxes together if they overlap
    static inline void joinBBoxes(std::vector<cv::Rect> &_bboxes, int minDistanceSquared)
    {
        bool bboxOverlap;
        do
        {
            bboxOverlap = false;
            for (size_t i{0}; i < _bboxes.size() - 1; ++i)
            {
                for (size_t j{i + 1}; j < _bboxes.size();)
                {
                    if (rects_distance_squared(_bboxes[i], _bboxes[j]) < minDistanceSquared)
                    {
                        bboxOverlap = true;
                        const int xmax = std::max(_bboxes[i].x + _bboxes[i].width, _bboxes[j].x + _bboxes[j].width);
                        const int ymax = std::max(_bboxes[i].y + _bboxes[i].height, _bboxes[j].y + _bboxes[j].height);
                        _bboxes[i].x = std::min(_bboxes[i].x, _bboxes[j].x);
                        _bboxes[i].y = std::min(_bboxes[i].y, _bboxes[j].y);
                        _bboxes[i].width = xmax - _bboxes[i].x;
                        _bboxes[i].height = ymax - _bboxes[i].y;
                        _bboxes.erase(_bboxes.begin() + j);
                    }
                    else
                    {
                        ++j;
                    }
                }
            }
        } while (bboxOverlap);
    }

    inline static void applySizeCut(std::vector<cv::Rect> &_bboxes, const int _sizeThreshold, const int _areaThreshold)
    {
        for (size_t i{0}; i < _bboxes.size();)
        {
            if ((_bboxes[i].width < _sizeThreshold) || (_bboxes[i].height < _sizeThreshold) || (_bboxes[i].area() < _areaThreshold))
            {
                _bboxes.erase(_bboxes.begin() + i);
            }
            else
            {
                ++i;
            }
        }
    }

    inline void ConnectedBlobDetection::pos_process_bboxes(std::vector<cv::Rect> &_bboxes)
    {
        const size_t numLabels = _bboxes.size();

        // Reseting returning bboxes to the MIN/MAX values
        for (size_t i{0}; i < numLabels; ++i)
        {
            _bboxes[i].x = _bboxes[i].y = INT_MAX;
            _bboxes[i].width = _bboxes[i].height = INT_MIN;
        }

        // Joining all parallel bboxes into one label
        for (size_t i{0}; i < m_num_processes_parallel; ++i)
        {
            const int addedY = (int)(m_img_sizes_parallel[i]->original_pixel_pos / m_img_sizes_parallel[i]->width);
            const std::vector<cv::Rect> &bboxesParallel = m_bboxes_parallel[i];
            for (size_t j{0}; j < numLabels; ++j)
            {
                // If the coordinates for the label were altered, process
                if (bboxesParallel[j].x != INT_MAX)
                {
                    _bboxes[j].x = std::min(_bboxes[j].x, bboxesParallel[j].x);
                    _bboxes[j].y = std::min(_bboxes[j].y, bboxesParallel[j].y + addedY);
                    _bboxes[j].width = std::max(_bboxes[j].width, (bboxesParallel[j].width - _bboxes[j].x) + 1);
                    _bboxes[j].height = std::max(_bboxes[j].height, ((bboxesParallel[j].height + addedY) - _bboxes[j].y) + 1);
                }
            }
        }

        // Joining bboxes that are overlaping each other
        joinBBoxes(_bboxes, m_params.min_distance_squared);

        // Removing bboxes that are below threshold
        applySizeCut(_bboxes, m_params.size_threshold, m_params.area_threshold);
    }

    // Finds the connected components in the image and returns a list of bounding boxes
    bool ConnectedBlobDetection::detect(const cv::Mat &_image, std::vector<cv::Rect> &_bboxes)
    {
        if (!m_initialized || *m_original_img_size != ImgSize(_image))
        {
            prepare_parallel(_image);
            m_initialized = true;
        }

        // Use connected component analysis to find the blobs in the image, subtract 1 because the background is considered as label 0
        // CCL_SAUF      = 0, //!< SAUF @cite Wu2009 algorithm for 8-way connectivity, SAUF algorithm for 4-way connectivity. The parallel implementation described in @cite Bolelli2017 is available for SAUF.
        // CCL_BBDT      = 1, //!< BBDT @cite Grana2010 algorithm for 8-way connectivity, SAUF algorithm for 4-way connectivity. The parallel implementation described in @cite Bolelli2017 is available for both BBDT and SAUF.
        // CCL_SPAGHETTI = 2, //!< Spaghetti @cite Bolelli2019 algorithm for 8-way connectivity, Spaghetti4C @cite Bolelli2021 algorithm for 4-way connectivity. The parallel implementation described in @cite Bolelli2017 is available for both Spaghetti and Spaghetti4C.
        const int numLabels = cv::connectedComponents(_image, m_labels, 8, CV_32S, cv::CCL_SPAGHETTI) - 1;

        if (numLabels > 0 && numLabels <= m_params.max_blobs)
        {
            _bboxes.resize(numLabels);
            std::for_each(
                std::execution::par,
                m_process_seq.begin(),
                m_process_seq.end(),
                [&](int np)
                {
                    // Reseting parallel bboxes to the MIN/MAX values
                    m_bboxes_parallel[np].resize(numLabels);
                    for (int j{0}; j < numLabels; ++j)
                    {
                        m_bboxes_parallel[np][j].x = m_bboxes_parallel[np][j].y = INT_MAX;
                        m_bboxes_parallel[np][j].width = m_bboxes_parallel[np][j].height = INT_MIN;
                    }
                    // Spliting the image into chuncks and processing
                    const cv::Mat imgSplit(m_img_sizes_parallel[np]->height, m_img_sizes_parallel[np]->width, m_labels.type(),
                                           m_labels.data + (m_img_sizes_parallel[np]->original_pixel_pos * m_img_sizes_parallel[np]->num_channels));
                    apply_detect_bboxes(imgSplit, m_bboxes_parallel[np]);
                });

            pos_process_bboxes(_bboxes);

            return true;
        }

        return false;
    }

    void ConnectedBlobDetection::apply_detect_bboxes(const cv::Mat &_labels, std::vector<cv::Rect> &_bboxes)
    {
        int *pLabel = (int *)_labels.data;
        for (int r = 0; r < _labels.rows; r++)
        {
            for (int c = 0; c < _labels.cols; c++)
            {
                const int label = *pLabel - 1;
                if (label >= 0)
                {
                    _bboxes[label].x = std::min(_bboxes[label].x, c);
                    _bboxes[label].y = std::min(_bboxes[label].y, r);
                    _bboxes[label].width = std::max(_bboxes[label].width, c);
                    _bboxes[label].height = std::max(_bboxes[label].height, r);
                }
                ++pLabel;
            }
        }
    }

    void ConnectedBlobDetection::prepare_parallel(const cv::Mat &_image)
    {
        m_original_img_size = ImgSize::create(_image.size().width, _image.size().height,
                                              _image.channels(),
                                              _image.elemSize1(),
                                              0);
        m_img_sizes_parallel.resize(m_num_processes_parallel);
        m_process_seq.resize(m_num_processes_parallel);
        m_bboxes_parallel.resize(m_num_processes_parallel);
        size_t y{0};
        size_t h{_image.size().height / m_num_processes_parallel};
        for (size_t i{0}; i < m_num_processes_parallel; ++i)
        {
            m_process_seq[i] = i;
            if (i == (m_num_processes_parallel - 1))
            {
                h = _image.size().height - y;
            }
            m_img_sizes_parallel[i] = ImgSize::create(_image.size().width, h,
                                                      4, 1,
                                                      y * _image.size().width);
            y += h;
        }
    }

}