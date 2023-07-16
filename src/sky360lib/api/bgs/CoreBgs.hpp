#pragma once

#include "../include/coreUtils.hpp"

#include <opencv2/core.hpp>

#include <vector>

namespace sky360lib::bgs
{
    class CoreParameters;

    class CoreBgs
    {
    public:
        /// Detects the number of available threads to use
        static const size_t DETECT_NUMBER_OF_THREADS{0};

        CoreBgs(size_t _numProcessesParallel = DETECT_NUMBER_OF_THREADS);

        virtual ~CoreBgs() {}

        void apply(const cv::Mat &_image, cv::Mat &_fgmask);
        cv::Mat apply_ret(const cv::Mat &_image);

        void restart();

        virtual CoreParameters &get_parameters() = 0;

        virtual void get_background_image(cv::Mat &_bgImage) = 0;

    protected:
        virtual void initialize(const cv::Mat &_image) = 0;
        virtual void process(const cv::Mat &_image, cv::Mat &_fgmask, int _numProcess) = 0;

        void prepare_parallel(const cv::Mat &_image);
        void apply_parallel(const cv::Mat &_image, cv::Mat &_fgmask);

        size_t m_num_processes_parallel;
        bool m_initialized;
        std::vector<size_t> m_process_seq;
        std::vector<std::unique_ptr<ImgSize>> m_img_sizes_parallel;
        std::unique_ptr<ImgSize> m_original_img_size;
    };
}