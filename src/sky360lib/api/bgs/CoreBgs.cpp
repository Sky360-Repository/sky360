#include "CoreBgs.hpp"
#include "CoreParameters.hpp"

#include <iostream>
#include <execution>
#include <algorithm>

using namespace sky360lib::bgs;

CoreBgs::CoreBgs(size_t _numProcessesParallel)
    : m_num_processes_parallel{_numProcessesParallel}
    , m_initialized{false}
{
    if (_numProcessesParallel == DETECT_NUMBER_OF_THREADS)
    {
        m_num_processes_parallel = calc_available_threads();;
    }
}

void CoreBgs::restart()
{
    m_initialized = false;
}

void CoreBgs::apply(const cv::Mat &_image, cv::Mat &_fgmask)
{
    if (!m_initialized || *m_original_img_size != ImgSize(_image))
    {
        prepare_parallel(_image);
        initialize(_image);
        _fgmask.create(_image.size(), CV_8UC1);
        m_initialized = true;
    }
    if (_fgmask.empty())
    {
        _fgmask.create(_image.size(), CV_8UC1);
    }

    if (m_num_processes_parallel == 1)
    {
        process(_image, _fgmask, 0);
    }
    else
    {
        apply_parallel(_image, _fgmask);
    }
}

cv::Mat CoreBgs::apply_ret(const cv::Mat &_image)
{
    cv::Mat imgMask;
    apply(_image, imgMask);
    return imgMask;
}

void CoreBgs::prepare_parallel(const cv::Mat &_image)
{
    m_original_img_size = ImgSize::create(_image.size().width, _image.size().height,
                                                  _image.channels(),
                                                  _image.elemSize1(),
                                                  0);
    m_img_sizes_parallel.resize(m_num_processes_parallel);
    m_process_seq.resize(m_num_processes_parallel);
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
                                                _image.channels(),
                                                _image.elemSize1(),
                                                y * _image.size().width);
        y += h;
    }
}

void CoreBgs::apply_parallel(const cv::Mat &_image, cv::Mat &_fgmask)
{
    std::for_each(
        std::execution::par,
        m_process_seq.begin(),
        m_process_seq.end(),
        [&](int np)
        {
            const cv::Mat imgSplit(m_img_sizes_parallel[np]->height, m_img_sizes_parallel[np]->width, _image.type(),
                                   _image.data + (m_img_sizes_parallel[np]->original_pixel_pos * m_img_sizes_parallel[np]->num_channels * m_img_sizes_parallel[np]->bytes_per_pixel));
            cv::Mat maskPartial(m_img_sizes_parallel[np]->height, m_img_sizes_parallel[np]->width, _fgmask.type(),
                                _fgmask.data + m_img_sizes_parallel[np]->original_pixel_pos);
            process(imgSplit, maskPartial, np);
        });
}
