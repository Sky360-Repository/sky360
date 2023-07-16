#pragma once

#include "../include/coreUtils.hpp"

#include <vector>
#include <execution>
#include <algorithm>

#include <opencv2/core.hpp>

namespace sky360lib::utils
{

    class ImageStacker
    {
    public:
        /// Detects the number of available threads to use
        static const size_t DETECT_NUMBER_OF_THREADS{0};

        ImageStacker(double _weight = 0.7, size_t _num_processes_parallel = DETECT_NUMBER_OF_THREADS)
            : m_weight{_weight}
            , m_num_processes_parallel{_num_processes_parallel}
        {
            if (m_num_processes_parallel == DETECT_NUMBER_OF_THREADS)
            {
                m_num_processes_parallel = calc_available_threads();
            }
        }

        void set_weight(double _weight)
        {
            m_weight = _weight;
            m_stacked_image.release();
        }

        double get_weight() const
        {
            return m_weight;
        }

        void stack(const cv::Mat &_image_in, cv::Mat &_image_out)
        {
            if (m_stacked_image.empty())
            {
                m_stacked_image = _image_in;
                _image_out = _image_in;
                return;
            }
            cv::addWeighted(_image_in, 1.0, m_stacked_image, m_weight, 0, _image_out);
            m_stacked_image = _image_out;
            // if (!m_initialized || *m_original_img_size != ImgSize(_image_in))
            // {
            //     prepare_parallel(_image_in);
            //     m_initialized = true;
            // }
            // if (_image_out.empty() || _image_out.elemSize1() != _image_in.elemSize1())
            // {
            //     _image_out.create(_image_in.size().height / 2, _image_in.size().width / 2, _image_in.elemSize1() == 1 ? CV_8UC1 : CV_16UC1);
            // }

            // if (m_num_processes_parallel == 1)
            // {
            //     process(_image_in, _image_out, 0);
            // }
            // else
            // {
            //     apply_parallel(_image_in, _image_out);
            // }
        }

        // void prepare_parallel(const cv::Mat &_image)
        // {
        //     m_original_img_size = ImgSize::create(_image.size().width, _image.size().height,
        //                                           _image.channels(),
        //                                           _image.elemSize1(),
        //                                           0);
        //     m_img_sizes_parallel.resize(m_num_processes_parallel);
        //     m_process_seq.resize(m_num_processes_parallel);
        //     size_t y{0};
        //     size_t h{_image.size().height / m_num_processes_parallel};
        //     for (size_t i{0}; i < m_num_processes_parallel; ++i)
        //     {
        //         m_process_seq[i] = i;
        //         if (i == (m_num_processes_parallel - 1))
        //         {
        //             h = _image.size().height - y;
        //         }
        //         m_img_sizes_parallel[i] = ImgSize::create(_image.size().width, h,
        //                                                   _image.channels(),
        //                                                   _image.elemSize1(),
        //                                                   y * _image.size().width);
        //         y += h;
        //     }
        // }

        // void apply_parallel(const cv::Mat &_image_in, cv::Mat &_image_out)
        // {
        //     std::for_each(
        //         std::execution::par,
        //         m_process_seq.begin(),
        //         m_process_seq.end(),
        //         [&](int np)
        //         {
        //             const cv::Mat img_in_split(m_img_sizes_parallel[np]->height, m_img_sizes_parallel[np]->width, _image_in.type(),
        //                                    _image_in.data + (m_img_sizes_parallel[np]->original_pixel_pos * m_img_sizes_parallel[np]->num_channels * m_img_sizes_parallel[np]->bytes_per_pixel));
        //             cv::Mat img_out_split(m_img_sizes_parallel[np]->height / 2, m_img_sizes_parallel[np]->width / 2, _image_out.type(),
        //                                 _image_out.data + (m_img_sizes_parallel[np]->original_pixel_pos * m_img_sizes_parallel[np]->bytes_per_pixel) / 4);
        //             if (m_img_sizes_parallel[np]->bytes_per_pixel == 1)
        //             {
        //                 process(img_in_split, img_out_split, np);
        //             }
        //             else
        //             {
        //                 process16(img_in_split, img_out_split, np);
        //             }
        //         });
        // }

    private:
        double m_weight;
        cv::Mat m_stacked_image;
        size_t m_num_processes_parallel;
        // std::vector<size_t> m_process_seq;
        // bool m_initialized = false;
        // std::unique_ptr<ImgSize> m_original_img_size;
        // std::vector<std::unique_ptr<ImgSize>> m_img_sizes_parallel;

        // void process(const cv::Mat &_image_in, cv::Mat &_image_out, int _numProcess)
        // {
        //     (void)_numProcess;
        //     const int width{_image_out.size().width};
        //     const int height{_image_out.size().height};

        //     uint8_t* p_data_in{_image_in.data};
        //     uint8_t* p_data_out{_image_out.data};
        //     for (int y = 0; y < height; ++y)
        //     {
        //         uint8_t* p_current_line = p_data_out;
        //         for (int x = 0; x < width; ++x, ++p_data_out)
        //         {
        //             *p_data_out = std::min(p_data_in[0] + p_data_in[1], 255);
        //             p_data_in += 2;
        //         }
        //         p_data_out = p_current_line;
        //         for (int x = 0; x < width; ++x, ++p_data_out)
        //         {
        //             *p_data_out = std::min(*p_data_out + p_data_in[0] + p_data_in[1], 255);
        //             p_data_in += 2;
        //         }
        //     }
        // }

        // void process16(const cv::Mat &_image_in, cv::Mat &_image_out, int _numProcess)
        // {
        //     (void)_numProcess;
        //     const int width{_image_out.size().width};
        //     const int height{_image_out.size().height};

        //     uint16_t* p_data_in{(uint16_t *)_image_in.data};
        //     uint16_t* p_data_out{(uint16_t *)_image_out.data};
        //     for (int y = 0; y < height; ++y)
        //     {
        //         uint16_t* p_current_line = p_data_out;
        //         for (int x = 0; x < width; ++x, ++p_data_out)
        //         {
        //             *p_data_out = std::min(p_data_in[0] + p_data_in[1], 65535);
        //             p_data_in += 2;
        //         }
        //         p_data_out = p_current_line;
        //         for (int x = 0; x < width; ++x, ++p_data_out)
        //         {
        //             *p_data_out = std::min(*p_data_out + p_data_in[0] + p_data_in[1], 65535);
        //             p_data_in += 2;
        //         }
        //     }
        // }
    };
}