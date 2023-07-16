#include "Vibe.hpp"

#include <iostream>
#include <execution>

using namespace sky360lib::bgs;

Vibe::Vibe(VibeParams _params, size_t _num_processes_parallel)
    : CoreBgs(_num_processes_parallel)
    , m_params{_params}
{
}

void Vibe::initialize(const cv::Mat &_init_img)
{
    std::vector<std::unique_ptr<Img>> img_split(m_num_processes_parallel);
    m_orig_img_size = ImgSize::create(_init_img.size().width, _init_img.size().height, _init_img.channels(), _init_img.elemSize1(), 0);
    Img frameImg(_init_img.data, *m_orig_img_size);
    split_img(frameImg, img_split, m_num_processes_parallel);

    m_random_generators.resize(m_num_processes_parallel);
    m_bg_img_samples.resize(m_num_processes_parallel);
    if (m_orig_img_size->bytes_per_pixel == 1)
    {
        for (size_t i{0}; i < m_num_processes_parallel; ++i)
        {
            initialize<uint8_t>(*img_split[i], m_bg_img_samples[i], m_random_generators[i]);
        }
    }
    else
    {
        for (size_t i{0}; i < m_num_processes_parallel; ++i)
        {
            initialize<uint16_t>(*img_split[i], m_bg_img_samples[i], m_random_generators[i]);
        }
    }
}

template<class T>
void Vibe::initialize(const Img &_init_img, std::vector<std::unique_ptr<Img>> &_bg_img_samples, Pcg32 &_rnd_gen)
{
    int y_sample, x_sample;
    _bg_img_samples.resize(m_params.bg_samples);
    for (size_t s{0}; s < m_params.bg_samples; ++s)
    {
        _bg_img_samples[s] = Img::create(_init_img.size, false);
        for (int y_orig{0}; y_orig < _init_img.size.height; y_orig++)
        {
            for (int x_orig{0}; x_orig < _init_img.size.width; x_orig++)
            {
                get_sample_position_7x7_std2(_rnd_gen.fast(), x_sample, y_sample, x_orig, y_orig, _init_img.size);
                const size_t pixel_pos = (y_orig * _init_img.size.width + x_orig) * _init_img.size.num_channels;
                const size_t sample_pos = (y_sample * _init_img.size.width + x_sample) * _init_img.size.num_channels;
                _bg_img_samples[s]->ptr<T>()[pixel_pos] = _init_img.ptr<T>()[sample_pos];
                if (_init_img.size.num_channels > 1)
                {
                    _bg_img_samples[s]->ptr<T>()[pixel_pos + 1] = _init_img.ptr<T>()[sample_pos + 1];
                    _bg_img_samples[s]->ptr<T>()[pixel_pos + 2] = _init_img.ptr<T>()[sample_pos + 2];
                }
            }
        }
    }
}

void Vibe::process(const cv::Mat &_image, cv::Mat &_fg_mask, int _num_process)
{
    Img img_split(_image.data, ImgSize(_image.size().width, _image.size().height, _image.channels(), _image.elemSize1(), 0));
    Img mask_partial(_fg_mask.data, ImgSize(_image.size().width, _image.size().height, _fg_mask.channels(), _fg_mask.elemSize1(), 0));
    if (img_split.size.num_channels > 1)
    {
        if (img_split.size.bytes_per_pixel == 1)
        {
            apply3<uint8_t>(img_split, m_bg_img_samples[_num_process], mask_partial, m_params, m_random_generators[_num_process]);
        }
        else
        {
            apply3<uint16_t>(img_split, m_bg_img_samples[_num_process], mask_partial, m_params, m_random_generators[_num_process]);
        }
    }
    else
    {
        if (img_split.size.bytes_per_pixel == 1)
        {
            apply1<uint8_t>(img_split, m_bg_img_samples[_num_process], mask_partial, m_params, m_random_generators[_num_process]);
        }
        else
        {
            apply1<uint16_t>(img_split, m_bg_img_samples[_num_process], mask_partial, m_params, m_random_generators[_num_process]);
        }
    }
}

template<class T>
void Vibe::apply3(const Img &_image,
                  std::vector<std::unique_ptr<Img>> &_bg_img,
                  Img &_fg_mask,
                  const VibeParams &_params,
                  Pcg32 &_rnd_gen)
{
    _fg_mask.clear();

    const int32_t n_color_dist_threshold = sizeof(T) == 1 ? _params.threshold_color_squared : _params.threshold_color16_squared;

    size_t pix_offset{0}, color_pix_offset{0};
    for (int y{0}; y < _image.size.height; ++y)
    {
        for (int x{0}; x < _image.size.width; ++x, ++pix_offset, color_pix_offset += _image.size.num_channels)
        {
            size_t n_good_samples_count{0},
                n_sample_idx{0};

            const T *const pix_data{&_image.ptr<T>()[color_pix_offset]};

            while (n_sample_idx < _params.bg_samples)
            {
                const T *const bg{&_bg_img[n_sample_idx]->ptr<T>()[color_pix_offset]};
                if (l2_dist3_squared(pix_data, bg) < n_color_dist_threshold)
                {
                    ++n_good_samples_count;
                    if (n_good_samples_count >= _params.required_bg_samples)
                    {
                        break;
                    }
                }
                ++n_sample_idx;
            }
            if (n_good_samples_count < _params.required_bg_samples)
            {
                _fg_mask.data[pix_offset] = UCHAR_MAX;
            }
            else
            {
                if ((_rnd_gen.fast() & _params.and_learning_rate) == 0)
                {
                    T *const bg_img_pix_data{&_bg_img[_rnd_gen.fast() & _params.and_learning_rate]->ptr<T>()[color_pix_offset]};
                    bg_img_pix_data[0] = pix_data[0];
                    bg_img_pix_data[1] = pix_data[1];
                    bg_img_pix_data[2] = pix_data[2];
                }
                if ((_rnd_gen.fast() & _params.and_learning_rate) == 0)
                {
                    const int neigh_data{get_neighbor_position_3x3(x, y, _image.size, _rnd_gen.fast()) * 3};
                    T *const xy_rand_data{&_bg_img[_rnd_gen.fast() & _params.and_learning_rate]->ptr<T>()[neigh_data]};
                    xy_rand_data[0] = pix_data[0];
                    xy_rand_data[1] = pix_data[1];
                    xy_rand_data[2] = pix_data[2];
                }
            }
        }
    }
}

template<class T>
void Vibe::apply1(const Img &_image,
                  std::vector<std::unique_ptr<Img>> &_bg_img,
                  Img &_fg_mask,
                  const VibeParams &_params,
                  Pcg32 &_rnd_gen)
{
    _fg_mask.clear();

    const int32_t n_color_dist_threshold = sizeof(T) == 1 ? _params.threshold_mono : _params.threshold_mono16;

    size_t pix_offset{0};
    for (int y{0}; y < _image.size.height; ++y)
    {
        for (int x{0}; x < _image.size.width; ++x, ++pix_offset)
        {
            uint32_t n_good_samples_count{0},
                n_sample_idx{0};

            const T pix_data{_image.ptr<T>()[pix_offset]};

            while (n_sample_idx < _params.bg_samples)
            {
                if (std::abs((int32_t)_bg_img[n_sample_idx]->ptr<T>()[pix_offset] - (int32_t)pix_data) < n_color_dist_threshold)
                {
                    ++n_good_samples_count;
                    if (n_good_samples_count >= _params.required_bg_samples)
                    {
                        break;
                    }
                }
                ++n_sample_idx;
            }
            if (n_good_samples_count < _params.required_bg_samples)
            {
                _fg_mask.data[pix_offset] = UCHAR_MAX;
            }
            else
            {
                if ((_rnd_gen.fast() & _params.and_learning_rate) == 0)
                {
                    _bg_img[_rnd_gen.fast() & _params.and_learning_rate]->ptr<T>()[pix_offset] = pix_data;
                }
                if ((_rnd_gen.fast() & _params.and_learning_rate) == 0)
                {
                    const int neigh_data{get_neighbor_position_3x3(x, y, _image.size, _rnd_gen.fast())};
                    _bg_img[_rnd_gen.fast() & _params.and_learning_rate]->ptr<T>()[neigh_data] = pix_data;
                }
            }
        }
    }
}

void Vibe::get_background_image(cv::Mat &_bg_image)
{
    cv::Mat avg_bg_img(m_orig_img_size->height, m_orig_img_size->width, CV_32FC(m_orig_img_size->num_channels));

    for (size_t t{0}; t < m_num_processes_parallel; ++t)
    {
        const std::vector<std::unique_ptr<Img>> &bg_samples = m_bg_img_samples[t];
        for (size_t n{0}; n < m_params.bg_samples; ++n)
        {
            size_t in_pix_offset{0};
            size_t out_pix_offset{bg_samples[0]->size.original_pixel_pos * sizeof(float) * bg_samples[0]->size.num_channels};
            for (; in_pix_offset < bg_samples[n]->size.size_in_bytes;
                 in_pix_offset += m_orig_img_size->num_channels,
                 out_pix_offset += sizeof(float) * bg_samples[0]->size.num_channels)
            {
                const uint8_t *const pix_data{&bg_samples[n]->data[in_pix_offset]};
                float *const out_data{(float *)(avg_bg_img.data + out_pix_offset)};
                for (int c{0}; c < m_orig_img_size->num_channels; ++c)
                {
                    out_data[c] += (float)pix_data[c] / (float)m_params.bg_samples;
                }
            }
        }
    }

    avg_bg_img.convertTo(_bg_image, CV_8U);
}
