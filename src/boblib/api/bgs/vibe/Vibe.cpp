#include "Vibe.hpp"

#include <iostream>
#include <execution>

// #include <opencv2/cudaarithm.hpp>
// #include <cuda_runtime.h>

namespace boblib::bgs
{
    Vibe::Vibe(VibeParams _params, bool use_cuda, size_t _num_processes_parallel)
        : CoreBgs(use_cuda, _num_processes_parallel), m_params(_params)
    {
    }

    Vibe::~Vibe()
    {
    }

    void Vibe::initialize(const boblib::base::Image &_init_img)
    {
        auto &init_img = _init_img.toMat();
        std::vector<std::unique_ptr<Img>> img_split(m_num_processes_parallel);
        m_orig_img_size = ImgSize::create(init_img);
        Img frameImg(init_img.data, *m_orig_img_size);
        split_img(frameImg, img_split, m_num_processes_parallel);

        m_random_generators.resize(m_num_processes_parallel);
        m_bg_img_samples.resize(m_num_processes_parallel);
        if (m_orig_img_size->bytes_per_channel == 1)
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

    template <class T>
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

    void Vibe::process(const boblib::base::Image &_image, boblib::base::Image &_fg_mask, const boblib::base::Image &_detect_mask, int _num_process)
    {
        auto &fg_mask = _fg_mask.toMat();
        Img img_split(_image.data(), ImgSize(_image));
        Img mask_partial(fg_mask.data, ImgSize(fg_mask));
        Img detect_mask_partial(_detect_mask.data(), ImgSize(_detect_mask));
        if (img_split.size.num_channels > 1)
        {
            if (img_split.size.bytes_per_channel == 1)
            {
                apply3<uint8_t>(img_split, mask_partial, detect_mask_partial, _num_process);
            }
            else
            {
                apply3<uint16_t>(img_split, mask_partial, detect_mask_partial, _num_process);
            }
        }
        else
        {
            if (img_split.size.bytes_per_channel == 1)
            {
                apply1<uint8_t>(img_split, mask_partial, detect_mask_partial, _num_process);
            }
            else
            {
                apply1<uint16_t>(img_split, mask_partial, detect_mask_partial, _num_process);
            }
        }
    }

    template <class T>
    void Vibe::apply3(const Img &_image,
                      Img &_fg_mask,
                      const Img &_detect_mask,
                      int _num_process)
    {
        auto &_bg_img = m_bg_img_samples[_num_process];
        auto &_rnd_gen = m_random_generators[_num_process];
        const auto has_detect_mask = !_detect_mask.empty();

        _fg_mask.clear();

        const int32_t n_color_dist_threshold = sizeof(T) == 1 ? m_params.threshold_color_squared : m_params.threshold_color16_squared;
        const size_t n_samples = m_params.bg_samples;

        // Pre-cache background sample pointers to avoid repeated vector indexing
        // and unique_ptr dereferences in the hot pixel loop
        std::vector<T *> bg_ptrs(n_samples);
        for (size_t s = 0; s < n_samples; ++s)
            bg_ptrs[s] = _bg_img[s]->ptr<T>();

        size_t pix_offset{0}, color_pix_offset{0};
        for (int y{0}; y < _image.size.height; ++y)
        {
            for (int x{0}; x < _image.size.width; ++x, ++pix_offset, color_pix_offset += _image.size.num_channels)
            {
                if (has_detect_mask && (_detect_mask.data[pix_offset] == 0))
                {
                    continue;
                }

                size_t n_good_samples_count{0},
                    n_sample_idx{0};

                const T *const pix_data{&_image.ptr<T>()[color_pix_offset]};

                while (n_sample_idx < n_samples)
                {
                    const T *const bg{&bg_ptrs[n_sample_idx][color_pix_offset]};
                    if (l2_dist3_squared(pix_data, bg) < n_color_dist_threshold)
                    {
                        ++n_good_samples_count;
                        if (n_good_samples_count >= m_params.required_bg_samples)
                        {
                            break;
                        }
                    }
                    ++n_sample_idx;
                }
                if (n_good_samples_count < m_params.required_bg_samples)
                {
                    _fg_mask.data[pix_offset] = UCHAR_MAX;
                }
                else
                {
                    if ((_rnd_gen.fast() & m_params.and_learning_rate) == 0)
                    {
                        T *const bg_img_pix_data{&bg_ptrs[_rnd_gen.fast() & m_params.and_bg_samples][color_pix_offset]};
                        bg_img_pix_data[0] = pix_data[0];
                        bg_img_pix_data[1] = pix_data[1];
                        bg_img_pix_data[2] = pix_data[2];
                    }
                    if ((_rnd_gen.fast() & m_params.and_learning_rate) == 0)
                    {
                        const int neigh_data{get_neighbor_position_3x3(x, y, _image.size, _rnd_gen.fast()) * 3};
                        T *const xy_rand_data{&bg_ptrs[_rnd_gen.fast() & m_params.and_bg_samples][neigh_data]};
                        xy_rand_data[0] = pix_data[0];
                        xy_rand_data[1] = pix_data[1];
                        xy_rand_data[2] = pix_data[2];
                    }
                }
            }
        }
    }

    template <class T>
    void Vibe::apply1(const Img &_image,
                      Img &_fg_mask,
                      const Img &_detect_mask,
                      int _num_process)
    {
        auto &_bg_img = m_bg_img_samples[_num_process];
        auto &_rnd_gen = m_random_generators[_num_process];
        const auto has_detect_mask = !_detect_mask.empty();

        const int32_t n_color_dist_threshold = sizeof(T) == 1 ? m_params.threshold_mono : m_params.threshold_mono16;
        const T *img_ptr = _image.ptr<T>();
        const uint8_t *mask_ptr = has_detect_mask ? _detect_mask.data : nullptr;
        uint8_t *fg_ptr = _fg_mask.data;
        const size_t n_samples = m_params.bg_samples;
        const size_t num_pixels = _image.size.num_pixels;
        const int width = _image.size.width;
        const int height = _image.size.height;

        // Pre-cache background sample pointers
        std::vector<T *> bg_ptrs(n_samples);
        for (size_t s = 0; s < n_samples; ++s)
            bg_ptrs[s] = _bg_img[s]->ptr<T>();

        // Pass 1: Count matching samples per pixel using sample-major loop ordering.
        // The inner pixel loop has no branches, contiguous memory access, and simple
        // arithmetic, allowing the compiler to auto-vectorize it (e.g. 32 uint8 pixels
        // per AVX2 instruction). We reuse fg_mask as the temporary count buffer.
        _fg_mask.clear();
        for (size_t s = 0; s < n_samples; ++s)
        {
            const T *bg = bg_ptrs[s];
            for (size_t i = 0; i < num_pixels; ++i)
            {
                fg_ptr[i] += (std::abs(static_cast<int32_t>(bg[i]) - static_cast<int32_t>(img_ptr[i])) < n_color_dist_threshold);
            }
        }

        // Pass 2: Classify pixels and update background model (sequential due to RNG)
        const uint32_t required = m_params.required_bg_samples;
        size_t pix_offset = 0;
        for (int y = 0; y < height; ++y)
        {
            for (int x = 0; x < width; ++x, ++pix_offset)
            {
                if (has_detect_mask && (mask_ptr[pix_offset] == 0))
                {
                    fg_ptr[pix_offset] = 0;
                    continue;
                }
                if (fg_ptr[pix_offset] < required)
                {
                    fg_ptr[pix_offset] = UCHAR_MAX;
                }
                else
                {
                    fg_ptr[pix_offset] = 0;
                    const T pix_data = img_ptr[pix_offset];
                    if ((_rnd_gen.fast() & m_params.and_learning_rate) == 0)
                    {
                        bg_ptrs[_rnd_gen.fast() & m_params.and_bg_samples][pix_offset] = pix_data;
                    }
                    if ((_rnd_gen.fast() & m_params.and_learning_rate) == 0)
                    {
                        const int neigh_data{get_neighbor_position_3x3(x, y, _image.size, _rnd_gen.fast())};
                        bg_ptrs[_rnd_gen.fast() & m_params.and_bg_samples][neigh_data] = pix_data;
                    }
                }
            }
        }
    }

    void Vibe::get_background_image(cv::Mat &_bg_image)
    {
        cv::Mat avg_bg_img = cv::Mat::zeros(m_orig_img_size->height, m_orig_img_size->width, CV_32FC(m_orig_img_size->num_channels));

        const float inv_bg_samples = 1.0f / static_cast<float>(m_params.bg_samples);

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
                        out_data[c] += static_cast<float>(pix_data[c]) * inv_bg_samples;
                    }
                }
            }
        }

        avg_bg_img.convertTo(_bg_image, CV_8U);
    }
}