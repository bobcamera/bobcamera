#include "Vibe.hpp"

#include <iostream>
#include <execution>

// #include <opencv2/cudaarithm.hpp>
// #include <cuda_runtime.h>

namespace boblib::bgs
{
    Vibe::Vibe(VibeParams _params, bool use_cuda, size_t _num_processes_parallel)
        : CoreBgs(use_cuda, _num_processes_parallel), m_params(_params), memory_allocated(false)
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
                //apply1_avx2_u8(img_split, mask_partial, detect_mask_partial, _num_process);
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

        size_t pix_offset{0}, color_pix_offset{0};
        for (int y{0}; y < _image.size.height; ++y)
        {
            for (int x{0}; x < _image.size.width; ++x, ++pix_offset, color_pix_offset += _image.size.num_channels)
            {
                if (has_detect_mask && (_detect_mask.ptr<T>()[pix_offset] == 0))
                {
                    continue;
                }

                size_t n_good_samples_count{0},
                    n_sample_idx{0};

                const T *const pix_data{&_image.ptr<T>()[color_pix_offset]};

                while (n_sample_idx < m_params.bg_samples)
                {
                    const T *const bg{&_bg_img[n_sample_idx]->ptr<T>()[color_pix_offset]};
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
                        T *const bg_img_pix_data{&_bg_img[_rnd_gen.fast() & m_params.and_learning_rate]->ptr<T>()[color_pix_offset]};
                        bg_img_pix_data[0] = pix_data[0];
                        bg_img_pix_data[1] = pix_data[1];
                        bg_img_pix_data[2] = pix_data[2];
                    }
                    if ((_rnd_gen.fast() & m_params.and_learning_rate) == 0)
                    {
                        const int neigh_data{get_neighbor_position_3x3(x, y, _image.size, _rnd_gen.fast()) * 3};
                        T *const xy_rand_data{&_bg_img[_rnd_gen.fast() & m_params.and_learning_rate]->ptr<T>()[neigh_data]};
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

        _fg_mask.clear();

        const int32_t n_color_dist_threshold = sizeof(T) == 1 ? m_params.threshold_mono : m_params.threshold_mono16;
        const T *img_ptr = _image.ptr<T>();
        const T *mask_ptr = has_detect_mask ? _detect_mask.ptr<T>() : nullptr;
        uint8_t *fg_ptr = _fg_mask.data;

        size_t pix_offset{0};
        for (int y{0}; y < _image.size.height; ++y)
        {
            for (int x{0}; x < _image.size.width; ++x, ++pix_offset)
            {
                if (has_detect_mask && (mask_ptr[pix_offset] == 0))
                {
                    continue;
                }

                const T pix_data{img_ptr[pix_offset]};

                uint32_t n_good_samples_count{0};
                for (size_t n_sample_idx = 0; n_sample_idx < m_params.bg_samples; ++n_sample_idx)
                {
                    if (std::abs((int32_t)_bg_img[n_sample_idx]->ptr<T>()[pix_offset] - (int32_t)pix_data) < n_color_dist_threshold)
                    {
                        ++n_good_samples_count;
                        if (n_good_samples_count >= m_params.required_bg_samples)
                        {
                            break;
                        }
                    }
                }
                if (n_good_samples_count < m_params.required_bg_samples)
                {
                    fg_ptr[pix_offset] = UCHAR_MAX;
                }
                else
                {
                    if ((_rnd_gen.fast() & m_params.and_learning_rate) == 0)
                    {
                        _bg_img[_rnd_gen.fast() & m_params.and_learning_rate]->ptr<T>()[pix_offset] = pix_data;
                    }
                    if ((_rnd_gen.fast() & m_params.and_learning_rate) == 0)
                    {
                        const int neigh_data{get_neighbor_position_3x3(x, y, _image.size, _rnd_gen.fast())};
                        _bg_img[_rnd_gen.fast() & m_params.and_learning_rate]->ptr<T>()[neigh_data] = pix_data;
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

#include <immintrin.h>

    void Vibe::apply1_avx2_u8(const Img &_image,
                              Img &_fg_mask,
                              const Img &_detect_mask,
                              int _num_process)
    {
        auto &_bg_img = m_bg_img_samples[_num_process];
        auto &thread_rng = m_random_generators[_num_process];
        const auto has_detect_mask = !_detect_mask.empty();

        _fg_mask.clear();

        const int32_t n_color_dist_threshold = m_params.threshold_mono;
        const uint8_t *img_ptr = _image.ptr<uint8_t>();
        const uint8_t *mask_ptr = has_detect_mask ? _detect_mask.ptr<uint8_t>() : nullptr;
        uint8_t *fg_ptr = _fg_mask.data;

        const int width = _image.size.width;
        const int height = _image.size.height;
        const size_t bg_samples = m_params.bg_samples;
        const uint32_t required_bg_samples = m_params.required_bg_samples;
        const uint32_t and_learning_rate = m_params.and_learning_rate;

        // Prepare background pointers for faster access
        alignas(32) std::vector<const uint8_t *> bg_ptrs(bg_samples);
        for (size_t s = 0; s < bg_samples; ++s)
        {
            bg_ptrs[s] = _bg_img[s]->ptr<uint8_t>();
        }

        // Threshold vector for SIMD comparison
        __m256i threshold_vec = _mm256_set1_epi8(static_cast<char>(n_color_dist_threshold));

//#pragma omp parallel for schedule(dynamic) if (_image.size.width * _image.size.height > 100000)
        for (int y = 0; y < height; ++y)
        {
            // Process row
            for (int x = 0; x < width; ++x)
            {
                size_t pix_offset = y * width + x;

                // Skip masked pixels
                if (has_detect_mask && (mask_ptr[pix_offset] == 0))
                {
                    continue;
                }

                const uint8_t pix_data = img_ptr[pix_offset];

                // Try scalar processing first for the first few samples to benefit from early exit
                uint32_t n_good_samples_count = 0;
                size_t n_sample_idx = 0;

                // Process first 8 samples with scalar code - often enough to make a decision
                const size_t scalar_samples = std::min(bg_samples, size_t(8));
                for (; n_sample_idx < scalar_samples; ++n_sample_idx)
                {
                    if (std::abs((int)bg_ptrs[n_sample_idx][pix_offset] - (int)pix_data) < n_color_dist_threshold)
                    {
                        ++n_good_samples_count;
                        if (n_good_samples_count >= required_bg_samples)
                        {
                            break; // Early exit if we have enough matches
                        }
                    }
                }

                // If we don't have enough matches and there are more samples, use SIMD
                if (n_good_samples_count < required_bg_samples && n_sample_idx < bg_samples)
                {
                    // Create pixel vector once for all comparisons
                    __m256i pixel_vec = _mm256_set1_epi8(static_cast<char>(pix_data));

                    // Process remaining samples in chunks of 32
                    alignas(32) uint8_t local_sample_values[32];

                    while (n_sample_idx + 32 <= bg_samples && n_good_samples_count < required_bg_samples)
                    {
// Fast gather - directly load into local buffer with fewer branches
//#pragma unroll(8)
                        for (int i = 0; i < 32; i += 4)
                        {
                            local_sample_values[i] = bg_ptrs[n_sample_idx + i][pix_offset];
                            local_sample_values[i + 1] = bg_ptrs[n_sample_idx + i + 1][pix_offset];
                            local_sample_values[i + 2] = bg_ptrs[n_sample_idx + i + 2][pix_offset];
                            local_sample_values[i + 3] = bg_ptrs[n_sample_idx + i + 3][pix_offset];
                        }

                        // Process 32 samples in parallel
                        __m256i bg_vals = _mm256_load_si256(reinterpret_cast<__m256i *>(local_sample_values));
                        __m256i diff = _mm256_or_si256(
                            _mm256_subs_epu8(pixel_vec, bg_vals), // pixel > bg
                            _mm256_subs_epu8(bg_vals, pixel_vec)  // bg > pixel
                        );
                        __m256i cmp_mask = _mm256_cmpgt_epi8(threshold_vec, diff);
                        uint32_t matches = _mm_popcnt_u32(_mm256_movemask_epi8(cmp_mask));

                        n_good_samples_count += matches;
                        n_sample_idx += 32;
                    }

                    // Handle remaining samples with scalar code
                    while (n_sample_idx < bg_samples && n_good_samples_count < required_bg_samples)
                    {
                        if (std::abs((int)bg_ptrs[n_sample_idx][pix_offset] - (int)pix_data) < n_color_dist_threshold)
                        {
                            ++n_good_samples_count;
                        }
                        ++n_sample_idx;
                    }
                }

                // Update foreground mask and model
                if (n_good_samples_count < required_bg_samples)
                {
                    fg_ptr[pix_offset] = UCHAR_MAX;
                }
                else
                {
                    // Use thread_rng for consistent random numbers
                    if ((thread_rng.fast() & and_learning_rate) == 0)
                    {
                        uint32_t model_idx = thread_rng.fast() & and_learning_rate;
                        _bg_img[model_idx]->ptr<uint8_t>()[pix_offset] = pix_data;
                    }

                    if ((thread_rng.fast() & and_learning_rate) == 0)
                    {
                        const int neigh_data = get_neighbor_position_3x3(x, y, _image.size, thread_rng.fast());
                        uint32_t model_idx = thread_rng.fast() & and_learning_rate;
                        _bg_img[model_idx]->ptr<uint8_t>()[neigh_data] = pix_data;
                    }
                }
            }
        }
    }
}