#include "WeightedMovingVariance.hpp"

#include <opencv2/imgproc/types_c.h>
#include <execution>
#include <iostream>

static const uint8_t ZERO_UC{0};

namespace boblib::bgs
{
    WeightedMovingVariance::WeightedMovingVariance(WMVParams _params, bool use_cuda, size_t _num_processes_parallel)
        : CoreBgs(use_cuda, _num_processes_parallel),
        m_params{_params}
    {
        m_params.set_bgs(this);
    }

    WeightedMovingVariance::~WeightedMovingVariance()
    {
    }

    void WeightedMovingVariance::get_background_image(cv::Mat &)
    {
        // Not implemented
    }

    void WeightedMovingVariance::initialize(const boblib::base::Image &)
    {
        m_img_input_prev.resize(m_num_processes_parallel);
        for (size_t i = 0; i < m_num_processes_parallel; ++i)
        {
            m_img_input_prev[i].current_rolling_idx = 0;
            m_img_input_prev[i].first_phase = 0;
            m_img_input_prev[i].p_img_size = m_img_sizes_parallel[i].get();
            m_img_input_prev[i].p_img_input = nullptr;
            m_img_input_prev[i].p_img_input_prev1 = nullptr;
            m_img_input_prev[i].p_img_input_prev2 = nullptr;
            m_img_input_prev[i].p_img_mem[0] = std::make_unique_for_overwrite<uint8_t[]>(m_img_input_prev[i].p_img_size->size_in_bytes);
            m_img_input_prev[i].p_img_mem[1] = std::make_unique_for_overwrite<uint8_t[]>(m_img_input_prev[i].p_img_size->size_in_bytes);
            m_img_input_prev[i].p_img_mem[2] = std::make_unique_for_overwrite<uint8_t[]>(m_img_input_prev[i].p_img_size->size_in_bytes);
            roll_images(m_img_input_prev[i]);
        }
    }

    void WeightedMovingVariance::roll_images(RollingImages &_rolling_images)
    {
        const auto rolling_idx = ROLLING_BG_IDX[_rolling_images.current_rolling_idx % 3];
        _rolling_images.p_img_input = _rolling_images.p_img_mem[rolling_idx[0]].get();
        _rolling_images.p_img_input_prev1 = _rolling_images.p_img_mem[rolling_idx[1]].get();
        _rolling_images.p_img_input_prev2 = _rolling_images.p_img_mem[rolling_idx[2]].get();

        ++_rolling_images.current_rolling_idx;
    }

    void WeightedMovingVariance::process(const boblib::base::Image &_img_input, boblib::base::Image &_img_output, const boblib::base::Image & _detectMask, int _num_process)
    {
        auto & img_input = _img_input.toMat();
        if (_img_output.empty())
        {
            _img_output.create(img_input.size(), CV_8UC1);
        }
        auto & img_output = _img_output.toMat();

        auto & _img_input_prev =  m_img_input_prev[_num_process];
        memcpy(_img_input_prev.p_img_input, img_input.data, _img_input_prev.p_img_size->size_in_bytes);

        if (_img_input_prev.first_phase < 2)
        {
            ++_img_input_prev.first_phase;
            return;
        }

        uint8_t* detectMaskPtr = _detectMask.empty() ? nullptr : _detectMask.data();

        if (_img_input_prev.p_img_size->num_channels == 1)
        {
            if (_img_input_prev.p_img_size->bytes_per_channel == 1)
            {
                weighted_variance_mono(_img_input_prev.p_img_input, _img_input_prev.p_img_input_prev1, _img_input_prev.p_img_input_prev2, detectMaskPtr,
                                    img_output.data, (size_t)_img_input_prev.p_img_size->num_pixels, 
                                    m_params.weight, m_params.enable_threshold, m_params.threshold_squared);
            }
            else
            {
                weighted_variance_mono((uint16_t*)_img_input_prev.p_img_input, (uint16_t*)_img_input_prev.p_img_input_prev1, (uint16_t*)_img_input_prev.p_img_input_prev2, detectMaskPtr,
                                    img_output.data, (size_t)_img_input_prev.p_img_size->num_pixels, 
                                    m_params.weight, m_params.enable_threshold, m_params.threshold_squared16);
            }
        }
        else
        {
            if (_img_input_prev.p_img_size->bytes_per_channel == 1)
            {
                weighted_variance_color(_img_input_prev.p_img_input, _img_input_prev.p_img_input_prev1, _img_input_prev.p_img_input_prev2, detectMaskPtr,
                                    img_output.data, (size_t)_img_input_prev.p_img_size->num_pixels, 
                                    m_params.weight, m_params.enable_threshold, m_params.threshold_squared);
            }
            else
            {
                weighted_variance_color((uint16_t*)_img_input_prev.p_img_input, (uint16_t*)_img_input_prev.p_img_input_prev1, (uint16_t*)_img_input_prev.p_img_input_prev2, detectMaskPtr,
                                    img_output.data, (size_t)_img_input_prev.p_img_size->num_pixels, 
                                    m_params.weight, m_params.enable_threshold, m_params.threshold_squared16);
            }
        }
    
        roll_images(m_img_input_prev[_num_process]);
    }

    template<class T>
    inline void calc_weighted_variance_mono(const T *const _i1, const T *const _i2, const T *const _i3, const uint8_t *const _imgMask,
                                        uint8_t *const _o, uint32_t _total_pixels, const float* _weight)
    {
        for (uint32_t i{0}; i < _total_pixels; ++i)
        {
            if (_imgMask && (_imgMask[i] == 0))
            {
                _o[i] = ZERO_UC;
                continue;
            }
            const float dI[]{(float)_i1[i], (float)_i2[i], (float)_i3[i]};
            const float mean{(dI[0] * _weight[0]) + (dI[1] * _weight[1]) + (dI[2] * _weight[2])};
            const float value[]{dI[0] - mean, dI[1] - mean, dI[2] - mean};
            _o[i] = std::sqrt(((value[0] * value[0]) * _weight[0]) + ((value[1] * value[1]) * _weight[1]) + ((value[2] * value[2]) * _weight[2]));
        }
    }

    template<class T>
    inline void calc_weighted_variance_mono_threshold(const T *const _i1, const T *const _i2, const T *const _i3, const uint8_t *const _imgMask,
                                                uint8_t *const _o, uint32_t _total_pixels, 
                                                const float* _weight, const float _threshold_squared)
    {
        for (uint32_t i{0}; i < _total_pixels; ++i)
        {
            if (_imgMask && (_imgMask[i] == 0))
            {
                _o[i] = ZERO_UC;
                continue;
            }
            const float dI[]{(float)_i1[i], (float)_i2[i], (float)_i3[i]};
            const float mean{(dI[0] * _weight[0]) + (dI[1] * _weight[1]) + (dI[2] * _weight[2])};
            const float value[]{dI[0] - mean, dI[1] - mean, dI[2] - mean};
            const float result{((value[0] * value[0]) * _weight[0]) + ((value[1] * value[1]) * _weight[1]) + ((value[2] * value[2]) * _weight[2])};
            _o[i] = result > _threshold_squared ? UCHAR_MAX : ZERO_UC;
        }
    }

    template<class T>
    inline void calc_weighted_variance_color(const T *const _i1, const T *const _i2, const T *const _i3, const uint8_t *const _imgMask,
                                        uint8_t *const _o, uint32_t _total_pixels, 
                                        const float* _weight)
    {
        for (uint32_t j{0}, j3{0}; j < _total_pixels; ++j, j3 += 3)
        {
            if (_imgMask && (_imgMask[j] == 0))
            {
                _o[j] = ZERO_UC;
                continue;
            }
            const float dI1[]{(float)_i1[j3], (float)_i1[j3 + 1], (float)_i1[j3 + 2]};
            const float dI2[]{(float)_i2[j3], (float)_i2[j3 + 1], (float)_i2[j3 + 2]};
            const float dI3[]{(float)_i3[j3], (float)_i3[j3 + 1], (float)_i3[j3 + 2]};
            const float meanR{(dI1[0] * _weight[0]) + (dI2[0] * _weight[1]) + (dI3[0] * _weight[2])};
            const float meanG{(dI1[1] * _weight[0]) + (dI2[1] * _weight[1]) + (dI3[1] * _weight[2])};
            const float meanB{(dI1[2] * _weight[0]) + (dI2[2] * _weight[1]) + (dI3[2] * _weight[2])};
            const float valueR[]{dI1[0] - meanR, dI2[0] - meanR, dI2[0] - meanR};
            const float valueG[]{dI1[1] - meanG, dI2[1] - meanG, dI2[1] - meanG};
            const float valueB[]{dI1[2] - meanB, dI2[2] - meanB, dI2[2] - meanB};
            const float r{std::sqrt(((valueR[0] * valueR[0]) * _weight[0]) + ((valueR[1] * valueR[1]) * _weight[1]) + ((valueR[2] * valueR[2]) * _weight[2]))};
            const float g{std::sqrt(((valueG[0] * valueG[0]) * _weight[0]) + ((valueG[1] * valueG[1]) * _weight[1]) + ((valueG[2] * valueG[2]) * _weight[2]))};
            const float b{std::sqrt(((valueB[0] * valueB[0]) * _weight[0]) + ((valueB[1] * valueB[1]) * _weight[1]) + ((valueB[2] * valueB[2]) * _weight[2]))};
            _o[j] = 0.299f * r + 0.587f * g + 0.114f * b;
        }
    }

    template<class T>
    inline void calc_weighted_variance_color_threshold(const T *const _i1, const T *const _i2, const T *const _i3, const uint8_t *const _imgMask,
                                                uint8_t *const _o, uint32_t _total_pixels, 
                                                const float* _weight, const float thresholdSquared)
    {
        for (uint32_t j{0}, j3{0}; j < _total_pixels; ++j, j3 += 3)
        {
            if (_imgMask && (_imgMask[j] == 0))
            {
                _o[j] = ZERO_UC;
                continue;
            }
            const float dI1[]{(float)_i1[j3], (float)_i1[j3 + 1], (float)_i1[j3 + 2]};
            const float dI2[]{(float)_i2[j3], (float)_i2[j3 + 1], (float)_i2[j3 + 2]};
            const float dI3[]{(float)_i3[j3], (float)_i3[j3 + 1], (float)_i3[j3 + 2]};
            const float meanR{(dI1[0] * _weight[0]) + (dI2[0] * _weight[1]) + (dI3[0] * _weight[2])};
            const float meanG{(dI1[1] * _weight[0]) + (dI2[1] * _weight[1]) + (dI3[1] * _weight[2])};
            const float meanB{(dI1[2] * _weight[0]) + (dI2[2] * _weight[1]) + (dI3[2] * _weight[2])};
            const float valueR[]{dI1[0] - meanR, dI2[0] - meanR, dI2[0] - meanR};
            const float valueG[]{dI1[1] - meanG, dI2[1] - meanG, dI2[1] - meanG};
            const float valueB[]{dI1[2] - meanB, dI2[2] - meanB, dI2[2] - meanB};
            const float r2{((valueR[0] * valueR[0]) * _weight[0]) + ((valueR[1] * valueR[1]) * _weight[1]) + ((valueR[2] * valueR[2]) * _weight[2])};
            const float g2{((valueG[0] * valueG[0]) * _weight[0]) + ((valueG[1] * valueG[1]) * _weight[1]) + ((valueG[2] * valueG[2]) * _weight[2])};
            const float b2{((valueB[0] * valueB[0]) * _weight[0]) + ((valueB[1] * valueB[1]) * _weight[1]) + ((valueB[2] * valueB[2]) * _weight[2])};
            const float result{0.299f * r2 + 0.587f * g2 + 0.114f * b2};
            _o[j] = result > thresholdSquared ? UCHAR_MAX : ZERO_UC;
        }
    }

    template<class T>
    void WeightedMovingVariance::weighted_variance_mono(
        const T *const _img1,
        const T *const _img2,
        const T *const _img3,
        const uint8_t *const _imgMask,
        uint8_t *const _out_img,
        const size_t _total_pixels,
        const float* _weight,
        const bool _enable_threshold, 
        const float _threshold_squared)
    {
        if (_enable_threshold)
            calc_weighted_variance_mono_threshold(_img1, _img2, _img3, _imgMask, _out_img, _total_pixels, _weight, _threshold_squared);
        else
            calc_weighted_variance_mono(_img1, _img2, _img3, _imgMask, _out_img, _total_pixels, _weight);
    }

    template<class T>
    void WeightedMovingVariance::weighted_variance_color(
        const T *const _img1,
        const T *const _img2,
        const T *const _img3,
        const uint8_t *const _imgMask,
        uint8_t *const _out_img,
        const size_t _total_pixels,
        const float* _weight, 
        const bool _enable_threshold,
        const float _threshold_squared)
    {
        if (_enable_threshold)
            calc_weighted_variance_color_threshold(_img1, _img2, _img3, _imgMask, _out_img, _total_pixels, _weight, _threshold_squared);
        else
            calc_weighted_variance_color(_img1, _img2, _img3, _imgMask, _out_img, _total_pixels, _weight);
    }
}