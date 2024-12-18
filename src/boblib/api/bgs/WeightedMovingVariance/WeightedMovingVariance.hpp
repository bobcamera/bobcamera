#pragma once

#include "../CoreBgs.hpp"
#include "../CoreParameters.hpp"
#include "WeightedMovingVarianceUtils.hpp"

#include <opencv2/opencv.hpp>

#include <array>
#include <vector>

namespace boblib::bgs
{
    class WeightedMovingVariance final
        : public CoreBgs 
    {
    public:
        WeightedMovingVariance(WMVParams _params = WMVParams(), bool use_cuda = true, size_t _num_processes_parallel = DETECT_NUMBER_OF_THREADS);
        ~WeightedMovingVariance();

        virtual WMVParams &get_parameters() { return m_params; }

        virtual void get_background_image(cv::Mat &_bgImage);

    private:
        virtual void initialize(const boblib::base::Image &_image);
        virtual void process(const boblib::base::Image &img_input, boblib::base::Image &img_output, const boblib::base::Image & _detectMask, int _num_process);

        static const inline int ROLLING_BG_IDX[3][3] = {{0, 1, 2}, {2, 0, 1}, {1, 2, 0}};

        WMVParams m_params;

        struct RollingImages
        {
            size_t current_rolling_idx;
            int first_phase;
            ImgSize* p_img_size;
            uint8_t* p_img_input;
            uint8_t* p_img_input_prev1;
            uint8_t* p_img_input_prev2;

            std::array<std::unique_ptr<uint8_t[]>, 3> p_img_mem;
        };
        std::vector<RollingImages> m_img_input_prev;

        static void roll_images(RollingImages& _rolling_images);
                    
        template<class T>
        static void weighted_variance_mono(
            const T *const _img1,
            const T *const _img2,
            const T *const _img3,
            const uint8_t *const _imgMask,
            uint8_t *const _out_Img,
            const size_t _total_pixels,
            const float* _weight, 
            const bool _enable_threshold,
            const float _threshold_squared);
        template<class T>
        static void weighted_variance_color(
            const T *const _img1,
            const T *const _img2,
            const T *const _img3,
            const uint8_t *const _imgMask,
            uint8_t *const _out_img,
            const size_t _total_pixels,
            const float* _weight, 
            const bool _enable_threshold,
            const float _threshold_squared);
    };
}
