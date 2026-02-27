#pragma once

#include "../../base/Image.hpp"
#include "../CoreBgs.hpp"
#include "../CoreParameters.hpp"
#include "VibeUtils.hpp"
#include "../../include/pcg32.hpp"

namespace boblib::bgs
{
    class Vibe
        : public CoreBgs
    {
    public:
        Vibe(VibeParams _params = VibeParams(),
             bool use_cuda = true,
             size_t _numProcessesParallel = DETECT_NUMBER_OF_THREADS);

        ~Vibe();

        VibeParams &get_parameters() override { return m_params; }

        void get_background_image(cv::Mat &_bgImage) override;

    private:
        void initialize(const boblib::base::Image & oInitImg) override;
        void process(const boblib::base::Image & _image, boblib::base::Image & _fgmask, const boblib::base::Image & _detectMask, int _numProcess) override;

        VibeParams m_params;

        std::unique_ptr<ImgSize> m_orig_img_size;
        std::vector<std::vector<std::unique_ptr<Img>>> m_bg_img_samples;
        std::vector<Pcg32> m_random_generators;

        template<class T>
        void initialize(const Img &_initImg, std::vector<std::unique_ptr<Img>> &_bg_img_samples, Pcg32 &_rnd_gen);
        
        template<class T>
        void apply1(const Img &_image, Img &_fg_mask, const Img & _detect_mask, int _num_process);
        template<class T>
        void apply3(const Img &_image, Img &_fg_mask, const Img & _detect_mask, int _num_process);
    };
}