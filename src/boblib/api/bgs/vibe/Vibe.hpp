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

        virtual VibeParams &get_parameters() { return m_params; }

        virtual void get_background_image(cv::Mat &_bgImage);

    private:
        virtual void initialize(const boblib::base::Image & oInitImg);
        virtual void process(const boblib::base::Image & _image, boblib::base::Image & _fgmask, const boblib::base::Image & _detectMask, int _numProcess);

        void allocate_memory_if_needed(const boblib::base::Image &_image);
        void free_memory();
        void process_cuda(const boblib::base::Image & _image, boblib::base::Image & _fgmask, const boblib::base::Image & _detectMask);

        VibeParams m_params;

        std::unique_ptr<ImgSize> m_orig_img_size;
        std::vector<std::vector<std::unique_ptr<Img>>> m_bg_img_samples;
        std::vector<Pcg32> m_random_generators;
        bool memory_allocated;

        template<class T>
        void initialize(const Img &_initImg, std::vector<std::unique_ptr<Img>> &_bg_img_samples, Pcg32 &_rnd_gen);
        
        template<class T>
        void apply1(const Img &_image, Img &_fg_mask, const Img & _detect_mask, int _num_process);
        template<class T>
        void apply3(const Img &_image, Img &_fg_mask, const Img & _detect_mask, int _num_process);
    };
}