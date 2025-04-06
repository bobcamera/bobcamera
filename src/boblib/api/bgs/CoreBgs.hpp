#pragma once

#include "../include/coreUtils.hpp"
#include "../base/Image.hpp"

#include <opencv2/core.hpp>

#include <vector>

namespace boblib::bgs
{
    class CoreParameters;

    class CoreBgs
    {
    public:
        /// Detects the number of available threads to use
        static const size_t DETECT_NUMBER_OF_THREADS{0};

        CoreBgs(bool use_cuda = false, size_t _numProcessesParallel = DETECT_NUMBER_OF_THREADS);

        virtual ~CoreBgs() {}

        void apply(const boblib::base::Image &_image, boblib::base::Image &_fgmask, const boblib::base::Image & _detectMask = boblib::base::Image());

        void restart();

        virtual CoreParameters &get_parameters() = 0;

        virtual void get_background_image(cv::Mat &_bgImage) = 0;

    protected:
        virtual void initialize(const boblib::base::Image &_image) = 0;
        virtual void process(const boblib::base::Image & _image, boblib::base::Image &_fgmask, const boblib::base::Image & _detectMask, int _numProcess) = 0;

        void prepare_parallel(const boblib::base::Image &_image);
        void apply_parallel(const boblib::base::Image &_image, boblib::base::Image &_fgmask, const boblib::base::Image & _detectMask);

        bool using_cuda_;
        size_t m_num_processes_parallel;
        bool m_initialized;
        std::vector<size_t> m_process_seq;
        std::vector<std::unique_ptr<ImgSize>> m_img_sizes_parallel;
        std::unique_ptr<ImgSize> m_original_img_size;
    };
}