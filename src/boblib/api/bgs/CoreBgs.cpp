#include "CoreBgs.hpp"
#include "CoreParameters.hpp"
#include "../base/Utils.hpp"

#include <iostream>
#include <execution>
#include <algorithm>

namespace boblib::bgs
{
    CoreBgs::CoreBgs(bool use_cuda, size_t _numProcessesParallel)
        : using_cuda_(use_cuda ? boblib::base::Utils::HasCuda() : false)
        , m_num_processes_parallel{_numProcessesParallel}
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

    void CoreBgs::apply(const boblib::base::Image &_image, boblib::base::Image &_fgmask, const boblib::base::Image & _detectMask)
    {
        if (!m_initialized || *m_original_img_size != ImgSize(_image))
        {
            prepare_parallel(_image);
            initialize(_image);
            _fgmask.create(_image.size(), CV_8UC1);
            m_initialized = true;
        }

        if (m_num_processes_parallel == 1)
        {
            process(_image, _fgmask, _detectMask, 0);
        }
        else
        {
            apply_parallel(_image, _fgmask, _detectMask);
        }
        _fgmask.upload();
    }

    void CoreBgs::prepare_parallel(const boblib::base::Image & _image)
    {
        m_original_img_size = ImgSize::create(_image);
        m_img_sizes_parallel.resize(m_num_processes_parallel);
        m_process_seq.resize(m_num_processes_parallel);
        size_t y{0};
        size_t h{m_original_img_size->height / m_num_processes_parallel};
        for (size_t i{0}; i < m_num_processes_parallel; ++i)
        {
            m_process_seq[i] = i;
            if (i == (m_num_processes_parallel - 1))
            {
                h = m_original_img_size->height - y;
            }
            m_img_sizes_parallel[i] = ImgSize::create(m_original_img_size->width, h,
                                                    m_original_img_size->num_channels,
                                                    m_original_img_size->bytes_per_channel,
                                                    y * m_original_img_size->width);
            y += h;
        }
    }

    void CoreBgs::apply_parallel(const boblib::base::Image & _image, boblib::base::Image & _fgmask, const boblib::base::Image & _detectMask)
    {
        auto & image = _image.toMat();
        auto & fgmask = _fgmask.toMat();
        std::for_each(
            std::execution::par,
            m_process_seq.begin(),
            m_process_seq.end(),
            [&](int np)
            {
                boblib::base::Image imgSplit(false);
                boblib::base::Image maskPartial(false);
                boblib::base::Image detectMaskPartial(false);
                imgSplit.create(m_img_sizes_parallel[np]->height, m_img_sizes_parallel[np]->width, image.type(),
                                    image.data + (m_img_sizes_parallel[np]->original_pixel_pos * m_img_sizes_parallel[np]->num_channels * m_img_sizes_parallel[np]->bytes_per_channel));
                maskPartial.create(m_img_sizes_parallel[np]->height, m_img_sizes_parallel[np]->width, fgmask.type(),
                                    fgmask.data + m_img_sizes_parallel[np]->original_pixel_pos);
                const boblib::base::Image & detectMask = _detectMask.empty() ? _detectMask : 
                                detectMaskPartial.create(m_img_sizes_parallel[np]->height, m_img_sizes_parallel[np]->width, _detectMask.type(), _detectMask.data() + m_img_sizes_parallel[np]->original_pixel_pos);
                process(imgSplit, maskPartial, detectMask, np);
            });
    }
}