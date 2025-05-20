#include "CoreBgs.hpp"
#include "CoreParameters.hpp"
#include "../base/Utils.hpp"

#include <iostream>
#include <execution>
#include <algorithm>

namespace boblib::bgs
{
    CoreBgs::CoreBgs(bool use_cuda, size_t _numProcessesParallel)
        : using_cuda_(use_cuda && false ? boblib::base::Utils::has_cuda() : false) // Forcing false for now since it is not done
          ,
          m_num_processes_parallel{_numProcessesParallel}, m_initialized{false}
    {
        if (_numProcessesParallel == DETECT_NUMBER_OF_THREADS)
        {
            m_num_processes_parallel = boblib::base::Utils::get_available_threads() / 2;
        }
    }

    void CoreBgs::restart()
    {
        m_initialized = false;
    }

    void CoreBgs::apply(const boblib::base::Image &_image, boblib::base::Image &_fgmask, const boblib::base::Image & _detectMask)
    {
        if (!using_cuda_)
        {
            _fgmask.download();
        }
        else
        {
            _fgmask.upload();
        }
        if (!m_initialized || *m_original_img_size != ImgSize(_image))
        {
            prepare_parallel(_image);
            initialize(_image);
            m_initialized = true;
        }
        if (_fgmask.empty())
        {
            _fgmask.create(_image.size(), CV_8UC1);
        }

        if (m_num_processes_parallel == 1)
        {
            process(_image, _fgmask, _detectMask, 0);
        }
        else
        {
            apply_parallel(_image, _fgmask, _detectMask);
        }
        if (!using_cuda_)
        {
            _fgmask.upload();
        }
        else
        {
            _fgmask.download();
        }
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

        std::vector<boblib::base::Image> imgSplits(m_num_processes_parallel, false);
        std::vector<boblib::base::Image> maskPartials(m_num_processes_parallel, false);
        std::vector<boblib::base::Image> detectMaskPartials(m_num_processes_parallel, false);
        for (size_t np = 0; np < m_num_processes_parallel; ++np)
        {
            imgSplits[np].create(m_img_sizes_parallel[np]->height, m_img_sizes_parallel[np]->width, image.type(),
                            image.data + (m_img_sizes_parallel[np]->original_pixel_pos * m_img_sizes_parallel[np]->num_channels * m_img_sizes_parallel[np]->bytes_per_channel));
            maskPartials[np].create(m_img_sizes_parallel[np]->height, m_img_sizes_parallel[np]->width, fgmask.type(),
                               fgmask.data + m_img_sizes_parallel[np]->original_pixel_pos);
            if (_detectMask.empty())
            {
                detectMaskPartials[np] = _detectMask;
            }
            else
            {
                detectMaskPartials[np].create(m_img_sizes_parallel[np]->height, m_img_sizes_parallel[np]->width, _detectMask.type(), _detectMask.data() + m_img_sizes_parallel[np]->original_pixel_pos);
            }
        }

        std::for_each(
            std::execution::par,
            m_process_seq.begin(),
            m_process_seq.end(),
            [&](int np)
            {
                process(imgSplits[np], maskPartials[np], detectMaskPartials[np], np);
            });
    }
}