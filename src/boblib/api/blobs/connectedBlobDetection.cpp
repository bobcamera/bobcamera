#include "connectedBlobDetection.hpp"

#include <opencv2/imgproc.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudaarithm.hpp>

#include <iostream>
#include <execution>
#include <algorithm>

namespace boblib::blobs
{
    ConnectedBlobDetection::ConnectedBlobDetection(const ConnectedBlobDetectionParams &_params, size_t _num_processes_parallel)
        : m_params{_params}, m_num_processes_parallel{_num_processes_parallel}, m_initialized{false}
    {
        if (m_num_processes_parallel == DETECT_NUMBER_OF_THREADS)
        {
            m_num_processes_parallel = calc_available_threads();
        }
    }

    inline bool rects_overlap(const cv::Rect &r1, const cv::Rect &r2)
    {
        if ((r1.width == 0 || r1.height == 0 || r2.width == 0 || r2.height == 0) ||
            (r1.x > (r2.x + r2.width) || r2.x > (r1.x + r1.width)) ||
            (r1.y > (r2.y + r2.height) || r2.y > (r1.y + r1.height)))
        {
            return false;
        }

        return true;
    }

    inline float rects_distance_squared(const cv::Rect &r1, const cv::Rect &r2)
    {
        if (rects_overlap(r1, r2))
        {
            return 0;
        }

        const int x_distance = std::max(0, std::max(r1.x, r2.x) - std::min(r1.x + r1.width, r2.x + r2.width));
        const int y_distance = std::max(0, std::max(r1.y, r2.y) - std::min(r1.y + r1.height, r2.y + r2.height));

        return (x_distance * x_distance) + (y_distance * y_distance);
    }

    // Joining bboxes together if they overlap
    inline void join_bboxes(std::unordered_map<int, cv::Rect> &_bboxes, int minDistanceSquared)
    {
        bool bboxOverlap;
        do
        {
            bboxOverlap = false;
            for (auto it1 = _bboxes.begin(); it1 != _bboxes.end(); ++it1)
            {
                auto it2 = std::next(it1); // Start checking from the next element
                while (it2 != _bboxes.end())
                {
                    if (rects_distance_squared(it1->second, it2->second) < minDistanceSquared)
                    {
                        bboxOverlap = true;

                        // Calculate new bounding box coordinates
                        const int xmax = std::max(it1->second.x + it1->second.width - 1, it2->second.x + it2->second.width - 1);
                        const int ymax = std::max(it1->second.y + it1->second.height - 1, it2->second.y + it2->second.height - 1);
                        it1->second.x = std::min(it1->second.x, it2->second.x);
                        it1->second.y = std::min(it1->second.y, it2->second.y);
                        it1->second.width = (xmax - it1->second.x) + 1;
                        it1->second.height = (ymax - it1->second.y) + 1;

                        // Erase the overlapping bounding box
                        it2 = _bboxes.erase(it2); // Returns the iterator to the next element after erasing
                    }
                    else
                    {
                        ++it2; // Only increment if no merge occurred
                    }
                }
            }
        } while (bboxOverlap);
    }

    inline static void apply_size_cut(std::unordered_map<int, cv::Rect> &_bboxes, const int _sizeThreshold, const int _areaThreshold)
    {
        for (auto it = _bboxes.begin(); it != _bboxes.end();)
        {
            if ((it->second.width < _sizeThreshold) || (it->second.height < _sizeThreshold) || (it->second.area() < _areaThreshold))
            {
                it = _bboxes.erase(it); // Erase and update the iterator
            }
            else
            {
                ++it; // Only increment if not erased
            }
        }
    }

    inline void ConnectedBlobDetection::pos_process_bboxes(std::unordered_map<int, cv::Rect> &_bboxes)
    {
        _bboxes.clear();

        // Joining all parallel bboxes into one label
        for (size_t i{0}; i < m_num_processes_parallel; ++i)
        {
            const int addedY = (int)(m_img_sizes_parallel[i]->original_pixel_pos / m_img_sizes_parallel[i]->width);

            for (auto &[label, bbox] : m_bboxes_parallel[i])
            {
                auto it = _bboxes.find(label);
                if (it == _bboxes.end())
                {
                    _bboxes[label].x = bbox.x;
                    _bboxes[label].y = bbox.y + addedY;
                    _bboxes[label].width = bbox.width;
                    _bboxes[label].height = bbox.height;
                }
                else
                {
                    const int x_max = std::max(it->second.x + it->second.width - 1, bbox.x + bbox.width - 1);
                    const int y_max = std::max(it->second.y + it->second.height - 1, bbox.y + bbox.height - 1 + addedY);

                    it->second.x = std::min(it->second.x, bbox.x);
                    it->second.y = std::min(it->second.y, bbox.y + addedY);
                    it->second.width = (x_max - it->second.x) + 1;
                    it->second.height = (y_max - it->second.y) + 1;
                }
            }
        }

        // Joining bboxes that are overlapping each other
        join_bboxes(_bboxes, m_params.min_distance_squared);

        // Removing bboxes that are below the threshold
        apply_size_cut(_bboxes, m_params.size_threshold, m_params.area_threshold);
    }

    // Finds the connected components in the image and returns a list of bounding boxes
    DetectionResult ConnectedBlobDetection::detect(const boblib::base::Image &_image, std::vector<cv::Rect> &_bboxes)
    {
        if (!m_initialized || *m_original_img_size != ImgSize(_image.size().width, _image.size().height, _image.channels(), _image.elemSize1(), 0))
        {
            prepare_parallel(_image);
            m_initialized = true;
        }

        cv::Mat labels;
        std::unordered_map<int, cv::Rect> bboxes_map;
        int num_labels(0);
        int num_blobs(0);

        // Use connected component analysis to find the blobs in the image
        if (_image.get_using_cuda())
        {
            cv::cuda::GpuMat gpu_labels;
            cv::cuda::connectedComponents(_image.toCudaMat(), gpu_labels, 8, CV_32S);
            gpu_labels.download(labels);

            num_blobs = run_parallel(labels, bboxes_map);
        }
        else
        {
            num_labels = cv::connectedComponents(_image.toMat(), labels, 8, CV_32S, cv::CCL_SPAGHETTI) - 1; // subtract 1 because the background is considered as label 0
            if ((num_labels > 0) && (num_labels < m_params.max_labels))
            {
                num_blobs = run_parallel(labels, bboxes_map);
            }
        }

        if ((num_blobs > 0) && (num_blobs <= m_params.max_blobs))
        {
            _bboxes.clear();
            _bboxes.reserve(bboxes_map.size());
            for (const auto& pair : bboxes_map) 
            {
                _bboxes.push_back(pair.second);
            }
            
            return DetectionResult::Success;
        }
        else if (num_blobs == 0)
        {
            return DetectionResult::NoBlobsDetected;
        }
        else
        {
            return DetectionResult::MaxBlobsReached;
        }
    }

    inline int ConnectedBlobDetection::run_parallel(const cv::Mat &labels, std::unordered_map<int, cv::Rect> &_bboxes)
    {
        // Parallel execution to process each chunk of the image
        std::for_each(
            std::execution::par,
            m_process_seq.begin(),
            m_process_seq.end(),
            [&](int np)
            {
                std::unordered_map<int, cv::Rect> &bboxesParallel = m_bboxes_parallel[np];
                bboxesParallel.clear();

                // Splitting the image into chunks and processing
                const cv::Mat imgSplit(
                    m_img_sizes_parallel[np]->height,
                    m_img_sizes_parallel[np]->width,
                    labels.type(),
                    labels.data + (m_img_sizes_parallel[np]->original_pixel_pos * m_img_sizes_parallel[np]->num_channels));
                apply_detect_bboxes(imgSplit, bboxesParallel);
            });

        // Post-process the bounding boxes after parallel execution
        pos_process_bboxes(_bboxes);

        return static_cast<int>(_bboxes.size());
    }

    inline void ConnectedBlobDetection::apply_detect_bboxes(const cv::Mat &_labels, std::unordered_map<int, cv::Rect> &_bboxes)
    {
        int *pLabel = (int *)_labels.data;
        for (int y = 0; y < _labels.rows; ++y)
        {
            for (int x = 0; x < _labels.cols; ++x)
            {
                const int label = *pLabel - 1;
                if (label >= 0)
                {
                    auto it = _bboxes.find(label);
                    if (it == _bboxes.end())
                    {
                        _bboxes[label].x = x;
                        _bboxes[label].y = y;
                        _bboxes[label].width = 1;
                        _bboxes[label].height = 1;
                    }
                    else
                    {
                        const int x_max = std::max(it->second.x + it->second.width - 1, x);
                        const int y_max = std::max(it->second.y + it->second.height - 1, y);
                        it->second.x = std::min(it->second.x, x);
                        it->second.y = std::min(it->second.y, y);
                        it->second.width = (x_max - it->second.x) + 1;
                        it->second.height = (y_max - it->second.y) + 1;
                        // std::cout << "bbox: " << _bboxes[label].x << ", " << _bboxes[label].y << ", " << _bboxes[label].width << ", " << _bboxes[label].height << std::endl;
                    }
                }
                ++pLabel;
            }
        }
    }

    void ConnectedBlobDetection::prepare_parallel(const boblib::base::Image &_image)
    {
        m_original_img_size = ImgSize::create(_image.size().width, _image.size().height,
                                              _image.channels(),
                                              _image.elemSize1(),
                                              0);
        m_img_sizes_parallel.resize(m_num_processes_parallel);
        m_process_seq.resize(m_num_processes_parallel);
        m_bboxes_parallel.resize(m_num_processes_parallel);
        size_t y{0};
        size_t h{_image.size().height / m_num_processes_parallel};
        for (size_t i{0}; i < m_num_processes_parallel; ++i)
        {
            m_process_seq[i] = i;
            if (i == (m_num_processes_parallel - 1))
            {
                h = _image.size().height - y;
            }
            m_img_sizes_parallel[i] = ImgSize::create(_image.size().width, h,
                                                      4, 1,
                                                      y * _image.size().width);
            y += h;
        }
    }

}