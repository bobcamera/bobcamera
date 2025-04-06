#include "connectedBlobDetection.hpp"

#include <opencv2/imgproc.hpp>
// #include <opencv2/cudaimgproc.hpp>
// #include <opencv2/cudaarithm.hpp>

#include <iostream>
#include <execution>
#include <algorithm>

namespace boblib::blobs
{
    ConnectedBlobDetection::ConnectedBlobDetection(const ConnectedBlobDetectionParams & params, size_t num_processes_parallel)
        : params_(params)
        , num_processes_parallel_(num_processes_parallel == DETECT_NUMBER_OF_THREADS ? boblib::base::Utils::get_available_threads() : num_processes_parallel)
        , initialized_(false)
    {
    }

    inline static bool rects_overlap(const cv::Rect & r1, const cv::Rect & r2)
    {
        if ((r1.width == 0 || r1.height == 0 || r2.width == 0 || r2.height == 0) ||
            (r1.x > (r2.x + r2.width) || r2.x > (r1.x + r1.width)) ||
            (r1.y > (r2.y + r2.height) || r2.y > (r1.y + r1.height)))
        {
            return false;
        }

        return true;
    }

    inline static float rects_distance_squared(const cv::Rect & r1, const cv::Rect & r2)
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
    inline static void join_bboxes(std::unordered_map<int, cv::Rect> & bboxes, int minDistanceSquared)
    {
        bool bboxOverlap;
        do
        {
            bboxOverlap = false;
            for (auto it1 = bboxes.begin(); it1 != bboxes.end(); ++it1)
            {
                auto it2 = std::next(it1); // Start checking from the next element
                while (it2 != bboxes.end())
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
                        it2 = bboxes.erase(it2); // Returns the iterator to the next element after erasing
                    }
                    else
                    {
                        ++it2; // Only increment if no merge occurred
                    }
                }
            }
        } while (bboxOverlap);
    }

    inline static void apply_size_cut(std::unordered_map<int, cv::Rect> & bboxes, const int sizeThreshold, const int areaThreshold)
    {
        for (auto it = bboxes.begin(); it != bboxes.end();)
        {
            if ((it->second.width < sizeThreshold) || (it->second.height < sizeThreshold) || (it->second.area() < areaThreshold))
            {
                it = bboxes.erase(it); // Erase and update the iterator
            }
            else
            {
                ++it; // Only increment if not erased
            }
        }
    }

    inline void ConnectedBlobDetection::pos_process_bboxes(std::unordered_map<int, cv::Rect> & bboxes)
    {
        bboxes.clear();

        // Joining all parallel bboxes into one label
        for (size_t i{0}; i < num_processes_parallel_; ++i)
        {
            const int addedY = (int)(img_sizes_parallel_[i]->original_pixel_pos / img_sizes_parallel_[i]->width);

            for (auto &[label, bbox] : bboxes_parallel_[i])
            {
                // Try to insert the bbox for the current label
                auto [it, inserted] = bboxes.emplace(label, cv::Rect{bbox.x, bbox.y + addedY, bbox.width, bbox.height});
                if (!inserted)
                {
                    // If label already exists, merge the bounding boxes
                    auto &existing_bbox = it->second;

                    const int x_max = std::max(existing_bbox.x + existing_bbox.width - 1, bbox.x + bbox.width - 1);
                    const int y_max = std::max(existing_bbox.y + existing_bbox.height - 1, bbox.y + bbox.height - 1 + addedY);

                    existing_bbox.x = std::min(existing_bbox.x, bbox.x);
                    existing_bbox.y = std::min(existing_bbox.y, bbox.y + addedY);
                    existing_bbox.width = (x_max - existing_bbox.x) + 1;
                    existing_bbox.height = (y_max - existing_bbox.y) + 1;
                }
            }
        }

        // Joining bboxes that are overlapping each other
        join_bboxes(bboxes, params_.min_distance_squared);

        // Removing bboxes that are below the threshold
        apply_size_cut(bboxes, params_.size_threshold, params_.area_threshold);
    }

    // Finds the connected components in the image and returns a list of bounding boxes
    DetectionResult ConnectedBlobDetection::detect(const boblib::base::Image & image, std::vector<cv::Rect> & bboxes)
    {
        if (!initialized_ || *original_img_size_ != ImgSize(image.size().width, image.size().height, image.channels(), image.elemSize1(), 0))
        {
            prepare_parallel(image);
            initialized_ = true;
        }

        cv::Mat labels;
        std::unordered_map<int, cv::Rect> bboxes_map;
        int num_labels(0);
        int num_blobs(0);

        // Use connected component analysis to find the blobs in the image
        // if (params_.use_cuda && image.get_using_cuda())
        // {
        //     cv::cuda::GpuMat gpu_labels;
        //     cv::cuda::connectedComponents(image.toCudaMat(), gpu_labels, 8, CV_32S);
        //     gpu_labels.download(labels);

        //     num_blobs = run_parallel(labels, bboxes_map);
        // }
        // else
        // {
            num_labels = cv::connectedComponents(image.toMat(), labels, 8, CV_32S, cv::CCL_SPAGHETTI) - 1; // subtract 1 because the background is considered as label 0
            if ((num_labels > 0) && (num_labels < params_.max_labels))
            {
                num_blobs = run_parallel(labels, bboxes_map);
            }
            else if (num_labels > params_.max_labels)
            {
                num_blobs = params_.max_blobs + 1;
            }
        // }

        if ((num_blobs > 0) && (num_blobs <= params_.max_blobs))
        {
            bboxes.clear();
            bboxes.reserve(bboxes_map.size());
            for (const auto &pair : bboxes_map)
            {
                bboxes.push_back(pair.second);
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

    inline int ConnectedBlobDetection::run_parallel(const cv::Mat & labels, std::unordered_map<int, cv::Rect> & bboxes)
    {
        bool max_labels(false);
        // Parallel execution to process each chunk of the image
        std::for_each(
            std::execution::par,
            process_seq_.begin(),
            process_seq_.end(),
            [&](int np)
            {
                std::unordered_map<int, cv::Rect> &bboxesParallel = bboxes_parallel_[np];
                bboxesParallel.clear();

                // Splitting the image into chunks and processing
                const cv::Mat imgSplit(
                    img_sizes_parallel_[np]->height,
                    img_sizes_parallel_[np]->width,
                    labels.type(),
                    labels.data + (img_sizes_parallel_[np]->original_pixel_pos * img_sizes_parallel_[np]->num_channels));
                apply_detect_bboxes(imgSplit, bboxesParallel);
                max_labels = max_labels || (static_cast<int>(bboxesParallel.size()) > params_.max_labels);
            });

        if (!max_labels)
        {
            // Post-process the bounding boxes after parallel execution
            pos_process_bboxes(bboxes);
        }

        return !max_labels ? static_cast<int>(bboxes.size()) : (params_.max_labels + 1);
    }

    inline void ConnectedBlobDetection::apply_detect_bboxes(const cv::Mat & labels, std::unordered_map<int, cv::Rect> & bboxes)
    {
        int *pLabel = (int *)labels.data;
        for (int y = 0; y < labels.rows; ++y)
        {
            for (int x = 0; x < labels.cols; ++x)
            {
                const int label = *pLabel - 1;
                if (label >= 0)
                {
                    auto [it, inserted] = bboxes.emplace(label, cv::Rect{x, y, 1, 1});
                    if (!inserted)
                    {
                        const int x_max = std::max(it->second.x + it->second.width - 1, x);
                        const int y_max = std::max(it->second.y + it->second.height - 1, y);
                        it->second.x = std::min(it->second.x, x);
                        it->second.y = std::min(it->second.y, y);
                        it->second.width = (x_max - it->second.x) + 1;
                        it->second.height = (y_max - it->second.y) + 1;
                    }
                }
                ++pLabel;
            }
        }
    }

    void ConnectedBlobDetection::prepare_parallel(const boblib::base::Image & image)
    {
        original_img_size_ = ImgSize::create(image);
        img_sizes_parallel_.resize(num_processes_parallel_);
        process_seq_.resize(num_processes_parallel_);
        bboxes_parallel_.resize(num_processes_parallel_);
        size_t y{0};
        size_t h{image.size().height / num_processes_parallel_};
        for (size_t i{0}; i < num_processes_parallel_; ++i)
        {
            process_seq_[i] = i;
            if (i == (num_processes_parallel_ - 1))
            {
                h = image.size().height - y;
            }
            img_sizes_parallel_[i] = ImgSize::create(image.size().width, h,
                                                      4, 1,
                                                      y * image.size().width);
            y += h;
        }
    }
}