#pragma once

#include "../include/coreUtils.hpp"
#include "../base/Image.hpp"

#include <opencv2/core.hpp>

#include <ranges>

namespace boblib::blobs
{
    enum class DetectionResult 
    {
        Success,
        NoBlobsDetected,
        MaxBlobsReached
    };

    struct ConnectedBlobDetectionParams final
    {
        static constexpr bool USE_CUDA = true;
        static constexpr int DEFAULT_SIZE_THRESHOLD = 5;
        static constexpr int DEFAULT_AREA_THRESHOLD = 25;
        static constexpr int DEFAULT_MIN_DISTANCE = 25;
        static constexpr int DEFAULT_MAX_BLOBS = 100;
        static constexpr int DEFAULT_MAX_LABELS = 1000;

        ConnectedBlobDetectionParams()
            : ConnectedBlobDetectionParams(USE_CUDA, DEFAULT_SIZE_THRESHOLD, DEFAULT_AREA_THRESHOLD, DEFAULT_MIN_DISTANCE, DEFAULT_MAX_BLOBS, DEFAULT_MAX_LABELS)
        {
        }

        ConnectedBlobDetectionParams(bool _use_cuda, int _sizeThreshold, int _areaThreshold, int _minDistance, int _maxBlobs, int _maxLabels = DEFAULT_MAX_LABELS)
            : use_cuda(_use_cuda)
            , size_threshold{_sizeThreshold}
            , area_threshold{_areaThreshold}
            , min_distance{_minDistance}
            , min_distance_squared{_minDistance * _minDistance}
            , max_blobs{_maxBlobs}
            , max_labels(_maxLabels)
        {
        }

        inline void set_size_threshold(int _threshold) { size_threshold = std::max(_threshold, 2); }
        inline void set_area_threshold(int _threshold) { area_threshold = std::max(_threshold, size_threshold * size_threshold); }
        inline void setMinDistance(int _min_distance)
        {
            min_distance = std::max(_min_distance, 2);
            min_distance_squared = min_distance * min_distance;
        }

        bool use_cuda;
        int size_threshold;
        int area_threshold;
        int min_distance;
        int min_distance_squared;
        int max_blobs;
        int max_labels;
    };

    class ConnectedBlobDetection final
    {
    public:
        /// Detects the number of available threads to use
        static const size_t DETECT_NUMBER_OF_THREADS{0};

        ConnectedBlobDetection(const ConnectedBlobDetectionParams &params = ConnectedBlobDetectionParams(),
                               size_t numProcessesParallel = DETECT_NUMBER_OF_THREADS) noexcept;

        inline void set_size_threshold(int threshold) noexcept { params_.set_size_threshold(threshold); }
        inline void set_area_threshold(int threshold) noexcept { params_.set_size_threshold(threshold); }
        inline void set_min_distance(int distance) noexcept { params_.setMinDistance(distance); }

        // Finds the connected components in the image and returns a list of bounding boxes
        DetectionResult detect(const boblib::base::Image &image, std::vector<cv::Rect> &bboxes) noexcept;

    private:
        ConnectedBlobDetectionParams params_;
        size_t num_processes_parallel_;
        bool initialized_;
        std::vector<size_t> process_seq_;
        std::vector<std::unique_ptr<ImgSize>> img_sizes_parallel_;
        std::vector<std::unordered_map<int, cv::Rect>> bboxes_parallel_;
        std::unique_ptr<ImgSize> original_img_size_;

        inline void prepare_parallel(const boblib::base::Image &image) noexcept;
        inline static void apply_detect_bboxes(const cv::Mat &labels, std::unordered_map<int, cv::Rect> &bboxes_map) noexcept;
        inline void pos_process_bboxes(std::unordered_map<int, cv::Rect> &bboxes_map) noexcept;
        inline int run_parallel(const cv::Mat &labels, std::unordered_map<int, cv::Rect> &bboxes_map) noexcept;
    };
}