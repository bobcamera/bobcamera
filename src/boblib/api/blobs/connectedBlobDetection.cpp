#include "connectedBlobDetection.hpp"

#include <opencv2/imgproc.hpp>
#ifdef HAVE_CUDA
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudaarithm.hpp>
#endif

#include <iostream>
#include <execution>
#include <algorithm>
#include <unordered_set>
#include <numeric>

namespace boblib::blobs
{
    ConnectedBlobDetection::ConnectedBlobDetection(const ConnectedBlobDetectionParams &params, size_t num_processes_parallel) noexcept
        : params_(params), num_processes_parallel_(num_processes_parallel == DETECT_NUMBER_OF_THREADS ? boblib::base::Utils::get_available_threads() / 2 : num_processes_parallel), initialized_(false)
    {
    }

    inline constexpr static bool rects_overlap(const cv::Rect &r1, const cv::Rect &r2) noexcept
    {
        if ((r1.width == 0 || r1.height == 0 || r2.width == 0 || r2.height == 0) ||
            (r1.x > (r2.x + r2.width) || r2.x > (r1.x + r1.width)) ||
            (r1.y > (r2.y + r2.height) || r2.y > (r1.y + r1.height)))
        {
            return false;
        }

        return true;
    }

    inline constexpr static float rects_distance_squared(const cv::Rect &r1, const cv::Rect &r2) noexcept
    {
        if (rects_overlap(r1, r2))
        {
            return 0;
        }

        const int x_distance = std::max(0, std::max(r1.x, r2.x) - std::min(r1.x + r1.width, r2.x + r2.width));
        const int y_distance = std::max(0, std::max(r1.y, r2.y) - std::min(r1.y + r1.height, r2.y + r2.height));

        return (x_distance * x_distance) + (y_distance * y_distance);
    }

    inline static void join_bboxes(std::unordered_map<int, cv::Rect> &bboxes_map, int minDistance) noexcept
    {
        if (bboxes_map.size() <= 1)
        {
            return;
        }

        // Sort rects by x-coordinate for faster merging
        std::vector<std::pair<int, cv::Rect>> sorted_bboxes;
        sorted_bboxes.reserve(bboxes_map.size());
        for (const auto &pair : bboxes_map)
        {
            sorted_bboxes.emplace_back(pair);
        }

        std::sort(sorted_bboxes.begin(), sorted_bboxes.end(),
                  [](const auto &a, const auto &b)
                  { return a.second.x < b.second.x; });

        // Extended bounding boxes to account for minDistance
        std::vector<std::pair<int, cv::Rect>> extended_bboxes;
        extended_bboxes.reserve(sorted_bboxes.size());

        for (const auto &[id, rect] : sorted_bboxes)
        {
            cv::Rect extended = rect;
            extended.x -= minDistance;
            extended.y -= minDistance;
            extended.width += 2 * minDistance;
            extended.height += 2 * minDistance;
            extended_bboxes.emplace_back(id, extended);
        }

        // Single-pass merge
        std::unordered_set<int> to_remove;
        for (size_t i = 0; i < sorted_bboxes.size(); ++i)
        {
            if (to_remove.count(i))
            {
                continue;
            }

            auto &[id1, rect1] = sorted_bboxes[i];
            auto &extended1 = extended_bboxes[i].second;

            for (size_t j = i + 1; j < sorted_bboxes.size(); ++j)
            {
                if (to_remove.count(j))
                {
                    continue;
                }

                auto &[id2, rect2] = sorted_bboxes[j];
                // Early termination: if current x > extended x + width, no more overlaps possible
                if (rect2.x > extended1.x + extended1.width)
                {
                    break;
                }

                // Only check y-overlap if x-overlap exists
                if (rects_overlap(extended1, extended_bboxes[j].second))
                {
                    // Merge rectangles
                    const int xmax = std::max(rect1.x + rect1.width, rect2.x + rect2.width);
                    const int ymax = std::max(rect1.y + rect1.height, rect2.y + rect2.height);
                    rect1.x = std::min(rect1.x, rect2.x);
                    rect1.y = std::min(rect1.y, rect2.y);
                    rect1.width = xmax - rect1.x;
                    rect1.height = ymax - rect1.y;

                    // Update extended rectangle
                    extended1.x = rect1.x - minDistance;
                    extended1.y = rect1.y - minDistance;
                    extended1.width = rect1.width + 2 * minDistance;
                    extended1.height = rect1.height + 2 * minDistance;

                    to_remove.insert(j);
                }
            }
        }

        // Rebuild the bboxes map
        bboxes_map.clear();
        for (size_t i = 0; i < sorted_bboxes.size(); ++i)
        {
            if (!to_remove.count(i))
            {
                bboxes_map.emplace(sorted_bboxes[i]);
            }
        }
    }

    inline static void join_bboxes_vec(std::vector<cv::Rect> &bboxes, int minDistance) noexcept
    {
        if (bboxes.size() <= 1)
        {
            return;
        }

        // sort by x
        std::sort(bboxes.begin(), bboxes.end(),
                  [](auto &a, auto &b)
                  { return a.x < b.x; });

        // make extended copies
        std::vector<cv::Rect> extended = bboxes;
        for (auto &r : extended)
        {
            r.x -= minDistance;
            r.y -= minDistance;
            r.width += 2 * minDistance;
            r.height += 2 * minDistance;
        }

        // mark which rects get merged away
        std::vector<bool> removed(bboxes.size(), false);
        for (size_t i = 0; i < bboxes.size(); ++i)
        {
            if (removed[i])
            {
                continue;
            }
            auto &r1 = bboxes[i];
            auto &e1 = extended[i];
            for (size_t j = i + 1; j < bboxes.size(); ++j)
            {
                if (removed[j])
                {
                    continue;
                }
                // x‐axis cull
                if (bboxes[j].x > e1.x + e1.width)
                {
                    break;
                }
                if (rects_overlap(e1, extended[j]))
                {
                    // merge j into i
                    int x0 = std::min(r1.x, bboxes[j].x);
                    int y0 = std::min(r1.y, bboxes[j].y);
                    int x2 = std::max(r1.x + r1.width, bboxes[j].x + bboxes[j].width);
                    int y2 = std::max(r1.y + r1.height, bboxes[j].y + bboxes[j].height);
                    r1 = cv::Rect{x0, y0, x2 - x0, y2 - y0};
                    e1 = cv::Rect{r1.x - minDistance,
                                  r1.y - minDistance,
                                  r1.width + 2 * minDistance,
                                  r1.height + 2 * minDistance};
                    removed[j] = true;
                }
            }
        }

        // compact out the removed ones
        std::vector<cv::Rect> kept;
        kept.reserve(bboxes.size());
        for (size_t i = 0; i < bboxes.size(); ++i)
        {
            if (!removed[i])
            {
                kept.push_back(bboxes[i]);
            }
        }

        bboxes.swap(kept);
    }

    inline static void apply_size_cut(std::unordered_map<int, cv::Rect> &bboxes_map,
                                      const int sizeThreshold,
                                      const int areaThreshold) noexcept
    {
        // build a new filtered map
        std::unordered_map<int, cv::Rect> filtered;
        filtered.reserve(bboxes_map.size());
        for (auto &kv : bboxes_map)
        {
            const cv::Rect &r = kv.second;
            if (r.width >= sizeThreshold &&
                r.height >= sizeThreshold &&
                r.area() >= areaThreshold)
            {
                filtered.emplace(kv.first, r);
            }
        }
        // replace original with filtered
        bboxes_map.swap(filtered);
    }

    inline static void apply_size_cut_vec(std::vector<cv::Rect> &bboxes,
                                      int sizeThreshold,
                                      int areaThreshold) noexcept
    {
        std::vector<cv::Rect> filtered;
        filtered.reserve(bboxes.size());
        for (auto &r : bboxes)
        {
            if (r.width >= sizeThreshold &&
                r.height >= sizeThreshold &&
                r.area() >= areaThreshold)
            {
                filtered.push_back(r);
            }
        }
        bboxes.swap(filtered);
    }

    struct UnionFind
    {
        std::vector<int> parent, rank;
        UnionFind(int n) 
            : parent(n)
            , rank(n, 0)
        {
            std::iota(parent.begin(), parent.end(), 0);
        }
        int find(int x) noexcept
        {
            return parent[x] == x ? x : parent[x] = find(parent[x]);
        }
        void unite(int a, int b) noexcept
        {
            a = find(a);
            b = find(b);
            if (a == b)
            {
                return;
            }
            if (rank[a] < rank[b])
            {
                parent[a] = b;
            }
            else if (rank[b] < rank[a])
            {
                parent[b] = a;
            }
            else
            {
                parent[b] = a;
                rank[a]++;
            }
        }
    };

    inline static void join_and_filter_bboxes(
        std::vector<cv::Rect> &bboxes,
        int minDistance,
        int sizeThreshold,
        int areaThreshold) noexcept
    {
        const int N = (int)bboxes.size();
        if (N <= 1)
        {
            if (N == 1)
            {
                auto &r = bboxes[0];
                if (r.width < sizeThreshold || r.height < sizeThreshold || r.area() < areaThreshold)
                    bboxes.clear();
            }
            return;
        }

        // 1) sort by x
        std::sort(bboxes.begin(), bboxes.end(),
                  [](auto &a, auto &b)
                  { return a.x < b.x; });

        // 2) build union-find & extended rects
        UnionFind uf(N);
        std::vector<cv::Rect> ext(N);
        for (int i = 0; i < N; ++i)
        {
            auto &r = bboxes[i];
            ext[i] = cv::Rect{r.x - minDistance,
                              r.y - minDistance,
                              r.width + 2 * minDistance,
                              r.height + 2 * minDistance};
        }

        // 3) sweep-line merge
        for (int i = 0; i < N; ++i)
        {
            auto &e1 = ext[i];
            for (int j = i + 1; j < N && bboxes[j].x <= e1.x + e1.width; ++j)
            {
                if (rects_overlap(e1, ext[j]))
                {
                    uf.unite(i, j);
                }
            }
        }

        // 4) collect per-component bounding‐box
        std::unordered_map<int, cv::Rect> comps;
        comps.reserve(N);
        for (int i = 0; i < N; ++i)
        {
            int p = uf.find(i);
            auto &r = bboxes[i];
            auto it = comps.find(p);
            if (it == comps.end())
            {
                comps[p] = r;
            }
            else
            {
                auto &cr = it->second;
                int x0 = std::min(cr.x, r.x),
                    y0 = std::min(cr.y, r.y),
                    x2 = std::max(cr.x + cr.width, r.x + r.width),
                    y2 = std::max(cr.y + cr.height, r.y + r.height);
                cr = cv::Rect{x0, y0, x2 - x0, y2 - y0};
            }
        }

        // 5) apply size+area filter
        bboxes.clear();
        bboxes.reserve(comps.size());
        for (auto &kv : comps)
        {
            auto &r = kv.second;
            if (r.width >= sizeThreshold &&
                r.height >= sizeThreshold &&
                r.area() >= areaThreshold)
            {
                bboxes.push_back(r);
            }
        }
    }

    inline void ConnectedBlobDetection::pos_process_bboxes(std::unordered_map<int, cv::Rect> &bboxes_map) noexcept
    {
        bboxes_map.clear();

        // Joining all parallel bboxes into one label
        for (size_t i{0}; i < num_processes_parallel_; ++i)
        {
            const int addedY = (int)(img_sizes_parallel_[i]->original_pixel_pos / img_sizes_parallel_[i]->width);

            for (auto &[label, bbox] : bboxes_parallel_[i])
            {
                // Try to insert the bbox for the current label
                auto [it, inserted] = bboxes_map.emplace(label, cv::Rect{bbox.x, bbox.y + addedY, bbox.width, bbox.height});
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
        join_bboxes(bboxes_map, params_.min_distance);

        // Removing bboxes that are below the threshold
        apply_size_cut(bboxes_map, params_.size_threshold, params_.area_threshold);
    }

    inline static void join_and_filter_bboxes_map(
        std::unordered_map<int, cv::Rect> &bboxes_map,
        int minDistance,
        int sizeThreshold,
        int areaThreshold) noexcept
    {
        // trivial cases
        if (bboxes_map.size() <= 1)
        {
            if (bboxes_map.size() == 1)
            {
                auto it = bboxes_map.begin();
                const auto &r = it->second;
                if (r.width < sizeThreshold ||
                    r.height < sizeThreshold ||
                    r.area() < areaThreshold)
                {
                    bboxes_map.clear();
                }
            }
            return;
        }

        // 1) gather & sort
        std::vector<std::pair<int, cv::Rect>> sorted;
        sorted.reserve(bboxes_map.size());
        for (auto &kv : bboxes_map)
            sorted.emplace_back(kv);
        std::sort(sorted.begin(), sorted.end(),
                  [](auto &a, auto &b)
                  { return a.second.x < b.second.x; });

        // 2) build extended rects
        const size_t N = sorted.size();
        std::vector<cv::Rect> ext(N);
        for (size_t i = 0; i < N; ++i)
        {
            auto &r = sorted[i].second;
            ext[i] = cv::Rect{
                r.x - minDistance,
                r.y - minDistance,
                r.width + 2 * minDistance,
                r.height + 2 * minDistance};
        }

        // 3) single‐pass merge into sorted & mark removals
        std::vector<bool> removed(N, false);
        for (size_t i = 0; i < N; ++i)
        {
            if (removed[i])
                continue;
            auto &r1 = sorted[i].second;
            auto &e1 = ext[i];
            for (size_t j = i + 1; j < N; ++j)
            {
                if (removed[j])
                    continue;
                // x‐axis cull
                if (sorted[j].second.x > e1.x + e1.width)
                    break;
                if (rects_overlap(e1, ext[j]))
                {
                    // merge j into i
                    auto &r2 = sorted[j].second;
                    int x0 = std::min(r1.x, r2.x);
                    int y0 = std::min(r1.y, r2.y);
                    int x2 = std::max(r1.x + r1.width, r2.x + r2.width);
                    int y2 = std::max(r1.y + r1.height, r2.y + r2.height);
                    r1 = cv::Rect{x0, y0, x2 - x0, y2 - y0};
                    e1 = cv::Rect{r1.x - minDistance,
                                  r1.y - minDistance,
                                  r1.width + 2 * minDistance,
                                  r1.height + 2 * minDistance};
                    removed[j] = true;
                }
            }
        }

        // 4) rebuild & filter
        std::unordered_map<int, cv::Rect> filtered;
        filtered.reserve(N);
        for (size_t i = 0; i < N; ++i)
        {
            if (removed[i])
                continue;
            auto &p = sorted[i];
            auto &r = p.second;
            if (r.width >= sizeThreshold &&
                r.height >= sizeThreshold &&
                r.area() >= areaThreshold)
            {
                filtered.emplace(p.first, r);
            }
        }

        // replace
        bboxes_map.swap(filtered);
    }

    // Finds the connected components in the image and returns a list of bounding boxes
    DetectionResult ConnectedBlobDetection::detect(const boblib::base::Image &image, std::vector<cv::Rect> &bboxes) noexcept
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
#ifdef HAVE_CUDA
        if (params_.use_cuda && image.get_using_cuda())
        {
            cv::cuda::GpuMat gpu_labels;
            cv::cuda::connectedComponents(image.toCudaMat(), gpu_labels, 8, CV_32S);
            gpu_labels.download(labels);

            num_blobs = run_parallel(labels, bboxes_map);
        }
        else
#endif
        {
            bool old_way = true;
            if (old_way)
            {
                num_labels = cv::connectedComponents(image.toMat(), labels, 8, CV_32S, cv::CCL_SPAGHETTI) - 1; // subtract 1 because the background is considered as label 0
                if ((num_labels > 0) && (num_labels < params_.max_labels))
                {
                    bboxes_map.reserve(num_labels);
                    num_blobs = run_parallel(labels, bboxes_map);
                }
                else if (num_labels > params_.max_labels)
                {
                    num_blobs = params_.max_blobs + 1;
                }
            }
            else
            {
                cv::Mat stats, centroids;
                num_labels = cv::connectedComponentsWithStats(
                    image.toMat(), // input
                    labels,        // output labels
                    stats,         // output [N×CV_32S] where columns are LEFT,TOP,WIDTH,HEIGHT,AREA
                    centroids,     // unused
                    8,             // connectivity
                    CV_32S,
                    cv::CCL_SPAGHETTI);
                if ((num_labels > 0) && (num_labels < params_.max_labels))
                {
                    bboxes.clear();
                    bboxes.reserve(num_labels - 1);
                    int *sdata = stats.ptr<int>();
                    const int N = stats.cols; // should be 5: {LEFT,TOP,WIDTH,HEIGHT,AREA}
                    for (int lbl = 1; lbl < num_labels; ++lbl)
                    {
                        const int base = lbl * N;
                        const int w = sdata[base + cv::CC_STAT_WIDTH];
                        const int h = sdata[base + cv::CC_STAT_HEIGHT];
                        const int x = sdata[base + cv::CC_STAT_LEFT];
                        const int y = sdata[base + cv::CC_STAT_TOP];
                        bboxes.emplace_back(x, y, w, h);
                    }
                    join_and_filter_bboxes(bboxes, params_.min_distance, params_.size_threshold, params_.area_threshold);

                    return bboxes.size() > 0 ? DetectionResult::Success
                                             : DetectionResult::NoBlobsDetected;
                }
                else if (num_labels > params_.max_labels)
                {
                    num_blobs = params_.max_blobs + 1;
                }
            }
        }

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

    inline int ConnectedBlobDetection::run_parallel(const cv::Mat &labels, std::unordered_map<int, cv::Rect> &bboxes) noexcept
    {
        std::atomic<bool> max_labels(false);

        // Parallel execution to process each chunk of the image
        std::for_each(
            std::execution::par,
            process_seq_.begin(),
            process_seq_.end(),
            [&](int np)
            {
                std::unordered_map<int, cv::Rect> &bboxesParallel = bboxes_parallel_[np];
                bboxesParallel.clear();
                bboxesParallel.reserve(params_.max_blobs / num_processes_parallel_);

                // Splitting the image into chunks and processing
                const cv::Mat imgSplit(
                    img_sizes_parallel_[np]->height,
                    img_sizes_parallel_[np]->width,
                    labels.type(),
                    labels.data + (img_sizes_parallel_[np]->original_pixel_pos * img_sizes_parallel_[np]->num_channels));
                apply_detect_bboxes(imgSplit, bboxesParallel);

                // Use atomic to prevent race conditions
                if (static_cast<int>(bboxesParallel.size()) > params_.max_labels)
                {
                    max_labels.store(true, std::memory_order_relaxed);
                }
            });

        if (!max_labels.load(std::memory_order_acquire))
        {
            // Post-process the bounding boxes after parallel execution
            pos_process_bboxes(bboxes);
            return static_cast<int>(bboxes.size());
        }

        return params_.max_labels + 1;
    }

    inline void ConnectedBlobDetection::apply_detect_bboxes(const cv::Mat &labels, std::unordered_map<int, cv::Rect> &bboxes) noexcept
    {
        int *pLabel = (int *)labels.data;
        for (int y = 0; y < labels.rows; ++y)
        {
            for (int x = 0; x < labels.cols; ++x)
            {
                const int label = *pLabel - 1;
                if (label >= 0)
                {
                    auto [it, inserted] = bboxes.try_emplace(label, cv::Rect{x, y, 1, 1});
                    if (!inserted)
                    {
                        auto &bbox = it->second;
                        const int x_max = std::max(bbox.x + bbox.width - 1, x);
                        const int y_max = std::max(bbox.y + bbox.height - 1, y);
                        bbox.x = std::min(bbox.x, x);
                        bbox.y = std::min(bbox.y, y);
                        bbox.width = (x_max - bbox.x) + 1;
                        bbox.height = (y_max - bbox.y) + 1;
                    }
                }
                ++pLabel;
            }
        }
    }

    void ConnectedBlobDetection::prepare_parallel(const boblib::base::Image &image) noexcept
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