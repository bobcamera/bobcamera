#include "connectedBlobDetection.hpp"

#include <iostream>
#include <execution>
#include <algorithm>
#include <numeric>

namespace boblib::blobs
{
    ConnectedBlobDetection::ConnectedBlobDetection(const ConnectedBlobDetectionParams &params,
                                                   size_t num_processes_parallel) noexcept
        : params_(params)
        , num_processes_parallel_(num_processes_parallel == DETECT_NUMBER_OF_THREADS ? boblib::base::Utils::get_available_threads() / 2 : num_processes_parallel)
        , initialized_(false)
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
                {
                    bboxes.clear();
                }
            }
            return;
        }

        // 1) sort by x
        std::sort(bboxes.begin(), bboxes.end(), [](auto &a, auto &b){ return a.x < b.x; });

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
                const int x0 = std::min(cr.x, r.x),
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

    // Finds the connected components in the image and returns a list of bounding boxes
    DetectionResult ConnectedBlobDetection::detect(const boblib::base::Image &image, std::vector<cv::Rect> &bboxes) noexcept
    {
        if (!initialized_ || *original_img_size_ != ImgSize(image.size().width, image.size().height, image.channels(), image.elemSize1(), 0))
        {
            prepare_parallel(image);
            initialized_ = true;
        }

        bboxes.clear();
        const int num_blobs = detect_parallel(image.toMat(), bboxes);
        return num_blobs <= params_.max_blobs ? (num_blobs > 0 ? DetectionResult::Success : DetectionResult::NoBlobsDetected)
               : DetectionResult::MaxBlobsReached;
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
                                                     1, 1,
                                                     y * image.size().width);
            y += h;
        }
    }

    inline static void apply_connect(const cv::Mat &img, std::vector<cv::Rect> &bboxes) noexcept
    {
        static constexpr int dx[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
        static constexpr int dy[8] = {-1, -1, -1, 0, 0, 1, 1, 1};

        const int H = img.rows, W = img.cols;
        const int N = H * W;

        // Use local buffers instead of member variables
        std::vector<uint8_t> seen_buf(N, 0);
        std::vector<std::pair<int, int>> queue_buf;
        queue_buf.reserve(N); // Reserve capacity, not resize

        // Access image data properly
        for (int y = 0; y < H; ++y)
        {
            const uint8_t *row = img.ptr<uint8_t>(y);
            for (int x = 0; x < W; ++x)
            {
                const int idx = y * W + x;
                if (row[x] == 0 || seen_buf[idx])
                    continue;

                // New component found
                queue_buf.clear();
                queue_buf.push_back({x, y});
                seen_buf[idx] = 1;

                int minx = x, maxx = x, miny = y, maxy = y;
                size_t qhead = 0;

                // BFS
                while (qhead < queue_buf.size())
                {
                    auto [cx, cy] = queue_buf[qhead++];

                    for (int d = 0; d < 8; ++d)
                    {
                        const int nx = cx + dx[d];
                        const int ny = cy + dy[d];

                        // Explicit bounds checking
                        if (nx >= 0 && nx < W && ny >= 0 && ny < H)
                        {
                            const int nidx = ny * W + nx;
                            if (img.ptr<uint8_t>(ny)[nx] != 0 && seen_buf[nidx] == 0)
                            {
                                seen_buf[nidx] = 1;
                                queue_buf.push_back({nx, ny});
                                minx = std::min(minx, nx);
                                maxx = std::max(maxx, nx);
                                miny = std::min(miny, ny);
                                maxy = std::max(maxy, ny);
                            }
                        }
                    }
                }

                bboxes.emplace_back(minx, miny,maxx - minx + 1,maxy - miny + 1);
            }
        }
    }

    inline int ConnectedBlobDetection::detect_parallel(const cv::Mat &img, std::vector<cv::Rect> &bboxes) noexcept
    {
        // Parallel execution to process each chunk of the image
        std::for_each(
            std::execution::par,
            process_seq_.begin(),
            process_seq_.end(),
            [&](int np)
            {
                std::vector<cv::Rect> &bboxesParallel = bboxes_parallel_[np];
                bboxesParallel.clear();
                bboxesParallel.reserve(params_.max_blobs / num_processes_parallel_);

                // Splitting the image into chunks and processing
                const cv::Mat imgSplit(
                    img_sizes_parallel_[np]->height,
                    img_sizes_parallel_[np]->width,
                    img.type(),
                    img.data + (img_sizes_parallel_[np]->original_pixel_pos * img_sizes_parallel_[np]->num_channels));

                apply_connect(imgSplit, bboxesParallel);
            });

        // 2) merge all partial results
        bboxes.clear();
        size_t total = 0;
        for (auto &v : bboxes_parallel_)
        {
            total += v.size();
        }
        bboxes.reserve(total);
        int i = 0;
        for (auto &v : bboxes_parallel_)
        {
            const int addedY = (int)(img_sizes_parallel_[i]->original_pixel_pos / img_sizes_parallel_[i]->width);
            for (auto &r : v)
            {
                bboxes.emplace_back(r.x, r.y + addedY, r.width, r.height);
            }
            ++i;
        }

        join_and_filter_bboxes(bboxes,
                               params_.min_distance,
                               params_.size_threshold,
                               params_.area_threshold);
        return static_cast<int>(bboxes.size());
    }
}