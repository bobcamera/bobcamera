/*********************************************************************
 *  STRESS‑TEST BUILD – queues 60 pre‑captured frames as fast as
 *  possible to hit the PubSub throughput limit.
 *********************************************************************/

#include <iostream>
#include <vector>
#include <chrono>
#include <atomic>
#include <opencv2/opencv.hpp>
#include "../../api/utils/pubsub/PubSub.hpp"

using Clock = std::chrono::steady_clock;
using MicroSecs = std::chrono::microseconds;

/* ---------- PubSub message --------------------------------------- */
struct FrameMessage
{
    std::shared_ptr<cv::Mat> framePtr;
    int frameCount;
    std::int64_t timestamp_us;

    FrameMessage(std::shared_ptr<cv::Mat> p,
                 int c,
                 std::int64_t ts)
        : framePtr(std::move(p)), frameCount(c), timestamp_us(ts) {}

    FrameMessage(FrameMessage &&) noexcept = default;
    FrameMessage &operator=(FrameMessage &&) noexcept = default;
    FrameMessage(const FrameMessage &) = delete;
    FrameMessage &operator=(const FrameMessage &) = delete;
};

/* ---------- global stats ----------------------------------------- */
std::atomic<int> g_frames_published{0};
std::atomic<int> g_frames_processed{0};
std::atomic<std::int64_t> g_sum_latency_us{0};
std::atomic<int> g_latency_samples{0};

void frameCallback(const FrameMessage &msg, void *)
{
    const auto now_us = std::chrono::duration_cast<MicroSecs>(
                            Clock::now().time_since_epoch())
                            .count();
    g_sum_latency_us += (now_us - msg.timestamp_us);
    ++g_latency_samples;
    ++g_frames_processed;
}

int main(int argc, const char **argv)
{
    /* ----------- 1. open camera & capture 60 frames -------------- */
    std::string rtsp =
        "rtsp://bob:Bobuser$01@192.168.50.10:554/Streaming/Channels/101";
    if (argc > 1)
        rtsp = argv[1];

    cv::VideoCapture cap(rtsp);
    if (!cap.isOpened())
    {
        cap.open(0);
    }
    if (!cap.isOpened())
    {
        std::cerr << "No camera source available\n";
        return -1;
    }

    constexpr int BUF = 64;
    alignas(64) std::array<cv::Mat, BUF> buffer;
    std::array<std::shared_ptr<cv::Mat>, BUF> ptrs;
    for (int i = 0; i < BUF; ++i)
    {
        cv::Mat f;
        if (!cap.read(f) || f.empty())
        {
            std::cerr << "Unable to fill buffer – camera delivered "
                      << i << " frames.\n";
            return -1;
        }
        buffer[i] = f.clone();
    }
    cap.release(); // camera no longer needed
    std::cout << "Buffered " << BUF << " frames – starting stress run ...\n";

    /* ----------- 2. set up PubSub -------------------------------- */
    boblib::utils::pubsub::PubSub<FrameMessage, 8> pubsub(128);
    if (!pubsub.subscribe(frameCallback, nullptr))
    {
        std::cerr << "Failed to subscribe\n";
        return -1;
    }

    /* ----------- 3. run as fast as possible ---------------------- */
    const auto start_time = Clock::now();
    auto last_stats = start_time;

    for (int frameCount = 0; /* forever */; ++frameCount)
    {
        const cv::Mat &src = buffer[frameCount & (BUF - 1)];
        auto imgPtr = std::make_shared<cv::Mat>(src.clone());

        const auto ts_us = std::chrono::duration_cast<MicroSecs>(
                               Clock::now().time_since_epoch())
                               .count();

        if (pubsub.publish(FrameMessage(std::move(imgPtr), frameCount, ts_us)))
            ++g_frames_published;

        /* print stats every 2 s – no sleeps anywhere -------------- */
        const auto now = Clock::now();
        if (now - last_stats >= std::chrono::seconds(5))
        {
            double secs = std::chrono::duration<double>(now - last_stats).count();
            int pubs = g_frames_published.exchange(0);
            int procs = g_frames_processed.exchange(0);
            int samples = g_latency_samples.exchange(0);
            double avg_us = samples ? (g_sum_latency_us.exchange(0) / static_cast<double>(samples))
                                    : 0.0;

            std::cout << pubs / secs << " pub/s   "
                      << procs / secs << " proc/s   "
                      << "avg latency " << avg_us << " µs   "
                      << "queue " << pubsub.queue_size() << '\n';

            last_stats = now;
        }
    }
}

// #include <iostream>
// #include <string>
// #include <algorithm>
// #include <thread>
// #include <chrono>
// #include <atomic>

// #include <opencv2/opencv.hpp>
// #include <opencv2/core/ocl.hpp>
// #include <opencv2/videoio.hpp>
// #include <opencv2/highgui.hpp>

// #include "../../api/utils/pubsub/PubSub.hpp"

// using Clock = std::chrono::steady_clock;
// using TimePoint = Clock::time_point;
// using MicroSecs = std::chrono::microseconds;

// /* ----------------------------------------------------------------- */
// /* Message passed through PubSub                                     */
// /* ----------------------------------------------------------------- */
// struct FrameMessage
// {
//     std::shared_ptr<cv::Mat> framePtr;
//     int frameCount;
//     std::int64_t timestamp_us; // ★ microseconds

//     FrameMessage(std::shared_ptr<cv::Mat> f,
//                  int count,
//                  std::int64_t ts_us) // ★
//         : framePtr(std::move(f)),
//           frameCount(count),
//           timestamp_us(ts_us)
//     {
//     }

//     FrameMessage(FrameMessage &&other) noexcept = default;
//     FrameMessage &operator=(FrameMessage &&other) noexcept = default;

//     FrameMessage(const FrameMessage &) = delete;
//     FrameMessage &operator=(const FrameMessage &) = delete;
//     ~FrameMessage() noexcept = default;
// };

// /* ----------------------------------------------------------------- */
// /* Global state                                                      */
// /* ----------------------------------------------------------------- */
// std::atomic<bool> g_running{true};
// std::atomic<int> g_frames_processed{0};
// std::atomic<int> g_frames_published{0};

// /* latency stats (µs) --------------------------------------------- */
// std::atomic<std::int64_t> g_sum_latency_us{0};
// std::atomic<std::int64_t> g_min_latency_us{std::numeric_limits<std::int64_t>::max()};
// std::atomic<std::int64_t> g_max_latency_us{0};
// std::atomic<int> g_latency_samples{0};

// /* ----------------------------------------------------------------- */
// /* Subscriber callback                                               */
// /* ----------------------------------------------------------------- */
// void frameCallback(const FrameMessage &msg, void *)
// {
//     const auto now = Clock::now();
//     const auto now_us = std::chrono::duration_cast<MicroSecs>(now.time_since_epoch()).count();
//     const auto lat_us = now_us - msg.timestamp_us;

//     g_frames_processed++;

//     g_sum_latency_us += lat_us;
//     g_latency_samples++;

//     std::int64_t old_min = g_min_latency_us.load();
//     while (lat_us < old_min && !g_min_latency_us.compare_exchange_weak(old_min, lat_us))
//     {
//     }

//     std::int64_t old_max = g_max_latency_us.load();
//     while (lat_us > old_max && !g_max_latency_us.compare_exchange_weak(old_max, lat_us))
//     {
//     }
// }

// /* ----------------------------------------------------------------- */
// /* main                                                              */
// /* ----------------------------------------------------------------- */
// int main(int argc, const char **argv)
// {
//     std::string url_capture{
//         "rtsp://bob:Bobuser$01@192.168.50.10:554/Streaming/Channels/101"};

//     boblib::utils::pubsub::PubSub<FrameMessage, 8> framePubSub(128);

//     if (!framePubSub.subscribe(frameCallback, nullptr))
//     {
//         std::cerr << "Failed to subscribe to frame messages\n";
//         return -1;
//     }

//     cv::VideoCapture cap;
//     if (argc > 1)
//         url_capture = argv[1];
//     cap.open(url_capture);
//     if (!cap.isOpened())
//     {
//         cap.open(0);
//     }
//     if (!cap.isOpened())
//     {
//         std::cerr << "No camera / RTSP source available\n";
//         return -1;
//     }

//     std::cout << "Enter loop\n";
//     auto last_report = Clock::now();
//     int last_pub_count = 0;
//     int last_proc_count = 0;
//     int last_lat_samples = 0;
//     std::int64_t last_sum_us = 0;

//     for (int frameCount = 0; g_running;)
//     {
//         auto framePtr = std::make_shared<cv::Mat>();
//         if (!cap.read(*framePtr) || framePtr->empty())
//             break;

//         const auto ts = Clock::now();
//         const auto ts_us = std::chrono::duration_cast<MicroSecs>(ts.time_since_epoch()).count();

//         if (framePubSub.publish(FrameMessage(std::move(framePtr), frameCount++, ts_us)))
//             ++g_frames_published;

//         /* stats every 2 s ---------------------------------------- */
//         const auto now = Clock::now();
//         const auto diff = now - last_report;
//         if (diff >= std::chrono::seconds(2))
//         {
//             const double secs = std::chrono::duration<double>(diff).count();
//             int pubs = g_frames_published.load() - last_pub_count;
//             int procs = g_frames_processed.load() - last_proc_count;
//             int lats = g_latency_samples.load() - last_lat_samples;
//             std::int64_t sum_us = g_sum_latency_us.load() - last_sum_us;

//             double avg_us = lats ? static_cast<double>(sum_us) / lats : 0.0;

//             std::ostringstream oss;
//             oss << "=== STATS (last " << secs << " s) ===\n"
//                 << "Producer:  " << pubs / secs << " msg/s\n"
//                 << "Consumer:  " << procs / secs << " msg/s\n"
//                 << "Latency :  avg " << avg_us
//                 << " µs  min " << g_min_latency_us.load()
//                 << "  max " << g_max_latency_us.load() << " µs\n"
//                 << "Queue   :  " << framePubSub.queue_size() << "\n"
//                 << "============================\n";
//             std::cout << oss.str() << std::flush;

//             /* reset interval stats ------------------------------ */
//             last_report = now;
//             last_pub_count = g_frames_published.load();
//             last_proc_count = g_frames_processed.load();
//             last_lat_samples = g_latency_samples.load();
//             last_sum_us = g_sum_latency_us.load();
//             g_min_latency_us = std::numeric_limits<std::int64_t>::max();
//             g_max_latency_us = 0;
//         }
//     }

//     return 0;
// }
