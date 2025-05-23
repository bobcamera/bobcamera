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

// ---------- PubSub message ---------------------------------------
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

// ---------- global stats -----------------------------------------
static std::atomic<int> g_frames_published{0};
static std::atomic<int> g_frames_processed{0};
static std::atomic<std::int64_t> g_sum_latency_us{0};
static std::atomic<int> g_latency_samples{0};
static std::atomic<int> g_frames_processed_cb2{0};
static std::atomic<std::int64_t> g_sum_latency_us_cb2{0};
static std::atomic<int> g_latency_samples_cb2{0};

void frameCallback(const std::shared_ptr<FrameMessage> &msg, void *)
{
    const auto now_us = std::chrono::duration_cast<MicroSecs>(
                            Clock::now().time_since_epoch())
                            .count();
    g_sum_latency_us += (now_us - msg->timestamp_us);
    ++g_latency_samples;
    ++g_frames_processed;
}

void frameCallback2(const std::shared_ptr<FrameMessage> &msg, void *)
{
    const auto now_us = std::chrono::duration_cast<MicroSecs>(
                        Clock::now().time_since_epoch())
                        .count();
    g_sum_latency_us_cb2 += (now_us - msg->timestamp_us);
    ++g_latency_samples_cb2;
    ++g_frames_processed_cb2;
}

int main(int argc, const char **argv)
{
    // ----------- Open camera & capture 60 frames --------------
    std::string rtsp = "rtsp://bob:Bobuser$01@192.168.50.10:554/Streaming/Channels/101";
    if (argc > 1)
    {
        rtsp = argv[1];
    }

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
    alignas(64) std::array<std::shared_ptr<cv::Mat>, BUF> ptrs;
    for (int i = 0; i < BUF; ++i)
    {
        cv::Mat f;
        if (!cap.read(f) || f.empty())
        {
            std::cerr << "Unable to fill buffer – camera delivered "
                      << i << " frames.\n";
            return -1;
        }
        ptrs[i] = std::make_shared<cv::Mat>(f.clone());
    }
    cap.release(); // camera no longer needed
    std::cout << "Buffered " << BUF << " frames – starting stress run ...\n";

    // ----------- Set up PubSub --------------------------------
    boblib::utils::pubsub::PubSub<FrameMessage, 8> pubsub(128);
    if (!pubsub.subscribe(frameCallback, nullptr))
    {
        std::cerr << "Failed to subscribe\n";
        return -1;
    }
    if (!pubsub.subscribe(frameCallback2, nullptr))
    {
        std::cerr << "Failed to subscribe\n";
        return -1;
    }

    // ----------- Run as fast as possible ----------------------
    const auto start_time = Clock::now();
    auto last_stats = start_time;

    for (int frameCount = 0; /* forever */; ++frameCount)
    {
        auto imgPtr = ptrs[frameCount & (BUF - 1)];

        const auto ts_us = std::chrono::duration_cast<MicroSecs>(
                               Clock::now().time_since_epoch())
                               .count();

        if (pubsub.publish(std::move(FrameMessage(std::move(imgPtr), frameCount, ts_us))))
        {
            ++g_frames_published;
        }

        const auto now = Clock::now();
        if (now - last_stats >= std::chrono::seconds(5))
        {
            double secs = std::chrono::duration<double>(now - last_stats).count();
            int pubs = g_frames_published.exchange(0);
            int procs1 = g_frames_processed.exchange(0);
            int samples1 = g_latency_samples.exchange(0);
            double avg_us1 = samples1 ? (g_sum_latency_us.exchange(0) / static_cast<double>(samples1)) : 0.0;
            int procs2 = g_frames_processed_cb2.exchange(0);
            int samples2 = g_latency_samples_cb2.exchange(0);
            double avg_us2 = samples2 ? (g_sum_latency_us_cb2.exchange(0) / static_cast<double>(samples2)) : 0.0;

            std::cout << pubs / secs << " pub/s   "
                      << procs1 / secs << " cb1 proc/s   "
                      << procs2 / secs << " cb2 proc/s   "
                      << "avg lat1 " << avg_us1 << " µs   "
                      << "avg lat2 " << avg_us2 << " µs   "
                      << "queue " << pubsub.queue_size() << '\n';

            last_stats = now;
        }
    }
}
