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
        {
            ++g_frames_published;
        }

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
