/*********************************************************************
 *  STRESS‑TEST BUILD – queues 60 pre‑captured frames as fast as
 *  possible to hit the PubSub throughput limit.
 *********************************************************************/

#include <iostream>
#include <vector>
#include <chrono>
#include <atomic>

#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/clock.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include "../../api/utils/pubsub/PubSub.hpp"

// alias clocks
using SteadyClock = std::chrono::steady_clock;
using MicroSecs = std::chrono::microseconds;
using SystemClock = std::chrono::system_clock;

// measure timestamps and latencies using system clock
static rclcpp::Clock ros_clock(RCL_SYSTEM_TIME);

/* ---------- global stats ----------------------------------------- */
std::atomic<int> g_frames_published{0};
std::atomic<int> g_frames_processed{0};
std::atomic<std::int64_t> g_sum_latency_us{0};
std::atomic<int> g_latency_samples{0};

void frameCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    // compute message timestamp in microseconds
    int64_t msg_time_us = static_cast<int64_t>(msg->header.stamp.sec) * 1000000
                        + static_cast<int64_t>(msg->header.stamp.nanosec) / 1000;
    // current system time in microseconds
    int64_t now_us = std::chrono::duration_cast<MicroSecs>(
                        SystemClock::now().time_since_epoch()).count();
    g_sum_latency_us += (now_us - msg_time_us);
    ++g_latency_samples;
    ++g_frames_processed;
}

int main(int argc, const char **argv)
{
    // ---- init ROS2 ----
    rclcpp::init(argc, argv);
    // create ROS2 node with intra-process communication enabled for zero-copy transport
    auto node_opts = rclcpp::NodeOptions().use_intra_process_comms(true);
    auto ros_node = rclcpp::Node::make_shared("demo_pubsub", node_opts);

    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);
    qos_profile.history(rclcpp::HistoryPolicy::KeepLast);

    // configure publisher to enable intra-process communication
    rclcpp::PublisherOptions pub_opts;
    pub_opts.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;
    auto ros_pub = ros_node->create_publisher<sensor_msgs::msg::Image>("image_raw", qos_profile, pub_opts);
    // configure subscriber for intra-process communication
    rclcpp::SubscriptionOptions sub_opts;
    sub_opts.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;
    auto ros_sub = ros_node->create_subscription<sensor_msgs::msg::Image>(
        "image_raw", qos_profile, frameCallback, sub_opts);
    std::thread ros_spin_thread([&]() { rclcpp::spin(ros_node); });

    // ----------- 1. open camera & capture X frames --------------
    constexpr int BUF = 64;
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

    alignas(64) std::array<std::shared_ptr<sensor_msgs::msg::Image>, BUF> msgs;
    for (int i = 0; i < BUF; ++i)
    {
        cv::Mat f;
        if (!cap.read(f) || f.empty())
        {
            std::cerr << "Unable to fill buffer – camera delivered "
                      << i << " frames.\n";
            return -1;
        }
        std_msgs::msg::Header hdr;
        hdr.frame_id = "camera";
        msgs[i] = cv_bridge::CvImage(hdr, "bgr8", f).toImageMsg();
    }
    cap.release(); // camera no longer needed
    std::cout << "Buffered " << BUF << " frames – starting stress run ...\n";

    // ----------- Run as fast as possible ---------------------- 
    const auto start_time = SteadyClock::now();
    auto last_stats = start_time;

    for (int frameCount = 0; rclcpp::ok(); ++frameCount)
    {
        auto & cv_msg = msgs[frameCount & (BUF - 1)];

        auto tp = SystemClock::now().time_since_epoch();
        cv_msg->header.stamp.sec = std::chrono::duration_cast<std::chrono::seconds>(tp).count();
        cv_msg->header.stamp.nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(tp).count() % 1000000000;
        // publish using shared_ptr to leverage zero-copy intra-process
        ros_pub->publish(*cv_msg);
        ++g_frames_published;

        const auto now = SteadyClock::now();
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
                      << "queue " << (pubs - procs) << std::endl;

            last_stats = now;
        }
    }
    // clean shutdown
    rclcpp::shutdown();
    ros_spin_thread.join();
}
