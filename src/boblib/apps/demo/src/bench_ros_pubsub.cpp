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
#include "./../../api/utils/profiler.hpp"

// alias clocks
using SteadyClock = std::chrono::steady_clock;
using MicroSecs = std::chrono::microseconds;
using SystemClock = std::chrono::system_clock;

/* ---------- global stats ----------------------------------------- */
std::atomic<int> g_frames_published{0};
std::atomic<int> g_frames_processed{0};
std::atomic<std::int64_t> g_sum_latency_us{0};
std::atomic<int> g_latency_samples{0};

void frameCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  using namespace std::chrono;

  // grab now (ns since SteadyClock epoch)
  int64_t now_ns = duration_cast<nanoseconds>(
    SteadyClock::now().time_since_epoch()).count();

  // reconstruct sent timestamp from header
  int64_t sent_ns =
    int64_t(msg->header.stamp.sec) * 1000000000LL +
    int64_t(msg->header.stamp.nanosec);

  int64_t latency_us = (now_ns - sent_ns) / 1000LL;

  g_sum_latency_us += latency_us;
  ++g_latency_samples;
  ++g_frames_processed;
}

int main(int argc, const char **argv)
{
    boblib::utils::Profiler profiler("Benchmark ROS2 PubSub", 5, true);
    size_t prof_ros_id = profiler.add_region("ROS2 Publish");

    // ---- init ROS2 ----
    rclcpp::init(argc, argv);

    // Print the RMW implementation identifier
    auto rmw_identifier = rmw_get_implementation_identifier();
    if (rmw_identifier)
    {
        std::cout << "Using RMW implementation: " << rmw_identifier << std::endl;
    }
    else
    {
        std::cout << "Could not retrieve RMW implementation identifier." << std::endl;
    }

    // create ROS2 node with intra-process communication enabled for zero-copy transport
    auto node_opts = rclcpp::NodeOptions().use_intra_process_comms(true);
    auto ros_node = rclcpp::Node::make_shared("demo_pubsub", node_opts);

    rclcpp::QoS qos_profile(100);
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

        profiler.start(prof_ros_id);
        auto loaned = ros_pub->borrow_loaned_message();
        auto & msg = loaned.get(); // get raw pointer to the loaned message
        msg.header = cv_msg->header;
        msg.height = cv_msg->height;
        msg.width = cv_msg->width;
        msg.encoding = cv_msg->encoding;
        msg.step = cv_msg->step;
        msg.data = cv_msg->data;
        {
          using namespace std::chrono;
          // stamp with SteadyClock now
          int64_t ns = duration_cast<nanoseconds>(
            SteadyClock::now().time_since_epoch()).count();
          msg.header.stamp.sec    = static_cast<int32_t>(ns / 1000000000LL);
          msg.header.stamp.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
        }

        ros_pub->publish(std::move(loaned));
        profiler.stop(prof_ros_id);
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
