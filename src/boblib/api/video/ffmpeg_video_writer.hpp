#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <chrono>
#include <memory>
#include <filesystem>

// FFmpeg includes
extern "C"
{
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libswscale/swscale.h>
}

namespace boblib::video
{
    namespace fs = std::filesystem;

    class FFmpegVideoWriter final
    {
    public:
        struct Options
        {
            enum class CodecType
            {
                H264,
                HEVC
            };
            std::string outputPath;              // Output file path
            CodecType codec = CodecType::H264;   // Encoder to use (libx264, h264_nvenc, hevc_nvenc, etc.)
            int width = 0;                       // Will be set from first frame if 0
            int height = 0;                      // Will be set from first frame if 0
            int bitrate = 0;                     // In bits/s, 0 for default
            double fps = 30.0;                   // Frames per second (0 for default)
            int threads = 0;                     // 0 for auto-detection
            int gop_size = 0;                    // GOP size (0 for default)
            bool useHardwareAcceleration = true; // Use hardware acceleration if available
            int quality = 23;                    // CRF for x264/x265 (lower is better quality, higher is smaller file)
            std::string pixelFormat = "yuv420p"; // Output pixel format
            std::string preset = "fast";         // Encoding preset (ultrafast, superfast, veryfast, faster, fast, medium, slow, slower, veryslow)
            size_t bufferSize = 300;             // Frame buffer size
            size_t numWorkerThreads = 1;         // Usually 1 is enough as FFmpeg has internal threading
            bool logPerformance = false;         // Log performance stats
            bool debug = false;                  // Enable debug output
            std::string extraOptions = "";       // Additional FFmpeg options formatted as "key1=value1:key2=value2"
        };

        // Constructor
        explicit FFmpegVideoWriter(const Options &options);

        // Destructor
        ~FFmpegVideoWriter();

        // Start the writer
        bool start();

        // Stop the writer and finalize the video
        void stop();

        // Get the codec long name
        std::string get_codec_name() const noexcept;

        // Write a frame to the video
        bool write_frame(const cv::Mat &frame);

        // Get current performance stats
        struct Stats
        {
            size_t framesReceived{0};
            size_t framesWritten{0};
            size_t queueSize{0};
            double fps{0.0};
            double avgProcessingTimeMs{0.0};
            bool isValid{false};
        };

        Stats get_stats() const;

        // Check if writer is in valid state
        bool is_valid() const noexcept;

    private:
        Options m_options;
        std::atomic<bool> m_isRunning{false};
        std::atomic<bool> m_isInitialized{false};
        std::atomic<bool> m_isValid{false};
        std::atomic<size_t> m_framesReceived{0};
        std::atomic<size_t> m_framesWritten{0};

        std::string m_codecLongName;

        // Thread-safe queue
        std::queue<cv::Mat> m_frameQueue;
        mutable std::mutex m_queueMutex;
        mutable std::condition_variable m_queueCondition;

        // Worker threads
        std::vector<std::thread> m_workerThreads;
        std::thread m_monitorThread;

        // Performance metrics
        std::chrono::steady_clock::time_point m_startTime;
        std::atomic<double> m_totalProcessingTime;

        // FFmpeg variables (protected by cleanup mutex)
        mutable std::mutex m_ffmpegMutex;
        AVFormatContext *m_formatContext{nullptr};
        AVCodecContext *m_codecContext{nullptr};
        AVStream *m_stream{nullptr};
        SwsContext *m_swsContext{nullptr};
        AVFrame *m_frame{nullptr};
        uint8_t *m_frameBuffer{nullptr};
        int m_ioBufferSize;

        // pull all the heavy lifting into this helper
        bool init_with_codec(const AVCodec *codec, int width, int height);

        // Initialize FFmpeg
        bool initialize_fFmpeg(int width, int height);

        // Convert cv::Mat to FFmpeg frame
        bool convert_frame(const cv::Mat &input, AVFrame *output);

        // Finalize the video
        void finalize_video();

        // Cleanup all FFmpeg resources (thread-safe)
        void cleanup();

        // Worker thread for processing frames
        void processing_thread();

        // Performance monitoring thread
        void monitor_thread();

        // Validate internal state
        bool validate_state() const noexcept;
    };
}

// Example usage
/*
int main()
{
    // Create writer options
    FFmpegVideoWriter::Options options;
    options.outputPath = "output.mp4";
    options.codec = "h264";
    options.fps = 80.0;
    options.useHardwareAcceleration = true;
    options.quality = 18;              // Lower CRF value for better quality
    options.logPerformance = true;
    options.extraOptions = "slices=4:rc-lookahead=20";  // Example of additional options

    // Create writer
    FFmpegVideoWriter writer(options);
    writer.start();

    // Feed frames to the writer
    cv::Mat frame(2160, 3840, CV_8UC3); // 4K frame as an example
    for (int i = 0; i < 1000; ++i)
    {
        // Generate or get your frame here
        // frame = getSomeFrame();

        writer.writeFrame(frame);

        // Optional: Simulate frame acquisition delay
        std::this_thread::sleep_for(std::chrono::milliseconds(12)); // ~80fps
    }

    // Stop the writer
    writer.stop();

    return 0;
}
*/