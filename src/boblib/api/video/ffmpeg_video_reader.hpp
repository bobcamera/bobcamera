#pragma once

extern "C"
{
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libavutil/pixdesc.h>
#include <libavutil/hwcontext.h>
#include <libswscale/swscale.h>
#include <libavutil/time.h>
}

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <stdexcept>
#include <iostream>

namespace boblib::video
{
    class FFmpegVideoReader final
    {
    public:
        FFmpegVideoReader();

        explicit FFmpegVideoReader(const std::string &src);

        ~FFmpegVideoReader();

        /// open / reopen a usb camera – returns true on success
        bool open(int camera_id, int width = 0, int height = 0);
        /// open / reopen a source (file path, RTSP url, or /dev/video* device) – returns true on success
        bool open(const std::string &src, int width = 0, int height = 0);
        /// close the video source and free all resources
        void close();

        /// enable/disable hardware acceleration (default: true)
        void set_hardware_acceleration(bool enabled) { hw_accel_enabled_ = enabled; }

        /// read next frame; returns true if a frame was read (false ⇒ EOF)
        bool read(cv::Mat &out);

        int get_width() const;
        int get_height() const;
        double get_fps() const;
        bool is_opened() const;

        // new info getters:
        std::string get_codec_name()   const;  ///< AVCodecID name from the stream
        std::string get_decoder_name() const;  ///< name of the decoder instance
        std::string get_pixel_format_name() const; ///< pixel format of decoded frames

        std::vector<std::pair<int, int>> list_camera_resolutions() const;

        bool set_resolution(int width, int height);

    private:
        void allocate_resources();

        bool init_decoder();

        bool init_hw_device(AVHWDeviceType type);

        cv::Mat avframe_to_mat(const AVFrame *src);

        std::string src_;
        AVFormatContext *fmt_ctx_{nullptr};
        AVCodecContext *codec_ctx_{nullptr};
        const AVCodec *decoder_{nullptr};
        AVStream *video_stream_{nullptr};
        AVPacket *pkt_{nullptr};
        AVFrame *frame_{nullptr};
        AVFrame *sw_frame_{nullptr};
        SwsContext *sws_ctx_{nullptr};
        AVBufferRef *hw_device_ctx_{nullptr};
        int video_stream_index_{-1};
        int width_{0};
        int height_{0};
        double fps_{0};
        AVPixelFormat hw_pix_fmt_{AV_PIX_FMT_NONE};
        bool is_opened_{false};
        bool hw_accel_enabled_{true};
        int64_t probe_start_us_{0};

        static int interrupt_cb(void *opaque)
        {
            auto *self = static_cast<FFmpegVideoReader *>(opaque);
            // if more than 2 seconds have elapsed, abort
            if (av_gettime_relative() - self->probe_start_us_ > 2'000'000)
                return 1;
            return 0;
        }
    };
}
    /* Example program:
    #include "ffmpeg_capture.hpp"
    int main() {
        FFmpegCapture cap("rtsp://username:password@192.168.1.10:554/stream");
        cv::Mat frame;
        while (cap.read(frame)) {
            cv::imshow("video", frame);
            if (cv::waitKey(1) == 27) break; // Esc to quit
        }
        return 0;
    }
    */
