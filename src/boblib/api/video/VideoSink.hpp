#pragma once

// video_sink.hpp — C++20 header-only wrapper around FFmpeg (libav*) with optional
// CUDA/NVENC hardware encoding.  Designed for >20 MP @ 80 fps.
//
// 🔄 2025-05-13 — public-domain / MIT-0
// -----------------------------------------------------------------------------

extern "C"
{
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/avassert.h>
#include <libavutil/avutil.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libavutil/pixdesc.h>
#include <libavutil/hwcontext.h>
#include <libswscale/swscale.h>
}

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <exception>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>
#include <iostream>

namespace boblib::video
{

    namespace detail
    {
        inline std::string ffmpeg_err(int err)
        {
            char buf[AV_ERROR_MAX_STRING_SIZE] = {};
            av_strerror(err, buf, sizeof(buf));
            return std::string(buf);
        }

        inline AVPixelFormat find_hw_upload_format(const AVCodec *codec)
        {
            // pick the first HW pix_fmt that supports a device context
            for (int i = 0; const AVCodecHWConfig *cfg = avcodec_get_hw_config(codec, i); ++i)
            {
                if (cfg->methods & AV_CODEC_HW_CONFIG_METHOD_HW_DEVICE_CTX)
                {
                    return cfg->pix_fmt;
                }
            }
            return AV_PIX_FMT_NONE;
        }
    }

    class VideoSink
    {
    public:
        enum class Codec
        {
            H264,
            H265,
            AV1
        };
        struct Options
        {
            int width = 3840;
            int height = 2160;
            int fps = 80;
            Codec codec = Codec::H264;
            std::string filename = "out.mp4";
            bool use_hw = true;     // try GPU encoder first
            int bitrate_mb_s = 300; // nominal bit-rate (Mb/s)
            int gop = 40;           // intra period (frames)
            int max_queue = 32;     // #frames held before push() blocks

            enum class HWAccel
            {
                NVENC,
                QSV,
                AMF,
                VAAPI,
                OPENCL,
                CPU
            };
            std::vector<HWAccel> hw_accel_priority = {
                HWAccel::NVENC, HWAccel::QSV,
                HWAccel::AMF, HWAccel::VAAPI,
                HWAccel::OPENCL, HWAccel::CPU};
        };

        explicit VideoSink(const Options &opt)
            : opt_{opt}
        {
            av_log_set_level(AV_LOG_ERROR);
            init_ffmpeg();
            worker_ = std::thread(&VideoSink::encode_loop_, this);
        }

        ~VideoSink()
        {
            stop_.store(true, std::memory_order_release);
            cv_.notify_all();
            if (worker_.joinable())
                worker_.join();
            cleanup_ffmpeg();
        }

        void push(const cv::Mat &bgr)
        {
            if (bgr.empty())
                return;
            std::unique_lock lk(m_);
            cv_.wait(lk, [&]
                     { return static_cast<int>(queue_.size()) < opt_.max_queue; });
            queue_.emplace(bgr.clone());
            cv_.notify_one();
        }

    private:
        Options opt_;

        // FFmpeg contexts
        AVFormatContext *oc_ = nullptr;
        AVStream *stream_ = nullptr;
        AVCodecContext *codec_ctx_ = nullptr;
        const AVCodec *codec_ = nullptr;

        // HW contexts
        AVBufferRef *hw_device_ctx_ = nullptr;
        AVBufferRef *hw_frames_ctx_ = nullptr;
        AVPixelFormat hw_pix_fmt_ = AV_PIX_FMT_NONE;

        // software conversion
        SwsContext *sws_ctx_ = nullptr;
        AVFrame *sw_frame_ = nullptr;

        // encode thread
        std::mutex m_;
        std::condition_variable cv_;
        std::queue<cv::Mat> queue_;
        std::atomic<bool> stop_{false};
        std::thread worker_;

        // Try initializing a hardware encoder for one accel type.
        bool try_init_hw_encoder_(Options::HWAccel accel)
        {
            std::string base = opt_.codec == Codec::H264   ? "h264"
                               : opt_.codec == Codec::H265 ? "hevc"
                                                           : "av1";
            std::string codec_name;
            AVHWDeviceType hw_type = AV_HWDEVICE_TYPE_NONE;

            switch (accel)
            {
            case Options::HWAccel::NVENC:
                codec_name = base + "_nvenc";
                hw_type = AV_HWDEVICE_TYPE_CUDA;
                break;
            case Options::HWAccel::QSV:
                codec_name = base + "_qsv";
                hw_type = AV_HWDEVICE_TYPE_QSV;
                break;
#ifdef AV_HWDEVICE_TYPE_AMF
            case Options::HWAccel::AMF:
                codec_name = base + "_amf";
                hw_type = AV_HWDEVICE_TYPE_AMF;
                break;
#endif
            case Options::HWAccel::VAAPI:
                codec_name = base + "_vaapi";
                hw_type = AV_HWDEVICE_TYPE_VAAPI;
                break;
#ifdef AV_HWDEVICE_TYPE_OPENCL
            case Options::HWAccel::OPENCL:
                codec_name = base + "_opencl";
                hw_type = AV_HWDEVICE_TYPE_OPENCL;
                break;
#endif
            default:
                throw std::runtime_error("invalid or unsupported HW accel in try_init_hw_encoder_");
            }

            // find encoder by name
            codec_ = avcodec_find_encoder_by_name(codec_name.c_str());
            if (!codec_)
                throw std::runtime_error("encoder not found: " + codec_name);

            // new stream & context
            stream_ = avformat_new_stream(oc_, nullptr);
            codec_ctx_ = avcodec_alloc_context3(codec_);
            codec_ctx_->width = opt_.width;
            codec_ctx_->height = opt_.height;
            codec_ctx_->time_base = {1, opt_.fps};
            codec_ctx_->framerate = {opt_.fps, 1};
            codec_ctx_->gop_size = opt_.gop;
            codec_ctx_->bit_rate = int64_t(opt_.bitrate_mb_s) * 1'000'000;
            codec_ctx_->codec_type = AVMEDIA_TYPE_VIDEO;

            // pick HW-friendly pix_fmt
            hw_pix_fmt_ = detail::find_hw_upload_format(codec_);
            if (hw_pix_fmt_ == AV_PIX_FMT_NONE)
                throw std::runtime_error("no suitable HW pix_fmt for " + codec_name);

            // create device context
            int e = av_hwdevice_ctx_create(&hw_device_ctx_, hw_type, nullptr, nullptr, 0);
            if (e < 0)
                throw std::runtime_error("hwdevice_ctx_create: " + detail::ffmpeg_err(e));

            codec_ctx_->hw_device_ctx = av_buffer_ref(hw_device_ctx_);
            codec_ctx_->pix_fmt = hw_pix_fmt_;

            // open encoder
            if ((e = avcodec_open2(codec_ctx_, codec_, nullptr)) < 0)
                throw std::runtime_error("avcodec_open2: " + detail::ffmpeg_err(e));
            if ((e = avcodec_parameters_from_context(stream_->codecpar, codec_ctx_)) < 0)
                throw std::runtime_error("avcodec_parameters_from_context: " + detail::ffmpeg_err(e));

            stream_->time_base = codec_ctx_->time_base;
            return true;
        }

        // Initialize CPU encoder path.
        void init_cpu_encoder_()
        {
            // select fallback
            if (opt_.codec == Codec::H264)
                codec_ = avcodec_find_encoder_by_name("libx264");
            else if (opt_.codec == Codec::H265)
                codec_ = avcodec_find_encoder_by_name("libx265");
            else
                codec_ = avcodec_find_encoder_by_name("libaom-av1");

            if (!codec_)
                throw std::runtime_error("No suitable CPU encoder found");

            // decide container format
            std::string fn = opt_.filename;
            std::string ext = fn.substr(fn.find_last_of('.') + 1);
            for (auto &c : ext)
                c = tolower(c);
            const char *container = ext == "mp4"   ? "mp4"
                                    : ext == "mkv" ? "matroska"
                                    : ext == "avi" ? "avi"
                                                   : "mp4";

            // check compatibility
            const AVOutputFormat *ofmt = av_guess_format(container, nullptr, nullptr);
            if (ofmt)
            {
                bool ok = false;
                enum AVCodecID cid = codec_->id;
                for (auto const *const *tags = ofmt->codec_tag; *tags; ++tags)
                {
                    if (av_codec_get_tag(ofmt->codec_tag, cid))
                    {
                        ok = true;
                        break;
                    }
                }
                if (!ok && opt_.codec == Codec::AV1)
                {
                    codec_ = avcodec_find_encoder_by_name("libx264");
                    opt_.codec = Codec::H264;
                    std::cerr << "[VideoSink] AV1 not supported in " << container << ", fallback to H264\n";
                }
            }

            // new stream & context
            stream_ = avformat_new_stream(oc_, nullptr);
            codec_ctx_ = avcodec_alloc_context3(codec_);
            codec_ctx_->width = opt_.width;
            codec_ctx_->height = opt_.height;
            codec_ctx_->time_base = {1, opt_.fps};
            codec_ctx_->framerate = {opt_.fps, 1};
            codec_ctx_->gop_size = opt_.gop;
            codec_ctx_->max_b_frames = 0;
            codec_ctx_->bit_rate = int64_t(opt_.bitrate_mb_s) * 1'000'000;
            codec_ctx_->pix_fmt = AV_PIX_FMT_YUV420P;
            codec_ctx_->codec_type = AVMEDIA_TYPE_VIDEO;

            // tuning
            if (!strcmp(codec_->name, "libx264"))
            {
                av_opt_set(codec_ctx_->priv_data, "preset", "ultrafast", 0);
                av_opt_set(codec_ctx_->priv_data, "tune", "zerolatency", 0);
                av_opt_set(codec_ctx_->priv_data, "profile", "baseline", 0);
            }
            else if (!strcmp(codec_->name, "libx265"))
            {
                av_opt_set(codec_ctx_->priv_data, "preset", "ultrafast", 0);
                av_opt_set(codec_ctx_->priv_data, "tune", "zerolatency", 0);
            }
            else if (!strcmp(codec_->name, "libaom-av1"))
            {
                av_opt_set(codec_ctx_->priv_data, "cpu-used", "8", 0);
                av_opt_set(codec_ctx_->priv_data, "usage", "realtime", 0);
            }

            int e;
            if ((e = avcodec_open2(codec_ctx_, codec_, nullptr)) < 0)
                throw std::runtime_error("avcodec_open2(CPU): " + detail::ffmpeg_err(e));
            if ((e = avcodec_parameters_from_context(stream_->codecpar, codec_ctx_)) < 0)
                throw std::runtime_error("avcodec_parameters_from_context: " + detail::ffmpeg_err(e));

            stream_->time_base = codec_ctx_->time_base;
        }

        void init_hw_frames_()
        {
            int e;
            hw_frames_ctx_ = av_hwframe_ctx_alloc(hw_device_ctx_);
            if (!hw_frames_ctx_)
                throw std::runtime_error("av_hwframe_ctx_alloc failed");
            auto *fctx = (AVHWFramesContext *)hw_frames_ctx_->data;
            fctx->format = hw_pix_fmt_;
            fctx->sw_format = AV_PIX_FMT_NV12;
            fctx->width = opt_.width;
            fctx->height = opt_.height;
            fctx->initial_pool_size = opt_.max_queue + 4;
            if ((e = av_hwframe_ctx_init(hw_frames_ctx_)) < 0)
                throw std::runtime_error("av_hwframe_ctx_init: " + detail::ffmpeg_err(e));

            sw_frame_ = av_frame_alloc();
            sw_frame_->format = fctx->sw_format;
            sw_frame_->width = opt_.width;
            sw_frame_->height = opt_.height;
            av_frame_get_buffer(sw_frame_, 0);

            sws_ctx_ = sws_getContext(
                opt_.width, opt_.height, AV_PIX_FMT_BGR24,
                opt_.width, opt_.height, fctx->sw_format,
                SWS_BILINEAR, nullptr, nullptr, nullptr);

            codec_ctx_->hw_frames_ctx = av_buffer_ref(hw_frames_ctx_);
        }

        void init_ffmpeg()
        {
            // 1) pick container by extension (H.264 → “mp4”, H.265 → “hevc” if you want raw, etc.)
            std::string fn  = opt_.filename;
            std::string ext = fn.substr(fn.find_last_of('.') + 1);
            for (auto &c : ext) c = tolower(c);
            const char *container =
                (opt_.codec == Codec::H264) ? "mp4"     // H.264 in MP4
              : (opt_.codec == Codec::H265 && ext == "hevc") ? "hevc"  // raw H.265
              : (ext == "mp4")  ? "mp4"
              : (ext == "mkv")  ? "matroska"
              : (ext == "avi")  ? "avi"
              :                  "mp4";

            // 2) allocate format context with explicit muxer
            int err;
            if ((err = avformat_alloc_output_context2(&oc_, nullptr, nullptr, fn.c_str())) < 0)
                throw std::runtime_error("avformat_alloc_output_context2: " + detail::ffmpeg_err(err));

            bool got = false;
            std::string errs;

            for (auto accel : opt_.hw_accel_priority)
            {
                try
                {
                    if (accel == Options::HWAccel::CPU)
                    {
                        opt_.use_hw = false;
                        hw_pix_fmt_ = AV_PIX_FMT_NONE;
                        init_cpu_encoder_();
                    }
                    else
                    {
                        if (!try_init_hw_encoder_(accel))
                            throw std::runtime_error("unsupported");
                    }
                    got = true;
                    break;
                }
                catch (std::exception &ex)
                {
                    errs += get_hw_accel_name_(accel) + std::string(": ") + ex.what() + "; ";
                }
            }
            if (!got)
                throw std::runtime_error("Failed to init any encoder: " + errs);

            // 3) make sure the stream parameters match the codec
            if ((err = avcodec_parameters_from_context(stream_->codecpar, codec_ctx_)) < 0)
                throw std::runtime_error("avcodec_parameters_from_context: " + detail::ffmpeg_err(err));

            // 4) open the file & write header
            if (!(oc_->oformat->flags & AVFMT_NOFILE))
                if ((err = avio_open(&oc_->pb, fn.c_str(), AVIO_FLAG_WRITE)) < 0)
                    throw std::runtime_error("avio_open: " + detail::ffmpeg_err(err));
            if ((err = avformat_write_header(oc_, nullptr)) < 0)
                throw std::runtime_error("avformat_write_header: " + detail::ffmpeg_err(err));

            if (opt_.use_hw)
                init_hw_frames_();
            else
            {
                sw_frame_ = av_frame_alloc();
                sw_frame_->format = codec_ctx_->pix_fmt;
                sw_frame_->width = opt_.width;
                sw_frame_->height = opt_.height;
                av_frame_get_buffer(sw_frame_, 0);
                sws_ctx_ = sws_getContext(
                    opt_.width, opt_.height, AV_PIX_FMT_BGR24,
                    opt_.width, opt_.height, codec_ctx_->pix_fmt,
                    SWS_BILINEAR, nullptr, nullptr, nullptr);
            }
        }

        void cleanup_ffmpeg()
        {
            if (codec_ctx_)
            {
                avcodec_send_frame(codec_ctx_, nullptr);
                AVPacket pkt;
                av_init_packet(&pkt);
                while (avcodec_receive_packet(codec_ctx_, &pkt) == 0)
                {
                    pkt.stream_index = stream_->index;
                    av_interleaved_write_frame(oc_, &pkt);
                    av_packet_unref(&pkt);
                }
            }
            if (oc_)
            {
                av_write_trailer(oc_);
                if (!(oc_->oformat->flags & AVFMT_NOFILE) && oc_->pb)
                    avio_closep(&oc_->pb);
                avformat_free_context(oc_);
            }
            if (codec_ctx_)
                avcodec_free_context(&codec_ctx_);
            if (hw_device_ctx_)
                av_buffer_unref(&hw_device_ctx_);
            if (hw_frames_ctx_)
                av_buffer_unref(&hw_frames_ctx_);
            if (sw_frame_)
                av_frame_free(&sw_frame_);
            if (sws_ctx_)
                sws_freeContext(sws_ctx_);
        }

        const char *get_hw_accel_name_(Options::HWAccel a)
        {
            switch (a)
            {
            case Options::HWAccel::NVENC:
                return "NVENC";
            case Options::HWAccel::QSV:
                return "QSV";
            case Options::HWAccel::AMF:
                return "AMF";
            case Options::HWAccel::VAAPI:
                return "VAAPI";
            case Options::HWAccel::OPENCL:
                return "OPENCL";
            case Options::HWAccel::CPU:
                return "CPU";
            default:
                return "Unknown";
            }
        }

        void encode_loop_()
        {
            AVPacket pkt;
            av_init_packet(&pkt);
            int64_t pts = 0;

            while (true)
            {
                cv::Mat bgr;
                {
                    std::unique_lock lk(m_);
                    cv_.wait(lk, [&]
                             { return !queue_.empty() || stop_.load(); });
                    if (queue_.empty() && stop_.load())
                        break;
                    bgr = std::move(queue_.front());
                    queue_.pop();
                    cv_.notify_one();
                }
                if (bgr.empty())
                    continue;

                // convert
                const uint8_t *in[] = {bgr.data};
                int stride[] = {int(bgr.step)};
                sws_scale(sws_ctx_, in, stride, 0, opt_.height, sw_frame_->data, sw_frame_->linesize);

                AVFrame *to_send = nullptr, *hwf = nullptr;
                if (opt_.use_hw)
                {
                    hwf = av_frame_alloc();
                    hwf->format = hw_pix_fmt_;
                    if (av_hwframe_get_buffer(hw_frames_ctx_, hwf, 0) < 0)
                        throw std::runtime_error("av_hwframe_get_buffer failed");
                    if (av_hwframe_transfer_data(hwf, sw_frame_, 0) < 0)
                        throw std::runtime_error("av_hwframe_transfer_data failed");
                    av_frame_unref(sw_frame_);
                    to_send = hwf;
                }
                else
                {
                    to_send = sw_frame_;
                }

                to_send->pts = pts++;
                int e = avcodec_send_frame(codec_ctx_, to_send);
                if (e < 0)
                    throw std::runtime_error("avcodec_send_frame: " + detail::ffmpeg_err(e));
                if (hwf)
                    av_frame_free(&hwf);

                while ((e = avcodec_receive_packet(codec_ctx_, &pkt)) == 0)
                {
                    pkt.stream_index = stream_->index;
                    pkt.duration = 1;
                    av_packet_rescale_ts(&pkt, codec_ctx_->time_base, stream_->time_base);
                    av_interleaved_write_frame(oc_, &pkt);
                    av_packet_unref(&pkt);
                }
                if (e != AVERROR(EAGAIN) && e != AVERROR_EOF)
                    throw std::runtime_error("avcodec_receive_packet: " + detail::ffmpeg_err(e));
            }
        }
    };

} // namespace boblib::video

#ifdef VIDEOSINK_DEMO
#include <opencv2/highgui.hpp>
using namespace boblib::video;
int main()
{
    VideoSink::Options opt;
    opt.width = 5472;
    opt.height = 3648;
    opt.fps = 80;
    opt.codec = VideoSink::Codec::H265;
    opt.filename = "sample.hevc";
    VideoSink sink(opt);
    cv::Mat frame(opt.height, opt.width, CV_8UC3, cv::Scalar::all(127));
    for (int i = 0; i < opt.fps * 5; ++i)
    {
        cv::putText(frame, std::to_string(i), {100, 200}, cv::FONT_HERSHEY_SIMPLEX,
                    5.0, {0, 255, 0}, 10);
        sink.push(frame);
    }
    return 0;
}
#endif