#include "ffmpeg_video_reader.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sstream> // for std::ostringstream
#include <linux/videodev2.h>
#include <libavutil/pixfmt.h> // for av_get_pix_fmt_name()
#include <libavcodec/avcodec.h>  // for avcodec_get_name()
#include <libavutil/hwcontext.h> // for AVHWDeviceContext & av_hwdevice_get_type_name()

namespace boblib::video
{
    FFmpegVideoReader::FFmpegVideoReader()
    {
        avformat_network_init();
        allocate_resources();
    }

    FFmpegVideoReader::FFmpegVideoReader(const std::string &src)
        : FFmpegVideoReader()
    {
        if (!open(src))
        {
            std::cerr << "Failed to open source: " << src << std::endl;
            throw std::runtime_error("Failed to open source");
        }
    }

    FFmpegVideoReader::~FFmpegVideoReader()
    {
        close();
    }

    bool FFmpegVideoReader::open(int camera_id, int width, int height)
    {
        auto src = "/dev/video" + std::to_string(camera_id);
        return open(src, width, height);
    }

    /// open / reopen a source (file path, RTSP url, or /dev/video* device) – returns true on success
    bool FFmpegVideoReader::open(const std::string &src, int width, int height)
    {
        close(); // in case already open
        allocate_resources();
        src_ = src;

        // set up interrupt callback timeout
        probe_start_us_ = av_gettime_relative();
        fmt_ctx_ = avformat_alloc_context();
        fmt_ctx_->interrupt_callback.callback = &FFmpegVideoReader::interrupt_cb;
        fmt_ctx_->interrupt_callback.opaque = this;

        AVDictionary *opts = nullptr;

        const AVInputFormat *input_format = nullptr;
        if (src.starts_with("/dev/video"))
        {
            input_format = av_find_input_format("video4linux2");
            auto resolution = std::to_string(width) + "x" + std::to_string(height);
            av_dict_set(&opts, "video_size", resolution.c_str(), 0);
        }

        // use TCP for RTSP
        av_dict_set(&opts, "rtsp_transport", "tcp", 0);
        // increase probe buffer to 1 MiB
        av_dict_set(&opts, "probesize", "1048576", 0);
        // analyze up to 2 seconds of data
        av_dict_set(&opts, "analyzeduration", "2000000", 0);
        // network read timeout 2 s
        av_dict_set(&opts, "stimeout", "2000000", 0);
        if (avformat_open_input(&fmt_ctx_, src.c_str(), input_format, &opts) < 0)
        {
            av_dict_free(&opts);
            return false;
        }
        av_dict_free(&opts);

        fmt_ctx_->flags |= AVFMT_FLAG_NONBLOCK;
        if (avformat_find_stream_info(fmt_ctx_, nullptr) < 0)
            return false;

        // locate first video stream
        video_stream_index_ = av_find_best_stream(fmt_ctx_, AVMEDIA_TYPE_VIDEO, -1, -1, &decoder_, 0);
        if (video_stream_index_ < 0)
            return false;
        video_stream_ = fmt_ctx_->streams[video_stream_index_];

        if (!init_decoder())
            return false;
        width_ = codec_ctx_->width;
        height_ = codec_ctx_->height;
        fps_ = av_q2d(video_stream_->r_frame_rate);
        is_opened_ = true;

        // after avformat_find_stream_info() and before returning:
        fmt_ctx_->interrupt_callback.callback = nullptr;
        fmt_ctx_->interrupt_callback.opaque = nullptr;
        fmt_ctx_->flags &= ~AVFMT_FLAG_NONBLOCK;
        return true;
    }

    /// read next frame; returns true if a frame was read (false ⇒ EOF)
    bool FFmpegVideoReader::read(cv::Mat &out)
    {
        // reset our timeout so interrupt_cb won't fire mid-stream
        probe_start_us_ = av_gettime_relative();

        while (true)
        {
            int ret = av_read_frame(fmt_ctx_, pkt_);
            if (ret < 0)
            {
                char errbuf[AV_ERROR_MAX_STRING_SIZE] = {0};
                av_strerror(ret, errbuf, sizeof(errbuf));
                std::cerr << "[ERROR] av_read_frame failed: " << errbuf << " (" << ret << ")" << std::endl;
                break;
            }

            if (pkt_->stream_index != video_stream_index_)
            {
                av_packet_unref(pkt_);
                continue;
            }

            if (avcodec_send_packet(codec_ctx_, pkt_) < 0)
            {
                std::cerr << "[ERROR] avcodec_send_packet failed" << std::endl;
                av_packet_unref(pkt_);
                continue;
            }

            while (true)
            {
                ret = avcodec_receive_frame(codec_ctx_, frame_);
                if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
                    break;
                if (ret < 0)
                {
                    std::cerr << "[ERROR] receive_frame error " << ret << std::endl;
                    av_packet_unref(pkt_);
                    return false;
                }

                AVFrame *use = frame_;
                if (frame_->format == hw_pix_fmt_)
                {
                    if (av_hwframe_transfer_data(sw_frame_, frame_, 0) < 0)
                    {
                        std::cerr << "[ERROR] hwframe_transfer_data failed" << std::endl;
                        av_packet_unref(pkt_);
                        return false;
                    }
                    use = sw_frame_;
                }

                out = avframe_to_mat(use);

                av_frame_unref(frame_);
                av_packet_unref(pkt_);
                return true;
            }
            av_packet_unref(pkt_);
        }
        return false;
    }

    void FFmpegVideoReader::close()
    {
        if (fmt_ctx_)
            avformat_close_input(&fmt_ctx_);
        if (codec_ctx_)
            avcodec_free_context(&codec_ctx_);
        if (hw_device_ctx_)
            av_buffer_unref(&hw_device_ctx_);
        if (frame_)
            av_frame_free(&frame_);
        if (sw_frame_)
            av_frame_free(&sw_frame_);
        if (pkt_)
            av_packet_free(&pkt_);
        if (sws_ctx_)
            sws_freeContext(sws_ctx_);
        fmt_ctx_ = nullptr;
        codec_ctx_ = nullptr;
        sws_ctx_ = nullptr;
        hw_device_ctx_ = nullptr;
        is_opened_ = false;
    }

    int FFmpegVideoReader::get_width() const { return width_; }
    int FFmpegVideoReader::get_height() const { return height_; }
    double FFmpegVideoReader::get_fps() const { return fps_; }
    bool FFmpegVideoReader::is_opened() const { return is_opened_; }

    void FFmpegVideoReader::allocate_resources()
    {
        pkt_ = av_packet_alloc();
        frame_ = av_frame_alloc();
        sw_frame_ = av_frame_alloc();
    }

    bool FFmpegVideoReader::init_decoder()
    {
        codec_ctx_ = avcodec_alloc_context3(decoder_);
        if (!codec_ctx_)
        {
            std::cerr << "[ERROR] avcodec_alloc_context3 failed" << std::endl;
            return false;
        }
        if (avcodec_parameters_to_context(
                codec_ctx_, video_stream_->codecpar) < 0)
        {
            std::cerr << "[ERROR] avcodec_parameters_to_context failed" << std::endl;
            return false;
        }

        codec_ctx_->opaque = this;
        for (int i = 0;; ++i)
        {
            const AVCodecHWConfig *cfg = avcodec_get_hw_config(decoder_, i);
            if (!cfg)
                break;
            if (cfg->methods & AV_CODEC_HW_CONFIG_METHOD_HW_DEVICE_CTX)
            {
                if (init_hw_device(cfg->device_type))
                {
                    hw_pix_fmt_ = cfg->pix_fmt;
                    codec_ctx_->get_format = [](AVCodecContext *ctx,
                                                const AVPixelFormat *pix)
                    {
                        auto *self = static_cast<FFmpegVideoReader *>(ctx->opaque);
                        for (; *pix != AV_PIX_FMT_NONE; ++pix)
                            if (*pix == self->hw_pix_fmt_)
                                return *pix;
                        return pix[0];
                    };
                    break;
                }
                else
                {
                    std::cerr << "[WARN] init_hw_device("
                              << cfg->device_type << ") failed" << std::endl;
                }
            }
        }

        if (avcodec_open2(codec_ctx_, decoder_, nullptr) < 0)
        {
            std::cerr << "[ERROR] avcodec_open2 failed" << std::endl;
            return false;
        }
        return true;
    }

    bool FFmpegVideoReader::init_hw_device(AVHWDeviceType type)
    {
        if (type == AV_HWDEVICE_TYPE_NONE)
            return false;
        if (av_hwdevice_ctx_create(&hw_device_ctx_, type, nullptr, nullptr, 0) < 0)
        {
            std::cerr << "[ERROR] av_hwdevice_ctx_create failed" << std::endl;
            hw_device_ctx_ = nullptr;
            return false;
        }
        codec_ctx_->hw_device_ctx = av_buffer_ref(hw_device_ctx_);
        return true;
    }

    cv::Mat FFmpegVideoReader::avframe_to_mat(const AVFrame *src)
    {
        if (!sws_ctx_ || src->width != width_ || src->height != height_)
        {
            if (sws_ctx_)
                sws_freeContext(sws_ctx_);
            sws_ctx_ = sws_getContext(src->width, src->height, static_cast<AVPixelFormat>(src->format),
                                      src->width, src->height, AV_PIX_FMT_BGR24,
                                      SWS_BILINEAR, nullptr, nullptr, nullptr);
            width_ = src->width;
            height_ = src->height;
        }
        cv::Mat dst(height_, width_, CV_8UC3);
        uint8_t *dst_data[1] = {dst.data};
        int dst_linesize[1] = {static_cast<int>(dst.step)};
        sws_scale(sws_ctx_, src->data, src->linesize, 0, src->height, dst_data, dst_linesize);
        return dst;
    }

    std::string FFmpegVideoReader::get_codec_name() const
    {
        if (!video_stream_) return {};
        return avcodec_get_name(video_stream_->codecpar->codec_id);
    }

    std::string FFmpegVideoReader::get_decoder_name() const
    {
        // if we have an active AVCodecContext, report codec name + SW/HW status
        if (codec_ctx_ && codec_ctx_->codec)
        {
            const char *codec_name = codec_ctx_->codec->name;
            if (codec_ctx_->hw_device_ctx)
            {
                auto *hw_ctx = reinterpret_cast<AVHWDeviceContext *>(codec_ctx_->hw_device_ctx->data);
                const char *hw_type_name = av_hwdevice_get_type_name(hw_ctx->type);
                std::ostringstream oss;
                oss << codec_name
                    << " (hardware accelerated"
                    << (hw_type_name ? ", type=" : "")
                    << (hw_type_name ? hw_type_name : "unknown")
                    << ")";
                return oss.str();
            }
            else
            {
                std::ostringstream oss;
                oss << codec_name << " (software)";
                return oss.str();
            }
        }

        // fallback to the AVCodec we found at stream opening
        return decoder_ ? decoder_->long_name : std::string{};
    }

    std::string FFmpegVideoReader::get_pixel_format_name() const
    {
        // prefer hw_pix_fmt_ if used, otherwise codec_ctx_->pix_fmt
        AVPixelFormat pf = codec_ctx_
            ? (codec_ctx_->sw_pix_fmt == AV_PIX_FMT_NONE
                  ? codec_ctx_->pix_fmt
                  : codec_ctx_->sw_pix_fmt)
            : AV_PIX_FMT_NONE;
        const char *name = av_get_pix_fmt_name(pf);
        return name ? name : std::string{};
    }

    bool FFmpegVideoReader::set_resolution(int width, int height)
    {
        if (!src_.starts_with("/dev/video"))
        {
            return false;
        }
        auto resolutions = list_camera_resolutions();
        if (resolutions.empty())
            return false;
        for (const auto &res : resolutions)
        {
            if (res.first == width && res.second == height)
            {
                return open(src_, width, height);
            }
        }
        return false;
    }

    std::vector<std::pair<int, int>> FFmpegVideoReader::list_camera_resolutions() const
    {
        std::vector<std::pair<int, int>> resolutions;
        int fd = ::open(src_.c_str(), O_RDWR);
        if (fd < 0)
            return resolutions;

        // enumerate all pixel formats
        v4l2_fmtdesc fmtdesc = {};
        fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        for (fmtdesc.index = 0;
             ioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc) == 0;
             ++fmtdesc.index)
        {
            // for each format, enumerate frame sizes
            v4l2_frmsizeenum frmsize = {};
            frmsize.pixel_format = fmtdesc.pixelformat;
            for (frmsize.index = 0;
                 ioctl(fd, VIDIOC_ENUM_FRAMESIZES, &frmsize) == 0;
                 ++frmsize.index)
            {
                if (frmsize.type == V4L2_FRMSIZE_TYPE_DISCRETE)
                {
                    resolutions.emplace_back(
                        frmsize.discrete.width,
                        frmsize.discrete.height);
                }
                else if (frmsize.type == V4L2_FRMSIZE_TYPE_STEPWISE)
                {
                    for (unsigned h = frmsize.stepwise.min_height;
                         h <= frmsize.stepwise.max_height;
                         h += frmsize.stepwise.step_height)
                        for (unsigned w = frmsize.stepwise.min_width;
                             w <= frmsize.stepwise.max_width;
                             w += frmsize.stepwise.step_width)
                            resolutions.emplace_back(w, h);
                }
            }
        }

        ::close(fd);
        return resolutions;
    }
}  // namespace boblib::video