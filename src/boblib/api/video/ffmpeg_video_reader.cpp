#include "ffmpeg_video_reader.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sstream> // for std::ostringstream
#include <linux/videodev2.h>
#include <libavutil/pixfmt.h>    // for av_get_pix_fmt_name()
#include <libavcodec/avcodec.h>  // for avcodec_get_name()
#include <libavutil/hwcontext.h> // for AVHWDeviceContext & av_hwdevice_get_type_name()
#include <cstdlib>               // for getenv

namespace boblib::video
{
    FFmpegVideoReader::FFmpegVideoReader()
    {
        try {
            avformat_network_init();
            allocate_resources();
        } catch (const std::exception& e) {
            std::cerr << "[ERROR] FFmpegVideoReader constructor failed: " << e.what() << std::endl;
            // Ensure clean state
            close();
        } catch (...) {
            std::cerr << "[ERROR] FFmpegVideoReader constructor failed with unknown exception" << std::endl;
            close();
        }
    }

    FFmpegVideoReader::~FFmpegVideoReader()
    {
        try {
            close();
        } catch (...) {
            // Never throw from destructor
            std::cerr << "[ERROR] Exception in FFmpegVideoReader destructor" << std::endl;
        }
    }

    bool FFmpegVideoReader::open(int camera_id, int width, int height)
    {
        if (camera_id < 0) {
            std::cerr << "[ERROR] Invalid camera_id: " << camera_id << std::endl;
            return false;
        }
        if (width < 0 || height < 0) {
            std::cerr << "[ERROR] Invalid resolution: " << width << "x" << height << std::endl;
            return false;
        }
        
        auto src = "/dev/video" + std::to_string(camera_id);
        return open(src, width, height);
    }

    /// open / reopen a source (file path, RTSP url, or /dev/video* device) – returns true on success
    bool FFmpegVideoReader::open(const std::string &src, int width, int height)
    {
        if (src.empty()) {
            std::cerr << "[ERROR] Empty source string" << std::endl;
            return false;
        }
        if (width < 0 || height < 0) {
            std::cerr << "[ERROR] Invalid resolution: " << width << "x" << height << std::endl;
            return false;
        }

        try {
            close(); // in case already open
            allocate_resources();
            
            if (!pkt_ || !frame_ || !sw_frame_) {
                std::cerr << "[ERROR] Failed to allocate FFmpeg resources" << std::endl;
                return false;
            }
            
            src_ = src;

            // set up interrupt callback timeout
            probe_start_us_ = av_gettime_relative();
            fmt_ctx_ = avformat_alloc_context();
            if (!fmt_ctx_) {
                std::cerr << "[ERROR] avformat_alloc_context failed" << std::endl;
                return false;
            }
            
            fmt_ctx_->interrupt_callback.callback = &FFmpegVideoReader::interrupt_cb;
            fmt_ctx_->interrupt_callback.opaque = this;

            AVDictionary *opts = nullptr;

            const AVInputFormat *input_format = nullptr;
            if (src.starts_with("/dev/video"))
            {
                input_format = av_find_input_format("video4linux2");
                if (!input_format) {
                    std::cerr << "[ERROR] video4linux2 input format not found" << std::endl;
                    av_dict_free(&opts);
                    return false;
                }
                auto resolution = std::to_string(width) + "x" + std::to_string(height);
                av_dict_set(&opts, "video_size", resolution.c_str(), 0);
            }

            // use TCP for RTSP
            av_dict_set(&opts, "rtsp_transport", "tcp", 0);
            // probe buffer 10 MiB
            av_dict_set(&opts, "probesize", "10485760", 0);
            // analyze up to 4 seconds of data
            av_dict_set(&opts, "analyzeduration", "4000000", 0);
            // network read timeout 4s
            av_dict_set(&opts, "stimeout", "5000000", 0);
            
            if (avformat_open_input(&fmt_ctx_, src.c_str(), input_format, &opts) < 0)
            {
                std::cerr << "[ERROR] avformat_open_input failed for source: " << src << std::endl;
                av_dict_free(&opts);
                return false;
            }
            av_dict_free(&opts);

            if (!fmt_ctx_) {
                std::cerr << "[ERROR] fmt_ctx_ is null after avformat_open_input" << std::endl;
                return false;
            }

            fmt_ctx_->flags |= AVFMT_FLAG_NONBLOCK;
            if (avformat_find_stream_info(fmt_ctx_, nullptr) < 0)
            {
                std::cerr << "[ERROR] avformat_find_stream_info failed" << std::endl;
                return false;
            }

            // locate first video stream
            video_stream_index_ = av_find_best_stream(fmt_ctx_, AVMEDIA_TYPE_VIDEO, -1, -1, &decoder_, 0);
            if (video_stream_index_ < 0)
            {
                std::cerr << "[ERROR] No video stream found" << std::endl;
                return false;
            }
            
            if (video_stream_index_ >= static_cast<int>(fmt_ctx_->nb_streams)) {
                std::cerr << "[ERROR] Invalid video stream index" << std::endl;
                return false;
            }
            
            video_stream_ = fmt_ctx_->streams[video_stream_index_];
            if (!video_stream_) {
                std::cerr << "[ERROR] Video stream is null" << std::endl;
                return false;
            }

            if (!init_decoder())
            {
                std::cerr << "[ERROR] Decoder initialization failed" << std::endl;
                return false;
            }
            
            if (!codec_ctx_) {
                std::cerr << "[ERROR] Codec context is null after init_decoder" << std::endl;
                return false;
            }
            
            width_ = codec_ctx_->width;
            height_ = codec_ctx_->height;

            // Validate dimensions
            if (width_ <= 0 || height_ <= 0 || width_ > 16384 || height_ > 16384) {
                std::cerr << "[ERROR] Invalid video dimensions: " << width_ << "x" << height_ << std::endl;
                return false;
            }

            // Try multiple sources for FPS
            fps_ = 0.0;
            // 1. Try avg_frame_rate first (often more accurate for RTSP)
            if (video_stream_->avg_frame_rate.num != 0 && video_stream_->avg_frame_rate.den != 0)
            {
                fps_ = av_q2d(video_stream_->avg_frame_rate);
            }
            // 2. Fall back to r_frame_rate if avg_frame_rate is invalid
            if (fps_ <= 0.0 || fps_ > 1000.0)
            {
                fps_ = av_q2d(video_stream_->r_frame_rate);
            }
            // 3. Check if the value is reasonable
            if (fps_ <= 0.0 || fps_ > 1000.0)
            {
                // Set a reasonable default if both values are invalid
                fps_ = 30.0;
                std::cerr << "[WARN] Could not determine valid FPS from stream, using default: " << fps_ << std::endl;
            }

            is_opened_ = true;

            // after avformat_find_stream_info() and before returning:
            fmt_ctx_->interrupt_callback.callback = nullptr;
            fmt_ctx_->interrupt_callback.opaque = nullptr;
            fmt_ctx_->flags &= ~AVFMT_FLAG_NONBLOCK;
            return true;
        } catch (const std::exception& e) {
            std::cerr << "[ERROR] Exception in open(): " << e.what() << std::endl;
            close();
            return false;
        } catch (...) {
            std::cerr << "[ERROR] Unknown exception in open()" << std::endl;
            close();
            return false;
        }
    }

    /// read next frame; returns true if a frame was read (false ⇒ EOF)
    bool FFmpegVideoReader::read(cv::Mat &out)
    {
        if (!is_opened_ || !fmt_ctx_ || !codec_ctx_ || !pkt_ || !frame_) {
            std::cerr << "[ERROR] Video reader not properly initialized" << std::endl;
            return false;
        }

        try {
            // reset our timeout so interrupt_cb won't fire mid-stream
            probe_start_us_ = av_gettime_relative();

            while (true)
            {
                int ret = av_read_frame(fmt_ctx_, pkt_);
                if (ret < 0)
                {
                    if (ret == AVERROR_EOF) {
                        // Normal end of file
                        return false;
                    }
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

                ret = avcodec_send_packet(codec_ctx_, pkt_);
                if (ret < 0)
                {
                    char errbuf[AV_ERROR_MAX_STRING_SIZE] = {0};
                    av_strerror(ret, errbuf, sizeof(errbuf));
                    std::cerr << "[ERROR] avcodec_send_packet failed: " << errbuf << " (" << ret << ")" << std::endl;
                    av_packet_unref(pkt_);
                    continue;
                }

                while (true)
                {
                    ret = avcodec_receive_frame(codec_ctx_, frame_);
                    if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
                    {
                        break;
                    }
                    if (ret < 0)
                    {
                        char errbuf[AV_ERROR_MAX_STRING_SIZE] = {0};
                        av_strerror(ret, errbuf, sizeof(errbuf));
                        std::cerr << "[ERROR] avcodec_receive_frame failed: " << errbuf << " (" << ret << ")" << std::endl;
                        av_packet_unref(pkt_);
                        return false;
                    }

                    if (!frame_->data[0]) {
                        std::cerr << "[ERROR] Received frame with no data" << std::endl;
                        av_frame_unref(frame_);
                        av_packet_unref(pkt_);
                        continue;
                    }

                    AVFrame *use = frame_;
                    if (hw_pix_fmt_ != AV_PIX_FMT_NONE && frame_->format == hw_pix_fmt_)
                    {
                        if (!sw_frame_) {
                            std::cerr << "[ERROR] Software frame buffer is null" << std::endl;
                            av_frame_unref(frame_);
                            av_packet_unref(pkt_);
                            return false;
                        }
                        
                        ret = av_hwframe_transfer_data(sw_frame_, frame_, 0);
                        if (ret < 0)
                        {
                            char errbuf[AV_ERROR_MAX_STRING_SIZE] = {0};
                            av_strerror(ret, errbuf, sizeof(errbuf));
                            std::cerr << "[ERROR] hwframe_transfer_data failed: " << errbuf << " (" << ret << ")" << std::endl;
                            av_frame_unref(frame_);
                            av_packet_unref(pkt_);
                            return false;
                        }
                        use = sw_frame_;
                    }

                    cv::Mat result = avframe_to_mat(use);
                    if (result.empty()) {
                        std::cerr << "[ERROR] Failed to convert frame to cv::Mat" << std::endl;
                        av_frame_unref(frame_);
                        av_packet_unref(pkt_);
                        return false;
                    }

                    out = result;
                    av_frame_unref(frame_);
                    av_packet_unref(pkt_);
                    return true;
                }
                av_packet_unref(pkt_);
            }
        } catch (const std::exception& e) {
            std::cerr << "[ERROR] Exception in read(): " << e.what() << std::endl;
            if (pkt_) av_packet_unref(pkt_);
            if (frame_) av_frame_unref(frame_);
            return false;
        } catch (...) {
            std::cerr << "[ERROR] Unknown exception in read()" << std::endl;
            if (pkt_) av_packet_unref(pkt_);
            if (frame_) av_frame_unref(frame_);
            return false;
        }
        return false;
    }

    void FFmpegVideoReader::close()
    {
        try {
            is_opened_ = false;
            
            if (fmt_ctx_)
            {
                avformat_close_input(&fmt_ctx_);
                fmt_ctx_ = nullptr;
            }
            if (codec_ctx_)
            {
                avcodec_free_context(&codec_ctx_);
                codec_ctx_ = nullptr;
            }
            if (hw_device_ctx_)
            {
                av_buffer_unref(&hw_device_ctx_);
                hw_device_ctx_ = nullptr;
            }
            if (frame_)
            {
                av_frame_free(&frame_);
                frame_ = nullptr;
            }
            if (sw_frame_)
            {
                av_frame_free(&sw_frame_);
                sw_frame_ = nullptr;
            }
            if (pkt_)
            {
                av_packet_free(&pkt_);
                pkt_ = nullptr;
            }
            if (sws_ctx_)
            {
                sws_freeContext(sws_ctx_);
                sws_ctx_ = nullptr;
            }
            
            // Reset other members
            decoder_ = nullptr;
            video_stream_ = nullptr;
            video_stream_index_ = -1;
            width_ = 0;
            height_ = 0;
            fps_ = 0.0;
            hw_pix_fmt_ = AV_PIX_FMT_NONE;
            probe_start_us_ = 0;
            src_.clear();
        } catch (const std::exception& e) {
            std::cerr << "[ERROR] Exception in close(): " << e.what() << std::endl;
        } catch (...) {
            std::cerr << "[ERROR] Unknown exception in close()" << std::endl;
        }
    }

    int FFmpegVideoReader::get_width() const { return width_; }
    int FFmpegVideoReader::get_height() const { return height_; }
    double FFmpegVideoReader::get_fps() const { return fps_; }
    bool FFmpegVideoReader::is_opened() const { return is_opened_; }

    void FFmpegVideoReader::allocate_resources()
    {
        try {
            pkt_ = av_packet_alloc();
            if (!pkt_) {
                throw std::runtime_error("Failed to allocate packet");
            }
            
            frame_ = av_frame_alloc();
            if (!frame_) {
                av_packet_free(&pkt_);
                pkt_ = nullptr;
                throw std::runtime_error("Failed to allocate frame");
            }
            
            sw_frame_ = av_frame_alloc();
            if (!sw_frame_) {
                av_packet_free(&pkt_);
                av_frame_free(&frame_);
                pkt_ = nullptr;
                frame_ = nullptr;
                throw std::runtime_error("Failed to allocate software frame");
            }
        } catch (const std::exception& e) {
            std::cerr << "[ERROR] allocate_resources failed: " << e.what() << std::endl;
            throw;
        }
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
        
        // Only try hardware acceleration if enabled
        bool hw_success = false;
        if (hw_accel_enabled_)
        {
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
                            {
                                if (*pix == self->hw_pix_fmt_)
                                {
                                    return *pix;
                                }
                            }
                            return pix[0];
                        };
                        hw_success = true;
                        std::cout << "[INFO] Hardware acceleration enabled with device type: " 
                                  << av_hwdevice_get_type_name(cfg->device_type) << std::endl;
                        break;
                    }
                    else
                    {
                        std::cerr << "[WARN] init_hw_device("
                                  << cfg->device_type << ") failed" << std::endl;
                    }
                }
            }
        }
        
        if (!hw_success)
        {
            std::cout << "[INFO] Using software decoding" << std::endl;
            hw_pix_fmt_ = AV_PIX_FMT_NONE;
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
            
        // Avoid trying certain hardware acceleration types in known problematic environments
        const char* type_name = av_hwdevice_get_type_name(type);
        
        // Skip CUDA if we're likely in a container without NVIDIA drivers
        if (type == AV_HWDEVICE_TYPE_CUDA)
        {
            // Try to detect if CUDA is available
            if (access("/usr/lib/x86_64-linux-gnu/libcuda.so.1", F_OK) != 0 &&
                access("/usr/lib64/libcuda.so.1", F_OK) != 0)
            {
                std::cerr << "[INFO] Skipping CUDA hardware acceleration (libcuda.so.1 not found)" << std::endl;
                return false;
            }
        }
        
        // Skip VAAPI if we're in a headless environment
        if (type == AV_HWDEVICE_TYPE_VAAPI)
        {
            if (!getenv("DISPLAY") && !getenv("WAYLAND_DISPLAY"))
            {
                std::cerr << "[INFO] Skipping VAAPI hardware acceleration (no display environment)" << std::endl;
                return false;
            }
        }
        
        int ret = av_hwdevice_ctx_create(&hw_device_ctx_, type, nullptr, nullptr, 0);
        if (ret < 0)
        {
            char errbuf[AV_ERROR_MAX_STRING_SIZE] = {0};
            av_strerror(ret, errbuf, sizeof(errbuf));
            std::cerr << "[WARN] Failed to create " << (type_name ? type_name : "unknown") 
                      << " hardware device: " << errbuf << std::endl;
            hw_device_ctx_ = nullptr;
            return false;
        }
        
        codec_ctx_->hw_device_ctx = av_buffer_ref(hw_device_ctx_);
        std::cout << "[INFO] Successfully initialized " << (type_name ? type_name : "unknown") 
                  << " hardware acceleration" << std::endl;
        return true;
    }

    cv::Mat FFmpegVideoReader::avframe_to_mat(const AVFrame *src)
    {
        if (!src || !src->data[0]) {
            std::cerr << "[ERROR] Invalid source frame" << std::endl;
            return cv::Mat();
        }
        
        if (src->width <= 0 || src->height <= 0 || src->width > 16384 || src->height > 16384) {
            std::cerr << "[ERROR] Invalid frame dimensions: " << src->width << "x" << src->height << std::endl;
            return cv::Mat();
        }

        try {
            if (!sws_ctx_ || src->width != width_ || src->height != height_)
            {
                if (sws_ctx_)
                {
                    sws_freeContext(sws_ctx_);
                }
                sws_ctx_ = sws_getContext(src->width, src->height, static_cast<AVPixelFormat>(src->format),
                                          src->width, src->height, AV_PIX_FMT_BGR24,
                                          SWS_BILINEAR, nullptr, nullptr, nullptr);
                if (!sws_ctx_) {
                    std::cerr << "[ERROR] Failed to create sws context" << std::endl;
                    return cv::Mat();
                }
                width_ = src->width;
                height_ = src->height;
            }
            
            cv::Mat dst(height_, width_, CV_8UC3);
            if (dst.empty()) {
                std::cerr << "[ERROR] Failed to create destination Mat" << std::endl;
                return cv::Mat();
            }
            
            uint8_t *dst_data[1] = {dst.data};
            int dst_linesize[1] = {static_cast<int>(dst.step)};
            
            int result = sws_scale(sws_ctx_, src->data, src->linesize, 0, src->height, dst_data, dst_linesize);
            if (result != src->height) {
                std::cerr << "[ERROR] sws_scale failed, expected " << src->height << " lines, got " << result << std::endl;
                return cv::Mat();
            }
            
            return dst;
        } catch (const std::exception& e) {
            std::cerr << "[ERROR] Exception in avframe_to_mat: " << e.what() << std::endl;
            return cv::Mat();
        } catch (...) {
            std::cerr << "[ERROR] Unknown exception in avframe_to_mat" << std::endl;
            return cv::Mat();
        }
    }

    std::string FFmpegVideoReader::get_codec_name() const
    {
        if (!video_stream_)
        {
            return {};
        }
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
        if (width <= 0 || height <= 0 || width > 16384 || height > 16384) {
            std::cerr << "[ERROR] Invalid resolution: " << width << "x" << height << std::endl;
            return false;
        }
        
        if (!src_.starts_with("/dev/video"))
        {
            std::cerr << "[WARN] Resolution change only supported for camera devices" << std::endl;
            return false;
        }
        
        try {
            auto resolutions = list_camera_resolutions();
            if (resolutions.empty())
            {
                std::cerr << "[WARN] Could not enumerate camera resolutions" << std::endl;
                return false;
            }
            for (const auto &res : resolutions)
            {
                if (res.first == width && res.second == height)
                {
                    return open(src_, width, height);
                }
            }
            std::cerr << "[WARN] Resolution " << width << "x" << height << " not supported by camera" << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "[ERROR] Exception in set_resolution: " << e.what() << std::endl;
        } catch (...) {
            std::cerr << "[ERROR] Unknown exception in set_resolution" << std::endl;
        }
        return false;
    }

    std::vector<std::pair<int, int>> FFmpegVideoReader::list_camera_resolutions() const
    {
        std::vector<std::pair<int, int>> resolutions;
        
        if (src_.empty() || !src_.starts_with("/dev/video")) {
            return resolutions;
        }
        
        try {
            int fd = ::open(src_.c_str(), O_RDWR);
            if (fd < 0)
            {
                std::cerr << "[WARN] Could not open " << src_ << " for resolution enumeration" << std::endl;
                return resolutions;
            }

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
                        // Validate dimensions
                        if (frmsize.discrete.width > 0 && frmsize.discrete.height > 0 &&
                            frmsize.discrete.width <= 16384 && frmsize.discrete.height <= 16384) {
                            resolutions.emplace_back(frmsize.discrete.width, frmsize.discrete.height);
                        }
                    }
                    else if (frmsize.type == V4L2_FRMSIZE_TYPE_STEPWISE)
                    {
                        // Limit iterations to prevent infinite loops
                        const unsigned max_iterations = 1000;
                        unsigned iteration_count = 0;
                        
                        for (unsigned h = frmsize.stepwise.min_height;
                             h <= frmsize.stepwise.max_height && iteration_count < max_iterations;
                             h += frmsize.stepwise.step_height, ++iteration_count)
                        {
                            for (unsigned w = frmsize.stepwise.min_width;
                                 w <= frmsize.stepwise.max_width && iteration_count < max_iterations;
                                 w += frmsize.stepwise.step_width, ++iteration_count)
                            {
                                // Validate dimensions
                                if (w > 0 && h > 0 && w <= 16384 && h <= 16384) {
                                    resolutions.emplace_back(w, h);
                                }
                            }
                        }
                    }
                }
            }

            ::close(fd);
        } catch (const std::exception& e) {
            std::cerr << "[ERROR] Exception in list_camera_resolutions: " << e.what() << std::endl;
        } catch (...) {
            std::cerr << "[ERROR] Unknown exception in list_camera_resolutions" << std::endl;
        }
        
        return resolutions;
    }
} // namespace boblib::video