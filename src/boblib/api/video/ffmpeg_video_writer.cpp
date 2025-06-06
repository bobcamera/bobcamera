#include "ffmpeg_video_writer.hpp"

namespace boblib::video
{
    FFmpegVideoWriter::FFmpegVideoWriter(const Options &options)
        : m_options(options)
        , m_isRunning(false)
        , m_framesReceived(0)
        , m_framesWritten(0)
        , m_totalProcessingTime(0.0)
        , m_formatContext(nullptr)
        , m_codecContext(nullptr)
        , m_stream(nullptr)
        , m_swsContext(nullptr)
        , m_frame(nullptr)
        , m_frameBuffer(nullptr)
        , m_ioBufferSize(1024 * 1024 * 4)
    {
        if (m_options.debug)
        {
            av_log_set_level(AV_LOG_INFO);
        }
        else
        {
            av_log_set_level(AV_LOG_QUIET);
        }

        // Initialize FFmpeg if needed
        static std::once_flag initFlag;
        std::call_once(initFlag, []()
                       {
// av_register_all() is deprecated in newer FFmpeg versions
#if LIBAVCODEC_VERSION_INT < AV_VERSION_INT(58, 0, 0)
                           av_register_all();
#endif
                       });
    }

    FFmpegVideoWriter::~FFmpegVideoWriter()
    {
        stop();
        cleanup();
    }

    bool FFmpegVideoWriter::start()
    {
        if (m_isRunning)
        {
            return false;
        }

        // Create output directory if needed
        fs::path outputPath(m_options.outputPath);
        fs::create_directories(outputPath.parent_path());

        // Will initialize the actual encoder when we receive the first frame
        // (if width/height are not specified in options)
        m_isRunning = true;
        m_isInitialized = false;

        // Immediate initialization when both dimensions are known
        if (m_options.width > 0 && m_options.height > 0)
        {
            if (!initialize_fFmpeg(m_options.width, m_options.height))
            {
                std::cerr << "Failed to initialize FFmpeg on start" << std::endl;
                m_isRunning = false;
                return false;
            }
            m_isInitialized = true;
        }

        // Start worker threads
        for (size_t i = 0; i < m_options.numWorkerThreads; ++i)
        {
            m_workerThreads.emplace_back(&FFmpegVideoWriter::processing_thread, this);
        }

        // Start performance monitoring thread if enabled
        if (m_options.logPerformance)
        {
            m_monitorThread = std::thread(&FFmpegVideoWriter::monitor_thread, this);
        }

        m_startTime = std::chrono::steady_clock::now();
        return true;
    }

    // Stop the writer and finalize the video
    void FFmpegVideoWriter::stop()
    {
        if (!m_isRunning)
        {
            return;
        }

        {
            std::unique_lock<std::mutex> lock(m_queueMutex);
            m_isRunning = false;
            m_queueCondition.notify_all();
        }

        // Wait for all worker threads to finish
        for (auto &thread : m_workerThreads)
        {
            if (thread.joinable())
            {
                thread.join();
            }
        }
        m_workerThreads.clear();

        // Wait for monitor thread
        if (m_monitorThread.joinable())
        {
            m_monitorThread.join();
        }

        // Print final stats
        if (m_options.logPerformance)
        {
            auto duration = std::chrono::duration_cast<std::chrono::seconds>(
                                std::chrono::steady_clock::now() - m_startTime)
                                .count();
            std::cout << "FFmpegVideoWriter final stats:" << std::endl;
            std::cout << "  Total frames: " << m_framesReceived << std::endl;
            std::cout << "  Frames written: " << m_framesWritten << std::endl;
            std::cout << "  Average FPS: " << (duration > 0 ? m_framesWritten / duration : 0) << std::endl;
            std::cout << "  Dropped frames: " << (m_framesReceived - m_framesWritten) << std::endl;
            std::cout << "  Average processing time per frame: "
                      << (m_framesWritten > 0 ? m_totalProcessingTime / m_framesWritten : 0) << " ms" << std::endl;
        }

        // Finalize the video
        finalize_video();
    }

    std::string FFmpegVideoWriter::get_codec_name() const noexcept
    {
        return m_codecLongName;
    }

    // Write a frame to the video
    bool FFmpegVideoWriter::write_frame(const cv::Mat &frame)
    {
        if (!m_isRunning)
        {
            return false;
        }

        m_framesReceived++;

        // Make a deep copy of the frame to ensure it persists
        cv::Mat frameCopy;
        frame.copyTo(frameCopy);
        {
            std::unique_lock<std::mutex> lock(m_queueMutex);

            // If buffer is full, drop frame for high-performance
            if (m_frameQueue.size() >= m_options.bufferSize)
            {
                if (m_options.logPerformance)
                {
                    std::cout << "Frame buffer full, dropping frame!" << std::endl;
                }
                return false;
            }

            m_frameQueue.push(std::move(frameCopy));
        }

        // Notify one worker that a new frame is available
        m_queueCondition.notify_one();
        return true;
    }

    FFmpegVideoWriter::Stats FFmpegVideoWriter::get_stats() const
    {
        std::unique_lock<std::mutex> lock(m_queueMutex);
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(
                            std::chrono::steady_clock::now() - m_startTime)
                            .count();
        return 
            {
                m_framesReceived,
                m_framesWritten,
                m_frameQueue.size(),
                (duration > 0 ? static_cast<double>(m_framesWritten) / duration : 0.0),
                m_totalProcessingTime > 0 && m_framesWritten > 0 ? m_totalProcessingTime / m_framesWritten : 0.0
            };
    }

    // pull all the heavy lifting into this helper
    bool FFmpegVideoWriter::init_with_codec(const AVCodec *codec, int width, int height)
    {
        // Create output format context
        if (avformat_alloc_output_context2(&m_formatContext, nullptr, nullptr, m_options.outputPath.c_str()) < 0)
        {
            if (m_options.debug)
            {
                std::cerr << "[" << codec->name << "] Could not create output context" << std::endl;
            }
            return false;
        }

        // create stream & codec context
        m_stream = avformat_new_stream(m_formatContext, nullptr);
        if (!m_stream)
        {
            if (m_options.debug)
            {
                std::cerr << "[" << codec->name << "] Could not allocate stream" << std::endl;
            }
            return false;
        }

        m_codecContext = avcodec_alloc_context3(codec);
        if (!m_codecContext)
        {
            if (m_options.debug)
            {
                std::cerr << "[" << codec->name << "] Could not alloc context" << std::endl;
            }
            return false;
        }
        // If this is a VAAPI encoder, create device & frames context
        if (strstr(codec->name, "vaapi"))
        {
            AVBufferRef *hw_dev_ctx = nullptr;
            if (av_hwdevice_ctx_create(&hw_dev_ctx,
                                       AV_HWDEVICE_TYPE_VAAPI,
                                       nullptr, nullptr, 0) < 0)
            {
                if (m_options.debug)
                {
                    std::cerr << "[" << codec->name << "] Failed to create VAAPI device" << std::endl;
                }
                return false;
            }
            m_codecContext->hw_device_ctx = av_buffer_ref(hw_dev_ctx);

            // Force VAAPI pixel format
            m_codecContext->pix_fmt = AV_PIX_FMT_VAAPI;

            // Allocate HW frames context
            AVBufferRef *hw_frames_ref = av_hwframe_ctx_alloc(hw_dev_ctx);
            if (!hw_frames_ref)
            {
                if (m_options.debug)
                {
                    std::cerr << "[" << codec->name << "] Could not alloc HW frames ctx" << std::endl;
                }
                return false;
            }
            AVHWFramesContext *frames_ctx =
                (AVHWFramesContext *)hw_frames_ref->data;
            frames_ctx->format = AV_PIX_FMT_VAAPI;
            frames_ctx->sw_format = av_get_pix_fmt(m_options.pixelFormat.c_str()) != AV_PIX_FMT_NONE
                                        ? av_get_pix_fmt(m_options.pixelFormat.c_str())
                                        : AV_PIX_FMT_YUV420P;
            frames_ctx->width = width;
            frames_ctx->height = height;
            frames_ctx->initial_pool_size = 32;
            if (av_hwframe_ctx_init(hw_frames_ref) < 0)
            {
                if (m_options.debug)
                {
                    std::cerr << "[" << codec->name << "] Could not init HW frames ctx" << std::endl;
                }
                return false;
            }
            m_codecContext->hw_frames_ctx = av_buffer_ref(hw_frames_ref);
        }

        // set parameters (width/height/fmt/bitrate/threads/gop/crf/preset/extraOptions…)
        m_codecContext->codec_id = codec->id;
        m_codecContext->codec_type = AVMEDIA_TYPE_VIDEO;
        m_codecContext->width = width;
        m_codecContext->height = height;
        m_codecContext->time_base = AVRational{1, int(m_options.fps)};
        m_stream->time_base = m_codecContext->time_base;

        // pixel format
        switch (codec->id)
        {
        case AV_CODEC_ID_H264:
        case AV_CODEC_ID_H265:
            m_codecContext->profile = FF_PROFILE_H264_MAIN;
            break;
        default:
            if (m_options.debug)
            {
                std::cerr << "[" << codec->name << "] unsupported codec id" << std::endl;
            }
            return false;
        }

        if (!codec->pix_fmts)
        {
            // no list → just pick the user request if valid, else YUV420P
            AVPixelFormat pf = av_get_pix_fmt(m_options.pixelFormat.c_str());
            if (pf == AV_PIX_FMT_NONE)
            {
                pf = AV_PIX_FMT_YUV420P;
            }
            m_codecContext->pix_fmt = pf;
        }
        else
        {
            // normal path: pick the user-wanted if supported, else the first supported
            AVPixelFormat wanted = av_get_pix_fmt(m_options.pixelFormat.c_str());
            const enum AVPixelFormat *p = codec->pix_fmts;
            AVPixelFormat chosen = AV_PIX_FMT_NONE;
            for (; *p != AV_PIX_FMT_NONE; ++p)
            {
                if (*p == wanted)
                {
                    chosen = wanted;
                    break;
                }
                if (chosen == AV_PIX_FMT_NONE)
                {
                    chosen = *p;
                }
            }
            if (chosen == AV_PIX_FMT_NONE)
            {
                if (m_options.debug)
                {
                    std::cerr << "[" << codec->name << "] No supported pixel format" << std::endl;
                }
                return false;
            }
            m_codecContext->pix_fmt = chosen;
        }

        if (m_options.bitrate > 0)
        {
            m_codecContext->bit_rate = m_options.bitrate;
        }
        if (m_options.threads > 0)
        {
            m_codecContext->thread_count = m_options.threads;
        }
        if (m_options.gop_size > 0)
        {
            m_codecContext->gop_size = m_options.gop_size;
        }
        if (strstr(codec->name, "264") || strstr(codec->name, "265"))
        {
            av_opt_set(m_codecContext->priv_data, "crf", std::to_string(m_options.quality).c_str(), 0);
        }
        if (!m_options.preset.empty())
        {
            av_opt_set(m_codecContext->priv_data, "preset", m_options.preset.c_str(), 0);
        }
        if (!m_options.extraOptions.empty())
        {
            std::stringstream ss(m_options.extraOptions);
            std::string opt;
            while (std::getline(ss, opt, ':'))
            {
                auto pos = opt.find('=');
                av_opt_set(m_codecContext->priv_data,
                           opt.substr(0, pos).c_str(),
                           opt.substr(pos + 1).c_str(), 0);
            }
        }

        // open codec, copy params, alloc frame/buffer, open file, write header
        if (avcodec_open2(m_codecContext, codec, nullptr) < 0)
        {
            if (m_options.debug)
            {
                std::cerr << "[" << codec->name << "] Could not open codec" << std::endl;
            }
            return false;
        }
        if (avcodec_parameters_from_context(m_stream->codecpar, m_codecContext) < 0)
        {
            if (m_options.debug)
            {
                std::cerr << "[" << codec->name << "] Could not copy params" << std::endl;
            }
            return false;
        }
        m_frame = av_frame_alloc();
        m_frame->format = m_codecContext->pix_fmt;
        m_frame->width = width;
        m_frame->height = height;
        if (av_frame_get_buffer(m_frame, 32) < 0)
        {
            if (m_options.debug)
            {
                std::cerr << "[" << codec->name << "] Could not alloc frame buffer" << std::endl;
            }
            return false;
        }
        if (!(m_formatContext->oformat->flags & AVFMT_NOFILE) &&
            avio_open(&m_formatContext->pb, m_options.outputPath.c_str(), AVIO_FLAG_WRITE) < 0)
        {
            if (m_options.debug)
            {
                std::cerr << "[" << codec->name << "] Could not open " << m_options.outputPath << std::endl;
            }
            return false;
        }
        if (avformat_write_header(m_formatContext, nullptr) < 0)
        {
            if (m_options.debug)
            {
                std::cerr << "[" << codec->name << "] Error writing header" << std::endl;
            }
            return false;
        }

        return true;
    }

    bool FFmpegVideoWriter::initialize_fFmpeg(int width, int height)
    {
        // build HW candidate list
        std::vector<std::string> hwList;
        if (m_options.useHardwareAcceleration)
        {
            if (m_options.codec == Options::CodecType::H264)
            {
                hwList = {"h264_nvenc", "h264_qsv", "h264_amf", "h264_v4l2m2m", "h264_vaapi"};
            }
            else
            {
                hwList = {"hevc_nvenc", "hevc_qsv", "hevc_amf", "hevc_v4l2m2m", "hevc_vaapi"};
            }
        }

        // try hardware paths
        for (auto &name : hwList)
        {
            const AVCodec *c = avcodec_find_encoder_by_name(name.c_str());
            if (!c)
            {
                continue;
            }
            cleanup(); // wipe any partial state
            if (init_with_codec(c, width, height))
            {
                if (m_options.debug)
                {
                    std::cout << "Using hardware accelerated encoder: " << c->long_name << std::endl;
                }
                m_codecLongName = c->long_name;
                return true;
            }
        }

        // fallback to software
        std::string swName = m_options.codec == Options::CodecType::H264 ? "libx264" : "libx265";
        const AVCodec *sw = avcodec_find_encoder_by_name(swName.c_str());
        if (!sw)
        {
            if (m_options.debug)
            {
                std::cerr << "Codec '" << swName << "' not found" << std::endl;
            }
            return false;
        }
        cleanup();
        if (!init_with_codec(sw, width, height))
        {
            if (m_options.debug)
            {
                std::cerr << "Failed to initialize software encoder: " << swName << std::endl;
            }
            return false;
        }

        m_codecLongName = sw->long_name;
        if (m_options.debug)
        {
            std::cout << "Using software encoder: " << sw->long_name << std::endl;
        }

        return true;
    }

    // Convert cv::Mat to FFmpeg frame
    bool FFmpegVideoWriter::convert_frame(const cv::Mat &input, AVFrame *output)
    {
        // Determine source pixel format
        AVPixelFormat srcFormat;
        if (input.channels() == 1)
        {
            srcFormat = AV_PIX_FMT_GRAY8;
        }
        else if (input.channels() == 3)
        {
            srcFormat = AV_PIX_FMT_BGR24;
        }
        else if (input.channels() == 4)
        {
            srcFormat = AV_PIX_FMT_BGRA;
        }
        else
        {
            std::cerr << "Unsupported number of channels: " << input.channels() << std::endl;
            return false;
        }

        // Initialize SwsContext if needed
        if (!m_swsContext)
        {
            m_swsContext = sws_getContext(
                input.cols, input.rows, srcFormat,
                m_codecContext->width, m_codecContext->height, m_codecContext->pix_fmt,
                SWS_BICUBIC, nullptr, nullptr, nullptr);

            if (!m_swsContext)
            {
                std::cerr << "Could not initialize SwsContext" << std::endl;
                return false;
            }
        }

        // Make sure the frame is writable
        if (av_frame_make_writable(output) < 0)
        {
            std::cerr << "Could not make frame writable" << std::endl;
            return false;
        }

        // Convert the frame
        const int srcStride[4] = {static_cast<int>(input.step[0]), 0, 0, 0};
        uint8_t *srcData[4] = {input.data, nullptr, nullptr, nullptr};

        sws_scale(m_swsContext, srcData, srcStride, 0, input.rows,
                  output->data, output->linesize);

        return true;
    }

    // Finalize the video
    void FFmpegVideoWriter::finalize_video()
    {
        if (!m_isInitialized)
        {
            return;
        }

        // Write trailer
        if (m_formatContext)
        {
            av_write_trailer(m_formatContext);
        }

        cleanup();
    }

    // Cleanup all FFmpeg resources
    void FFmpegVideoWriter::cleanup()
    {
        // Clean up SwsContext
        if (m_swsContext)
        {
            sws_freeContext(m_swsContext);
            m_swsContext = nullptr;
        }

        // Free the frame
        if (m_frame)
        {
            av_frame_free(&m_frame);
        }

        // Close the codec
        if (m_codecContext)
        {
            avcodec_free_context(&m_codecContext);
        }

        // Close the output file
        if (m_formatContext && !(m_formatContext->oformat->flags & AVFMT_NOFILE))
        {
            avio_closep(&m_formatContext->pb);
        }

        // Free the format context
        if (m_formatContext)
        {
            avformat_free_context(m_formatContext);
            m_formatContext = nullptr;
        }

        m_isInitialized = false;
    }

    // Worker thread for processing frames
    void FFmpegVideoWriter::processing_thread()
    {
        while (m_isRunning)
        {
            cv::Mat frame;

            // Get a frame from the queue
            {
                std::unique_lock<std::mutex> lock(m_queueMutex);
                m_queueCondition.wait(lock, [this]
                                      { return !m_isRunning || !m_frameQueue.empty(); });

                if (!m_isRunning && m_frameQueue.empty())
                {
                    break;
                }

                if (!m_frameQueue.empty())
                {
                    frame = std::move(m_frameQueue.front());
                    m_frameQueue.pop();
                }
            }

            if (!frame.empty())
            {
                auto startTime = std::chrono::high_resolution_clock::now();

                // Initialize FFmpeg if this is the first frame
                if (!m_isInitialized)
                {
                    int width = m_options.width > 0 ? m_options.width : frame.cols;
                    int height = m_options.height > 0 ? m_options.height : frame.rows;

                    if (!initialize_fFmpeg(width, height))
                    {
                        std::cerr << "Failed to initialize FFmpeg" << std::endl;
                        m_isRunning = false;
                        break;
                    }

                    m_isInitialized = true;
                }

                // Convert the frame
                if (!convert_frame(frame, m_frame))
                {
                    std::cerr << "Failed to convert frame" << std::endl;
                    continue;
                }

                // Set the frame's timestamp
                m_frame->pts = m_framesWritten;

                // Encode the frame
                int ret = avcodec_send_frame(m_codecContext, m_frame);
                if (ret < 0)
                {
                    char errBuf[AV_ERROR_MAX_STRING_SIZE];
                    av_strerror(ret, errBuf, AV_ERROR_MAX_STRING_SIZE);
                    std::cerr << "Error sending frame for encoding: " << errBuf << std::endl;
                    continue;
                }

                AVPacket pkt = {};
                while (ret >= 0)
                {
                    ret = avcodec_receive_packet(m_codecContext, &pkt);

                    if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
                    {
                        break;
                    }
                    else if (ret < 0)
                    {
                        char errBuf[AV_ERROR_MAX_STRING_SIZE];
                        av_strerror(ret, errBuf, AV_ERROR_MAX_STRING_SIZE);
                        std::cerr << "Error during encoding: " << errBuf << std::endl;
                        break;
                    }

                    // Rescale timestamp
                    av_packet_rescale_ts(&pkt, m_codecContext->time_base, m_stream->time_base);
                    pkt.stream_index = m_stream->index;

                    // Write the compressed frame to the media file
                    ret = av_interleaved_write_frame(m_formatContext, &pkt);
                    if (ret < 0)
                    {
                        char errBuf[AV_ERROR_MAX_STRING_SIZE];
                        av_strerror(ret, errBuf, AV_ERROR_MAX_STRING_SIZE);
                        std::cerr << "Error while writing frame: " << errBuf << std::endl;
                    }
                }

                av_packet_unref(&pkt);

                m_framesWritten++;

                auto endTime = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
                                    endTime - startTime)
                                    .count() /
                                1000.0;

                m_totalProcessingTime += duration;
            }
        }

        // Process any remaining frames in the queue
        std::unique_lock<std::mutex> lock(m_queueMutex);
        while (!m_frameQueue.empty())
        {
            cv::Mat frame = std::move(m_frameQueue.front());
            m_frameQueue.pop();
            lock.unlock();

            if (!frame.empty() && m_isInitialized)
            {
                // Convert the frame
                if (convert_frame(frame, m_frame))
                {
                    // Set the frame's timestamp
                    m_frame->pts = m_framesWritten;

                    // Encode the frame
                    int ret = avcodec_send_frame(m_codecContext, m_frame);
                    if (ret >= 0)
                    {
                        AVPacket pkt = {};
                        while (ret >= 0)
                        {
                            ret = avcodec_receive_packet(m_codecContext, &pkt);

                            if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
                            {
                                break;
                            }
                            else if (ret >= 0)
                            {
                                // Rescale timestamp
                                av_packet_rescale_ts(&pkt, m_codecContext->time_base, m_stream->time_base);
                                pkt.stream_index = m_stream->index;

                                // Write the compressed frame to the media file
                                av_interleaved_write_frame(m_formatContext, &pkt);
                            }
                        }

                        av_packet_unref(&pkt);
                        m_framesWritten++;
                    }
                }
            }

            lock.lock();
        }

        // Flush encoder
        if (m_isInitialized && m_codecContext)
        {
            int ret = avcodec_send_frame(m_codecContext, nullptr);
            if (ret >= 0)
            {
                AVPacket pkt = {};

                while (ret >= 0)
                {
                    ret = avcodec_receive_packet(m_codecContext, &pkt);

                    if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
                    {
                        break;
                    }
                    else if (ret >= 0)
                    {
                        // Rescale timestamp
                        av_packet_rescale_ts(&pkt, m_codecContext->time_base, m_stream->time_base);
                        pkt.stream_index = m_stream->index;

                        // Write the compressed frame to the media file
                        av_interleaved_write_frame(m_formatContext, &pkt);
                    }
                }

                av_packet_unref(&pkt);
            }
        }
    }

    // Performance monitoring thread
    void FFmpegVideoWriter::monitor_thread()
    {
        while (m_isRunning)
        {
            std::this_thread::sleep_for(std::chrono::seconds(5));

            auto stats = get_stats();
            std::cout << "FFmpegVideoWriter stats:" << std::endl;
            std::cout << "  Queue size: " << stats.queueSize << "/" << m_options.bufferSize << std::endl;
            std::cout << "  Current FPS: " << stats.fps << std::endl;
            std::cout << "  Avg processing time: " << stats.avgProcessingTimeMs << "ms" << std::endl;
            std::cout << "  Frames received: " << stats.framesReceived << std::endl;
            std::cout << "  Frames written: " << stats.framesWritten << std::endl;
        }
    }
}