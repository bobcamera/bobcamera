#include "ffmpeg_video_writer.hpp"

namespace boblib::video
{
    FFmpegVideoWriter::FFmpegVideoWriter(const Options &options)
        : m_options(options)
        , m_totalProcessingTime(0.0)
        , m_ioBufferSize(1024 * 1024 * 4)
    {
        // Validate options early
        if (m_options.outputPath.empty()) {
            std::cerr << "[ERROR] Empty output path provided" << std::endl;
            return;
        }
        
        if (m_options.fps <= 0.0 || m_options.fps > 1000.0) {
            std::cerr << "[ERROR] Invalid FPS: " << m_options.fps << std::endl;
            return;
        }
        
        if (m_options.width < 0 || m_options.height < 0 ||
            m_options.width > 16384 || m_options.height > 16384) {
            std::cerr << "[ERROR] Invalid dimensions: " << m_options.width << "x" << m_options.height << std::endl;
            return;
        }

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
                       
        m_isValid.store(true);
    }

    FFmpegVideoWriter::~FFmpegVideoWriter()
    {
        stop();
        cleanup();
    }

    bool FFmpegVideoWriter::start()
    {
        if (!m_isValid.load()) {
            std::cerr << "[ERROR] FFmpegVideoWriter not in valid state" << std::endl;
            return false;
        }
        
        if (m_isRunning.load())
        {
            if (m_options.debug) {
                std::cerr << "[WARNING] FFmpegVideoWriter already running" << std::endl;
            }
            return false;
        }

        // Create output directory if needed
        try {
            fs::path outputPath(m_options.outputPath);
            fs::create_directories(outputPath.parent_path());
        } catch (const std::exception& e) {
            std::cerr << "[ERROR] Failed to create output directory: " << e.what() << std::endl;
            return false;
        }

        // Will initialize the actual encoder when we receive the first frame
        // (if width/height are not specified in options)
        m_isRunning.store(true);
        m_isInitialized.store(false);

        // Immediate initialization when both dimensions are known
        if (m_options.width > 0 && m_options.height > 0)
        {
            if (!initialize_fFmpeg(m_options.width, m_options.height))
            {
                std::cerr << "Failed to initialize FFmpeg on start" << std::endl;
                m_isRunning.store(false);
                m_isValid.store(false);
                return false;
            }
            m_isInitialized.store(true);
        }

        try {
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
        } catch (const std::exception& e) {
            std::cerr << "[ERROR] Failed to start worker threads: " << e.what() << std::endl;
            m_isRunning.store(false);
            m_isValid.store(false);
            return false;
        }

        m_startTime = std::chrono::steady_clock::now();
        return true;
    }

    // Stop the writer and finalize the video
    void FFmpegVideoWriter::stop()
    {
        if (!m_isRunning.load())
        {
            return;
        }

        {
            std::unique_lock<std::mutex> lock(m_queueMutex);
            m_isRunning.store(false);
            m_queueCondition.notify_all();
        }

        // Wait for all worker threads to finish
        try {
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
        } catch (const std::exception& e) {
            std::cerr << "[ERROR] Exception while stopping threads: " << e.what() << std::endl;
        }

        // Print final stats
        if (m_options.logPerformance)
        {
            auto duration = std::chrono::duration_cast<std::chrono::seconds>(
                                std::chrono::steady_clock::now() - m_startTime)
                                .count();
            std::cout << "FFmpegVideoWriter final stats:" << std::endl;
            std::cout << "  Total frames: " << m_framesReceived.load() << std::endl;
            std::cout << "  Frames written: " << m_framesWritten.load() << std::endl;
            std::cout << "  Average FPS: " << (duration > 0 ? m_framesWritten.load() / duration : 0) << std::endl;
            std::cout << "  Dropped frames: " << (m_framesReceived.load() - m_framesWritten.load()) << std::endl;
            std::cout << "  Average processing time per frame: "
                      << (m_framesWritten.load() > 0 ? m_totalProcessingTime.load() / m_framesWritten.load() : 0) << " ms" << std::endl;
        }

        // Finalize the video
        try {
            finalize_video();
        } catch (const std::exception& e) {
            std::cerr << "[ERROR] Exception during finalization: " << e.what() << std::endl;
        }
    }

    std::string FFmpegVideoWriter::get_codec_name() const noexcept
    {
        return m_codecLongName;
    }

    // Write a frame to the video
    bool FFmpegVideoWriter::write_frame(const cv::Mat &frame)
    {
        if (!m_isValid.load() || !m_isRunning.load())
        {
            if (m_options.debug) {
                std::cerr << "[ERROR] Writer not in valid or running state" << std::endl;
            }
            return false;
        }
        
        if (frame.empty()) {
            if (m_options.debug) {
                std::cerr << "[ERROR] Empty frame provided" << std::endl;
            }
            return false;
        }

        m_framesReceived.fetch_add(1);

        // Make a deep copy of the frame to ensure it persists
        cv::Mat frameCopy;
        try {
            frame.copyTo(frameCopy);
        } catch (const std::exception& e) {
            std::cerr << "[ERROR] Failed to copy frame: " << e.what() << std::endl;
            return false;
        }
        
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
        return Stats{
            m_framesReceived.load(),
            m_framesWritten.load(),
            m_frameQueue.size(),
            (duration > 0 ? static_cast<double>(m_framesWritten.load()) / duration : 0.0),
            m_totalProcessingTime.load() > 0 && m_framesWritten.load() > 0 ? 
                m_totalProcessingTime.load() / m_framesWritten.load() : 0.0,
            m_isValid.load()
        };
    }
    
    bool FFmpegVideoWriter::is_valid() const noexcept
    {
        return m_isValid.load();
    }

    bool FFmpegVideoWriter::init_with_codec(const AVCodec *codec, int width, int height)
    {
        if (!codec || width <= 0 || height <= 0) {
            if (m_options.debug) {
                std::cerr << "[ERROR] Invalid parameters for codec initialization" << std::endl;
            }
            return false;
        }
        
        std::lock_guard<std::mutex> lock(m_ffmpegMutex);
        
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
            av_buffer_unref(&hw_dev_ctx); // Release our reference

            // Force VAAPI pixel format
            m_codecContext->pix_fmt = AV_PIX_FMT_VAAPI;

            // Allocate HW frames context
            AVBufferRef *hw_frames_ref = av_hwframe_ctx_alloc(m_codecContext->hw_device_ctx);
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
                av_buffer_unref(&hw_frames_ref);
                return false;
            }
            m_codecContext->hw_frames_ctx = av_buffer_ref(hw_frames_ref);
            av_buffer_unref(&hw_frames_ref); // Release our reference
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
        if (width <= 0 || height <= 0) {
            if (m_options.debug) {
                std::cerr << "[ERROR] Invalid dimensions for FFmpeg initialization: " 
                          << width << "x" << height << std::endl;
            }
            return false;
        }
        
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
            m_isValid.store(false);
            return false;
        }
        cleanup();
        if (!init_with_codec(sw, width, height))
        {
            if (m_options.debug)
            {
                std::cerr << "Failed to initialize software encoder: " << swName << std::endl;
            }
            m_isValid.store(false);
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
        if (input.empty() || !output) {
            if (m_options.debug) {
                std::cerr << "[ERROR] Invalid input for frame conversion" << std::endl;
            }
            return false;
        }
        
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

        // Initialize SwsContext if needed (not thread-safe, but only called from worker thread)
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

        int result = sws_scale(m_swsContext, srcData, srcStride, 0, input.rows,
                              output->data, output->linesize);
        
        if (result != m_codecContext->height) {
            std::cerr << "SwsScale failed: expected " << m_codecContext->height 
                      << " lines, got " << result << std::endl;
            return false;
        }

        return true;
    }

    // Finalize the video
    void FFmpegVideoWriter::finalize_video()
    {
        std::lock_guard<std::mutex> lock(m_ffmpegMutex);
        
        if (!m_isInitialized.load())
        {
            return;
        }

        try {
            // Write trailer
            if (m_formatContext)
            {
                int ret = av_write_trailer(m_formatContext);
                if (ret < 0 && m_options.debug) {
                    char errBuf[AV_ERROR_MAX_STRING_SIZE];
                    av_strerror(ret, errBuf, AV_ERROR_MAX_STRING_SIZE);
                    std::cerr << "[WARNING] Error writing trailer: " << errBuf << std::endl;
                }
            }
        } catch (const std::exception& e) {
            std::cerr << "[ERROR] Exception writing trailer: " << e.what() << std::endl;
        }

        cleanup();
    }
    
    bool FFmpegVideoWriter::validate_state() const noexcept
    {
        return m_isValid.load() && m_isRunning.load();
    }

    // Cleanup all FFmpeg resources
    void FFmpegVideoWriter::cleanup()
    {
        std::lock_guard<std::mutex> lock(m_ffmpegMutex);
        
        try {
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
                m_frame = nullptr;
            }

            // Close the codec
            if (m_codecContext)
            {
                avcodec_free_context(&m_codecContext);
                m_codecContext = nullptr;
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

            m_stream = nullptr; // This is owned by format context, so just null it
            m_isInitialized.store(false);
        } catch (const std::exception& e) {
            std::cerr << "[ERROR] Exception during FFmpeg cleanup: " << e.what() << std::endl;
        } catch (...) {
            std::cerr << "[ERROR] Unknown exception during FFmpeg cleanup" << std::endl;
        }
    }

    // Worker thread for processing frames
    void FFmpegVideoWriter::processing_thread()
    {
        while (validate_state())
        {
            cv::Mat frame;

            // Get a frame from the queue
            {
                std::unique_lock<std::mutex> lock(m_queueMutex);
                m_queueCondition.wait(lock, [this]
                                      { return !m_isRunning.load() || !m_frameQueue.empty(); });

                if (!m_isRunning.load() && m_frameQueue.empty())
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

                try {
                    // Initialize FFmpeg if this is the first frame
                    if (!m_isInitialized.load())
                    {
                        int width = m_options.width > 0 ? m_options.width : frame.cols;
                        int height = m_options.height > 0 ? m_options.height : frame.rows;

                        if (!initialize_fFmpeg(width, height))
                        {
                            std::cerr << "Failed to initialize FFmpeg" << std::endl;
                            m_isRunning.store(false);
                            m_isValid.store(false);
                            break;
                        }

                        m_isInitialized.store(true);
                    }

                    // Validate codec context before use
                    if (!m_codecContext || !m_frame) {
                        std::cerr << "[ERROR] Invalid codec context or frame" << std::endl;
                        m_isValid.store(false);
                        break;
                    }

                    // Convert the frame
                    if (!convert_frame(frame, m_frame))
                    {
                        std::cerr << "Failed to convert frame" << std::endl;
                        continue;
                    }

                    // Set the frame's timestamp
                    m_frame->pts = m_framesWritten.load();

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

                    m_framesWritten.fetch_add(1);

                    auto endTime = std::chrono::high_resolution_clock::now();
                    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
                                        endTime - startTime)
                                        .count() /
                                    1000.0;

                    m_totalProcessingTime.store(m_totalProcessingTime.load() + duration);
                } catch (const std::exception& e) {
                    std::cerr << "[ERROR] Exception in processing thread: " << e.what() << std::endl;
                    m_isValid.store(false);
                    break;
                } catch (...) {
                    std::cerr << "[ERROR] Unknown exception in processing thread" << std::endl;
                    m_isValid.store(false);
                    break;
                }
            }
        }

        // Process any remaining frames in the queue
        std::unique_lock<std::mutex> lock(m_queueMutex);
        while (!m_frameQueue.empty())
        {
            cv::Mat frame = std::move(m_frameQueue.front());
            m_frameQueue.pop();
            lock.unlock();

            if (!frame.empty() && m_isInitialized.load())
            {
                try {
                    // Convert the frame
                    if (convert_frame(frame, m_frame))
                    {
                        // Set the frame's timestamp
                        m_frame->pts = m_framesWritten.load();

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
                            m_framesWritten.fetch_add(1);
                        }
                    }
                } catch (const std::exception& e) {
                    std::cerr << "[ERROR] Exception processing remaining frame: " << e.what() << std::endl;
                } catch (...) {
                    std::cerr << "[ERROR] Unknown exception processing remaining frame" << std::endl;
                }
            }

            lock.lock();
        }

        // Flush encoder
        if (m_isInitialized.load() && m_codecContext)
        {
            try {
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
            } catch (const std::exception& e) {
                std::cerr << "[ERROR] Exception flushing encoder: " << e.what() << std::endl;
            } catch (...) {
                std::cerr << "[ERROR] Unknown exception flushing encoder" << std::endl;
            }
        }
    }

    // Performance monitoring thread
    void FFmpegVideoWriter::monitor_thread()
    {
        while (m_isRunning.load())
        {
            std::this_thread::sleep_for(std::chrono::seconds(5));

            if (!m_isRunning.load()) break;

            try {
                auto stats = get_stats();
                std::cout << "FFmpegVideoWriter stats:" << std::endl;
                std::cout << "  Queue size: " << stats.queueSize << "/" << m_options.bufferSize << std::endl;
                std::cout << "  Current FPS: " << stats.fps << std::endl;
                std::cout << "  Avg processing time: " << stats.avgProcessingTimeMs << "ms" << std::endl;
                std::cout << "  Frames received: " << stats.framesReceived << std::endl;
                std::cout << "  Frames written: " << stats.framesWritten << std::endl;
                std::cout << "  Valid state: " << (stats.isValid ? "true" : "false") << std::endl;
            } catch (const std::exception& e) {
                std::cerr << "[ERROR] Exception in monitor thread: " << e.what() << std::endl;
            } catch (...) {
                std::cerr << "[ERROR] Unknown exception in monitor thread" << std::endl;
            }
        }
    }
}