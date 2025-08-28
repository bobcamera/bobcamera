#include "VideoWriter.hpp"
#include "../base/Utils.hpp"

using namespace boblib::video;

VideoWriter::VideoWriter(const std::string &fileName, const cv::Size &frame_size, boblib::video::Codec codec, double fps, bool use_opencv, bool use_cuda) noexcept
    : use_opencv_(use_opencv)
    , using_cuda_(use_cuda ? boblib::base::Utils::has_cuda() : false)
    , fileName_(fileName)  // Store as copy
    , codec_(codec)
    , fps_(fps)
    , frame_size_(frame_size)
{
    if (fileName.empty()) {
        std::cerr << "[ERROR] Empty filename provided" << std::endl;
        return;
    }
    if (frame_size.width <= 0 || frame_size.height <= 0 || 
        frame_size.width > 16384 || frame_size.height > 16384) {
        std::cerr << "[ERROR] Invalid frame size: " << frame_size.width << "x" << frame_size.height << std::endl;
        return;
    }
    if (fps <= 0.0 || fps > 1000.0) {
        std::cerr << "[ERROR] Invalid FPS: " << fps << std::endl;
        return;
    }
    
    try {
        create_video_writer();
        if (is_open()) {
            is_valid_.store(true);
            is_initialized_.store(true);
        }
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] VideoWriter constructor failed: " << e.what() << std::endl;
        safe_cleanup();
    } catch (...) {
        std::cerr << "[ERROR] VideoWriter constructor failed with unknown exception" << std::endl;
        safe_cleanup();
    }
}

VideoWriter::~VideoWriter() noexcept
{
    safe_cleanup();
}

void VideoWriter::write(const cv::Mat &image) noexcept
{
    if (!is_valid_.load() || image.empty()) {
        if (image.empty()) {
            std::cerr << "[ERROR] Empty image provided to write" << std::endl;
        } else {
            std::cerr << "[ERROR] VideoWriter is not in valid state" << std::endl;
        }
        return;
    }
    
    std::lock_guard<std::mutex> lock(writer_mutex_);
    
    if (!is_initialized_.load()) {
        std::cerr << "[ERROR] VideoWriter not properly initialized" << std::endl;
        return;
    }
    
    try {
        if (!use_opencv_)
        {
            if (!ffmpeg_video_writer_ptr_) {
                std::cerr << "[ERROR] FFmpeg video writer not initialized" << std::endl;
                is_valid_.store(false);
                return;
            }
            if (!ffmpeg_video_writer_ptr_->write_frame(image)) {
                std::cerr << "[ERROR] Failed to write frame with FFmpeg" << std::endl;
            }
            return;
        }
#ifdef HAVE_CUDA
        if (using_cuda_)
        {
            if (!cuda_video_writer_ptr_) {
                std::cerr << "[ERROR] CUDA video writer not initialized" << std::endl;
                is_valid_.store(false);
                return;
            }
            cv::cuda::GpuMat gpu_frame;
            gpu_frame.upload(image);
            cuda_video_writer_ptr_->write(gpu_frame);
        }
        else
#endif
        {
            if (!video_writer_ptr_ || !video_writer_ptr_->isOpened()) {
                std::cerr << "[ERROR] OpenCV video writer not initialized or not opened" << std::endl;
                is_valid_.store(false);
                return;
            }
            video_writer_ptr_->write(image);
        }
    } catch (const cv::Exception& e) {
        std::cerr << "[ERROR] OpenCV exception in write: " << e.what() << std::endl;
        is_valid_.store(false);
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Exception in write: " << e.what() << std::endl;
        is_valid_.store(false);
    } catch (...) {
        std::cerr << "[ERROR] Unknown exception in write" << std::endl;
        is_valid_.store(false);
    }
}

void VideoWriter::write(const boblib::base::Image &image) noexcept
{
    if (!is_valid_.load()) {
        std::cerr << "[ERROR] VideoWriter is not in valid state" << std::endl;
        return;
    }
    
    try {
        cv::Mat mat = image.toMat();
        if (mat.empty()) {
            std::cerr << "[ERROR] Failed to convert Image to Mat" << std::endl;
            return;
        }
        
        write(mat);  // Use the cv::Mat write method which has all the safety checks
        
    } catch (const cv::Exception& e) {
        std::cerr << "[ERROR] OpenCV exception in write(Image): " << e.what() << std::endl;
        is_valid_.store(false);
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Exception in write(Image): " << e.what() << std::endl;
        is_valid_.store(false);
    } catch (...) {
        std::cerr << "[ERROR] Unknown exception in write(Image)" << std::endl;
        is_valid_.store(false);
    }
}

void VideoWriter::release() noexcept
{
    safe_cleanup();
}

void VideoWriter::safe_cleanup() noexcept
{
    std::lock_guard<std::mutex> lock(writer_mutex_);
    
    is_valid_.store(false);
    is_initialized_.store(false);
    
    try {
        if (!use_opencv_)
        {
            if (ffmpeg_video_writer_ptr_)
            {
                ffmpeg_video_writer_ptr_->stop();
                ffmpeg_video_writer_ptr_.reset();
            }
        }
        else
        {
#ifdef HAVE_CUDA
            if (using_cuda_ && cuda_video_writer_ptr_)
            {
                cuda_video_writer_ptr_->release();
                cuda_video_writer_ptr_.reset();
            }
            else
#endif
            if (video_writer_ptr_)
            {
                video_writer_ptr_->release();
                video_writer_ptr_.reset();
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Exception during cleanup: " << e.what() << std::endl;
    } catch (...) {
        std::cerr << "[ERROR] Unknown exception during cleanup" << std::endl;
    }
}

bool VideoWriter::is_open() const noexcept
{
    std::lock_guard<std::mutex> lock(writer_mutex_);
    
    if (!use_opencv_) {
        return ffmpeg_video_writer_ptr_ != nullptr;
    }
    
#ifdef HAVE_CUDA
    if (using_cuda_) {
        return cuda_video_writer_ptr_ != nullptr;
    }
#endif
    
    return video_writer_ptr_ && video_writer_ptr_->isOpened();
}

bool VideoWriter::using_cuda() const noexcept
{
    return use_opencv_ && using_cuda_;
}

bool VideoWriter::is_valid() const noexcept
{
    return is_valid_.load() && is_initialized_.load();
}

inline void VideoWriter::create_video_writer() noexcept
{
    if (!use_opencv_)
    {
        try {
            auto options = FFmpegVideoWriter::Options();
            options.outputPath = fileName_;
            options.codec = codec_ == boblib::video::Codec::HEVC ? FFmpegVideoWriter::Options::CodecType::HEVC
                                                                 : FFmpegVideoWriter::Options::CodecType::H264; // Default to H264 if AVC1 is used
            options.width = frame_size_.width;
            options.height = frame_size_.height;
            options.fps = fps_;
            options.useHardwareAcceleration = true;
            options.debug = true; // Enable debug for better error reporting
            
            ffmpeg_video_writer_ptr_ = std::make_unique<FFmpegVideoWriter>(options);
            if (!ffmpeg_video_writer_ptr_->start()) {
                std::cerr << "[ERROR] Failed to start FFmpeg video writer" << std::endl;
                ffmpeg_video_writer_ptr_.reset();
                return;
            }
        } catch (const std::exception& e) {
            std::cerr << "[ERROR] Exception creating FFmpeg writer: " << e.what() << std::endl;
            ffmpeg_video_writer_ptr_.reset();
        }
        return;
    }

#ifdef HAVE_CUDA
    if (using_cuda_)
    {
        std::cout << "Using cuda" << std::endl;
        std::function<cv::cudacodec::Codec(boblib::video::Codec)> cuda_codec = [](boblib::video::Codec codec)
        {
            switch (codec)
            {
            case boblib::video::Codec::H264:
                return cv::cudacodec::Codec::H264;
            case boblib::video::Codec::HEVC:
                return cv::cudacodec::Codec::HEVC;
            default:
                return cv::cudacodec::Codec::H264;
            }
        };

        try
        {
            cuda_video_writer_ptr_ = cv::cudacodec::createVideoWriter(fileName_, frame_size_, cuda_codec(codec_), fps_);
            if (!cuda_video_writer_ptr_) {
                std::cerr << "[ERROR] Failed to create CUDA video writer" << std::endl;
                return;
            }
            std::cout << "cuda_video_writer_ptr_: " << cuda_video_writer_ptr_ << std::endl;
        }
        catch(const std::exception& e)
        {
            std::cerr << "[ERROR] CUDA video writer creation failed: " << e.what() << std::endl;
            cuda_video_writer_ptr_.reset();
        }
        
        return;
    }
#endif
    
    std::function<int(boblib::video::Codec)> fourcc = [](boblib::video::Codec codec)
    {
        switch (codec)
        {
        case boblib::video::Codec::H264:
            return cv::VideoWriter::fourcc('H', '2', '6', '4');
        case boblib::video::Codec::HEVC:
            return cv::VideoWriter::fourcc('H', 'E', 'V', 'C');
        case boblib::video::Codec::AVC1:
            return cv::VideoWriter::fourcc('a', 'v', 'c', '1');
        default:
            return cv::VideoWriter::fourcc('a', 'v', 'c', '1');
        }
    };

    try {
        video_writer_ptr_ = std::make_unique<cv::VideoWriter>(fileName_, cv::CAP_FFMPEG, fourcc(codec_), fps_, frame_size_);
        if (!video_writer_ptr_->isOpened()) {
            std::cerr << "[ERROR] Failed to open OpenCV video writer for file: " << fileName_ << std::endl;
            video_writer_ptr_.reset();
        }
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Exception creating OpenCV writer: " << e.what() << std::endl;
        video_writer_ptr_.reset();
    }
}

std::string VideoWriter::get_backend_name() const noexcept
{
    std::lock_guard<std::mutex> lock(writer_mutex_);
    
    try {
        if (!use_opencv_)
        {
            return ffmpeg_video_writer_ptr_ ? ffmpeg_video_writer_ptr_->get_codec_name() : "FFmpeg (not initialized)";
        }
        return video_writer_ptr_ ? video_writer_ptr_->getBackendName() : "OpenCV (not initialized)";
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Exception getting backend name: " << e.what() << std::endl;
        return "Unknown (error)";
    } catch (...) {
        std::cerr << "[ERROR] Unknown exception getting backend name" << std::endl;
        return "Unknown (error)";
    }
}

[[nodiscard]] boblib::video::Codec VideoWriter::codec_from_string(std::string_view codec_str) noexcept
{
    using namespace std::literals;

    // Case-insensitive comparison
    if (codec_str == "H264"sv || codec_str == "h264"sv)
    {
        return Codec::H264;
    }
    else if (codec_str == "HEVC"sv || codec_str == "hevc"sv)
    {
        return Codec::HEVC;
    }
    else if (codec_str == "AVC1"sv || codec_str == "avc1"sv)
    {
        return Codec::AVC1;
    }

    return Codec::AVC1;
}