#include "VideoReader.hpp"

using namespace boblib::video;

VideoReader::VideoReader(int usb_camera_id, bool use_opencv, const std::vector<int> & params)
    : use_opencv_(use_opencv)
    , using_cuda_(false)
    , is_usb_(true)
    , usb_camera_id_(usb_camera_id)
    , params_(params)
{
    if (usb_camera_id < 0) {
        std::cerr << "[ERROR] Invalid USB camera ID: " << usb_camera_id << std::endl;
        return;
    }
    
    try {
        create_video_capture();
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] VideoReader constructor failed: " << e.what() << std::endl;
    } catch (...) {
        std::cerr << "[ERROR] VideoReader constructor failed with unknown exception" << std::endl;
    }
}

VideoReader::VideoReader(const std::string & camera_uri, bool use_opencv, bool use_cuda, const std::vector<int> & params)
    : use_opencv_(use_opencv)
    , using_cuda_(use_cuda ? boblib::base::Utils::has_cuda() : false)
    , is_usb_(false)
    , camera_uri_(camera_uri)
    , params_(params)
{
    if (camera_uri.empty()) {
        std::cerr << "[ERROR] Empty camera URI provided" << std::endl;
        return;
    }
    
    try {
        create_video_capture();
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] VideoReader constructor failed: " << e.what() << std::endl;
    } catch (...) {
        std::cerr << "[ERROR] VideoReader constructor failed with unknown exception" << std::endl;
    }
}

VideoReader::~VideoReader() noexcept
{
    release();
}

void VideoReader::release() noexcept
{
    try {
#ifdef HAVE_CUDA
        if (using_cuda_)
        {
            if (cuda_video_reader_ptr_) {
                cuda_video_reader_ptr_.reset();
            }
        }
        else
#endif
        {
            if (ffmpeg_video_reader_ptr_)
            {
                ffmpeg_video_reader_ptr_->close();
                ffmpeg_video_reader_ptr_.reset();
            }
            if (video_capture_ptr_) {
                video_capture_ptr_->release();
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Exception in release(): " << e.what() << std::endl;
    } catch (...) {
        std::cerr << "[ERROR] Unknown exception in release()" << std::endl;
    }
}

bool VideoReader::read(boblib::base::Image &image) noexcept
{
    try
    {
#ifdef HAVE_CUDA
        if (using_cuda_)
        {
            if (!cuda_video_reader_ptr_->nextFrame(image.toCudaMat())) 
            {
                return false;
            }
            if (!image.get_using_cuda())
            {
                image.download();
            }
            return true;
        }
#endif
        if (!use_opencv_)
        {
            return ffmpeg_video_reader_ptr_->read(image.toMat());
        }
        return video_capture_ptr_->read(image.toMat());
    }
    catch (const std::exception & e)
    {
        std::cerr << e.what() << std::endl;
    }
    return false;
}

bool VideoReader::using_cuda() const noexcept
{
    return using_cuda_;
}

bool VideoReader::is_open() const noexcept
{
    if (!use_opencv_)
    {
        return ffmpeg_video_reader_ptr_ ? ffmpeg_video_reader_ptr_->is_opened() : false;
    }
    return using_cuda_ ? true : video_capture_ptr_->isOpened();
}

inline void VideoReader::create_video_capture() noexcept
{
    static const std::vector<int> default_params_normal = {cv::CAP_PROP_HW_ACCELERATION, cv::VIDEO_ACCELERATION_ANY};
    static const std::vector<int> default_params_cuda = {cv::CAP_PROP_OPEN_TIMEOUT_MSEC, 10000, cv::CAP_PROP_CONVERT_RGB, 1};

    try {
        if (!use_opencv_)
        {
            ffmpeg_video_reader_ptr_ = std::make_unique<FFmpegVideoReader>();
            if (!ffmpeg_video_reader_ptr_) {
                std::cerr << "[ERROR] Failed to create FFmpegVideoReader" << std::endl;
                return;
            }
            
            bool is_open = false;
            if (is_usb_) {
                is_open = ffmpeg_video_reader_ptr_->open(usb_camera_id_);
            } else {
                is_open = ffmpeg_video_reader_ptr_->open(camera_uri_);
            }
            
            if (!is_open)
            {
                std::cerr << "[ERROR] Failed to open FFmpeg video reader" << std::endl;
                ffmpeg_video_reader_ptr_.reset();
            }
            return;
        }

        if (is_usb_)
        {
            try {
                video_capture_ptr_ = std::make_unique<cv::VideoCapture>(
                    usb_camera_id_, cv::CAP_ANY, 
                    params_.empty() ? default_params_normal : params_);
                
                if (!video_capture_ptr_->isOpened()) {
                    std::cerr << "[ERROR] Failed to open USB camera " << usb_camera_id_ << std::endl;
                    video_capture_ptr_.reset();
                }
            } catch (const cv::Exception& e) {
                std::cerr << "[ERROR] OpenCV exception opening USB camera: " << e.what() << std::endl;
                video_capture_ptr_.reset();
            }
#ifdef HAVE_CUDA
            cuda_video_reader_ptr_.reset();
#endif
            return;
        }

#ifdef HAVE_CUDA
        if (using_cuda_)
        {
            try {
                cuda_video_reader_ptr_ = cv::cudacodec::createVideoReader(
                    camera_uri_, params_.empty() ? default_params_cuda : params_);
                
                if (cuda_video_reader_ptr_) {
                    cuda_video_reader_ptr_->set(cv::cudacodec::ColorFormat::BGR);
                } else {
                    std::cerr << "[ERROR] Failed to create CUDA video reader" << std::endl;
                }
            } catch (const cv::Exception& e) {
                std::cerr << "[ERROR] OpenCV exception creating CUDA video reader: " << e.what() << std::endl;
                cuda_video_reader_ptr_.reset();
            }
            video_capture_ptr_.reset();
            return;
        }
        cuda_video_reader_ptr_.reset();
#endif
        
        try {
            video_capture_ptr_ = std::make_unique<cv::VideoCapture>(
                camera_uri_, cv::CAP_ANY, 
                params_.empty() ? default_params_normal : params_);
            
            if (!video_capture_ptr_->isOpened()) {
                std::cerr << "[ERROR] Failed to open video source: " << camera_uri_ << std::endl;
                video_capture_ptr_.reset();
            }
        } catch (const cv::Exception& e) {
            std::cerr << "[ERROR] OpenCV exception opening video source: " << e.what() << std::endl;
            video_capture_ptr_.reset();
        }
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Exception in create_video_capture: " << e.what() << std::endl;
    } catch (...) {
        std::cerr << "[ERROR] Unknown exception in create_video_capture" << std::endl;
    }
}

std::string VideoReader::get_codec_name() const noexcept
{
    try {
        if (ffmpeg_video_reader_ptr_)
        {
            return ffmpeg_video_reader_ptr_->get_codec_name();
        }
        
        if (!video_capture_ptr_) {
            return "Unknown";
        }
        
        auto fourcc_int = static_cast<int>(video_capture_ptr_->get(cv::CAP_PROP_FOURCC));
        if (fourcc_int <= 0) {
            return "Unknown";
        }
        
        const char *fourcc_chars = reinterpret_cast<const char *>(&fourcc_int);
        return std::string(fourcc_chars, 4);
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Exception in get_codec_name: " << e.what() << std::endl;
        return "Error";
    } catch (...) {
        std::cerr << "[ERROR] Unknown exception in get_codec_name" << std::endl;
        return "Error";
    }
}

std::string VideoReader::get_decoder_name() const noexcept
{
    try {
        if (ffmpeg_video_reader_ptr_)
        {
            return ffmpeg_video_reader_ptr_->get_decoder_name();
        }

        return video_capture_ptr_ ? video_capture_ptr_->getBackendName() : "Unknown";
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Exception in get_decoder_name: " << e.what() << std::endl;
        return "Error";
    } catch (...) {
        std::cerr << "[ERROR] Unknown exception in get_decoder_name" << std::endl;
        return "Error";
    }
}

std::string VideoReader::get_pixel_format_name() const noexcept
{
    try {
        if (ffmpeg_video_reader_ptr_)
        {
            return ffmpeg_video_reader_ptr_->get_pixel_format_name();
        }
        
        if (!video_capture_ptr_) {
            return "Unknown";
        }

        auto pixel_format_int = static_cast<int>(video_capture_ptr_->get(cv::CAP_PROP_CODEC_PIXEL_FORMAT));
        // First try to decode as FOURCC (4-character code)
        if (pixel_format_int > 1000) // Likely a FOURCC value
        {
            char fourcc_chars[5] = {0}; // 4 chars + null terminator
            fourcc_chars[0] = static_cast<char>(pixel_format_int & 0xFF);
            fourcc_chars[1] = static_cast<char>((pixel_format_int >> 8) & 0xFF);
            fourcc_chars[2] = static_cast<char>((pixel_format_int >> 16) & 0xFF);
            fourcc_chars[3] = static_cast<char>((pixel_format_int >> 24) & 0xFF);
            return std::string(fourcc_chars);
        }
        // Map common OpenCV pixel format values to readable names
        switch (pixel_format_int)
        {
            case 0: return "YUV420P";
            case 1: return "YUYV422";
            case 2: return "RGB24";
            case 3: return "BGR24";
            case 4: return "YUV422P";
            case 5: return "YUV444P";
            case 6: return "YUV410P";
            case 7: return "YUV411P";
            case 8: return "GRAY8";
            case 9: return "MONOWHITE";
            case 10: return "MONOBLACK";
            case 11: return "PAL8";
            case 12: return "YUVJ420P";
            case 13: return "YUVJ422P";
            case 14: return "YUVJ444P";
            case 15: return "XVMC_MPEG2_MC";
            case 16: return "XVMC_MPEG2_IDCT";
            case 17: return "UYVY422";
            case 18: return "UYYVYY411";
            case 19: return "BGR8";
            case 20: return "BGR4";
            case 21: return "BGR4_BYTE";
            case 22: return "RGB8";
            case 23: return "RGB4";
            case 24: return "RGB4_BYTE";
            case 25: return "NV12";
            case 26: return "NV21";
            case 27: return "ARGB";
            case 28: return "RGBA";
            case 29: return "ABGR";
            case 30: return "BGRA";
            case 31: return "GRAY16BE";
            case 32: return "GRAY16LE";
            case 33: return "YUV440P";
            case 34: return "YUVJ440P";
            case 35: return "YUVA420P";
            case -1: return "Unknown/Not Set";
            default: return "Unknown (" + std::to_string(pixel_format_int) + ")";
        }
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Exception in get_pixel_format_name: " << e.what() << std::endl;
        return "Error";
    } catch (...) {
        std::cerr << "[ERROR] Unknown exception in get_pixel_format_name" << std::endl;
        return "Error";
    }
}

int VideoReader::get_width() const noexcept
{
    try {
        if (ffmpeg_video_reader_ptr_)
        {
            return ffmpeg_video_reader_ptr_->get_width();
        }

#ifdef HAVE_CUDA
        if (using_cuda_ && cuda_video_reader_ptr_)
        {
            double value = 0.0;
            cuda_video_reader_ptr_->get(cv::CAP_PROP_FRAME_WIDTH, value);
            return static_cast<int>(value);
        }
#endif
        return video_capture_ptr_ ? static_cast<int>(video_capture_ptr_->get(cv::CAP_PROP_FRAME_WIDTH)) : 0;
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Exception in get_width: " << e.what() << std::endl;
        return 0;
    } catch (...) {
        std::cerr << "[ERROR] Unknown exception in get_width" << std::endl;
        return 0;
    }
}

int VideoReader::get_height() const noexcept
{
    try {
        if (ffmpeg_video_reader_ptr_)
        {
            return ffmpeg_video_reader_ptr_->get_height();
        }

#ifdef HAVE_CUDA
        if (using_cuda_ && cuda_video_reader_ptr_)
        {
            double value = 0.0;
            cuda_video_reader_ptr_->get(cv::CAP_PROP_FRAME_HEIGHT, value);
            return static_cast<int>(value);
        }
#endif
        return video_capture_ptr_ ? static_cast<int>(video_capture_ptr_->get(cv::CAP_PROP_FRAME_HEIGHT)) : 0;
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Exception in get_height: " << e.what() << std::endl;
        return 0;
    } catch (...) {
        std::cerr << "[ERROR] Unknown exception in get_height" << std::endl;
        return 0;
    }
}

double VideoReader::get_fps() const noexcept
{
    try {
        if (ffmpeg_video_reader_ptr_)
        {
            return ffmpeg_video_reader_ptr_->get_fps();
        }

#ifdef HAVE_CUDA
        if (using_cuda_ && cuda_video_reader_ptr_)
        {
            double value = 0.0;
            cuda_video_reader_ptr_->get(cv::CAP_PROP_FPS, value);
            return value;
        }
#endif
        return video_capture_ptr_ ? video_capture_ptr_->get(cv::CAP_PROP_FPS) : 0.0;
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Exception in get_fps: " << e.what() << std::endl;
        return 0.0;
    } catch (...) {
        std::cerr << "[ERROR] Unknown exception in get_fps" << std::endl;
        return 0.0;
    }
}

bool VideoReader::set_resolution(int width, int height) noexcept
{
    if (width <= 0 || height <= 0 || width > 16384 || height > 16384) {
        std::cerr << "[ERROR] Invalid resolution: " << width << "x" << height << std::endl;
        return false;
    }
    
    try {
        if (ffmpeg_video_reader_ptr_)
        {
            return ffmpeg_video_reader_ptr_->set_resolution(width, height);
        }
        if (video_capture_ptr_ && !using_cuda_)
        {
            return video_capture_ptr_->set(cv::CAP_PROP_FRAME_WIDTH, width) && 
                   video_capture_ptr_->set(cv::CAP_PROP_FRAME_HEIGHT, height);
        }
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Exception in set_resolution: " << e.what() << std::endl;
    } catch (...) {
        std::cerr << "[ERROR] Unknown exception in set_resolution" << std::endl;
    }
    return false;
}

std::vector<std::pair<int, int>> VideoReader::list_camera_resolutions() const noexcept
{
    try {
        if (ffmpeg_video_reader_ptr_)
        {
            return ffmpeg_video_reader_ptr_->list_camera_resolutions();
        }
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Exception in list_camera_resolutions: " << e.what() << std::endl;
    } catch (...) {
        std::cerr << "[ERROR] Unknown exception in list_camera_resolutions" << std::endl;
    }
    return {};
}