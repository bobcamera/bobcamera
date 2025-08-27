#include "api/video/ffmpeg_video_reader.hpp"
#include <iostream>
#include <chrono>

int main()
{
    // Test the video reader with an RTSP stream
    std::string rtsp_url = "rtsp://bob:Passw0rd@192.168.140.243/Streaming/Channels/101?transportmode=unicast&profile=profile_1";
    
    boblib::video::FFmpegVideoReader reader;
    
    // Disable hardware acceleration to avoid the crashes you're experiencing
    reader.set_hardware_acceleration(false);
    
    std::cout << "Attempting to open RTSP stream: " << rtsp_url << std::endl;
    
    if (!reader.open(rtsp_url))
    {
        std::cerr << "Failed to open RTSP stream" << std::endl;
        return 1;
    }
    
    std::cout << "Successfully opened stream!" << std::endl;
    std::cout << "Resolution: " << reader.get_width() << "x" << reader.get_height() << std::endl;
    std::cout << "FPS: " << reader.get_fps() << std::endl;
    std::cout << "Codec: " << reader.get_codec_name() << std::endl;
    std::cout << "Decoder: " << reader.get_decoder_name() << std::endl;
    std::cout << "Pixel format: " << reader.get_pixel_format_name() << std::endl;
    
    // Try to read a few frames
    cv::Mat frame;
    int frame_count = 0;
    auto start = std::chrono::steady_clock::now();
    
    for (int i = 0; i < 10; ++i)
    {
        if (reader.read(frame))
        {
            frame_count++;
            std::cout << "Read frame " << frame_count << " (" << frame.cols << "x" << frame.rows << ")" << std::endl;
        }
        else
        {
            std::cerr << "Failed to read frame " << i+1 << std::endl;
            break;
        }
    }
    
    auto end = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    
    std::cout << "Read " << frame_count << " frames in " << duration.count() << "ms" << std::endl;
    if (frame_count > 0)
    {
        double actual_fps = frame_count * 1000.0 / duration.count();
        std::cout << "Actual FPS: " << actual_fps << std::endl;
    }
    
    reader.close();
    std::cout << "Stream closed successfully" << std::endl;
    
    return 0;
}
