// Example of how to modify your ROS2 camera node to avoid hardware acceleration issues

// In your camera node initialization (likely in the constructor or configure method):

void CameraNode::configure()
{
    // ... other initialization code ...
    
    // Create video reader
    video_reader_ = std::make_unique<boblib::video::FFmpegVideoReader>();
    
    // IMPORTANT: Disable hardware acceleration to avoid crashes in containerized environments
    video_reader_->set_hardware_acceleration(false);
    
    // Get RTSP URL from parameters
    std::string rtsp_url = "rtsp://bob:Passw0rd@192.168.140.243/Streaming/Channels/101?transportmode=unicast&profile=profile_1";
    
    RCLCPP_INFO(this->get_logger(), "Trying to open RTSP Stream '%s'", rtsp_url.c_str());
    
    if (!video_reader_->open(rtsp_url))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open RTSP stream: %s", rtsp_url.c_str());
        throw std::runtime_error("Failed to open RTSP stream");
    }
    
    RCLCPP_INFO(this->get_logger(), "Successfully opened RTSP stream");
    RCLCPP_INFO(this->get_logger(), "Resolution: %dx%d", 
                video_reader_->get_width(), video_reader_->get_height());
    RCLCPP_INFO(this->get_logger(), "FPS: %.2f", video_reader_->get_fps());
    RCLCPP_INFO(this->get_logger(), "Codec: %s", video_reader_->get_codec_name().c_str());
    RCLCPP_INFO(this->get_logger(), "Decoder: %s", video_reader_->get_decoder_name().c_str());
    
    // ... rest of initialization ...
}

// Alternative: You can also enable hardware acceleration selectively:
void CameraNode::configure_with_selective_hw_accel()
{
    video_reader_ = std::make_unique<boblib::video::FFmpegVideoReader>();
    
    // Check if we're in a container or headless environment
    bool in_container = (getenv("container") != nullptr) || 
                        (access("/.dockerenv", F_OK) == 0);
    bool has_display = (getenv("DISPLAY") != nullptr) || 
                       (getenv("WAYLAND_DISPLAY") != nullptr);
    
    if (in_container || !has_display)
    {
        RCLCPP_INFO(this->get_logger(), "Detected containerized/headless environment, disabling hardware acceleration");
        video_reader_->set_hardware_acceleration(false);
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Hardware acceleration enabled");
        // hardware acceleration remains enabled by default
    }
    
    // ... rest of initialization ...
}
