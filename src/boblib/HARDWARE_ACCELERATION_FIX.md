# FFmpeg Video Reader Hardware Acceleration Fix

## Problem
Your FFmpegVideoReader was crashing with exit code -11 (SIGSEGV) when trying to open RTSP streams. The errors showed:

```
[AVHWDeviceContext @ 0x76b9c17bcf00] Cannot load libcuda.so.1
[AVHWDeviceContext @ 0x76b9c17bcf00] Could not dynamically load CUDA
[ERROR] av_hwdevice_ctx_create failed
[WARN] init_hw_device(2) failed
[ERROR] av_hwdevice_ctx_create failed
[WARN] init_hw_device(3) failed
[AVHWDeviceContext @ 0x76b9c17bcf00] Cannot open the X11 display :0.
[ERROR] av_hwdevice_ctx_create failed
[WARN] init_hw_device(1) failed
[AVHWDeviceContext @ 0x76b9c85f3080] Instance creation failure: VK_ERROR_INCOMPATIBLE_DRIVER
[ERROR] av_hwdevice_ctx_create failed
[WARN] init_hw_device(11) failed
```

## Root Cause
The FFmpeg video reader was attempting to initialize all available hardware acceleration methods (CUDA, VAAPI, Vulkan) and when they all failed in your containerized environment, it wasn't gracefully falling back to software decoding, leading to a segmentation fault.

## Solution
I've implemented several improvements to make hardware acceleration optional and more robust:

### 1. Added Hardware Acceleration Control
- Added `set_hardware_acceleration(bool enabled)` method to allow disabling hardware acceleration
- Added `hw_accel_enabled_` private member to track this setting

### 2. Improved Hardware Device Initialization
- Added environment detection to skip hardware acceleration types that are known to fail:
  - Skip CUDA if `libcuda.so.1` is not found
  - Skip VAAPI if no display environment is detected
- Better error handling with descriptive messages
- Graceful fallback to software decoding

### 3. Enhanced Frame Processing
- Fixed frame processing to handle cases where hardware acceleration is not used
- More robust checking of hardware pixel format

## Usage

### Quick Fix for Your ROS2 Node
Add this line before opening the stream:

```cpp
video_reader_->set_hardware_acceleration(false);
```

### Example Integration
```cpp
void CameraNode::configure()
{
    video_reader_ = std::make_unique<boblib::video::FFmpegVideoReader>();
    
    // Disable hardware acceleration to avoid crashes in containers
    video_reader_->set_hardware_acceleration(false);
    
    std::string rtsp_url = "rtsp://bob:Passw0rd@192.168.140.243/Streaming/Channels/101?transportmode=unicast&profile=profile_1";
    
    if (!video_reader_->open(rtsp_url))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open RTSP stream");
        throw std::runtime_error("Failed to open RTSP stream");
    }
    
    RCLCPP_INFO(this->get_logger(), "Successfully opened RTSP stream");
    RCLCPP_INFO(this->get_logger(), "Decoder: %s", video_reader_->get_decoder_name().c_str());
}
```

### Smart Hardware Acceleration Detection
For production code, you can auto-detect the environment:

```cpp
// Check if we're in a container or headless environment
bool in_container = (getenv("container") != nullptr) || 
                    (access("/.dockerenv", F_OK) == 0);
bool has_display = (getenv("DISPLAY") != nullptr) || 
                   (getenv("WAYLAND_DISPLAY") != nullptr);

if (in_container || !has_display)
{
    video_reader_->set_hardware_acceleration(false);
}
```

## Files Modified

1. **ffmpeg_video_reader.hpp**: Added `set_hardware_acceleration()` method and `hw_accel_enabled_` member
2. **ffmpeg_video_reader.cpp**: 
   - Improved `init_decoder()` to respect hardware acceleration setting
   - Enhanced `init_hw_device()` with environment detection and better error handling
   - Fixed frame processing logic

## Benefits

1. **No More Crashes**: Software decoding prevents segmentation faults
2. **Better Logging**: Clear indication of whether hardware or software decoding is used
3. **Environment Aware**: Automatically detects problematic environments
4. **Backwards Compatible**: Hardware acceleration is still enabled by default
5. **Performance Info**: Logs which acceleration method is used

## Performance Impact

Software decoding will use more CPU than hardware acceleration, but it's reliable and should work in all environments. For RTSP streams, the bottleneck is often network rather than decoding, so the impact may be minimal.

## Testing

Use the provided `test_video_reader.cpp` to verify the fixes work with your RTSP stream:

```bash
cd /workspaces/bobcamera/src/boblib
./test_video_reader
```

This will test opening your RTSP stream with hardware acceleration disabled and show detailed information about the stream and decoding process.
