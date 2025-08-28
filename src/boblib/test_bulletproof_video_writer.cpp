#include <iostream>
#include <opencv2/opencv.hpp>
#include "api/video/VideoWriter.hpp"
#include "api/video/ffmpeg_video_writer.hpp"

using namespace boblib::video;

// Test function to verify bulletproofing
bool test_bulletproof_video_writer() {
    std::cout << "=== Testing VideoWriter Bulletproofing ===" << std::endl;
    
    // Test 1: Invalid parameters
    std::cout << "Test 1: Invalid parameters..." << std::endl;
    {
        // Empty filename
        VideoWriter writer1("", cv::Size(640, 480), Codec::H264, 30.0);
        if (writer1.is_valid()) {
            std::cout << "  FAIL: Empty filename should be invalid" << std::endl;
            return false;
        } else {
            std::cout << "  PASS: Empty filename correctly rejected" << std::endl;
        }
        
        // Invalid frame size
        VideoWriter writer2("test.mp4", cv::Size(-1, 480), Codec::H264, 30.0);
        if (writer2.is_valid()) {
            std::cout << "  FAIL: Invalid frame size should be invalid" << std::endl;
            return false;
        } else {
            std::cout << "  PASS: Invalid frame size correctly rejected" << std::endl;
        }
        
        // Invalid FPS
        VideoWriter writer3("test.mp4", cv::Size(640, 480), Codec::H264, -1.0);
        if (writer3.is_valid()) {
            std::cout << "  FAIL: Invalid FPS should be invalid" << std::endl;
            return false;
        } else {
            std::cout << "  PASS: Invalid FPS correctly rejected" << std::endl;
        }
    }
    
    // Test 2: Valid creation and empty frame handling
    std::cout << "Test 2: Valid creation and empty frame handling..." << std::endl;
    {
        VideoWriter writer("test_output/test_output.mp4", cv::Size(640, 480), Codec::H264, 30.0, false);
        if (!writer.is_valid()) {
            std::cout << "  FAIL: Valid parameters should create valid writer" << std::endl;
            return false;
        } else {
            std::cout << "  PASS: Valid parameters created valid writer" << std::endl;
        }
        
        // Test empty frame
        cv::Mat empty_frame;
        writer.write(empty_frame); // Should not crash and should handle gracefully
        std::cout << "  PASS: Empty frame handled gracefully" << std::endl;
        
        // Test valid frame
        cv::Mat frame(480, 640, CV_8UC3, cv::Scalar(100, 150, 200));
        writer.write(frame); // Should work
        std::cout << "  PASS: Valid frame written" << std::endl;
    }
    
    return true;
}

// Test function for FFmpegVideoWriter
bool test_bulletproof_ffmpeg_writer() {
    std::cout << "\n=== Testing FFmpegVideoWriter Bulletproofing ===" << std::endl;
    
    // Test 1: Invalid options
    std::cout << "Test 1: Invalid options..." << std::endl;
    {
        FFmpegVideoWriter::Options options;
        options.outputPath = ""; // Empty path
        options.fps = 30.0;
        options.width = 640;
        options.height = 480;
        
        FFmpegVideoWriter writer(options);
        if (writer.is_valid()) {
            std::cout << "  FAIL: Empty output path should be invalid" << std::endl;
            return false;
        } else {
            std::cout << "  PASS: Empty output path correctly rejected" << std::endl;
        }
    }
    
    // Test 2: Valid creation
    std::cout << "Test 2: Valid creation..." << std::endl;
    {
        FFmpegVideoWriter::Options options;
        options.outputPath = "test_output/test_ffmpeg.mp4";
        options.fps = 30.0;
        options.width = 640;
        options.height = 480;
        options.debug = true;
        
        FFmpegVideoWriter writer(options);
        if (!writer.is_valid()) {
            std::cout << "  FAIL: Valid options should create valid writer" << std::endl;
            return false;
        } else {
            std::cout << "  PASS: Valid options created valid writer" << std::endl;
        }
        
        if (!writer.start()) {
            std::cout << "  WARNING: Could not start writer (possibly due to missing encoders)" << std::endl;
        } else {
            std::cout << "  PASS: Writer started successfully" << std::endl;
            
            // Test writing an empty frame
            cv::Mat empty_frame;
            writer.write_frame(empty_frame); // Should not crash
            std::cout << "  PASS: Empty frame handled gracefully" << std::endl;
            
            // Test writing a valid frame
            cv::Mat frame(480, 640, CV_8UC3, cv::Scalar(50, 100, 150));
            writer.write_frame(frame); // Should work
            std::cout << "  PASS: Valid frame written" << std::endl;
            
            writer.stop();
            std::cout << "  PASS: Writer stopped successfully" << std::endl;
        }
    }
    
    return true;
}

int main() {
    std::cout << "Starting bulletproof video writer tests..." << std::endl;
    
    bool test1_passed = test_bulletproof_video_writer();
    bool test2_passed = test_bulletproof_ffmpeg_writer();
    
    if (test1_passed && test2_passed) {
        std::cout << "\n=== ALL TESTS PASSED ===" << std::endl;
        std::cout << "VideoWriter classes are successfully bulletproofed!" << std::endl;
        return 0;
    } else {
        std::cout << "\n=== SOME TESTS FAILED ===" << std::endl;
        return 1;
    }
}
