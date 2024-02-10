#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include "PTZLookuptable.h" // Assuming this header file defines BOB_RTSP_WIDTH and BOB_RTSP_HEIGHT
//#include "rclcpp/rclcpp.hpp"

const std::string PTZ_CALIBRATION_FILE_X = "/workspaces/bobcamera/src/ros2/src/bob_tracking/src/includes/PTZCalibrationDataX.txt";
const std::string PTZ_CALIBRATION_FILE_Y = "/workspaces/bobcamera/src/ros2/src/bob_tracking/src/includes/PTZCalibrationDataY.txt";

// Function to populate PTZXAbsoluteMoveFromTrack from file
void populatePTZXAbsoluteMoveFromTrack(std::vector<std::vector<float>>& PTZXAbsoluteMoveFromTrack) {
    std::ifstream file(PTZ_CALIBRATION_FILE_X);
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << PTZ_CALIBRATION_FILE_X << std::endl;
        return;
    }

    for(int x = 0; x <= BOB_RTSP_WIDTH; ++x) {
        for (int y = 0; y <= BOB_RTSP_HEIGHT; ++y) {
            float value;
            if (!(file >> value)) {
                std::cerr << "Error reading from file: " << PTZ_CALIBRATION_FILE_X << std::endl;
                return;
            }
            PTZXAbsoluteMoveFromTrack[x][y] = value;
        }
    }
    file.close();
}

// Function to populate PTZYAbsoluteMoveFromTrack from file
void populatePTZYAbsoluteMoveFromTrack(std::vector<std::vector<float>>& PTZYAbsoluteMoveFromTrack) {
    std::ifstream file(PTZ_CALIBRATION_FILE_Y);
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << PTZ_CALIBRATION_FILE_Y << std::endl;
        return;
    }

    for(int y = 0; y <= BOB_RTSP_HEIGHT; ++y) {
        for (int x = 0; x <= BOB_RTSP_WIDTH; ++x) {
            float value;
            if (!(file >> value)) {
                std::cerr << "Error reading from file: " << PTZ_CALIBRATION_FILE_Y << std::endl;
                return;
            }
            PTZYAbsoluteMoveFromTrack[x][y] = value;
        }
    }
    file.close();
}


// Function to get the PTZXAbsoluteMoveFromTrack lookup table
const std::vector<std::vector<float>>& getPTZXAbsoluteMoveFromTrack() {
    static std::vector<std::vector<float>> PTZXAbsoluteMoveFromTrack(BOB_RTSP_WIDTH+1, std::vector<float>(BOB_RTSP_HEIGHT+1, 0.0));
    static bool initialized = false;
    if (!initialized) {
        populatePTZXAbsoluteMoveFromTrack(PTZXAbsoluteMoveFromTrack);
        initialized = true;
    }
    return PTZXAbsoluteMoveFromTrack;
}

// Function to get the PTZYAbsoluteMoveFromTrack lookup table
const std::vector<std::vector<float>>& getPTZYAbsoluteMoveFromTrack() {
    static std::vector<std::vector<float>> PTZYAbsoluteMoveFromTrack(BOB_RTSP_WIDTH+1, std::vector<float>(BOB_RTSP_HEIGHT+1, 0.0));
    static bool initialized = false;
    if (!initialized) {
        populatePTZYAbsoluteMoveFromTrack(PTZYAbsoluteMoveFromTrack);
        initialized = true;
    }
    return PTZYAbsoluteMoveFromTrack;
}
