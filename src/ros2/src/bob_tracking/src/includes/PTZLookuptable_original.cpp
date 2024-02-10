#include "PTZLookuptable.h"
#include <vector>


// Define and initialize the PTZXAbsoluteMoveFromTrack lookup table
static std::vector<std::vector<float>> PTZXAbsoluteMoveFromTrack(BOB_RTSP_WIDTH+1, std::vector<float>(BOB_RTSP_HEIGHT+1, 0.0));
// Populate PTZXAbsoluteMoveFromTrack
void populatePTZXAbsoluteMoveFromTrack() {
    for(int x = 0; x < BOB_RTSP_WIDTH; ++x) {
        float valueX = -1.0f + (2.0f * x) / (BOB_RTSP_WIDTH - 1);
        for (int y = 0; y < BOB_RTSP_HEIGHT; ++y) {
            PTZXAbsoluteMoveFromTrack[x][y] = valueX;
        }
    }
}
// Function to get the PTZXAbsoluteMoveFromTrack lookup table
const std::vector<std::vector<float>>& getPTZXAbsoluteMoveFromTrack() {
    static bool initialized = false;
    if (!initialized) {
        populatePTZXAbsoluteMoveFromTrack();
        initialized = true;
    }
    return PTZXAbsoluteMoveFromTrack;
}

// Define and initialize the PTZYAbsoluteMoveFromTrack lookup table
static std::vector<std::vector<float>> PTZYAbsoluteMoveFromTrack(BOB_RTSP_WIDTH+1, std::vector<float>(BOB_RTSP_HEIGHT+1, 0.0));
// Populate PTZYAbsoluteMoveFromTrack
void populatePTZYAbsoluteMoveFromTrack() {
    for (int y = 0; y < BOB_RTSP_HEIGHT; ++y) {
        float valueY = -1.0f + (2.0f * y) / (BOB_RTSP_HEIGHT - 1);
        for (int x = 0; x < BOB_RTSP_WIDTH; ++x) {
            PTZYAbsoluteMoveFromTrack[x][y] = valueY;
        }
    }
}
// Function to get the PTZYAbsoluteMoveFromTrack lookup table
const std::vector<std::vector<float>>& getPTZYAbsoluteMoveFromTrack() {
    static bool initialized = false;
    if (!initialized) {
        populatePTZYAbsoluteMoveFromTrack();
        initialized = true;
    }
    return PTZYAbsoluteMoveFromTrack;
}

