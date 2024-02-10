// PTZLookuptable.h
#ifndef PTZ_LOOKUPTABLE_H
#define PTZ_LOOKUPTABLE_H

#include <vector>

const int BOB_RTSP_WIDTH = 1080;
const int BOB_RTSP_HEIGHT = 1920;

// Function to get the PTZXAbsoluteMoveFromTrack lookup table
const std::vector<std::vector<float>>& getPTZXAbsoluteMoveFromTrack();

// Function to get the PTZYAbsoluteMoveFromTrack lookup table
const std::vector<std::vector<float>>& getPTZYAbsoluteMoveFromTrack();

#endif //PTZ_LOOKUPTABLE_H
