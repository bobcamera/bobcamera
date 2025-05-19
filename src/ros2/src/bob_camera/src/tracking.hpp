#pragma once

#include <memory>
#include <rcpputils/endian.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <boblib/api/base/Image.hpp>

namespace bob_camera
{
    struct TrackState
    {
        // The number of blobs being tracked that have been promoted to a state of : ACTIVE_TARGET
        int32_t trackable;

        // The number of blobs currently being tracked of all states
        int32_t alive;

        // The total number of blobs tracked, past and present
        int32_t started;

        // The total number of blobs no longer bing tracked
        int32_t ended;
    };

    struct TrackDetection
    {
        int32_t id;
        int32_t state;
        cv::Rect bbox;
    };

    // Defines a point that is part of the track or path of a 2D detection.
    struct TrackPoint
    {
        // The tracking state of this point, we currently support:
        //   PROVISIONARY_TARGET = 1
        //   ACTIVE_TARGET = 2
        //   LOST_TARGET = 3
        int32_t tracking_state;

        // The center point of this trajectory
        cv::Point center;
    };

    // Defines a trajectory or path of a 2D detection.
    struct TrackTrajectory
    {
        // ID used for consistency across multiple detection messages.Detections
        // of the same object in different detection messages should have the same id.
        // This field may be empty.
        std::string id;
        // The points representing the trajectory of this detection
        std::vector<TrackPoint> trajectory;
    };

    struct Tracking
    {
        using HeaderPtr = std::shared_ptr<std_msgs::msg::Header>;
        using ImagePtr = std::shared_ptr<boblib::base::Image>;

        Tracking() = default;

        Tracking(HeaderPtr header,
                ImagePtr image,
                float fps)
            : header_ptr(std::move(header))
            , image_ptr(std::move(image))
            , fps(fps)
        {
        }

        HeaderPtr header_ptr;
        ImagePtr image_ptr;
        TrackState state;
        std::vector<TrackDetection> detections;
        std::vector<TrackTrajectory> trajectories;
        std::vector<TrackTrajectory> predictions;

        float fps;
    };
}