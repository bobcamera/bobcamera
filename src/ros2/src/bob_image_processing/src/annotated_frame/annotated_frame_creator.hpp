#pragma once
#ifndef __ANOTATED_FRAME_CREATOR_HPP__
#define __ANOTATED_FRAME_CREATOR_HPP__

#include <map>
#include <string>
#include <vector>
#include <sstream>

#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include "bob_interfaces/msg/tracking.hpp"

#include "../../bob_shared/include/tracking_state.hpp"

class AnnotatedFrameCreator
{
private:
    std::map<std::string, std::string> settings;
    rclcpp::Logger logger;
    cv::Scalar font_colour;
    cv::Scalar prediction_colour;
    int prediction_radius;
    int bbox_line_thickness;
    std::string frame_type;
    int fontScaleWidth;
    double fontScale;

public:
    AnnotatedFrameCreator(std::map<std::string, std::string> settings)
        : settings(settings),
          logger(rclcpp::get_logger("annotated_frame_creator")),
          font_colour(50, 170, 50),
          prediction_colour(255, 0, 0),
          prediction_radius(1),
          fontScaleWidth(0),
          fontScale(1)
    {
        bbox_line_thickness = 1; //(settings["visualiser_bbox_line_thickness"]),
        frame_type = "masked";         //(settings["visualiser_frame_source"]),
    }

    cv::Mat create_frame(const cv::Mat& annotated_frame,
                         const bob_interfaces::msg::Tracking& msg_tracking)
    {
        int cropped_track_counter = 0;
        bool enable_cropped_tracks = true; // settings.at("visualiser_show_cropped_tracks");
        double zoom_factor = 2.0;          // settings.at("visualiser_cropped_zoom_factor");
        std::map<int, cv::Rect> detections;
        std::map<std::string, bob_interfaces::msg::TrackPoint> final_trajectory_points;

        cv::Size frame_size = annotated_frame.size();
        int total_height = frame_size.height;
        int total_width = frame_size.width;

        std::string status_message = "(bob) Tracker Status: trackable:" +
                                     std::to_string(msg_tracking.state.trackable) +
                                     ", alive:" + std::to_string(msg_tracking.state.alive) +
                                     ", started:" + std::to_string(msg_tracking.state.started) +
                                     ", ended:" + std::to_string(msg_tracking.state.ended);

        if (total_width != fontScaleWidth)
        {
            fontScale = get_optimal_font_scale(status_message, total_width * 0.30);
            fontScaleWidth = total_width;
        }

        cv::putText(annotated_frame, status_message, cv::Point(25, 50), cv::FONT_HERSHEY_SIMPLEX,
                    fontScale, font_colour, 2);

        for (const auto &detection : msg_tracking.detections)
        {
            auto id = detection.id;
            TrackingStateEnum tracking_state = TrackingStateEnum(detection.state);
            if (tracking_state == TrackingStateEnum::ActiveTarget || tracking_state == TrackingStateEnum::ProvisionaryTarget)
            {
                cv::Rect bbox = get_sized_bbox(detection.bbox);
                detections[detection.id] = bbox;
                cv::Point p1(bbox.x, bbox.y);
                cv::Point p2(bbox.x + bbox.width, bbox.y + bbox.height);
                cv::Scalar color = _color(tracking_state);

                double orientation = detection.covariance_ellipse_orientation;
                double semi_major_axis = detection.covariance_ellipse_semi_major_axis;
                double semi_minor_axis = detection.covariance_ellipse_semi_minor_axis;

                cv::RotatedRect ellipse;
                ellipse.center = cv::Point2f(bbox.x + bbox.width / 2, bbox.y + bbox.height / 2);
                ellipse.size = cv::Size2f(semi_major_axis * 2, semi_minor_axis * 2);
                ellipse.angle = orientation * 180 / M_PI; 

                if (enable_cropped_tracks)
                {
                    int margin = (cropped_track_counter == 0) ? 0 : 10;
                    double zoom_w = bbox.width * zoom_factor;
                    // double zoom_h = bbox.height * zoom_factor;
                    int cropped_image_x = 10 + (cropped_track_counter * zoom_w) + margin;
                    // int cropped_image_y = total_height - (zoom_h + 10);
                    if (cropped_image_x + zoom_w < total_width)
                    {
                        try
                        {
                            cv::Mat cropped_image = annotated_frame(cv::Rect(bbox.x, bbox.y, bbox.width, bbox.height));
                            const double cropped_percentage = 0.04;  
                            int cropped_size = static_cast<int>(std::min(total_width, total_height) * cropped_percentage);

                            cv::Mat resized_cropped_image;
                            cv::resize(cropped_image, resized_cropped_image, cv::Size(cropped_size, cropped_size));

                            int cropped_image_x_position = 10 + (cropped_track_counter * (cropped_size + 10)); 
                            int cropped_image_y_position = total_height - cropped_size - 10; 

                            resized_cropped_image.copyTo(annotated_frame(cv::Rect(cropped_image_x_position, cropped_image_y_position, cropped_size, cropped_size)));
                            cv::rectangle(annotated_frame, cv::Rect(cropped_image_x_position, cropped_image_y_position, cropped_size, cropped_size), color, 1);

                            int textHeight = cv::getTextSize(std::to_string(id), cv::FONT_HERSHEY_SIMPLEX, fontScale, 2, nullptr).height;
                            cv::putText(annotated_frame, std::to_string(id),
                                        cv::Point(cropped_image_x_position, cropped_image_y_position - textHeight / 2),
                                        cv::FONT_HERSHEY_SIMPLEX, fontScale, color, 2);

                        }
                        catch (cv::Exception &e)
                        {
                            // Handle the exception
                        }
                        cropped_track_counter++;
                    }
                }
                cv::rectangle(annotated_frame, p1, p2, color, bbox_line_thickness, 1);
                cv::putText(annotated_frame, std::to_string(id), cv::Point(p1.x, p1.y - 4), cv::FONT_HERSHEY_SIMPLEX, fontScale, color, 1);
                cv::ellipse(annotated_frame, ellipse, cv::Scalar(255, 0, 0), 1, cv::LINE_8);
            }
        }

        // for (const auto &trajectory : msg_tracking.trajectories)
        // {
        //     const auto &trajectory_array = trajectory.trajectory;
        //     const bob_interfaces::msg::TrackPoint *previous_trajectory_point = nullptr;
        //     for (const auto &trajectory_point : trajectory_array)
        //     {
        //         if (previous_trajectory_point != nullptr)
        //         {
        //             cv::line(annotated_frame,
        //                      cv::Point(static_cast<int>(previous_trajectory_point->center.x),
        //                                static_cast<int>(previous_trajectory_point->center.y)),
        //                      cv::Point(static_cast<int>(trajectory_point.center.x),
        //                                static_cast<int>(trajectory_point.center.y)),
        //                      _color(TrackingStateEnum(trajectory_point.tracking_state)),
        //                      bbox_line_thickness);
        //         }
        //         previous_trajectory_point = &trajectory_point;
        //         final_trajectory_points[trajectory.id] = *previous_trajectory_point;
        //     }
        // }

        // for (const auto &prediction : msg_tracking.trajectories)
        // {
        //     const auto &prediction_array = prediction.trajectory;
        //     if (final_trajectory_points.count(prediction.id) > 0)
        //     {
        //         auto &previous_prediction_point = final_trajectory_points[prediction.id];
        //         for (const auto &prediction_point : prediction_array)
        //         {
        //             if (!previous_prediction_point.center.x && !previous_prediction_point.center.y)
        //             {
        //                 cv::line(annotated_frame,
        //                          cv::Point(static_cast<int>(previous_prediction_point.center.x),
        //                                    static_cast<int>(previous_prediction_point.center.y)),
        //                          cv::Point(static_cast<int>(prediction_point.center.x),
        //                                    static_cast<int>(prediction_point.center.y)),
        //                          prediction_colour, bbox_line_thickness);
        //             }
        //             previous_prediction_point = prediction_point;
        //         }
        //     }
        // }

        return annotated_frame;
    }

    double get_optimal_font_scale(const std::string &text, int width)
    {
        const int fontFace = cv::FONT_HERSHEY_SIMPLEX;
        const int thickness = 1;
        int baseline = 0;

        // Get the font scale based on the text width
        double fontScale = 1.0;
        int textWidth = cv::getTextSize(text, fontFace, fontScale, thickness, &baseline).width;
        while (textWidth > width)
        {
            fontScale -= 0.05;
            textWidth = cv::getTextSize(text, fontFace, fontScale, thickness, &baseline).width;
        }

        return fontScale;
    }

    cv::Rect get_sized_bbox(vision_msgs::msg::BoundingBox2D bbox_msg)
    {
        int x = static_cast<int>(bbox_msg.center.position.x - (bbox_msg.size_x / 2));
        int y = static_cast<int>(bbox_msg.center.position.y - (bbox_msg.size_y / 2));
        int w = static_cast<int>(bbox_msg.size_x);
        int h = static_cast<int>(bbox_msg.size_y);

        int size = std::max(w, h) + 15;
        int x1 = x + (w / 2) - (size / 2);
        int y1 = y + (h / 2) - (size / 2);
        return {x1, y1, size, size};
    }

    cv::Scalar _color(TrackingStateEnum tracking_state)
    {
        switch (tracking_state)
        {
        case TrackingStateEnum::ProvisionaryTarget:
            return cv::Scalar(25, 175, 175);
        case TrackingStateEnum::ActiveTarget:
            return cv::Scalar(50, 170, 50);
        case TrackingStateEnum::LostTarget:
            return cv::Scalar(50, 50, 225);
        default:
            return cv::Scalar(0, 0, 0); // Default color
        }
    }
};

#endif