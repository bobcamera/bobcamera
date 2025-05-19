#pragma once

#include <map>
#include <string>
#include <vector>
#include <sstream>

#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include "bob_interfaces/msg/tracking.hpp"

#include "tracking_state.hpp"
#include "utils.hpp"

class AnnotatedFrameCreator
{
public:
    AnnotatedFrameCreator(std::unordered_map<std::string, std::string> settings)
        : settings_(settings),
          logger_(rclcpp::get_logger("annotated_frame_creator")),
          prediction_colour_(255, 0, 0)
    {
        bbox_line_thickness_ = string_to_number(settings_["bbox_line_thickness"], 1);
        font_scale_width_ = string_to_number(settings_["font_scale_width"], 0);
        font_scale_ = string_to_number(settings_["font_scale"], 1.0);
        zoom_factor_ = string_to_number(settings_["zoom_factor"], 2.0);
        enable_cropped_tracks_ = (settings_.contains("enable_cropped_tracks") ? (to_lowercase(settings_["enable_cropped_tracks"]) == "true") : true);
        const auto &[fr, fg, fb] = (settings_.contains("font_colour") ? extract_rgb(settings_["font_colour"]) : std::tuple<int, int, int>{50, 170, 50});
        font_colour_ = cv::Scalar(fr, fg, fb);
        const auto &[pr, pg, pb] = (settings_.contains("prediction_colour") ? extract_rgb(settings_["prediction_colour"]) : std::tuple<int, int, int>{255, 0, 0});
        prediction_colour_ = cv::Scalar(pr, pg, pb);
    }

    void create_frame(const std::shared_ptr<bob_camera::Tracking> &tracking_ptr,
                      bool enable_tracking_status)
    {
        int cropped_track_counter = 0;
        std::unordered_map<int, cv::Rect> detections;
        detections.reserve(tracking_ptr->detections.size());

        auto &annotated_frame = tracking_ptr->image_ptr->toMat();
        // std::unordered_map<std::string, bob_interfaces::msg::TrackPoint> final_trajectory_points;

        const cv::Size frame_size = tracking_ptr->image_ptr->size();
        const int total_height = frame_size.height;
        const int total_width = frame_size.width;

        std::string status_message = "(bob) Tracker Status: trackable:" +
                                     std::to_string(tracking_ptr->state.trackable) +
                                     ", alive:" + std::to_string(tracking_ptr->state.alive) +
                                     ", started:" + std::to_string(tracking_ptr->state.started) +
                                     ", ended:" + std::to_string(tracking_ptr->state.ended);

        if (total_width != font_scale_width_)
        {
            font_scale_ = get_optimal_font_scale(status_message, total_width * 0.30);
            font_scale_width_ = total_width;
        }

        if (enable_tracking_status)
        {
            cv::putText(annotated_frame, status_message, cv::Point(25, 50), cv::FONT_HERSHEY_SIMPLEX,
                        font_scale_, font_colour_, 2);
        }

        for (const auto &detection : tracking_ptr->detections)
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

                if (enable_cropped_tracks_)
                {
                    int margin = (cropped_track_counter == 0) ? 0 : 10;
                    double zoom_w = bbox.width * zoom_factor_;
                    int cropped_image_x = 10 + (cropped_track_counter * zoom_w) + margin;
                    if (cropped_image_x + zoom_w < total_width)
                    {
                        try
                        {
                            cv::Mat cropped_image = annotated_frame(cv::Rect(bbox.x, bbox.y, bbox.width, bbox.height));
                            const double cropped_percentage = 0.04;
                            int cropped_size = static_cast<int>(std::min(total_width, total_height) * cropped_percentage);

                            cv::Mat resized_cropped_image;
                            cv::resize(cropped_image, resized_cropped_image, cv::Size(cropped_size, cropped_size), 0, 0, cv::INTER_NEAREST);

                            int cropped_image_x_position = 10 + (cropped_track_counter * (cropped_size + 10));
                            int cropped_image_y_position = total_height - cropped_size - 10;

                            resized_cropped_image.copyTo(annotated_frame(cv::Rect(cropped_image_x_position, cropped_image_y_position, cropped_size, cropped_size)));
                            cv::rectangle(annotated_frame, cv::Rect(cropped_image_x_position, cropped_image_y_position, cropped_size, cropped_size), color, 1);

                            int textHeight = cv::getTextSize(std::to_string(id), cv::FONT_HERSHEY_SIMPLEX, font_scale_, 2, nullptr).height;
                            cv::putText(annotated_frame, std::to_string(id),
                                        cv::Point(cropped_image_x_position, cropped_image_y_position - textHeight / 2),
                                        cv::FONT_HERSHEY_SIMPLEX, font_scale_, color, 2);
                        }
                        catch (cv::Exception &e)
                        {
                            // Handle the exception
                        }
                        cropped_track_counter++;
                    }
                }
                cv::rectangle(annotated_frame, p1, p2, color, bbox_line_thickness_, 1);
                cv::putText(annotated_frame, std::to_string(id), cv::Point(p1.x, p1.y - 4), cv::FONT_HERSHEY_SIMPLEX, font_scale_, color, 1);
                // cv::ellipse(annotated_frame, ellipse, cv::Scalar(255, 0, 0), 1, cv::LINE_8);
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
        //                      cv::Point(static_cast<int>(previous_trajectory_point->center.x + x_offset),
        //                                static_cast<int>(previous_trajectory_point->center.y + y_offset)),
        //                      cv::Point(static_cast<int>(trajectory_point.center.x + x_offset),
        //                                static_cast<int>(trajectory_point.center.y + y_offset)),
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

        // return annotated_frame;
    }

private:
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

    inline cv::Rect get_sized_bbox(const cv::Rect &bbox_msg)
    {
        const int x = static_cast<int>(bbox_msg.x);
        const int y = static_cast<int>(bbox_msg.y);
        const int w = static_cast<int>(bbox_msg.width);
        const int h = static_cast<int>(bbox_msg.height);

        const int size = std::max(w, h) + 15;
        const int x1 = x + (w / 2) - (size / 2);
        const int y1 = y + (h / 2) - (size / 2);
        return {x1, y1, size, size};
    }

    inline cv::Scalar _color(TrackingStateEnum tracking_state)
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

    std::unordered_map<std::string, std::string> settings_;
    rclcpp::Logger logger_;

    cv::Scalar font_colour_;
    cv::Scalar prediction_colour_;
    int bbox_line_thickness_;
    int font_scale_width_;
    double font_scale_;
    bool enable_cropped_tracks_;
    double zoom_factor_;
};
