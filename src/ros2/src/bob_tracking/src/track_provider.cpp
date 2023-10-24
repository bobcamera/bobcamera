#include <opencv2/opencv.hpp>

#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/bounding_box2_d_array.hpp>
#include "bob_interfaces/msg/tracking.hpp"

//#include "tracking/cv_trackers/video_tracker.hpp"
#include "tracking/sort/include/sort_tracker.h"

#include <boblib/api/utils/profiler.hpp>

#include "parameter_node.hpp"
#include "image_utils.hpp"

#include <visibility_control.h>

class TrackProvider
    : public ParameterNode
{
public:
    COMPOSITION_PUBLIC
    explicit TrackProvider(const rclcpp::NodeOptions & options)
        : ParameterNode("frame_provider_node", options)
        , enable_profiling_(false)
    {
        video_tracker_ = std::unique_ptr<BaseTracker>(new SORT::Tracker(std::map<std::string, std::string>({{"tracker_type", "MOSSE"}}))); // CV only currently - CSRT, MOSSE, KCF
        
        timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&TrackProvider::init, this));
    }

private:
    using Clock = std::chrono::high_resolution_clock;
    using TimePoint = std::chrono::time_point<Clock>;

    boblib::utils::Profiler profiler_;
    bool enable_profiling_;

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> masked_frame_subscription_;
    std::shared_ptr<message_filters::Subscriber<vision_msgs::msg::BoundingBox2DArray>> detector_bounding_boxes_subscription_;
    std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, vision_msgs::msg::BoundingBox2DArray>> time_synchronizer_;
    rclcpp::Publisher<bob_interfaces::msg::Tracking>::SharedPtr pub_tracker_tracking_;
    rclcpp::TimerBase::SharedPtr timer_;

    //VideoTracker video_tracker_;
    std::unique_ptr<BaseTracker> video_tracker_;

    friend std::shared_ptr<TrackProvider> std::make_shared<TrackProvider>();

    void init()
    {
        RCLCPP_INFO(get_logger(), "Initializing TrackProvider");
        timer_->cancel();

        rclcpp::QoS pub_qos_profile{10};
        pub_qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        pub_qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);
        pub_qos_profile.history(rclcpp::HistoryPolicy::KeepLast);
        
        rclcpp::QoS sub_qos_profile{10};
        sub_qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        sub_qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);
        sub_qos_profile.history(rclcpp::HistoryPolicy::KeepLast);
        auto rmw_qos_profile = sub_qos_profile.get_rmw_qos_profile();

        masked_frame_subscription_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(shared_from_this(), "bob/camera/all_sky/bayer", rmw_qos_profile);
        detector_bounding_boxes_subscription_ = std::make_shared<message_filters::Subscriber<vision_msgs::msg::BoundingBox2DArray>>(shared_from_this(), "bob/detector/all_sky/bounding_boxes", rmw_qos_profile);

        time_synchronizer_ = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, vision_msgs::msg::BoundingBox2DArray>>(*masked_frame_subscription_, *detector_bounding_boxes_subscription_, 10);
        time_synchronizer_->registerCallback(&TrackProvider::callback, this);

        pub_tracker_tracking_ = create_publisher<bob_interfaces::msg::Tracking>("bob/tracker/tracking", pub_qos_profile);

        declare_node_parameters();
    }

    void declare_node_parameters()
    {
        std::vector<ParameterNode::ActionParam> params = {
            ParameterNode::ActionParam(
                rclcpp::Parameter("enable_profiling", false), 
                [this](const rclcpp::Parameter& param) {enable_profiling_ = param.as_bool();}
            ),
            // ParameterNode::ActionParam(
            //     rclcpp::Parameter("vibe_params", R"({"threshold": 50, "bgSamples": 16, "requiredBGSamples": 1, "learningRate": 2})"), 
            //     [this](const rclcpp::Parameter& param) 
            //     {
            //         Json::CharReaderBuilder builder;
            //         std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
            //         std::string errors;
            //         Json::Value jsonObj;
            //         std::string jsonData = param.as_string();

            //         if (reader->parse(jsonData.c_str(), jsonData.c_str() + jsonData.size(), &jsonObj, &errors)) 
            //         {
            //             auto threshold = jsonObj["threshold"].asInt();
            //             auto bgSamples = jsonObj["bgSamples"].asInt();
            //             auto requiredBGSamples = jsonObj["requiredBGSamples"].asInt();
            //             auto learningRate = jsonObj["learningRate"].asInt();

            //             vibe_params_ = std::make_unique<boblib::bgs::VibeParams>(threshold, bgSamples, requiredBGSamples, learningRate);

            //             if (bgs_type_ == Vibe)
            //             {
            //                 createBGS(Vibe);
            //             }
            //         } 
            //         else 
            //         {
            //             RCLCPP_ERROR(get_logger(), "1. Failed to parse the JSON data: %s", errors.c_str());
            //         }
            //     }
            // ),
            // ParameterNode::ActionParam(
            //     rclcpp::Parameter("bgs", "vibe"), 
            //     [this](const rclcpp::Parameter& param) 
            //     {
            //         RCLCPP_INFO(get_logger(), "Setting BGS: %s", param.as_string().c_str());
            //         if (param.as_string() == "vibe")
            //         {
            //             bgsPtr = createBGS(Vibe);
            //         }
            //         else
            //         {
            //             bgsPtr = createBGS(WMV);
            //         }
            //     }
            // ),
        };
        add_action_parameters(params);
    }

    void callback(const sensor_msgs::msg::Image::SharedPtr &image_msg, const vision_msgs::msg::BoundingBox2DArray::SharedPtr &bounding_boxes_msg)
    {
        try
        {
            profile_start("Frame");

            cv::Mat img;
            ImageUtils::convert_image_msg(image_msg, img, true);

            std::vector<cv::Rect> bboxes;
            for (const auto &bbox2D : bounding_boxes_msg->boxes)
            {
                bboxes.push_back(cv::Rect(bbox2D.center.position.x - bbox2D.size_x / 2, bbox2D.center.position.y - bbox2D.size_y / 2, bbox2D.size_x, bbox2D.size_y));
            }

            video_tracker_->update_trackers(bboxes, img);

            publish_tracking(image_msg->header);

            profile_stop("Frame");
            profile_dump();
        }
        catch (std::exception &e)
        {
            RCLCPP_ERROR(get_logger(), "exception: %s", e.what());
        }
    }

    void publish_tracking(std_msgs::msg::Header &header)
    {
        bob_interfaces::msg::Tracking tracking_msg;
        tracking_msg.header = header;
        tracking_msg.state.trackable = video_tracker_->get_total_trackable_trackers();
        tracking_msg.state.alive = video_tracker_->get_total_live_trackers();
        tracking_msg.state.started = video_tracker_->get_total_trackers_started();
        tracking_msg.state.ended = video_tracker_->get_total_trackers_finished();

        for (const auto &tracker : video_tracker_->get_live_trackers())
        {
            add_track_detection(tracker, tracking_msg.detections);
            add_trajectory_detection(tracker, tracking_msg.trajectories);
            add_prediction(tracker, tracking_msg.predictions);
        }
        pub_tracker_tracking_->publish(tracking_msg);
    }

    void add_track_detection(const std::shared_ptr<BaseTrack> &tracker, std::vector<bob_interfaces::msg::TrackDetection> &detection_array)
    {
        auto bbox = tracker->get_bbox();

        bob_interfaces::msg::TrackDetection detect_msg;
        detect_msg.id = tracker->get_id();
        detect_msg.state = (int)tracker->get_tracking_state();
        detect_msg.bbox.center.position.x = bbox.x + bbox.width / 2;
        detect_msg.bbox.center.position.y = bbox.y + bbox.height / 2;
        detect_msg.bbox.size_x = bbox.width;
        detect_msg.bbox.size_y = bbox.height;

        detection_array.push_back(detect_msg);
    }

    void add_trajectory_detection(const std::shared_ptr<BaseTrack> &tracker, std::vector<bob_interfaces::msg::TrackTrajectory> &trajectory_array)
    {
        bob_interfaces::msg::TrackTrajectory track_msg;
        track_msg.id = std::to_string(tracker->get_id()) + std::string("-") + std::to_string(tracker->get_tracking_state());

        size_t i = 0;
        track_msg.trajectory.resize(tracker->get_center_points().size());
        for (const auto &center_point : tracker->get_center_points())
        {
            bob_interfaces::msg::TrackPoint& point = track_msg.trajectory[i++];
            point.center.x = center_point.first.x;
            point.center.y = center_point.first.y;
            point.tracking_state = (int)center_point.second;
        }

        trajectory_array.push_back(track_msg);
    }

    void add_prediction(const std::shared_ptr<BaseTrack> &tracker, std::vector<bob_interfaces::msg::TrackTrajectory> &prediction_array)
    {
        bob_interfaces::msg::TrackTrajectory track_msg;
        track_msg.id = std::to_string(tracker->get_id()) + std::string("-") + std::to_string(tracker->get_tracking_state());

        size_t i = 0;
        track_msg.trajectory.resize(tracker->get_predictor_center_points().size());
        for (const auto &center_point : tracker->get_predictor_center_points())
        {
            bob_interfaces::msg::TrackPoint& point = track_msg.trajectory[i++];
            point.center.x = center_point.x;
            point.center.y = center_point.y;
        }

        prediction_array.push_back(track_msg);
    }

    inline void profile_start(const std::string& region)
    {
        if (enable_profiling_)
        {
            profiler_.start(region);
        }
    }

    inline void profile_stop(const std::string& region)
    {
        if (enable_profiling_)
        {
            profiler_.stop(region);
        }
    }

    inline void profile_dump()
    {
        if (enable_profiling_)
        {
            if (profiler_.get_data("Frame").duration_in_seconds() > 1.0)
            {
                auto report = profiler_.report();
                RCLCPP_INFO(get_logger(), report.c_str());
                profiler_.reset();
            }
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrackProvider>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}

RCLCPP_COMPONENTS_REGISTER_NODE(TrackProvider)