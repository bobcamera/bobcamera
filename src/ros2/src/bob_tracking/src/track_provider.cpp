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
#include "bob_interfaces/msg/ptz_absolute_move.hpp"

#include "sort/include/sort_tracker.h"

#include "parameter_node.hpp"
#include "image_utils.hpp"
#include "includes/PTZLookuptable.h"
#include <visibility_control.h>

static int numofframes = 0;

/*
const int BOB_RTSP_WIDTH = 1080; 
const int BOB_RTSP_HEIGHT = 1920;
std::vector<std::vector<float>> PTZXAbsoluteMoveFromTrack(BOB_RTSP_WIDTH, std::vector<float>(BOB_RTSP_HEIGHT, 0.0));
std::vector<std::vector<float>> PTZYAbsoluteMoveFromTrack(BOB_RTSP_WIDTH, std::vector<float>(BOB_RTSP_HEIGHT, 0.0));
*/
const auto& PTZXAbsoluteMoveFromTrack = getPTZXAbsoluteMoveFromTrack();
const auto& PTZYAbsoluteMoveFromTrack = getPTZYAbsoluteMoveFromTrack();

class TrackProvider
    : public ParameterNode
{
public:
    COMPOSITION_PUBLIC
    explicit TrackProvider(const rclcpp::NodeOptions & options)
        : ParameterNode("frame_provider_node", options)
        , video_tracker_(get_logger()) 
    {
        one_shot_timer_ = this->create_wall_timer(
            std::chrono::seconds(1), 
            [this]() {
                this->init(); 
                this->one_shot_timer_.reset();  
            }
        );
    }

private:
    using Clock = std::chrono::high_resolution_clock;
    using TimePoint = std::chrono::time_point<Clock>;

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> masked_frame_subscription_;
    std::shared_ptr<message_filters::Subscriber<vision_msgs::msg::BoundingBox2DArray>> detector_bounding_boxes_subscription_;
    std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, vision_msgs::msg::BoundingBox2DArray>> time_synchronizer_;
    rclcpp::Publisher<bob_interfaces::msg::Tracking>::SharedPtr pub_tracker_tracking_;
    rclcpp::Publisher<bob_interfaces::msg::PTZAbsoluteMove>::SharedPtr pub_tracker_PTZabsolutemove_;
    rclcpp::TimerBase::SharedPtr one_shot_timer_;
    SORT::Tracker video_tracker_;
    double fps_;

    friend std::shared_ptr<TrackProvider> std::make_shared<TrackProvider>();

    void init()
    {
        RCLCPP_INFO(get_logger(), "Initializing TrackProvider");

        declare_node_parameters();

        rclcpp::QoS pub_qos_profile{10};
        pub_qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        pub_qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);
        pub_qos_profile.history(rclcpp::HistoryPolicy::KeepLast);
        
        rclcpp::QoS sub_qos_profile{10};
        sub_qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        sub_qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);
        sub_qos_profile.history(rclcpp::HistoryPolicy::KeepLast);
        auto rmw_qos_profile = sub_qos_profile.get_rmw_qos_profile();

        masked_frame_subscription_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(shared_from_this(), "bob/frames/allsky/masked/detection", rmw_qos_profile);
        detector_bounding_boxes_subscription_ = std::make_shared<message_filters::Subscriber<vision_msgs::msg::BoundingBox2DArray>>(shared_from_this(), "bob/detection/allsky/boundingboxes", rmw_qos_profile);

        time_synchronizer_ = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, vision_msgs::msg::BoundingBox2DArray>>(*masked_frame_subscription_, *detector_bounding_boxes_subscription_, 10);
        time_synchronizer_->registerCallback(&TrackProvider::callback, this);        

        pub_tracker_tracking_ = create_publisher<bob_interfaces::msg::Tracking>("bob/tracker/tracking", pub_qos_profile);
        pub_tracker_PTZabsolutemove_= create_publisher<bob_interfaces::msg::PTZAbsoluteMove>("bob/ptz/move/absolute", pub_qos_profile);
    }

    void declare_node_parameters() {
        std::vector<ParameterNode::ActionParam> params = {
            ParameterNode::ActionParam(rclcpp::Parameter("video_fps", 30.0), [this](const rclcpp::Parameter& param) {fps_ = param.as_double();}),
        };
        add_action_parameters(params);
    }

    void callback(const sensor_msgs::msg::Image::SharedPtr &image_msg, const vision_msgs::msg::BoundingBox2DArray::SharedPtr &bounding_boxes_msg)
    {
        try
        {
            cv::Mat img;
            std::vector<cv::Rect> bboxes;
            for (const auto &bbox2D : bounding_boxes_msg->boxes)
            {
                bboxes.push_back(cv::Rect(bbox2D.center.position.x - bbox2D.size_x / 2, bbox2D.center.position.y - bbox2D.size_y / 2, bbox2D.size_x, bbox2D.size_y));
            }

            video_tracker_.update_trackers(bboxes, img);

            publish_tracking(image_msg->header);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(get_logger(), "CV bridge exception: %s", e.what());
        }
        catch (cv::Exception &cve)
        {
            RCLCPP_ERROR(get_logger(), "Open CV exception: %s", cve.what());
        }        
    }
    void publish_tracking(std_msgs::msg::Header &header)
    {

        bob_interfaces::msg::Tracking tracking_msg;
        bob_interfaces::msg::PTZAbsoluteMove PTZ_msg;

        tracking_msg.header = header;
        tracking_msg.state.trackable = video_tracker_.get_total_trackable_trackers();
        tracking_msg.state.alive = video_tracker_.get_total_live_trackers();
        tracking_msg.state.started = video_tracker_.get_total_trackers_started();
        tracking_msg.state.ended = video_tracker_.get_total_trackers_finished();
        for (const auto &tracker : video_tracker_.get_live_trackers())
        {
            add_track_detection(tracker, tracking_msg.detections);
            add_trajectory_detection(tracker, tracking_msg.trajectories);
            add_prediction(tracker, tracking_msg.predictions);
        }
        pub_tracker_tracking_->publish(tracking_msg);
 
    

        //export BOB_RTSP_WIDTH=${BOB_RTSP_WIDTH:-"1920"}
        //export BOB_RTSP_HEIGHT=${BOB_RTSP_HEIGHT:-"1080"}    

        /*    
        float PTZ_pospantilt_X[BOB_RTSP_WIDTH][BOB_RTSP_HEIGHT];
        float PTZ_pospantilt_Y[BOB_RTSP_WIDTH][BOB_RTSP_HEIGHT];


        int FisheyeX = video_tracker_.get_active_trackers().begin()->get_predictor_center_points().begin()->x;
        int FisheyeY = video_tracker_.get_active_trackers().begin()->get_predictor_center_points().begin()->y;

        PTZ_msg.pospantiltx = PTZ_pospantilt_X[FisheyeX][FisheyeY];
        PTZ_msg.pospantiltx = PTZ_pospantilt_Y[FisheyeX][FisheyeY];
        */
       // Define two 2D arrays of floats


/*
        PTZ_msg.pospantiltx = (float)0.5;
        PTZ_msg.pospantilty = (float)1.5;
        PTZ_msg.poszoomx = (float)0.0;
        PTZ_msg.speedpantiltx = (float)1.0;
        PTZ_msg.speedpantilty = (float)1.0;
        PTZ_msg.speedzoomx = float(1.0);



        if(numofframes == 0){
            // Populate PTZXAbsoluteMoveFromTrack from -1 to 1 from left to right
            for (int x = 0; x < BOB_RTSP_WIDTH; ++x) {
                float valueX = -1.0f + (2.0f * x) / (BOB_RTSP_WIDTH - 1); // Linear mapping from -1 to 1
                for (int y = 0; y < BOB_RTSP_HEIGHT; ++y) {
                    PTZXAbsoluteMoveFromTrack[x][y] = valueX;
                }
            }

            // PTZYAbsoluteMoveFromTrack arrayY from -1 to 1 from top to bottom
            for (int y = 0; y < BOB_RTSP_HEIGHT; ++y) {
                float valueY = -1.0f + (2.0f * y) / (BOB_RTSP_HEIGHT - 1); // Linear mapping from -1 to 1
                for (int x = 0; x < BOB_RTSP_WIDTH; ++x) {
                    PTZYAbsoluteMoveFromTrack[x][y] = valueY;
                }
            }
        }
*/
        if(numofframes % 6 == 0){


            const auto& live_trackers = video_tracker_.get_live_trackers();
            if (!live_trackers.empty()){
                const auto& first_tracker = *live_trackers.begin();
                //RCLCPP_INFO(get_logger(),"video_tracker_.get_live_trackers() not empty");// 
                if (!first_tracker.get_predictor_center_points().empty()) {
                    const std::vector<cv::Point>& PredCentPoints = first_tracker.get_predictor_center_points();
                    const cv::Point& lastPoint = PredCentPoints.back();
                    RCLCPP_INFO(get_logger(),"X predicted point: %d", lastPoint.x);// 
                    RCLCPP_INFO(get_logger(),"Y predicted point: %d", lastPoint.y);// 
                    int FisheyeOldestTrackIdNewestPredictedMeanX = (lastPoint.x+BOB_RTSP_WIDTH/2)%(BOB_RTSP_WIDTH-1);
                    int FisheyeOldestTrackIdNewestPredictedMeanY = (lastPoint.y+BOB_RTSP_HEIGHT/2)%(BOB_RTSP_HEIGHT-1);
                    if((0 <= FisheyeOldestTrackIdNewestPredictedMeanX) && (FisheyeOldestTrackIdNewestPredictedMeanX <= BOB_RTSP_WIDTH) && (0 <= FisheyeOldestTrackIdNewestPredictedMeanY)  && (FisheyeOldestTrackIdNewestPredictedMeanY <= BOB_RTSP_HEIGHT)){
                        PTZ_msg.pospantiltx =  PTZXAbsoluteMoveFromTrack[FisheyeOldestTrackIdNewestPredictedMeanX][FisheyeOldestTrackIdNewestPredictedMeanY];
                        PTZ_msg.pospantilty =  PTZYAbsoluteMoveFromTrack[FisheyeOldestTrackIdNewestPredictedMeanX][FisheyeOldestTrackIdNewestPredictedMeanY];
                        PTZ_msg.poszoomx = (float)0.0;
                        PTZ_msg.speedpantiltx = (float)1.0;
                        PTZ_msg.speedpantilty = (float)1.0;
                        PTZ_msg.speedzoomx = float(1.0);
                        //RCLCPP_INFO_STREAM(this->get_logger(),"In publisher loop" << numofframes);
                        RCLCPP_INFO_STREAM(this->get_logger(),"Current Position Target: x" << PTZ_msg.pospantiltx <<  " : y" << PTZ_msg.pospantilty);
                        pub_tracker_PTZabsolutemove_->publish(PTZ_msg);
                    } else {
                        RCLCPP_WARN_STREAM(this->get_logger(),"Requested Point out of Bounds> x" << FisheyeOldestTrackIdNewestPredictedMeanX <<  " : y" << FisheyeOldestTrackIdNewestPredictedMeanY);
                    }
                }
            }

            
        }
        numofframes = (numofframes + 1) % (2147483647); //+1..
  

    }


    void add_track_detection(const auto &tracker, std::vector<bob_interfaces::msg::TrackDetection> &detection_array)
    {
        auto bbox = tracker.get_bbox();
        vision_msgs::msg::BoundingBox2D bbox_msg;
        bbox_msg.center.position.x = bbox.x + bbox.width / 2;
        bbox_msg.center.position.y = bbox.y + bbox.height / 2;
        bbox_msg.size_x = bbox.width;
        bbox_msg.size_y = bbox.height;

        bob_interfaces::msg::TrackDetection detect_msg;
        detect_msg.id = tracker.get_id();
        detect_msg.state = (int)tracker.get_tracking_state();
        detect_msg.bbox = bbox_msg;

        auto result = tracker.get_ellipse(); 
        double semi_major_axis = std::get<0>(result);
        double semi_minor_axis = std::get<1>(result);
        double orientation = std::get<2>(result);
        detect_msg.covariance_ellipse_semi_major_axis = semi_major_axis;
        detect_msg.covariance_ellipse_semi_minor_axis = semi_minor_axis;
        detect_msg.covariance_ellipse_orientation = orientation;

        detection_array.push_back(detect_msg);
    }

    void add_trajectory_detection(const auto &tracker, std::vector<bob_interfaces::msg::TrackTrajectory> &trajectory_array)
    {

        bob_interfaces::msg::TrackTrajectory track_msg;
        track_msg.id = std::to_string(tracker.get_id()) + std::string("-") + std::to_string(tracker.get_tracking_state());

        for (const auto &center_point : tracker.get_center_points())
        {
            bob_interfaces::msg::TrackPoint point;
            point.center.x = center_point.first.x;
            point.center.y = center_point.first.y;
            point.tracking_state = (int)center_point.second;
            track_msg.trajectory.push_back(point);
        }

        trajectory_array.push_back(track_msg);
    }

    void add_prediction(const auto &tracker, std::vector<bob_interfaces::msg::TrackTrajectory> &prediction_array)
    {
        bob_interfaces::msg::TrackTrajectory track_msg;
        track_msg.id = std::to_string(tracker.get_id()) + std::string("-") + std::to_string(tracker.get_tracking_state());

        for (const auto &center_point : tracker.get_predictor_center_points())
        {
            bob_interfaces::msg::TrackPoint point;
            point.center.x = center_point.x;
            point.center.y = center_point.y;
            track_msg.trajectory.push_back(point); 
        }

        prediction_array.push_back(track_msg);
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