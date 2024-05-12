#include <opencv2/opencv.hpp>
#include <filesystem>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp/experimental/executors/events_executor/events_executor.hpp>
#include <rcpputils/endian.hpp>
#include <sensor_msgs/msg/region_of_interest.hpp>
#include <bob_interfaces/srv/bgs_reset_request.hpp>
#include <bob_interfaces/srv/mask_override_request.hpp>

#include <boblib/api/bgs/bgs.hpp>
#include <boblib/api/bgs/WeightedMovingVariance/WeightedMovingVarianceUtils.hpp>
#include <boblib/api/blobs/connectedBlobDetection.hpp>
#include <boblib/api/utils/profiler.hpp>

#include "parameter_node.hpp"
#include "image_utils.hpp"
#include "background_subtractor_companion.hpp"

#include <visibility_control.h>

#include <bob_interfaces/srv/bgs_reset_request.hpp>
#include <bob_interfaces/srv/sensitivity_change_request.hpp>
#include <bob_interfaces/msg/detector_state.hpp>
#include <bob_interfaces/msg/detector_b_box_array.hpp>


class BackgroundSubtractor
    : public ParameterNode
{
public:
    COMPOSITION_PUBLIC
    explicit BackgroundSubtractor(const rclcpp::NodeOptions & options)
        : ParameterNode("background_subtractor_node", options)
        , enable_profiling_(false)
        , median_filter_(false)
        , pub_qos_profile_(10)
        , sub_qos_profile_(10)
        , mask_enable_override_(true)
        , mask_enabled_(false)
        , mask_enable_roi_(false)        
    {
        init();
    }
    
private:
    enum BGSType
    {
        Unknown,
        Vibe,
        WMV
    };

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::Publisher<bob_interfaces::msg::DetectorBBoxArray>::SharedPtr detection_publisher_;
    rclcpp::Publisher<bob_interfaces::msg::DetectorState>::SharedPtr state_publisher_;
    rclcpp::Service<bob_interfaces::srv::BGSResetRequest>::SharedPtr bgs_reset_service_;

    BGSType bgs_type_;
    std::unique_ptr<boblib::bgs::CoreBgs> bgsPtr{nullptr};
    std::unique_ptr<boblib::blobs::ConnectedBlobDetection> blob_detector_ptr_{nullptr};
    boblib::utils::Profiler profiler_;
    bool enable_profiling_;
    bool median_filter_;
    std::string sensitivity_;
    bool ready_;
    std::string image_resized_publish_topic_;

    SensitivityConfigCollection sensitivityConfigCollection_;
    
    rclcpp::QoS pub_qos_profile_;
    rclcpp::QoS sub_qos_profile_;

    std::unique_ptr<boblib::bgs::VibeParams> vibe_params_;
    std::unique_ptr<boblib::bgs::WMVParams> wmv_params_;
    std::unique_ptr<boblib::blobs::ConnectedBlobDetectionParams> blob_params_;

    std::unique_ptr<RosCvImageMsg> ros_cv_foreground_mask_;

    rclcpp::TimerBase::SharedPtr mask_timer_;
    bool mask_enable_override_;
    bool mask_enabled_;
    bool mask_enable_roi_;
    std::string mask_filename_;
    std::optional<std::filesystem::file_time_type> mask_last_modified_time_;
    rclcpp::Client<bob_interfaces::srv::BGSResetRequest>::SharedPtr bgs_reset_client_;
    rclcpp::Publisher<sensor_msgs::msg::RegionOfInterest>::SharedPtr roi_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_resized_publisher_;
    rclcpp::Service<bob_interfaces::srv::MaskOverrideRequest>::SharedPtr mask_override_service_;
    cv::Mat grey_mask_;
    cv::Rect bounding_box_;
    int resize_height_;

    void init()
    {
        sub_qos_profile_.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        sub_qos_profile_.durability(rclcpp::DurabilityPolicy::Volatile);
        sub_qos_profile_.history(rclcpp::HistoryPolicy::KeepLast);

        pub_qos_profile_.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        pub_qos_profile_.durability(rclcpp::DurabilityPolicy::Volatile);
        pub_qos_profile_.history(rclcpp::HistoryPolicy::KeepLast);

        declare_node_parameters();
        init_sensitivity(sensitivity_);

        image_subscription_ = create_subscription<sensor_msgs::msg::Image>("bob/frames/allsky/original", sub_qos_profile_,
            std::bind(&BackgroundSubtractor::imageCallback, this, std::placeholders::_1));
        image_publisher_ = create_publisher<sensor_msgs::msg::Image>("bob/frames/foreground_mask", pub_qos_profile_);
        detection_publisher_ = create_publisher<bob_interfaces::msg::DetectorBBoxArray>("bob/detection/allsky/boundingboxes", pub_qos_profile_);        
        state_publisher_ = create_publisher<bob_interfaces::msg::DetectorState>("bob/detection/detector_state", pub_qos_profile_);

        mask_timer_ = create_wall_timer(std::chrono::seconds(60), std::bind(&BackgroundSubtractor::mask_timer_callback, this));
        mask_timer_callback(); // Calling it the first time
    }

    void declare_node_parameters()
    {
        std::vector<ParameterNode::ActionParam> params = {
            ParameterNode::ActionParam(
                rclcpp::Parameter("enable_profiling", false), 
                [this](const rclcpp::Parameter& param) {enable_profiling_ = param.as_bool();}
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("json_params", ""), 
                [this](const rclcpp::Parameter& param) 
                {
                    try {
                        sensitivityConfigCollection_ = SensitivityConfigCollection::fromJsonString(param.as_string());
                    } catch (const std::exception& e) {
                        RCLCPP_ERROR(get_logger(), "Failed to parse the JSON data: %s", e.what());
                    }
                }
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("sensitivity", "medium_c"), 
                [this](const rclcpp::Parameter& param) 
                {
                    RCLCPP_INFO(get_logger(), "Setting Sensitivity: %s", param.as_string().c_str());
                    sensitivity_ = param.as_string();
                }
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("bgs", "vibe"), 
                [this](const rclcpp::Parameter& param) 
                {
                    RCLCPP_INFO(get_logger(), "Setting BGS: %s", param.as_string().c_str());
                    if (param.as_string() == "vibe")
                    {
                        bgs_type_ = Vibe;
                    }
                    else
                    {
                        bgs_type_ = WMV;
                    }
                }
            ),            
            // MASK parameters
            ParameterNode::ActionParam(
                rclcpp::Parameter("mask_file", "mask.pgm"), 
                [this](const rclcpp::Parameter& param) {mask_filename_ = param.as_string();}
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("mask_enable_offset_correction", true), 
                [this](const rclcpp::Parameter& param) {mask_enable_roi_ = param.as_bool();}
            ),
            // Image resizing
            ParameterNode::ActionParam(
                rclcpp::Parameter("image_resized_publish_topic", "bob/frames/foreground_mask/resized"), 
                [this](const rclcpp::Parameter& param) {
                    image_resized_publish_topic_ = param.as_string();
                    if (!image_resized_publish_topic_.empty())
                    {
                        image_resized_publisher_ = create_publisher<sensor_msgs::msg::Image>(image_resized_publish_topic_, pub_qos_profile_);
                    }
                    else
                    {
                        image_resized_publisher_.reset();
                        RCLCPP_INFO(get_logger(), "Resizer topic disabled");
                    }
                }
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("resize_height", 960), 
                [this](const rclcpp::Parameter& param) {
                    resize_height_ = param.as_int();
                }
            ),        
        };
        add_action_parameters(params);
    }

    void init_sensitivity(const std::string & sensitivity)
    {
        if (sensitivity.empty() || (sensitivity.length() == 0))
        {
            RCLCPP_DEBUG(get_logger(), "Ignoring sensitivity request change");
            return;
        }
        if (!sensitivityConfigCollection_.configs.contains(sensitivity))
        {
            RCLCPP_ERROR(get_logger(), "Unknown config specified: %s", sensitivity.c_str());
            return;
        }

        ready_ = false;

        const SensitivityConfig & config = sensitivityConfigCollection_.configs.at(sensitivity);

        wmv_params_ = std::make_unique<boblib::bgs::WMVParams>(config.sensitivity.wmv_enableWeight, config.sensitivity.wmv_enableThreshold, config.sensitivity.wmv_threshold, config.sensitivity.wmv_weight1, config.sensitivity.wmv_weight2, config.sensitivity.wmv_weight3);
        vibe_params_ = std::make_unique<boblib::bgs::VibeParams>(config.sensitivity.vibe_threshold, config.sensitivity.vibe_bgSamples, config.sensitivity.vibe_requiredBGSamples, config.sensitivity.vibe_learningRate);

        bgsPtr = createBGS(bgs_type_);

        blob_params_ = std::make_unique<boblib::blobs::ConnectedBlobDetectionParams>(config.sensitivity.blob_sizeThreshold, config.sensitivity.blob_areaThreshold, config.sensitivity.blob_minDistance, config.sensitivity.blob_maxBlobs);
        blob_detector_ptr_ = std::make_unique<boblib::blobs::ConnectedBlobDetection>(*blob_params_);

        median_filter_ = config.sensitivity.median_filter;
        sensitivity_ = sensitivity;
        ready_ = true;
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr img_msg)
    {
        if (ready_)
        {
            try
            {
                profile_start("Frame");

                cv::Mat img;
                ImageUtils::convert_image_msg(img_msg, img, false);

                cv::Mat gray_img;
                if (img.channels() == 1)
                {
                    gray_img = img;
                }
                else
                {
                    cv::cvtColor(img, gray_img, cv::COLOR_BGR2GRAY);
                }

                apply_mask(gray_img);

                if (!ros_cv_foreground_mask_ || (gray_img.size() != ros_cv_foreground_mask_->image_ptr->size()))
                {
                    ros_cv_foreground_mask_ = std::make_unique<RosCvImageMsg>(gray_img, sensor_msgs::image_encodings::MONO8, false);
                }

                profile_start("BGS");
                bgsPtr->apply(gray_img, *ros_cv_foreground_mask_->image_ptr);

                image_publisher_->publish(*ros_cv_foreground_mask_->msg_ptr);

                publish_resized_frame(*ros_cv_foreground_mask_);

                if (median_filter_)
                {
                    cv::medianBlur(*ros_cv_foreground_mask_->image_ptr, *ros_cv_foreground_mask_->image_ptr, 3);
                }
                profile_stop("BGS");

                profile_start("Blob");
                bob_interfaces::msg::DetectorState state;
                state.sensitivity = sensitivity_;
                bob_interfaces::msg::DetectorBBoxArray bbox2D_array;
                bbox2D_array.header = img_msg->header;
                bbox2D_array.image_width = gray_img.size().width;
                bbox2D_array.image_height = gray_img.size().height;
                std::vector<cv::Rect> bboxes;

                boblib::blobs::DetectionResult det_result = blob_detector_ptr_->detect(*ros_cv_foreground_mask_->image_ptr, bboxes);
                if (det_result == boblib::blobs::DetectionResult::Success)
                {
                    state.max_blobs_reached = false;
                    add_bboxes(bbox2D_array, bboxes);
                }
                else if(det_result == boblib::blobs::DetectionResult::MaxBlobsReached)
                {
                    state.max_blobs_reached = true;
                    //RCLCPP_WARN(get_logger(), "Maximum blobs reached - please check detection mask");
                }
                
                state_publisher_->publish(state);
                detection_publisher_->publish(bbox2D_array);            

                profile_stop("Blob");

                profile_stop("Frame");
                profile_dump();
            }
            catch (cv_bridge::Exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "CV bridge exception: %s", e.what());
            }
            catch (cv::Exception &cve)
            {
                RCLCPP_ERROR(get_logger(), "Open CV exception: %s", cve.what());
            }
        }
    }

    void add_bboxes(bob_interfaces::msg::DetectorBBoxArray &bbox2D_array, const std::vector<cv::Rect> &bboxes)
    {
        for (const auto &bbox : bboxes)
        {
            bob_interfaces::msg::DetectorBBox bbox_msg;
            bbox_msg.x = bbox.x;
            bbox_msg.y = bbox.y;
            bbox_msg.width = bbox.width;
            bbox_msg.height = bbox.height;
            bbox2D_array.detections.push_back(bbox_msg);
        }
    }

    void reset_bgs_request(const std::shared_ptr<bob_interfaces::srv::BGSResetRequest::Request> request, 
        std::shared_ptr<bob_interfaces::srv::BGSResetRequest::Response> response)
    {
        //RCLCPP_INFO(get_logger(), "Restarting the BGS");
        if(!request->bgs_params.empty() && request->bgs_params.length() > 0) { 
            RCLCPP_INFO(get_logger(), "We have updated bgs params to apply...");
        }
        bgsPtr->restart();
        response->success = true;
        //RCLCPP_INFO(get_logger(), "Restarting BGS: SUCCESS");
    }

    std::unique_ptr<boblib::bgs::CoreBgs> createBGS(BGSType _type)
    {
        switch (_type)
        {
        case BGSType::Vibe:
            return std::make_unique<boblib::bgs::Vibe>(*vibe_params_);
        case BGSType::WMV:
            return std::make_unique<boblib::bgs::WeightedMovingVariance>(*wmv_params_);
        default:
            return nullptr;
        }
    }

    inline void roi_calculation()
    {
        if (mask_enable_roi_ && mask_enabled_)
        {
            sensor_msgs::msg::RegionOfInterest roi_msg;
            if(grey_mask_.empty())
            {
                roi_msg.x_offset = 0;
                roi_msg.y_offset = 0;
                roi_msg.width = grey_mask_.size().width;
                roi_msg.height = grey_mask_.size().height;
            }
            else
            {
                bounding_box_ = cv::Rect(grey_mask_.cols, grey_mask_.rows, 0, 0);                                
                for (int y = 0; y < grey_mask_.rows; ++y) 
                {
                    for (int x = 0; x < grey_mask_.cols; ++x) 
                    {
                        if (grey_mask_.at<uchar>(y, x) == 255) 
                        { 
                            bounding_box_.x = std::min(bounding_box_.x, x);
                            bounding_box_.y = std::min(bounding_box_.y, y);
                            bounding_box_.width = std::max(bounding_box_.width, x - bounding_box_.x);
                            bounding_box_.height = std::max(bounding_box_.height, y - bounding_box_.y);
                        }
                    }
                }

                roi_msg.x_offset = bounding_box_.x;
                roi_msg.y_offset = bounding_box_.y;
                roi_msg.width = bounding_box_.width;
                roi_msg.height = bounding_box_.height;
            }

            roi_publisher_->publish(roi_msg);

            RCLCPP_INFO(get_logger(), "Detection frame size determined from mask: %d x %d", roi_msg.width, roi_msg.height);
        }
    }

    inline void apply_mask(const cv::Mat & img)
    {
        if (!mask_enabled_ || !mask_enable_override_)
        {
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Applying mask");
        if (img.size() != grey_mask_.size())
        {
            RCLCPP_WARN(this->get_logger(), "Frame and mask dimensions do not match. Attempting resize.");
            RCLCPP_WARN(this->get_logger(), "Note: Please ensure your mask has not gone stale, you might want to recreate it.");
            cv::resize(grey_mask_, grey_mask_, img.size());
        }
        if (mask_enable_roi_)
        {
            cv::Mat image_roi = img(bounding_box_);
            cv::Mat mask_roi = grey_mask_(bounding_box_);
            cv::Mat result_roi;
            cv::bitwise_and(image_roi, image_roi, result_roi, mask_roi);
            result_roi.copyTo(img(bounding_box_));
        }
        else
        {
            cv::bitwise_and(img, img, img, grey_mask_);
        }        
    }

    void mask_timer_callback()
    {
        mask_timer_->cancel();
        try
        {
            if (!std::filesystem::exists(mask_filename_))
            {
                if (mask_enabled_)
                {
                    RCLCPP_INFO(get_logger(), "Mask Disabled.");
                }
                mask_enabled_ = false;
                mask_timer_->reset();
                return;
            }

            auto current_modified_time = std::filesystem::last_write_time(mask_filename_);
            if (mask_last_modified_time_ == current_modified_time) 
            {
                mask_timer_->reset();
                return;
            }
            mask_last_modified_time_ = current_modified_time;

            grey_mask_= cv::imread(mask_filename_, cv::IMREAD_UNCHANGED);
            mask_enabled_ = !grey_mask_.empty();
            if (grey_mask_.empty())
            {
                RCLCPP_INFO(get_logger(), "Mask Disabled, mask image was empty");
            }
            else
            {
                RCLCPP_INFO(get_logger(), "Mask Enabled.");
            }
            roi_calculation();
        }
        catch (cv::Exception &cve)
        {
            RCLCPP_ERROR(get_logger(), "Open CV exception on timer callback: %s", cve.what());
        }
        
        mask_timer_->reset();
    }

    void mask_override_request(const std::shared_ptr<bob_interfaces::srv::MaskOverrideRequest::Request> request, 
        std::shared_ptr<bob_interfaces::srv::MaskOverrideRequest::Response> response)
    {
        mask_enable_override_ = request->mask_enabled;
        if (request->mask_enabled)
        {
            RCLCPP_DEBUG(get_logger(), "Mask Override set to: True");
        }
        else
        {
            RCLCPP_DEBUG(get_logger(), "Mask Override set to: False");
        }
        response->success = true;        
    }

    inline void publish_resized_frame(const RosCvImageMsg & image_msg)
    {
        if (!image_resized_publisher_ || (count_subscribers(image_resized_publish_topic_) <= 0))
        {
            return;
        }
        // TODO: Think about replacing the resized_img by the RosCvImageMsg, has to take into consideration the resizing of the resize_height and the image
        cv::Mat resized_img;
        if (resize_height_ > 0)
        {
            const double aspect_ratio = (double)image_msg.image_ptr->size().width / (double)image_msg.image_ptr->size().height;
            const int frame_height = resize_height_;
            const int frame_width = (int)(aspect_ratio * (double)frame_height);
            cv::resize(*image_msg.image_ptr, resized_img, cv::Size(frame_width, frame_height));
        }
        else
        {
            resized_img = *image_msg.image_ptr;
        }

        auto resized_frame_msg = cv_bridge::CvImage(image_msg.msg_ptr->header, image_msg.msg_ptr->encoding, resized_img).toImageMsg();
        image_resized_publisher_->publish(*resized_frame_msg);            
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
    rclcpp::experimental::executors::EventsExecutor executor;
    executor.add_node(std::make_shared<BackgroundSubtractor>(rclcpp::NodeOptions()));
    executor.spin();
    rclcpp::shutdown();
    return 0;
}

RCLCPP_COMPONENTS_REGISTER_NODE(BackgroundSubtractor)