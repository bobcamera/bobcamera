#include <opencv2/opencv.hpp>
#include <json/json.h>
#include <filesystem>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp/experimental/executors/events_executor/events_executor.hpp>
#include <rcpputils/endian.hpp>
#include <sensor_msgs/msg/region_of_interest.hpp>
#include <bob_interfaces/srv/bgs_reset_request.hpp>
#include <bob_interfaces/srv/mask_override_request.hpp>

#include <vision_msgs/msg/bounding_box2_d_array.hpp>

#include <boblib/api/bgs/bgs.hpp>
#include <boblib/api/bgs/WeightedMovingVariance/WeightedMovingVarianceUtils.hpp>
#include <boblib/api/blobs/connectedBlobDetection.hpp>
#include <boblib/api/utils/profiler.hpp>

#include "parameter_node.hpp"
#include "image_utils.hpp"

#include <visibility_control.h>

#include "bob_interfaces/srv/bgs_reset_request.hpp"
#include "bob_interfaces/srv/sensitivity_change_request.hpp"
#include "bob_interfaces/msg/detector_state.hpp"

class Sensitivity 
{
public:
    int vibe_threshold;
    int vibe_bgSamples;
    int vibe_requiredBGSamples;
    int vibe_learningRate;
    bool wmv_enableWeight;
    bool wmv_enableThreshold;
    double wmv_threshold;
    double wmv_weight1;
    double wmv_weight2;
    double wmv_weight3;
    int blob_sizeThreshold;
    int blob_areaThreshold;
    int blob_minDistance;
    int blob_maxBlobs;
    bool median_filter;
};

class SensitivityConfig
{
public:
    std::string name;
    Sensitivity sensitivity;
};

class SensitivityConfigCollection
{
public:
    std::vector<SensitivityConfig> configs;

    // Method to parse JSON string into SensitivityConfigCollection object
    static SensitivityConfigCollection fromJsonString(const std::string& jsonString) 
    {
        SensitivityConfigCollection collection;
        Json::Value root;
        Json::Reader reader;

        if (!reader.parse(jsonString, root)) 
        {
            // Handle parse error
            throw std::invalid_argument("Failed to parse JSON");
        }

        if (!root.isArray()) 
        {
            // Handle invalid JSON format
            throw std::invalid_argument("JSON is not an array");
        }

        for (const auto& item : root) {
            SensitivityConfig config;
            config.name = item["name"].asString();
            config.sensitivity.vibe_threshold = item["sensitivity"]["vibe"]["threshold"].asInt();
            config.sensitivity.vibe_bgSamples = item["sensitivity"]["vibe"]["bgSamples"].asInt();
            config.sensitivity.vibe_requiredBGSamples = item["sensitivity"]["vibe"]["requiredBGSamples"].asInt();
            config.sensitivity.vibe_learningRate = item["sensitivity"]["vibe"]["learningRate"].asInt();
            config.sensitivity.wmv_enableWeight = item["sensitivity"]["wmv"]["enableWeight"].asBool();
            config.sensitivity.wmv_enableThreshold = item["sensitivity"]["wmv"]["enableThreshold"].asBool();
            config.sensitivity.wmv_threshold = item["sensitivity"]["vibe"]["threshold"].asDouble();
            config.sensitivity.wmv_weight1 = item["sensitivity"]["wmv"]["weight1"].asDouble();
            config.sensitivity.wmv_weight2 = item["sensitivity"]["wmv"]["weight2"].asDouble();
            config.sensitivity.wmv_weight3 = item["sensitivity"]["wmv"]["weight3"].asDouble();
            config.sensitivity.blob_sizeThreshold = item["sensitivity"]["blob"]["sizeThreshold"].asInt();
            config.sensitivity.blob_areaThreshold = item["sensitivity"]["blob"]["areaThreshold"].asInt();
            config.sensitivity.blob_minDistance = item["sensitivity"]["blob"]["minDistance"].asInt();
            config.sensitivity.blob_maxBlobs = item["sensitivity"]["blob"]["maxBlobs"].asInt();
            config.sensitivity.median_filter = item["sensitivity"]["median_filter"].asBool();

            collection.configs.push_back(config);
        }

        return collection;
    }
};

class BackgroundSubtractor2
    : public ParameterNode
{
public:
    COMPOSITION_PUBLIC
    explicit BackgroundSubtractor2(const rclcpp::NodeOptions & options)
        : ParameterNode("background_subtractor_v2_node", options)
        , enable_profiling_(false)
        , median_filter_(false)
        , pub_qos_profile_(10)
        , sub_qos_profile_(10)
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
    rclcpp::Publisher<vision_msgs::msg::BoundingBox2DArray>::SharedPtr detection_publisher_;
    rclcpp::Publisher<bob_interfaces::msg::DetectorState>::SharedPtr state_publisher_;
    rclcpp::Service<bob_interfaces::srv::BGSResetRequest>::SharedPtr bgs_reset_service_;
    rclcpp::Service<bob_interfaces::srv::SensitivityChangeRequest>::SharedPtr sensitivity_change_service_;

    BGSType bgs_type_;
    std::unique_ptr<boblib::bgs::CoreBgs> bgsPtr{nullptr};
    std::unique_ptr<boblib::blobs::ConnectedBlobDetection> blob_detector_ptr_{nullptr};
    boblib::utils::Profiler profiler_;
    bool enable_profiling_;
    bool median_filter_;
    std::string sensitivity_;
    bool ready_;

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
    rclcpp::Service<bob_interfaces::srv::MaskOverrideRequest>::SharedPtr mask_override_service_;
    cv::Mat grey_mask_;
    cv::Rect bounding_box_;
    int image_height_;
    int image_width_;

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
            std::bind(&BackgroundSubtractor2::imageCallback, this, std::placeholders::_1));
        image_publisher_ = create_publisher<sensor_msgs::msg::Image>("bob/frames/foreground_mask", pub_qos_profile_);
        detection_publisher_ = create_publisher<vision_msgs::msg::BoundingBox2DArray>("bob/detection/allsky/boundingboxes", pub_qos_profile_);
        state_publisher_ = create_publisher<bob_interfaces::msg::DetectorState>("bob/detection/detector_state", pub_qos_profile_);

        sensitivity_change_service_ = create_service<bob_interfaces::srv::SensitivityChangeRequest>("bob/bgs/sensitivity_update", 
            std::bind(&BackgroundSubtractor2::change_sensitivity_request, 
            this, 
            std::placeholders::_1, 
            std::placeholders::_2));

        mask_timer_ = create_wall_timer(std::chrono::seconds(60), std::bind(&BackgroundSubtractor2::mask_timer_callback, this));
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
            ParameterNode::ActionParam(
                rclcpp::Parameter("image_width", 0), 
                [this](const rclcpp::Parameter& param) {image_width_ = param.as_int();}
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("image_height", 0), 
                [this](const rclcpp::Parameter& param) {image_height_ = param.as_int();}
            ),                        
        };
        add_action_parameters(params);
    }

    void init_sensitivity(const std::string sensitivity)
    {
        ready_ = false;
        bool found_config = false;
        SensitivityConfig config;
        for (const auto& _config : sensitivityConfigCollection_.configs) 
        {
            if (_config.name == sensitivity)
            {
                RCLCPP_DEBUG(get_logger(), "Config found: %s", _config.name.c_str());
                config = _config;
                found_config = true;
                break;
            }
        }

        if (!found_config) 
        {
            RCLCPP_ERROR(get_logger(), "Unknown config specified: %s", sensitivity.c_str());
            throw std::invalid_argument("Unknown config");
        }

        wmv_params_ = std::make_unique<boblib::bgs::WMVParams>(config.sensitivity.wmv_enableWeight, config.sensitivity.wmv_enableThreshold, config.sensitivity.wmv_threshold, config.sensitivity.wmv_weight1, config.sensitivity.wmv_weight2, config.sensitivity.wmv_weight3);
        vibe_params_ = std::make_unique<boblib::bgs::VibeParams>(config.sensitivity.vibe_threshold, config.sensitivity.vibe_bgSamples, config.sensitivity.vibe_requiredBGSamples, config.sensitivity.vibe_learningRate);

        bgsPtr = createBGS(bgs_type_);

        blob_params_ = std::make_unique<boblib::blobs::ConnectedBlobDetectionParams>(config.sensitivity.blob_sizeThreshold, config.sensitivity.blob_areaThreshold, config.sensitivity.blob_minDistance, config.sensitivity.blob_maxBlobs);
        blob_detector_ptr_ = std::make_unique<boblib::blobs::ConnectedBlobDetection>(*blob_params_);

        median_filter_ = config.sensitivity.median_filter;
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

                if (median_filter_)
                {
                    cv::medianBlur(*ros_cv_foreground_mask_->image_ptr, *ros_cv_foreground_mask_->image_ptr, 3);
                }
                profile_stop("BGS");

                profile_start("Blob");
                bob_interfaces::msg::DetectorState state;
                state.sensitivity = sensitivity_;
                vision_msgs::msg::BoundingBox2DArray bbox2D_array;
                bbox2D_array.header = img_msg->header;
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

    void add_bboxes(vision_msgs::msg::BoundingBox2DArray &bbox2D_array, const std::vector<cv::Rect> &bboxes)
    {
        for (const auto &bbox : bboxes)
        {
            vision_msgs::msg::BoundingBox2D bbox2D;
            bbox2D.center.position.x = bbox.x + bbox.width / 2.0;
            bbox2D.center.position.y = bbox.y + bbox.height / 2.0;
            bbox2D.size_x = bbox.width;
            bbox2D.size_y = bbox.height;
            bbox2D_array.boxes.push_back(bbox2D);
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

    void change_sensitivity_request(const std::shared_ptr<bob_interfaces::srv::SensitivityChangeRequest::Request> request, 
        std::shared_ptr<bob_interfaces::srv::SensitivityChangeRequest::Response> response)
    {
        response->success = false;
        if (!request->sensitivity.empty() && request->sensitivity.length() > 0) 
        {
            if (sensitivity_ == request->sensitivity)
            {
                response->success = true;
                RCLCPP_DEBUG(get_logger(), "Ignoring sensitivity request change");
            }
            else
            {
                response->sensitivity_from = sensitivity_;
                init_sensitivity(request->sensitivity);
                sensitivity_ = request->sensitivity;   
                response->sensitivity_to = sensitivity_;
                response->success = true;
            }
        }
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

            cv::Mat mask(cv::imread(mask_filename_, cv::IMREAD_UNCHANGED));
            if (mask.empty())
            {
                if (mask_enabled_)
                {
                    RCLCPP_INFO(get_logger(), "Mask Disabled.");
                }
                mask_enabled_ = false;
                mask_timer_->reset();
                return;             
            }

            grey_mask_= mask;

            if (mask_enable_roi_)
            {
                sensor_msgs::msg::RegionOfInterest roi_msg;
                if(mask.empty())
                {
                    roi_msg.x_offset = 0;
                    roi_msg.y_offset = 0;
                    roi_msg.width = image_width_;
                    roi_msg.height = image_height_;
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

            mask_enabled_ = true;
            RCLCPP_INFO(get_logger(), "Mask Enabled.");
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
    executor.add_node(std::make_shared<BackgroundSubtractor2>(rclcpp::NodeOptions()));
    executor.spin();
    rclcpp::shutdown();
    return 0;
}

RCLCPP_COMPONENTS_REGISTER_NODE(BackgroundSubtractor2)