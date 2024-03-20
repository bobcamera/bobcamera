#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <json/json.h>

#include <vision_msgs/msg/bounding_box2_d_array.hpp>

#include <boblib/api/bgs/bgs.hpp>
#include <boblib/api/bgs/WeightedMovingVariance/WeightedMovingVarianceUtils.hpp>
#include <boblib/api/blobs/connectedBlobDetection.hpp>
#include <boblib/api/utils/profiler.hpp>

#include "parameter_node.hpp"
#include "image_utils.hpp"

#include <visibility_control.h>

#include "bob_interfaces/srv/bgs_reset_request.hpp"
#include "bob_interfaces/msg/detector_state.hpp"

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

    BGSType bgs_type_;
    std::unique_ptr<boblib::bgs::CoreBgs> bgsPtr{nullptr};
    std::unique_ptr<boblib::blobs::ConnectedBlobDetection> blob_detector_ptr_{nullptr};
    boblib::utils::Profiler profiler_;
    bool enable_profiling_;
    bool median_filter_;
    
    rclcpp::QoS pub_qos_profile_;
    rclcpp::QoS sub_qos_profile_;

    std::unique_ptr<boblib::bgs::VibeParams> vibe_params_;
    std::unique_ptr<boblib::bgs::WMVParams> wmv_params_;
    std::unique_ptr<boblib::blobs::ConnectedBlobDetectionParams> blob_params_;

    void init()
    {
        sub_qos_profile_.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        sub_qos_profile_.durability(rclcpp::DurabilityPolicy::Volatile);
        sub_qos_profile_.history(rclcpp::HistoryPolicy::KeepLast);

        pub_qos_profile_.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        pub_qos_profile_.durability(rclcpp::DurabilityPolicy::Volatile);
        pub_qos_profile_.history(rclcpp::HistoryPolicy::KeepLast);

        declare_node_parameters();

        image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
            "bob/frames/allsky/masked/detection", 
            sub_qos_profile_,
            std::bind(&BackgroundSubtractor::imageCallback, this, std::placeholders::_1)
        );
        image_publisher_ = create_publisher<sensor_msgs::msg::Image>("bob/frames/foreground_mask", pub_qos_profile_);
        detection_publisher_ = create_publisher<vision_msgs::msg::BoundingBox2DArray>("bob/detection/allsky/boundingboxes", pub_qos_profile_);
        state_publisher_ = create_publisher<bob_interfaces::msg::DetectorState>("bob/detection/detector_state", pub_qos_profile_);

        bgs_reset_service_ = create_service<bob_interfaces::srv::BGSResetRequest>(
            "bob/bgs/reset", 
            std::bind(&BackgroundSubtractor::reset_bgs_request, 
            this, 
            std::placeholders::_1, 
            std::placeholders::_2));
    }

    void declare_node_parameters()
    {
        std::vector<ParameterNode::ActionParam> params = {
            ParameterNode::ActionParam(
                rclcpp::Parameter("enable_profiling", false), 
                [this](const rclcpp::Parameter& param) {enable_profiling_ = param.as_bool();}
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("vibe_params", R"({"threshold": 50, "bgSamples": 16, "requiredBGSamples": 1, "learningRate": 2})"), 
                [this](const rclcpp::Parameter& param) 
                {
                    Json::CharReaderBuilder builder;
                    std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
                    std::string errors;
                    Json::Value jsonObj;
                    std::string jsonData = param.as_string();

                    if (reader->parse(jsonData.c_str(), jsonData.c_str() + jsonData.size(), &jsonObj, &errors)) 
                    {
                        auto threshold = jsonObj["threshold"].asInt();
                        auto bgSamples = jsonObj["bgSamples"].asInt();
                        auto requiredBGSamples = jsonObj["requiredBGSamples"].asInt();
                        auto learningRate = jsonObj["learningRate"].asInt();

                        vibe_params_ = std::make_unique<boblib::bgs::VibeParams>(threshold, bgSamples, requiredBGSamples, learningRate);

                        if (bgs_type_ == Vibe)
                        {
                            createBGS(Vibe);
                        }
                    } 
                    else 
                    {
                        RCLCPP_ERROR(get_logger(), "1. Failed to parse the JSON data: %s", errors.c_str());
                    }
                }
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("wmv_params", R"({"enableWeight": true, "enableThreshold": true, "threshold": 25.0, "weight1": 0.5, "weight2": 0.3, "weight3": 0.2})"), 
                [this](const rclcpp::Parameter& param) 
                {
                    Json::CharReaderBuilder builder;
                    std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
                    std::string errors;
                    Json::Value jsonObj;
                    std::string jsonData = param.as_string();

                    if (reader->parse(jsonData.c_str(), jsonData.c_str() + jsonData.size(), &jsonObj, &errors)) 
                    {
                        auto enableWeight = jsonObj["enableWeight"].asBool();
                        auto enableThreshold = jsonObj["enableThreshold"].asInt();
                        auto threshold = jsonObj["threshold"].asFloat();
                        auto weight1 = jsonObj["weight1"].asFloat();
                        auto weight2 = jsonObj["weight2"].asFloat();
                        auto weight3 = jsonObj["weight3"].asFloat();

                        wmv_params_ = std::make_unique<boblib::bgs::WMVParams>(enableWeight, enableThreshold, threshold, weight1, weight2, weight3);

                        if (bgs_type_ == WMV)
                        {
                            createBGS(WMV);
                        }
                    } 
                    else 
                    {
                        RCLCPP_ERROR(get_logger(), "2. Failed to parse the JSON data: %s", errors.c_str());
                    }
                }
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("bgs", "vibe"), 
                [this](const rclcpp::Parameter& param) 
                {
                    RCLCPP_INFO(get_logger(), "Setting BGS: %s", param.as_string().c_str());
                    if (param.as_string() == "vibe")
                    {
                        bgsPtr = createBGS(Vibe);
                    }
                    else
                    {
                        bgsPtr = createBGS(WMV);
                    }
                }
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("blob_params", R"({"sizeThreshold": 7, "areaThreshold": 49, "minDistance": 40, "maxBlobs": 50})"), 
                [this](const rclcpp::Parameter& param) 
                {
                    Json::CharReaderBuilder builder;
                    std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
                    std::string errors;
                    Json::Value jsonObj;
                    std::string jsonData = param.as_string();

                    if (reader->parse(jsonData.c_str(), jsonData.c_str() + jsonData.size(), &jsonObj, &errors)) 
                    {
                        auto sizeThreshold = jsonObj["sizeThreshold"].asInt();
                        auto areaThreshold = jsonObj["areaThreshold"].asInt();
                        auto minDistance = jsonObj["minDistance"].asInt();
                        auto maxBlobs = jsonObj["maxBlobs"].asInt();

                        blob_params_ = std::make_unique<boblib::blobs::ConnectedBlobDetectionParams>(sizeThreshold, areaThreshold, minDistance, maxBlobs);
                        blob_detector_ptr_ = std::make_unique<boblib::blobs::ConnectedBlobDetection>(*blob_params_);
                    } 
                    else 
                    {
                        RCLCPP_ERROR(get_logger(), "Failed to parse the JSON data: %s", errors.c_str());
                    }
                }
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("median_filter", false), 
                [this](const rclcpp::Parameter& param) {median_filter_ = param.as_bool();}
            ),
        };
        add_action_parameters(params);
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr img_msg)
    {
        try
        {
            profile_start("Frame");

            cv::Mat img;
            ImageUtils::convert_image_msg(img_msg, img, false);

            // msg->encoding = msg->encoding != sensor_msgs::image_encodings::BGR8 ? sensor_msgs::image_encodings::MONO8 : sensor_msgs::image_encodings::BGR8;
            // auto bayer_img_bridge = cv_bridge::toCvShare(msg);

            cv::Mat gray_img;
            if (img.channels() == 1)
            {
                gray_img = img;
            }
            else
            {
                cv::cvtColor(img, gray_img, cv::COLOR_BGR2GRAY);
            }

            profile_start("BGS");
            cv::Mat mask;
            bgsPtr->apply(gray_img, mask);

            if(median_filter_)
            {
                cv::medianBlur(mask, mask, 3);
            }

            auto image_msg = cv_bridge::CvImage(img_msg->header, sensor_msgs::image_encodings::MONO8, mask).toImageMsg();
            image_publisher_->publish(*image_msg);
            profile_stop("BGS");

            profile_start("Blob");
            bob_interfaces::msg::DetectorState state;
            vision_msgs::msg::BoundingBox2DArray bbox2D_array;
            bbox2D_array.header = img_msg->header;
            std::vector<cv::Rect> bboxes;

            boblib::blobs::DetectionResult det_result = blob_detector_ptr_->detect(mask, bboxes);
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

    std::unique_ptr<boblib::bgs::CoreBgs> createBGS(BGSType _type)
    {
        bgs_type_ = _type;
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
    rclcpp::executors::StaticSingleThreadedExecutor executor;
    executor.add_node(std::make_shared<BackgroundSubtractor>(rclcpp::NodeOptions()));
    executor.spin();
    rclcpp::shutdown();
    return 0;
}

RCLCPP_COMPONENTS_REGISTER_NODE(BackgroundSubtractor)