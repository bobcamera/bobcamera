#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <vision_msgs/msg/bounding_box2_d_array.hpp>

#include <boblib/api/bgs/bgs.hpp>
#include <boblib/api/bgs/WeightedMovingVariance/WeightedMovingVarianceUtils.hpp>
#include <boblib/api/blobs/connectedBlobDetection.hpp>
#include <boblib/api/utils/profiler.hpp>

#include "parameter_node.hpp"
#include "image_utils.hpp"

class BackgroundSubtractor
    : public ParameterNode
{
public:
    static std::shared_ptr<BackgroundSubtractor> create()
    {
        auto result = std::shared_ptr<BackgroundSubtractor>(new BackgroundSubtractor());
        result->init();
        return result;
    }
    
private:
    enum BGSType
    {
        Vibe,
        WMV
    };

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::Publisher<vision_msgs::msg::BoundingBox2DArray>::SharedPtr detection_publisher_;

    std::unique_ptr<boblib::bgs::CoreBgs> bgsPtr{nullptr};
    boblib::blobs::ConnectedBlobDetection blob_detector_{boblib::blobs::ConnectedBlobDetectionParams(7, 49, 40, 100)};
    boblib::utils::Profiler profiler_;
    bool enable_profiling_;

    BackgroundSubtractor()
        : ParameterNode("background_subtractor_node")
        , enable_profiling_(false)
    {
    }

    void init()
    {
        // Define the QoS profile for the subscriber
        rclcpp::QoS sub_qos_profile(2);
        sub_qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        sub_qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);
        sub_qos_profile.history(rclcpp::HistoryPolicy::KeepLast);

        // Define the QoS profile for the publisher
        rclcpp::QoS pub_qos_profile(2);
        pub_qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        pub_qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);
        pub_qos_profile.history(rclcpp::HistoryPolicy::KeepLast);

        image_subscription_ = create_subscription<sensor_msgs::msg::Image>("bob/camera/all_sky/bayer", sub_qos_profile,
            std::bind(&BackgroundSubtractor::imageCallback, this, std::placeholders::_1));
        image_publisher_ = create_publisher<sensor_msgs::msg::Image>("bob/frames/all_sky/foreground_mask", pub_qos_profile);
        detection_publisher_ = create_publisher<vision_msgs::msg::BoundingBox2DArray>("bob/detector/all_sky/bounding_boxes", pub_qos_profile);

        bgsPtr = createBGS(WMV);

        declare_node_parameters();
    }

    void declare_node_parameters()
    {
        std::vector<ParameterNode::ActionParam> params = {
            ParameterNode::ActionParam(
                rclcpp::Parameter("enable_profiling", false), 
                [this](const rclcpp::Parameter& param) {enable_profiling_ = param.as_bool();}
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

            auto image_msg = cv_bridge::CvImage(img_msg->header, sensor_msgs::image_encodings::MONO8, mask).toImageMsg();
            image_publisher_->publish(*image_msg);
            profile_stop("BGS");

            profile_start("Blob");
            std::vector<cv::Rect> bboxes;
            if (blob_detector_.detect(mask, bboxes))
            {
                vision_msgs::msg::BoundingBox2DArray bbox2D_array;
                bbox2D_array.header = img_msg->header;
                add_bboxes(bbox2D_array, bboxes);

                detection_publisher_->publish(bbox2D_array);
            }
            profile_stop("Blob");

            profile_stop("Frame");
            profile_dump();
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "CV bridge exception: %s", e.what());
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

    std::unique_ptr<boblib::bgs::CoreBgs> createBGS(BGSType _type)
    {
        switch (_type)
        {
        case BGSType::Vibe:
            return std::make_unique<boblib::bgs::Vibe>(boblib::bgs::VibeParams(50, 20, 2, 4));
        case BGSType::WMV:
            return std::make_unique<boblib::bgs::WeightedMovingVariance>(boblib::bgs::WMVParams(true, true, 25.0f, 0.5f, 0.3f, 0.2f));
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
    rclcpp::spin(BackgroundSubtractor::create());
    rclcpp::shutdown();
    return 0;
}
