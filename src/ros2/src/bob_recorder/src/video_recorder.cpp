#include <filesystem>
#include <mutex>

#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <sensor_msgs/msg/image.hpp>
#include "bob_interfaces/msg/tracking.hpp"

#include "parameter_node.hpp"
#include "image_utils.hpp"

#include <visibility_control.h>

class VideoRecorder 
    : public ParameterNode
{
public:
    COMPOSITION_PUBLIC
    explicit VideoRecorder(const rclcpp::NodeOptions & options) 
        : ParameterNode("video_recorder", options)
        , current_state_(RecordingState::BeforeStart)
    {
        timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&VideoRecorder::init, this));
    }

private:
    enum class RecordingState {
        BeforeStart,
        BetweenEvents,
        AfterEnd
    };

    std::string video_directory_;
    std::string img_topic_;
    std::string tracking_topic_;
    double video_fps_;
    std::string codec_str_;
    std::string prefix_str_;
    int number_seconds_save_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> sub_masked_frame_;
    std::shared_ptr<message_filters::Subscriber<bob_interfaces::msg::Tracking>> sub_tracking_;
    std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, bob_interfaces::msg::Tracking>> time_synchronizer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_annotated_frame_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<cv::VideoWriter> video_writer_ptr_;
    RecordingState current_state_;
    std::mutex buffer_mutex_;
    size_t current_end_frame_;
    size_t total_pre_frames_;
    std::unique_ptr<std::deque<std::unique_ptr<cv::Mat>>> pre_buffer_ptr_;

    friend std::shared_ptr<VideoRecorder> std::make_shared<VideoRecorder>();

    void init()
    {
        RCLCPP_INFO(get_logger(), "Initializing VideoRecorder");

        timer_->cancel();

        pre_buffer_ptr_ = std::make_unique<std::deque<std::unique_ptr<cv::Mat>>>();
        video_writer_ptr_ = std::make_unique<cv::VideoWriter>();

        declare_node_parameters();

        rclcpp::QoS sub_qos_profile{10};
        sub_qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        sub_qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);
        sub_qos_profile.history(rclcpp::HistoryPolicy::KeepLast);
        auto rmw_qos_profile = sub_qos_profile.get_rmw_qos_profile();

        sub_masked_frame_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(shared_from_this(), img_topic_, rmw_qos_profile);
        sub_tracking_ = std::make_shared<message_filters::Subscriber<bob_interfaces::msg::Tracking>>(shared_from_this(), tracking_topic_, rmw_qos_profile);

        time_synchronizer_ = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, bob_interfaces::msg::Tracking>>(*sub_masked_frame_, *sub_tracking_, 10);
        time_synchronizer_->registerCallback(&VideoRecorder::callback, this);
    }

    static bool create_dir(const std::string& path) 
    {
        std::filesystem::path dirPath = path;

        if (!std::filesystem::exists(dirPath)) 
        {
            return std::filesystem::create_directory(dirPath);
        }

        return true;
    }

    void declare_node_parameters()
    {
        std::vector<ParameterNode::ActionParam> params = {
            ParameterNode::ActionParam(
                rclcpp::Parameter("video_directory", "."), 
                [this](const rclcpp::Parameter& param) 
                {
                    video_directory_ = param.as_string();
                    create_dir(video_directory_);
                }
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("img_topic", "bob/camera/all_sky/bayer"), 
                [this](const rclcpp::Parameter& param) {img_topic_ = param.as_string();}
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("tracking_topic", "bob/tracker/tracking"), 
                [this](const rclcpp::Parameter& param) {tracking_topic_ = param.as_string();}
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("prefix", "video"), 
                [this](const rclcpp::Parameter& param) {prefix_str_ = param.as_string();}
            ),            
            ParameterNode::ActionParam(
                rclcpp::Parameter("codec", "X264"), 
                [this](const rclcpp::Parameter& param) {codec_str_ = param.as_string();}
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("video_fps", 30.0), 
                [this](const rclcpp::Parameter& param) 
                {
                    video_fps_ = param.as_double();
                    total_pre_frames_ = (size_t)(number_seconds_save_ * video_fps_);
                }
            ),
            ParameterNode::ActionParam(
                rclcpp::Parameter("seconds_save", 2), 
                [this](const rclcpp::Parameter& param) 
                {
                    number_seconds_save_ = param.as_int();
                    total_pre_frames_ = (size_t)(number_seconds_save_ * video_fps_);
                }
            ),
        };
        add_action_parameters(params);
    }

    void add_to_ring_buffer(const cv::Mat& img)
    {
        if (pre_buffer_ptr_->size() >= total_pre_frames_) 
        {
            pre_buffer_ptr_->pop_front();
        }
        pre_buffer_ptr_->push_back(std::make_unique<cv::Mat>(img.clone()));
    }

    void callback(const sensor_msgs::msg::Image::SharedPtr& image_msg
                , const bob_interfaces::msg::Tracking::SharedPtr& tracking_msg)
    {
        try
        {
            std::unique_lock<std::mutex> lock(buffer_mutex_);

            cv::Mat img;
            ImageUtils::convert_image_msg(image_msg, img, true);

            switch (current_state_) 
            {
            case RecordingState::BeforeStart:
                if (tracking_msg->state.trackable > 0) 
                {
                    current_state_ = RecordingState::BetweenEvents;
                    if (open_new_video(img.size(), img.channels() == 3))
                    {
                        video_writer_ptr_->write(img);
                    }
                } 
                else 
                {
                    add_to_ring_buffer(img);
                }
                break;

            case RecordingState::BetweenEvents:
                video_writer_ptr_->write(img);
                if (tracking_msg->state.trackable == 0) 
                {
                    current_state_ = RecordingState::AfterEnd;
                    current_end_frame_ = total_pre_frames_;
                }
                break;

            case RecordingState::AfterEnd:
                if (current_end_frame_ == 0) 
                {
                    close_video();
                    current_state_ = RecordingState::BeforeStart;
                }
                else 
                {
                    video_writer_ptr_->write(img);
                    --current_end_frame_;

                    // If we receive a wake while ending, get back recording and clear pre-buffer
                    if (tracking_msg->state.trackable > 0) 
                    {
                        current_state_ = RecordingState::BetweenEvents;
                        pre_buffer_ptr_->clear();
                    }
                }
                // Always add to pre-buffer
                add_to_ring_buffer(img);
                break;
            }
        }
        catch (std::exception &e)
        {
            RCLCPP_ERROR(get_logger(), "exception: %s", e.what());
        }
    }

    static std::string generate_filename()
    {
        auto t = std::time(nullptr);
        auto tm = *std::localtime(&t);

        std::ostringstream oss;
        oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
        return oss.str();
    }

    bool open_new_video(const cv::Size& frame_size, bool is_color)
    {
        // Create the video
        const auto name = video_directory_ + "/" + prefix_str_ + generate_filename() + ".mkv";
        const int codec = cv::VideoWriter::fourcc(codec_str_[0], codec_str_[1], codec_str_[2], codec_str_[3]);
        RCLCPP_INFO(get_logger(), "new video: %s, codec: %s, fps: %g", name.c_str(), codec_str_.c_str(), video_fps_);
        if (video_writer_ptr_->open(name, codec, video_fps_, frame_size, is_color))
        {
            // Save the current pre-buffer into the video
            RCLCPP_INFO(get_logger(), "writing pre: %ld", pre_buffer_ptr_->size());
            for (const auto& img_ptr : *pre_buffer_ptr_)
            {
                video_writer_ptr_->write(*img_ptr);
            }
            pre_buffer_ptr_->clear();
            return true;
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Error creating video: %s", name.c_str());
            current_state_ = RecordingState::BeforeStart;
        }
        return false;
    }

    void close_video()
    {
        video_writer_ptr_->release();
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VideoRecorder>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}

RCLCPP_COMPONENTS_REGISTER_NODE(VideoRecorder)