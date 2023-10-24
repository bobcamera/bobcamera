#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <visibility_control.h>
#include <image_transport/image_transport.hpp>
#include "rclcpp_components/register_node_macro.hpp"
#include "parameter_node.hpp"
#include "image_utils.hpp"

class IPCamera : public ParameterNode {
public:
    COMPOSITION_PUBLIC
    explicit IPCamera(const rclcpp::NodeOptions & options)
        : ParameterNode("ipcamera_node", options),
          cinfo_manager_(nullptr),
          camera_calibration_file_param_(""),
          //image_publisher_(nullptr),
          pub_qos_profile_(10),
          source_(""),
          width_(640),
          height_(480),
          frame_id_(0) {

        RCLCPP_INFO(get_logger(), "namespace: %s", get_namespace());
        RCLCPP_INFO(get_logger(), "name: %s", get_name());
        RCLCPP_INFO(get_logger(), "middleware: %s", rmw_get_implementation_identifier());

        pub_qos_profile_.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        pub_qos_profile_.durability(rclcpp::DurabilityPolicy::Volatile);
        pub_qos_profile_.history(rclcpp::HistoryPolicy::KeepLast);

        initialize_parameters();
        configure();

        timer_ = create_wall_timer(std::chrono::milliseconds(10), std::bind(&IPCamera::timer_callback, this));
		image_publisher_ = image_transport::create_camera_publisher(this, "~/image_raw", pub_qos_profile_.get_rmw_qos_profile());

		startCapture();
    }

	~IPCamera()
	{
		stopCapture();
	}
private:
    std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_manager_;
    std::string camera_calibration_file_param_;
	image_transport::CameraPublisher image_publisher_;
    rclcpp::QoS pub_qos_profile_;
    rclcpp::TimerBase::SharedPtr timer_;

    cv::VideoCapture cap_;
    std::string source_;
    int width_;
    int height_;
    size_t frame_id_;
	std::thread      mCaptureThread;
	std::mutex       mFrameMutex;
	bool             mRun;
	cv::Mat          mCurrentFrame;

	void captureLoop()
	{
		while(mRun)
		{
			cv::Mat frame;
			cap_ >> frame;
			{
				const std::lock_guard<std::mutex> lock(mFrameMutex);
				mCurrentFrame = frame;
			}
			std::this_thread::yield(); // Allow other threads to run
		}
	}

	void startCapture()
	{
		mRun = true;
		mCaptureThread = std::thread(&IPCamera::captureLoop, this);
	}

	void stopCapture()
	{
		mRun = false;
		if (mCaptureThread.joinable())
			mCaptureThread.join();
	}

	void initialize_parameters()
	{
		rcl_interfaces::msg::ParameterDescriptor rtsp_uri_descriptor;
		rtsp_uri_descriptor.name = "rtsp_uri";
		rtsp_uri_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
		rtsp_uri_descriptor.description = "RTSP URI of the IP camera.";
		rtsp_uri_descriptor.additional_constraints = "Should be of the form 'rtsp://";
		declare_parameter("rtsp_uri", "", rtsp_uri_descriptor);

		rcl_interfaces::msg::ParameterDescriptor camera_calibration_file_descriptor;
		camera_calibration_file_descriptor.name = "camera_calibration_file";
		camera_calibration_file_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
		declare_parameter("camera_calibration_file", "", camera_calibration_file_descriptor);

		rcl_interfaces::msg::ParameterDescriptor image_width_descriptor;
		image_width_descriptor.name = "image_width";
		image_width_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
		declare_parameter("image_width", 640, image_width_descriptor);

		rcl_interfaces::msg::ParameterDescriptor image_height_descriptor;
		image_height_descriptor.name = "image_height";
		image_height_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
		declare_parameter("image_height", 480, image_height_descriptor);

		frame_id_ = 0;
	}

	void configure()
	{
		rclcpp::Logger node_logger = get_logger();

		get_parameter<std::string>("rtsp_uri", source_);
		RCLCPP_INFO(node_logger, "rtsp_uri: %s", source_.c_str());

		get_parameter<std::string>("camera_calibration_file", camera_calibration_file_param_);
		RCLCPP_INFO(node_logger, "camera_calibration_file: %s", camera_calibration_file_param_.c_str());

		get_parameter<int>("image_width", width_);
		RCLCPP_INFO(node_logger, "image_width: %d", width_);

		get_parameter<int>("image_height", height_);
		RCLCPP_INFO(node_logger, "image_height: %d", height_);

		cap_.open(source_);

		cap_.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(width_));
		cap_.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(height_));

		if (!cap_.isOpened()) {
			RCLCPP_ERROR(node_logger, "Could not open video stream with URI: %s", source_.c_str());
			throw std::runtime_error("Could not open video stream with URI: " + source_);
		}

		cinfo_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(this);
		if (cinfo_manager_->validateURL(camera_calibration_file_param_)) {
			cinfo_manager_->loadCameraInfo(camera_calibration_file_param_);
		} else {
			RCLCPP_WARN(node_logger, "CameraInfo URL not valid.");
			RCLCPP_WARN(node_logger, "URL IS %s", camera_calibration_file_param_.c_str());
		}
	}

	void timer_callback()
	{
		try
        {
			rclcpp::Logger node_logger = get_logger();
			cv::Mat frame;
			{
				const std::lock_guard<std::mutex> lock(mFrameMutex);
				frame = mCurrentFrame.clone();
			}

			if (!frame.empty())
			{
				std_msgs::msg::Header header;
				header.stamp = get_clock()->now();
				header.frame_id = std::to_string(frame_id_); 

				auto image_msg = cv_bridge::CvImage(header, frame.channels() == 1 ? sensor_msgs::image_encodings::MONO8 : sensor_msgs::image_encodings::BGR8, frame).toImageMsg();
				auto camera_info_msg = std::make_shared<sensor_msgs::msg::CameraInfo>(cinfo_manager_->getCameraInfo());
				image_publisher_.publish(std::move(image_msg), camera_info_msg);

				++frame_id_;
			}
			else 
			{
				RCLCPP_WARN(node_logger, "Received an empty frame from the camera source: %s", source_.c_str());
			}
		}
        catch (cv::Exception &cve)
        {
            RCLCPP_ERROR(get_logger(), "Open CV exception: %s", cve.what());
        } 		
	}

	std::string mat_type2encoding(int mat_type)
	{
		switch (mat_type) {
		case CV_8UC1:
			return "mono8";
		case CV_8UC3:
			return "bgr8";
		case CV_16SC1:
			return "mono16";
		case CV_8UC4:
			return "rgba8";
		default:
			throw std::runtime_error("Unsupported encoding type");
		}
	}

	void convert_frame_to_message(const cv::Mat & frame, size_t frame_id, sensor_msgs::msg::Image & msg, sensor_msgs::msg::CameraInfo & camera_info_msg)
	{
		// copy cv information into ros message
		msg.height = frame.rows;
		msg.width = frame.cols;
		msg.encoding = mat_type2encoding(frame.type());
		msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
		size_t size = frame.step * frame.rows;
		msg.data.resize(size);
		memcpy(&msg.data[0], frame.data, size);

		rclcpp::Time timestamp = get_clock()->now();

		msg.header.frame_id = std::to_string(frame_id);
		msg.header.stamp = timestamp;
		camera_info_msg.header.frame_id = std::to_string(frame_id);
		camera_info_msg.header.stamp = timestamp;
	}
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IPCamera>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}

RCLCPP_COMPONENTS_REGISTER_NODE(IPCamera)