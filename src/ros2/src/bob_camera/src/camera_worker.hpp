#pragma once

#include <filesystem>
#include <thread>

#include <boblib/api/video/VideoReader.hpp>
#include <boblib/api/base/SynchronizedQueue.hpp>

#include <mask_worker.hpp>
#include <circuit_breaker.hpp>
#include <video_recorder.hpp>

#include <parameter_node.hpp>

#include "camera_worker_params.hpp"
#include "object_simulator.hpp"
#include "publish_image.hpp"

class CameraWorker final
{
public:
    explicit CameraWorker(ParameterNode &node,
                          CameraWorkerParams &params,
                          const std::function<void(float fps_, const std_msgs::msg::Header &, const boblib::base::Image &)> &user_callback = nullptr)
        : node_(node),
          params_(params),
          user_callback_(user_callback)
    {
        mask_worker_ptr_ = std::make_unique<MaskWorker>(node_, 
            [this](MaskWorker::MaskCheckType detection_mask_result, const cv::Mat & mask)
            {
                mask_timer_callback(detection_mask_result, mask);
            });
    }

    ~CameraWorker()
    {
        stop_capture();
    }

    void init()
    {
        try
        {
            current_video_idx_ = 0;
            run_ = false;
            fps_ = UNKNOWN_DEVICE_FPS;
            mask_enabled_ = false;
            is_open_ = false;
            is_camera_info_auto_ = false;

            camera_info_msg_.frame_height = 0;
            camera_info_msg_.frame_width = 0;
            camera_info_msg_.fps = 0;

            using_cuda_ = params_.get_use_cuda() ? boblib::base::Utils::has_cuda() : false;
            privacy_mask_ptr_ = std::make_unique<boblib::base::Image>(using_cuda_);

            circuit_breaker_ptr_ = std::make_unique<CircuitBreaker>(CIRCUIT_BREAKER_MAX_RETRIES, CIRCUIT_BREAKER_INITIAL_TIMEOUT, CIRCUIT_BREAKER_MAX_TIMEOUT);

            if (params_.get_simulator_enable())
            {
                object_simulator_ptr_ = std::make_unique<ObjectSimulator>(params_.get_simulator_num_objects());
            }

            image_queue_ptr_ = std::make_unique<SynchronizedQueue<boblib::base::Image>>(params_.get_max_queue_process_size());
            publish_queue_ptr_ = std::make_unique<SynchronizedQueue<PublishImage>>();
            record_queue_ptr_ = std::make_unique<SynchronizedQueue<boblib::base::Image>>(params_.get_max_queue_process_size());

            mask_worker_ptr_->init(params_.get_mask_timer_seconds(), params_.get_mask_filename());

            is_initialized_ = true;
 
            open_camera();
            start_capture();
        }
        catch (const std::exception& e)
        {
            node_.log_error("camera_worker: init: Exception: %s", e.what());
            throw;
        }
        catch (...) 
        {
            node_.log_error("camera_worker: init: Unknown exception");
            throw;
        }
    }

    void restart_mask()
    {
        if (mask_worker_ptr_ && mask_worker_ptr_->is_running())
        {
            mask_worker_ptr_->init(params_.get_mask_timer_seconds(), params_.get_mask_filename());
        }
    }

    [[nodiscard]] bool is_initialized() const noexcept
    {
        return is_initialized_;
    }

    [[nodiscard]] bool is_open() const noexcept
    {
        return is_open_;
    }

    bool open_camera()
    {
        if (!is_initialized())
        {
            return false;
        }
        try
        {
            switch (params_.get_source_type())
            {
                case CameraWorkerParams::SourceType::USB_CAMERA:
                {
                    node_.log_send_info("Trying to open camera %d", params_.get_camera_id());
                    video_reader_ptr_ = std::make_unique<boblib::video::VideoReader>(params_.get_camera_id());
                    set_camera_resolution();
                }
                break;

                case CameraWorkerParams::SourceType::VIDEO_FILE:
                {
                    const auto & video_path = params_.get_videos()[current_video_idx_];
                    node_.log_send_info("Trying to open video '%s'", video_path.c_str());
                    video_reader_ptr_ = std::make_unique<boblib::video::VideoReader>(video_path, using_cuda_);
                }
                break;

                case CameraWorkerParams::SourceType::RTSP_STREAM:
                {
                    node_.log_send_info("Trying to open RTSP Stream '%s'", params_.get_rtsp_uri().c_str());
                    video_reader_ptr_ = std::make_unique<boblib::video::VideoReader>(params_.get_rtsp_uri(), using_cuda_);
                }
                break;

                default:
                    node_.log_send_error("Unknown SOURCE_TYPE");
                    return false;
            }

            if (video_reader_ptr_->using_cuda())
            {
                node_.log_send_info("Using CUDA for decoding.");
            }

            is_open_ = video_reader_ptr_->is_open();
            if (is_open_)
            {
                node_.log_send_info("Stream is open.");
                update_capture_info();
                last_camera_connect_ = node_.now();
                initial_camera_connect_ = (initial_camera_connect_.seconds() == 0 && initial_camera_connect_.nanoseconds() == 0) ? last_camera_connect_ : initial_camera_connect_;
            }
            else
            {
                node_.log_send_error("Could not open stream.");
                video_reader_ptr_.reset();
            }

            return is_open_;
        }
        catch (const std::exception & ex)
        {
            node_.log_error("open_camera: exception %s", ex.what());
        }
        catch (...)
        {
            node_.log_error("open_camera: unknown exception");
        }

        video_reader_ptr_.reset();
        return false;
    }

    void recording_event(const bob_interfaces::msg::RecordingEvent & event)
    {
        last_recording_event_ = event;
    }

private:
    void update_capture_info()
    {
        if (is_open_)
        {
            double fps;
            double frame_width;
            double frame_height;
            fps_ = video_reader_ptr_->get(cv::CAP_PROP_FPS, fps) ? static_cast<float>(fps) : UNKNOWN_DEVICE_FPS;
            const int cv_camera_width = video_reader_ptr_->get(cv::CAP_PROP_FRAME_WIDTH, frame_width) ? static_cast<int>(frame_width) : 0;
            const int cv_camera_height = video_reader_ptr_->get(cv::CAP_PROP_FRAME_HEIGHT, frame_height) ? static_cast<int>(frame_height) : 0;
            node_.log_send_info("Stream capture Info: %dx%d at %.2g FPS", cv_camera_width, cv_camera_height, fps_);
            loop_rate_ptr_ = std::make_unique<rclcpp::WallRate>(fps_);

            video_recorder_ptr_ = std::make_unique<VideoRecorder>(static_cast<int>(std::ceil(fps_)) * params_.get_recording_seconds_save());

            create_camera_info_msg();
        }
        else
        {
            loop_rate_ptr_ = std::make_unique<rclcpp::WallRate>(UNKNOWN_DEVICE_FPS);
        }        
    }

    bool acquire_image(boblib::base::Image & camera_img)
    {
        try
        {
            if (!circuit_breaker_ptr_->allow_request()) 
            {
                node_.log_send_error("Could not acquire image, Waiting to connect to camera");
                std::this_thread::sleep_for(std::chrono::milliseconds(CIRCUIT_BREAKER_SLEEP_MS));
                return false;
            }

            if ((video_reader_ptr_ == nullptr) || !video_reader_ptr_->read(camera_img))
            {
                if (params_.get_source_type() == CameraWorkerParams::SourceType::VIDEO_FILE)
                {
                    current_video_idx_ = current_video_idx_ >= (params_.get_videos().size() - 1) ? 0 : current_video_idx_ + 1;
                    if (!open_camera())
                    {
                        circuit_breaker_ptr_->record_failure();
                    }
                }
                else if (!open_camera())
                {
                    circuit_breaker_ptr_->record_failure();
                }
                return false;
            }
            circuit_breaker_ptr_->record_success();
            return true;
        }
        catch(const std::exception & ex)
        {
            node_.log_error("create_video_capture: Exception %s", ex.what());
            return false;
        }
        catch (...)
        {
            node_.log_error("create_video_capture: Unknown exception");
            return false;
        }
    }

    void capture_loop()
    {
        boblib::base::Image camera_img(using_cuda_);

        while (run_ && rclcpp::ok())
        {
            if (!is_initialized())
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(INITIALIZED_SLEEP_MS));
                continue;
            }

            try
            {
                // This block will acquire the image apply the simulation and then the Privacy mask
                if (!acquire_image(camera_img))
                {
                    continue;
                }

                if (params_.get_simulator_enable())
                {
                    object_simulator_ptr_->move(camera_img);
                }

                apply_mask(camera_img);

                // Pushing the image to the processing queue
                image_queue_ptr_->push(std::move(camera_img));

                // Only limiting fps if it is video and the limit_fps param is set
                if (params_.get_limit_fps() && (params_.get_source_type() == CameraWorkerParams::SourceType::VIDEO_FILE))
                {
                    loop_rate_ptr_->sleep();
                }
            }
            catch (const std::exception & e)
            {
                node_.log_send_error("CameraWorker: capture_loop: Exception: %s", e.what());
                rcutils_reset_error();
            }
            catch (...) 
            {
                node_.log_send_error("CameraWorker: capture_loop: Unknown exception");
                rcutils_reset_error();
            }
        }
        node_.log_send_info("CameraWorker: Leaving capture_loop");

        boblib::base::Utils::reset_cuda();
    }

    void process_images()
    {
        sensor_msgs::msg::Image camera_msg;

        while (rclcpp::ok())
        {
            boblib::base::Image camera_img(using_cuda_);
            if (image_queue_ptr_->pop_move(camera_img))
            {
                try
                {
                    if (image_queue_ptr_->size() > 0)
                    {
                        node_.log_info("Process Queue size: %d", image_queue_ptr_->size());
                    }

                    fill_header(camera_msg, camera_img);

                    if (user_callback_)
                    {
                        user_callback_(fps_, camera_msg.header, camera_img);
                    }

                    publish_queue_ptr_->push(PublishImage(std::move(camera_msg), std::move(camera_img)));
                }
                catch (const std::exception & e)
                {
                    node_.log_send_error("CameraWorker: process_images: Exception: %s", e.what());
                    rcutils_reset_error();
                }
                catch (...)
                {
                    node_.log_send_error("CameraWorker: process_images: Unknown exception");
                    rcutils_reset_error();
                }
            }
        }
        node_.log_send_info("CameraWorker: Leaving process_images");
    }

    void publish_images()
    {
        while (rclcpp::ok())
        {
            auto publish_image = publish_queue_ptr_->pop();
            if (publish_image.has_value())
            {
                try
                {
                    if (publish_queue_ptr_->size() > 0)
                    {
                        node_.log_info("Publish Queue size: %d", publish_queue_ptr_->size());
                    }

                    auto & camera_msg = publish_image.value().Image_msg;
                    auto & camera_img = publish_image.value().Image;

                    publish_frame(camera_msg, camera_img);

                    publish_resized_frame(camera_msg, camera_img);

                    publish_image_info(camera_msg, camera_img);

                    publish_camera_info(camera_msg.header, camera_img);

                    // Pushing the image to the saving queue
                    record_queue_ptr_->push(std::move(camera_img));
                }
                catch (const std::exception & e)
                {
                    node_.log_send_error("CameraWorker: publish_images: Exception: %s", e.what());
                    rcutils_reset_error();
                }
                catch (...)
                {
                    node_.log_send_error("CameraWorker: publish_images: Unknown exception");
                    rcutils_reset_error();
                }
            }
        }
        node_.log_send_info("CameraWorker: Leaving publish_images");
    }

    void record_images()
    {
        while (rclcpp::ok())
        {
            boblib::base::Image camera_img(using_cuda_);
            if (record_queue_ptr_->pop_move(camera_img))
            {
                try
                {
                    if (record_queue_ptr_->size() > 0)
                    {
                        node_.log_info("Record Queue size: %d", record_queue_ptr_->size());
                    }

                    if (params_.get_recording_enabled())
                    {
                        if (last_recording_event_.recording)
                        {
                            if (!video_recorder_ptr_->is_recording())
                            {
                                const auto complete_filename = last_recording_event_.recording_path + "/" + last_recording_event_.filename + ".mp4";
                                node_.log_info("Opening new video: %s", complete_filename.c_str());
                                if (!video_recorder_ptr_->open_new_video(complete_filename, params_.get_recording_codec(), fps_, camera_img.size()))
                                {
                                    node_.log_info("Could not create new video");
                                }
                            }

                            if (video_recorder_ptr_->is_recording())
                            {
                                video_recorder_ptr_->write_frame(std::move(camera_img));
                            }
                        }
                        else
                        {
                            if (video_recorder_ptr_->is_recording())
                            {
                                node_.log_info("Closing video");
                                video_recorder_ptr_->close_video();
                            }
                            video_recorder_ptr_->add_to_pre_buffer(std::move(camera_img));
                        }
                    }
                }
                catch (const std::exception & e)
                {
                    node_.log_send_error("CameraWorker: process_images: Exception: %s", e.what());
                    rcutils_reset_error();
                }
                catch (...)
                {
                    node_.log_send_error("CameraWorker: process_images: Unknown exception");
                    rcutils_reset_error();
                }
            }
        }
        node_.log_send_info("CameraWorker: Leaving record_images");

        if (video_recorder_ptr_ &&  video_recorder_ptr_->is_recording())
        {
            node_.log_info("Closing video");
            video_recorder_ptr_->close_video();
        }

        stop_capture();
    }

    inline void fill_header(sensor_msgs::msg::Image & camera_msg, boblib::base::Image & camera_img)
    {
        camera_msg.header.stamp = node_.now();
        camera_msg.header.frame_id = ParameterNode::generate_uuid();

        camera_msg.height = camera_img.size().height;
        camera_msg.width = camera_img.size().width;
        camera_msg.encoding = ImageUtils::type_to_encoding(camera_img.type());
        camera_msg.is_bigendian = (rcpputils::endian::native == rcpputils::endian::big);
        camera_msg.step = camera_img.size().width * camera_img.elemSize();
    }

    inline void apply_mask(boblib::base::Image & img)
    {
        if (!mask_enabled_ || !params_.get_mask_enable_override())
        {
            return;
        }
        img.apply_mask(*privacy_mask_ptr_);
    }

    inline void publish_frame(sensor_msgs::msg::Image & camera_msg, boblib::base::Image & camera_img)
    {
        if (node_.count_subscribers(params_.get_image_publisher()->get_topic_name()) <= 0)
        {
            return;
        }
        const size_t totalBytes = camera_img.total() * camera_img.elemSize();
        camera_msg.data.assign(camera_img.data(), camera_img.data() + totalBytes);;        

        params_.get_image_publisher()->publish(camera_msg);
    }

    void publish_resized_frame(sensor_msgs::msg::Image & camera_msg, const boblib::base::Image & camera_img) const
    {
        if (!params_.get_image_resized_publisher()
            || (params_.get_resize_height() <= 0)
            || (node_.count_subscribers(params_.get_image_resized_publisher()->get_topic_name()) <= 0))
        {
            return;
        }
        
        boblib::base::Image resized_img(using_cuda_);
        if (params_.get_resize_height() > 0)
        {
            const auto frame_width = static_cast<int>(camera_img.size().aspectRatio() * static_cast<double>(params_.get_resize_height()));
            camera_img.resizeTo(resized_img, cv::Size(frame_width, params_.get_resize_height()));
        }

        auto resized_frame_msg = cv_bridge::CvImage(camera_msg.header, camera_msg.encoding, resized_img.toMat()).toImageMsg();
        params_.get_image_resized_publisher()->publish(*resized_frame_msg);            
    }

    void start_capture()
    {
        run_ = true;
        processing_thread_ = std::jthread(&CameraWorker::process_images, this);
        capture_thread_ = std::jthread(&CameraWorker::capture_loop, this);
        publish_thread_ = std::jthread(&CameraWorker::publish_images, this);
        record_thread_ = std::jthread(&CameraWorker::record_images, this);
    }

    void stop_capture()
    {
        run_ = false;

        if (capture_thread_.joinable())
        {
            capture_thread_.join();
        }
        if (processing_thread_.joinable())
        {
            processing_thread_.join();
        }
        if (publish_thread_.joinable())
        {
            publish_thread_.join();
        }
        if (record_thread_.joinable())
        {
            record_thread_.join();
        }
    }

    bool set_camera_resolution()
    {
        static const std::vector<std::pair<int, int>> resolutions = 
        {
            {3840, 2160},  // 4K UHD
            {2560, 1440}, // QHD, WQHD, 2K
            {1920, 1080}, // Full HD
            {1600, 900},  // HD+
            {1280, 720},  // HD
            {1024, 768},  // XGA
            {800, 600},   // SVGA
            {640, 480},   // VGA
            {320, 240}   // QVGA
        };

        auto set_check_resolution = [this](long width, long height)
        {
            double width_double;
            double height_double;
            video_reader_ptr_->set(cv::CAP_PROP_FRAME_WIDTH, width);
            video_reader_ptr_->set(cv::CAP_PROP_FRAME_HEIGHT, height);
            long cur_video_width = static_cast<long>(video_reader_ptr_->get(cv::CAP_PROP_FRAME_WIDTH, width_double) ? width_double : -1);
            long cur_video_height = static_cast<long>(video_reader_ptr_->get(cv::CAP_PROP_FRAME_HEIGHT, height_double) ? height_double : -1);

            return (cur_video_width == width) && (cur_video_height == height);
        };

        // If we have the values defined, try setting
        if ((params_.get_usb_resolution().size() == 2) 
            && (set_check_resolution(params_.get_usb_resolution()[0], params_.get_usb_resolution()[1])))
        {
            return true;
        }

        // If not, we set the highest resolution
        for (const auto & [width, height] : resolutions)
        {
            if (set_check_resolution(width, height))
            {
                node_.log_info("Setting resolution for %d x %d", width, height);
                return true;
            }
        }

        return false;
    }

    void publish_image_info(sensor_msgs::msg::Image & camera_msg, const boblib::base::Image & camera_img) const
    {
        if (!params_.get_image_info_publisher() || (node_.count_subscribers(params_.get_image_info_publisher()->get_topic_name()) <= 0))
        {
            return;
        }

        bob_camera::msg::ImageInfo image_info_msg;
        image_info_msg.header = camera_msg.header;

        image_info_msg.roi.start_x = 0;
        image_info_msg.roi.start_y = 0;
        image_info_msg.roi.width = camera_img.size().width;
        image_info_msg.roi.height = camera_img.size().height;
        image_info_msg.bpp = camera_img.elemSize1() == 1 ? 8 : 16;
        image_info_msg.exposure = 0;
        image_info_msg.gain = 0;
        image_info_msg.offset = 0;
        image_info_msg.white_balance.r = 0;
        image_info_msg.white_balance.g = 0;
        image_info_msg.white_balance.b = 0;
        image_info_msg.contrast = 0;
        image_info_msg.brightness = 0;
        image_info_msg.gamma = 1.0;
        image_info_msg.channels = camera_img.channels();
        image_info_msg.bin_mode = 1;
        image_info_msg.current_temp = 0;
        image_info_msg.cool_enabled = false;
        image_info_msg.target_temp = 0;
        image_info_msg.auto_exposure = false;

        params_.get_image_info_publisher()->publish(image_info_msg);
    }

    void publish_camera_info(const std_msgs::msg::Header & header, const boblib::base::Image & camera_img)
    {
        if (!params_.get_camera_info_publisher() || (node_.count_subscribers(params_.get_camera_info_publisher()->get_topic_name()) <= 0))
        {
            return;
        }

        camera_info_msg_.header = header;
        camera_info_msg_.fps = static_cast<float>(fps_);
        camera_info_msg_.frame_width = camera_img.size().width;
        camera_info_msg_.frame_height = camera_img.size().height;
        camera_info_msg_.is_color = camera_img.channels() >= 3;
        camera_info_msg_.initial_connection = initial_camera_connect_;
        camera_info_msg_.last_connection = last_camera_connect_;
        params_.get_camera_info_publisher()->publish(camera_info_msg_);
    }

    void request_camera_settings(const std::string_view & host, int port, const std::string_view & user, const std::string_view & password,
                                const std::function<void(const bob_interfaces::srv::CameraSettings::Response::SharedPtr &)> & user_callback) const
    {
        if (host.empty() || port == 0)
        {
            return;
        }
        auto request = std::make_shared<bob_interfaces::srv::CameraSettings::Request>();
        request->host = host;
        request->port = port;
        request->user = user;
        request->password = password;

        auto response_received_callback = [user_callback](rclcpp::Client<bob_interfaces::srv::CameraSettings>::SharedFuture future) 
        {
            auto response = future.get();
            if (response->success)
            {
                user_callback(response);  
            }
        };

        params_.get_camera_settings_client()->async_send_request(request, response_received_callback);
    }

    void create_camera_info_msg()
    {
        is_camera_info_auto_ = false;
        if (params_.get_source_type() == CameraWorkerParams::SourceType::RTSP_STREAM)
        {
            auto update_camera_info = [this](const bob_interfaces::srv::CameraSettings::Response::SharedPtr& response) 
            {
                camera_info_msg_.id = response->hardware_id;
                camera_info_msg_.manufacturer = response->manufacturer;
                camera_info_msg_.model = response->model;
                camera_info_msg_.serial_num = response->serial_number;
                camera_info_msg_.firmware_version = response->firmware_version;
                camera_info_msg_.num_configurations = response->num_configurations;
                camera_info_msg_.encoding = response->encoding;
                camera_info_msg_.frame_width = response->frame_width;
                camera_info_msg_.frame_height = response->frame_height;
                camera_info_msg_.fps = response->fps;
                is_camera_info_auto_ = true;
                loop_rate_ptr_ = std::make_unique<rclcpp::WallRate>(response->fps);
            };
            request_camera_settings(params_.get_onvif_host(), params_.get_onvif_port(), params_.get_onvif_user(), params_.get_onvif_password(), update_camera_info);
        } 
        else if (params_.get_source_type() == CameraWorkerParams::SourceType::USB_CAMERA) 
        {
            camera_info_msg_.model = "USB Camera";
        } 
        else if (params_.get_source_type() == CameraWorkerParams::SourceType::VIDEO_FILE) 
        {
            camera_info_msg_.model = "Video File";
        }
    }

    void mask_timer_callback(MaskWorker::MaskCheckType detection_mask_result, const cv::Mat & mask)
    {
        if (detection_mask_result == MaskWorker::MaskCheckType::Enable)
        {
            if (!mask_enabled_)
            {
                mask_enabled_ = true;
                node_.log_send_info("CameraWorker: Privacy Mask Enabled.");
            }
            else
            {
                node_.log_send_info("CameraWorker: Privacy Mask Changed.");
            }
            privacy_mask_ptr_->create(mask);
        }
        else if ((detection_mask_result == MaskWorker::MaskCheckType::Disable) && mask_enabled_)
        {
            node_.log_send_info("CameraWorker: Privacy Mask Disabled.");
            mask_enabled_ = false;
            privacy_mask_ptr_.release();
        }
    }

    static constexpr double UNKNOWN_DEVICE_FPS = 30.0;
    static constexpr int CIRCUIT_BREAKER_MAX_RETRIES = 10;
    static constexpr int CIRCUIT_BREAKER_INITIAL_TIMEOUT = 10;
    static constexpr int CIRCUIT_BREAKER_MAX_TIMEOUT = 10;
    static constexpr int CIRCUIT_BREAKER_SLEEP_MS = 1000;
    static constexpr int INITIALIZED_SLEEP_MS = 200;

    ParameterNode &node_;
    CameraWorkerParams & params_;

    std::unique_ptr<MaskWorker> mask_worker_ptr_;

    std::function<void(float fps, const std_msgs::msg::Header &, const boblib::base::Image &)> user_callback_;
    
    std::unique_ptr<boblib::video::VideoReader> video_reader_ptr_;
    bob_camera::msg::CameraInfo camera_info_msg_;

    bool run_{false};
    std::jthread capture_thread_;
    std::jthread processing_thread_;
    std::jthread publish_thread_;
    std::jthread record_thread_;

    uint32_t current_video_idx_{0};
    
    std::unique_ptr<rclcpp::WallRate> loop_rate_ptr_;
    float fps_{UNKNOWN_DEVICE_FPS};
    bool is_open_{false};
    bool is_camera_info_auto_{false};
    bool is_initialized_{false};
    std::unique_ptr<ObjectSimulator> object_simulator_ptr_;

    bool mask_enabled_{false};
    std::unique_ptr<boblib::base::Image> privacy_mask_ptr_;

    std::unique_ptr<CircuitBreaker> circuit_breaker_ptr_;

    rclcpp::Time initial_camera_connect_;
    rclcpp::Time last_camera_connect_;

    std::unique_ptr<SynchronizedQueue<boblib::base::Image>> image_queue_ptr_;
    std::unique_ptr<SynchronizedQueue<PublishImage>> publish_queue_ptr_;
    std::unique_ptr<SynchronizedQueue<boblib::base::Image>> record_queue_ptr_;

    bool using_cuda_{false};

    bob_interfaces::msg::RecordingEvent last_recording_event_;
    std::unique_ptr<VideoRecorder> video_recorder_ptr_;
};
