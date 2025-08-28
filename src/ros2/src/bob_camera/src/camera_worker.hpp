#pragma once

#include <filesystem>
#include <thread>
#include <mutex>
#include <atomic>

#include <boblib/api/video/VideoReader.hpp>
#include <boblib/api/utils/pubsub/TopicManager.hpp>
#include <boblib/api/utils/fps_tracker.hpp>

#include <mask_worker.hpp>
#include <circuit_breaker.hpp>
#include <parameter_node.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <bob_camera/msg/camera_info.hpp>
#include <bob_camera/msg/image_info.hpp>
#include <bob_interfaces/srv/camera_settings.hpp>

#include "camera_bgs_params.hpp"
#include "object_simulator.hpp"
#include "publish_image.hpp"

class CameraWorker final
{
public:
    explicit CameraWorker(ParameterNode &node
                          , CameraBgsParams &params
                          , const rclcpp::QoS &qos_publish_profile
                          , boblib::utils::pubsub::TopicManager &topic_manager
                          , boblib::utils::Profiler &profiler)
        : node_(node)
        , params_(params)
        , qos_publish_profile_(qos_publish_profile)
        , topic_manager_(topic_manager)
        , profiler_(profiler)
    {
        mask_worker_ptr_ = std::make_unique<MaskWorker>(node_,
                                                        [this](MaskWorker::MaskCheckType detection_mask_result, const cv::Mat &mask)
                                                        {
                                                            mask_timer_callback(detection_mask_result, mask);
                                                        });
    }

    ~CameraWorker() noexcept
    {
        node_.log_info("CameraWorker destructor");
        cleanup();
    }

    // Move constructor
    CameraWorker(CameraWorker&& other) noexcept = delete;
    
    // Move assignment operator
    CameraWorker& operator=(CameraWorker&& other) noexcept = delete;
    
    // Copy constructor and assignment operator are deleted
    CameraWorker(const CameraWorker&) = delete;
    CameraWorker& operator=(const CameraWorker&) = delete;

    void init()
    {
        std::lock_guard<std::recursive_mutex> lock(state_mutex_);
        
        if (is_initialized_.load()) {
            node_.log_warn("CameraWorker already initialized");
            return;
        }

        try {
            initialize_state();
            initialize_profiler_regions();
            initialize_components();
            initialize_publishers();
            initialize_pubsub();
            initialize_mask_worker();
            
            // Mark as initialized only after all initialization succeeds
            is_initialized_.store(true);
            
            // Start camera operations
            if (!open_camera()) {
                throw std::runtime_error("Failed to open camera during initialization");
            }
            start_capture();
            
            node_.log_info("CameraWorker initialized successfully");
        }
        catch (const std::exception &e) {
            cleanup_on_init_failure();
            node_.log_error("CameraWorker init failed: %s", e.what());
            throw;
        }
        catch (...) {
            cleanup_on_init_failure();
            node_.log_error("CameraWorker init failed: unknown exception");
            throw;
        }
    }

    void restart_mask()
    {
        if (mask_worker_ptr_ && mask_worker_ptr_->is_running())
        {
            mask_worker_ptr_->init(params_.camera.privacy_mask.timer_seconds, params_.camera.privacy_mask.filename);
        }
    }

    [[nodiscard]] bool is_initialized() const noexcept
    {
        return is_initialized_.load();
    }

    [[nodiscard]] bool is_open() const noexcept
    {
        return is_open_.load();
    }

    bool open_camera()
    {
        if (!is_initialized())
        {
            node_.log_error("open_camera: CameraWorker not initialized");
            return false;
        }

        try
        {
            // Reset previous video reader
            video_reader_ptr_.reset();
            is_open_.store(false);

            switch (params_.camera.source_type)
            {
            case CameraBgsParams::SourceType::USB_CAMERA:
            {
                node_.log_send_info("Trying to open camera %d", params_.camera.camera_id);
                video_reader_ptr_ = std::make_unique<boblib::video::VideoReader>(params_.camera.camera_id, params_.camera.use_opencv);
                if (video_reader_ptr_->is_open())
                {
                    set_camera_resolution();
                }
            }
            break;

            case CameraBgsParams::SourceType::VIDEO_FILE:
            {
                const uint32_t current_idx = current_video_idx_.load();
                if (current_idx >= params_.camera.videos.size())
                {
                    node_.log_send_error("Invalid video index: %u", current_idx);
                    return false;
                }
                const auto video_path = params_.camera.videos[current_idx];
                node_.log_send_info("Trying to open video '%s'", video_path.c_str());
                video_reader_ptr_ = std::make_unique<boblib::video::VideoReader>(video_path, params_.camera.use_opencv, using_cuda_.load());
            }
            break;

            case CameraBgsParams::SourceType::RTSP_STREAM:
            {
                node_.log_send_info("Trying to open RTSP Stream '%s'", params_.camera.rtsp_uri.c_str());
                video_reader_ptr_ = std::make_unique<boblib::video::VideoReader>(params_.camera.rtsp_uri, params_.camera.use_opencv, using_cuda_.load());
            }
            break;

            default:
                node_.log_send_error("Unknown SOURCE_TYPE: %d", static_cast<int>(params_.camera.source_type));
                return false;
            }

            if (!video_reader_ptr_)
            {
                node_.log_send_error("Failed to create video reader");
                return false;
            }

            if (video_reader_ptr_->using_cuda())
            {
                node_.log_send_info("Using CUDA for decoding.");
            }

            is_open_.store(video_reader_ptr_->is_open());
            if (is_open_.load())
            {
                node_.log_send_info("Stream is open.");
                update_capture_info();
                const auto now = node_.now();
                last_camera_connect_ = now;
                initial_camera_connect_ = (initial_camera_connect_.seconds() == 0 && initial_camera_connect_.nanoseconds() == 0) ? now : initial_camera_connect_;
            }
            else
            {
                node_.log_send_error("Could not open stream.");
                video_reader_ptr_.reset();
            }

            return is_open_.load();
        }
        catch (const std::exception &ex)
        {
            node_.log_error("open_camera: exception %s", ex.what());
            video_reader_ptr_.reset();
            is_open_.store(false);
            return false;
        }
    }

private:
    void initialize_profiler_regions()
    {
        prof_camera_worker_id_ = profiler_.add_region("Camera Worker");
        prof_acquire_image_id_ = profiler_.add_region("Acquire Image", prof_camera_worker_id_);
        prof_simulator_id_ = profiler_.add_region("Simulator", prof_camera_worker_id_);
        prof_mask_id_ = profiler_.add_region("Privacy Mask", prof_camera_worker_id_);
        prof_prepare_publish_id_ = profiler_.add_region("Prepare Publish", prof_camera_worker_id_);
        prof_publish_id_ = profiler_.add_region("Publish Frame", prof_camera_worker_id_);
        prof_publish_camera_info_id_ = profiler_.add_region("Publish Camera Info", prof_camera_worker_id_);
        prof_publish_resized_resizing_id_ = profiler_.add_region("Publish Resized Resizing", prof_camera_worker_id_);
        prof_publish_resized_msg_id_ = profiler_.add_region("Publish Resized Msg", prof_camera_worker_id_);
        prof_sample_resize_id_ = profiler_.add_region("Sample Resize", prof_camera_worker_id_);
        prof_sample_publish_id_ = profiler_.add_region("Sample Publish", prof_camera_worker_id_);
    }

    void initialize_state() 
    {
        // Initialize all state variables first
        current_video_idx_.store(0);
        run_.store(false);
        fps_ = UNKNOWN_DEVICE_FPS;
        mask_enabled_.store(false);
        is_open_.store(false);
        is_camera_info_auto_.store(false);
        using_cuda_.store(params_.use_cuda ? boblib::base::Utils::has_cuda() : false);
        cache_full_.store(false);
        current_test_idx_.store(0);
    }

    void initialize_components()
    {
        // Initialize camera info message
        camera_info_msg_ptr_ = std::make_shared<bob_camera::msg::CameraInfo>();
        camera_info_msg_ptr_->frame_height = 0;
        camera_info_msg_ptr_->frame_width = 0;
        camera_info_msg_ptr_->fps = 0;

        // Initialize timing
        update_interval_ = std::chrono::duration<double>(params_.sample_frame.interval);
        last_update_time_ = std::chrono::high_resolution_clock::now();

        // Initialize components that can throw
        privacy_mask_ptr_ = std::make_unique<boblib::base::Image>(using_cuda_.load());
        circuit_breaker_ptr_ = std::make_unique<CircuitBreaker>(
            CIRCUIT_BREAKER_MAX_RETRIES, 
            CIRCUIT_BREAKER_INITIAL_TIMEOUT, 
            CIRCUIT_BREAKER_MAX_TIMEOUT
        );

        if (params_.camera.simulator.enable) {
            object_simulator_ptr_ = std::make_unique<ObjectSimulator>(params_.camera.simulator.num_objects);
        }
    }

    void initialize_publishers()
    {
        // Initialize publishers - these can throw
        image_publisher_ = node_.create_publisher<sensor_msgs::msg::Image>(
            params_.topics.image_publish_topic, qos_publish_profile_);
        image_resized_publisher_ = node_.create_publisher<sensor_msgs::msg::Image>(
            params_.topics.image_resized_publish_topic, qos_publish_profile_);
        
        if (params_.sample_frame.enabled) {
            image_sample_publisher_ = node_.create_publisher<sensor_msgs::msg::Image>(
                params_.topics.sample_frame_publisher_topic, qos_publish_profile_);
        }
        
        image_info_publisher_ = node_.create_publisher<bob_camera::msg::ImageInfo>(
            params_.topics.image_info_publish_topic, qos_publish_profile_);
        camera_info_publisher_ = node_.create_publisher<bob_camera::msg::CameraInfo>(
            params_.topics.camera_info_publish_topic, qos_publish_profile_);

        camera_settings_client_ = node_.create_client<bob_interfaces::srv::CameraSettings>(
            params_.topics.camera_settings_client_topic);
    }

    void initialize_pubsub()
    {
        // Initialize pubsub
        publish_pubsub_ptr_ = topic_manager_.get_topic<PublishImage>(
            params_.topics.image_publish_topic + "_publish");
        publish_pubsub_ptr_->subscribe<CameraWorker, &CameraWorker::publish_image>(this);

        camera_info_pubsub_ptr_ = topic_manager_.get_topic<bob_camera::msg::CameraInfo>(
            params_.topics.camera_info_publish_topic);
    }

    void initialize_mask_worker()
    {
        // Initialize mask worker
        mask_worker_ptr_->init(params_.camera.privacy_mask.timer_seconds, 
                              params_.camera.privacy_mask.filename);
    }

    void cleanup() noexcept
    {
        try {
            stop_capture();
            
            // Reset unique_ptrs in reverse order of creation
            object_simulator_ptr_.reset();
            circuit_breaker_ptr_.reset();
            privacy_mask_ptr_.reset();
            speed_test_images_ptr_.reset();
            loop_rate_ptr_.reset();
            video_reader_ptr_.reset();
            
            // Reset shared_ptrs
            camera_info_msg_ptr_.reset();
            
            // Reset publishers and clients
            image_publisher_.reset();
            image_resized_publisher_.reset();
            image_sample_publisher_.reset();
            image_info_publisher_.reset();
            camera_info_publisher_.reset();
            camera_settings_client_.reset();
            
            // Reset pubsub
            publish_pubsub_ptr_.reset();
            camera_info_pubsub_ptr_.reset();
            
            // Reset state
            is_initialized_.store(false);
        }
        catch (...) {
            // Swallow all exceptions in cleanup
        }
    }

    void cleanup_on_init_failure() noexcept
    {
        is_initialized_.store(false);
        cleanup();
    }

    void update_capture_info()
    {
        if (is_open_.load())
        {
            fps_ = static_cast<float>(video_reader_ptr_->get_fps());
            const int cv_camera_width = video_reader_ptr_->get_width();
            const int cv_camera_height = video_reader_ptr_->get_height();
            node_.log_send_info("Stream capture Info: %dx%d at %.2g FPS", cv_camera_width, cv_camera_height, fps_);
            node_.log_send_info("              codec: %s, decoder: %s, Pixel Format: %s",
                                video_reader_ptr_->get_codec_name().c_str(), video_reader_ptr_->get_decoder_name().c_str(), video_reader_ptr_->get_pixel_format_name().c_str());
            loop_rate_ptr_ = std::make_unique<rclcpp::WallRate>(fps_);

            create_camera_info_msg();
        }
        else
        {
            loop_rate_ptr_ = std::make_unique<rclcpp::WallRate>(UNKNOWN_DEVICE_FPS);
        }
    }

    void cache_test(boblib::base::Image &camera_img)
    {
        if (!params_.camera.speed_test 
            || (params_.camera.speed_test && cache_full_.load()))
        {
            return;
        }
        if (!speed_test_images_ptr_)
        {
            speed_test_images_ptr_ = std::make_unique<std::vector<boblib::base::Image>>();
            speed_test_images_ptr_->reserve(params_.camera.test_frames);
            node_.log_send_info("Speed test cache created");
        }
        speed_test_images_ptr_->push_back(camera_img);
        if (speed_test_images_ptr_->size() >= params_.camera.test_frames)
        {
            cache_full_.store(true);
            current_test_idx_.store(0);
            // compute frame duration and init next‐time
            speed_test_frame_duration_ = std::chrono::milliseconds(MILLISECONDS_PER_SECOND / std::max(1, params_.camera.speed_test_fps));
            next_speed_test_time_ = std::chrono::steady_clock::now();
            node_.log_send_info("Speed test cache is full, size: %zu", speed_test_images_ptr_->size());
        }
    }

    bool acquire_image(boblib::base::Image &camera_img)
    {
        try
        {
            if (params_.camera.speed_test && cache_full_.load())
            {
                const size_t current_idx = current_test_idx_.load();
                camera_img = (*speed_test_images_ptr_)[current_idx];
                const size_t next_idx = (current_idx + 1) % params_.camera.test_frames;
                current_test_idx_.store(next_idx);

                // steady-clock sleep-until
                auto now = std::chrono::steady_clock::now();
                if (now < next_speed_test_time_)
                {
                    std::this_thread::sleep_until(next_speed_test_time_);
                    now = next_speed_test_time_;
                }
                next_speed_test_time_ = now + speed_test_frame_duration_;

                return true;
            }
            if (!circuit_breaker_ptr_->allow_request())
            {
                node_.log_send_error("Could not acquire image, Waiting to connect to camera");
                std::this_thread::sleep_for(std::chrono::milliseconds(CIRCUIT_BREAKER_SLEEP_MS));
                return false;
            }

            if ((video_reader_ptr_ == nullptr) || !video_reader_ptr_->read(camera_img))
            {
                if (params_.camera.source_type == CameraBgsParams::SourceType::VIDEO_FILE)
                {
                    const uint32_t current_idx = current_video_idx_.load();
                    const uint32_t next_idx = current_idx >= (params_.camera.videos.size() - 1) ? 0 : current_idx + 1;
                    current_video_idx_.store(next_idx);
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
            cache_test(camera_img);
            return true;
        }
        catch (const std::exception &ex)
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
        while (run_.load() && rclcpp::ok())
        {
            if (!is_initialized())
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(INITIALIZED_SLEEP_MS));
                continue;
            }

            try
            {
                // This block will acquire the image apply the simulation and then the privacy mask
                profiler_.start(prof_acquire_image_id_);
                auto camera_img_ptr = std::make_shared<boblib::base::Image>(using_cuda_.load());

                if (!acquire_image(*camera_img_ptr))
                {
                    profiler_.stop(prof_acquire_image_id_);
                    continue;
                }
                profiler_.stop(prof_acquire_image_id_);

                profiler_.start(prof_simulator_id_);
                if (params_.camera.simulator.enable)
                {
                    object_simulator_ptr_->move(*camera_img_ptr);
                }
                profiler_.stop(prof_simulator_id_);

                profiler_.start(prof_mask_id_);
                apply_mask(*camera_img_ptr);
                profiler_.stop(prof_mask_id_);

                profiler_.start(prof_prepare_publish_id_);
                auto header_ptr = std::make_shared<std_msgs::msg::Header>();
                header_ptr->stamp = node_.now();
                header_ptr->frame_id = node_.generate_uuid();

                fill_camera_info(*header_ptr, *camera_img_ptr);

                publish_pubsub_ptr_->publish(PublishImage(header_ptr, camera_img_ptr, camera_info_msg_ptr_));
                profiler_.stop(prof_prepare_publish_id_);

                // Only limiting fps if it is video and the limit_fps param is set
                if (params_.camera.limit_fps && (params_.camera.source_type == CameraBgsParams::SourceType::VIDEO_FILE))
                {
                    loop_rate_ptr_->sleep();
                }
            }
            catch (const std::exception &e)
            {
                node_.log_send_error("CameraWorker: capture_loop: Exception: %s", e.what());
                rcutils_reset_error();
            }
        }
        node_.log_send_info("CameraWorker: Leaving capture_loop");

        boblib::base::Utils::reset_cuda();
    }

    void publish_image(const std::shared_ptr<PublishImage> &publish_image) noexcept
    {
        try
        {
            profiler_.start(prof_publish_id_);
            ImageUtils::publish_image(image_publisher_, *publish_image->header_ptr, publish_image->image_ptr->toMat());
            profiler_.stop(prof_publish_id_);
            publish_resized_frame(*publish_image->header_ptr, *publish_image->image_ptr);
            // publish_image_info(*publish_image->header_ptr, *publish_image->image_ptr);
            profiler_.start(prof_publish_camera_info_id_);
            publish_camera_info();
            profiler_.stop(prof_publish_camera_info_id_);
            publish_sample(publish_image);
        }
        catch (const std::exception &e)
        {
            node_.log_send_error("CameraWorker: publish_images: Exception: %s", e.what());
            rcutils_reset_error();
        }
    }

    void publish_sample(const std::shared_ptr<PublishImage> &publish_image) noexcept
    {
        if (!params_.sample_frame.enabled)
        {
            return;
        }
        auto now = std::chrono::high_resolution_clock::now();
        auto time_since_last_update = now - last_update_time_;
        if (time_since_last_update >= update_interval_)
        {
            last_update_time_ = now;

            profiler_.start(prof_sample_resize_id_);
            auto &camera_img = *publish_image->image_ptr;

            const auto frame_width = static_cast<int>(camera_img.size().aspectRatio() * static_cast<double>(params_.sample_frame.height));
            const cv::Size frame_size(frame_width, params_.sample_frame.height);
            const size_t total_bytes = frame_size.area() * camera_img.elemSize();

            auto loaned = image_resized_publisher_->borrow_loaned_message();
            auto &resized_frame_msg = loaned.get();
            ImageUtils::fill_imagemsg_header(resized_frame_msg, *publish_image->header_ptr, frame_size, camera_img.type());
            resized_frame_msg.data.resize(total_bytes);

            cv::Mat resized_frame(frame_size, camera_img.type(), resized_frame_msg.data.data());
            cv::resize(camera_img.toMat(), resized_frame, frame_size, 0, 0, cv::INTER_NEAREST);
            profiler_.stop(prof_sample_resize_id_);

            profiler_.start(prof_sample_publish_id_);
            image_sample_publisher_->publish(std::move(resized_frame_msg));
            profiler_.stop(prof_sample_publish_id_);
        }
    }

    inline void apply_mask(boblib::base::Image &img)
    {
        if (!mask_enabled_.load() || !params_.camera.privacy_mask.enable_override)
        {
            return;
        }
        img.apply_mask(*privacy_mask_ptr_);
    }

    void publish_resized_frame(const std_msgs::msg::Header &header, const boblib::base::Image &camera_img) const
    {
        if ((params_.resize_height <= 0) || (image_resized_publisher_->get_subscription_count() <= 0))
        {
            return;
        }

        // Alocating the data for the resized image directly in the message so we don't have to copy it
        profiler_.start(prof_publish_resized_resizing_id_);

        const auto frame_width = static_cast<int>(camera_img.size().aspectRatio() * static_cast<double>(params_.resize_height));
        const cv::Size frame_size(frame_width, params_.resize_height);
        const size_t total_bytes = frame_size.area() * camera_img.elemSize();

        auto loaned = image_resized_publisher_->borrow_loaned_message();
        auto &resized_frame_msg = loaned.get();
        ImageUtils::fill_imagemsg_header(resized_frame_msg, header, frame_size, camera_img.type());
        resized_frame_msg.data.resize(total_bytes);

        cv::Mat resized_frame(frame_size, camera_img.type(), resized_frame_msg.data.data());
        cv::resize(camera_img.toMat(), resized_frame, frame_size, 0, 0, cv::INTER_NEAREST);

        profiler_.stop(prof_publish_resized_resizing_id_);

        profiler_.start(prof_publish_resized_msg_id_);
        image_resized_publisher_->publish(std::move(resized_frame_msg));
        profiler_.stop(prof_publish_resized_msg_id_);
    }

    void start_capture() noexcept
    {
        std::lock_guard<std::recursive_mutex> lock(state_mutex_);
        try {
            if (!run_.load() && !capture_thread_.joinable()) {
                run_.store(true);
                capture_thread_ = std::thread(&CameraWorker::capture_loop, this);
                node_.log_info("Camera capture thread started");
            } else {
                node_.log_warn("Camera capture already running");
            }
        }
        catch (const std::exception& e) {
            node_.log_error("Failed to start capture thread: %s", e.what());
            run_.store(false);
        }
    }

    void stop_capture() noexcept
    {
        try {
            // Signal thread to stop
            run_.store(false);
            
            // Wait for thread to finish
            std::thread capture_thread_local;
            {
                std::lock_guard<std::recursive_mutex> lock(state_mutex_);
                if (capture_thread_.joinable()) {
                    capture_thread_local = std::move(capture_thread_);
                }
            }
            
            // Join outside of mutex to avoid deadlock
            if (capture_thread_local.joinable()) {
                capture_thread_local.join();
                node_.log_info("Camera capture thread stopped");
            }
        }
        catch (const std::exception& e) {
            node_.log_error("Error stopping capture thread: %s", e.what());
        }
        catch (...) {
            node_.log_error("Unknown error stopping capture thread");
        }
    }

    bool set_camera_resolution()
    {
        // Try user-defined resolution first
        if ((params_.camera.usb_resolution.size() == 2) && 
            (video_reader_ptr_->set_resolution(params_.camera.usb_resolution[0], params_.camera.usb_resolution[1]))) {
            return true;
        }

        // Try supported resolutions in order of preference
        for (const auto &[width, height] : USB_SUPPORTED_RESOLUTIONS) {
            if (video_reader_ptr_->set_resolution(width, height)) {
                node_.log_info("Setting resolution for %d x %d", width, height);
                return true;
            }
        }

        return false;
    }

    void publish_image_info(const std_msgs::msg::Header &header, const boblib::base::Image &camera_img) const
    {
        if (!image_info_publisher_ || image_info_publisher_->get_subscription_count() <= 0)
        {
            return;
        }

        bob_camera::msg::ImageInfo image_info_msg;
        image_info_msg.header = header;

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

        image_info_publisher_->publish(image_info_msg);
    }
    void fill_camera_info(const std_msgs::msg::Header &header, const boblib::base::Image &camera_img)
    {
        camera_info_msg_ptr_->header = header;
        camera_info_msg_ptr_->fps = fps_;
        camera_info_msg_ptr_->frame_width = camera_img.size().width;
        camera_info_msg_ptr_->frame_height = camera_img.size().height;
        camera_info_msg_ptr_->is_color = camera_img.channels() >= 3;
        camera_info_msg_ptr_->initial_connection = initial_camera_connect_;
        camera_info_msg_ptr_->last_connection = last_camera_connect_;
    }

    void publish_camera_info()
    {
        node_.publish_if_subscriber(camera_info_publisher_, *camera_info_msg_ptr_);
        camera_info_pubsub_ptr_->publish(camera_info_msg_ptr_);
    }

    void request_camera_settings(const std::string_view &uri,
                                 const std::function<void(const bob_interfaces::srv::CameraSettings::Response::SharedPtr &)> &user_callback) const
    {
        if (uri.empty())
        {
            return;
        }
        auto request = std::make_shared<bob_interfaces::srv::CameraSettings::Request>();
        request->uri = uri;

        auto response_received_callback = [user_callback](rclcpp::Client<bob_interfaces::srv::CameraSettings>::SharedFuture future)
        {
            auto response = future.get();
            if (response->success)
            {
                user_callback(response);
            }
        };

        camera_settings_client_->async_send_request(request, response_received_callback);
    }

    void create_camera_info_msg()
    {
        is_camera_info_auto_.store(false);
        if (params_.camera.source_type == CameraBgsParams::SourceType::RTSP_STREAM)
        {
            auto update_camera_info = [this](const bob_interfaces::srv::CameraSettings::Response::SharedPtr &response)
            {
                camera_info_msg_ptr_->id = response->hardware_id;
                camera_info_msg_ptr_->manufacturer = response->manufacturer;
                camera_info_msg_ptr_->model = response->model;
                camera_info_msg_ptr_->serial_num = response->serial_number;
                camera_info_msg_ptr_->firmware_version = response->firmware_version;
                camera_info_msg_ptr_->num_configurations = response->num_configurations;
                camera_info_msg_ptr_->encoding = response->encoding;
                camera_info_msg_ptr_->frame_width = response->frame_width;
                camera_info_msg_ptr_->frame_height = response->frame_height;
                camera_info_msg_ptr_->fps = response->fps;
                is_camera_info_auto_.store(true);
                loop_rate_ptr_ = std::make_unique<rclcpp::WallRate>(response->fps);
            };
            request_camera_settings(params_.camera.onvif_uri, update_camera_info);
        }
        else if (params_.camera.source_type == CameraBgsParams::SourceType::USB_CAMERA)
        {
            camera_info_msg_ptr_->model = "USB Camera";
        }
        else if (params_.camera.source_type == CameraBgsParams::SourceType::VIDEO_FILE)
        {
            camera_info_msg_ptr_->model = "Video File";
        }
    }

    void mask_timer_callback(MaskWorker::MaskCheckType detection_mask_result, const cv::Mat &mask)
    {
        if (detection_mask_result == MaskWorker::MaskCheckType::Enable)
        {
            if (!mask_enabled_.load())
            {
                mask_enabled_.store(true);
                node_.log_send_info("CameraWorker: Privacy Mask Enabled.");
            }
            else
            {
                node_.log_send_info("CameraWorker: Privacy Mask Changed.");
            }
            privacy_mask_ptr_->create(mask);
        }
        else if ((detection_mask_result == MaskWorker::MaskCheckType::Disable) && mask_enabled_.load())
        {
            node_.log_send_info("CameraWorker: Privacy Mask Disabled.");
            mask_enabled_.store(false);
            privacy_mask_ptr_.reset();
        }
    }

    // ==================== CONFIGURATION CONSTANTS ====================
    
    // Camera performance constants
    static constexpr double UNKNOWN_DEVICE_FPS = 30.0;
    static constexpr int MILLISECONDS_PER_SECOND = 1000;
    
    // Circuit breaker configuration
    static constexpr int CIRCUIT_BREAKER_MAX_RETRIES = 10;
    static constexpr int CIRCUIT_BREAKER_INITIAL_TIMEOUT = 10;
    static constexpr int CIRCUIT_BREAKER_MAX_TIMEOUT = 10;
    static constexpr int CIRCUIT_BREAKER_SLEEP_MS = 1000;
    
    // Thread timing configuration  
    static constexpr int INITIALIZED_SLEEP_MS = 10;
    static constexpr int ERROR_RECOVERY_SLEEP_MS = 10;
    
    // Camera resolution presets (ordered by preference)
    static constexpr std::array<std::pair<int, int>, 9> USB_SUPPORTED_RESOLUTIONS = {{
        {3840, 2160}, // 4K UHD
        {2560, 1440}, // QHD, WQHD, 2K  
        {1920, 1080}, // Full HD
        {1600, 900},  // HD+
        {1280, 720},  // HD
        {1024, 768},  // XGA
        {800, 600},   // SVGA
        {640, 480},   // VGA
        {320, 240}    // QVGA
    }};

    // ==================== MEMBER VARIABLES ====================

    ParameterNode &node_;
    CameraBgsParams &params_;
    const rclcpp::QoS &qos_publish_profile_;
    boblib::utils::pubsub::TopicManager &topic_manager_;

    std::unique_ptr<MaskWorker> mask_worker_ptr_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_resized_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_sample_publisher_;

    rclcpp::Publisher<bob_camera::msg::ImageInfo>::SharedPtr image_info_publisher_;
    rclcpp::Publisher<bob_camera::msg::CameraInfo>::SharedPtr camera_info_publisher_;
    rclcpp::Client<bob_interfaces::srv::CameraSettings>::SharedPtr camera_settings_client_;

    std::unique_ptr<boblib::video::VideoReader> video_reader_ptr_;
    bob_camera::msg::CameraInfo::SharedPtr camera_info_msg_ptr_;

    std::atomic<bool> run_{false};
    std::thread capture_thread_;
    mutable std::recursive_mutex state_mutex_;

    std::atomic<uint32_t> current_video_idx_{0};

    std::unique_ptr<rclcpp::WallRate> loop_rate_ptr_;
    float fps_{UNKNOWN_DEVICE_FPS};
    std::atomic<bool> is_open_{false};
    std::atomic<bool> is_camera_info_auto_{false};
    std::atomic<bool> is_initialized_{false};
    std::unique_ptr<ObjectSimulator> object_simulator_ptr_;

    std::atomic<bool> mask_enabled_{false};
    std::unique_ptr<boblib::base::Image> privacy_mask_ptr_;

    std::unique_ptr<CircuitBreaker> circuit_breaker_ptr_;

    rclcpp::Time initial_camera_connect_;
    rclcpp::Time last_camera_connect_;

    std::atomic<bool> using_cuda_{false};

    std::shared_ptr<boblib::utils::pubsub::PubSub<PublishImage>> publish_pubsub_ptr_;
    std::shared_ptr<boblib::utils::pubsub::PubSub<bob_camera::msg::CameraInfo>> camera_info_pubsub_ptr_;

    std::unique_ptr<std::vector<boblib::base::Image>> speed_test_images_ptr_;
    std::atomic<bool> cache_full_{false};
    std::atomic<size_t> current_test_idx_{0};

    std::chrono::steady_clock::time_point next_speed_test_time_;
    std::chrono::milliseconds speed_test_frame_duration_{0};

    boblib::utils::Profiler &profiler_;

    size_t prof_camera_worker_id_;
    size_t prof_acquire_image_id_;
    size_t prof_simulator_id_;
    size_t prof_mask_id_;
    size_t prof_prepare_publish_id_;
    size_t prof_publish_id_;
    size_t prof_publish_camera_info_id_;
    size_t prof_publish_resized_resizing_id_;
    size_t prof_publish_resized_msg_id_;
    size_t prof_sample_resize_id_;
    size_t prof_sample_publish_id_;

    std::chrono::high_resolution_clock::time_point last_update_time_;
    std::chrono::duration<double> update_interval_;
};
