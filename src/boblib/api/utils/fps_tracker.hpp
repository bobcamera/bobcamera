#pragma once

#include <chrono>
#include <string>

namespace boblib::utils
{
    class FpsTracker final
    {
    public:
        explicit FpsTracker(bool enabled = true, double update_interval_seconds = 5) noexcept
            : enabled_(enabled), update_interval_(std::chrono::duration<double>(update_interval_seconds))
        {
            reset();
        }

        // Record a new frame
        void add_frame() noexcept
        {
            if (!enabled_)
            {
                return;
            }
            frame_count_++;
            auto now = std::chrono::high_resolution_clock::now();
            current_duration_ = now - start_time_;
        }

        // Compute current FPS based on accumulated frames and time
        [[nodiscard]] double compute_fps() const noexcept
        {
            if (!enabled_)
            {
                return 0.0;
            }
            if (current_duration_.count() > 0)
            {
                return static_cast<double>(frame_count_) /
                       std::chrono::duration<double>(current_duration_).count();
            }
            return 0.0;
        }

        // Return FPS only if update interval has passed
        [[nodiscard]] bool get_fps_if_ready(double &fps) noexcept
        {
            if (!enabled_)
            {
                return false;
            }
            auto now = std::chrono::high_resolution_clock::now();
            auto time_since_last_update = now - last_update_time_;

            if (time_since_last_update >= update_interval_)
            {
                fps = compute_fps();
                //last_update_time_ = now;
                reset();
                return true;
            }

            return false;
        }

        // Reset the tracker
        void reset() noexcept
        {
            if (!enabled_)
            {
                return;
            }
            frame_count_ = 0;
            start_time_ = std::chrono::high_resolution_clock::now();
            last_update_time_ = start_time_;
            current_duration_ = std::chrono::high_resolution_clock::duration::zero();
        }

    private:
        bool enabled_{true};
        size_t frame_count_{0};
        std::chrono::high_resolution_clock::time_point start_time_;
        std::chrono::high_resolution_clock::time_point last_update_time_;
        std::chrono::high_resolution_clock::duration current_duration_;
        std::chrono::duration<double> update_interval_;
    };
}