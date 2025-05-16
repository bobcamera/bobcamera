#pragma once

#include <iostream>
#include <string>
#include <unordered_map>
#include <chrono>
#include <mutex>
#include <thread>

namespace boblib::utils
{
    using Clock = std::chrono::high_resolution_clock;
    using TimePoint = std::chrono::time_point<Clock>;
    using Duration = std::chrono::duration<double, std::nano>;

    struct ProfilerData
    {
        size_t region_id;
        std::string name;
        TimePoint start_time;
        TimePoint stop_time;
        Duration duration;
        size_t count;

        double fps() const noexcept
        {
            return duration.count() > 0 ? (double)count / duration_in_seconds() : 0.0;
        }

        double avg_time_in_ns() const noexcept
        {
            return count > 0 ? duration.count() / count : 0.0;
        }

        double avg_time_in_us() const noexcept
        {
            return count > 0 ? (duration.count() * 1e-3) / count : 0.0;
        }

        double avg_time_in_ms() const noexcept
        {
            return count > 0 ? (duration.count() * 1e-6) / count : 0.0;
        }

        double avg_time_in_s() const noexcept
        {
            return count > 0 ? (duration.count() * 1e-9) / count : 0.0;
        }

        double duration_in_seconds() const noexcept
        {
            return duration.count() * 1e-9;
        }
    };

    using DataMap = std::unordered_map<size_t, ProfilerData>;

    class Profiler final
    {
    public:
        Profiler(int report_time_seconds = 5, bool enabled = true) noexcept;
        ~Profiler() noexcept = default;

        void set_enabled(bool enabled) noexcept;

        size_t add_region(std::string_view region) noexcept;

        void start(size_t region_id) noexcept;

        void stop(size_t region_id) noexcept;

        void reset() noexcept;

        TimePoint get_start_time() const noexcept;

        Duration get_total_duration() const noexcept;

        std::string report() const noexcept;

        //bool report_if_greater(double time_in_seconds, std::string &the_report) noexcept;

    private:
        int report_time_seconds_;
        bool enabled_;
        DataMap profiler_data_;
        TimePoint start_time_;
        TimePoint stop_time_;
        int max_name_length_;
        mutable std::mutex mutex_;  // protect all shared state
        std::jthread monitor_thread_;

        std::string report_individual(const ProfilerData &data) const noexcept;
        void monitor_thread(std::stop_token stoken);
    };
}

