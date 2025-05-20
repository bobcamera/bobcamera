#pragma once

#include <iostream>
#include <string>
#include <unordered_map>
#include <chrono>
#include <mutex>
#include <thread>
#include <vector>

#include "console_colors.hpp"

namespace boblib::utils
{
    using Clock = std::chrono::high_resolution_clock;
    using TimePoint = std::chrono::time_point<Clock>;
    using Duration = std::chrono::duration<double, std::nano>;

    struct ProfilerData
    {
        size_t                 region_id   = 0;
        std::string            name;
        size_t                 parent_id   = 0;
        std::vector<size_t>    children;
        TimePoint              start_time  = TimePoint{};
        TimePoint              stop_time   = TimePoint{};
        Duration               duration    = Duration{};
        size_t                 count       = 0;
    };

    using DataMap = std::unordered_map<size_t, ProfilerData>;

    class Profiler final
    {
    public:
        Profiler(std::string_view name, int report_time_seconds = 5, bool enabled = true) noexcept;
        ~Profiler() noexcept = default;

        void set_enabled(bool enabled) noexcept;

        // takes optional parent_id (0 == root)
        size_t add_region(std::string_view region, size_t parent_id = 0) noexcept;

        void start(size_t region_id) noexcept;

        void stop(size_t region_id) noexcept;

        void reset() noexcept;

        TimePoint get_start_time() const noexcept;

        Duration get_total_duration() const noexcept;

        std::string report() const noexcept;

    private:
        std::string name_;
        int report_time_seconds_;
        bool enabled_;
        DataMap profiler_data_;
        TimePoint start_time_;
        TimePoint stop_time_;
        int max_name_length_;
        mutable std::mutex mutex_;
        std::jthread monitor_thread_;

        std::string report_line(const ProfilerData *d,
                                int indent_level,
                                int name_width,
                                double local_total_us,
                                double total_us) const noexcept;
        void monitor_thread(std::stop_token stoken);
    };
}

