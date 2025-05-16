#include "profiler.hpp"
#include <iomanip>

namespace boblib::utils
{
    Profiler::Profiler(int report_time_seconds, bool enabled) noexcept
        : report_time_seconds_(report_time_seconds)
        , max_name_length_(0)
    {
        set_enabled(enabled);
    }

    void Profiler::set_enabled(bool enabled) noexcept
    {
        enabled_ = enabled;

        if (enabled_ && !monitor_thread_.joinable())
        {
            monitor_thread_ = std::jthread(&Profiler::monitor_thread, this);
        }
        else if (!enabled_ && monitor_thread_.joinable())
        {
            monitor_thread_.request_stop();
            monitor_thread_.join();
        }
    }

    size_t Profiler::add_region(std::string_view region) noexcept
    {
        std::lock_guard<std::mutex> lock(mutex_);
        size_t region_id = profiler_data_.size();
        profiler_data_[region_id].name = region;
        profiler_data_[region_id].region_id = region_id;
        max_name_length_ = std::max(max_name_length_, static_cast<int>(region.length()));
        return region_id;
    }

    void Profiler::start(size_t region_id) noexcept
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!enabled_)
        {
            return;
        }
        auto time = Clock::now();
        if (start_time_.time_since_epoch().count() == 0)
        {
            start_time_ = time;
        }
        auto &data = profiler_data_[region_id];
        data.region_id = region_id;
        data.start_time = time;
    }

    void Profiler::stop(size_t region_id) noexcept
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!enabled_)
        {
            return;
        }
        auto &data = profiler_data_[region_id];
        data.stop_time = Clock::now();
        if (data.stop_time > stop_time_)
        {
            stop_time_ = data.stop_time;
        }
        data.duration += data.stop_time - data.start_time;
        data.count++;
    }

    void Profiler::reset() noexcept
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!enabled_)
        {
            return;
        }
        for (auto &entry : profiler_data_)
        {
            entry.second.start_time = TimePoint{};
            entry.second.stop_time = TimePoint{};
            entry.second.duration = Duration{};
            entry.second.count = 0;
        }
        start_time_ = TimePoint{};
        stop_time_ = TimePoint{};
    }

    TimePoint Profiler::get_start_time() const noexcept
    {
        return start_time_;
    }

    Duration Profiler::get_total_duration() const noexcept
    {
        return Clock::now() - start_time_;
    }

    std::string Profiler::report() const noexcept
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!enabled_)
        {
            return {};
        }
        // Reserve a reasonable initial size for the report
        std::ostringstream oss;
        oss.str().reserve(profiler_data_.size() * 200); // Estimate 200 chars per entry

        // Create a vector of entries that can be sorted by key
        std::vector<std::pair<size_t, const ProfilerData *>> sorted_entries;
        sorted_entries.reserve(profiler_data_.size());

        for (const auto &entry : profiler_data_)
        {
            sorted_entries.push_back({entry.first, &entry.second});
        }

        // Sort by key
        std::sort(sorted_entries.begin(), sorted_entries.end(),
                  [](const auto &a, const auto &b)
                  { return a.first < b.first; });

        // Output in sorted order
        for (const auto &entry : sorted_entries)
        {
            oss << '\n'
                << report_individual(*entry.second);
        }
        
        return oss.str();
    }

    std::string Profiler::report_individual(const ProfilerData &data) const noexcept
    {
        std::ostringstream oss;
        auto totalDuration = stop_time_ - start_time_;

        // Calculate values first to avoid multiple calls
        double avg_ns = data.avg_time_in_ns();
        double avg_us = data.avg_time_in_us();
        double fps = data.fps();
        double percentage = (data.duration / totalDuration) * 100.0;

        // Set formatting flags
        oss << std::fixed;

        // Format name with right alignment in a field of 15 characters
        oss << '[' << std::setw(max_name_length_) << std::right << (!data.name.empty() ? data.name : std::to_string(data.region_id)) << "]: ";

        // Format numeric values with consistent width and precision
        oss << "Avg.Time(ns): " << std::setw(9) << std::right << std::setprecision(0) << avg_ns << ", ";
        oss << "Avg.Time(us): " << std::setw(9) << std::right << std::setprecision(2) << avg_us << ", ";
        oss << "Count: " << std::setw(4) << std::right << data.count << ", ";
        oss << "FPS: " << std::setw(7) << std::right << std::setprecision(2) << fps << ", ";
        oss << "%: " << std::setw(5) << std::right << std::setprecision(2) << percentage;

        return oss.str();
    }

    // bool Profiler::report_if_greater(double time_in_seconds, std::string &the_report) noexcept
    // {
    //     std::lock_guard<std::mutex> lock(mutex_);
    //     if (!enabled_)
    //     {
    //         return false;
    //     }

    //     const double elapsed_seconds = (stop_time_ - start_time_).count() * 1e-9;
    //     if (elapsed_seconds >= time_in_seconds)
    //     {
    //         the_report = report();
    //         reset();
    //         return true;
    //     }
    //     return false;
    // }

    void Profiler::monitor_thread(std::stop_token stoken)
    {
        while (!stoken.stop_requested())
        {
            std::this_thread::sleep_for(std::chrono::seconds(report_time_seconds_));
            if (start_time_.time_since_epoch().count() > 0)
            {
                std::cout << report() << std::endl;
                reset();
            }
        }
    }
}