#include "profiler.hpp"
#include <iomanip>

namespace boblib::utils
{
    Profiler::Profiler(bool enabled) noexcept
        : enabled_(enabled)
    {
    }

    void Profiler::set_enabled(bool enabled) noexcept
    {
        enabled_ = enabled;
    }

    void Profiler::start(size_t region_id, std::string_view region) noexcept
    {
        if (!enabled_)
        {
            return;
        }
        auto &data = profiler_data_[region_id];
        if (data.name.empty())
        {
            data.name = region;
            max_name_length_ = std::max(max_name_length_, static_cast<int>(region.length()));
        }

        auto time = Clock::now();
        if (start_time_.time_since_epoch().count() == 0)
        {
            start_time_ = time;
        }

        data.start_time = time;
    }

    void Profiler::stop(size_t region_id) noexcept
    {
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
        if (!enabled_)
        {
            return;
        }
        profiler_data_.clear();
        start_time_ = TimePoint{};
        stop_time_ = TimePoint{};
        max_name_length_ = 0;
    }

    ProfilerData const &Profiler::get_data(size_t region_id) const noexcept
    {
        return profiler_data_.at(region_id);
    }

    DataMap const &Profiler::get_data() const noexcept
    {
        return profiler_data_;
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
        if (!enabled_)
        {
            return {};
        }
        // Reserve a reasonable initial size for the report
        std::ostringstream oss;
        oss.str().reserve(profiler_data_.size() * 200); // Estimate 200 chars per entry

        for (const auto &entry : profiler_data_)
        {
            oss << '\n'
                << report_individual(entry.second);
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
        oss << '[' << std::setw(max_name_length_) << std::right << data.name << "]: ";

        // Format numeric values with consistent width and precision
        oss << "Avg.Time(ns): " << std::setw(9) << std::right << std::setprecision(0) << avg_ns << ", ";
        oss << "Avg.Time(us): " << std::setw(9) << std::right << std::setprecision(2) << avg_us << ", ";
        oss << "Count: " << std::setw(4) << std::right << data.count << ", ";
        oss << "FPS: " << std::setw(7) << std::right << std::setprecision(2) << fps << ", ";
        oss << "%: " << std::setw(5) << std::right << std::setprecision(2) << percentage;

        return oss.str();
    }

    bool Profiler::report_if_greater(double time_in_seconds, std::string & the_report) noexcept
    {
        if (!enabled_)
        {
            return false;
        }

        const double elapsed_seconds = (stop_time_ - start_time_).count() * 1e-9;
        if (elapsed_seconds >= time_in_seconds)
        {
            the_report = report();
            reset();
            return true;
        }
        return false;
    }
}