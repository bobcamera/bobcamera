#include "profiler.hpp"
#include <iomanip>
#include <sstream>

namespace boblib::utils
{
    Profiler::Profiler(std::string_view name, int report_time_seconds, bool enabled) noexcept
        : name_(name)
        , report_time_seconds_(report_time_seconds)
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

    size_t Profiler::add_region(std::string_view region, size_t parent_id) noexcept
    {
        std::scoped_lock lock(mutex_);
        size_t new_id = profiler_data_.size() + 1;
        ProfilerData d;
        d.region_id = new_id;
        d.name = std::string(region);
        d.parent_id = parent_id;
        profiler_data_[new_id] = std::move(d);

        if (parent_id != 0)
        {
            auto it = profiler_data_.find(parent_id);
            if (it != profiler_data_.end())
            {
                it->second.children.push_back(new_id);
            }
        }
        return new_id;
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

        if (data.start_time.time_since_epoch().count() == 0)
        {
            return;
        }

        auto now = Clock::now();
        data.stop_time = now;
        if (data.stop_time > stop_time_)
        {
            stop_time_ = data.stop_time;
        }

        data.duration += (data.stop_time - data.start_time);
        data.start_time = TimePoint{};
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
        std::scoped_lock lock(mutex_);
        std::ostringstream out;

        // 1) find the longest name of any region:
        size_t max_len = 0;
        double total_us = 0.0;
        for (auto const& [id, data] : profiler_data_)
        {
            max_len = std::max(max_len, data.name.size());
            total_us += std::chrono::duration<double, std::micro>(data.duration).count();
        }

        // 2) make a single global column width (add 2 for indent padding)
        int name_w = static_cast<int>(max_len) + 2;

        out << boblib::utils::console_colors::BRIGHT_GREEN << boblib::utils::console_colors::BOLD
            << name_ << " Report" << boblib::utils::console_colors::RESET << "\n"
            << boblib::utils::console_colors::GREEN
            << std::string(name_w + 78, '=')
            << boblib::utils::console_colors::RESET << "\n";

        for (auto const& [id, root] : profiler_data_)
        {
            if (root.parent_id != 0) continue;

            // collect only children that actually ran
            std::vector<const ProfilerData*> kids;
            for (auto cid : root.children)
            {
                auto it = profiler_data_.find(cid);
                if (it != profiler_data_.end() && it->second.count > 0)
                    kids.push_back(&it->second);
            }

            // compute total μs over all children
            double local_total_us = std::chrono::duration<double, std::micro>(root.duration).count();
            for (auto d : kids)
            {
                local_total_us += std::chrono::duration<double, std::micro>(d->duration).count();
            }

            // print the root itself (if it ever ran)
            if (root.count > 0)
            {
                out << report_line(&root, 0, name_w + 2, local_total_us, total_us);
            }
            else
            {
                double root_pct = total_us > 0 ? (local_total_us / total_us) * 100.0 : 0.0;
                out << boblib::utils::console_colors::YELLOW
                    << std::left << std::setw(name_w + 60) << root.name
                    << boblib::utils::console_colors::RESET
                    << " | " << boblib::utils::console_colors::BOLD << "% total: "
                    << boblib::utils::console_colors::YELLOW << std::right << std::setw(6) << std::fixed << std::setprecision(2) << root_pct
                    << boblib::utils::console_colors::RESET << "\n";
            }

            for (auto d : kids)
            {
                out << report_line(d, 1, name_w, local_total_us, total_us);
            }
        }
        out << boblib::utils::console_colors::GREEN
            << std::string(name_w + 78, '=')
            << boblib::utils::console_colors::RESET << "\n";

        return out.str();
    }

    std::string Profiler::report_line(const ProfilerData *d,
                                      int indent_level,
                                      int name_width,
                                      double local_total_us,
                                      double total_us) const noexcept
    {
        std::ostringstream out;
        std::string indent(indent_level * 2, ' ');

        double dur_us = std::chrono::duration<double, std::micro>(d->duration).count();
        double avg_us = dur_us / d->count;
        double local_pct = local_total_us > 0 ? (dur_us / local_total_us) * 100.0 : 0.0;
        double total_pct = total_us > 0 ? (dur_us / total_us) * 100.0: 0.0;
        bool is_root = d->parent_id == 0;
        auto color = is_root ? boblib::utils::console_colors::BRIGHT_YELLOW : boblib::utils::console_colors::BRIGHT_CYAN;

        out << indent
            << color << std::left << std::setw(name_width) << d->name << boblib::utils::console_colors::RESET
            << " | " << boblib::utils::console_colors::BOLD << "count: " << color << std::right << std::setw(8) << d->count << boblib::utils::console_colors::RESET
            << " | " << boblib::utils::console_colors::BOLD << "avg(us): " << color << std::right << std::setw(10) << std::fixed << std::setprecision(4) << avg_us << boblib::utils::console_colors::RESET
            << " | " << boblib::utils::console_colors::BOLD << "% local: " << color << std::right << std::setw(6) << std::fixed << std::setprecision(2) << local_pct << boblib::utils::console_colors::RESET
            << " | " << boblib::utils::console_colors::BOLD << "% total: " << color << std::right << std::setw(6) << std::fixed << std::setprecision(2) << total_pct << boblib::utils::console_colors::RESET
            << boblib::utils::console_colors::RESET
            << "\n";

        return out.str();
    }

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