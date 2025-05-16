#include "profiler.hpp"
#include <iomanip>
#include <sstream>

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

        out << "Profiler Report\n" << "=============================\n";
        for (auto const& [id, root] : profiler_data_)
        {
            if (root.parent_id != 0)
            {
                continue;
            }

            // collect only nodes with data
            std::vector<const ProfilerData*> kids;
            for (auto cid : root.children)
            {
                auto it = profiler_data_.find(cid);
                if (it != profiler_data_.end() && it->second.count > 0)
                {
                    kids.push_back(&it->second);
                }
            }
            if (kids.empty()) 
            {
                out << report_line(&root, 0, 0, 0.0);
                continue;
            }

            // total in μs
            double total_us = 0.0;
            for (auto d : kids)
            {
                double dur_us = std::chrono::duration<double, std::micro>(d->duration).count();
                total_us += dur_us;
            }

            // compute name column width
            int name_w = root.name.size() + 2; // 2 for the ident level
            for (auto d : kids)
            {
                name_w = std::max(name_w, (int)d->name.size());
            }

            if (root.count > 0)
            {
                out << report_line(&root, 0, name_w, total_us);
            }
            else
            {
                out << root.name << "\n";
            }
            // print each child
            for (auto d : kids)
            {
                out << report_line(d, 1, name_w, total_us);
            }
        }

        return out.str();
    }

    std::string Profiler::report_line(const ProfilerData *d,
                                      int indent_level,
                                      int name_width,
                                      double total_us) const noexcept
    {
        std::ostringstream out;
        std::string indent(indent_level * 2, ' ');

        double dur_us = std::chrono::duration<double, std::micro>(d->duration).count();
        double avg_us = dur_us / d->count;
        double pct    = total_us > 0
                        ? (dur_us / total_us) * 100.0
                        : 0.0;

        out << indent
            << std::left   << std::setw(name_width) << d->name << " | "
            << "count: "   << std::right << std::setw(6) << d->count << " | "
            << "avg(us): " << std::right << std::setw(10) << std::fixed << std::setprecision(4) << avg_us << " | "
            << "%: "       << std::right << std::setw(5) << std::fixed << std::setprecision(2) << pct << "\n";

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