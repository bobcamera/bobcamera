#include "TopicManager.hpp"
#include <iostream>
#include <iomanip> // For std::setw
#include "../console_colors.hpp"

namespace boblib::utils::pubsub
{
    size_t TransparentStringHash::operator()(std::string_view sv) const noexcept
    {
        return std::hash<std::string_view>{}(sv);
    }
    size_t TransparentStringHash::operator()(const std::string &s) const noexcept
    {
        return std::hash<std::string_view>{}(s);
    }

    TopicManager::TopicData::TopicData(std::unique_ptr<TopicBase> t, std::type_index id, std::shared_ptr<void> p)
        : topic(std::move(t)), type_id(id), pubsub(std::move(p))
    {
    }

    TopicManager::TopicManager(size_t queue_size, bool enable_monitoring, int report_time_seconds)
        : default_queue_size(queue_size)
    {
        set_monitoring(enable_monitoring, report_time_seconds);
    }

    void TopicManager::set_monitoring(bool enable, int report_time_seconds) noexcept
    {
        enable_monitoring_ = enable;
        report_time_seconds_ = report_time_seconds;
        if (enable_monitoring_)
        {
            monitor_thread_ = std::jthread(&TopicManager::monitor_thread, this);
        }
        else
        {
            monitor_thread_.request_stop();
            monitor_thread_.join();
        }
    }

    // Check if a topic exists
    bool TopicManager::topic_exists(std::string_view topic_name) const noexcept
    {
        std::shared_lock lock(topics_mutex);
        return topics.find(topic_name) != topics.end();
    }

    // Get the current number of topics
    size_t TopicManager::topic_count() const noexcept
    {
        std::shared_lock lock(topics_mutex);
        return topics.size();
    }

    // Get queue size for a specific topic
    size_t TopicManager::queue_size(std::string_view n) const noexcept
    { 
        return with_topic(n, &TopicBase::queue_size); 
    }

    void TopicManager::monitor_thread(std::stop_token stoken) noexcept
    {
        while (!stoken.stop_requested())
        {
            std::this_thread::sleep_for(std::chrono::seconds(report_time_seconds_));
            if (topics.empty())
            {
                continue;
            }

            std::shared_lock lock(topics_mutex);
            std::ostringstream out;

            out << boblib::utils::console_colors::BRIGHT_GREEN << boblib::utils::console_colors::BOLD
                << "Topics Report" << boblib::utils::console_colors::RESET << "\n"
                << boblib::utils::console_colors::GREEN
                << std::string(bigger_name_size_ + 65, '=')
                << boblib::utils::console_colors::RESET << "\n";
            
            for (const auto& [topic_name, topic_data] : topics)
            {
                auto stats = topic_data.topic->get_queue_stats();
                
                out << boblib::utils::console_colors::BOLD << "Topic: " << boblib::utils::console_colors::RESET
                    << boblib::utils::console_colors::BRIGHT_CYAN << std::left << std::setw(bigger_name_size_) << topic_name << boblib::utils::console_colors::RESET
                    << " | " << boblib::utils::console_colors::BOLD << "Subs: " << boblib::utils::console_colors::RESET
                    << boblib::utils::console_colors::BRIGHT_CYAN << std::right << std::setw(2) << stats.active_subscribers << boblib::utils::console_colors::RESET
                    << " | " << boblib::utils::console_colors::BOLD << "Total Q: " << boblib::utils::console_colors::RESET
                    << boblib::utils::console_colors::BRIGHT_CYAN << std::right << std::setw(4) << stats.total_subscriber_queue_size << boblib::utils::console_colors::RESET
                    << " | " << boblib::utils::console_colors::BOLD << "Max Q: " << boblib::utils::console_colors::RESET
                    << boblib::utils::console_colors::BRIGHT_CYAN << std::right << std::setw(4) << stats.max_subscriber_queue_size << boblib::utils::console_colors::RESET
                    << " | " << boblib::utils::console_colors::BOLD << "Dropped: " << boblib::utils::console_colors::RESET
                    << boblib::utils::console_colors::BRIGHT_CYAN << std::right << std::setw(5) << topic_data.topic->dropped_count() << boblib::utils::console_colors::RESET
                    << std::endl;
            }
            
            out << boblib::utils::console_colors::GREEN
                << std::string(bigger_name_size_ + 65, '=')
                << boblib::utils::console_colors::RESET << "\n";

            std::cout << out.str() << std::endl;
        }
    }
} // namespace boblib::utils::pubsub