#pragma once

#include <string>
#include <string_view>
#include <memory>
#include <unordered_map>
#include <mutex>
#include <thread>
#include <typeindex>
#include <shared_mutex>
#include <functional>

#include "PubSub.hpp"

namespace boblib::utils::pubsub
{
    // Base class for type-erased topic storage
    class TopicBase
    {
    public:
        virtual ~TopicBase() = default;
        virtual QueueStats get_queue_stats() const noexcept = 0;
        virtual size_t queue_size() const noexcept = 0;
        virtual std::size_t dropped_count() const noexcept = 0;
    };

    // Concrete implementation for specific message types
    template <typename T>
    class TypedTopic : public TopicBase
    {
    private:
        std::shared_ptr<PubSub<T>> pubsub;

    public:
        explicit TypedTopic(std::shared_ptr<PubSub<T>> p) 
            : pubsub(std::move(p)) {}

        QueueStats get_queue_stats() const noexcept override
        {
            return pubsub->get_queue_stats();
        }
        size_t queue_size() const noexcept override 
        { 
            return pubsub->queue_size(); 
        }
        std::size_t dropped_count() const noexcept override
        {
            return pubsub->dropped_count();
        }
    };

    // transparent hasher that can take either std::string or std::string_view
    struct TransparentStringHash
    {
        using is_transparent = void; // enables heterogeneous lookup

        size_t operator()(std::string_view sv) const noexcept;

        size_t operator()(const std::string &s) const noexcept;
    };

    // TopicManager class to manage multiple topics with different message types
    class TopicManager
    {
    public:
        TopicManager(const TopicManager &) = delete;
        TopicManager &operator=(const TopicManager &) = delete;
        TopicManager(TopicManager &&) = default;
        TopicManager &operator=(TopicManager &&) = default;

        // Constructor with default values
        explicit TopicManager(size_t queue_size = 100, bool enable_monitoring = true, int report_time_seconds = 10);

        // Get or create a PubSub instance for a topic with specific type
        template <typename T>
            requires CopyableOrMovable<T>
        std::shared_ptr<PubSub<T>> get_topic(std::string_view topic_name)
        {
            std::unique_lock lock(topics_mutex);
            auto it = topics.find(topic_name);

            if (it == topics.end())
            {
                // Create a new topic if it doesn't exist
                auto pubsub = std::make_shared<PubSub<T>>(default_queue_size);
                auto typed_topic = std::make_unique<TypedTopic<T>>(pubsub);
                auto [new_it, _] = topics.emplace(
                    topic_name,
                    TopicData(std::move(typed_topic), typeid(T), pubsub));
                it = new_it;
                bigger_name_size_ = std::max(bigger_name_size_, topic_name.size());
                return pubsub;
            }
            else if (it->second.type_id != typeid(T))
            {
                // Topic exists but with a different type
                throw std::runtime_error("Topic '" + std::string(topic_name) + "' exists with a different message type");
            }

            // Return the stored PubSub instance
            return std::static_pointer_cast<PubSub<T>>(it->second.pubsub);
        }

        // set monitoring for topics
        // If enabled, it will start a monitoring thread that reports the status of topics
        void set_monitoring(bool enable, int report_time_seconds = 10) noexcept;

        // Check if a topic exists
        bool topic_exists(std::string_view topic_name) const noexcept;

        // Get the current number of topics
        size_t topic_count() const noexcept;

        template <typename F>
        size_t with_topic(std::string_view name, F fn) const
        {
            std::shared_lock lock(topics_mutex);
            if (auto it = topics.find(std::string(name)); it != topics.end())
            {
                return std::invoke(fn, *it->second.topic);
            }
            return 0;
        }

        // Get queue size for a specific topic
        size_t queue_size(std::string_view n) const noexcept;

    private:
        // Struct to hold topic data
        struct TopicData
        {
            std::unique_ptr<TopicBase> topic;
            std::type_index type_id;
            std::shared_ptr<void> pubsub; // Store the PubSub instance directly

            TopicData(std::unique_ptr<TopicBase> t, std::type_index id, std::shared_ptr<void> p);
        };

        // Map from topic name to topic data
        std::unordered_map<
            std::string,
            TopicData,
            TransparentStringHash, // heterogeneous hash
            std::equal_to<>        // transparent equals
            >
            topics;

        // Mutex to protect the topics map
        mutable std::shared_mutex topics_mutex;

        // Default queue size and timeout for new topics
        size_t default_queue_size;

        size_t bigger_name_size_ = 0; // for monitoring purposes
        bool enable_monitoring_;
        int report_time_seconds_;
        std::jthread monitor_thread_;

        void monitor_thread(std::stop_token stoken) noexcept;
    };
} // namespace boblib::utils::pubsub