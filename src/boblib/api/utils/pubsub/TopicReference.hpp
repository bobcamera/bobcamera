#pragma once

// Remove the include of TopicManager.hpp to break the circular dependency
// #include "TopicManager.hpp"
#include <string>
#include <memory>
#include <functional>
#include <typeindex>
#include <shared_mutex>
#include <atomic>
#include <chrono>

namespace boblib::utils::pubsub
{
    // Forward declarations
    class TopicManager;
    class IPubSubBase;

    // TopicReference class to cache references to frequently accessed topics
    template <typename T>
    class TopicReference
    {
    private:
        TopicManager &manager;
        std::string topic_name;
        IPubSubBase *pubsub_ptr;                 // Raw pointer for fast access
        std::atomic<uint64_t> cached_version{0}; // Cached version for change detection
        std::type_index type_id;
        mutable std::shared_mutex ref_mutex; // Shared mutex for less contention
        std::atomic<bool> valid{false};

        // Cache for subscriber count to avoid frequent locking
        mutable std::atomic<size_t> cached_subscriber_count{0};
        mutable std::atomic<bool> subscriber_count_dirty{true};

        // Cache for queue status to avoid frequent locking
        mutable std::atomic<bool> cached_queue_full{false};
        mutable std::atomic<size_t> cached_queue_size{0};
        mutable std::atomic<bool> queue_status_dirty{true};

        // Update the cached subscriber count if needed
        void update_subscriber_count() const
        {
            if (subscriber_count_dirty.load(std::memory_order_relaxed))
            {
                cached_subscriber_count.store(manager.subscriber_count(topic_name),
                                            std::memory_order_release);
                subscriber_count_dirty.store(false, std::memory_order_release);
            }
        }

        // Update the cached queue status if needed
        void update_queue_status() const
        {
            if (queue_status_dirty.load(std::memory_order_relaxed))
            {
                cached_queue_full.store(manager.is_queue_full(topic_name),
                                      std::memory_order_release);
                cached_queue_size.store(manager.queue_size(topic_name),
                                      std::memory_order_release);
                queue_status_dirty.store(false, std::memory_order_release);
            }
        }

    public:
        TopicReference(TopicManager &mgr, const std::string &name);
        void refresh();
        bool is_valid() const;
        bool publish(const T &message);
        size_t subscribe(std::function<void(const T &)> callback);
        bool unsubscribe(size_t id);
        size_t subscriber_count() const;
        bool is_queue_full() const;
        size_t queue_size() const;
        size_t queue_capacity() const;
        void set_timeout(std::chrono::milliseconds timeout);
    };
} 