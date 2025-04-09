#pragma once

#include "PubSub.hpp"
#include <string>
#include <memory>
#include <functional>
#include <typeindex>
#include <unordered_map>
#include <shared_mutex>
#include <atomic>
#include <vector>

namespace boblib::utils::pubsub
{
    // Forward declarations
    class ISubscriber;
    template <typename T>
    class TopicReference;

    // Base class for type-erased PubSub instances
    class IPubSubBase
    {
    public:
        virtual ~IPubSubBase() = default;
        virtual bool publish(const void *message) = 0;
        virtual size_t subscribe(std::function<void(const void *)> callback) = 0;
        virtual bool unsubscribe(size_t id) = 0;
        virtual size_t subscriber_count() const = 0;
        virtual bool is_queue_full() const = 0;
        virtual size_t queue_size() const = 0;
        virtual size_t queue_capacity() const = 0;
        virtual void set_timeout(std::chrono::milliseconds timeout) = 0;
        virtual uint64_t get_version() const = 0;
    };

    // TopicManager class that abstracts message types
    class TopicManager
    {
    private:
        // Type-erased wrapper for PubSub<T>
        template <typename T>
        class PubSubWrapper : public IPubSubBase
        {
        private:
            PubSub<T> pubsub;
            std::atomic<uint64_t> version{1};

        public:
            explicit PubSubWrapper(size_t queue_size, std::chrono::milliseconds timeout);
            bool publish(const void *message) override;
            size_t subscribe(std::function<void(const void *)> callback) override;
            bool unsubscribe(size_t id) override;
            size_t subscriber_count() const override;
            bool is_queue_full() const override;
            size_t queue_size() const override;
            size_t queue_capacity() const override;
            void set_timeout(std::chrono::milliseconds timeout) override;
            uint64_t get_version() const override { return version.load(std::memory_order_acquire); }
            void increment_version() { version.fetch_add(1, std::memory_order_release); }
        };

        // Add a type identifier to each topic
        struct TopicInfo
        {
            std::shared_ptr<IPubSubBase> pubsub; // Use shared_ptr for reference counting
            std::type_index type_id;
            std::atomic<uint64_t> version{0}; // Version counter for change detection
        };
        std::unordered_map<std::string, TopicInfo> topics;

        // Replace single mutex with shared_mutex for better concurrency
        mutable std::shared_mutex topics_mutex;

        // Default queue size for new PubSub instances
        size_t default_queue_size;

        // Default timeout for new PubSub instances
        std::chrono::milliseconds default_timeout;

    public:
        explicit TopicManager(size_t queue_size = 100,
                              std::chrono::milliseconds timeout = std::chrono::milliseconds(5));

        // Get a reference to a topic (for frequent access)
        template <typename T>
        TopicReference<T> get_topic_reference(const std::string &topic_name);

        // Create a new topic with a specific message type
        template <typename T>
        void create_topic(const std::string &topic_name);

        // Subscribe to a topic with a specific message type
        template <typename T>
        size_t subscribe(const std::string &topic_name, std::function<void(const T &)> callback);

        // Unsubscribe from a topic
        bool unsubscribe(const std::string &topic_name, size_t subscriber_id);

        // Publish a message to a topic
        template <typename T>
        bool publish(const std::string &topic_name, const T &message);

        // Get the number of subscribers for a topic
        size_t subscriber_count(const std::string &topic_name) const;

        // Check if a topic's queue is full
        bool is_queue_full(const std::string &topic_name) const;

        // Get the current queue size for a topic
        size_t queue_size(const std::string &topic_name) const;

        // Get the queue capacity for a topic
        size_t queue_capacity(const std::string &topic_name) const;

        // Set the timeout for a topic
        void set_timeout(const std::string &topic_name, std::chrono::milliseconds timeout);

        // Set the default timeout for new topics
        void set_default_timeout(std::chrono::milliseconds timeout);

        // Set the default queue size for new topics
        void set_default_queue_size(size_t queue_size);

        // Check if a topic exists
        bool topic_exists(const std::string &topic_name) const;

        // Get a list of all topic names
        std::vector<std::string> get_topic_names() const;
        
        // Get a raw pointer to the PubSub instance for a topic
        IPubSubBase* get_pubsub_ptr(const std::string &topic_name) const;
    };
}
