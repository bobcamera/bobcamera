#pragma once

#include <string>
#include <memory>
#include <unordered_map>
#include <mutex>
#include <typeindex>
#include <any>
#include <functional>
#include <shared_mutex>
#include "PubSub.hpp"
namespace boblib::utils::pubsub
{
    // Base class for type-erased topic storage
    class TopicBase
    {
    public:
        virtual ~TopicBase() = default;
        virtual size_t subscriber_count() const = 0;
        virtual size_t queue_size() const = 0;
        virtual size_t queue_capacity() const = 0;
        virtual bool is_queue_full() const = 0;
        virtual void set_timeout(std::chrono::milliseconds timeout) = 0;
    };

    // Concrete implementation for specific message types
    template <typename T>
    class TypedTopic : public TopicBase
    {
    private:
        std::shared_ptr<PubSub<T>> pubsub;

    public:
        explicit TypedTopic(size_t queue_size, std::chrono::milliseconds timeout)
            : pubsub(std::make_shared<PubSub<T>>(queue_size, timeout)) {}

        std::shared_ptr<PubSub<T>> get_pubsub() const { return pubsub; }

        size_t subscriber_count() const override { return pubsub->subscriber_count(); }
        size_t queue_size() const override { return pubsub->queue_size(); }
        size_t queue_capacity() const override { return pubsub->queue_capacity(); }
        bool is_queue_full() const override { return pubsub->is_queue_full(); }
        void set_timeout(std::chrono::milliseconds timeout) override { pubsub->set_timeout(timeout); }
    };

    // TopicManager class to manage multiple topics with different message types
    class TopicManager
    {
    private:
        // Struct to hold topic data
        struct TopicData
        {
            std::shared_ptr<TopicBase> topic;
            std::type_index type_id;
            size_t ref_count;

            TopicData(std::shared_ptr<TopicBase> t, std::type_index id)
                : topic(std::move(t)), type_id(id), ref_count(0) {}
        };

        // Map from topic name to topic data
        std::unordered_map<std::string, TopicData> topics;

        // Mutex to protect the topics map
        mutable std::shared_mutex topics_mutex;

        // Default queue size and timeout for new topics
        size_t default_queue_size;
        std::chrono::milliseconds default_timeout;

    public:
        // Constructor with default values
        explicit TopicManager(
            size_t queue_size = 1024,
            std::chrono::milliseconds timeout = std::chrono::milliseconds(5))
            : default_queue_size(queue_size), default_timeout(timeout) {}

        // Get or create a PubSub instance for a topic with specific type
        template <typename T>
            requires CopyableOrMovable<T>
        std::shared_ptr<PubSub<T>> get_topic(const std::string &topic_name)
        {
            std::unique_lock lock(topics_mutex);
            auto it = topics.find(topic_name);

            if (it == topics.end())
            {
                // Create a new topic if it doesn't exist
                auto typed_topic = std::make_shared<TypedTopic<T>>(default_queue_size, default_timeout);
                auto [new_it, inserted] = topics.emplace(
                    topic_name,
                    TopicData(typed_topic, typeid(T)));
                it = new_it;
            }
            else if (it->second.type_id != typeid(T))
            {
                // Topic exists but with a different type
                throw std::runtime_error("Topic '" + topic_name + "' exists with a different message type");
            }

            // Increment reference count
            it->second.ref_count++;
            // Downcast to the correct type
            auto typed_topic = std::static_pointer_cast<TypedTopic<T>>(it->second.topic);
            return typed_topic->get_pubsub();
        }

        // Release a topic (decrement reference count and remove if zero)
        void release_topic(const std::string &topic_name)
        {
            std::unique_lock lock(topics_mutex);
            auto it = topics.find(topic_name);
            if (it != topics.end())
            {
                if (--it->second.ref_count == 0)
                {
                    // Remove topic if no more references
                    topics.erase(it);
                }
            }
        }

        // Publish a message to a specific topic
        template <typename T>
            requires CopyableOrMovable<T>
        bool publish(const std::string &topic_name, T message)
        {
            std::shared_ptr<PubSub<T>> pubsub;
            {
                std::shared_lock lock(topics_mutex);
                auto it = topics.find(topic_name);
                
                if (it == topics.end() || it->second.type_id != typeid(T))
                {
                    // Topic doesn't exist or has wrong type, create/get it
                    lock.unlock();
                    pubsub = get_topic<T>(topic_name);
                }
                else
                {
                    // Topic exists with correct type
                    auto typed_topic = std::static_pointer_cast<TypedTopic<T>>(it->second.topic);
                    pubsub = typed_topic->get_pubsub();
                }
            }
            return pubsub->publish(std::move(message));
        }

        // Subscribe to a specific topic
        template <typename T>
            requires CopyableOrMovable<T>
        size_t subscribe(const std::string &topic_name, std::function<void(const T &)> callback)
        {
            auto pubsub = get_topic<T>(topic_name);
            return pubsub->subscribe(std::move(callback));
        }

        // Unsubscribe from a specific topic
        template <typename T>
            requires CopyableOrMovable<T>
        bool unsubscribe(const std::string &topic_name, size_t subscriber_id)
        {
            std::shared_lock lock(topics_mutex);
            auto it = topics.find(topic_name);
            if (it != topics.end() && it->second.type_id == typeid(T))
            {
                auto typed_topic = std::static_pointer_cast<TypedTopic<T>>(it->second.topic);
                return typed_topic->get_pubsub()->unsubscribe(subscriber_id);
            }
            return false;
        }

        // Check if a topic exists
        bool topic_exists(const std::string &topic_name) const
        {
            std::shared_lock lock(topics_mutex);
            return topics.find(topic_name) != topics.end();
        }

        // Check if a topic exists with a specific type
        template <typename T>
        bool topic_exists_with_type(const std::string &topic_name) const
        {
            std::shared_lock lock(topics_mutex);
            auto it = topics.find(topic_name);
            return it != topics.end() && it->second.type_id == typeid(T);
        }

        // Get the number of subscribers for a topic
        size_t subscriber_count(const std::string &topic_name) const
        {
            std::shared_lock lock(topics_mutex);
            auto it = topics.find(topic_name);
            if (it != topics.end())
            {
                return it->second.topic->subscriber_count();
            }
            return 0;
        }

        // Get the current number of topics
        size_t topic_count() const
        {
            std::shared_lock lock(topics_mutex);
            return topics.size();
        }

        // Configure a specific topic's timeout
        void set_topic_timeout(const std::string &topic_name, std::chrono::milliseconds timeout)
        {
            std::shared_lock lock(topics_mutex);
            auto it = topics.find(topic_name);
            if (it != topics.end())
            {
                it->second.topic->set_timeout(timeout);
            }
        }

        // Set the default timeout for new topics
        void set_default_timeout(std::chrono::milliseconds timeout)
        {
            std::unique_lock lock(topics_mutex);
            default_timeout = timeout;
        }

        // Get queue size for a specific topic
        size_t queue_size(const std::string &topic_name) const
        {
            std::shared_lock lock(topics_mutex);
            auto it = topics.find(topic_name);
            if (it != topics.end())
            {
                return it->second.topic->queue_size();
            }
            return 0;
        }

        // Get queue capacity for a specific topic
        size_t queue_capacity(const std::string &topic_name) const
        {
            std::shared_lock lock(topics_mutex);
            auto it = topics.find(topic_name);
            if (it != topics.end())
            {
                return it->second.topic->queue_capacity();
            }
            return 0;
        }

        // Check if a topic's queue is full
        bool is_queue_full(const std::string &topic_name) const
        {
            std::shared_lock lock(topics_mutex);
            auto it = topics.find(topic_name);
            if (it != topics.end())
            {
                return it->second.topic->is_queue_full();
            }
            return false;
        }

        // Get the type_index for a topic
        std::optional<std::type_index> get_topic_type(const std::string &topic_name) const
        {
            std::shared_lock lock(topics_mutex);
            auto it = topics.find(topic_name);
            if (it != topics.end())
            {
                return it->second.type_id;
            }
            return std::nullopt;
        }
    };

} // namespace boblib::utils::pubsub