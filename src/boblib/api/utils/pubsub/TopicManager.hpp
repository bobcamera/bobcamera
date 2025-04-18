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
    };

    // Concrete implementation for specific message types
    template <typename T>
    class TypedTopic : public TopicBase
    {
    private:
        std::shared_ptr<PubSub<T>> pubsub;

    public:
        explicit TypedTopic(std::shared_ptr<PubSub<T>> p) : pubsub(std::move(p)) {}

        size_t subscriber_count() const override { return pubsub->subscriber_count(); }
        size_t queue_size() const override { return pubsub->queue_size(); }
        size_t queue_capacity() const override { return pubsub->queue_capacity(); }
    };

    // TopicManager class to manage multiple topics with different message types
    class TopicManager
    {
    private:
        // Struct to hold topic data
        struct TopicData
        {
            std::unique_ptr<TopicBase> topic;
            std::type_index type_id;
            std::shared_ptr<void> pubsub;  // Store the PubSub instance directly

            TopicData(std::unique_ptr<TopicBase> t, std::type_index id, std::shared_ptr<void> p)
                : topic(std::move(t)), type_id(id), pubsub(std::move(p)) {}
        };

        // Map from topic name to topic data
        std::unordered_map<std::string, TopicData> topics;

        // Mutex to protect the topics map
        mutable std::shared_mutex topics_mutex;

        // Default queue size and timeout for new topics
        size_t default_queue_size;

    public:
        // Constructor with default values
        explicit TopicManager(
            size_t queue_size = 100)
            : default_queue_size(queue_size) {}

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
                auto pubsub = std::make_shared<PubSub<T>>(default_queue_size);
                auto typed_topic = std::make_unique<TypedTopic<T>>(pubsub);
                auto [new_it, _] = topics.emplace(
                    topic_name,
                    TopicData(std::move(typed_topic), typeid(T), pubsub));
                it = new_it;
                return pubsub;
            }
            else if (it->second.type_id != typeid(T))
            {
                // Topic exists but with a different type
                throw std::runtime_error("Topic '" + topic_name + "' exists with a different message type");
            }

            // Return the stored PubSub instance
            return std::static_pointer_cast<PubSub<T>>(it->second.pubsub);
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