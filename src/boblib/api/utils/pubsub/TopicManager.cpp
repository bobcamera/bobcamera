#include "TopicManager.hpp"
#include "TopicReference.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <algorithm>

namespace boblib::utils::pubsub
{
    // Implementation of PubSubWrapper methods
    template <typename T>
    TopicManager::PubSubWrapper<T>::PubSubWrapper(size_t queue_size, std::chrono::milliseconds timeout)
        : pubsub(queue_size, timeout) {}

    template <typename T>
    bool TopicManager::PubSubWrapper<T>::publish(const void *message)
    {
        if (message)
        {
            return pubsub.publish(*static_cast<const T *>(message));
        }
        return false;
    }

    template <typename T>
    size_t TopicManager::PubSubWrapper<T>::subscribe(std::function<void(const void *)> callback)
    {
        return pubsub.subscribe([callback](const T &msg)
                                { callback(&msg); });
    }

    template <typename T>
    bool TopicManager::PubSubWrapper<T>::unsubscribe(size_t id)
    {
        return pubsub.unsubscribe(id);
    }

    template <typename T>
    size_t TopicManager::PubSubWrapper<T>::subscriber_count() const
    {
        return pubsub.subscriber_count();
    }

    template <typename T>
    bool TopicManager::PubSubWrapper<T>::is_queue_full() const
    {
        return pubsub.is_queue_full();
    }

    template <typename T>
    size_t TopicManager::PubSubWrapper<T>::queue_size() const
    {
        return pubsub.queue_size();
    }

    template <typename T>
    size_t TopicManager::PubSubWrapper<T>::queue_capacity() const
    {
        return pubsub.queue_capacity();
    }

    template <typename T>
    void TopicManager::PubSubWrapper<T>::set_timeout(std::chrono::milliseconds timeout)
    {
        pubsub.set_timeout(timeout);
    }

    // Implementation of TopicManager methods
    TopicManager::TopicManager(size_t queue_size, std::chrono::milliseconds timeout)
        : default_queue_size(queue_size), default_timeout(timeout)
    {
    }

    template <typename T>
    TopicReference<T> TopicManager::get_topic_reference(const std::string &topic_name)
    {
        // Ensure the topic exists
        create_topic<T>(topic_name);
        return TopicReference<T>(*this, topic_name);
    }

    template <typename T>
    void TopicManager::create_topic(const std::string &topic_name)
    {
        // Use exclusive lock for writing
        std::unique_lock<std::shared_mutex> lock(topics_mutex);

        if (topics.find(topic_name) == topics.end())
        {
            topics[topic_name] = TopicInfo{
                std::make_shared<PubSubWrapper<T>>(default_queue_size, default_timeout),
                std::type_index(typeid(T)),
                std::atomic<uint64_t>{1} // Initial version
            };
        }
    }

    template <typename T>
    size_t TopicManager::subscribe(const std::string &topic_name, std::function<void(const T &)> callback)
    {
        // First try with a shared lock for reading
        {
            std::shared_lock<std::shared_mutex> lock(topics_mutex);

            auto it = topics.find(topic_name);
            if (it != topics.end())
            {
                // Check type using type_index instead of dynamic_cast
                if (it->second.type_id == std::type_index(typeid(T)))
                {
                    // Safe to cast without dynamic_cast since we've verified the type
                    auto *wrapper = static_cast<PubSubWrapper<T> *>(it->second.pubsub.get());
                    return wrapper->subscribe([callback](const void *msg)
                                              { callback(*static_cast<const T *>(msg)); });
                }
            }
        }

        // If topic doesn't exist or has wrong type, create it with exclusive lock
        std::unique_lock<std::shared_mutex> lock(topics_mutex);

        // Check again in case another thread created it while we were waiting
        auto it = topics.find(topic_name);
        if (it != topics.end())
        {
            if (it->second.type_id == std::type_index(typeid(T)))
            {
                auto *wrapper = static_cast<PubSubWrapper<T> *>(it->second.pubsub.get());
                return wrapper->subscribe([callback](const void *msg)
                                          { callback(*static_cast<const T *>(msg)); });
            }
        }

        // Create the topic if it doesn't exist
        create_topic<T>(topic_name);
        auto *wrapper = static_cast<PubSubWrapper<T> *>(topics[topic_name].pubsub.get());
        return wrapper->subscribe([callback](const void *msg)
                                  { callback(*static_cast<const T *>(msg)); });
    }

    bool TopicManager::unsubscribe(const std::string &topic_name, size_t subscriber_id)
    {
        // Use shared lock for reading
        std::shared_lock<std::shared_mutex> lock(topics_mutex);

        auto it = topics.find(topic_name);
        if (it != topics.end())
        {
            return it->second.pubsub->unsubscribe(subscriber_id);
        }

        return false;
    }

    template <typename T>
    bool TopicManager::publish(const std::string &topic_name, const T &message)
    {
        // First try with a shared lock for reading
        {
            std::shared_lock<std::shared_mutex> lock(topics_mutex);

            auto it = topics.find(topic_name);
            if (it != topics.end())
            {
                // Check type using type_index instead of dynamic_cast
                if (it->second.type_id == std::type_index(typeid(T)))
                {
                    // Safe to cast without dynamic_cast since we've verified the type
                    auto *wrapper = static_cast<PubSubWrapper<T> *>(it->second.pubsub.get());
                    return wrapper->publish(&message);
                }
            }
        }

        // If topic doesn't exist or has wrong type, create it with exclusive lock
        std::unique_lock<std::shared_mutex> lock(topics_mutex);

        // Check again in case another thread created it while we were waiting
        auto it = topics.find(topic_name);
        if (it != topics.end())
        {
            if (it->second.type_id == std::type_index(typeid(T)))
            {
                auto *wrapper = static_cast<PubSubWrapper<T> *>(it->second.pubsub.get());
                return wrapper->publish(&message);
            }
        }

        // Create the topic if it doesn't exist
        create_topic<T>(topic_name);
        auto *wrapper = static_cast<PubSubWrapper<T> *>(topics[topic_name].pubsub.get());
        return wrapper->publish(&message);
    }

    size_t TopicManager::subscriber_count(const std::string &topic_name) const
    {
        // Use shared lock for reading
        std::shared_lock<std::shared_mutex> lock(topics_mutex);

        auto it = topics.find(topic_name);
        if (it != topics.end())
        {
            return it->second.pubsub->subscriber_count();
        }

        return 0;
    }

    bool TopicManager::is_queue_full(const std::string &topic_name) const
    {
        // Use shared lock for reading
        std::shared_lock<std::shared_mutex> lock(topics_mutex);

        auto it = topics.find(topic_name);
        if (it != topics.end())
        {
            return it->second.pubsub->is_queue_full();
        }

        return false;
    }

    size_t TopicManager::queue_size(const std::string &topic_name) const
    {
        // Use shared lock for reading
        std::shared_lock<std::shared_mutex> lock(topics_mutex);

        auto it = topics.find(topic_name);
        if (it != topics.end())
        {
            return it->second.pubsub->queue_size();
        }

        return 0;
    }

    size_t TopicManager::queue_capacity(const std::string &topic_name) const
    {
        // Use shared lock for reading
        std::shared_lock<std::shared_mutex> lock(topics_mutex);

        auto it = topics.find(topic_name);
        if (it != topics.end())
        {
            return it->second.pubsub->queue_capacity();
        }

        return 0;
    }

    void TopicManager::set_timeout(const std::string &topic_name, std::chrono::milliseconds timeout)
    {
        // Use shared lock for reading
        std::shared_lock<std::shared_mutex> lock(topics_mutex);

        auto it = topics.find(topic_name);
        if (it != topics.end())
        {
            it->second.pubsub->set_timeout(timeout);
        }
    }

    void TopicManager::set_default_timeout(std::chrono::milliseconds timeout)
    {
        default_timeout = timeout;
    }

    void TopicManager::set_default_queue_size(size_t queue_size)
    {
        default_queue_size = queue_size;
    }

    bool TopicManager::topic_exists(const std::string &topic_name) const
    {
        // Use shared lock for reading
        std::shared_lock<std::shared_mutex> lock(topics_mutex);
        return topics.find(topic_name) != topics.end();
    }

    std::vector<std::string> TopicManager::get_topic_names() const
    {
        // Use shared lock for reading
        std::shared_lock<std::shared_mutex> lock(topics_mutex);
        
        std::vector<std::string> names;
        names.reserve(topics.size());
        
        for (const auto &[name, _] : topics)
        {
            names.push_back(name);
        }
        
        return names;
    }

    IPubSubBase* TopicManager::get_pubsub_ptr(const std::string &topic_name) const
    {
        // Use shared lock for reading
        std::shared_lock<std::shared_mutex> lock(topics_mutex);
        
        auto it = topics.find(topic_name);
        if (it != topics.end())
        {
            return it->second.pubsub.get();
        }
        
        return nullptr;
    }
}
