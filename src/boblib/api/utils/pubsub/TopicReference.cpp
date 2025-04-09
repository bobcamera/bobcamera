#include "TopicManager.hpp"
#include "TopicReference.hpp"
#include <iostream>
#include <thread>
#include <chrono>

namespace boblib::utils::pubsub
{
    // Implementation of TopicReference methods
    template <typename T>
    TopicReference<T>::TopicReference(TopicManager &mgr, const std::string &name)
        : manager(mgr), topic_name(name), type_id(typeid(T))
    {
        refresh();
    }

    template <typename T>
    void TopicReference<T>::refresh()
    {
        std::unique_lock<std::shared_mutex> lock(ref_mutex);
        if (manager.topic_exists(topic_name))
        {
            auto *base_ptr = manager.get_pubsub_ptr(topic_name);
            if (base_ptr)
            {
                auto *wrapper = dynamic_cast<typename TopicManager::PubSubWrapper<T> *>(base_ptr);
                if (wrapper)
                {
                    pubsub_ptr = wrapper;
                    valid.store(true, std::memory_order_release);
                    cached_version.store(wrapper->get_version(), std::memory_order_release);
                }
                else
                {
                    pubsub_ptr = nullptr;
                    valid.store(false, std::memory_order_release);
                }
            }
            else
            {
                pubsub_ptr = nullptr;
                valid.store(false, std::memory_order_release);
            }
        }
        else
        {
            pubsub_ptr = nullptr;
            valid.store(false, std::memory_order_release);
        }
    }

    template <typename T>
    bool TopicReference<T>::is_valid() const
    {
        return valid.load(std::memory_order_acquire);
    }

    template <typename T>
    bool TopicReference<T>::publish(const T &message)
    {
        std::shared_lock<std::shared_mutex> lock(ref_mutex);
        if (!valid.load(std::memory_order_acquire))
        {
            return false;
        }

        auto *current_ptr = pubsub_ptr;
        if (current_ptr)
        {
            return current_ptr->publish(&message);
        }

        // If we don't have a valid pointer, try to refresh
        lock.unlock();
        refresh();
        lock.lock();

        current_ptr = pubsub_ptr;
        if (current_ptr)
        {
            return current_ptr->publish(&message);
        }

        return false;
    }

    template <typename T>
    size_t TopicReference<T>::subscribe(std::function<void(const T &)> callback)
    {
        std::shared_lock<std::shared_mutex> lock(ref_mutex);
        if (!valid.load(std::memory_order_acquire))
        {
            return 0;
        }

        auto *wrapper = static_cast<typename TopicManager::PubSubWrapper<T> *>(pubsub_ptr);
        if (wrapper)
        {
            return wrapper->subscribe([callback](const void *msg)
                                     { callback(*static_cast<const T *>(msg)); });
        }

        return 0;
    }

    template <typename T>
    bool TopicReference<T>::unsubscribe(size_t id)
    {
        std::shared_lock<std::shared_mutex> lock(ref_mutex);
        if (!valid.load(std::memory_order_acquire))
        {
            return false;
        }

        auto *current_ptr = pubsub_ptr;
        if (current_ptr)
        {
            return current_ptr->unsubscribe(id);
        }

        return false;
    }

    template <typename T>
    size_t TopicReference<T>::subscriber_count() const
    {
        std::shared_lock<std::shared_mutex> lock(ref_mutex);
        if (!valid.load(std::memory_order_acquire))
        {
            return 0;
        }

        auto *current_ptr = pubsub_ptr;
        if (current_ptr)
        {
            return current_ptr->subscriber_count();
        }

        return 0;
    }

    template <typename T>
    bool TopicReference<T>::is_queue_full() const
    {
        std::shared_lock<std::shared_mutex> lock(ref_mutex);
        if (!valid.load(std::memory_order_acquire))
        {
            return true;
        }

        auto *current_ptr = pubsub_ptr;
        if (current_ptr)
        {
            return current_ptr->is_queue_full();
        }

        return true;
    }

    template <typename T>
    size_t TopicReference<T>::queue_size() const
    {
        std::shared_lock<std::shared_mutex> lock(ref_mutex);
        if (!valid.load(std::memory_order_acquire))
        {
            return 0;
        }

        auto *current_ptr = pubsub_ptr;
        if (current_ptr)
        {
            return current_ptr->queue_size();
        }

        return 0;
    }

    template <typename T>
    size_t TopicReference<T>::queue_capacity() const
    {
        std::shared_lock<std::shared_mutex> lock(ref_mutex);
        if (!valid.load(std::memory_order_acquire))
        {
            return 0;
        }

        auto *current_ptr = pubsub_ptr;
        if (current_ptr)
        {
            return current_ptr->queue_capacity();
        }

        return 0;
    }

    template <typename T>
    void TopicReference<T>::set_timeout(std::chrono::milliseconds timeout)
    {
        std::shared_lock<std::shared_mutex> lock(ref_mutex);
        if (!valid.load(std::memory_order_acquire))
        {
            return;
        }

        auto *current_ptr = pubsub_ptr;
        if (current_ptr)
        {
            current_ptr->set_timeout(timeout);
        }
    }
} 