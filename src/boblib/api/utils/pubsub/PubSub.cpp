#include "PubSub.hpp"
#include <iostream>
#include <thread>
#include <chrono>

namespace boblib::utils::pubsub
{
    template <typename T>
        requires CopyableOrMovable<T>
    void PubSub<T>::dispatch_loop()
    {
        // Local cache of callbacks to avoid repeated allocations
        std::vector<SubscriberCallback> callbacks;
        callbacks.reserve(INITIAL_SUBSCRIBER_CAPACITY);

        while (running.load(std::memory_order_acquire))
        {
            bool processed = false;

            // Try to process messages
            auto message = queue.pop();
            if (message)
            {
                processed = true;

                // Only lock if we have subscribers
                if (cached_subscriber_count.load(std::memory_order_relaxed) > 0)
                {
                    // Copy subscribers list to avoid holding lock during callbacks
                    callbacks.clear();
                    {
                        std::lock_guard<std::mutex> lock(subscribers_mutex);
                        callbacks.reserve(subscribers.size());
                        for (const auto &[_, callback] : subscribers)
                        {
                            callbacks.push_back(callback);
                        }
                    }

                    // Notify all subscribers
                    for (const auto &callback : callbacks)
                    {
                        callback(*message);
                    }
                }
            }

            // If nothing was processed, wait for notification of new data
            if (!processed)
            {
                std::unique_lock<std::mutex> lock(cv_mutex);
                cv.wait_for(lock, timeout, [this]
                            { return !running.load(std::memory_order_acquire) ||
                                     queue.head.load(std::memory_order_acquire) !=
                                         queue.tail.load(std::memory_order_acquire); });
            }
        }
    }

    template <typename T>
        requires CopyableOrMovable<T>
    PubSub<T>::PubSub(size_t queue_size, std::chrono::milliseconds timeout_value)
        : queue(queue_size), timeout(timeout_value)
    {
        // Pre-allocate subscriber vector
        subscribers.reserve(INITIAL_SUBSCRIBER_CAPACITY);
        dispatch_thread = std::thread(&PubSub::dispatch_loop, this);
    }

    template <typename T>
        requires CopyableOrMovable<T>
    PubSub<T>::~PubSub()
    {
        running.store(false, std::memory_order_release);
        cv.notify_all();
        if (dispatch_thread.joinable())
        {
            dispatch_thread.join();
        }
    }

    template <typename T>
        requires CopyableOrMovable<T>
    bool PubSub<T>::publish(T message)
    {
        auto result = queue.push(std::move(message));
        if (result)
        {
            cv.notify_one();
        }
        return result;
    }

    template <typename T>
        requires CopyableOrMovable<T>
    typename PubSub<T>::SubscriberID PubSub<T>::subscribe(SubscriberCallback callback)
    {
        std::lock_guard<std::mutex> lock(subscribers_mutex);
        SubscriberID id = next_id.fetch_add(1, std::memory_order_relaxed);
        subscribers.emplace_back(id, std::move(callback));
        cached_subscriber_count.store(subscribers.size(), std::memory_order_release);
        return id;
    }

    template <typename T>
        requires CopyableOrMovable<T>
    bool PubSub<T>::unsubscribe(SubscriberID id)
    {
        std::lock_guard<std::mutex> lock(subscribers_mutex);
        auto it = std::find_if(subscribers.begin(), subscribers.end(),
                               [id](const auto &pair)
                               { return pair.first == id; });
        if (it != subscribers.end())
        {
            subscribers.erase(it);
            cached_subscriber_count.store(subscribers.size(), std::memory_order_release);
            return true;
        }
        return false;
    }

    template <typename T>
        requires CopyableOrMovable<T>
    size_t PubSub<T>::subscriber_count() const
    {
        // Use cached count for better performance
        return cached_subscriber_count.load(std::memory_order_relaxed);
    }

    template <typename T>
        requires CopyableOrMovable<T>
    bool PubSub<T>::is_queue_full() const
    {
        const size_t head = queue.head.load(std::memory_order_relaxed);
        const size_t next_head = (head + 1) % queue.capacity;
        return next_head == queue.tail.load(std::memory_order_acquire);
    }

    template <typename T>
        requires CopyableOrMovable<T>
    size_t PubSub<T>::queue_size() const
    {
        const size_t head = queue.head.load(std::memory_order_relaxed);
        const size_t tail = queue.tail.load(std::memory_order_relaxed);

        if (head >= tail)
        {
            return head - tail;
        }
        else
        {
            return queue.capacity - tail + head;
        }
    }

    template <typename T>
        requires CopyableOrMovable<T>
    size_t PubSub<T>::queue_capacity() const
    {
        return queue.capacity;
    }

    template <typename T>
        requires CopyableOrMovable<T>
    void PubSub<T>::set_timeout(std::chrono::milliseconds new_timeout)
    {
        timeout = new_timeout;
    }
}