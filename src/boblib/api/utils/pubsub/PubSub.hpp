#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <thread>
#include <vector>
#include <optional>
#include <iostream>
#include <type_traits>
#include <concepts>
#include <mutex>
#include <shared_mutex>
#include <condition_variable>
#include <chrono>
#include <unordered_map>
#include <typeindex>

namespace boblib::utils::pubsub
{
    // Define concepts for message types
    template <typename T>
    concept CopyableOrMovable = std::is_copy_constructible_v<T> || std::is_move_constructible_v<T>;

    template <typename T>
        requires CopyableOrMovable<T>
    class PubSub
    {
    private:
        // Structure representing a single slot in our circular buffer
        struct Slot
        {
            std::atomic<bool> occupied{false};
            alignas(64) std::aligned_storage_t<sizeof(T), alignof(T)> storage; // Raw storage instead of T directly

            Slot() = default;
            Slot(const Slot &) = delete;            // Prevent copying
            Slot &operator=(const Slot &) = delete; // Prevent assignment
            Slot(Slot &&) = delete;                 // Prevent moving
            Slot &operator=(Slot &&) = delete;

            ~Slot()
            {
                if (occupied.load(std::memory_order_relaxed))
                {
                    get().~T(); // Call destructor if needed
                }
            }

            T &get()
            {
                return *reinterpret_cast<T *>(&storage);
            }

            const T &get() const
            {
                return *reinterpret_cast<const T *>(&storage);
            }

            template <typename... Args>
            void construct(Args &&...args)
            {
                new (&storage) T(std::forward<Args>(args)...);
            }
        };

        // Circular buffer with atomic indices
        struct CircularBuffer
        {
            std::unique_ptr<Slot[]> buffer;
            std::atomic<size_t> head{0}; // Position to write to
            std::atomic<size_t> tail{0}; // Position to read from
            size_t capacity;

            explicit CircularBuffer(size_t size) : capacity(size)
            {
                buffer = std::make_unique<Slot[]>(size);
            }

            bool push(T &&item)
            {
                // Use relaxed ordering for local reads
                const size_t current_head = head.load(std::memory_order_relaxed);
                const size_t next_head = (current_head + 1) % capacity;

                // Use acquire to ensure we see the latest tail value
                if (next_head == tail.load(std::memory_order_acquire))
                {
                    return false;
                }

                // Place the item at the current head position
                if constexpr (std::is_move_constructible_v<T>)
                {
                    buffer[current_head].construct(std::move(item));
                }
                else
                {
                    buffer[current_head].construct(item);
                }

                // Use release to ensure the item is fully constructed before it's visible
                buffer[current_head].occupied.store(true, std::memory_order_release);

                // Update head
                head.store(next_head, std::memory_order_release);
                return true;
            }

            std::optional<T> pop()
            {
                // Use relaxed ordering for local reads
                const size_t current_tail = tail.load(std::memory_order_relaxed);

                // Use acquire to ensure we see the latest head value
                if (current_tail == head.load(std::memory_order_acquire))
                {
                    return std::nullopt;
                }

                // Use acquire to ensure we see the latest occupied value
                if (!buffer[current_tail].occupied.load(std::memory_order_acquire))
                {
                    return std::nullopt;
                }

                // Get the item
                T result;
                if constexpr (std::is_move_constructible_v<T>)
                {
                    result = std::move(buffer[current_tail].get());
                }
                else
                {
                    result = buffer[current_tail].get();
                }

                buffer[current_tail].get().~T();
                
                // Use release to ensure the slot is marked as unoccupied before updating tail
                buffer[current_tail].occupied.store(false, std::memory_order_release);
                tail.store((current_tail + 1) % capacity, std::memory_order_release);
                return result;
            }
        };

        using SubscriberCallback = std::function<void(const T &)>;
        using SubscriberID = size_t;

        CircularBuffer queue;
        
        // Pre-allocate subscriber vector to avoid reallocations
        static constexpr size_t INITIAL_SUBSCRIBER_CAPACITY = 8;
        std::vector<std::pair<SubscriberID, SubscriberCallback>> subscribers;
        std::mutex subscribers_mutex;
        
        // Cache the subscriber count to avoid locking when possible
        std::atomic<size_t> cached_subscriber_count{0};

        std::atomic<bool> running{true};
        std::thread dispatch_thread;
        std::condition_variable cv;
        std::mutex cv_mutex;

        std::atomic<SubscriberID> next_id{0};
        
        // Configurable timeout for condition variable
        static constexpr std::chrono::milliseconds DEFAULT_TIMEOUT{5};
        std::chrono::milliseconds timeout;

        void dispatch_loop()
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

    public:
        explicit PubSub(size_t queue_size, std::chrono::milliseconds timeout_value = DEFAULT_TIMEOUT)
            : queue(queue_size), timeout(timeout_value)
        {
            // Pre-allocate subscriber vector
            subscribers.reserve(INITIAL_SUBSCRIBER_CAPACITY);
            dispatch_thread = std::thread(&PubSub::dispatch_loop, this);
        }
        
        ~PubSub()
        {
            running.store(false, std::memory_order_release);
            cv.notify_all();
            if (dispatch_thread.joinable())
            {
                dispatch_thread.join();
            }
        }

        // Publish a message to the queue
        bool publish(T message)
        {
            auto result = queue.push(std::move(message));
            if (result)
            {
                cv.notify_one();
            }
            return result;
        }

        // Subscribe to messages with a callback
        SubscriberID subscribe(SubscriberCallback callback)
        {
            std::lock_guard<std::mutex> lock(subscribers_mutex);
            SubscriberID id = next_id.fetch_add(1, std::memory_order_relaxed);
            subscribers.emplace_back(id, std::move(callback));
            cached_subscriber_count.store(subscribers.size(), std::memory_order_release);
            return id;
        }

        // Unsubscribe using the ID returned from subscribe
        bool unsubscribe(SubscriberID id)
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

        // Get the number of subscribers
        size_t subscriber_count() const
        {
            // Use cached count for better performance
            return cached_subscriber_count.load(std::memory_order_relaxed);
        }

        // Check if queue is full
        bool is_queue_full() const
        {
            const size_t head = queue.head.load(std::memory_order_relaxed);
            const size_t next_head = (head + 1) % queue.capacity;
            return next_head == queue.tail.load(std::memory_order_acquire);
        }

        // Get current queue size
        size_t queue_size() const
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

        // Get queue capacity
        size_t queue_capacity() const
        {
            return queue.capacity;
        }

        // Method to configure the timeout
        void set_timeout(std::chrono::milliseconds new_timeout)
        {
            timeout = new_timeout;
        }
    };
}