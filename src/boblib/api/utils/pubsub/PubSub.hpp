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
#include <array>

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
            alignas(64) std::atomic<bool> occupied{false};
            alignas(64) std::aligned_storage_t<sizeof(T), alignof(T)> storage;
            alignas(64) std::atomic<bool> being_processed{false};
            char padding[64 - (sizeof(std::atomic<bool>) * 2 + sizeof(std::aligned_storage_t<sizeof(T), alignof(T)>)) % 64];

            Slot() = default;
            Slot(const Slot &) = delete;            // Prevent copying
            Slot &operator=(const Slot &) = delete; // Prevent assignment
            Slot(Slot &&) = delete;                 // Prevent moving
            Slot &operator=(Slot &&) = delete;

            ~Slot() noexcept
            {
                if (occupied.load(std::memory_order_acquire))
                {
                    get().~T(); // Call destructor if needed
                }
            }

            T &get() noexcept
            {
                return *reinterpret_cast<T *>(&storage);
            }

            const T &get() const noexcept
            {
                return *reinterpret_cast<const T *>(&storage);
            }

            template <typename... Args>
            void construct(Args &&...args) noexcept(noexcept(T(std::forward<Args>(args)...)))
            {
                new (&storage) T(std::forward<Args>(args)...);
            }

            // New method to mark slot as being processed
            bool try_acquire_processing() noexcept
            {
                bool expected = false;
                return being_processed.compare_exchange_strong(expected, true,
                                                               std::memory_order_acquire, std::memory_order_relaxed);
            }

            // New method to release processing flag
            void release_processing() noexcept
            {
                being_processed.store(false, std::memory_order_release);
            }
        };

        // Circular buffer with atomic indices
        struct CircularBuffer
        {
            alignas(64) std::unique_ptr<Slot[]> buffer;
            alignas(64) std::atomic<size_t> head{0};
            alignas(64) std::atomic<size_t> tail{0};
            alignas(64) size_t capacity;
            char padding[64 - (sizeof(std::atomic<size_t>) * 2 + sizeof(size_t)) % 64];

            explicit CircularBuffer(size_t size) : capacity(size)
            {
                buffer = std::make_unique<Slot[]>(size);
            }

            // Optimized push that avoids unnecessary copies
            bool push(T &&item) noexcept(noexcept(std::declval<Slot>().construct(std::move(item))))
            {
                const size_t current_head = head.load(std::memory_order_acquire);
                const size_t next_head = (current_head + 1) % capacity;

                if (next_head == tail.load(std::memory_order_acquire))
                {
                    return false;
                }

                buffer[current_head].construct(std::move(item));
                std::atomic_thread_fence(std::memory_order_release);
                buffer[current_head].occupied.store(true, std::memory_order_release);
                head.store(next_head, std::memory_order_release);
                return true;
            }

            // New method for zero-copy access to the next message
            template <typename F>
            bool process_next(F &&processor) noexcept(noexcept(processor(std::declval<T&>())))
            {
                const size_t current_tail = tail.load(std::memory_order_acquire);

                if (current_tail == head.load(std::memory_order_acquire))
                {
                    return false;
                }

                Slot &slot = buffer[current_tail];

                if (!slot.occupied.load(std::memory_order_acquire))
                {
                    return false;
                }

                if (!slot.try_acquire_processing())
                {
                    return false;
                }

                try
                {
                    processor(slot.get());
                    slot.get().~T();
                    slot.occupied.store(false, std::memory_order_release);
                    slot.release_processing();
                    tail.store((current_tail + 1) % capacity, std::memory_order_release);
                    return true;
                }
                catch (...)
                {
                    slot.release_processing();
                    throw;
                }
            }
        };

        using SubscriberCallback = std::function<void(const T &)>;
        using SubscriberID = size_t;

        CircularBuffer queue;
        
        // Fixed-size array for subscribers
        static constexpr size_t MAX_SUBSCRIBERS = 4;
        alignas(64) std::array<SubscriberCallback, MAX_SUBSCRIBERS> subscribers;
        alignas(64) std::atomic<size_t> active_subscriber_count{0};
        alignas(64) std::atomic<bool> running{true};
        alignas(64) std::thread dispatch_thread;
        alignas(64) std::condition_variable cv;
        alignas(64) std::mutex cv_mutex;
        
        // Add a shared mutex for thread-safe subscriber access
        alignas(64) std::shared_mutex subscribers_mutex;
        
        // Configurable timeout for condition variable
        static constexpr std::chrono::milliseconds DEFAULT_TIMEOUT{5};
        std::chrono::milliseconds timeout;

        void dispatch_loop() noexcept
        {
            // Remove the constant load of subscriber count
            // Spin for a short time before sleeping
            constexpr int SPIN_COUNT = 1000;
            int spin_attempts = 0;

            while (running.load(std::memory_order_acquire))
            {
                bool processed = false;
                // Get current count inside the loop to pick up new subscribers
                const size_t count = active_subscriber_count.load(std::memory_order_acquire);
                
                if (count > 0)
                {
                    // Create the lambda inside the loop using the current count
                    processed = queue.process_next([this, count](const T& message) noexcept {
                        // Acquire a shared lock when accessing subscribers
                        std::shared_lock<std::shared_mutex> lock(subscribers_mutex);
                        for (size_t i = 0; i < count; ++i) {
                            subscribers[i](message);
                        }
                    });
                }
                else
                {
                    processed = queue.process_next([](const T&) noexcept {});
                }

                if (!processed)
                {
                    if (spin_attempts < SPIN_COUNT)
                    {
                        spin_attempts++;
                        std::this_thread::yield();
                        continue;
                    }

                    spin_attempts = 0;
                    std::unique_lock<std::mutex> lock(cv_mutex);
                    cv.wait_for(lock, timeout, [this]() noexcept {
                        return !running.load(std::memory_order_acquire) ||
                               queue.head.load(std::memory_order_acquire) !=
                               queue.tail.load(std::memory_order_acquire);
                    });
                }
            }
        }

    public:
        explicit PubSub(size_t queue_size, 
                       std::chrono::milliseconds timeout_value = DEFAULT_TIMEOUT) noexcept
            : queue(queue_size), timeout(timeout_value)
        {
            // Always create a dispatch thread for each instance
            dispatch_thread = std::thread(&PubSub::dispatch_loop, this);
        }
        
        ~PubSub() noexcept
        {
            // Always shut down the dispatch thread for this instance
            running.store(false, std::memory_order_release);
            cv.notify_all();
            if (dispatch_thread.joinable())
            {
                dispatch_thread.join();
            }
            // Clear any remaining messages
            while (queue.process_next([](const T&){})) {}
        }

        // Publish a message to the queue
        bool publish(T &&message) noexcept(noexcept(queue.push(std::move(message))))
        {
            auto result = queue.push(std::move(message));
            if (result)
            {
                cv.notify_one();
            }
            return result;
        }

        // Subscribe to messages with a callback
        bool subscribe(SubscriberCallback callback) noexcept
        {
            // Use atomic fetch_add to safely get the current index and increment atomically
            const size_t index = active_subscriber_count.fetch_add(1, std::memory_order_acq_rel);
            if (index >= MAX_SUBSCRIBERS)
            {
                // Roll back the increment if we exceeded capacity
                active_subscriber_count.fetch_sub(1, std::memory_order_acq_rel);
                return false;
            }
            
            // Acquire an exclusive lock when modifying subscribers
            std::unique_lock<std::shared_mutex> lock(subscribers_mutex);
            subscribers[index] = std::move(callback);
            return true;
        }

        // Check if queue is full
        bool is_queue_full() const noexcept
        {
            const size_t head = queue.head.load(std::memory_order_acquire);
            const size_t next_head = (head + 1) % queue.capacity;
            return next_head == queue.tail.load(std::memory_order_acquire);
        }

        // Get current queue size
        size_t queue_size() const noexcept
        {
            const size_t head = queue.head.load(std::memory_order_acquire);
            const size_t tail = queue.tail.load(std::memory_order_acquire);

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
        size_t queue_capacity() const noexcept
        {
            return queue.capacity;
        }

        // Check if the queue is empty
        bool empty() const noexcept
        {
            return queue_size() == 0;
        }

        // Get the number of active subscribers
        size_t subscriber_count() const noexcept
        {
            return active_subscriber_count.load(std::memory_order_acquire);
        }

        // Method to configure the timeout
        void set_timeout(std::chrono::milliseconds new_timeout) noexcept
        {
            timeout = new_timeout;
        }
    };
}