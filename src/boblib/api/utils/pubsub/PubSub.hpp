#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <thread>
#include <optional>
#include <type_traits>
#include <concepts>
#include <mutex>
#include <shared_mutex>
#include <condition_variable>
#include <chrono>
#include <array>

namespace boblib::utils::pubsub
{
    // Define concepts for message types
    template <typename T>
    concept CopyableOrMovable = std::is_copy_constructible_v<T> || std::is_move_constructible_v<T>;

    template <typename T, size_t MaxSubscribers = 4>
        requires CopyableOrMovable<T>
    class PubSub
    {
    private:
        static constexpr size_t CacheLineSize = 64;

        // Structure representing a single slot in our circular buffer
        struct Slot
        {
            alignas(CacheLineSize) std::atomic<bool> occupied{false};
            alignas(CacheLineSize) std::aligned_storage_t<sizeof(T), alignof(T)> storage;
            alignas(CacheLineSize) std::atomic<bool> being_processed{false};
            char padding[CacheLineSize - (sizeof(std::atomic<bool>) * 2 + sizeof(std::aligned_storage_t<sizeof(T), alignof(T)>)) % CacheLineSize];

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
            alignas(CacheLineSize) std::unique_ptr<Slot[]> buffer;
            alignas(CacheLineSize) std::atomic<size_t> head{0};
            alignas(CacheLineSize) std::atomic<size_t> tail{0};
            alignas(CacheLineSize) size_t capacity;
            char padding[CacheLineSize - (sizeof(std::atomic<size_t>) * 2 + sizeof(size_t)) % CacheLineSize];

            explicit CircularBuffer(size_t size) : capacity(size)
            {
                buffer = std::make_unique<Slot[]>(size);
            }

            // Optimized push that avoids unnecessary copies
            bool push(T &&item) noexcept(noexcept(std::declval<Slot>().construct(std::move(item))))
            {
                const size_t current_head = head.load(std::memory_order_acquire);
                const size_t next_head = (current_head + 1) % capacity;

                // Prefetch the next slot to improve cache locality
                __builtin_prefetch(&buffer[current_head], 1, 3); // Write with high temporal locality

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
            bool process_next(F &&processor) noexcept(noexcept(processor(std::declval<T &>())))
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
        using SubscriberID = size_t; // Represents the index in the subscribers array

        CircularBuffer queue;

        // Fixed-size array for subscribers
        alignas(CacheLineSize) std::array<SubscriberCallback, MaxSubscribers> subscribers;
        alignas(CacheLineSize) std::atomic<size_t> active_subscriber_count{0};
        alignas(CacheLineSize) std::atomic<bool> running{true};
        alignas(CacheLineSize) std::thread dispatch_thread;
        alignas(CacheLineSize) std::condition_variable cv;
        alignas(CacheLineSize) std::mutex cv_mutex;

        // Add a shared mutex for thread-safe subscriber access
        alignas(CacheLineSize) std::shared_mutex subscribers_mutex;

        // Configurable timeout for condition variable
        static constexpr std::chrono::milliseconds DEFAULT_TIMEOUT{5};
        std::chrono::milliseconds timeout;

        void dispatch_loop() noexcept
        {
            // Remove fixed spin count in favor of exponential backoff
            std::chrono::microseconds backoff_time{1};
            constexpr std::chrono::microseconds max_backoff{1000}; // 1ms max

            while (running.load(std::memory_order_acquire))
            {
                bool processed = false;
                // Get current count inside the loop to pick up new subscribers
                const size_t count = active_subscriber_count.load(std::memory_order_acquire);

                if (count > 0)
                {
                    // Create the lambda inside the loop using the current count
                    processed = queue.process_next([this, count](const T &message) noexcept
                                                   {
                            //static_assert(noexcept(processor(std::declval<T&>())), "Processor function must be noexcept for real-time operation");
                            std::shared_lock<std::shared_mutex> lock(subscribers_mutex);
                            for (size_t i = 0; i < count; ++i) 
                            {
                                subscribers[i](message);
                            } });
                }
                else
                {
                    // Still process (and discard) the message if there are no subscribers to keep the queue moving.
                    processed = queue.process_next([](const T &) noexcept {});
                }

                if (!processed)
                {
                    if (backoff_time < max_backoff)
                    {
                        std::this_thread::sleep_for(backoff_time);
                        backoff_time = std::min(backoff_time * 2, max_backoff);
                    }
                    else
                    {
                        // Reset backoff when we've reached maximum and need to wait longer
                        backoff_time = std::chrono::microseconds{1};
                        std::unique_lock<std::mutex> lock(cv_mutex);
                        cv.wait_for(lock, timeout);
                    }
                }
                else
                {
                    // Reset backoff time when we successfully processed something
                    backoff_time = std::chrono::microseconds{1};
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
            while (queue.process_next([](const T &) {}))
            {
            }
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
        // Returns an optional SubscriberID (the index) on success
        std::optional<SubscriberID> subscribe(SubscriberCallback callback) noexcept
        {
            std::unique_lock<std::shared_mutex> lock(subscribers_mutex);

            // Since we have a lock, use simple counter logic
            size_t current_count = active_subscriber_count.load(std::memory_order_relaxed);
            if (current_count >= MaxSubscribers)
            {
                return std::nullopt;
            }

            subscribers[current_count] = std::move(callback);
            active_subscriber_count.store(current_count + 1, std::memory_order_relaxed);
            return current_count;
        }

        // Unsubscribe using the ID obtained from subscribe
        bool unsubscribe(SubscriberID id) noexcept
        {
            // Acquire an exclusive lock to modify the subscribers array and count
            std::unique_lock<std::shared_mutex> lock(subscribers_mutex);

            size_t current_count = active_subscriber_count.load(std::memory_order_acquire);

            // Validate the ID: must be within the bounds of currently active subscribers
            if (id >= current_count)
            {
                return false; // Invalid ID or ID points beyond the last active subscriber
            }

            // Find the index of the last *actual* subscriber
            size_t last_active_index = current_count - 1;

            // If the ID to remove is not the last one, swap it with the last one
            if (id < last_active_index)
            {
                subscribers[id] = std::move(subscribers[last_active_index]);
            }

            // Clear the (now unused) last slot
            subscribers[last_active_index] = nullptr; // Or {}

            // Atomically decrement the count
            active_subscriber_count.fetch_sub(1, std::memory_order_release);

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