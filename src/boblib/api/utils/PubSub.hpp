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
#include <condition_variable>

namespace boblib::utils
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
                const size_t current_head = head.load(std::memory_order_relaxed);
                const size_t next_head = (current_head + 1) % capacity;

                // Check if buffer is full
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

                // Mark as occupied
                buffer[current_head].occupied.store(true, std::memory_order_release);

                // Update head
                head.store(next_head, std::memory_order_release);
                return true;
            }

            std::optional<T> pop()
            {
                const size_t current_tail = tail.load(std::memory_order_relaxed);

                // Check if buffer is empty
                if (current_tail == head.load(std::memory_order_acquire))
                {
                    return std::nullopt;
                }

                // Check if slot is ready to be read
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

                // Destroy the object
                buffer[current_tail].get().~T();

                // Mark as unoccupied
                buffer[current_tail].occupied.store(false, std::memory_order_release);

                // Update tail
                tail.store((current_tail + 1) % capacity, std::memory_order_release);
                return result;
            }
        };

        using SubscriberCallback = std::function<void(const T &)>;
        using SubscriberID = size_t;

        CircularBuffer queue;
        std::vector<std::pair<SubscriberID, SubscriberCallback>> subscribers;
        std::mutex subscribers_mutex; // Only for subscriber management, not for the queue

        std::atomic<bool> running{true};
        std::thread dispatch_thread;
        std::condition_variable cv;
        std::mutex cv_mutex;

        std::atomic<SubscriberID> next_id{0};

        void dispatch_loop()
        {
            while (running.load(std::memory_order_acquire))
            {
                bool processed = false;

                // Try to process messages
                auto message = queue.pop();
                if (message)
                {
                    processed = true;
                    // Copy subscribers list to avoid holding lock during callbacks
                    std::vector<SubscriberCallback> callbacks;
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

                // If nothing was processed, wait for notification of new data
                if (!processed)
                {
                    std::unique_lock<std::mutex> lock(cv_mutex);
                    cv.wait_for(lock, std::chrono::milliseconds(10), [this]
                                { return !running.load(std::memory_order_acquire) ||
                                         queue.head.load(std::memory_order_acquire) !=
                                             queue.tail.load(std::memory_order_acquire); });
                }
            }
        }

    public:
        explicit PubSub(size_t queue_size) : queue(queue_size)
        {
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
                return true;
            }
            return false;
        }

        // Get the number of subscribers
        size_t subscriber_count() const
        {
            std::lock_guard<std::mutex> lock(subscribers_mutex);
            return subscribers.size();
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
    };

// Example usage
#if 0
int main() {
    // Create a publisher with a queue of 100 messages
    PubSub<std::string> pubsub(100);
    
    // Subscribe to messages
    auto id1 = pubsub.subscribe([](const std::string& msg) {
        std::cout << "Subscriber 1 received: " << msg << std::endl;
    });
    
    auto id2 = pubsub.subscribe([](const std::string& msg) {
        std::cout << "Subscriber 2 received: " << msg << std::endl;
    });
    
    // Publish some messages
    pubsub.publish("Hello");
    pubsub.publish("World");
    
    // Unsubscribe the first subscriber
    pubsub.unsubscribe(id1);
    
    // Publish another message
    pubsub.publish("After unsubscribe");
    
    // Sleep to allow processing
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    return 0;
}
#endif
}