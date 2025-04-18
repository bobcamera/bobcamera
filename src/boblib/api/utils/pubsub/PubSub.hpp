#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <thread>
#include <optional>
#include <type_traits>
#include <concepts>
#include <array>
#include <cstddef>
#include <algorithm>
#include <iostream>
#include <immintrin.h> // for _mm_pause

namespace boblib::utils::pubsub
{
    // ------------------------------------------------------------------ helpers
    template <typename T>
    concept CopyableOrMovable = std::movable<T>;

    [[nodiscard]] constexpr std::size_t next_power_of_two(std::size_t n) noexcept
    {
        if (n < 2)
            return 2;
        --n;
        for (std::size_t s = 1; s < sizeof(std::size_t) * 8; s <<= 1)
            n |= n >> s;
        return ++n;
    }

    // ------------------------------------------------------------------ PubSub
    template <typename T, std::size_t MaxSubscribers = 5>
        requires CopyableOrMovable<T>
    class PubSub
    {
        static_assert(std::is_nothrow_destructible_v<T>, "T must be noexcept‑destructible");

    private:
        // ---------------------------- cache‑line padded atomic wrapper
        struct alignas(64) PaddedAtomic
        {
            std::atomic<std::size_t> v{0};
        };

        // ---------------------------- slot with Lamport sequence number
        struct Slot
        {
            std::aligned_storage_t<sizeof(T), alignof(T)> storage;
            std::atomic<std::size_t> seq{0};

            T &get() noexcept { return *reinterpret_cast<T *>(&storage); }
            const T &get() const noexcept { return *reinterpret_cast<const T *>(&storage); }
        };

        // ---------------------------- single‑producer / single‑consumer ring
        struct CircularBuffer
        {
            explicit CircularBuffer(std::size_t requested)
                : capacity(next_power_of_two(requested)),
                  buffer(std::make_unique<Slot[]>(capacity)),
                  mask(capacity - 1)
            {
                // initialise sequence numbers
                for (std::size_t i = 0; i < capacity; ++i)
                    buffer[i].seq.store(i, std::memory_order_relaxed);
            }

            // publish ------------------------------------------------------
            bool push(T &&item) noexcept
            {
                const std::size_t head_idx = head.v.load(std::memory_order_relaxed);
                Slot &s = buffer[head_idx & mask];

                if (s.seq.load(std::memory_order_acquire) != head_idx) // queue full
                    return false;

                new (&s.storage) T(std::move(item));
                s.seq.store(head_idx + 1, std::memory_order_release);
                head.v.store(head_idx + 1, std::memory_order_relaxed);
                return true;
            }

            // consume ------------------------------------------------------
            template <typename F>
            bool pop(F &&proc) noexcept
            {
                const std::size_t tail_idx = tail.v.load(std::memory_order_relaxed);
                Slot &s = buffer[tail_idx & mask];

                if (s.seq.load(std::memory_order_acquire) != tail_idx + 1)
                    return false; // empty

                proc(s.get());
                s.get().~T();
                s.seq.store(tail_idx + capacity, std::memory_order_release);
                tail.v.store(tail_idx + 1, std::memory_order_relaxed);
                return true;
            }

            // data ---------------------------------------------------------
            const std::size_t capacity;
            std::unique_ptr<Slot[]> buffer;
            const std::size_t mask;
            PaddedAtomic head; // producer‑only write
            PaddedAtomic tail; // consumer‑only write
        };

        // ---------------------------- subscriber bookkeeping
        using Callback = void (*)(const T &, void *);
        struct Handler
        {
            Callback fn{nullptr};
            void *ctx{nullptr};
        };

        CircularBuffer queue;
        std::array<Handler, MaxSubscribers> subscribers{};
        std::atomic<std::size_t> active_cnt{0};

        std::atomic<bool> running{true};
        std::thread dispatch_thread;

        // ---------------------------- dispatch loop
        void dispatch_loop() noexcept
        {
            constexpr int kSpin = 64;
            while (running.load(std::memory_order_relaxed))
            {
                bool progressed = queue.pop(
                        [this](const T &msg) noexcept
                        {
                            const std::size_t cnt = active_cnt.load(std::memory_order_acquire);
                            for (std::size_t i = 0; i < cnt; ++i)
                                if (auto fn = subscribers[i].fn)
                                    fn(msg, subscribers[i].ctx); 
                        });

                if (!progressed)
                {
                    for (int i = 0; i < kSpin; ++i)
                        _mm_pause();
                    std::this_thread::yield();
                }
            }
        }

    public:
        explicit PubSub(std::size_t queue_size = 128)
            : queue(queue_size)
        {
            dispatch_thread = std::thread(&PubSub::dispatch_loop, this);
        }

        ~PubSub() noexcept
        {
            running.store(false, std::memory_order_relaxed);
            if (dispatch_thread.joinable())
                dispatch_thread.join();
            while (queue.pop([](const T &) noexcept {}))
            {
            }
        }

        // ---------------------------- publish
        [[gnu::always_inline]]
        bool publish(T &&msg) noexcept
        {
            return queue.push(std::move(msg));
        }

        // ---------------------------- subscribe
        std::optional<std::size_t> subscribe(Callback fn, void *ctx = nullptr) noexcept
        {
            std::size_t idx = active_cnt.load(std::memory_order_acquire);
            if (idx >= MaxSubscribers)
                return std::nullopt;

            subscribers[idx] = Handler{fn, ctx};
            std::atomic_thread_fence(std::memory_order_release);
            active_cnt.store(idx + 1, std::memory_order_release);
            return idx;
        }

        // ---------------------------- unsubscribe
        bool unsubscribe(std::size_t id) noexcept
        {
            std::size_t cnt = active_cnt.load(std::memory_order_acquire);
            if (id >= cnt)
                return false;

            subscribers[id].fn = nullptr; // 1. quiesce
            std::atomic_thread_fence(std::memory_order_release);

            subscribers[id] = subscribers[cnt - 1];
            subscribers[cnt - 1] = Handler{};
            active_cnt.store(cnt - 1, std::memory_order_release);
            return true;
        }

        // ---------------------------- stats helpers
        [[nodiscard]] std::size_t queue_capacity() const noexcept { return queue.capacity; }
        [[nodiscard]] std::size_t queue_size() const noexcept
        {
            const auto head_idx = queue.head.v.load(std::memory_order_relaxed);
            const auto tail_idx = queue.tail.v.load(std::memory_order_relaxed);
            return (head_idx - tail_idx) & queue.mask;
        }
        [[nodiscard]] bool empty() const noexcept { return queue_size() == 0; }
        [[nodiscard]] std::size_t subscriber_count() const noexcept { return active_cnt.load(std::memory_order_relaxed); }
    };

} // namespace boblib::utils::pubsub
