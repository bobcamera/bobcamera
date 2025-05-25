#pragma once

#include <atomic>
#include <memory>
#include <thread>
#include <stop_token>
#include <optional>
#include <type_traits>
#include <concepts>
#include <array>
#include <vector>
#include <cstddef>

#if defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86)
#include <immintrin.h>
static inline void cpu_relax() noexcept { _mm_pause(); }
#elif defined(__aarch64__) || defined(__ARM_ARCH)
static inline void cpu_relax() noexcept { __asm__ __volatile__("yield" ::: "memory"); }
#else
static inline void cpu_relax() noexcept { std::this_thread::yield(); }
#endif

namespace boblib::utils::pubsub
{
    // ------------------------------------------------------------------ helpers
    template <typename T, typename Msg, void (T::*Method)(const Msg &) noexcept>
    struct MemberCallback
    {
        static void callback(const Msg &msg, void *ctx) noexcept
        {
            (static_cast<T *>(ctx)->*Method)(msg);
        }
    };

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

    struct QueueStats
    {
        std::size_t central_queue_size;
        std::size_t total_subscriber_queue_size;
        std::size_t max_subscriber_queue_size;
        std::size_t min_subscriber_queue_size;
        std::size_t active_subscribers;
    };
    
    // ------------------------------------------------------------------ PubSub
    template <typename M, std::size_t MaxSubscribers = 5>
        requires CopyableOrMovable<M>
    class PubSub
    {
        // internally we always queue/copy shared_ptr<M>
        using T = std::shared_ptr<M>;

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
                  mask(capacity - 1),
                  dropped_count(0)
            {
                for (std::size_t i = 0; i < capacity; ++i)
                    buffer[i].seq.store(i, std::memory_order_relaxed);
            }

            // push an item into the ring buffer, dropping oldest if full
            // returns true if successful, false if item was dropped
            void push(T &&item) noexcept
            {
                const auto head_idx = head.v.load(std::memory_order_relaxed);
                Slot &s = buffer[head_idx & mask];

                if (s.seq.load(std::memory_order_acquire) != head_idx)
                {
                    // Buffer is full - drop the oldest item
                    const auto tail_idx = tail.v.load(std::memory_order_relaxed);
                    Slot &tail_slot = buffer[tail_idx & mask];

                    // Force pop the oldest item
                    if (tail_slot.seq.load(std::memory_order_acquire) == tail_idx + 1)
                    {
                        tail_slot.get().~T();
                        tail_slot.seq.store(tail_idx + capacity, std::memory_order_release);
                        tail.v.store(tail_idx + 1, std::memory_order_relaxed);
                        dropped_count.fetch_add(1, std::memory_order_relaxed);
                    }
                }

                // Now push the new item
                new (&s.storage) T(std::move(item));
                s.seq.store(head_idx + 1, std::memory_order_release);
                head.v.store(head_idx + 1, std::memory_order_relaxed);
            }

            // copy‑push overload for copyable T (eg shared_ptr<Msg>)
            void push(const T &item) noexcept
            {
                const auto head_idx = head.v.load(std::memory_order_relaxed);
                Slot &s = buffer[head_idx & mask];

                if (s.seq.load(std::memory_order_acquire) != head_idx)
                {
                    // Buffer is full - drop the oldest item
                    const auto tail_idx = tail.v.load(std::memory_order_relaxed);
                    Slot &tail_slot = buffer[tail_idx & mask];

                    // Force pop the oldest item
                    if (tail_slot.seq.load(std::memory_order_acquire) == tail_idx + 1)
                    {
                        tail_slot.get().~T();
                        tail_slot.seq.store(tail_idx + capacity, std::memory_order_release);
                        tail.v.store(tail_idx + 1, std::memory_order_relaxed);
                        dropped_count.fetch_add(1, std::memory_order_relaxed);
                    }
                }

                // Now push the new item
                new (&s.storage) T(item);
                s.seq.store(head_idx + 1, std::memory_order_release);
                head.v.store(head_idx + 1, std::memory_order_relaxed);
            }

            std::optional<T> try_pop_item() noexcept
            {
                const auto tail_idx = tail.v.load(std::memory_order_relaxed);
                Slot &s = buffer[tail_idx & mask];
                if (s.seq.load(std::memory_order_acquire) != tail_idx + 1)
                    return std::nullopt; // empty
                T item = std::move(s.get());
                s.get().~T();
                s.seq.store(tail_idx + capacity, std::memory_order_release);
                tail.v.store(tail_idx + 1, std::memory_order_relaxed);
                return item;
            }

            // ---------------------------- stats
            [[nodiscard]] std::size_t queue_size() const noexcept
            {
                // Properly handle wraparound for circular buffer
                auto head_idx = head.v.load(std::memory_order_acquire);
                auto tail_idx = tail.v.load(std::memory_order_acquire);

                // Since we use a power-of-2 capacity, we can safely subtract
                // and the result will be correct even with wraparound
                auto size = head_idx - tail_idx;

                // Clamp to capacity to handle any race conditions
                return std::min(size, capacity);
            }

            [[nodiscard]] std::size_t get_dropped_count() const noexcept
            {
                return dropped_count.load(std::memory_order_relaxed);
            }

            void reset_dropped_count() noexcept
            {
                dropped_count.store(0, std::memory_order_relaxed);
            }

            const std::size_t capacity;
            std::unique_ptr<Slot[]> buffer;
            const std::size_t mask;
            PaddedAtomic head, tail;
            std::atomic<std::size_t> dropped_count;
        };

        using Callback = void (*)(const T &, void *);

        // ---------------------------- per‐subscriber state
        struct alignas(64) Subscriber
        {
            Callback fn;
            void *ctx;
            CircularBuffer local_q;
            std::jthread thr;

            Subscriber(Callback f, void *c, std::size_t qsize)
                : fn(f), ctx(c), local_q(qsize),
                  thr([this](std::stop_token stop)
                      {
                        constexpr int kSpin = 64;
                        while (!stop.stop_requested())
                        {
                            if (auto opt = local_q.try_pop_item())
                            {
                                fn(*opt, ctx);
                            }
                            else
                            {
                                for (int i = 0; i < kSpin; ++i)
                                    cpu_relax();
                                std::this_thread::yield();
                            }
                        } })
            {
            }
        };

        // central queue + subscribers
        CircularBuffer queue;
        std::array<std::optional<Subscriber>, MaxSubscribers> subscribers;
        std::atomic<std::size_t> active_cnt{0};
        std::jthread dispatch_thread;

        // ---------------------------- dispatch loop
        void dispatch_loop(std::stop_token stop) noexcept
        {
            constexpr int kSpin = 64;
            int backoff = 1;
            while (!stop.stop_requested())
            {
                if (auto opt = queue.try_pop_item())
                {
                    backoff = 1;
                    auto cnt = active_cnt.load(std::memory_order_acquire);

                    // copy the shared_ptr into each subscriber
                    for (size_t i = 0; i < cnt; ++i)
                        subscribers[i]->local_q.push(*opt);
                }
                else
                {
                    // exponential pause then yield
                    for (int i = 0; i < backoff; ++i)
                        cpu_relax();
                    backoff = std::min(backoff << 1, kSpin);
                    if (backoff == kSpin)
                        std::this_thread::yield();
                }
            }
        }

    public:
        explicit PubSub(std::size_t queue_size = 128)
            : queue(queue_size)
        {
            dispatch_thread = std::jthread(
                [this](std::stop_token st)
                { dispatch_loop(st); });
        }

        ~PubSub() noexcept
        {
            // drain central queue
            while (queue.try_pop_item())
            {
            }
            // stop dispatch thread
            dispatch_thread.request_stop();
            // destroy subscribers (jthread destructor joins)
            for (std::size_t i = 0, n = active_cnt.load(); i < n; ++i)
                subscribers[i].reset();
        }

        // overload for raw M → we wrap in shared_ptr
        void publish(M &&raw) noexcept
        {
            queue.push(std::make_shared<M>(std::move(raw)));
        }

        // overload for when you already have a shared_ptr<M>
        void publish(const T &msg) noexcept
        {
            queue.push(msg);
        }

        // ---------------------------- subscribe
        std::optional<std::size_t> subscribe(Callback fn, void *ctx = nullptr) noexcept
        {
            auto idx = active_cnt.load(std::memory_order_acquire);
            if (idx >= MaxSubscribers)
            {
                return std::nullopt;
            }
            // construct Subscriber in-place into the optional
            subscribers[idx].emplace(fn, ctx, queue.capacity);
            std::atomic_thread_fence(std::memory_order_release);
            active_cnt.store(idx + 1, std::memory_order_release);
            return idx;
        }

        template <typename U, void (U::*Method)(const T &) noexcept>
        std::optional<std::size_t> subscribe(U *inst) noexcept
        {
            return subscribe(&MemberCallback<U, T, Method>::callback, inst);
        }

        // ---------------------------- unsubscribe
        bool unsubscribe(std::size_t id) noexcept
        {
            auto cnt = active_cnt.load(std::memory_order_acquire);
            if (id >= cnt)
                return false;
            // stop that subscriber
            subscribers[id].reset();
            // compact list
            if (id + 1 < cnt)
                subscribers[id] = std::move(subscribers[cnt - 1]);
            subscribers[cnt - 1].reset();
            active_cnt.store(cnt - 1, std::memory_order_release);
            return true;
        }

        // ---------------------------- stats
        [[nodiscard]] std::size_t central_queue_size() const noexcept { return queue.queue_size(); }
        [[nodiscard]] std::size_t total_subscriber_queue_size() const noexcept
        {
            std::size_t total_size = 0;
            auto cnt = active_cnt.load(std::memory_order_acquire);

            for (std::size_t i = 0; i < cnt; ++i)
            {
                if (subscribers[i].has_value())
                {
                    total_size += subscribers[i]->local_q.queue_size();
                }
            }

            return total_size;
        }

        [[nodiscard]] std::size_t max_subscriber_queue_size() const noexcept
        {
            std::size_t max_size = 0;
            auto cnt = active_cnt.load(std::memory_order_acquire);

            for (std::size_t i = 0; i < cnt; ++i)
            {
                if (subscribers[i].has_value())
                {
                    max_size = std::max(max_size, subscribers[i]->local_q.queue_size());
                }
            }

            return max_size;
        }

        [[nodiscard]] std::size_t dropped_count() const noexcept
        {
            auto total_dropped = queue.get_dropped_count();
            auto cnt = active_cnt.load(std::memory_order_acquire);

            // Add dropped counts from all active subscriber queues
            for (std::size_t i = 0; i < cnt; ++i)
            {
                if (subscribers[i].has_value())
                {
                    total_dropped += subscribers[i]->local_q.get_dropped_count();
                }
            }

            return total_dropped;
        }

        void reset_dropped_count() noexcept
        {
            queue.reset_dropped_count();
            auto cnt = active_cnt.load(std::memory_order_acquire);

            // Reset dropped counts for all active subscriber queues
            for (std::size_t i = 0; i < cnt; ++i)
            {
                if (subscribers[i].has_value())
                {
                    subscribers[i]->local_q.reset_dropped_count();
                }
            }
        }

        // ---------------------------- detailed stats
        [[nodiscard]] QueueStats get_queue_stats() const noexcept
        {
            auto cnt = active_cnt.load(std::memory_order_acquire);

            QueueStats stats{};
            stats.central_queue_size = queue.queue_size();
            stats.active_subscribers = cnt;
            stats.min_subscriber_queue_size = SIZE_MAX;

            for (std::size_t i = 0; i < cnt; ++i)
            {
                if (subscribers[i].has_value())
                {
                    auto sub_size = subscribers[i]->local_q.queue_size();
                    stats.total_subscriber_queue_size += sub_size;
                    stats.max_subscriber_queue_size = std::max(stats.max_subscriber_queue_size, sub_size);
                    stats.min_subscriber_queue_size = std::min(stats.min_subscriber_queue_size, sub_size);
                }
            }

            if (cnt == 0)
            {
                stats.min_subscriber_queue_size = 0;
            }

            return stats;
        }

        // Keep simple interface for backward compatibility
        [[nodiscard]] std::size_t queue_size() const noexcept
        {
            return get_queue_stats().total_subscriber_queue_size;
        }
    };

} // namespace boblib::utils::pubsub