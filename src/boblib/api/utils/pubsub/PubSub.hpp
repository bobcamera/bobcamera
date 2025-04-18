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
// #pragma once

// #include <atomic>
// #include <functional>
// #include <memory>
// #include <thread>
// #include <optional>
// #include <type_traits>
// #include <concepts>
// #include <mutex>
// #include <shared_mutex>
// #include <condition_variable>
// #include <chrono>
// #include <array>
// #include <cstddef>
// #include <algorithm>
// #include <iostream>

// namespace boblib::utils::pubsub
// {
//     // *** Helper utilities ********************************************************

//     // C++23 already provides std::movable, but keep our own alias for pre‑23
//     template <typename T>
//     concept CopyableOrMovable = std::movable<T>;

//     /// Round up to next power‑of‑two (32 / 64‑bit agnostic).
//     [[nodiscard]]
//     constexpr std::size_t next_power_of_two(std::size_t n) noexcept
//     {
//         if (n < 2)
//             return 2;
//         --n;
//         for (std::size_t shift = 1; shift < sizeof(std::size_t) * 8; shift <<= 1)
//             n |= n >> shift;
//         return ++n;
//     }

//     // *** PubSub *******************************************************************

//     template <typename T, std::size_t MaxSubscribers = 4>
//         requires CopyableOrMovable<T>
//     class PubSub
//     {
//         static_assert(std::is_nothrow_destructible_v<T>,
//                       "T must have a noexcept destructor");

//     private:
//         // ------------------------------------------------------------ Slot
//         static constexpr std::size_t CacheLineSize = 64;

//         struct alignas(CacheLineSize) Slot
//         {
//             // Flags and storage -------------------------------------------------
//             std::atomic<bool> occupied{false};

//             std::aligned_storage_t<sizeof(T), alignof(T)> storage;

//             // Padding so that each slot occupies an integral number of cache lines
//             static constexpr std::size_t RawSize =
//                 sizeof(std::atomic<bool>) + sizeof(storage);

//             static constexpr std::size_t PadSize =
//                 (CacheLineSize - (RawSize % CacheLineSize)) % CacheLineSize;

//             std::array<std::byte, PadSize == 0 ? 1 : PadSize> padding{};

//             // Construction / access --------------------------------------------
//             template <typename... Args>
//             void construct(Args &&...args) noexcept(noexcept(T(std::forward<Args>(args)...)))
//             {
//                 new (&storage) T(std::forward<Args>(args)...);
//             }

//             T &get() noexcept { return *reinterpret_cast<T *>(&storage); }

//             const T &get() const noexcept { return *reinterpret_cast<const T *>(&storage); }

//             ~Slot() noexcept
//             {
//                 if (occupied.load(std::memory_order_acquire))
//                 {
//                     get().~T();
//                 }
//             }

//             Slot() = default;
//             Slot(const Slot &) = delete;
//             Slot &operator=(const Slot &) = delete;
//             Slot(Slot &&) = delete;
//             Slot &operator=(Slot &&) = delete;
//         };

//         // ------------------------------------------------------------ CircularBuffer (SPSC)
//         struct alignas(CacheLineSize) CircularBuffer
//         {
//             explicit CircularBuffer(std::size_t requested)
//                 : capacity(next_power_of_two(requested))
//                 , buffer(std::make_unique<Slot[]>(capacity))
//                 , mask(capacity - 1)
//             {
//             }

//             // publish ----------------------------------------------------------
//             bool push(T &&item) noexcept(noexcept(std::declval<Slot>().construct(std::move(item))))
//             {
//                 const std::size_t current_head = head.load(std::memory_order_relaxed);
//                 const std::size_t next_head = (current_head + 1) & mask;

//                 if (next_head == tail.load(std::memory_order_acquire))
//                     return false; // queue full

//                 buffer[current_head].construct(std::move(item));
//                 buffer[current_head].occupied.store(true, std::memory_order_release);
//                 head.store(next_head, std::memory_order_release);
//                 return true;
//             }

//             // consume ----------------------------------------------------------
//             template <typename F>
//             bool process_next(F &&processor) noexcept(noexcept(processor(std::declval<const T &>())))
//             {
//                 const std::size_t current_tail = tail.load(std::memory_order_relaxed);
//                 if (current_tail == head.load(std::memory_order_acquire))
//                     return false; // empty

//                 Slot &slot = buffer[current_tail];
//                 if (!slot.occupied.load(std::memory_order_acquire))
//                     return false; // shouldn't happen

//                 processor(slot.get());
//                 slot.get().~T();
//                 slot.occupied.store(false, std::memory_order_release);
//                 tail.store((current_tail + 1) & mask, std::memory_order_release);
//                 return true;
//             }

//             // data -------------------------------------------------------------
//             const std::size_t capacity;
//             std::unique_ptr<Slot[]> buffer;
//             std::atomic<std::size_t> head{0};
//             std::atomic<std::size_t> tail{0};
//             const std::size_t mask;
//             // query ------------------------------------------------------------
//             [[nodiscard]]
//             bool empty() const noexcept
//             {
//                 return head.load(std::memory_order_acquire) == tail.load(std::memory_order_acquire);
//             }
//         };

//         // ------------------------------------------------------------ types
//         using SubscriberCallback = void (*)(const T &, void *);
//         struct Handler
//         {
//             SubscriberCallback fn{nullptr};
//             void *ctx{nullptr};
//         };

//         // ------------------------------------------------------------ data members
//         CircularBuffer queue;

//         std::array<Handler, MaxSubscribers> subscribers{};
//         std::atomic<std::size_t> active_subscriber_count{0};

//         std::atomic<bool> running{true};
//         std::thread dispatch_thread;
//         // no synchronization primitives needed for busy‑spin dispatch

//         // ------------------------------------------------------------ dispatch loop (busy spin)
//         void dispatch_loop() noexcept
//         {
//             while (running.load())
//             {
//                 if (!queue.process_next(
//                         [this](const T &msg) noexcept
//                         {
//                             Handler local[MaxSubscribers];
//                             std::size_t cnt = active_subscriber_count.load();
//                             std::copy_n(subscribers.begin(), cnt, local);
//                             for (std::size_t i = 0; i < cnt; ++i)
//                                 if (local[i].fn)
//                                     local[i].fn(msg, local[i].ctx);
//                         }))
//                 {
//                     std::this_thread::yield();
//                 }
//             }
//         }

//     public:
//         // ------------------------------------------------------------ ctor / dtor
//         explicit PubSub(std::size_t queue_size = 128) noexcept
//             : queue(queue_size)
//         {
//             dispatch_thread = std::thread(&PubSub::dispatch_loop, this);
//         }

//         ~PubSub() noexcept
//         {
//             running.store(false, std::memory_order_release);
//             if (dispatch_thread.joinable())
//                 dispatch_thread.join();

//             // drain remaining messages
//             while (queue.process_next([](const T &) noexcept {}))
//             {
//             }
//         }

//         // ------------------------------------------------------------ publish
//         [[gnu::always_inline]]
//         bool publish(T &&message) noexcept(noexcept(queue.push(std::move(message))))
//         {
//             if (!queue.push(std::move(message)))
//             {
//                 std::cerr << "[PubSub] publish: queue full\n";
//                 return false;
//             }
//             return true;
//         }

//         // ------------------------------------------------------------ subscribe
//         // Returns slot index on success, std::nullopt on failure.
//         std::optional<std::size_t> subscribe(SubscriberCallback fn, void *ctx = nullptr) noexcept
//         {
//             const std::size_t idx = active_subscriber_count.load();
//             if (idx >= MaxSubscribers)
//                 return std::nullopt;

//             subscribers[idx] = Handler{fn, ctx};
//             active_subscriber_count.store(idx + 1);
//             return idx;
//         }

//         // ------------------------------------------------------------ unsubscribe
//         bool unsubscribe(std::size_t id) noexcept
//         {
//             const std::size_t cnt = active_subscriber_count.load();
//             if (id >= cnt)
//                 return false;

//             subscribers[id] = subscribers[cnt - 1]; // move last into removed slot
//             subscribers[cnt - 1] = Handler{};
//             active_subscriber_count.store(cnt - 1);
//             return true;
//         }

//         // ------------------------------------------------------------ stats
//         [[nodiscard]]
//         bool is_queue_full() const noexcept
//         {
//             const auto head_v = queue.head.load(std::memory_order_relaxed);
//             const auto next = (head_v + 1) & queue.mask;
//             return next == queue.tail.load(std::memory_order_acquire);
//         }

//         [[nodiscard]]
//         std::size_t queue_size() const noexcept
//         {
//             std::size_t tail_snapshot, head_snapshot;
//             do
//             {
//                 tail_snapshot = queue.tail.load(std::memory_order_acquire);
//                 head_snapshot = queue.head.load(std::memory_order_acquire);
//             } while (tail_snapshot != queue.tail.load(std::memory_order_acquire));

//             return (head_snapshot - tail_snapshot) & queue.mask;
//         }

//         [[nodiscard]] std::size_t queue_capacity() const noexcept { return queue.capacity; }
//         [[nodiscard]] bool empty() const noexcept { return queue_size() == 0; }
//         [[nodiscard]] std::size_t subscriber_count() const noexcept
//         {
//             return active_subscriber_count.load();
//         }
//     };

// } // namespace boblib::utils::pubsub