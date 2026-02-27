#pragma once

#include <atomic>
#include <queue>
#include <mutex>
#include <condition_variable>

template <typename T>
class SynchronizedQueue
{
public:
    SynchronizedQueue(size_t max_size = 0)
        : max_size_(max_size)
    {
    }

    // Add an item to the queue (copy version)
    bool push(const T & item)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (max_size_ > 0 && queue_.size() >= max_size_)
        {
            return false;
        }

        queue_.push(item);
        cv_.notify_one();
        return true;
    }

    // Add an item to the queue (move version)
    bool push(T && item)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (max_size_ > 0 && queue_.size() >= max_size_)
        {
            return false;
        }

        queue_.push(std::move(item));
        cv_.notify_one();
        return true;
    }

    // Retrieve and remove an item from the queue (copy version)
    bool pop(T & item)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [this] { return !queue_.empty() || !running_; });

        if (!running_ && queue_.empty())
            return false;

        item = queue_.front();  // This performs a copy
        queue_.pop();
        return true;
    }

    // Retrieve and remove an item from the queue (move version)
    bool pop_move(T & item)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [this] { return !queue_.empty() || !running_; });

        if (!running_ && queue_.empty())
            return false;

        item = std::move(queue_.front());  // This moves the object
        queue_.pop();
        return true;
    }

    // Alternative: pop method that returns the item directly (move semantics)
    std::optional<T> pop()
    {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [this] { return !queue_.empty() || !running_; });

        if (!running_ && queue_.empty())
            return std::nullopt;

        T item = std::move(queue_.front());
        queue_.pop();
        return item;
    }

    // Stop the queue processing
    void stop()
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            running_ = false;
        }
        cv_.notify_all();
    }

    // Start or resume the queue processing
    void start()
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            running_ = true;
        }
        cv_.notify_all();
    }

    // Check if the queue is running (lock-free)
    bool is_running() const
    {
        return running_.load();
    }

    // Check if the queue is empty
    bool empty() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.empty();
    }

    size_t size() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.size();
    }

    // Discard all queued items
    void clear()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        std::queue<T> empty;
        queue_.swap(empty);
    }

private:
    const size_t max_size_;
    std::queue<T> queue_;
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::atomic<bool> running_{true};
};

