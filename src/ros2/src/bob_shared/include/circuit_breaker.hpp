#pragma once
#ifndef __CIRCUIT_BREAKER_H__
#define __CIRCUIT_BREAKER_H__

#include <chrono>

class CircuitBreaker 
{
public:
    CircuitBreaker(int failure_threshold, int initial_reset_timeout, int max_reset_timeout)
        : state_(CircuitBreakerState::Closed),
          failure_count_(0),
          failure_threshold_(failure_threshold),
          initial_reset_timeout_(initial_reset_timeout),
          max_reset_timeout_(max_reset_timeout),
          current_reset_timeout_(initial_reset_timeout) 
    {}

    bool allow_request() 
    {
        if (state_ == CircuitBreakerState::Open) 
        {
            auto now = std::chrono::steady_clock::now();
            if ((now - last_failure_time_) >= std::chrono::milliseconds(current_reset_timeout_)) 
            {
                state_ = CircuitBreakerState::HalfOpen;
                return true;
            }
            return false;
        }
        return true;
    }

    void record_success() 
    {
        state_ = CircuitBreakerState::Closed;
        failure_count_ = 0;
        current_reset_timeout_ = initial_reset_timeout_; // Reset the timeout on success
    }

    void record_failure() 
    {
        ++failure_count_;
        if (failure_count_ >= failure_threshold_) 
        {
            state_ = CircuitBreakerState::Open;
            last_failure_time_ = std::chrono::steady_clock::now();
            increase_reset_timeout();
        }
    }

private:
    void increase_reset_timeout() 
    {
        if (current_reset_timeout_ < max_reset_timeout_) 
        {
            current_reset_timeout_ = std::min(current_reset_timeout_ * increase_multiply_, max_reset_timeout_);
        }
    }

    enum class CircuitBreakerState 
    {
        Closed,
        Open,
        HalfOpen
    };

    const int increase_multiply_ = 2;
    CircuitBreakerState state_;
    int failure_count_;
    int failure_threshold_;
    int initial_reset_timeout_;
    int max_reset_timeout_;
    int current_reset_timeout_;
    std::chrono::steady_clock::time_point last_failure_time_;
};

#endif