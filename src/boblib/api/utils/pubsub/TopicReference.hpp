// #pragma once

// #include <string>
// #include <memory>
// #include <functional>
// #include <typeindex>
// #include <shared_mutex>
// #include <atomic>
// #include <chrono>

// namespace boblib::utils::pubsub
// {
//     // Forward declarations
//     class TopicManager;
//     class IPubSubBase;

//     // TopicReference class to cache references to frequently accessed topics
//     template <typename T>
//     class TopicReference
//     {
//     private:
//         TopicManager &manager;
//         std::string topic_name;
//         IPubSubBase *pubsub_ptr;                 // Raw pointer for fast access
//         std::atomic<uint64_t> cached_version{0}; // Cached version for change detection
//         std::type_index type_id;
//         mutable std::shared_mutex ref_mutex; // Shared mutex for less contention
//         std::atomic<bool> valid{false};

//         // Cache for subscriber count to avoid frequent locking
//         mutable std::atomic<size_t> cached_subscriber_count{0};
//         mutable std::atomic<bool> subscriber_count_dirty{true};

//         // Cache for queue status to avoid frequent locking
//         mutable std::atomic<bool> cached_queue_full{false};
//         mutable std::atomic<size_t> cached_queue_size{0};
//         mutable std::atomic<bool> queue_status_dirty{true};

//         // Update the cached subscriber count if needed
//         void update_subscriber_count() const
//         {
//             if (subscriber_count_dirty.load(std::memory_order_relaxed))
//             {
//                 cached_subscriber_count.store(manager.subscriber_count(topic_name),
//                                             std::memory_order_release);
//                 subscriber_count_dirty.store(false, std::memory_order_release);
//             }
//         }

//         // Update the cached queue status if needed
//         void update_queue_status() const
//         {
//             if (queue_status_dirty.load(std::memory_order_relaxed))
//             {
//                 cached_queue_full.store(manager.is_queue_full(topic_name),
//                                       std::memory_order_release);
//                 cached_queue_size.store(manager.queue_size(topic_name),
//                                       std::memory_order_release);
//                 queue_status_dirty.store(false, std::memory_order_release);
//             }
//         }

//     public:
//         TopicReference(TopicManager &mgr, const std::string &name)
//             : manager(mgr), topic_name(name), type_id(typeid(T))
//         {
//             refresh();
//         }

//         void refresh()
//         {
//             std::cout << "Refreshing topic: " << topic_name << std::endl;
            
//             // First, check if the topic exists without holding the ref_mutex
//             bool topic_exists = false;
//             IPubSubBase* base_ptr = nullptr;
            
//             // Get the necessary information from the manager without holding the ref_mutex
//             topic_exists = manager.topic_exists(topic_name);
//             if (topic_exists) {
//                 base_ptr = manager.get_pubsub_ptr(topic_name);
//             }
            
//             // Now update our internal state with the ref_mutex held
//             std::unique_lock<std::shared_mutex> lock(ref_mutex);
//             std::cout << "Refreshing topic: lock" << std::endl;
            
//             if (topic_exists)
//             {
//                 std::cout << "Refreshing topic: topic exists" << std::endl;
//                 if (base_ptr)
//                 {
//                     std::cout << "Refreshing topic: base ptr" << std::endl;
//                     // Instead of using dynamic_cast to PubSubWrapper, use the type_id to check compatibility
//                     if (base_ptr->get_type_id() == type_id)
//                     {
//                         pubsub_ptr = base_ptr;
//                         valid.store(true, std::memory_order_release);
//                         cached_version.store(base_ptr->get_version(), std::memory_order_release);
//                     }
//                     else
//                     {
//                         pubsub_ptr = nullptr;
//                         valid.store(false, std::memory_order_release);
//                     }
//                 }
//                 else
//                 {
//                     std::cout << "Refreshing topic: base ptr is nullptr" << std::endl;
//                     pubsub_ptr = nullptr;
//                     valid.store(false, std::memory_order_release);
//                 }
//             }
//             else
//             {
//                 std::cout << "Refreshing topic: topic does not exist" << std::endl;
//                 pubsub_ptr = nullptr;
//                 valid.store(false, std::memory_order_release);
//             }
//         }

//         bool is_valid() const
//         {
//             return valid.load(std::memory_order_acquire);
//         }

//         bool publish(const T &message)
//         {
//             std::shared_lock<std::shared_mutex> lock(ref_mutex);
//             if (!valid.load(std::memory_order_acquire))
//             {
//                 return false;
//             }

//             auto *current_ptr = pubsub_ptr;
//             if (current_ptr)
//             {
//                 return current_ptr->publish(&message);
//             }

//             // If we don't have a valid pointer, try to refresh
//             lock.unlock();
//             refresh();
//             lock.lock();

//             current_ptr = pubsub_ptr;
//             if (current_ptr)
//             {
//                 return current_ptr->publish(&message);
//             }

//             return false;
//         }

//         size_t subscribe(std::function<void(const T &)> callback)
//         {
//             std::shared_lock<std::shared_mutex> lock(ref_mutex);
//             if (!valid.load(std::memory_order_acquire))
//             {
//                 return 0;
//             }

//             // Use the base class interface instead of casting to PubSubWrapper
//             return pubsub_ptr->subscribe([callback](const void *msg)
//                                          { callback(*static_cast<const T *>(msg)); });
//         }

//         bool unsubscribe(size_t id)
//         {
//             std::shared_lock<std::shared_mutex> lock(ref_mutex);
//             if (!valid.load(std::memory_order_acquire))
//             {
//                 return false;
//             }

//             auto *current_ptr = pubsub_ptr;
//             if (current_ptr)
//             {
//                 return current_ptr->unsubscribe(id);
//             }

//             return false;
//         }

//         size_t subscriber_count() const
//         {
//             std::shared_lock<std::shared_mutex> lock(ref_mutex);
//             if (!valid.load(std::memory_order_acquire))
//             {
//                 return 0;
//             }

//             auto *current_ptr = pubsub_ptr;
//             if (current_ptr)
//             {
//                 return current_ptr->subscriber_count();
//             }

//             return 0;
//         }

//         bool is_queue_full() const
//         {
//             std::shared_lock<std::shared_mutex> lock(ref_mutex);
//             if (!valid.load(std::memory_order_acquire))
//             {
//                 return true;
//             }

//             auto *current_ptr = pubsub_ptr;
//             if (current_ptr)
//             {
//                 return current_ptr->is_queue_full();
//             }

//             return true;
//         }

//         size_t queue_size() const
//         {
//             std::shared_lock<std::shared_mutex> lock(ref_mutex);
//             if (!valid.load(std::memory_order_acquire))
//             {
//                 return 0;
//             }

//             auto *current_ptr = pubsub_ptr;
//             if (current_ptr)
//             {
//                 return current_ptr->queue_size();
//             }

//             return 0;
//         }

//         size_t queue_capacity() const
//         {
//             std::shared_lock<std::shared_mutex> lock(ref_mutex);
//             if (!valid.load(std::memory_order_acquire))
//             {
//                 return 0;
//             }

//             auto *current_ptr = pubsub_ptr;
//             if (current_ptr)
//             {
//                 return current_ptr->queue_capacity();
//             }

//             return 0;
//         }

//         void set_timeout(std::chrono::milliseconds timeout)
//         {
//             std::shared_lock<std::shared_mutex> lock(ref_mutex);
//             if (!valid.load(std::memory_order_acquire))
//             {
//                 return;
//             }

//             auto *current_ptr = pubsub_ptr;
//             if (current_ptr)
//             {
//                 current_ptr->set_timeout(timeout);
//             }
//         }
//     };
// } 
