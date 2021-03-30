#pragma once

#include <queue>
#include <memory>
#include <atomic>
#include <condition_variable>

#include <glog/logging.h>

#include "uom//utils/macros.hpp"
#include "uom/utils/common_types.hpp"


/**
 * @brief Basic template class for ThreadsafeQueue.
 * @tparam T Stored data type
 */
template <typename T>
class ThreadsafeQueueBase
{
public:

    POINTER_TYPEDEFS(ThreadsafeQueueBase)

    DELETE_COPY_CONSTRUCTORS(ThreadsafeQueueBase)

    using InternalType = std::shared_ptr<T>;
    using InternalQueue = std::queue<InternalType>;
    using ValueType = T;

    /// ctor
    explicit ThreadsafeQueueBase(const std::string& queue_id) :
        queue_id_(std::move(queue_id)), shutdown_(false)
    {
    }

    /// dtor
    virtual ~ThreadsafeQueueBase() = default;


    inline std::string get_id() const
    {
        return queue_id_;
    }


    /**
     * @brief Get a reference of the front element of the queue. Not thread-safe
     * @return Returned the refenrece of front element.
     */
    virtual T& front() = 0;


    ///**
    // * @brief Get a copy of the front element of the queue. Waits for data to be available in the queue.
    // * @param value Returned front value.
    // * @return False if empty.
    // */
    //virtual bool front_blocking(T& value) = 0;
    //
    //
    ///**
    // * @brief Get a copy of the front element of the queue. Waits for data to be available in the queue. But
    // * further returns early if the given duration has passed.
    // * @param value Returned front value.
    // * @param duration_ms Time to wait for a msg [ns].
    // * @return Returns false if the queue has been shutdown or if it was timeout.
    // */
    //virtual bool front_blocking_with_timeout(T& value, size_t duration) = 0;


    /**
     * @brief Push by value.
     * @param new_value New value to add to the queue.
     * @return False if the queue has been shutdown.
     */
    virtual bool push(T new_value) = 0;


    /**
     * @brief push_blocking_if_full pushes a value into the queue only if the queue is not filled with more than
     * a given maximum size.
     * @param new_value new value to add to the queue.
     * @param max_queue_size if the queue is filled with more than max_queue_size messages, it will not push to the
     * queue, and it will wait for a consumer to remove messages.
     * @return False if the queue has been shutdown.
     */
    virtual bool push_blocking_if_full(T new_value, size_t max_queue_size = 10u) = 0;


    /**
     * @brief Pop without blocking, just checks once if the queue is empty.
     * @param value Returned value.
     * @return True if the value could be retrieved, false otherwise.
     */
    virtual bool pop(T& value) = 0;


    /**
     * @brief Pop without blocking, just checks once if the queue is empty.
     * @return A shared_ptr to the value retrieved.
     */
    virtual std::shared_ptr<T> pop() = 0;


    /**
     * @brief pop value. Waits for data to be available in the queue.
     * @param value Returned value.
     * @return False if the queue has been shutdown.
     */
    virtual bool pop_blocking(T& value) = 0;


    /**
     * @brief Pop value. Waits for data to be available in the queue.
     * @return Null if the queue has been shutdown, otherwise a shared_ptr to the value retrieved.
     */
    virtual std::shared_ptr<T> pop_blocking() = 0;


    /**
     * @brief Swap queue with empty queue if not empty.
     * @return True if values were retrieved.
     */
    virtual bool batch_pop(InternalQueue* output_queue) = 0;


    /**
     * @brief Pop value. Waits for data to be available in the queue. But further returns early if the given
     * duration has passed.
     * @param value Returned value.
     * @param duration Time to wait for a msg [ns].
     * @return Returns false if the queue has been shutdown or if it was timeout.
     */
    virtual bool pop_blocking_with_timeout(T& value, size_t duration) = 0;


    /**
     * @brief Shutdown the queue
     */
    inline void shutdown()
    {
        VLOG(3) << "Shutting down queue: " << queue_id_;

        std::unique_lock<std::mutex> lck(data_mtx_);

        shutdown_ = true;

        lck.unlock();
        data_cv_.notify_all();
    }


    /**
     * @brief Checks if the queue is shutdown. the state of the queue might change right after this query.
     */
    inline bool is_shutdown() const
    {
        return shutdown_;
    }


    /**
     * @brief Resume the queue
     */
    inline void resume()
    {
        VLOG(3) << "Resuming queue: " << queue_id_;

        std::unique_lock<std::mutex> lck(data_mtx_);

        shutdown_ = false;

        lck.unlock();
        data_cv_.notify_all();
    }


    /**
     * @brief Checks if the queue is empty. the state of the queue might change right after this query.
     * @return True if the queue is empty.
     */
    inline bool empty() const
    {
        std::lock_guard<std::mutex> lk(data_mtx_);
        return data_queue_.empty();
    }


protected:

    /// Indicates the ID of ThreadSafe queue
    std::string queue_id_;

    /// Where the data is stored
    InternalQueue data_queue_;

    /// Flag for signaling queue shutdown.
    std::atomic_bool shutdown_;

    mutable std::mutex data_mtx_;

    std::condition_variable data_cv_;

};


/**
 * @brief ThreadSafe queue implementation
 * @tparam T Stored data type
 */
template <typename T>
class ThreadsafeQueue : public ThreadsafeQueueBase<T>
{
public:

    using TQB = ThreadsafeQueueBase<T>;

    POINTER_TYPEDEFS(ThreadsafeQueue)

    DELETE_COPY_CONSTRUCTORS(ThreadsafeQueue)


    /// Ctor
    using TQB::ThreadsafeQueueBase;

    /// Dtor
    ~ThreadsafeQueue() override = default;


    /// dirty implement
    T& front() override
    {
        if (shutdown_)
        {
            throw std::runtime_error(queue_id_ + " is down. You should lock queue before call front() method.");
        }

        // Try to lock
        std::unique_lock<std::mutex> lk(data_mtx_);

        if (data_queue_.empty())
        {
            throw std::runtime_error(queue_id_ + " is empty. "
                                                 "You should lock queue before call front() method.");
        }

        return *data_queue_.front();
    }


    //bool front(T& value) override
    //{
    //    // Return if the queue is shutdown
    //    if (shutdown_) return false;
    //
    //    // Try to lock
    //    std::unique_lock<std::mutex> lk(data_mtx_);
    //
    //    // Return if the queue is empty
    //    if (data_queue_.empty()) return false;
    //
    //    // Get value before unlock.
    //    value = std::move(std::ref(*data_queue_.front()));
    //
    //    // Unlock before notify.
    //    lk.unlock();
    //    data_cv_.notify_one();
    //
    //    return true;
    //}
    //
    //
    //bool front_blocking(T& value) override
    //{
    //    // Return if the queue is shutdown
    //    if (shutdown_) return false;
    //
    //    // Try to lock
    //    std::unique_lock<std::mutex> lk(data_mtx_);
    //
    //    // Wait until the queue has space or shutdown requested.
    //    data_cv_.wait(lk, [this] { return !data_queue_.empty() || shutdown_; });
    //
    //    // Return if the queue is shutdown
    //    if (shutdown_) return false;
    //
    //    // Get value before unlock.
    //    value = *data_queue_.front();
    //
    //    // Unlock before notify.
    //    lk.unlock();
    //    data_cv_.notify_one();
    //
    //    return true;
    //}
    //
    //
    //virtual bool front_blocking_with_timeout(T& value, size_t duration) override
    //{
    //    // Return if the queue is shutdown
    //    if (shutdown_) return false;
    //
    //    // Try to lock
    //    std::unique_lock<std::mutex> lk(data_mtx_);
    //
    //    // Wait until the queue has space or shutdown requested.
    //    data_cv_.wait_for(lk, std::chrono::nanoseconds(duration),
    //                      [this] { return !data_queue_.empty() || shutdown_; });
    //
    //    // Return if the queue is shutdown or empty
    //    if (shutdown_ || data_queue_.empty()) return false;
    //
    //    // Get value before unlock.
    //    value = *data_queue_.front();
    //
    //    return true;
    //}


    bool push(T new_value) override
    {
        // Return if the queue is shutdown
        if (shutdown_)
        {
            return false;
        }

        // Move data before lock
        auto data = std::make_shared<T>(std::move(new_value));

        // Try to lock
        std::unique_lock<std::mutex> lk(data_mtx_);

        // Store the new value
        data_queue_.emplace(data);

        // Get size of queue before unlock.
        const auto queue_size = data_queue_.size();

        // Unlock before notify.
        lk.unlock();
        data_cv_.notify_one();

        VLOG(4) << "Queue with id: " << queue_id_ << " is getting full, size: " << queue_size;

        return true;
    }


    bool push_blocking_if_full(T new_value, size_t max_queue_size = 10u) override
    {
        // Return if the queue is shutdown
        if (shutdown_)
        {
            return false;
        }

        // Move data before lock
        auto data = std::make_shared<T>(std::move(new_value));

        // Try to lock
        std::unique_lock<std::mutex> lk(data_mtx_);

        // Wait until the queue has space or shutdown requested.
        data_cv_.wait(lk, [this, max_queue_size]
        {
            return data_queue_.size() < max_queue_size || shutdown_;
        });

        // Return if the queue is shutdown
        if (shutdown_)
        {
            return false;
        }

        // Store the new value
        data_queue_.emplace(data);

        // Get size of queue before unlock.
        const auto queue_size = data_queue_.size();

        // Unlock before notify.
        lk.unlock();
        data_cv_.notify_one();

        VLOG(4) << "Queue with id: " << queue_id_ << " is getting full, size: " << queue_size;

        return true;
    }


    bool pop(T& value) override
    {
        // Return if the queue is shutdown
        if (shutdown_)
        {
            return false;
        }

        // Try to lock
        std::unique_lock<std::mutex> lk(data_mtx_);

        // Return if the queue is empty
        if (data_queue_.empty())
        {
            return false;
        }

        // Get value before unlock.
        value = std::move(*data_queue_.front());
        data_queue_.pop();

        // Unlock before notify.
        lk.unlock();
        data_cv_.notify_one();

        return true;
    }


    std::shared_ptr<T> pop() override
    {
        // Return if the queue is shutdown
        if (shutdown_)
        {
            return nullptr;
        }

        // Try to lock
        std::unique_lock<std::mutex> lk(data_mtx_);

        // Wait until the queue has space or shutdown requested.
        data_cv_.wait(lk, [this]
        {
            return !data_queue_.empty() || shutdown_;
        });

        if (shutdown_)
        {
            return nullptr;
        }

        // Return if the queue is empty
        if (data_queue_.empty())
        {
            return nullptr;
        }

        // Get value before unlock.
        std::shared_ptr<T> result = data_queue_.front();
        data_queue_.pop();

        // Unlock before notify.
        lk.unlock();
        data_cv_.notify_one();

        return result;
    }


    bool pop_blocking(T& value) override
    {
        // Return if the queue is shutdown
        if (shutdown_)
        {
            return false;
        }

        // Try to lock
        std::unique_lock<std::mutex> lk(data_mtx_);

        // Wait until the queue has space or shutdown requested.
        data_cv_.wait(lk, [this]
        {
            return !data_queue_.empty() || shutdown_;
        });

        // Return if the queue is shutdown
        if (shutdown_)
        {
            return false;
        }

        // Get value before unlock.
        value = std::move(*data_queue_.front());
        data_queue_.pop();

        // Unlock before notify.
        lk.unlock();
        data_cv_.notify_one();

        return true;
    }


    std::shared_ptr<T> pop_blocking() override
    {
        // Return if the queue is shutdown
        if (shutdown_)
        {
            return nullptr;
        }

        // Try to lock
        std::unique_lock<std::mutex> lk(data_mtx_);

        // Wait until the queue has space or shutdown requested.
        data_cv_.wait(lk, [this]
        {
            return !data_queue_.empty() || shutdown_;
        });

        // Return if the queue is shutdown
        if (shutdown_)
        {
            return nullptr;
        }

        // Get value before unlock.
        std::shared_ptr<T> result = data_queue_.front();
        data_queue_.pop();

        // Unlock before notify.
        lk.unlock();
        data_cv_.notify_one();

        return result;
    }


    bool batch_pop(typename TQB::InternalQueue* output_queue) override
    {
        // Check if output_queue is valid
        CHECK_NOTNULL(output_queue);
        CHECK(output_queue->empty());

        // Try to lock
        std::unique_lock<std::mutex> lk(data_mtx_);

        bool success = false;
        if (!data_queue_.empty())
        {
            data_queue_.swap(*output_queue);
            success = true;
        }

        // Unlock before notify.
        lk.unlock();
        data_cv_.notify_one();

        return success;
    }


    bool pop_blocking_with_timeout(T& value, size_t duration) override
    {
        // Return if the queue is shutdown
        if (shutdown_)
        {
            return false;
        }

        // Try to lock
        std::unique_lock<std::mutex> lk(data_mtx_);

        // Wait until the queue has space or shutdown requested.
        data_cv_.wait_for(lk, std::chrono::nanoseconds(duration),
                          [this]
                          {
                              return !data_queue_.empty() || shutdown_;
                          });

        // Return if the queue is shutdown or empty
        if (shutdown_ || data_queue_.empty())
        {
            return false;
        }

        // Get value before unlock.
        value = std::move(*data_queue_.front());
        data_queue_.pop();

        return true;
    }

protected:

    using TQB::queue_id_;
    using TQB::shutdown_;
    using TQB::data_mtx_;
    using TQB::data_cv_;
    using TQB::data_queue_;

};


/**
 * @brief The ThreadsafeNullQueue class acts as a placeholder queue, but does
 * nothing. Useful for pipeline modules that do not require a queue.
 */
template <typename T>
class ThreadsafeNullQueue : public ThreadsafeQueue<T>
{
public:
    POINTER_TYPEDEFS(ThreadsafeNullQueue);
    DELETE_COPY_CONSTRUCTORS(ThreadsafeNullQueue);

    explicit ThreadsafeNullQueue(const std::string& queue_id) : ThreadsafeQueue<T>(queue_id)
    {
    }

    ~ThreadsafeNullQueue() override = default;

    //! Do nothing
    virtual bool push(T) override
    {
        return true;
    }

    virtual bool push_blocking_if_full(T, size_t)
    {
        return true;
    };

    virtual bool pop_blocking(T&) override
    {
        return true;
    }

    virtual std::shared_ptr<T> pop_blocking() override
    {
        return nullptr;
    }

    virtual bool pop(T&) override
    {
        return true;
    }

    virtual std::shared_ptr<T> pop() override
    {
        return nullptr;
    }

    virtual bool front(T&) override
    {
        return true;
    }

    virtual bool front_blocking(T&) override
    {
        return true;
    }
};


/**
 * @brief A very simple naive sync approach: Just loop over the messages in the queue until you find the matching
 * timestamp. If we are at a timestamp greater than the queried one.
 * @tparam T Templated type on a POINTER to a class that is derived from PipelinePayload (Such as Packet::Ptr).
 * @param[in]  timestamp Timestamp of the payload we want to retrieve from the queue.
 * @param[in]  queue Threadsafe queue templated on a POINTER to a class that is derived from PipelinePayload.
 * @param[out] pipeline_payload Returns payload to be found in the given queue at the given timestamp.
 * @param[in]  max_iterations Number of times we try to find the payload at the given timestamp in the given queue.
 * @param[in]  callback User defined function to be called on each successful retrieval of a payload in the queue,
 * the callback should be lighting fast!
 * @return A boolean indicating whether the synchronizing was successful (i.e. we found a payload with the
 * requested timestamp) or we failed because a payload with an older timestamp was retrieved.
 */

/*
template <typename T>
bool synchronize_queue(const Timestamp& timestamp, ThreadsafeQueue<T>* queue, T* pipeline_payload,
                       size_t max_iterations = 10, std::function<void(const T&)>* callback = nullptr)
{

    CHECK_NOTNULL(queue);
    CHECK_NOTNULL(pipeline_payload);

    static_assert(std::is_base_of<PipelinePayload, typename std::pointer_traits<T>::element_type>::value,
                  "T must be a pointer to a class that derives from PipelinePayload.");

    Timestamp packet_timestamp = std::numeric_limits<Timestamp>::min();

    size_t i = 0;
    static constexpr size_t timeout_ns = 1e7;  // Wait 10ms at most!
    for (; i < max_iterations && timestamp > packet_timestamp; ++i)
    {
        if (!queue->pop_blocking_with_timeout(*pipeline_payload, timeout_ns))
        {
            LOG(ERROR) << "Sync failed for queue id: " << queue->get_id() << "\n Reason: \n"
                       << "Queue status: " << (queue->is_shutdown() ? "Shutdown..." : "Timeout...");
            return false;
        }
        else
        {
            VLOG(4) << "Popping from: " << queue->get_id();
        }


        if (*pipeline_payload)
        {
            packet_timestamp = (*pipeline_payload)->timestamp;
            // Call any user defined callback at this point (should be fast!!).
            if (callback)
            {
                (*callback)(*pipeline_payload);
            }
        }
        else
        {
            LOG(WARNING) << "Payload synchronization failed. Missing payload for Queue: " << queue->get_id();
        }
    }
    CHECK_EQ(timestamp, packet_timestamp)
        << "Syncing queue " << queue->get_id() << " failed;\n"
        << "Could not retrieve exact timestamp requested: \n"
        << " - Requested timestamp: " << timestamp << '\n'
        << " - Actual timestamp:    " << packet_timestamp << '\n'
        << (i >= max_iterations ? "Reached max number of sync attempts: " + std::to_string(max_iterations) : "");

    CHECK(*pipeline_payload);
    return true;

}
*/