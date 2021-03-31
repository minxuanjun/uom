#pragma once

#include <map>
#include <mutex>
#include <string>
#include <condition_variable>

#include <glog/logging.h>

#include "uom/utils/timer.hpp"
#include "uom/utils/macros.hpp"
#include "uom/utils/common_types.hpp"
#include "uom/utils/math_utils.hpp"


enum class TemporalBufferQueryResult : int
{
    AVAILABLE = 0,        // Query was successful and the data is available.
    NOT_YAT_AVALIABLE,    // The required data is not (yet) available. The query timestamp is above the last sample's time.
    NERVER_AVAILABLE,     // The queried timestamp lies before the first IMU sample in the buffer.
    SHUTDOWN,             // Queue shutdown.
    TOO_FEW_MEASUREMENT   //
};


/// For print
inline std::ostream& operator <<(std::ostream& os, const TemporalBufferQueryResult& model)
{
    switch (model)
    {
        case TemporalBufferQueryResult::AVAILABLE:             os << "AVAILABLE";           break;
        case TemporalBufferQueryResult::NOT_YAT_AVALIABLE:     os << "NOT_YAT_AVALIABLE";   break;
        case TemporalBufferQueryResult::NERVER_AVAILABLE:      os << "NERVER_AVAILABLE";    break;
        case TemporalBufferQueryResult::SHUTDOWN:              os << "SHUTDOWN";            break;
        case TemporalBufferQueryResult::TOO_FEW_MEASUREMENT:   os << "TOO_FEW_MEASUREMENT"; break;
    }
    return os;
}


template <typename ValueType, typename AllocatorType = std::allocator<std::pair<const Timestamp, ValueType>>>
class TemporalBuffer
{
public:

    POINTER_TYPEDEFS(TemporalBuffer)

    DELETE_COPY_CONSTRUCTORS(TemporalBuffer)

    using BufferType = std::map<Timestamp, ValueType, std::less<Timestamp>, AllocatorType>;
    using BufferItemType = std::pair<const Timestamp, ValueType>;


    /// Ctor
    explicit TemporalBuffer(const Timestamp& buffer_length = 0,
                            const std::string& buffer_id = "ThreadsafeTemporalBuffer") :
        buffer_id_(buffer_id), buffer_length_(buffer_length)
    {
    }


    /// Dtor
    virtual ~TemporalBuffer() = default;


    /// Basic operations

    inline std::string get_id() const
    {
        return buffer_id_;
    }

    /**
     * @brief Checks if the buffer is empty.
     * @return True if the buffer is empty.
     */
    inline bool empty() const
    {
        return buffer_.empty();
    }

    /**
     * @brief Get the size of buffer.
     * @return Size of the stored elements.
     */
    inline size_t size() const
    {
        return buffer_.size();
    }


    /**
     * @brief Clear the buffer.
     */
    inline void clear()
    {
        buffer_.clear();
    }


    inline bool get_oldest(ValueType* value, Timestamp* value_timestamp = nullptr) const
    {
        CHECK_NOTNULL(value);

        // Return false if no values in buffer.
        if (empty())
        {
            return false;
        }

        if (value_timestamp)
        {
            *value_timestamp = buffer_.begin()->first;
        }
        *value = buffer_.begin()->second;

        return true;
    }


    inline bool get_newest(ValueType* value, Timestamp* value_timestamp = nullptr) const
    {
        CHECK_NOTNULL(value);

        // Return false if no values in buffer.
        if (empty())
        {
            return false;
        }

        if (value_timestamp)
        {
            *value_timestamp = buffer_.rbegin()->first;
        }
        *value = buffer_.rbegin()->second;

        return true;
    }


    inline bool add_value(const Timestamp& timestamp, ValueType value)
    {
        const bool value_overwritten = buffer_.emplace(timestamp, std::move(value)).second;
        remove_outdated_items();
        return value_overwritten;
    }


    inline bool get_value_at(const Timestamp& timestamp, ValueType* value = nullptr) const
    {
        auto it = buffer_.find(timestamp);
        if (it != buffer_.end())
        {
            if (value)
            {
                *value = it->second;
            }
            return true;
        }
        else
        {
            return false;
        }
    }


    inline bool get_nearest_to(const Timestamp& timestamp,
                               ValueType* value = nullptr,
                               Timestamp* value_timestamp = nullptr,
                               const Timestamp& maximum_delta = std::numeric_limits<Timestamp>::max()) const
    {
        // Return false if the buffer is empty.
        if (empty())
        {
            return false;
        }

        auto it_lower = buffer_.lower_bound(timestamp);

        // Verify if we got an exact match.
        if (it_lower != buffer_.end() && it_lower->first == timestamp)
        {
            if (value_timestamp)
            {
                *value_timestamp = it_lower->first;
            }
            if (value)
            {
                *value = it_lower->second;
            }
            return true;
        }

        // If the lower bound points out of the array, we have to return the last element.
        if (it_lower == buffer_.end())
        {
            auto it_last = std::prev(buffer_.end());
            Timestamp delta = std::abs(it_last->first - timestamp);
            if (delta <= maximum_delta)
            {
                if (value_timestamp)
                {
                    *value_timestamp = it_last->first;
                }
                if (value)
                {
                    *value = it_last->second;
                }
                return true;
            }
            else
            {
                return false;
            }
        }

        // If the lower bound points to begin() and no exact match was found, we have to return the first element.
        if (it_lower == buffer_.begin())
        {
            Timestamp delta = std::abs(it_lower->first - timestamp);
            if (delta <= maximum_delta)
            {
                if (value)
                {
                    *value = it_lower->second;
                }
                if (value_timestamp)
                {
                    *value_timestamp = it_lower->first;
                }
                return true;
            }
            else
            {
                return false;
            }
        }

        // Both iterators are within range so need to find out which of them is closer to the timestamp.
        auto it_before = std::prev(it_lower);
        const Timestamp delta_before = std::abs(it_before->first - timestamp);
        const Timestamp delta_after = std::abs(it_lower->first - timestamp);
        if (delta_before < delta_after)
        {
            if (delta_before <= maximum_delta)
            {
                if (value)
                {
                    *value = it_before->second;
                }
                if (value_timestamp)
                {
                    *value_timestamp = it_before->first;
                }
                return true;
            }
        }
        else
        {
            if (delta_after <= maximum_delta)
            {
                if (value)
                {
                    *value = it_lower->second;
                }
                if (value_timestamp)
                {
                    *value_timestamp = it_lower->first;
                }
                return true;
            }
        }
        return false;
    }


    bool get_iter_at_or_before(const Timestamp& timestamp,
                               typename BufferType::const_iterator* it_lower) const
    {
        // Return false if no values in buffer.
        if (empty())
        {
            return false;
        }

        // Returns first element with a time that compares not less to timestamp.
        auto it_lower_bound = buffer_.lower_bound(timestamp);

        if (it_lower)
        {
            *it_lower = std::move(it_lower_bound);
        }

        return it_lower_bound != buffer_.end() && it_lower_bound->first == timestamp;
    }


    inline bool get_at_or_before(const Timestamp& timestamp,
                                 ValueType* value = nullptr,
                                 Timestamp* value_timestamp = nullptr) const
    {
        // Return false if no values in buffer.
        if (empty())
        {
            return false;
        }

        typename BufferType::const_iterator it_lower_bound;
        const bool has_exact_match = get_iter_at_or_before(timestamp, &it_lower_bound);

        if (!has_exact_match)
        {
            if (it_lower_bound == buffer_.begin())
            {
                return false;
            }

            --it_lower_bound;
        }

        if (value)
        {
            *value = it_lower_bound->second;
        }
        if (value_timestamp)
        {
            *value_timestamp = it_lower_bound->first;
        }

        DCHECK_LE(it_lower_bound->first, timestamp);
        return true;
    }


    inline bool get_at_or_after(const Timestamp& timestamp,
                                ValueType* value = nullptr,
                                Timestamp* value_timestamp = nullptr) const
    {
        // Return false if no values in buffer.
        if (empty())
        {
            return false;
        }

        typename BufferType::const_iterator it_lower_bound;
        const bool has_exact_match = get_iter_at_or_before(timestamp, &it_lower_bound);

        if (!has_exact_match)
        {
            if (it_lower_bound == buffer_.end())
            {
                return false;
            }

        }
        if (value)
        {
            *value = it_lower_bound->second;
        }
        if (value_timestamp)
        {
            *value_timestamp = it_lower_bound->first;
        }

        DCHECK_GE(it_lower_bound->first, timestamp);
        return true;
    }


    /**
     * @brief Get a series values between given time border.
     * @param[in] lower Try to get the values from this time [ns].
     * @param[in] higher Try to get the values up this time [ns].
     * @param[out] value_container Returned list of values.
     * @param[out] stamp_container Returned list of timestamps.
     * @param[in] include_lower
     * @return The return code signal
     */
    template <typename Alloc = std::allocator<ValueType>>
    inline TemporalBufferQueryResult
    get_value_between(const Timestamp& lower,
                      const Timestamp& higher,
                      std::vector<ValueType, Alloc>* value_container,
                      std::vector<Timestamp>* stamp_container,
                      bool include_lower = false)
    {
        CHECK_NOTNULL(value_container)->clear();
        if(stamp_container) stamp_container->clear();

        if (auto result = is_data_available(lower, higher); result != TemporalBufferQueryResult::AVAILABLE)
        {
            return result;
        }

        // Find all item between lower and higher
        auto it = buffer_.lower_bound(lower);

        // Find all item between (or equal when include_lower is true) lower and higher
        for (; it != buffer_.end() && it->first < higher; ++it)
        {
            if (!include_lower && it->first == lower)
            {
                // Skip append current item if we don't need the border
                continue;
            }

            value_container->emplace_back(it->second);
            if(stamp_container) stamp_container->emplace_back(it->first);
        }

        return value_container->empty() ?
               TemporalBufferQueryResult::TOO_FEW_MEASUREMENT : TemporalBufferQueryResult::AVAILABLE;
    }


    /**
     * @brief Get a series values between given time border.
     * @param[in] lower Try to get the values from this time [ns]. Must larger or equal then the oldest stamp in buffer.
     * @param[in] higher Try to get the values up this time [ns]. Must less or equal then the newest stamp in buffer.
     * @param[out] value_container Returned list of values.
     * @param[out] stamp_container Returned list of timestamps.
     * @param[in] with_left_border_interpolated
     * @param[in] with_right_border_interpolated
     * @return The return code signal
     */
    template <typename Alloc = std::allocator<ValueType>>
    inline TemporalBufferQueryResult
    get_value_btw_with_interp(const Timestamp& lower,
                              const Timestamp& higher,
                              std::vector<ValueType, Alloc>* value_container,
                              std::vector<Timestamp>* stamp_container,
                              bool with_left_border_interpolated = false,
                              bool with_right_border_interpolated = false)
    {
        CHECK_NOTNULL(value_container)->clear();
        if(stamp_container) stamp_container->clear();

        if (auto result = is_data_available(lower, higher); result != TemporalBufferQueryResult::AVAILABLE)
        {
            return result;
        }

        // Find all item between lower and higher
        auto it = buffer_.lower_bound(lower);

        // Interpolation for 'from' side bound, it->timestamp must larger or equal to `lower` and it can not be
        // buffer->begin() as we have check above.
        if (with_left_border_interpolated)
        {
            if (it->first == lower)
            {
                value_container->emplace_back(it->second);
                if(stamp_container) stamp_container->emplace_back(lower);
            }
            else
            {
                // Do interpolate!
                auto it_prev = std::prev(it);

                DCHECK(it_prev->first <= lower);

                ValueType interpolation_value;
                interp_value(it_prev, it, lower, interpolation_value);

                value_container->emplace_back(interpolation_value);
                if(stamp_container)stamp_container->emplace_back(lower);
            }
        }

        // Find all item between (or equal when include_lower is true) lower and higher
        for (; it != buffer_.end() && it->first < higher; ++it)
        {
            if (it->first == lower)
            {
                // Skip append current item
                continue;
            }

            value_container->emplace_back(it->second);
            if(stamp_container) stamp_container->emplace_back(it->first);
        }

        // Interpolation for `to` side bound. it->timestamp must larger or equal to `higher`
        if (with_right_border_interpolated && it != buffer_.end() && it->first >= higher)
        {
            // Do interpolate!
            auto it_prev = std::prev(it);

            DCHECK(it_prev->first <= higher);

            ValueType interpolation_value;
            interp_value(it_prev, it, higher, interpolation_value);

            value_container->emplace_back(interpolation_value);
            if(stamp_container) stamp_container->emplace_back(higher);
        }

        return value_container->empty() ?
               TemporalBufferQueryResult::TOO_FEW_MEASUREMENT : TemporalBufferQueryResult::AVAILABLE;
    }


    inline TemporalBufferQueryResult
    is_data_available(const Timestamp& timestamp_from, const Timestamp& timestamp_to) const
    {
        CHECK_LT(timestamp_from, timestamp_to);

        // Early exit if there are not any items
        if (empty())
        {
            return TemporalBufferQueryResult::TOO_FEW_MEASUREMENT;
        }

        // Early exit if there are not any items in given range
        auto oldest_ts = buffer_.begin()->first;
        auto newest_ts = buffer_.rbegin()->first;

        // `NERVER_AVAILABLE` has a higher priority.
        if (timestamp_from < oldest_ts)
        {
            // This is triggered if the user requests data previous to the oldest
            // measurement present in the buffer, meaning that there is missing data
            // from the timestamp_from requested to the oldest stored timestamp.
            return TemporalBufferQueryResult::NERVER_AVAILABLE;
        }

        if (newest_ts < timestamp_to)
        {
            // This is triggered if the timestamp_to requested exceeds the newest
            // measurement, meaning that there is data still to arrive to reach the
            // requested point in time.
            return TemporalBufferQueryResult::NOT_YAT_AVALIABLE;
        }

        return TemporalBufferQueryResult::AVAILABLE;
    }


protected:


    void interp_value(typename BufferType::const_iterator it_from,
                      typename BufferType::const_iterator it_to,
                      Timestamp t, ValueType& interpolation_value) const
    {
        LinearInterpolation<Timestamp, ValueType>()(it_from->first, it_from->second,
                                                    it_to->first, it_to->second,
                                                    t, interpolation_value);
    }


    inline void remove_outdated_items()
    {
        // Return if the buffer is empty or we set an unlimited time
        if (empty() || buffer_length_ <= 0)
        {
            return;
        }

        const Timestamp& newest_timestamp = buffer_.rbegin()->first;
        const Timestamp buffer_threshold = newest_timestamp - buffer_length_;

        if (buffer_.begin()->first < buffer_threshold)
        {
            auto it = buffer_.lower_bound(buffer_threshold);

            CHECK(it != buffer_.end());
            CHECK(it != buffer_.begin());
            buffer_.erase(buffer_.begin(), it);
        }
    }


protected:

    /// Indicates the ID of ThreadSafe temporal buffer
    std::string buffer_id_;

    /// Max allowed time of buffer
    Timestamp buffer_length_;

    /// Where the data is stored
    BufferType buffer_;
};


/**
 * @brief Basic template class for ThreadsafeTemporalBuffer.
 * @tparam T Stored data type
 */
template <typename ValueType, typename AllocatorType = std::allocator<std::pair<const Timestamp, ValueType>>>
class ThreadsafeTemporalBuffer
{
    // Note that: we cannot make ThreadsafeTemporalBuffer derived from TemporalBuffer. Combining Templated-param
    // with virtual causes some issue.
    using InternalBuffer = TemporalBuffer<ValueType, AllocatorType>;

public:

    POINTER_TYPEDEFS(ThreadsafeTemporalBuffer)

    DELETE_COPY_CONSTRUCTORS(ThreadsafeTemporalBuffer)


    /// Ctor
    explicit ThreadsafeTemporalBuffer(const Timestamp& buffer_length = 0,
                                      const std::string& buffer_id = "ThreadsafeTemporalBuffer") :
        buffer_(buffer_length, buffer_id), shutdown_(false)
    {
    }


    /// Dtor
    ~ThreadsafeTemporalBuffer() = default;


    /**
     * @brief Shutdown the buffer
     */
    inline void shutdown()
    {
        VLOG(3) << "Shutting down buffer: " << buffer_.get_id();

        std::unique_lock<std::mutex> lck(data_mtx_);

        shutdown_ = true;

        lck.unlock();
        data_cv_.notify_all();
    }


    /**
     * @brief Checks if the buffer is shutdown. the state of the buffer might change right after this query.
     */
    inline bool is_shutdown() const
    {
        return shutdown_;
    }


    /**
     * @brief Resume the buffer
     */
    inline void resume()
    {
        VLOG(3) << "Resuming buffer: " << buffer_.get_id();

        std::unique_lock<std::mutex> lck(data_mtx_);

        shutdown_ = false;

        lck.unlock();
        data_cv_.notify_all();
    }


    /**
     * @brief Checks if the buffer is empty. The state of the buffer might change right after this query.
     * @return True if the buffer is empty or shutdown.
     */
    inline bool empty() const
    {
        if (shutdown_)
        {
            return true;
        }

        std::lock_guard<std::mutex> lk(data_mtx_);
        return buffer_.empty();
    }


    /**
     * @brief Get the size of buffer. The state of the buffer might change right after this query.
     * @return Size of the stored elements. Or return 0 if the buffer is down.
     */
    inline size_t size() const
    {
        if (shutdown_)
        {
            return 0u;
        }

        std::lock_guard<std::mutex> lk(data_mtx_);
        return buffer_.size();
    }


    /**
     * @brief Clear the buffer.
     */
    inline void clear()
    {
        if (shutdown_)
        {
            return;
        }

        std::lock_guard<std::mutex> lck(data_mtx_);

        buffer_.clear();
        data_cv_.notify_all();
    }


    bool get_oldest(ValueType* value, Timestamp* value_timestamp = nullptr) const
    {
        if (shutdown_)
        {
            return false;
        }

        std::lock_guard<std::mutex> lck(data_mtx_);
        return buffer_.get_oldest(value, value_timestamp);
    }


    bool get_newest(ValueType* value, Timestamp* value_timestamp = nullptr) const
    {
        if (shutdown_)
        {
            return false;
        }

        std::lock_guard<std::mutex> lck(data_mtx_);
        return buffer_.get_newest(value, value_timestamp);
    }


    bool add_value(const Timestamp& timestamp, ValueType value)
    {
        if (shutdown_)
        {
            return false;
        }

        std::lock_guard<std::mutex> lck(data_mtx_);

        const bool value_overwritten = buffer_.add_value(timestamp, value);

        data_cv_.notify_one();

        return value_overwritten;
    }


    bool get_value_at(const Timestamp& timestamp, ValueType* value = nullptr) const
    {
        if (shutdown_)
        {
            return false;
        }

        std::lock_guard<std::mutex> lck(data_mtx_);
        return buffer_.get_value_at(timestamp, value);
    }


    bool get_nearest_to(const Timestamp& timestamp,
                        ValueType* value = nullptr,
                        Timestamp* value_timestamp = nullptr,
                        const Timestamp& maximum_delta = std::numeric_limits<Timestamp>::max()) const
    {
        if (shutdown_)
        {
            return false;
        }

        std::lock_guard<std::mutex> lck(data_mtx_);
        return buffer_.get_nearest_to(timestamp, value, value_timestamp, maximum_delta);
    }


    bool get_at_or_before(const Timestamp& timestamp,
                          ValueType* value = nullptr,
                          Timestamp* value_timestamp = nullptr) const
    {
        if (shutdown_)
        {
            return false;
        }

        std::lock_guard<std::mutex> lck(data_mtx_);
        return buffer_.get_at_or_before(timestamp, value, value_timestamp);
    }


    bool get_at_or_after(const Timestamp& timestamp,
                         ValueType* value = nullptr,
                         Timestamp* value_timestamp = nullptr) const
    {
        std::lock_guard<std::mutex> lck(data_mtx_);
        return buffer_.get_at_or_after(timestamp, value, value_timestamp);
    }


    TemporalBufferQueryResult
    get_value_between(const Timestamp& lower,
                      const Timestamp& higher,
                      std::vector<ValueType>* value_container,
                      std::vector<Timestamp>* stamp_container,
                      bool include_lower = false)
    {
        // Return if the queue is shutdown
        if (shutdown_)
        {
            return TemporalBufferQueryResult::SHUTDOWN;
        }

        std::lock_guard<std::mutex> lck(data_mtx_);
        return buffer_.get_value_between(lower, higher, value_container, stamp_container, include_lower);
    }


    TemporalBufferQueryResult
    get_value_btw_with_interp(const Timestamp& lower,
                              const Timestamp& higher,
                              std::vector<ValueType>& value_container,
                              std::vector<Timestamp>& stamp_container,
                              bool with_left_border_interpolated = false,
                              bool with_right_border_interpolated = false)
    {
        // Return if the queue is shutdown
        if (shutdown_)
        {
            return TemporalBufferQueryResult::SHUTDOWN;
        }

        std::lock_guard<std::mutex> lck(data_mtx_);
        return buffer_.get_value_btw_with_interp(lower, higher, value_container, stamp_container,
                                                 with_left_border_interpolated,
                                                 with_right_border_interpolated);
    }


    template <typename Alloc = std::allocator<ValueType>>
    TemporalBufferQueryResult
    get_value_between_blocking(const Timestamp& lower,
                               const Timestamp& higher,
                               std::vector<ValueType, Alloc>* value_container,
                               std::vector<Timestamp>* stamp_container,
                               bool include_lower = false,
                               const Timestamp& wait_duration = 10 * ms_to_ns)
    {
        // Lock region
        std::unique_lock<std::mutex> lck(data_mtx_);

        // Wait for data
        TemporalBufferQueryResult query_result;
        data_cv_.wait_for(lck, std::chrono::nanoseconds(wait_duration), [&]
        {
            query_result = buffer_.get_value_between(lower, higher, value_container, stamp_container, include_lower);

            if (shutdown_) return true;
            if (query_result == TemporalBufferQueryResult::NOT_YAT_AVALIABLE) return false;
            return true;
        });

        if (shutdown_)
        {
            return TemporalBufferQueryResult::SHUTDOWN;
        }

        // Unlock before notify.
        lck.unlock();
        data_cv_.notify_one();

        return query_result;
    }

    template <typename Alloc = std::allocator<ValueType>>
    TemporalBufferQueryResult
    get_value_btw_with_interp_blocking(const Timestamp& lower, const Timestamp& higher,
                                       std::vector<ValueType, Alloc>* value_container,
                                       std::vector<Timestamp>*stamp_container,
                                       bool with_left_border_interpolated = false,
                                       bool with_right_border_interpolated = false,
                                       const Timestamp& wait_duration = 1e7)
    {
        // Lock region
        std::unique_lock<std::mutex> lck(data_mtx_);

        // Wait for data
        TemporalBufferQueryResult query_result;
        data_cv_.wait_for(lck, std::chrono::nanoseconds(wait_duration), [&]
        {
            query_result = buffer_.get_value_btw_with_interp(lower, higher, value_container, stamp_container,
                                                             with_left_border_interpolated,
                                                             with_right_border_interpolated);

            if (shutdown_) return true;
            if (query_result == TemporalBufferQueryResult::NOT_YAT_AVALIABLE) return false;
            return true;
        });

        if (shutdown_)
        {
            return TemporalBufferQueryResult::SHUTDOWN;
        }

        // Unlock before notify.
        lck.unlock();
        data_cv_.notify_one();

        return query_result;
    }


protected:

    InternalBuffer buffer_;

    /// Flag for signaling buffer shutdown.
    std::atomic_bool shutdown_;

    mutable std::mutex data_mtx_;

    std::condition_variable data_cv_;

};
