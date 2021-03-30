#pragma once

#include <atomic>
#include <thread>
#include <functional>


template <class F, class... Args>
void setInterval(std::atomic_bool& cancel_token, size_t interval, F&& f, Args&& ... args)
{
    cancel_token.store(true);
    auto cb = std::bind(std::forward<F>(f), std::forward<Args>(args)...);
    std::thread t([=, &cancel_token]() mutable
                  {
                      while (cancel_token.load())
                      {
                          cb();
                          std::this_thread::sleep_for(std::chrono::milliseconds(interval));
                      }
                  });
    t.detach();
}

struct TicToc
{
    using TimePoint = std::chrono::high_resolution_clock::time_point;

    static TimePoint tic()
    {
        return std::chrono::high_resolution_clock::now();
    }

    /**
     * @brief Stop timer and report duration in given time.
     * call .count() on returned duration to have number of ticks.
     * @return duration in milliseconds by default.
     */
    template <typename T = std::chrono::milliseconds>
    static T toc(const TimePoint& start)
    {
        return std::chrono::duration_cast<T>(std::chrono::high_resolution_clock::now() - start);
    }
};


/// @brief Usage: measure<>::execution(function, arguments)
template <typename T = std::chrono::nanoseconds>
struct Measure
{
    template <typename Callable, typename... Args>
    static typename T::rep execution(Callable&& func, Args&& ... args)
    {
        static_assert(std::is_invocable<typename std::decay<Callable>::type,
                          typename std::decay<Args>::type...>::value,
                      "Measure<> arguments must be invocable after conversion to rvalues");

        auto start = std::chrono::steady_clock::now();
        std::forward<Callable>(func)(std::forward<Args>(args)...);
        auto duration = std::chrono::duration_cast<T>(std::chrono::steady_clock::now() - start);
        return duration.count();
    }
};