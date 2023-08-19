#pragma once
/*
run_task.h - Implementation of a C++20 async coroutines task runner featuring continue_with.
Copyright (c) 2023 Dirk O. Kaar. All rights reserved.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "task.h"

#include <functional>
#include <atomic>

namespace ghostl
{
    namespace details
    {
        struct final_task final
        {
            struct promise_type final
            {
                final_task get_return_object() noexcept
                {
                    return { std::coroutine_handle<promise_type>::from_promise(*this) };
                }
                constexpr std::suspend_never initial_suspend() const noexcept { return {}; }
                void unhandled_exception() const { std::rethrow_exception(std::current_exception()); }
                constexpr void return_void() const noexcept {}
                constexpr std::suspend_never final_suspend() const noexcept { return {}; }
            };
            final_task() = default;
            final_task(std::coroutine_handle<promise_type>&& handle) : coroutine(std::move(handle)) {}
            std::coroutine_handle<promise_type> coroutine;
        };
    }

    template<typename T = void>
    struct run_task
    {
        /// <summary>
        /// Provide a non-coroutine continuation to run when the task completes.
        /// </summary>
        /// <typeparam name="F"></typeparam>
        /// <param name="cont">The continuation function. Caveat: lambda captures can leak when the function is never invoked due to prior cancellation etc. of the task.</param>
        template<typename F> void continue_with(F cont)
        {
            continuation = std::move(cont);
        };
        run_task() = delete;
        run_task(const ghostl::run_task<T>& other) = delete;
        run_task(ghostl::run_task<T>&& other) noexcept :
            task(std::exchange(other.task, {})),
            continuation(std::exchange(other.continuation, nullptr)),
            final_task(std::exchange(other.final_task, {})) { }
        run_task(ghostl::task<T>&& t) noexcept : task(std::move(t)) { }

        auto operator=(const ghostl::run_task<T>& other) ->ghostl::run_task<T> & = delete;
        auto operator=(ghostl::run_task<T>&& other) noexcept -> ghostl::run_task<T>&
        {
            if (std::addressof(other) != this)
            {
                task = std::exchange(other.task, {});
                continuation = std::exchange(other.continuation, nullptr);
                final_task = std::exchange(other.final_task, {});
            }
            return *this;
        }

        void resume() {
            final_task = coroutine();
        }
    private:
        ghostl::details::final_task coroutine() {
            auto t = std::exchange(task, {});
            auto cont = std::exchange(continuation, nullptr);
            T res = co_await t;
            if (cont) cont(res);
        };
        ghostl::task<T> task;
        std::function<void(T)> continuation;
        ghostl::details::final_task final_task;
    };
    template<>
    struct run_task<void>
    {
        /// <summary>
        /// Provide a non-coroutine continuation to run when the task completes.
        /// </summary>
        /// <typeparam name="F"></typeparam>
        /// <param name="cont">The continuation function. Caveat: lambda captures can leak when the function is never invoked due to prior cancellation etc. of the task.</param>
        template<typename F> void continue_with(F cont)
        {
            continuation = std::move(cont);
        };
        run_task() = delete;
        run_task(const ghostl::run_task<>& other) = delete;
        run_task(ghostl::run_task<>&& other) noexcept :
            task(std::exchange(other.task, {})),
            continuation(std::exchange(other.continuation, nullptr)),
            final_task(std::exchange(other.final_task, {})) { }
        run_task(ghostl::task<>&& t) noexcept : task(std::move(t)) { }

        auto operator=(const ghostl::run_task<>& other) ->ghostl::run_task<> & = delete;
        auto operator=(ghostl::run_task<>&& other) noexcept -> ghostl::run_task<>&
        {
            if (std::addressof(other) != this)
            {
                task = std::exchange(other.task, {});
                continuation = std::exchange(other.continuation, nullptr);
                final_task = std::exchange(other.final_task, {});
            }
            return *this;
        }

        void resume() {
            final_task = coroutine();
        }
        void destroy() {
            if (auto handle = std::exchange(final_task.coroutine, nullptr); handle && handle.done())
            {
                handle.destroy();
            }
        }
    private:
        ghostl::details::final_task coroutine() {
            auto t = std::exchange(task, {});
            auto cont = std::exchange(continuation, nullptr);
            co_await t;
            if (cont) cont();
        };
        ghostl::task<> task;
        std::function<void()> continuation;
        ghostl::details::final_task final_task;
    };
} // namespace ghostl
