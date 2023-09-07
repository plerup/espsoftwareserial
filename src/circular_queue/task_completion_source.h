#pragma once
/*
task_completion_source.h - Implementation of a C++20 async coroutines task completion source.
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

#include <atomic>
#include <memory>
#include <coroutine>

#if defined(__GNUC__)
#undef ALWAYS_INLINE_ATTR
#define ALWAYS_INLINE_ATTR __attribute__((always_inline))
#else
#define ALWAYS_INLINE_ATTR
#endif

namespace ghostl
{
template<typename T = void>
struct task_completion_source
{
    task_completion_source() = default;
    task_completion_source(const task_completion_source& other) noexcept : state(other.state) { }
    task_completion_source(task_completion_source&& other) noexcept
    {
        state = std::exchange(other.state, nullptr);
    }
    auto operator=(const task_completion_source& other) -> task_completion_source&
    {
        if (std::addressof(other) != this)
        {
             state = other.state;
        }
        return *this;
    }
    auto operator=(task_completion_source&& other) noexcept -> task_completion_source&
    {
        if (std::addressof(other) != this)
        {
            state = std::exchange(other.state, nullptr);
        }
        return *this;
    }
    auto set_value(T&& val) const -> void
    {
        for (bool expect{false}; !state->is_set.compare_exchange_strong(expect, true);)
            return;
        state->value = std::make_shared<T>(std::move(val));
        std::atomic_thread_fence(std::memory_order_release);
        for (bool expect{false}; !state->ready.compare_exchange_weak(expect, true); expect = false) {}
        if (auto handle = state->coroutine.load(); handle && !handle.done()) { handle.resume(); }
    }
    auto set_value(const T& val) const -> void ALWAYS_INLINE_ATTR
    {
        T v(val);
        set_value(std::move(v));
    }
    [[nodiscard]] auto token() const { return awaiter(state); }
private:
    struct state_type final
    {
        explicit state_type() {}
        state_type(const state_type&)                = delete;
        state_type(state_type&& other) noexcept      = delete;
        auto                                 operator=(const state_type&) -> state_type& = delete;
        auto                                 operator=(state_type&& other) noexcept -> state_type& = delete;
        std::atomic<bool>                    is_set{false};
        std::atomic<bool>                    ready{false};
        std::atomic<std::coroutine_handle<>> coroutine{nullptr};
        std::shared_ptr<T>                   value;
    };
    struct awaiter final
    {
        awaiter() = delete;
        explicit awaiter(std::shared_ptr<state_type> _state) : state(std::move(_state)) {}
        awaiter(const awaiter&)           = default;
        awaiter(awaiter&& other) noexcept = default;
        auto                        operator=(const awaiter&) -> awaiter& = default;
        auto                        operator=(awaiter&& other) noexcept -> awaiter& = default;
        bool await_ready() const noexcept { return state->ready.exchange(true); }
        void await_suspend(std::coroutine_handle<> handle) const noexcept
        {
            state->coroutine.store(handle);
            state->ready.store(false);
        }
        T await_resume() const noexcept { return *state->value; }
    private:
        std::shared_ptr<state_type> state;
    };

    std::shared_ptr<state_type> state{std::make_shared<state_type>()};
};

template<>
struct task_completion_source<void>
{
    task_completion_source() = default;
    task_completion_source(const task_completion_source& other) noexcept : state(other.state) { }
    task_completion_source(task_completion_source&& other) noexcept
    {
        state = std::exchange(other.state, nullptr);
    }
    auto operator=(const task_completion_source& other) -> task_completion_source&
    {
        if (std::addressof(other) != this)
        {
             state = other.state;
        }
        return *this;
    }
    auto operator=(task_completion_source&& other) noexcept -> task_completion_source&
    {
        if (std::addressof(other) != this)
        {
            state = std::exchange(other.state, nullptr);
        }
        return *this;
    }
    void set_value() const
    {
        for (bool expect{false}; !state->is_set.compare_exchange_strong(expect, true);)
            return;
        std::atomic_thread_fence(std::memory_order_release);
        for (bool expect{false}; !state->ready.compare_exchange_weak(expect, true); expect = false) {}
        if (auto handle = state->coroutine.load(); handle && !handle.done()) { handle.resume(); }
    }
    [[nodiscard]] auto token() const { return awaiter(state); }
private:
    struct state_type final
    {
        explicit state_type() {}
        state_type(const state_type&)                = delete;
        state_type(state_type&& other) noexcept      = delete;
        auto                                 operator=(const state_type&) -> state_type& = delete;
        auto                                 operator=(state_type&& other) noexcept -> state_type& = delete;
        std::atomic<bool>                    is_set{false};
        std::atomic<bool>                    ready{false};
        std::atomic<std::coroutine_handle<>> coroutine{nullptr};
    };
    struct awaiter final
    {
        awaiter() = delete;
        explicit awaiter(std::shared_ptr<state_type> _state) : state(std::move(_state)) {}
        awaiter(const awaiter&)           = default;
        awaiter(awaiter&& other) noexcept = default;
        auto                        operator=(const awaiter&) -> awaiter& = default;
        auto                        operator=(awaiter&& other) noexcept -> awaiter& = default;
        bool await_ready() const noexcept { return state->ready.exchange(true); }
        void await_suspend(std::coroutine_handle<> handle) const noexcept
        {
            state->coroutine.store(handle);
            state->ready.store(false);
        }
        constexpr void await_resume() const noexcept {}
    private:
        std::shared_ptr<state_type> state;
    };

    std::shared_ptr<state_type> state{std::make_shared<state_type>()};
};

} // namespace ghostl
