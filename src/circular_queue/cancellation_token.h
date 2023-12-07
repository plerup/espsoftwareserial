#pragma once
/*
cancellation_token.h - Implementation of a C++20 async coroutines cancellation token.
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

#include "task_completion_source.h"
#include "task.h"
#include "lfllist.h"

#include <atomic>
#include <memory>

namespace ghostl
{
    struct cancellation_token_source;

    struct cancellation_state_type final
    {
        explicit cancellation_state_type() = default;
        cancellation_state_type(const cancellation_state_type&) = delete;
        cancellation_state_type(cancellation_state_type&& other) noexcept = delete;
        auto operator=(const cancellation_state_type&)->cancellation_state_type & = delete;
        auto operator=(cancellation_state_type&& other) noexcept -> cancellation_state_type & = delete;
        ~cancellation_state_type()
        {
            if (!cancelled.load()) list.for_each([](task_completion_source<bool>&& tcs) { auto _tcs = std::move(tcs); _tcs.set_value(false); });
        }
        void cancel()
        {
            if (!cancelled.exchange(true))
            {
                list.for_each([](task_completion_source<bool>&& tcs) { auto _tcs = std::move(tcs); _tcs.set_value(true); });
            }
        }
        [[nodiscard]] auto is_cancellation_requested() const
        {
            return cancelled.load();
        }
        [[nodiscard]] auto make_tcs_list_node()
        {
            return list.emplace_front(task_completion_source<bool>());
        }
        auto erase_tcs_list_node(lfllist<task_completion_source<bool>>::node_type* tcs_list_node) -> void
        {
            list.erase(tcs_list_node);
        }
    private:
        std::atomic<bool> cancelled{ false };
        lfllist<task_completion_source<bool>> list;
    };

    /// <summary>
    /// The cancellation_token class exists to check if a cancellation request
    /// has been made for its associated cancellation_token_source object.
    /// The cancellation_token provides a co_await'able cancellation_request()
    /// member function, that completes if the cancellation_token's associated
    /// cancellation_token_source is cancelled.
    /// </summary>
    struct cancellation_token
    {
    private:
        friend cancellation_token_source;
        std::shared_ptr<cancellation_state_type> state;
        decltype(ghostl::cancellation_state_type().make_tcs_list_node()) tcs_node{nullptr};
        cancellation_token(decltype(state) _state) : state(std::move(_state)) { }
    public:
        /// <summary>
        /// create a default cancellation token that can never be cancelled
        /// </summary>
        cancellation_token() = default;
        cancellation_token(const cancellation_token& other) : state(other.state) { }
        cancellation_token(cancellation_token&& other) noexcept
        {
            state = std::exchange(other.state, nullptr);
        }
        ~cancellation_token()
        {
            if (tcs_node) state->erase_tcs_list_node(tcs_node);
        }
        auto operator=(const cancellation_token& other) -> cancellation_token&
        {
            if (std::addressof(other) != this)
            {
                state = other.state;
            }
            return *this;
        }
        auto operator=(cancellation_token&& other) noexcept -> cancellation_token&
        {
            if (std::addressof(other) != this)
            {
                state = std::exchange(other.state, nullptr);
            }
            return *this;
        }
        [[nodiscard]] auto is_cancellation_requested() const
        {
            return !state || state->is_cancellation_requested();
        }
        /// <summary>
        /// Get an awaitable that completes if the cancellation_token's associated
        /// cancellation_token_source is cancelled. Never call this more than once
        /// on the same cancellation_token, but get another cancellation token.
        /// </summary>
        /// <returns>A bool task, that has a value of true if cancellation occurred,
        /// or false if the cancellation source etc was deleted without cancellation.</returns>
        [[nodiscard]] auto cancellation_request() -> ghostl::task<bool>
        {
            if (!state || state->is_cancellation_requested()) co_return true;
            tcs_node = state->make_tcs_list_node();
            // if concurrently cancelled, re-cancel to force processing of the new token. 
            if (state->is_cancellation_requested()) state->cancel();
            auto token = tcs_node->item.token();
            tcs_node = nullptr;
            co_return co_await token;
        }
    };

    /// <summary>
    /// The cancellation_token_source class can issue a cancellation request,
    /// similar to what std::stop_token_source does for std::jthread cancellation.
    /// A cancellation request made for one cancellation_token_source object
    /// is visible to all cancellation_token_sources and cancellation_tokens
    /// of the same associated cancellation state.
    /// </summary>
    struct cancellation_token_source
    {
        explicit cancellation_token_source() = default;
        cancellation_token_source(const cancellation_token_source& other) : state(other.state) { }
        cancellation_token_source(cancellation_token_source&& other) noexcept
        {
            state = std::exchange(other.state, nullptr);
        }
        auto operator=(const cancellation_token_source& other) -> cancellation_token_source&
        {
            if (std::addressof(other) != this)
            {
                state = other.state;
            }
            return *this;
        }
        auto operator=(cancellation_token_source&& other) noexcept -> cancellation_token_source&
        {
            if (std::addressof(other) != this)
            {
                state = std::exchange(other.state, nullptr);
            }
            return *this;
        }
        void cancel() const
        {
            std::shared_ptr<cancellation_state_type> _state(state);
            if (_state) _state->cancel();
        }
        [[nodiscard]] auto is_cancellation_requested() const
        {
            return state ? state->is_cancellation_requested() : true;
        }
        [[nodiscard]] auto token() const { return cancellation_token(state); }
    private:
        std::shared_ptr<cancellation_state_type> state{ std::make_shared<cancellation_state_type>() };
    };
} // namespace ghostl
