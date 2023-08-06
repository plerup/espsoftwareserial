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
#include "circular_queue_mp.h"

#include <atomic>
#include <memory>

namespace ghostl
{
    struct cancellation_token_source;

    struct cancellation_state
    {
        ~cancellation_state()
        {
            queue.for_each([](task_completion_source<bool>&& tcs) { tcs.set_value(false); });
        }
        void cancel()
        {
            cancelled.store(true);
            queue.for_each([](task_completion_source<bool>&& tcs) { tcs.set_value(true); });
        }
        [[nodiscard]] auto is_cancellation_requested() const
        {
            return cancelled.load();
        }
        [[nodiscard]] auto token()
        {
            task_completion_source<bool> tcs;
            auto ct = tcs.token();
            queue.push(std::move(tcs));
            return ct;
        }
        std::atomic<bool> cancelled{ false };
        circular_queue_mp<task_completion_source<bool>> queue;
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
        /// <summary>
        /// create a default cancellation token that can never be cancelled
        /// </summary>
        cancellation_token() = default;
        cancellation_token(const cancellation_token&) = default;
        cancellation_token(cancellation_token&& other) noexcept = default;
        auto operator=(const cancellation_token&)->cancellation_token & = default;
        auto operator=(cancellation_token&& other) noexcept -> cancellation_token & = default;
        [[nodiscard]] auto is_cancellation_requested() const
        {
            return state && state->is_cancellation_requested();
        }
        [[nodiscard]] ghostl::task<bool> cancellation_request() const
        {
            if (!state) co_return false;
            if (state->is_cancellation_requested()) co_return true;
            auto token = state->token();
            // if concurrently cancelled, re-cancel to force processing of the new token. 
            if (state->is_cancellation_requested()) state->cancel();
            co_return co_await token;
        }
    private:
        friend cancellation_token_source;
        std::shared_ptr<cancellation_state> state;
        cancellation_token(decltype(state) _state) : state(std::move(_state)) {}
    };

    /// <summary>
    /// The cancellation_token_source class can issue a cancellation request,
    /// similar to what std::stop_token_source does for std::jthread cancellation.
    /// A cancellation request made for one cancellation_token_source object
    /// is visible to all cancellation_token_sources and cancellation_tokens
    /// of the same associated cancellation_state.
    /// </summary>
    struct cancellation_token_source
	{
        explicit cancellation_token_source() {}
        cancellation_token_source(const cancellation_token_source&) = default;
        cancellation_token_source(cancellation_token_source&& other) noexcept = default;
        auto operator=(const cancellation_token_source&)->cancellation_token_source & = default;
        auto operator=(cancellation_token_source&& other) noexcept -> cancellation_token_source & = default;
        void cancel() const
        {
            state->cancel();
        }
        [[nodiscard]] auto is_cancellation_requested() const
        {
            return state->is_cancellation_requested();
        }
        [[nodiscard]] auto token() const { return cancellation_token(state); }
    private:
        decltype(cancellation_token::state) state{ std::make_shared<cancellation_state>() };
	};
} // namespace ghostl
