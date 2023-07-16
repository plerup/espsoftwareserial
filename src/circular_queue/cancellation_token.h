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
#include "circular_queue_mp.h"

#include <atomic>
#include <memory>

namespace ghostl
{
    struct cancellation_token_source;

    struct cancellation_state
    {
        void cancel()
        {
            cancelled.store(true);
            queue.for_each([](task_completion_source<bool>&& tcs) { tcs.set_value(true); });
        }
        [[nodiscard]] auto is_cancellation_requested() const
        {
            return cancelled.load();
        }
        [[nodiscard]] auto is_cancellation_requested(task_completion_source<bool>&& tcs)
        {
            queue.push(std::move(tcs));
            return tcs.token();
        }
        std::atomic<bool> cancelled{ false };
        circular_queue_mp<task_completion_source<bool>> queue;
    };

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
    private:
        friend cancellation_token_source;
        std::shared_ptr<cancellation_state> state;
        cancellation_token(decltype(state) _state) : state(std::move(_state)) {}
    };

    struct cancellation_token_source
	{
        explicit cancellation_token_source() {}
        cancellation_token_source(const cancellation_token_source&) = delete;
        cancellation_token_source(cancellation_token_source&& other) noexcept = default;
        auto operator=(const cancellation_token_source&)->cancellation_token_source & = delete;
        auto operator=(cancellation_token_source&& other) noexcept -> cancellation_token_source & = default;
        void cancel()
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
