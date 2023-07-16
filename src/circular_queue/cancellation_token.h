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

#include <atomic>
#include <memory>

namespace ghostl
{
    struct cancellation_token_source;

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
        [[nodiscard]] bool is_cancellation_requested() const
        {
            return state && state->load();
        }
    private:
        friend cancellation_token_source;
        std::shared_ptr<std::atomic<bool>> state;
        cancellation_token(decltype(state) _state) : state(_state) {}
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
            state->store(true);
        }
        [[nodiscard]] bool is_cancellation_requested() const
        {
            return state->load();
        }
        [[nodiscard]] auto token() const { return cancellation_token(state); }
    private:
        decltype(cancellation_token::state) state{ std::make_shared<std::atomic<bool>>(false) };
	};
} // namespace ghostl
