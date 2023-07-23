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
		std::function<void(T)> continuation;
		ghostl::task<T> task;
		ghostl::details::final_task final_task;
		void continue_with(std::function<void(T)>&& cont)
		{
			continuation = std::move(cont);
		};
		ghostl::details::final_task coroutine(ghostl::task<T>&& t) {
			auto task = std::move(t);
			auto cont = std::move(continuation);
			T res = co_await task;
			if (cont) cont(res);
		};
		run_task(ghostl::task<T>&& t) {
			task = std::move(t);
		}
		void resume() {
			auto t = std::move(task);
			final_task = coroutine(std::move(t));
		}
		void destroy() {
			final_task.coroutine.destroy();
		}
	};
	template<>
	struct run_task<void>
	{
		std::function<void()> continuation;
		ghostl::task<> task;
		ghostl::details::final_task final_task;
		void continue_with(std::function<void()>&& cont)
		{
			continuation = std::move(cont);
		};
		ghostl::details::final_task coroutine(ghostl::task<>&& t) {
			auto task = std::move(t);
			auto cont = std::move(continuation);
			co_await task;
			if (cont) cont();
		};
		run_task(ghostl::task<>&& t) {
			task = std::move(t);
		}
		void resume() {
			auto t = std::move(task);
			final_task = coroutine(std::move(t));
		}
		void destroy() {
			final_task.coroutine.destroy();
		}
	};
} // namespace ghostl
