#pragma once
/*
when_any.h - Implementation of a C++20 async coroutines when-any awaiter.
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

#include "run_task.h"
#include "task_completion_source.h"

#include <vector>

namespace ghostl
{
	template<typename T = void>
	struct when_any
	{
		std::shared_ptr<std::vector<ghostl::run_task<T>>> continuations{
			std::make_shared<std::vector<ghostl::run_task<T>>>() };
		std::shared_ptr<std::atomic<bool>> completed{
			std::make_shared<std::atomic<bool>>(false) };
		ghostl::task_completion_source<T> tcs{};

		static auto continuation(T result,
			std::shared_ptr<std::atomic<bool>> completed,
			ghostl::task_completion_source<T> tcs) -> void
		{
			if (auto isCompleted = completed->exchange(true); !isCompleted)
			{
				tcs.set_value(std::move(result));
			}
		}

		when_any() = delete;
		template<typename C> explicit when_any(C&& tasks)
		{
			auto completed = this->completed;
			auto tcs = this->tcs;
			for (ghostl::task<T>& task : tasks)
			{
				auto runner = ghostl::run_task<T>(std::move(std::exchange(task, {})));
				runner.continue_with([completed, tcs](T result) { continuation(result, completed, tcs); });
				continuations->emplace_back(std::move(runner));
			}
			for (auto& runner : *continuations)
			{
				runner.resume();
			}
		}
		when_any(const when_any& other) = delete;
		when_any(when_any&& other) = delete;
		when_any& operator=(when_any& other) = delete;
		when_any& operator=(when_any&& other) = delete;
		auto operator ()() {
			return tcs.token();
		}
	};
	template<>
	struct when_any<void>
	{
		std::shared_ptr<std::vector<ghostl::run_task<>>> continuations{
			std::make_shared<std::vector<ghostl::run_task<>>>() };
		std::shared_ptr<std::atomic<bool>> completed{
			std::make_shared<std::atomic<bool>>(false) };
		ghostl::task_completion_source<> tcs{};

		static auto continuation(
			std::shared_ptr<std::atomic<bool>> completed,
			ghostl::task_completion_source<> tcs) -> void
		{
			if (auto isCompleted = completed->exchange(true); !isCompleted)
			{
				tcs.set_value();
			}
		}

		when_any() = delete;
		template<typename C> explicit when_any(C&& tasks)
		{
			auto completed = this->completed;
			auto tcs = this->tcs;
			for (ghostl::task<>& task : tasks)
			{
				auto runner = ghostl::run_task<>(std::move(std::exchange(task, {})));
				runner.continue_with([completed, tcs]() { continuation(completed, tcs); });
				continuations->emplace_back(std::move(runner));
			}
			for (auto& runner : *continuations)
			{
				runner.resume();
			}
		}
		when_any(const when_any& other) = delete;
		when_any(when_any&& other) = delete;
		when_any& operator=(when_any& other) = delete;
		when_any& operator=(when_any&& other) = delete;
		auto operator ()() {
			return tcs.token();
		}
	};
} // namespace ghostl
