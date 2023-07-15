#pragma once

#include "task.h"
#include "task_completion_source.h"

#include <vector>

namespace ghostl
{
	template<typename T = void>
	struct when_any
	{
		std::shared_ptr<std::vector<ghostl::task<>>> continuations{
			std::make_shared<std::vector<ghostl::task<>>>() };
		std::shared_ptr<std::atomic<bool>> completed{
			std::make_shared<std::atomic<bool>>(false) };
		ghostl::task_completion_source<T> tcs{};

		auto run_task(ghostl::task<T> t,
			std::shared_ptr<std::vector<ghostl::task<>>> _continuations,
			std::shared_ptr<std::atomic<bool>> _completed) -> ghostl::task<>
		{
			auto continuations = std::move(_continuations);
			auto completed = std::move(_completed);
			auto result = std::move(co_await t);
			if (auto isCompleted = completed->exchange(true); !isCompleted)
			{
				tcs.set_value(std::move(result));
				if (auto handle = tcs.handle(); handle && !handle.done()) {
					handle.resume();
				}
			}
			co_return;
		}

		when_any() = delete;
		template<typename C> explicit when_any(C&& tasks)
		{
			auto completed = this->completed;
			for (auto& task : tasks)
			{
				continuations->emplace_back(run_task(std::exchange(task, {}), continuations, completed));
			}
			for (auto& task : *continuations)
			{
				task.resume();
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
	struct when_any<>
	{
		std::shared_ptr<std::vector<ghostl::task<>>> continuations{
			std::make_shared<std::vector<ghostl::task<>>>() };
		std::shared_ptr<std::atomic<bool>> completed{
			std::make_shared<std::atomic<bool>>(false) };
		ghostl::task_completion_source<> tcs{};

		auto run_task(ghostl::task<> t,
			std::shared_ptr<std::vector<ghostl::task<>>> _continuations,
			std::shared_ptr<std::atomic<bool>> _completed) -> ghostl::task<>
		{
			auto continuations = std::move(_continuations);
			auto completed = std::move(_completed);
			co_await t;
			if (auto isCompleted = completed->exchange(true); !isCompleted)
			{
				tcs.set_value();
				if (auto handle = tcs.handle(); handle && !handle.done()) {
					handle.resume();
				}
			}
			co_return;
		}

		when_any() = delete;
		template<typename C> explicit when_any(C&& tasks)
		{
			auto completed = this->completed;
			for (auto& task : tasks)
			{
				continuations->emplace_back(run_task(std::exchange(task, {}), continuations, completed));
			}
			for (auto& task : *continuations)
			{
				task.resume();
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
