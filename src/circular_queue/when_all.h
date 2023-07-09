#pragma once

#include <atomic>
#include <memory>
#include "task.h"
#include "task_completion_source.h"

namespace ghostl
{
	template<typename T, size_t S>
	struct when_all
	{
		std::shared_ptr<std::array<task<void>, S>> continuations =
			std::make_shared<std::array<task<void>, S>>();
		std::shared_ptr<size_t> completedCnt =
			std::make_shared<size_t>();
		size_t size;
		std::shared_ptr<ghostl::task_completion_source<>> tcs =
			std::make_shared<ghostl::task_completion_source<>>();

		auto make_continuation(T& t) -> task<void>
		{
			auto c = continuations;
			auto tc = tcs;
			auto cc = completedCnt;
			auto value = co_await t;
			if (++*cc == size)
			{
				tc->set_value();
				if (auto handle = tc->handle(); handle && !handle.done()) {
					handle.resume();
				}
			}
			co_return;
		}

		when_all() = delete;
		explicit when_all(std::array<T, S>& tasks)
		{
			size = tasks.size();
			*completedCnt = 0;
			for (size_t i = 0; i < size; ++i)
			{
				continuations->at(i) = make_continuation(tasks[i]);
				continuations->at(i).resume();
			}
		}
		when_all(const when_all& other) = delete;
		when_all(when_all&& other) = delete;
		when_all& operator=(when_all& other) = delete;
		when_all& operator=(when_all&& other) = delete;
		//auto operator co_await() { return tcs->token(); }
		ghostl::task<> token() {
			auto c = continuations;
			auto tc = tcs;
			auto cc = completedCnt;
			co_await tc->token();
		}
	};

	template<size_t S>
	struct when_all<task<void>, S>
	{
		typedef task<void> T;
		std::shared_ptr<std::array<task<void>, S>> continuations =
			std::make_shared<std::array<task<void>, S>>();
		std::shared_ptr<size_t> completedCnt =
			std::make_shared<size_t>();
		size_t size;
		std::shared_ptr<ghostl::task_completion_source<>> tcs =
			std::make_shared<ghostl::task_completion_source<>>();

		auto make_continuation(T& t) -> task<void>
		{
			auto c = continuations;
			auto tc = tcs;
			auto cc = completedCnt;
			Serial.printf("co_await t; completedCnt == %u/%u\n",
				*cc, size);
			co_await t;
			Serial.printf("done: co_await t; completedCnt == %u/%u\n",
				*cc + 1, size);
			if (++*cc == size)
			{
				Serial.println("tc->set_value();");
				tc->set_value();
				Serial.println("done: tc->set_value();");
				if (auto handle = tc->handle(); handle && !handle.done()) {
					Serial.println("handle.resume();");
					handle.resume();
				}
			}
			co_return;
		}

		when_all() = delete;
		explicit when_all(std::array<T, S>& tasks)
		{
			size = tasks.size();
			*completedCnt = 0;
			for (size_t i = 0; i < size; ++i)
			{
				continuations->at(i) = make_continuation(tasks[i]);
				continuations->at(i).resume();
			}
		}
		when_all(const when_all& other) = delete;
		when_all(when_all&& other) = delete;
		when_all& operator=(when_all& other) = delete;
		when_all& operator=(when_all&& other) = delete;
		//auto operator co_await() { return tcs->token(); }
		ghostl::task<> token() {
			auto c = continuations;
			auto tc = tcs;
			auto cc = completedCnt;
			Serial.println("co_await ts->token()");
			co_await tc->token();
			Serial.println("done: co_await ts->token()");
		}
	};
} // namespace ghostl
