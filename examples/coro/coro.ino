#include <coroutine>
#include <string>
#include <sstream>
#include <vector>

#ifdef ESP8266
#include <Schedule.h>
#include <SoftwareSerial.h>
#include <circular_queue/ghostl.h>
#else
#include <FastScheduler.h>
#endif

#ifdef ARDUINO
#define PRINTF Serial.printf
#define PRINTLN Serial.println
#elif defined(__ZEPHYR__)
#include <zephyr/kernel.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/sys/printk.h>
#define PRINTF printk
#define PRINTLN(s) printk("%s\n", (s))
#else
#define PRINTF printf
#define PRINTLN puts
#endif

#ifdef __ZEPHYR__
#include <zephyr/kernel.h>
namespace {
	long unsigned micros() { return static_cast<long unsigned>(k_uptime_get() * 1000UL); }
	long unsigned millis() { return static_cast<long unsigned>(k_uptime_get()); }
}
#elif !defined(ARDUINO) // __ZEPHYR__
#include <thread>
#include <chrono>
namespace {
    long unsigned micros()
    {
        static auto start = std::chrono::steady_clock::now();
        return static_cast<long unsigned>(std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - start).count());
    }
	long unsigned millis()
	{
		static auto start = std::chrono::steady_clock::now();
		return static_cast<long unsigned>(std::chrono::duration_cast<std::chrono::milliseconds>(
			std::chrono::steady_clock::now() - start).count());
	}
}
#endif // __ZEPHYR__

//#include <Ticker.h>

#include <utility>

#include <circular_queue/task_completion_source.h>
#include <circular_queue/task.h>
#include <circular_queue/generator.h>
#include <circular_queue/when_all.h>
#include <circular_queue/when_any.h>
#include <circular_queue/cancellation_token.h>
#include <circular_queue/run_task.h>

constexpr unsigned DELAY = 10000;

struct schedule : std::suspend_always
{
	ghostl::cancellation_token ct;
	schedule(ghostl::cancellation_token _ct = {}) : ct(_ct) {}

	void await_suspend(std::coroutine_handle<> handle)
	{
		schedule_function([handle]() {
			if (handle && !handle.done()) handle.resume();
		});
	}
};

struct co_delay : std::suspend_always
{
	co_delay(decltype(micros()) _delay_us, ghostl::cancellation_token _ct = {}) : delay_us(_delay_us), ct(_ct) {}

	decltype(micros()) delay_us;
	ghostl::cancellation_token ct;
	//Ticker ticker;
	//std::coroutine_handle<> coroutine;

	void await_suspend(std::coroutine_handle<> handle)
	{
		// Using many Tickers at once appears to crash the ESP32...
		//coroutine = handle;
		//struct local {
		//	static auto schedule(co_delay* self) -> void {
		//		schedule_function([self]() { self->coroutine.resume(); });
		//	}
		//};
		//ticker.once_ms(delay_us / 1000, local::schedule, this);
		schedule_recurrent_function_us([handle]() {
			handle.resume(); return false; }, delay_us,
			[this]() { return ct.is_cancellation_requested(); });
	}
};

ghostl::generator<std::uint64_t> fibonacci_sequence(unsigned n)
{
	if (n == 0)
		co_return;

	if (n > 94)
	{
		// throw std::runtime_error("Too big Fibonacci sequence. Elements would overflow.");
	}

	co_yield 0;

	if (n == 1)
		co_return;

	co_yield 1;

	if (n == 2)
		co_return;

	std::uint64_t a = 0;
	std::uint64_t b = 1;

	for (unsigned i = 2; i <= n; i++)
	{
		std::uint64_t s = a + b;
		co_yield s;
		a = b;
		b = s;
	}
}

int main_gen()
{
	// max 93 before uint64_t overflows
	auto fib = fibonacci_sequence(93);
	for (int j = 1; fib; ++j)
	{
		auto v = fib();
		// PRINTF("fib(%u)=%llu\n", j, v);
	}

	return 0;
}

auto eventAsync(int id, ghostl::cancellation_token ct = {})
{
	ghostl::task_completion_source<std::string> tcs;
	if (id % 2) {
		std::stringstream hello;
		hello << "Hello async (" << id << ") = " << micros() / 1000 << std::ends;
		tcs.set_value(hello.str());
	}
	else {
		struct local {
			static auto task(int id, ghostl::task_completion_source<std::string> tcs, ghostl::cancellation_token ct = {}) -> ghostl::task<> {
				co_await co_delay(id * DELAY, ct);
				std::stringstream hello;
				if (ct.is_cancellation_requested()) {
					hello << "Hello async (" << id << ") cancelled" << std::ends;
					auto str = hello.str();
					tcs.set_value(str);
					// PRINTF("eventAsync: %s\n", str.c_str());
					co_return;
				}
				hello << "Hello async (" << id << ") = " << micros() / 1000 << std::ends;
				tcs.set_value(hello.str());
			}
		};
		auto task = local::task(id, tcs, ct);
		auto runner = ghostl::run_task(std::move(task));
		runner.resume();
	}
	return tcs.token();
}

void main_tcs(ghostl::cancellation_token ct = {});

auto run_tcs(ghostl::cancellation_token ct = {}) -> ghostl::task<>
{
	if (ct.is_cancellation_requested()) { PRINTLN("run_tcs(): cancelled"); co_return; }
	auto preset_tcs = ghostl::task_completion_source<>();
	auto preset_tok = preset_tcs.token();
	preset_tcs.set_value();
	if (ct.is_cancellation_requested()) { PRINTLN("run_tcs(): cancelled"); co_return; }
	co_await preset_tok;
	auto posted_tcs = ghostl::task_completion_source<>();
	schedule_recurrent_function_us(
		[posted_tcs]() { posted_tcs.set_value(); return false; }, DELAY / 2,
		[ct]() { return ct.is_cancellation_requested(); });
	auto posted_tok = posted_tcs.token();
	co_await posted_tok;
	if (ct.is_cancellation_requested()) { PRINTLN("run_tcs(): cancelled"); co_return; }
	co_await co_delay(DELAY / 2, ct);
}

void main_tcs(ghostl::cancellation_token ct)
{
	auto task = run_tcs(ct);
	auto runner = ghostl::run_task(std::move(task));
	runner.continue_with([ct]() { schedule_function([ct]() { if (!ct.is_cancellation_requested()) main_tcs(ct); }); });
	runner.resume();
}

auto make_task(int id, ghostl::cancellation_token ct = {}) -> ghostl::task<std::string>
{
	std::string result = co_await eventAsync(id, ct);
	if (id % 2) co_await co_delay(id * DELAY, ct);
	if (ct.is_cancellation_requested()) { PRINTF("make_task(%u): cancelled\n", id); }
	co_return result;
}

void main_when_all(ghostl::cancellation_token ct = {});

ghostl::task<> make_when_all_tasks(ghostl::cancellation_token outer_ct = {})
{
	auto fwd_ct = ghostl::run_task(outer_ct.cancellation_request());
	ghostl::cancellation_token_source cts;
	auto ct = cts.token();
	fwd_ct.continue_with([cts](bool cancelled) { if (cancelled) cts.cancel(); });
	fwd_ct.resume();

	std::vector<ghostl::task<std::string>> main_tasks;
	constexpr unsigned TASKCNT = 100;
	for (unsigned id = 0; id < TASKCNT; ++id)
	{
		main_tasks.emplace(main_tasks.begin(), make_task(id, ct));
	}
	auto wall = ghostl::when_all<std::string>(std::exchange(main_tasks, {}));
	auto results = co_await wall();
	cts.cancel();
	// PRINTLN("make_when_all_tasks results begin");
	// for (auto r : results) PRINTLN(r.c_str());
	// PRINTLN("make_when_all_tasks results end");
	co_await co_delay(DELAY, ct);
}

void main_when_all(ghostl::cancellation_token ct)
{
	auto task = make_when_all_tasks(ct);
	auto runner = ghostl::run_task(std::move(task));
	runner.continue_with([ct]() { schedule_function([ct]() { if(!ct.is_cancellation_requested()) main_when_all(ct); }); });
	runner.resume();
}

void main_when_any(ghostl::cancellation_token ct = {});

ghostl::task<> make_when_any_tasks(ghostl::cancellation_token outer_ct = {})
{
	auto fwd_ct = ghostl::run_task(outer_ct.cancellation_request());
	ghostl::cancellation_token_source cts;
	auto ct = cts.token();
	fwd_ct.continue_with([cts](bool cancelled) { if (cancelled) cts.cancel(); });
	fwd_ct.resume();

	std::vector<ghostl::task<std::string>> main_tasks;
	constexpr unsigned TASKCNT = 100;
	for (unsigned id = 0; id < TASKCNT; ++id)
	{
		main_tasks.emplace(main_tasks.begin(), make_task(id, ct));
	}
	auto wany = ghostl::when_any<std::string>(std::exchange(main_tasks, {}));
	auto result = co_await wany();
	cts.cancel();
	// PRINTF("make_when_any_tasks result = %s\n", result.c_str());
	co_await co_delay(DELAY, ct);
}

void main_when_any(ghostl::cancellation_token ct)
{
	auto task = make_when_any_tasks(ct);
	auto runner = ghostl::run_task(std::move(task));
	runner.continue_with([ct]() { schedule_function([ct]() { if (!ct.is_cancellation_requested()) main_when_any(ct); }); });
	runner.resume();
}

ghostl::cancellation_token_source cts;
auto start = micros();
bool pending_restart = false;

void setup()
{
#if defined(ESP32) || defined(ESP8266)
	Serial.begin(115200);
	while (!Serial) delay(1);
#endif

	auto ct = cts.token();

	main_gen();
	main_tcs(ct);
	main_when_all(ct);
	main_when_any(ct);
}

void loop()
{
	if (!pending_restart && micros() - start > 1000000)
	{
		PRINTLN("Global cancel and restart after 10 ms");
		pending_restart = true;
		cts.cancel();
		schedule_recurrent_function_us([]() {
			cts = ghostl::cancellation_token_source{};
			start = micros();
			auto ct = cts.token();
			pending_restart = false;
			main_gen();
			main_tcs(ct);
			main_when_all(ct);
			main_when_any(ct);
			return false;
			}, 10000);
	}

#ifndef ESP8266
	auto schedDelay_us = get_scheduled_recurrent_delay_us();
#else
	auto schedDelay_us = std::min(get_scheduled_delay_us(), get_scheduled_recurrent_delay_us());
#endif
	if (schedDelay_us >= 10)
	{
		PRINTF("Delaying scheduler: %lu us (now = %lu s)\n", schedDelay_us, millis() / 1000);
	}

#if defined(ARDUINO)
	delay(schedDelay_us / 1000UL);
#elif defined(__ZEPHYR__)
#ifdef CONFIG_BOARD_NATIVE_POSIX
	// The native POSIX board emulation does not advance system timeout during code execution,
	// which is incompatible with the FastScheduler comparing system time to target times.
	k_sleep(K_USEC(schedDelay_us ? schedDelay_us : 1));
#else // CONFIG_BOARD_NATIVE_POSIX
	k_sleep(K_USEC(schedDelay_us));
#endif // CONFIG_BOARD_NATIVE_POSIX
#else //__ZEPHYR__
	std::this_thread::sleep_for(schedDelay_us < 4000000 ? std::chrono::microseconds(schedDelay_us) : std::chrono::microseconds(4000000));
#endif //__ZEPHYR__

#ifndef ESP8266
	run_scheduled_functions();
#endif
}

#ifdef __ZEPHYR__
void cpp_test_fn(void *arg1, void *arg2, void *arg3)
{
        (void)arg1;
        (void)arg2;
        (void)arg3;
        setup();
        while (1) {
                loop();
        }
}
K_THREAD_DEFINE(cpp_test, 3072, cpp_test_fn, NULL, NULL, NULL, 10, 0, 0);
#elif !defined(ARDUINO)
int main(int argc, char* argv[])
{
	(void)argc;
	(void)argv;
	setup();
	for (;;) loop();
	return 0;
}
#endif

