/**
 * Exercise periodic timer requeue behavior against isolated platform stubs.
 */
#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <thread>
#define __EXPORT
#include HRT_SOURCE

static std::atomic<uint64_t> simulated_us{1000000};
static work_s *scheduled_work = nullptr;
int system_clock_gettime(clockid_t, timespec *timestamp) {
  const auto now = simulated_us.load();
  timestamp->tv_sec = now / 1000000;
  timestamp->tv_nsec = (now % 1000000) * 1000;
  return 0;
}
extern "C" int hrt_work_queue(work_s *work, worker_t worker, void *arg,
                              uint32_t delay) {
  work->worker = worker;
  work->arg = arg;
  work->deadline = simulated_us.load() + delay;
  scheduled_work = work;
  return 0;
}
extern "C" void hrt_work_cancel(work_s *work) {
  if (scheduled_work == work) {
    scheduled_work = nullptr;
  }
}
static hrt_call changing_interval{};
static const char *scenario = "slow-interval";
static unsigned reconfigured_callbacks = 0;
static unsigned battery_callbacks = 0;
static unsigned power_callbacks = 0;
static unsigned load_callbacks = 0;
static void count_battery(void *) { ++battery_callbacks; }
static void count_power(void *) { ++power_callbacks; }
static void count_load(void *) { ++load_callbacks; }
static void reconfigure(void *) {
  if (++reconfigured_callbacks != 1) {
    return;
  }
  if (std::strcmp(scenario, "delay") == 0) {
    hrt_call_delay(&changing_interval, 50000);
    return;
  }
  if (std::strcmp(scenario, "normal") == 0) {
    return;
  }
  // Force a production-permitted ordering: the work item changes its timer
  // before the dispatcher's scheduling callback has returned.
  std::thread worker([] {
    if (std::strcmp(scenario, "cancel") == 0) {
      hrt_cancel(&changing_interval);
    } else if (std::strcmp(scenario, "one-shot") == 0) {
      hrt_call_after(&changing_interval, 50000, reconfigure, nullptr);
    } else if (std::strcmp(scenario, "fast-interval") == 0) {
      hrt_call_every(&changing_interval, 0, 1000, reconfigure, nullptr);
    } else if (std::strcmp(scenario, "future-interval") == 0) {
      hrt_call_every(&changing_interval, 50000, 20000, reconfigure, nullptr);
    } else {
      if (std::strcmp(scenario, "cancel-reschedule") == 0) {
        hrt_cancel(&changing_interval);
      }
      hrt_call_every(&changing_interval, 0, 10000000, reconfigure, nullptr);
    }
  });
  worker.join();
}
int main(int argc, char **argv) {
  if (argc == 2) {
    scenario = argv[1];
  }
  hrt_init();
  hrt_call battery{}, power{}, load{};
  hrt_call_every(&battery, 0, 10000, count_battery, nullptr);
  hrt_call_every(&power, 0, 10000, count_power, nullptr);
  hrt_call_every(&load, 0, 500000, count_load, nullptr);
  hrt_call_every(&changing_interval, 0, 20000, reconfigure, nullptr);
  const auto end = simulated_us.load() + 1000000;
  while (simulated_us.load() < end) {
    simulated_us.fetch_add(1000);
    if (scheduled_work && scheduled_work->deadline <= simulated_us.load()) {
      auto *work = scheduled_work;
      scheduled_work = nullptr;
      auto worker = work->worker;
      auto arg = work->arg;
      worker(arg);
    }
  }
  bool interval_valid = reconfigured_callbacks == 1;
  if (std::strcmp(scenario, "normal") == 0 ||
      std::strcmp(scenario, "delay") == 0 ||
      std::strcmp(scenario, "future-interval") == 0) {
    interval_valid =
        reconfigured_callbacks >= 40 && reconfigured_callbacks <= 51;
  } else if (std::strcmp(scenario, "one-shot") == 0) {
    interval_valid = reconfigured_callbacks == 2;
  } else if (std::strcmp(scenario, "fast-interval") == 0) {
    interval_valid = reconfigured_callbacks >= 999;
  }
  const bool passed = battery_callbacks >= 99 && power_callbacks >= 99 &&
                      load_callbacks >= 2 && interval_valid;
  std::printf("{\"battery_callbacks\":%u,\"power_callbacks\":%u,"
              "\"load_callbacks\":%u,\"interval_callbacks\":%u}\n",
              battery_callbacks, power_callbacks, load_callbacks,
              reconfigured_callbacks);
  return passed ? 0 : 1;
}
