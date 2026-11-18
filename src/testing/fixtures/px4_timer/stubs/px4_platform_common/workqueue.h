// Test-only clock, wakeup, logging, and OS shims; timer code remains upstream.
#pragma once
#include <cstdint>
using worker_t = void (*)(void *);
struct work_s {
  worker_t worker;
  void *arg;
  uint64_t deadline;
};
struct wqueue_s {};
