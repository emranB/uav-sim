// Test-only clock, wakeup, logging, and OS shims; timer code remains upstream.
#pragma once
#include <cstdint>
#include <ctime>
int px4_clock_gettime(clockid_t, timespec *);
int system_clock_gettime(clockid_t, timespec *);
