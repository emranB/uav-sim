// Test-only clock, wakeup, logging, and OS shims; timer code remains upstream.
#pragma once
#include <semaphore.h>
using px4_sem_t = sem_t;
inline int px4_sem_init(px4_sem_t *s, int p, unsigned v) {
  return sem_init(s, p, v);
}
inline int px4_sem_wait(px4_sem_t *s) { return sem_wait(s); }
inline int px4_sem_post(px4_sem_t *s) { return sem_post(s); }
