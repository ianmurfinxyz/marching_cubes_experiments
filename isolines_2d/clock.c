#include <assert.h>
#include "clock.h"

static void
delta(struct timespec *start, struct timespec *end, struct timespec *delta)
{
  if((end->tv_nsec - start->tv_nsec) < 0)
  {
    delta->tv_sec = end->tv_sec - start->tv_sec - 1;
    delta->tv_nsec = (1000000000 + (end->tv_nsec - start->tv_nsec));
  }
  else
  {
    delta->tv_sec = end->tv_sec - start->tv_sec;
    delta->tv_nsec = end->tv_nsec - start->tv_nsec;
  }
}

unsigned long
clock_resolution_ns(struct clock *c)
{
  struct timespec res;
  int r = clock_getres(c->clock_id, &res);
  assert(!r);
  return (unsigned long)res.tv_nsec; /* assumes resolution is much less than 1 second */
}

void
clock_init(struct clock *c, clockid_t clock_id)
{
  c->clock_id = clock_id;
  clock_reset(c);
}

void
clock_reset(struct clock *c)
{
  int r = clock_gettime(c->clock_id, &c->origin);
  assert(!r);
}

const struct timespec* const
clock_time_ns(struct clock *c)
{
  int r = clock_gettime(c->clock_id, &c->current);
  assert(!r);
  delta(&c->origin, &c->current, &c->current);
  return &c->current;
}

unsigned long
clock_time_ms(struct clock *c)
{
  static double factor_ns_to_ms = 1.0 / 1000000.0;
  clock_time_ns(c);
  return (c->current.tv_sec * 1000) + (unsigned long)(c->current.tv_nsec * factor_ns_to_ms);
}

double
clock_time_s(struct clock *c)
{
  static double factor_ns_to_s = 1.0 / 1000000000.0;
  clock_time_ns(c);
  return (double)c->current.tv_sec + ((double)c->current.tv_nsec * factor_ns_to_s);
}

