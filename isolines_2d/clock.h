#ifndef _CLOCK_H_
#define _CLOCK_H_

#include <time.h>

/**
 * represents a continuous timeline; the reading on the clock gives the time
 * since the clock was reset. Thus the clock is more like a stopwatch that
 * can only be reset.
 */
struct clock
{
  clockid_t clock_id;      /* POSIX clock to use for time measurements */
  struct timespec origin;  /* origin of the clock's timeline */
  struct timespec current; /* current time on the clock */
};

/**
 * clock_init - sets the clock timelines origin to the current time on the clockid_t.
 */
void
clock_init(struct clock *c, clockid_t clock_id);

/**
 * clock_resultion_ns - access resolution of clock in nanoseconds. Limited by underlying clockid_t used.
 *
 * note - the returned value determines the real precision of the clock, e.g. if the resolution is 4ms then
 *   the clock has a real precision of 4ms, even if the clock returns time in nanoseconds (the nanosecond time 
 *   values returned will be multiples of 4ms in this example case).
 *
 * note - the accuracy of the clock is dependent on the kernel and is unkown.
 */
unsigned long
clock_resolution_ns(struct clock *c);

/**
 * clock_reset - set the clock origin to the current time. Future time readings will be w.r.t to
 *   the new origin.
 */
void
clock_reset(struct clock *c);

/**
 * clock_time_ns - access the current time on the clock w.r.t the origin in nanoseconds.
 *
 * note - returns with a timespec array for the benifit of 32-bit systems; 32-bit unsigned integer would
 *   overflow after just (pow(2, 32) / (pow(10, 9) * 60)) = 4.29 seconds, or ~2.1 seconds for a signed 
 *   integer (which timespec uses).
 */
const struct timespec* const
clock_time_ns(struct clock *c);

/**
 * clock_time_ms - get the time since the last reset in milliseconds.
 *
 * note - on a 32-bit system where unsigned long is 32-bits, this will overflow after:
 *          pow(2, 32) / (pow(10, 3) * 60 * 60) = ~1193 hours == 49.7 days
 */
unsigned long
clock_time_ms(struct clock *c);

/**
 * clock_time_s - get the time since the last reset in seconds.
 *
 * note - TODO: precision of double as seconds increase? How precise is the double after say 49.7 days?
 */
double
clock_time_s(struct clock *c);

#endif
