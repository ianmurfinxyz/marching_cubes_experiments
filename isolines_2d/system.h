#ifndef _SYSTEM_H_
#define _SYSTEM_H_

#include <stdlib.h>
#include <stdio.h>

#define LIKELY(cond) __builtin_expect((cond), 1)
#define UNLIKELY(cond) __builtin_expect((cond), 0)

static inline void *
xmalloc(size_t size)
{
  void *mem = malloc(size);
  if(UNLIKELY(mem == 0))
  {
    fprintf(stderr, "fatal: out of memory\n");
    exit(EXIT_FAILURE);
  }
  return mem;
}

#endif
