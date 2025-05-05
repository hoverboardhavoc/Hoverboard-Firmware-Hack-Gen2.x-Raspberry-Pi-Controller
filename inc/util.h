#ifndef UTIL_H
#define UTIL_H

#include <stdio.h>

// General interpolation function
static int16_t interpolate(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max)
{
  return (int16_t)((x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min);
}

#endif // UTIL_H
