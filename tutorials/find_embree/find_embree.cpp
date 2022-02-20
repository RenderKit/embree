// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include <embree3/rtcore.h>
RTC_NAMESPACE_USE

#include <xmmintrin.h>
//#include <pmmintrin.h> // use this to get _MM_SET_DENORMALS_ZERO_MODE when compiling for SSE3 or higher

#if !defined(_MM_SET_DENORMALS_ZERO_MODE)
#define _MM_DENORMALS_ZERO_ON   (0x0040)
#define _MM_DENORMALS_ZERO_OFF  (0x0000)
#define _MM_DENORMALS_ZERO_MASK (0x0040)
#define _MM_SET_DENORMALS_ZERO_MODE(x) (_mm_setcsr((_mm_getcsr() & ~_MM_DENORMALS_ZERO_MASK) | (x)))
#endif

int main(int argc, char* argv[])
{
  /* for best performance set FTZ and DAZ flags in MXCSR control and status register */
  _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
  _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);

  /* create new Embree device */
  RTCDevice device = rtcNewDevice("verbose=1");

  /* delete device again */
  rtcReleaseDevice(device);
  
  return 0;
}
