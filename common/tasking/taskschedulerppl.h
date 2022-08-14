// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "common/sys/platform.h"
#include "common/sys/alloc.h"
#include "common/sys/barrier.h"
#include "common/sys/thread.h"
#include "common/sys/mutex.h"
#include "common/sys/condition.h"
#include "common/sys/ref.h"

#if !defined(__WIN32__)
#error PPL tasking system only available under windows
#endif

#include <ppl.h>

namespace embree
{
  struct TaskScheduler
  {
    /*! initializes the task scheduler */
    static void create(size_t numThreads, bool set_affinity, bool start_threads);

    /*! destroys the task scheduler again */
    static void destroy();

    /* returns the ID of the current thread */
    static __forceinline size_t threadID() {
      return GetCurrentThreadId();
    }

    /* returns the index (0..threadCount-1) of the current thread */
    /* FIXME: threadIndex is NOT supported by PPL! */
    static __forceinline size_t threadIndex() {
      return 0;
    }

    /* returns the total number of threads */
    static __forceinline size_t threadCount() {
      return GetMaximumProcessorCount(ALL_PROCESSOR_GROUPS) + 1;
    }
  };
};
