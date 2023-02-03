// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#ifndef __APPLE__
#define __forceinline __attribute__((always_inline))
#endif

#include <hpx/local/thread.hpp>

namespace embree
{
  struct TaskScheduler
  {
    /*! initializes the task scheduler */
    static void create(size_t numThreads, bool set_affinity, bool start_threads);

    /*! destroys the task scheduler again */
    static void destroy();

    /* returns the ID of the current thread */
    static size_t threadID() {
      return hpx::get_worker_thread_num();
    }

    /* returns the index (0..threadCount-1) of the current thread */
    /* FIXME: threadIndex is NOT supported by PPL! */
    static size_t threadIndex() {
      return 0;
    }

    /* returns the total number of threads */
    static size_t threadCount() {
      return static_cast<std::size_t>(hpx::get_worker_thread_num());
    }
  };
};
