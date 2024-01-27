// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#ifndef __APPLE__
#if !defined(__forceinline)
#define __forceinline __attribute__((always_inline))
#endif
#elif defined(__APPLE__) && (__arm__)
#if !defined(__forceinline)
#define __forceinline __attribute__((always_inline))
#endif
#endif

#include <hpx/thread.hpp>

namespace embree
{
  struct TaskScheduler
  {

#ifdef __APPLE__
__attribute__((visibility("default")))
#endif
    /*! initializes the task scheduler */
    static void create(size_t numThreads, bool set_affinity, bool start_threads);

#ifdef __APPLE__
__attribute__((visibility("default")))
#endif
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
