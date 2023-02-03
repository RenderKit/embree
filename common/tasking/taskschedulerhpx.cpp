// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "taskschedulerhpx.h"

#include <hpx/hpx_start.hpp>
#include <hpx/hpx_suspend.hpp>

#include <hpx/local/future.hpp>
#include <hpx/local/init.hpp>

namespace embree
{
  static bool g_hpx_threads_initialized = false;
    
  void TaskScheduler::create(size_t numThreads, bool set_affinity, bool start_threads)
  {
    g_hpx_threads_initialized = true;
    hpx::start( nullptr, 0, nullptr);
    numThreads = threadCount();
  }
  
  void TaskScheduler::destroy()
  {
    if (g_hpx_threads_initialized) {
      hpx::apply([]() { hpx::finalize(); });
      hpx::stop();
    }
  }
}
