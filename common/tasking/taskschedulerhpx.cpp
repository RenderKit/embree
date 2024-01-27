// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "taskschedulerhpx.h"

#include <hpx/hpx_start.hpp>
#include <hpx/hpx_suspend.hpp>

#include <hpx/future.hpp>
#include <hpx/init.hpp>

namespace embree
{
  static bool g_hpx_threads_initialized = false;
    
  void TaskScheduler::create(size_t numThreads, bool set_affinity, bool start_threads)
  {
    g_hpx_threads_initialized = true;
    std::string count = std::to_string(numThreads);
    std::string thread_arg = "--hpx:threads=" + count;
    hpx::init_params params;
    params.cfg = { thread_arg };
    hpx::start(nullptr, 0, nullptr, params);
    numThreads = threadCount();
  }
  
  void TaskScheduler::destroy()
  {
    if (g_hpx_threads_initialized) {
      hpx::post([]() { hpx::finalize(); });
      hpx::stop();
    }
  }
}
