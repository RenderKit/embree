// Copyright 2009-2020 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "taskschedulertbb.h"

namespace embree
{
  static bool g_tbb_threads_initialized = false;
  static tbb::task_scheduler_init g_tbb_threads(tbb::task_scheduler_init::deferred);
  
  class TBBAffinity: public tbb::task_scheduler_observer
  {
  public:
    
    void on_scheduler_entry( bool ) {
      setAffinity(TaskScheduler::threadIndex()); 
    }
    
  } tbb_affinity;
  
  void TaskScheduler::create(size_t numThreads, bool set_affinity, bool start_threads)
  {
    assert(numThreads);

    /* first terminate threads in case we configured them */
    if (g_tbb_threads_initialized) {
      g_tbb_threads.terminate();
      g_tbb_threads_initialized = false;
    }
    
    /* only set affinity if requested by the user */
#if TBB_INTERFACE_VERSION >= 9000 // affinity not properly supported by older TBB versions
    if (set_affinity)
      tbb_affinity.observe(true);
#endif 
    
    /* now either keep default settings or configure number of threads */
    if (numThreads == std::numeric_limits<size_t>::max()) {
      g_tbb_threads_initialized = false;
      numThreads = threadCount();
      //numThreads = tbb::task_scheduler_init::default_num_threads();
    } else {
      g_tbb_threads_initialized = true;
      const size_t max_concurrency = threadCount();
      if (numThreads > max_concurrency) numThreads = max_concurrency;
      g_tbb_threads.initialize(int(numThreads));
    }

    /* start worker threads */
    if (start_threads) 
    {
      BarrierSys barrier(numThreads);
      tbb::parallel_for(size_t(0), size_t(numThreads), size_t(1), [&] ( size_t i ) {
          barrier.wait();
        });
    }
  }
  
  void TaskScheduler::destroy()
  {
    if (g_tbb_threads_initialized) {
      g_tbb_threads.terminate();
      g_tbb_threads_initialized = false;
    }
  }
}
