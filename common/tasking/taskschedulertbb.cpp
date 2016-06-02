// ======================================================================== //
// Copyright 2009-2016 Intel Corporation                                    //
//                                                                          //
// Licensed under the Apache License, Version 2.0 (the "License");          //
// you may not use this file except in compliance with the License.         //
// You may obtain a copy of the License at                                  //
//                                                                          //
//     http://www.apache.org/licenses/LICENSE-2.0                           //
//                                                                          //
// Unless required by applicable law or agreed to in writing, software      //
// distributed under the License is distributed on an "AS IS" BASIS,        //
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. //
// See the License for the specific language governing permissions and      //
// limitations under the License.                                           //
// ======================================================================== //

#include "taskschedulertbb.h"

namespace embree
{
  bool g_tbb_threads_initialized = false;
  tbb::task_scheduler_init g_tbb_threads(tbb::task_scheduler_init::deferred);
  
  class TBBAffinity: public tbb::task_scheduler_observer
  {
    tbb::atomic<int> threadCount;
    
    void on_scheduler_entry( bool ) {
      ++threadCount;
      setAffinity(TaskScheduler::threadIndex()); // FIXME: use threadCount?
    }
    
    void on_scheduler_exit( bool ) { 
      --threadCount; 
    }
  public:
    
    TBBAffinity() { threadCount = 0; }
    
    int  get_concurrency()      { return threadCount; }
    void set_concurrency(int i) { threadCount = i; }
    
  } tbb_affinity;
  
  size_t TaskScheduler::g_numThreads = 0;
  
  void TaskScheduler::create(size_t numThreads, bool set_affinity)
  {
    /* first terminate threads in case we configured them */
    if (g_tbb_threads_initialized) {
      g_tbb_threads.terminate();
      g_tbb_threads_initialized = false;
    }
    
    /* only set affinity if requested by the user */
    if (set_affinity) {
      tbb_affinity.set_concurrency(0);
      tbb_affinity.observe(true); 
    }
    
    /* now either keep default settings or configure number of threads */
    if (numThreads == 0) 
    {
      g_tbb_threads_initialized = false;
      g_numThreads = tbb::task_scheduler_init::default_num_threads();
    } else {
      g_tbb_threads_initialized = true;
      g_tbb_threads.initialize(int(numThreads));
      g_numThreads = numThreads;
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
