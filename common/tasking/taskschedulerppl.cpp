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

#include "taskschedulerppl.h"

namespace embree
{
  bool g_ppl_threads_initialized = false;
    
  size_t TaskScheduler::g_numThreads = 0;
  
  void TaskScheduler::create(size_t numThreads, bool set_affinity, bool start_threads)
  {
    /* first terminate threads in case we configured them */
    if (g_ppl_threads_initialized) {
      g_ppl_threads_initialized = false;
    }
    
    /* now either keep default settings or configure number of threads */
    if (numThreads == 0)
    {
      g_ppl_threads_initialized = false;
      g_numThreads = threadCount();
    }
    else {
      g_ppl_threads_initialized = true;
      g_numThreads = numThreads;
      try {
        concurrency::Scheduler::SetDefaultSchedulerPolicy(concurrency::SchedulerPolicy(2, concurrency::MinConcurrency, g_numThreads, concurrency::MaxConcurrency, g_numThreads));
      }
      catch(concurrency::default_scheduler_exists &)
      { }
    }
  }
  //
  
  void TaskScheduler::destroy()
  {
    if (g_ppl_threads_initialized)
    {
      g_ppl_threads_initialized = false;
    }
  }
}
