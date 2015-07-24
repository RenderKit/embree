// ======================================================================== //
// Copyright 2009-2015 Intel Corporation                                    //
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

#pragma once

#include "../common/default.h"
#include "range.h"

#if defined(TASKING_LOCKSTEP)
#include "../../common/tasking/taskscheduler_mic.h"
#endif // if defined(TASKING_TBB_INTERNAL) // FIXME
#include "../../common/tasking/taskscheduler_tbb.h"

namespace embree
{
#if defined(TASKING_LOCKSTEP)
  template<typename Index, typename Func>
    class ParallelForTask
  {
  public:
    __forceinline ParallelForTask (const Index taskCount, const Func& func)
      : func(func)
    {
#if 0
      for (size_t taskIndex=0; taskIndex<taskCount; taskIndex++)
	func(taskIndex);
#else
      if (taskCount == 0) return;
      else if (taskCount == 1) func(0);
      else LockStepTaskScheduler::instance()->dispatchTaskSet(task_set,this,taskCount);
#endif
    }

    static void task_set(void* data, const size_t threadIndex, const size_t threadCount, const size_t taskIndex, const size_t taskCount) {
      ((ParallelForTask*)data)->func(taskIndex);
    }
    
    private:
      const Func& func;
  };
#endif

  /* simple parallel_for without range optimization (similar to a task set) */
  template<typename Index, typename Func>
    __forceinline void parallel_for( const Index N, const Func& func)
  {
#if defined(TASKING_LOCKSTEP)
    ParallelForTask<Index,Func>(N,func);

#elif defined(TASKING_TBB_INTERNAL)
    if (N) {
      TaskSchedulerTBB::spawn(Index(0),N,Index(1),[&] (const range<Index>& r) {
          assert(r.size() == 1);
          func(r.begin());
        });
      TaskSchedulerTBB::wait();
    }

#else 
    tbb::parallel_for(Index(0),N,Index(1),[&](Index i) { 
	func(i);
      });
    if (tbb::task::self().is_cancelled())
      throw std::runtime_error("task group cancelled");
#endif
  }

  /* sequential for with range optimization */
  template<typename Index, typename Func>
    __forceinline void sequential_for( const Index first, const Index last, const Func& func) 
  {
    assert(first <= last);
    func(range<Index>(first,last));
  }
 
  /* sequential for with range optimization and minimal granularity per thread */
  template<typename Index, typename Func>
    __forceinline void sequential_for( const Index first, const Index last, const Index minStepSize, const Func& func)
  {
    assert(first <= last);
    func(range<Index>(first,last));
  }

  /* parallel for with range optimization */
  template<typename Index, typename Func>
    __forceinline void parallel_for( const Index first, const Index last, const Index minStepSize, const Func& func)
  {
    assert(first <= last);
#if defined(TASKING_LOCKSTEP)
    size_t taskCount = (last-first+minStepSize-1)/minStepSize;
    if (taskCount > 1) taskCount = min(taskCount,LockStepTaskScheduler::instance()->getNumThreads());

    parallel_for(taskCount, [&](const size_t taskIndex) {
        const size_t k0 = first+(taskIndex+0)*(last-first)/taskCount;
        const size_t k1 = first+(taskIndex+1)*(last-first)/taskCount;
        func(range<Index>(k0,k1));
      });

#elif defined(TASKING_TBB_INTERNAL)
    TaskSchedulerTBB::spawn(first,last,minStepSize,func);
    TaskSchedulerTBB::wait();

#else
    tbb::parallel_for(tbb::blocked_range<Index>(first,last,minStepSize),[&](const tbb::blocked_range<Index>& r) { 
      func(range<Index>(r.begin(),r.end()));
    });
    if (tbb::task::self().is_cancelled())
      throw std::runtime_error("task group cancelled");
#endif
  }

  /* parallel for with range optimization and minimal granularity per thread */
  template<typename Index, typename Func>
    __forceinline void parallel_for( const Index first, const Index last, const Func& func)
  {
    assert(first <= last);
    parallel_for(first,last,(Index)1,func);
  }
}
