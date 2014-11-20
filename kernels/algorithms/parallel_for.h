// ======================================================================== //
// Copyright 2009-2014 Intel Corporation                                    //
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

#include "range.h"

namespace embree
{
  template<typename Index, typename Func>
    __forceinline void sequential_for( const Index first, const Index last, const Func& func) 
  {
    func(range<Index>(first,last));
  }

  template<typename Index, typename Func>
    __forceinline void sequential_for( const Index first, const Index last, const Index minStepSize, const Func& func)
  {
    func(range<Index>(first,last));
  }

  template<typename Index, typename Func>
    class ParallelForTask
  {
  public:
    __forceinline ParallelForTask (const Index first, const Index last, const Index minStepSize, const Func& func)
      : first(first), last(last), minStepSize(minStepSize), func(func)
    {
      const size_t blocks  = (last-first+minStepSize-1)/minStepSize;
      if (blocks == 1) {
        func(range<Index>(first,last));
      }
      else
      {
        LockStepTaskScheduler* scheduler = LockStepTaskScheduler::instance();
        const size_t threads = scheduler->getNumThreads();
        const size_t N = min(threads,blocks);
        scheduler->dispatchTaskSet(task_for,this,N);
      }
    }

    __forceinline void _for(const size_t taskIndex, const size_t taskCount) 
    {
      const size_t k0 = first+(taskIndex+0)*(last-first)/taskCount;
      const size_t k1 = first+(taskIndex+1)*(last-first)/taskCount;
      func(range<Index>(k0,k1));
    }
      
    static void task_for(void* data, const size_t threadIndex, const size_t threadCount, const size_t taskIndex, const size_t taskCount) {
      ((ParallelForTask*)data)->_for(taskIndex,taskCount);
    }
    
    private:
      const Index first;
      const Index last; 
      const Index minStepSize;
      const Func& func;
  };

  template<typename Index, typename Func>
    __forceinline void parallel_for( const Index first, const Index last, const Func& func)
  {
    ParallelForTask<Index,Func>(first,last,1,func);
  }

  template<typename Index, typename Func>
    __forceinline void parallel_for( const Index first, const Index last, const Index minStepSize, const Func& func)
  {
    ParallelForTask<Index,Func>(first,last,minStepSize,func);
  }
}
