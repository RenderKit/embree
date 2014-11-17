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

#include "common/default.h"

namespace embree
{
  template<typename Ty>
    struct range 
    {
      range(const Ty& begin) 
      : _begin(begin), _end(begin+1) {}

      range(const Ty& begin, const Ty& end) 
      : _begin(begin), _end(end) {}

      __forceinline Ty begin() const {
	return _begin;
      }

      __forceinline Ty end() const {
	return _end;
      }

      Ty _begin, _end;
    };
  
  template<typename Index, typename Func>
    class ParallelForTask
  {
  public:
    ParallelForTask (const Index first, const Index last, const Index step, const Func& f)
      : first(first), last(last), step(step), f(f)
    {
      LockStepTaskScheduler::instance()->dispatchTaskSet(task_function,this,(last-first+step-1)/step);
    }
      
    static void task_function(void* data, const size_t threadIndex, const size_t threadCount, const size_t taskIndex, const size_t taskCount) 
    {
      ParallelForTask* This = (ParallelForTask*)data;
      const size_t i0 = This->first+taskIndex*This->step;
      const size_t i1 = min(i0+This->step,This->last);
      This->f(range<Index>(i0,i1));
    }
    
    private:
      const Index first;
      const Index last; 
      const Index step;
      const Func& f;
  };

  template<typename Index, typename Func>
    __forceinline void parallel_for( const Index first, const Index last, const Func& f)
  {
    ParallelForTask<Index,Func>(first,last,1,f);
  }

  template<typename Index, typename Func>
    __forceinline void parallel_for( const Index first, const Index last, const Index step, const Func& f)
  {
    ParallelForTask<Index,Func>(first,last,step,f);
  }
}
