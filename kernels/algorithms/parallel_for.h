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

#pragma once

#include "../common/default.h"
#include "range.h"

#include "../common/tasking/taskscheduler.h"

namespace embree
{
  /* simple parallel_for without range optimization (similar to a task set) */
  template<typename Index, typename Func>
    __forceinline void parallel_for( const Index N, const Func& func)
  {
#if defined(TASKING_INTERNAL)
    if (N) {
      TaskScheduler::spawn(Index(0),N,Index(1),[&] (const range<Index>& r) {
          assert(r.size() == 1);
          func(r.begin());
        });
      if (!TaskScheduler::wait())
        throw std::runtime_error("task cancelled");
    }

#else 
    tbb::parallel_for(Index(0),N,Index(1),[&](Index i) { 
	func(i);
      });
    if (tbb::task::self().is_cancelled())
      throw std::runtime_error("task cancelled");
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
#if defined(TASKING_INTERNAL)
    TaskScheduler::spawn(first,last,minStepSize,func);
    if (!TaskScheduler::wait())
        throw std::runtime_error("task cancelled");
#else
    tbb::parallel_for(tbb::blocked_range<Index>(first,last,minStepSize),[&](const tbb::blocked_range<Index>& r) { 
      func(range<Index>(r.begin(),r.end()));
      });
    if (tbb::task::self().is_cancelled())
      throw std::runtime_error("task cancelled");
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
