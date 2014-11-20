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

#include "parallel_for_for.h"

namespace embree
{
  /*template<typename ArrayArray, typename Value, typename Func, typename Reduction>
    __forceinline void sequential_for_for_prefix_sum( ParallelForForPrefixSumState<ArrayArray,Value>& state, const Func& func, const Reduction& reduction)
  {
    for (size_t i=0; i<state.array2.size(); i++) {
      auto arr = state.array2[i];
      
      for (size_t j=0; j<arr->size(); j++) {
        
      }
    }
    }*/

  template<typename ArrayArray, typename Value>
    class ParallelForForPrefixSumState : public ParallelForForState<ArrayArray>
  {
  public:
    ParallelForForPrefixSumState ( ArrayArray& array2, const size_t minStepSize, const Value& identity ) 
      : ParallelForForState<ArrayArray>(array2,minStepSize), value(identity), _blocks(0), scheduler(LockStepTaskScheduler::instance())
    {
      _blocks  = min((this->K+this->minStepSize-1)/this->minStepSize,scheduler->getNumThreads());
      counts.resize(_blocks);
      sums.resize(_blocks);
    }

    /*size_t size() const {
      return value;
      }*/

  public:
    Value value;
    size_t _blocks;
    std::vector<Value> counts;
    std::vector<Value> sums;
    LockStepTaskScheduler* scheduler;
  };

  template<typename ArrayArray, typename Value, typename Func, typename Reduction>
    class ParallelForForPrefixSumTask
  {
  public:
    
    ParallelForForPrefixSumTask (ParallelForForPrefixSumState<ArrayArray,Value>& state, const Value& identity, const Func& func, const Reduction& reduction) 
      : state(state), func(func), reduction(reduction)
    {
      const size_t blocks = state._blocks;
      state.scheduler->dispatchTaskSet(task_execute,this,blocks);
      
      /* calculate prefix sum */
      Value sum=0;
      for (size_t i=0; i<blocks; i++)
      {
        const Value c = state.counts[i];
        state.sums[i] = sum;
        sum=reduction(sum,c);
      }
      state.value = sum;
    }
    
    void execute(const size_t threadIndex, const size_t threadCount, const size_t taskIndex, const size_t taskCount) 
    {
      /* calculate range */
      const size_t k0 = (taskIndex+0)*state.K/taskCount;
      const size_t k1 = (taskIndex+1)*state.K/taskCount;
      size_t i0, j0; state.start_indices(k0,i0,j0);
      
      /* iterate over arrays */
      size_t k=k0, N=0;
      for (size_t i=i0; k<k1; i++) {
        const size_t r0 = j0, r1 = min(state.sizes[i],r0+k1-k);
        if (r1 > r0) N = reduction(N, func(state.array2[i],range<size_t>(r0,r1),k,reduction(state.sums[taskIndex],N)));
        k+=r1-r0; j0 = 0;
      }
      state.counts[taskIndex] = N;
    }
    
    static void task_execute(void* data, const size_t threadIndex, const size_t threadCount, const size_t taskIndex, const size_t taskCount) {
      ((ParallelForForPrefixSumTask*)data)->execute(threadIndex,threadCount,taskIndex,taskCount);
    }
    
  public:
    ParallelForForPrefixSumState<ArrayArray,Value>& state;
  private:
    const Func& func;
    const Reduction& reduction;
  };
  
  template<typename ArrayArray, typename Value, typename Func, typename Reduction>
    __forceinline Value parallel_for_for_prefix_sum( ParallelForForPrefixSumState<ArrayArray,Value>& state, const Value& identity, const Func& func, const Reduction& reduction)
  {
    ParallelForForPrefixSumTask<ArrayArray,Value,Func,Reduction> task(state,identity,func,reduction);
    return task.state.value;
  }
}
