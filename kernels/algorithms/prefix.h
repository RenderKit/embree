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
  template<typename Ty, typename Op>
  class ParallelPrefixOp
  {
    static const size_t MAX_THREADS = 32;

  public:
    ParallelPrefixOp () {}

    class Task
    {
    public:
      Task (ParallelPrefixOp* parent, const Ty* src, Ty* dst, const size_t N, const Op op, const Ty id)
	: parent(parent), src(src), dst(dst), N(N), op(op), id(id)
      {
	/* perform single threaded prefix operation for small N */
	if (N < 3000000) {
	  for (size_t i=0, sum=0; i<N; sum+=src[i++])
	    dst[i] = sum;
	}

	/* perform parallel prefix operation for large N */
	else 
	{
	  LockStepTaskScheduler* scheduler = LockStepTaskScheduler::instance();
	  const size_t numThreads = min(scheduler->getNumThreads(),MAX_THREADS);
	  
	  /* first calculate range for each block */
	  scheduler->dispatchTask(task_count,this,0,numThreads);
	  
	  /* now calculate prefix_op for each block */
	  scheduler->dispatchTask(task_prefix_op,this,0,numThreads);
	}
      }
      
      void count(const size_t threadIndex, const size_t threadCount)
      {
	const size_t start = (threadIndex+0)*N/threadCount;
	const size_t end   = (threadIndex+1)*N/threadCount;
	
	Ty count = id;
	for (size_t i=start; i<end; i++) 
	  count = op(count,src[i]);
	
	parent->state[threadIndex] = count;
      }

      void prefix_op(const size_t threadIndex, const size_t threadCount)
      {
	const size_t start = (threadIndex+0)*N/threadCount;
	const size_t end   = (threadIndex+1)*N/threadCount;
		
	/* calculate start sum for block */
	Ty count = id;
	for (size_t i=0; i<threadIndex; i++)
	  count += parent->state[i];
	
	/* calculate prefix sums for the block */
	for (size_t i=start; i<end; i++) 
	{
	  const Ty v = src[i];
	  dst[i] = count;
	  count = op(count,v);
	}
      }
      
      static void task_count (void* data, const size_t threadIndex, const size_t threadCount) { 
	((Task*)data)->count(threadIndex,threadCount);                          
      }
      
      static void task_prefix_op (void* data, const size_t threadIndex, const size_t threadCount) { 
	((Task*)data)->prefix_op(threadIndex,threadCount);                          
      }

    private:
      ParallelPrefixOp* const parent;
      const Ty* const src;
      Ty* const dst;
      const size_t N;
      const Op op;
      const Ty id;
    };

    void operator() (const Ty* src, Ty* dst, const size_t N, const Op op, const Ty id) {
      Task(this,src,dst,N,op,id);
    }

  private:
    Ty state[MAX_THREADS];
  };

  template<typename Ty>
  struct my_add { // FIXME: use lambda expressions
    Ty operator()(const Ty& a, const Ty& b) const { return a+b; }
  };

  __forceinline void parallel_prefix_sum(const uint32* src, uint32* dst, size_t N) {
    ParallelPrefixOp<uint32,my_add<uint32> > op; op(src,dst,N,my_add<uint32>(),0);
  }
}
