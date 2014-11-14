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

#include "sys/platform.h"
#include "sys/sysinfo.h"
#include "sys/taskscheduler.h"
#include "math/math.h"
#include <algorithm>

namespace embree
{
  class ParallelRadixSort
  {
  public:

    static const size_t MAX_THREADS = 32;
    static const size_t BITS = 11;
    static const size_t BUCKETS = (1 << BITS);
    typedef unsigned int TyRadixCount[MAX_THREADS][BUCKETS];
    
    template<typename Ty, typename Key>
      class Task
    {
      template<typename Ty>
	static bool compare(const Ty& v0, const Ty& v1) {
	return (Key)v0 < (Key)v1;
      }

    public:
      Task (ParallelRadixSort* parent, Ty* src, Ty* tmp, Ty* dst, const size_t N)
	: parent(parent), src(src), tmp(tmp), dst(dst), N(N) 
      {
	/* perform single threaded sort for small N */
	if (N<3000) 
	{
	  /* copy data to destination array */
	  for (size_t i=0; i<N; i++)
	    dst[i] = src[i];
	  
	  /* do inplace sort inside destination array */
	  std::sort(dst,dst+N,compare<Ty>);
	}
	
	/* perform parallel sort for large N */
	else {
	  LockStepTaskScheduler* scheduler = LockStepTaskScheduler::instance();
	  const size_t numThreads = min(scheduler->getNumThreads(),MAX_THREADS);
	  parent->barrier.init(numThreads);
	  scheduler->dispatchTask(task_radixsort,this,0,numThreads);
	}
      }
      
    private:
      
      void radixIteration(const Key shift, 
			  const Ty* __restrict src, Ty* __restrict dst, 
			  const size_t startID, const size_t endID, 
			  const size_t threadIndex, const size_t threadCount)
      {
	/* mask to extract some number of bits */
	const Key mask = BUCKETS-1;
        
	/* count how many items go into the buckets */
	for (size_t i=0; i<BUCKETS; i++)
	  parent->radixCount[threadIndex][i] = 0;
	
	for (size_t i=startID; i<endID; i++) {
	  const Key index = ((Key)src[i] >> shift) & mask;
	  parent->radixCount[threadIndex][index]++;
	}
	parent->barrier.wait(threadIndex,threadCount);
	
	/* calculate total number of items for each bucket */
	__aligned(64) size_t total[BUCKETS];
	for (size_t i=0; i<BUCKETS; i++)
	  total[i] = 0;
	
	for (size_t i=0; i<threadCount; i++)
	  for (size_t j=0; j<BUCKETS; j++)
	    total[j] += parent->radixCount[i][j];
	
	/* calculate start offset of each bucket */
	__aligned(64) size_t offset[BUCKETS];
	offset[0] = 0;
	for (size_t i=1; i<BUCKETS; i++)    
	  offset[i] = offset[i-1] + total[i-1];
	
	/* calculate start offset of each bucket for this thread */
	for (size_t i=0; i<threadIndex; i++)
	  for (size_t j=0; j<BUCKETS; j++)
	    offset[j] += parent->radixCount[i][j];
	
	/* copy items into their buckets */
	for (size_t i=startID; i<endID; i++) {
	  const Ty elt = src[i];
	  const Key index = ((Key)src[i] >> shift) & mask;
	  dst[offset[index]++] = elt;
	}
	//if (b < 2) 
	parent->barrier.wait(threadIndex,threadCount); // FIXME: optimize
      }
      
      void radixsort(const size_t threadIndex, const size_t numThreads)
      {
	const size_t startID = (threadIndex+0)*N/numThreads;
	const size_t endID   = (threadIndex+1)*N/numThreads;

	if (sizeof(Key) == sizeof(uint32)) {
	  radixIteration(0*BITS,src,dst,startID,endID,threadIndex,numThreads);
	  radixIteration(1*BITS,dst,tmp,startID,endID,threadIndex,numThreads);
	  radixIteration(2*BITS,tmp,dst,startID,endID,threadIndex,numThreads);
	}
	else if (sizeof(Key) == sizeof(uint64))
	{
	  Ty* const tmp = this->tmp;
	  Ty* const dst = this->dst;
	  radixIteration(0,src,dst,startID,endID,threadIndex,numThreads);
	  radixIteration(BITS,dst,tmp,startID,endID,threadIndex,numThreads);
	  for (uint64 shift=2*BITS; shift<64; shift+=BITS) {
	    radixIteration(shift,tmp,dst,startID,endID,threadIndex,numThreads);
	    std::swap(dst,tmp);
	  }
	}
      }
      
      static void task_radixsort (void* data, const size_t threadIndex, const size_t threadCount) { 
	((Task*)data)->radixsort(threadIndex,threadCount);                          
      }

    private:
      ParallelRadixSort* const parent;
      Ty* const src;
      Ty* const tmp;
      Ty* const dst;
      const size_t N;
    };
    
  private:
    TyRadixCount radixCount;
    LinearBarrierActive barrier;
  };

  /*! shared state for parallel radix sort */
  extern ParallelRadixSort shared_radix_sort_state;

  /*! parallel radix sort */
  template<typename Key>
  struct ParallelRadixSortT
  {
    ParallelRadixSortT (ParallelRadixSort& state) 
      : state(state) {} 

    template<typename Ty>
    void operator() (Ty* src, Ty* tmp, Ty* dst, const size_t N) {
      ParallelRadixSort::Task<Ty,Key>(&state,src,tmp,dst,N);
    }

    ParallelRadixSort& state;
  };

  template<typename Ty, typename Key = Ty>
    void radix_sort(Ty* src, Ty* tmp, Ty* dst, const size_t N)
  {
    ParallelRadixSortT<Key> sort(shared_radix_sort_state);
    sort(src,tmp,dst,N);
  }

  extern ParallelRadixSortT<uint32> radix_sort_u32;
  extern ParallelRadixSortT<uint64> radix_sort_u64;
}
