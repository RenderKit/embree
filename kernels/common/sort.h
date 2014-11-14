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
#include <algorithm>

namespace embree
{
  class ParallelRadixSortU32
  {
    static const size_t MAX_THREADS = 32;
    static const size_t RADIX_BITS = 11;
    static const size_t RADIX_BUCKETS = (1 << RADIX_BITS);
    static const size_t RADIX_BUCKETS_MASK = (RADIX_BUCKETS-1);
    typedef unsigned int TyRadixCount[MAX_THREADS][RADIX_BUCKETS];
    
  private:
    
    template<typename Ty>
      class ParallelTask
    {
    public:
      ParallelTask (ParallelRadixSortU32* parent, Ty* src, Ty* tmp, Ty* dst, const size_t N)
	: parent(parent), src(src), tmp(tmp), dst(dst), N(N) 
      {
	LockStepTaskScheduler* scheduler = LockStepTaskScheduler::instance();
	parent->barrier.init(scheduler->getNumThreads());
	scheduler->dispatchTask( task_radixsort, this );
      }
      
    private:
      
      void radixIteration(const unsigned b, 
			  const Ty* __restrict src, Ty* __restrict dst, 
			  const size_t startID, const size_t endID, 
			  const size_t threadIndex, const size_t threadCount)
      {
	/* shift and mask to extract some number of bits */
	const unsigned mask = RADIX_BUCKETS_MASK;
	const unsigned shift = b * RADIX_BITS;
        
	/* count how many items go into the buckets */
	for (size_t i=0; i<RADIX_BUCKETS; i++)
	  parent->radixCount[threadIndex][i] = 0;
	
	for (size_t i=startID; i<endID; i++) {
	  const unsigned int index = ((unsigned)src[i] >> shift) & mask;
	  parent->radixCount[threadIndex][index]++;
	}
	parent->barrier.wait(threadIndex,threadCount);
	
	/* calculate total number of items for each bucket */
	__aligned(64) size_t total[RADIX_BUCKETS];
	for (size_t i=0; i<RADIX_BUCKETS; i++)
	  total[i] = 0;
	
	for (size_t i=0; i<threadCount; i++)
	  for (size_t j=0; j<RADIX_BUCKETS; j++)
	    total[j] += parent->radixCount[i][j];
	
	/* calculate start offset of each bucket */
	__aligned(64) size_t offset[RADIX_BUCKETS];
	offset[0] = 0;
	for (size_t i=1; i<RADIX_BUCKETS; i++)    
	  offset[i] = offset[i-1] + total[i-1];
	
	/* calculate start offset of each bucket for this thread */
	for (size_t i=0; i<threadIndex; i++)
	  for (size_t j=0; j<RADIX_BUCKETS; j++)
	    offset[j] += parent->radixCount[i][j];
	
	/* copy items into their buckets */
	for (size_t i=startID; i<endID; i++) {
	  const Ty elt = src[i];
	  const unsigned int index = ((unsigned)elt >> shift) & mask;
	  dst[offset[index]++] = elt;
	}
	if (b < 2) 
	  parent->barrier.wait(threadIndex,threadCount);
      }
      
      void radixsort(const size_t threadIndex, const size_t numThreads)
      {
	const size_t startID = (threadIndex+0)*N/numThreads;
	const size_t endID   = (threadIndex+1)*N/numThreads;
	radixIteration(0,src,dst,startID,endID,threadIndex,numThreads);
	radixIteration(1,dst,tmp,startID,endID,threadIndex,numThreads);
	radixIteration(2,tmp,dst,startID,endID,threadIndex,numThreads);
      }
      
      static void task_radixsort (void* data, const size_t threadIndex, const size_t threadCount) { 
	((ParallelTask*)data)->radixsort(threadIndex,threadCount);                          
      }
      
    private:
      ParallelRadixSortU32* const parent;
      Ty* const src;
      Ty* const tmp;
      Ty* const dst;
      const size_t N;
    };
    
    template<typename Ty>
      static bool compare(const Ty& v0, const Ty& v1) {
      return (unsigned)v0 < (unsigned)v1;
    }
    
  public:
    
    template<typename Ty>
      void operator() (Ty* src, Ty* tmp, Ty* dst, const size_t N)
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
      else        
	ParallelTask<Ty>(this,src,tmp,dst,N);
    }
    
  private:
    TyRadixCount radixCount;
    LinearBarrierActive barrier;
  };
  
  extern ParallelRadixSortU32 radix_sort_u32;
}
