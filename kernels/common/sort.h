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

namespace embree
{
  //namespace isa
  //{
    template<typename Ty>
    class ParallelSortUInt32
    {
      static const size_t RADIX_BITS = 11;
      static const size_t RADIX_BUCKETS = (1 << RADIX_BITS);
      static const size_t RADIX_BUCKETS_MASK = (RADIX_BUCKETS-1);

      typedef unsigned int ThreadRadixCountTy[RADIX_BUCKETS];

    public:
      ParallelSortUInt32 () {} // FIXME: remove

      ParallelSortUInt32 (LockStepTaskScheduler* scheduler) : scheduler(scheduler), N(0) 
      {
        const size_t numThreads = scheduler->getNumThreads();
        radixCount = (ThreadRadixCountTy*) alignedMalloc(numThreads*sizeof(ThreadRadixCountTy));
      }

      ~ParallelSortUInt32 () {
        alignedFree(radixCount);
      }

      void radixIteration(const unsigned b, 
                          const Ty* __restrict src, Ty* __restrict dst, 
                          const size_t startID, const size_t endID, 
                          const size_t threadID, const size_t numThreads)
      {
        ThreadRadixCountTy* radixCount = this->radixCount;

        /* shift and mask to extract some number of bits */
        const unsigned mask = RADIX_BUCKETS_MASK;
        const unsigned shift = b * RADIX_BITS;
          
        /* count how many items go into the buckets */
        for (size_t i=0; i<RADIX_BUCKETS; i++)
          radixCount[threadID][i] = 0;
        
        for (size_t i=startID; i<endID; i++) {
          const unsigned int index = ((unsigned)src[i] >> shift) & mask;
          radixCount[threadID][index]++;
        }
        barrier.wait(threadID,numThreads);
        
        /* calculate total number of items for each bucket */
        __aligned(64) size_t total[RADIX_BUCKETS];
        for (size_t i=0; i<RADIX_BUCKETS; i++)
          total[i] = 0;
        
        for (size_t i=0; i<numThreads; i++)
          for (size_t j=0; j<RADIX_BUCKETS; j++)
            total[j] += radixCount[i][j];
        
        /* calculate start offset of each bucket */
        __aligned(64) size_t offset[RADIX_BUCKETS];
        offset[0] = 0;
        for (size_t i=1; i<RADIX_BUCKETS; i++)    
          offset[i] = offset[i-1] + total[i-1];
        
        /* calculate start offset of each bucket for this thread */
        for (size_t j=0; j<RADIX_BUCKETS; j++)
          for (size_t i=0; i<threadID; i++) // FIXME: switch order of loops?
            offset[j] += radixCount[i][j];
          
        /* copy items into their buckets */
        for (size_t i=startID; i<endID; i++) {
          const Ty elt = src[i];
          const unsigned int index = ((unsigned)elt >> shift) & mask;
          dst[offset[index]++] = elt;
        }
        if (b < 2) 
          barrier.wait(threadID,numThreads);
      }

      void radixsort(const size_t threadID, const size_t numThreads)
      {
        const size_t startID = (threadID+0)*N/numThreads;
        const size_t endID   = (threadID+1)*N/numThreads;
        radixIteration(0,src,dst,startID,endID,threadID,numThreads);
        radixIteration(1,dst,tmp,startID,endID,threadID,numThreads);
        radixIteration(2,tmp,dst,startID,endID,threadID,numThreads);
      }

      static void task_radixsort (void* data, const size_t threadID, const size_t numThreads) { 
        ((ParallelSortUInt32*)data)->radixsort(threadID,numThreads);                          
      }

      void operator() (Ty* src, Ty* tmp, Ty* dst, const size_t N)
      {
        this->src = src;
        this->tmp = tmp;
        this->dst = dst;
        this->N = N;

        /* sort morton codes */
        barrier.init(scheduler->getNumThreads());
        scheduler->dispatchTask( task_radixsort, this );
        
        /* verify that array is sorted */
#if defined(DEBUG)
        for (size_t i=1; i<N; i++)
          assert(morton[i-1].code <= morton[i].code);
#endif	    
      }
      
      /* state shared over multiple sorts */
    private:
      size_t numThreads;
      ThreadRadixCountTy* radixCount;
      LinearBarrierActive barrier;
      LockStepTaskScheduler* scheduler;

      /* temporary state for each sort */
    private:
      size_t N;
      Ty* src;
      Ty* tmp;
      Ty* dst;
    };
    //}
}
