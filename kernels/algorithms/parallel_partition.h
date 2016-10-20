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

#include "parallel_for.h"

namespace embree
{
  /* serial partitioning */
  template<typename T, typename V, typename IsLeft, typename Reduction_T>
    __forceinline size_t serial_partitioning(T* array, 
                                             const size_t begin,
                                             const size_t end, 
                                             V& leftReduction,
                                             V& rightReduction,
                                             const IsLeft& is_left, 
                                             const Reduction_T& reduction_t)
  {
    T* l = array + begin;
    T* r = array + end - 1;
    
    while(1)
    {
      /* *l < pivot */
      while (likely(l <= r && is_left(*l) )) 
      {
#if defined(__AVX512F__)
        prefetch<PFHINT_L1EX>(l+4);	  
#endif
        reduction_t(leftReduction,*l);
        ++l;
      }
      /* *r >= pivot) */
      while (likely(l <= r && !is_left(*r)))
      {
#if defined(__AVX512F__)
        prefetch<PFHINT_L1EX>(r-4);	  
#endif
        reduction_t(rightReduction,*r);
        --r;
      }
      if (r<l) break;
      
      reduction_t(leftReduction ,*r);
      reduction_t(rightReduction,*l);
      xchg(*l,*r);
      l++; r--;
    }
    
    return l - array;        
  }
  
  template<size_t BLOCK_SIZE, typename T, typename V, typename Vi, typename IsLeft, typename Reduction_T, typename Reduction_V>
    class __aligned(64) parallel_partition_static_task
  {
    ALIGNED_CLASS;
  private:

    struct Range 
    {
      ssize_t start;
      ssize_t end;

      __forceinline Range() {}

      __forceinline Range (ssize_t start, ssize_t end) 
      : start(start), end(end) {}

      __forceinline void reset() { 
        start = 0; end = -1; 
      } 
	
      __forceinline Range intersect(const Range& r) const {
        return Range (max(start,r.start),min(end,r.end));
      }

      __forceinline bool empty() const { 
        return end < start; 
      } 
	
      __forceinline size_t size() const { 
        assert(!empty());
        return end-start+1; 
      }
    };

  private:

    static const size_t MAX_TASKS = 512;

    T* array;
    size_t N;
    const IsLeft& is_left;
    const Reduction_T& reduction_t;
    const Reduction_V& reduction_v;
    const Vi& init;

    size_t numTasks; 
    __aligned(64) size_t counter_start[MAX_TASKS+1]; 
    __aligned(64) size_t counter_left[MAX_TASKS+1];  
    __aligned(64) Range leftMisplacedRanges[MAX_TASKS];  
    __aligned(64) Range rightMisplacedRanges[MAX_TASKS]; 
    __aligned(64) V leftReductions[MAX_TASKS];           
    __aligned(64) V rightReductions[MAX_TASKS];    

  public:
     
    __forceinline parallel_partition_static_task(T* array, 
                                                 const size_t N, 
                                                 const size_t maxNumThreads,
                                                 const Vi& init, 
                                                 const IsLeft& is_left, 
                                                 const Reduction_T& reduction_t, 
                                                 const Reduction_V& reduction_v) 

      : array(array), N(N), is_left(is_left), reduction_t(reduction_t), reduction_v(reduction_v), init(init),
      numTasks(min((N+BLOCK_SIZE-1)/BLOCK_SIZE,min(maxNumThreads,MAX_TASKS))) {}

    __forceinline const Range* findStartRange(size_t& index, const Range* const r, const size_t numRanges)
    {
      size_t i = 0;
      while(index >= r[i].size())
      {
        assert(i < numRanges);
        index -= r[i].size();
        i++;
      }	    
      return &r[i];
    }

    __forceinline void swapItemsInMisplacedRanges(const size_t numLeftMisplacedRanges,
                                                  const size_t numRightMisplacedRanges,
                                                  const size_t startID,
                                                  const size_t endID)
    {

      size_t leftLocalIndex  = startID;
      size_t rightLocalIndex = startID;

      const Range* l_range = findStartRange(leftLocalIndex,leftMisplacedRanges,numLeftMisplacedRanges);
      const Range* r_range = findStartRange(rightLocalIndex,rightMisplacedRanges,numRightMisplacedRanges);

      size_t l_left = l_range->size() - leftLocalIndex;
      size_t r_left = r_range->size() - rightLocalIndex;

      size_t size = endID - startID;

      T *__restrict__ l = &array[l_range->start + leftLocalIndex];
      T *__restrict__ r = &array[r_range->start + rightLocalIndex];

      size_t items = min(size,min(l_left,r_left)); 

      while(size)
      {
        if (unlikely(l_left == 0))
        {
          l_range++;
          l_left = l_range->size();
          l = &array[l_range->start];
          items = min(size,min(l_left,r_left));

        }

        if (unlikely(r_left == 0))
        {		
          r_range++;
          r_left = r_range->size();
          r = &array[r_range->start];          
          items = min(size,min(l_left,r_left));
        }

        size   -= items;
        l_left -= items;
        r_left -= items;

        while(items) {
          items--;
          xchg(*l++,*r++);
        }
      }
    }

    __forceinline size_t partition(V& leftReduction, V& rightReduction)
    {
      /* fall back to single threaded partition for small N */
      if (unlikely(N < BLOCK_SIZE))
        return serial_partitioning(array,0,N,leftReduction,rightReduction,is_left,reduction_t);

      /* partition the individual ranges for each task */
      parallel_for(numTasks,[&] (const size_t taskID) {
          const size_t startID = (taskID+0)*N/numTasks;
          const size_t endID   = (taskID+1)*N/numTasks;
          V local_left(init);
          V local_right(init);
          const size_t mid = serial_partitioning(array,startID,endID,local_left,local_right,is_left,reduction_t);
          counter_start[taskID] = startID;
          counter_left [taskID] = mid-startID;
          leftReductions[taskID]  = local_left;
          rightReductions[taskID] = local_right;
        });
      counter_start[numTasks] = N;
      counter_left[numTasks]  = 0;
      
      /* finalize the reductions */
      for (size_t i=0; i<numTasks; i++) {
        reduction_v(leftReduction,leftReductions[i]);
        reduction_v(rightReduction,rightReductions[i]);
      }

      /* calculate mid point for partitioning */
      size_t mid = counter_left[0];
      for (size_t i=1; i<numTasks; i++)
        mid += counter_left[i];
      const Range globalLeft (0,mid-1);
      const Range globalRight(mid,N-1);

      /* calculate all left and right ranges that are on the wrong global side */
      size_t numMisplacedRangesLeft  = 0;
      size_t numMisplacedRangesRight = 0;
      size_t numMisplacedItemsLeft   = 0;
      size_t numMisplacedItemsRight  = 0;

      for (size_t i=0; i<numTasks; i++)
      {	    
        const Range left_range (counter_start[i], counter_start[i] + counter_left[i]-1);
        const Range right_range(counter_start[i] + counter_left[i], counter_start[i+1]-1);
        const Range left_misplaced  = globalLeft. intersect(right_range);
        const Range right_misplaced = globalRight.intersect(left_range);

        if (!left_misplaced.empty())  
        {
          numMisplacedItemsLeft += left_misplaced.size();
          leftMisplacedRanges[numMisplacedRangesLeft++] = left_misplaced;
        }

        if (!right_misplaced.empty()) 
        {
          numMisplacedItemsRight += right_misplaced.size();
          rightMisplacedRanges[numMisplacedRangesRight++] = right_misplaced;
        }
      }
      assert( numMisplacedItemsLeft == numMisplacedItemsRight );

      /* if no items are misplaced we are done */
      if (numMisplacedItemsLeft == 0)
        return mid;

      /* otherwise we copy the items to the right place in parallel */
      parallel_for(numTasks,[&] (const size_t taskID) {
          const size_t startID = (taskID+0)*numMisplacedItemsLeft/numTasks;
          const size_t endID   = (taskID+1)*numMisplacedItemsLeft/numTasks;
          swapItemsInMisplacedRanges(numMisplacedRangesLeft,numMisplacedRangesRight,startID,endID);	                             
        });

      return mid;
    }
  };

  template<size_t BLOCK_SIZE, typename T, typename V, typename Vi, typename IsLeft, typename Reduction_T, typename Reduction_V>
    __forceinline size_t parallel_in_place_partitioning_static(T *array, 
                                                               const size_t N, 
                                                               const Vi &init,
                                                               V &leftReduction,
                                                               V &rightReduction,
                                                               const IsLeft& is_left, 
                                                               const Reduction_T& reduction_t,
                                                               const Reduction_V& reduction_v,
                                                               const size_t numThreads = TaskScheduler::threadCount())
  {
#if defined(__X86_64__) 
    typedef parallel_partition_static_task<BLOCK_SIZE, T,V,Vi,IsLeft,Reduction_T,Reduction_V> partition_task;
    std::unique_ptr<partition_task> p(new partition_task(array,N,numThreads,init,is_left,reduction_t,reduction_v));
    return p->partition(leftReduction,rightReduction);    
#else
    return serial_partitioning(array,size_t(0),N,leftReduction,rightReduction,is_left,reduction_t);
#endif
  }

}
