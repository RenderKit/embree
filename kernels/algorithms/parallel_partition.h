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

namespace embree
{
  /* serial partitioning */
  template<typename T, typename V, typename Compare, typename Reduction_T>
    __forceinline size_t serial_partitioning(T* array, 
                                             const size_t begin,
                                             const size_t end, 
                                             V& leftReduction,
                                             V& rightReduction,
                                             const Compare& cmp, 
                                             const Reduction_T& reduction_t)
  {
    T* l = array + begin;
    T* r = array + end - 1;
    
    while(1)
    {
      /* *l < pivot */
      while (likely(l <= r && cmp(*l) )) 
      {
#if defined(__AVX512F__)
        prefetch<PFHINT_L1EX>(l+4);	  
#endif
        reduction_t(leftReduction,*l);
        ++l;
      }
      /* *r >= pivot) */
      while (likely(l <= r && !cmp(*r)))
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
  

  template<size_t BLOCK_SIZE, typename T, typename V, typename Compare, typename Reduction_T, typename Reduction_V>
    class __aligned(64) parallel_partition
  {
  private:

    static const size_t MAX_TASKS = 256;

    const Compare& cmp;
    const Reduction_T& reduction_t;
    const Reduction_V& reduction_v;
      
    const V &init;

    size_t N;
    size_t blocks;
    T* array;
     

    __aligned(64) std::atomic<uint64_t> blockID;
    __aligned(64) std::atomic<uint64_t> numLeftRemainderBlocks; 
    __aligned(64) std::atomic<uint64_t> numRightRemainderBlocks; 
    __aligned(64) std::atomic<uint32_t> maxLeftBlockID;
    __aligned(64) std::atomic<uint32_t> maxRightBlockID;
      
    unsigned int  leftRemainderBlockIDs[MAX_TASKS]; 
    unsigned int rightRemainderBlockIDs[MAX_TASKS];

    V leftReductions[MAX_TASKS]; // FIXME: remove explicit storage
    V rightReductions[MAX_TASKS];


    enum {
      NEED_LEFT_BLOCK           = 1,
      NEED_RIGHT_BLOCK          = 2
    };

    /* do we need a left block? */
    __forceinline bool needLeftBlock(const size_t mode)  { return (mode & NEED_LEFT_BLOCK)  == NEED_LEFT_BLOCK; }

    /* do we need a right block? */
    __forceinline bool needRightBlock(const size_t mode) { return (mode & NEED_RIGHT_BLOCK) == NEED_RIGHT_BLOCK; }

    /* do we need both blocks? */
    __forceinline bool needBothBlocks(const size_t mode) { return needLeftBlock(mode) && needRightBlock(mode); }
      
    /* get left/right atomic block id */
    __forceinline int64_t getBlockID(const size_t mode)
    {
      int64_t v = 0;
      if (needLeftBlock(mode))  v |= 1;
      if (needRightBlock(mode)) v |= (int64_t)1 << 32;
      int64_t val = blockID.fetch_add(v);
      return val;
    }

    /* get left index from block id */
    __forceinline int32_t getLeftBlockIndex(const int64_t id) { return id & 0xffffffff; }

    /* get right index from block id */
    __forceinline int32_t getRightBlockIndex(const int64_t id) { return id >> 32; }
 
    /* get left array index from block index */
    __forceinline void getLeftArrayIndex(const size_t blockIndex, size_t &begin, size_t &end) 
    { 
      begin = blockIndex * BLOCK_SIZE; 
      end   = begin + BLOCK_SIZE; 
    }

    /* get right array index from block index */
    __forceinline void getRightArrayIndex(const size_t blockIndex, size_t &begin, size_t &end) 
    { 
      begin = N - (blockIndex+1) * BLOCK_SIZE; 
      end   = begin + BLOCK_SIZE; 
    }

    /* is block id valid? */
    __forceinline bool validBlockID(const int64_t id)
    {
      const size_t numLeftBlocks  = getLeftBlockIndex(id) + 1;
      const size_t numRightBlocks = getRightBlockIndex(id) + 1;
      return numLeftBlocks+numRightBlocks <= blocks;
    }

    /* swap to blocks */
    __forceinline void swapTwoBlocks(const size_t index0, const size_t index1)
    {
      assert(index0 != index1);
      for (size_t i=0;i<BLOCK_SIZE;i++)
        xchg(array[index0+i],array[index1+i]);                
    }

    /* swap to left blocks */
    __forceinline void swapTwoLeftBlocks(const size_t leftID0, const size_t leftID1)
    {
      assert( leftID0 != leftID1 );
      size_t left0_begin, left0_end;
      size_t left1_begin, left1_end;

      getLeftArrayIndex(leftID0, left0_begin, left0_end);
      getLeftArrayIndex(leftID1, left1_begin, left1_end);

      swapTwoBlocks(left0_begin,left1_begin);
    }

    /* swap to right blocks */
    __forceinline void swapTwoRightBlocks(const size_t rightID0, const size_t rightID1)
    {
      assert( rightID0 != rightID1 );
      size_t right0_begin, right0_end;
      size_t right1_begin, right1_end;

      getRightArrayIndex(rightID0, right0_begin, right0_end);
      getRightArrayIndex(rightID1, right1_begin, right1_end);

      swapTwoBlocks(right0_begin,right1_begin);
    }

    /* serial partitioning */
    __forceinline size_t serialPartitioning(const size_t begin, 
                                            const size_t end, 
                                            V &leftReduc, 
                                            V &rightReduc)
    {
      T* l = array + begin;
      T* r = array + end - 1;

      while(1)
      {
        /* *l < pivot */
        while (likely(l <= r && cmp(*l) )) 
        {
#if defined(__AVX512F__)
          prefetch<PFHINT_L1EX>(l+4);	  
#endif
          //if (!cmp(*l)) break;
          reduction_t(leftReduc,*l);
          ++l;
        }
        /* *r >= pivot) */
        while (likely(l <= r && !cmp(*r)))
        {
#if defined(__AVX512F__)
          prefetch<PFHINT_L1EX>(r-4);	  
#endif
          //if (cmp(*r)) break;

          reduction_t(rightReduc,*r);
          --r;
        }
        if (r<l) break;

        reduction_t(leftReduc ,*r);
        reduction_t(rightReduc,*l);
        xchg(*l,*r);
        l++; r--;
      }
      
      return l - array;        
    }

    /* neutralize left and right block */
    __forceinline size_t neutralizeBlocks(size_t &left_begin,
                                          const size_t &left_end,
                                          size_t &right_begin,
                                          const size_t &right_end,
                                          V &leftReduc, 
                                          V &rightReduc)
    {
      while(left_begin < left_end && right_begin < right_end)
      {
        while(cmp(array[left_begin]) /* array[left_begin] < pivot */)
        {
#if defined(__AVX512F__)
          prefetch<PFHINT_L1EX>(&array[left_begin] + 2);	  
#endif


          left_begin++;
          if (left_begin >= left_end) break;
        }

        while(!cmp(array[right_begin]) /* array[right_begin] >= pivot */)
        {
#if defined(__AVX512F__)
          prefetch<PFHINT_L1EX>(&array[right_begin] - 2);	  
#endif

          right_begin++;
          if (right_begin >= right_end) break;
        }

        if (unlikely(left_begin == left_end || right_begin == right_end)) break;
            
        xchg(array[left_begin++],array[right_begin++]);
      }

      size_t mode = 0;
      if (unlikely(left_begin == left_end))
      {
        for (size_t i=left_end-BLOCK_SIZE;i<left_end;i++)
          reduction_t(leftReduc,array[i]);

        mode |= NEED_LEFT_BLOCK;
      }

      if (unlikely(right_begin == right_end))
      {
        for (size_t i=right_end-BLOCK_SIZE;i<right_end;i++)
          reduction_t(rightReduc,array[i]);

        mode |= NEED_RIGHT_BLOCK;
      }

      assert(mode != 0);
      return mode;
    }

  public:

    /* initialize atomic counters */
    __forceinline parallel_partition(T *array, size_t N, const V& init, const Compare& cmp, const Reduction_T& reduction_t, const Reduction_V& reduction_v) : array(array), N(N), init(init), cmp(cmp), reduction_t(reduction_t) , reduction_v(reduction_v)
    {
      blockID.store(0);
      numLeftRemainderBlocks  = 0;
      numRightRemainderBlocks = 0;
      maxLeftBlockID          = 0;
      maxRightBlockID         = 0;
        
      blocks = N/BLOCK_SIZE;
    }

    /* each thread neutralizes blocks taken from left and right */
    void thread_partition(V &leftReduction,
                          V &rightReduction)
    {
      size_t mode = NEED_LEFT_BLOCK | NEED_RIGHT_BLOCK;
        
      size_t left_begin  = (size_t)-1;
      size_t left_end    = (size_t)-1;
      size_t right_begin = (size_t)-1;
      size_t right_end   = (size_t)-1;

      size_t currentLeftBlock  = (size_t)-1;
      size_t currentRightBlock = (size_t)-1;

      leftReduction  = init;
      rightReduction = init;

      while(1)
      {
        int64_t id = getBlockID(mode);
        if (!validBlockID(id)) break;

        /* need a left block? */
        if (needLeftBlock(mode))
        {

          const size_t blockIndex = getLeftBlockIndex(id);
          getLeftArrayIndex(blockIndex,left_begin,left_end);                
          currentLeftBlock = blockIndex;
        }

        /* need a right block? */
        if (needRightBlock(mode))
        {
          const size_t blockIndex = getRightBlockIndex(id);
          getRightArrayIndex(blockIndex,right_begin,right_end);
          currentRightBlock = blockIndex;
        }
            
        assert(left_begin  < left_end);
        assert(right_begin < right_end);
        assert(left_end <= right_begin);

        mode = neutralizeBlocks(left_begin,left_end,right_begin,right_end,leftReduction,rightReduction);
      }        

      assert(left_end <= right_begin);

      if (left_begin != left_end)
      {
        const size_t index = numLeftRemainderBlocks++;
        leftRemainderBlockIDs[index] = currentLeftBlock;
      }

      if (currentLeftBlock != (size_t)-1)
        atomic_max(maxLeftBlockID,(uint32_t)currentLeftBlock);

      if (right_begin != right_end)
      {
        const size_t index = numRightRemainderBlocks++;
        rightRemainderBlockIDs[index] = currentRightBlock;
      }

      if (currentRightBlock != (size_t)-1)
        atomic_max(maxRightBlockID,(uint32_t)currentRightBlock);

    }

    static void task_thread_partition(void* data, const size_t threadIndex, const size_t threadCount) {
      parallel_partition<BLOCK_SIZE,T,V,Compare,Reduction_T,Reduction_V>* p = (parallel_partition<BLOCK_SIZE,T,V,Compare,Reduction_T,Reduction_V>*)data;
      V left;
      V right;
      p->thread_partition(left,right);
      p->leftReductions[threadIndex]  = left;
      p->rightReductions[threadIndex] = right;
    } 

    /* main function for parallel in-place partitioning */
    size_t partition_parallel(V &leftReduction,
                              V &rightReduction)
    //				Scheduler &scheduler)
    {    
      leftReduction = init;
      rightReduction = init;
      const size_t numThreads = min(TaskScheduler::threadCount(),MAX_TASKS);

      if (N <= 2 * BLOCK_SIZE * numThreads) // need at least 1 block from the left and 1 block from the right per thread
      {
        size_t mid = serialPartitioning(0,N,leftReduction,rightReduction);
        return mid;
      }

      parallel_for(numThreads,[&] (const size_t threadIndex) {
          task_thread_partition(this,threadIndex,numThreads);
        });

      /* ---------------------------------- */
      /* ------ serial cleanup phase ------ */
      /* ---------------------------------- */

      size_t left_begin = (size_t)-1;
      size_t left_end   = (size_t)-1;
      size_t right_begin = (size_t)-1;
      size_t right_end   = (size_t)-1;
        
      if (maxLeftBlockID != 0 || maxRightBlockID != 0) // has any thread done anything?
      {
        /* sort remainder blocks */
        insertionsort_ascending<unsigned int>(leftRemainderBlockIDs,(unsigned int)numLeftRemainderBlocks);
        insertionsort_ascending<unsigned int>(rightRemainderBlockIDs,(unsigned int)numRightRemainderBlocks);

        /* compact left remaining blocks */        
        for (size_t i=0;i<numLeftRemainderBlocks;i++)
        {
          assert(i<=maxLeftBlockID);
          const unsigned int index0 = leftRemainderBlockIDs[numLeftRemainderBlocks-1-i];
          const unsigned int index1 = maxLeftBlockID-i;
          if (index0 != index1)
            swapTwoLeftBlocks(index0,index1);
        }
        assert(numLeftRemainderBlocks-1 <= maxLeftBlockID);

        const size_t left_border_index = maxLeftBlockID-(numLeftRemainderBlocks-1);


        getLeftArrayIndex(left_border_index,left_begin,left_end);

        assert(left_begin != (size_t)-1);

        /* compact right remaining blocks */


        for (size_t i=0;i<numRightRemainderBlocks;i++)
        {
          assert(i<=maxRightBlockID);
          const unsigned int index0 = rightRemainderBlockIDs[numRightRemainderBlocks-1-i];
          const unsigned int index1 = maxRightBlockID-i;
          if (index0 != index1)
            swapTwoRightBlocks(index0,index1);
        }
        assert(numRightRemainderBlocks-1 <= maxRightBlockID);

        const size_t right_border_index = maxRightBlockID-(numRightRemainderBlocks-1);


        getRightArrayIndex(right_border_index,right_begin,right_end);

        assert(right_end  != (size_t)-1);

      }
      else
      {
        left_begin = 0;
        right_end  = N;
      }

      const size_t mid = serialPartitioning(left_begin,right_end,leftReduction,rightReduction);

      for (size_t i=0;i<numThreads;i++)
      {
        reduction_v(leftReduction,leftReductions[i]);
        reduction_v(rightReduction,rightReductions[i]);
      }
        

      return mid;
    }


    size_t partition_serial(V &leftReduction,
                            V &rightReduction)
    {
      leftReduction = init;
      rightReduction = init;
      const size_t mid = serialPartitioning(0,N,leftReduction,rightReduction);      
      return mid;
    }

  };

  template<size_t BLOCK_SIZE, typename T, typename V, typename Compare, typename Reduction_T, typename Reduction_V>
    __forceinline size_t parallel_in_place_partitioning_dynamic(T *array, 
                                                        size_t N, 
                                                        const V &init,
                                                        V &leftReduction,
                                                        V &rightReduction,
                                                        const Compare& cmp, 
                                                        const Reduction_T& reduction_t,
                                                        const Reduction_V& reduction_v)
  //							Scheduler &scheduler)
  {
#if defined(__X86_64__) 
    parallel_partition<BLOCK_SIZE,T,V,Compare,Reduction_T,Reduction_V> p(array,N,init,cmp,reduction_t,reduction_v);
    return p.partition_parallel(leftReduction,rightReduction);    
#else
    return serial_partitioning(array,size_t(0),N,leftReduction,rightReduction,cmp,reduction_t);
#endif
  }

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////

  template<size_t BLOCK_SIZE, typename T, typename V, typename Compare, typename Reduction_T, typename Reduction_V>
    class __aligned(64) parallel_partition_static_task
  {
    ALIGNED_CLASS;
  private:

    struct Range 
    {
      ssize_t start;
      ssize_t end;
      Range() {}

      Range (ssize_t start, ssize_t end) 
      : start(start), end(end) {}

      __forceinline void reset() { 
        start = 0; end = -1; 
      } 
	
      __forceinline Range intersect(const Range& r) const {
        return Range (max(start,r.start),min(end,r.end)); // carefull with ssize_t here
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
    size_t tasks; 
    const Compare& cmp;
    const Reduction_T& reduction_t;
    const Reduction_V& reduction_v;
    const V &init;

    size_t numMisplacedRangesLeft;
    size_t numMisplacedRangesRight;
    size_t numMisplacedItems;

    __aligned(64) size_t counter_start[MAX_TASKS]; 
    __aligned(64) size_t counter_left[MAX_TASKS];  
    __aligned(64) Range leftMisplacedRanges[MAX_TASKS];  
    __aligned(64) Range rightMisplacedRanges[MAX_TASKS]; 
    __aligned(64) V leftReductions[MAX_TASKS];           
    __aligned(64) V rightReductions[MAX_TASKS];    

  public:
     
    __forceinline parallel_partition_static_task(T *array, 
                                                 const size_t N, 
                                                 const size_t maxNumThreads,
                                                 const V& init, 
                                                 const Compare& cmp, 
                                                 const Reduction_T& reduction_t, 
                                                 const Reduction_V& reduction_v) 
      : array(array), N(N), cmp(cmp), reduction_t(reduction_t), reduction_v(reduction_v), init(init)
    {
      numMisplacedRangesLeft  = 0;
      numMisplacedRangesRight = 0;
      numMisplacedItems  = 0;
      tasks = (N+maxNumThreads-1)/maxNumThreads >= BLOCK_SIZE ? maxNumThreads : (N+BLOCK_SIZE-1)/BLOCK_SIZE;
      tasks = min(tasks,MAX_TASKS);
    }

    __forceinline const Range *findStartRange(size_t &index,const Range *const r,const size_t numRanges)
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

    __forceinline void swapItemsInMisplacedRanges(const Range * const leftMisplacedRanges,
                                                  const size_t numLeftMisplacedRanges,
                                                  const Range * const rightMisplacedRanges,
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


    __forceinline size_t partition(V &leftReduction,
                                   V &rightReduction)
    {
      if (unlikely(N < BLOCK_SIZE))
      {
        leftReduction = empty;
        rightReduction = empty;
        return serial_partitioning(array,0,N,leftReduction,rightReduction,cmp,reduction_t);
      }

      parallel_for(tasks,[&] (const size_t taskID) 
                   {
                     const size_t startID = (taskID+0)*N/tasks;
                     const size_t endID   = (taskID+1)*N/tasks;
                     V local_left(empty);
                     V local_right(empty);
                     const size_t mid = serial_partitioning(array,startID,endID,local_left,local_right,cmp,reduction_t);
                     counter_start[taskID] = startID;
                     counter_left [taskID] = mid-startID;
                     leftReductions[taskID]  = local_left;
                     rightReductions[taskID] = local_right;
                   });

      leftReduction = empty;
      rightReduction = empty;

      for (size_t i=0;i<tasks;i++)
      {
        reduction_v(leftReduction,leftReductions[i]);
        reduction_v(rightReduction,rightReductions[i]);
      }

      numMisplacedRangesLeft  = 0;
      numMisplacedRangesRight = 0;
      size_t numMisplacedItemsLeft   = 0;
      size_t numMisplacedItemsRight  = 0;
	
      counter_start[tasks] = N;
      counter_left[tasks]  = 0;

      size_t mid = counter_left[0];
      for (size_t i=1;i<tasks;i++)
        mid += counter_left[i];

      const Range globalLeft (0,mid-1);
      const Range globalRight(mid,N-1);

      // without pragma the compiler makes a mess out of this loop
#if defined(__INTEL_COMPILER)
#pragma novector // FIXME: does this make a performance difference at all?
#endif
      for (size_t i=0;i<tasks;i++)
      {	    
        const size_t left_start  = counter_start[i];
        const size_t left_end    = counter_start[i] + counter_left[i]-1;
        const size_t right_start = counter_start[i] + counter_left[i];
        const size_t right_end   = counter_start[i+1]-1;

        Range left_range (left_start,left_end); // counter[i].start,counter[i].start+counter[i].left-1);
        Range right_range(right_start,right_end); // counter[i].start+counter[i].left,counter[i].start+counter[i].size-1);

        Range left_misplaced = globalLeft.intersect(right_range);
        Range right_misplaced = globalRight.intersect(left_range);

        if (!left_misplaced.empty())  
        {
          numMisplacedItemsLeft  += left_misplaced.size();
          leftMisplacedRanges[numMisplacedRangesLeft++] = left_misplaced;
        }

        if (!right_misplaced.empty()) 
        {
          numMisplacedItemsRight += right_misplaced.size();
          rightMisplacedRanges[numMisplacedRangesRight++] = right_misplaced;
        }
      }

      assert( numMisplacedItemsLeft == numMisplacedItemsRight );
	
      numMisplacedItems = numMisplacedItemsLeft;

      const size_t global_mid = mid;


      ////////////////////////////////////////////////////////////////////////////
      ////////////////////////////////////////////////////////////////////////////
      ////////////////////////////////////////////////////////////////////////////

      if (numMisplacedItems)
      {

        parallel_for(tasks,[&] (const size_t taskID) 
                     {
                       const size_t startID = (taskID+0)*numMisplacedItems/tasks;
                       const size_t endID   = (taskID+1)*numMisplacedItems/tasks;
                       swapItemsInMisplacedRanges(leftMisplacedRanges,
                                                  numMisplacedRangesLeft,
                                                  rightMisplacedRanges,
                                                  numMisplacedRangesRight,
                                                  startID,
                                                  endID);	                             
                     });
      }

      return global_mid;
    }

  };


  template<size_t BLOCK_SIZE, typename T, typename V, typename Compare, typename Reduction_T, typename Reduction_V>
    __forceinline size_t parallel_in_place_partitioning_static(T *array, 
                                                               const size_t N, 
                                                               const V &init,
                                                               V &leftReduction,
                                                               V &rightReduction,
                                                               const Compare& cmp, 
                                                               const Reduction_T& reduction_t,
                                                               const Reduction_V& reduction_v,
                                                               const size_t numThreads = TaskScheduler::threadCount())
  {
#if defined(__X86_64__) 
    typedef parallel_partition_static_task<BLOCK_SIZE, T,V,Compare,Reduction_T,Reduction_V> partition_task;
    std::unique_ptr<partition_task> p(new partition_task(array,N,numThreads,init,cmp,reduction_t,reduction_v));
    return p->partition(leftReduction,rightReduction);    
#else
    return serial_partitioning(array,size_t(0),N,leftReduction,rightReduction,cmp,reduction_t);
#endif
  }

}
