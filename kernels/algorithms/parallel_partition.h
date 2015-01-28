// ======================================================================== //
// Copyright 2009-2015 Intel Corporation                                    //
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
#include "sys/platform.h"
#include "sys/sysinfo.h"
#include "tasking/taskscheduler.h"

#define DBG_PART(x) 
#define DBG_PART2(x) 
#define DBG_CHECK(x) 

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
#if defined(__MIC__)
        prefetch<PFHINT_L1EX>(l+4);	  
#endif
        reduction_t(leftReduction,*l);
        ++l;
      }
      /* *r >= pivot) */
      while (likely(l <= r && !cmp(*r)))
      {
#if defined(__MIC__)
        prefetch<PFHINT_L1EX>(r-4);	  
#endif
        reduction_t(rightReduction,*r);
        --r;
      }
      if (r<l) break;
      
      reduction_t(leftReduction ,*r);
      reduction_t(rightReduction,*l);
      std::swap(*l,*r);
      l++; r--;
    }
    
    return l - array;        
  }
  

  template<size_t BLOCK_SIZE, typename T, typename V, typename Compare, typename Reduction_T, typename Reduction_V>
  class __aligned(64) parallel_partition
    {
    private:

      const Compare& cmp;
      const Reduction_T& reduction_t;
      const Reduction_V& reduction_v;
      
      const V &init;

      size_t N;
      size_t blocks;
      T* array;
     

      AlignedAtomicCounter64 blockID;

      AlignedAtomicCounter64 numLeftRemainderBlocks;
      AlignedAtomicCounter64 numRightRemainderBlocks;
      AlignedAtomicCounter32 maxLeftBlockID;
      AlignedAtomicCounter32 maxRightBlockID;
      
      unsigned int  leftRemainderBlockIDs[MAX_MIC_THREADS];
      unsigned int rightRemainderBlockIDs[MAX_MIC_THREADS];

      V leftReductions[MAX_MIC_THREADS];
      V rightReductions[MAX_MIC_THREADS];


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
      __forceinline size_t getBlockID(const size_t mode)
      {
        size_t v = 0;
        if (needLeftBlock(mode))  v |= 1;
        if (needRightBlock(mode)) v |= (size_t)1 << 32;
        size_t val = blockID.add(v);
        return val;
      }

      /* get left index from block id */
      __forceinline size_t getLeftBlockIndex(const size_t id) { return id & 0xffffffff; }

      /* get right index from block id */
      __forceinline size_t getRightBlockIndex(const size_t id) { return id >> 32; }
 
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
      __forceinline bool validBlockID(const size_t id)
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
          std::swap(array[index0+i],array[index1+i]);                
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
#if defined(__MIC__)
		prefetch<PFHINT_L1EX>(l+4);	  
#endif
		//if (!cmp(*l)) break;
                reduction_t(leftReduc,*l);
               ++l;
              }
	    /* *r >= pivot) */
            while (likely(l <= r && !cmp(*r)))
              {
#if defined(__MIC__)
		prefetch<PFHINT_L1EX>(r-4);	  
#endif
		//if (cmp(*r)) break;

                reduction_t(rightReduc,*r);
                --r;
              }
            if (r<l) break;

            reduction_t(leftReduc ,*r);
            reduction_t(rightReduc,*l);
            std::swap(*l,*r);
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
#if defined(__MIC__)
		prefetch<PFHINT_L1EX>(&array[left_begin] + 2);	  
#endif


                left_begin++;
                if (left_begin >= left_end) break;
              }

            while(!cmp(array[right_begin]) /* array[right_begin] >= pivot */)
              {
#if defined(__MIC__)
		prefetch<PFHINT_L1EX>(&array[right_begin] - 2);	  
#endif

                right_begin++;
                if (right_begin >= right_end) break;
              }

            if (unlikely(left_begin == left_end || right_begin == right_end)) break;
            
            std::swap(array[left_begin++],array[right_begin++]);
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

      /* check left part of array */
      void checkLeft(const size_t begin, const size_t end)
      {
        for (size_t i=begin;i<end;i++)
          if (!cmp(array[i]))
            {
              DBG_PRINT(i);
              DBG_PRINT(array[i]);
              FATAL("partition error on left side");
            }
      }

      /* check right part of array */
      void checkRight(const size_t begin, const size_t end)
      {
        for (size_t i=begin;i<end;i++)
          if (cmp(array[i]))
            {
              DBG_PRINT(i);
              DBG_PRINT(array[i]);
              FATAL("partition error on right side");
            }
      }

    public:

      /* initialize atomic counters */
      __forceinline parallel_partition(T *array, size_t N, const V& init, const Compare& cmp, const Reduction_T& reduction_t, const Reduction_V& reduction_v) : array(array), N(N), init(init), cmp(cmp), reduction_t(reduction_t) , reduction_v(reduction_v)
      {
        blockID.reset();
        numLeftRemainderBlocks  = 0;
        numRightRemainderBlocks = 0;
        maxLeftBlockID          = 0;
        maxRightBlockID         = 0;
        
        blocks = N/BLOCK_SIZE;
        DBG_PART(
                 DBG_PRINT(blocks);
                 );
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
            DBG_PART(
                     std::cout << std::endl;
                     DBG_PRINT("NEXT ITERATION");
                     );

            size_t id = getBlockID(mode);

            DBG_PART(
                     DBG_PRINT(id);
                     DBG_PRINT(validBlockID(id));
                     );

            if (!validBlockID(id)) break;

            /* need a left block? */
            if (needLeftBlock(mode))
              {

                const size_t blockIndex = getLeftBlockIndex(id);
                getLeftArrayIndex(blockIndex,left_begin,left_end);                
                currentLeftBlock = blockIndex;

                DBG_PART(
                         DBG_PRINT("LEFT BLOCK");
                         DBG_PRINT(currentLeftBlock);
                         );

              }

            /* need a right block? */
            if (needRightBlock(mode))
              {
                const size_t blockIndex = getRightBlockIndex(id);
                getRightArrayIndex(blockIndex,right_begin,right_end);
                currentRightBlock = blockIndex;

                DBG_PART(
                         DBG_PRINT("RIGHT BLOCK");
                         DBG_PRINT(currentRightBlock);
                         );
              }

            DBG_PART(
                     DBG_PRINT(left_begin);
                     DBG_PRINT(left_end);
                     DBG_PRINT(right_begin);
                     DBG_PRINT(right_end);
                     );
            
            assert(left_begin  < left_end);
            assert(right_begin < right_end);

            assert(left_end <= right_begin);

            mode = neutralizeBlocks(left_begin,left_end,right_begin,right_end,leftReduction,rightReduction);
          }        

        assert(left_end <= right_begin);

        if (left_begin != left_end)
          {
            const size_t index = numLeftRemainderBlocks.inc();
            leftRemainderBlockIDs[index] = currentLeftBlock;
          }

        if (currentLeftBlock != (size_t)-1)
          maxLeftBlockID.max(currentLeftBlock);

        if (right_begin != right_end)
          {
            const size_t index = numRightRemainderBlocks.inc();
            rightRemainderBlockIDs[index] = currentRightBlock;
          }

        if (currentRightBlock != (size_t)-1)
          maxRightBlockID.max(currentRightBlock);       

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
      {    
        leftReduction = init;
        rightReduction = init;

        LockStepTaskScheduler* scheduler = LockStepTaskScheduler::instance();
        const size_t numThreads = scheduler->getNumThreads();
        if (N <= 2 * BLOCK_SIZE * numThreads) // need at least 1 block from the left and 1 block from the right per thread
          {
#if defined(__MIC__)
            DBG_PRINT("SERIAL FALLBACK");
            DBG_PRINT("numThreads");
#endif
            size_t mid = serialPartitioning(0,N,leftReduction,rightReduction);
            DBG_PART(
                     DBG_PRINT( mid );
                     checkLeft(0,mid);
                     checkRight(mid,N);
                     );
            return mid;
          }

        DBG_PART2(
                 DBG_PRINT("PARALLEL MODE");
                 );


        scheduler->dispatchTask(task_thread_partition,this,0,numThreads);

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

	    DBG_PART(
		     DBG_PRINT(numLeftRemainderBlocks);
		     for (size_t i=0;i<numLeftRemainderBlocks;i++)
		       std::cout << i << " -> " << leftRemainderBlockIDs[i] << std::endl;
		     DBG_PRINT(numRightRemainderBlocks);
		     for (size_t i=0;i<numRightRemainderBlocks;i++)
		       std::cout << i << " -> " << rightRemainderBlockIDs[i] << std::endl;
		     DBG_PRINT(maxLeftBlockID);
		     DBG_PRINT(maxRightBlockID);                 
		     );

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

	    DBG_CHECK( checkLeft(0,left_begin) );

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

	    DBG_CHECK( checkRight(right_end,N) );

	    /* if (!numLeftRemainderBlocks) */
	    /*   left_begin += BLOCK_SIZE; */
        
	    /* if (!numRightRemainderBlocks) */
	    /*   right_end -= BLOCK_SIZE; */
        
	    DBG_PART(
		     DBG_PRINT("CLEANUP");
                 
		     DBG_PRINT(left_begin);
		     DBG_PRINT(left_end);
		     DBG_PRINT(right_begin);
		     DBG_PRINT(right_end);

		     //for (size_t i=0;i<N;i++)
		     //std::cout << i << " -> " << array[i] << std::endl;
		     );

	    DBG_CHECK(
                 
		      checkLeft(0,left_begin);
		      checkRight(right_end,N);
		      );


        
	    DBG_CHECK(
		      assert( right_end - left_begin <= numThreads*3*BLOCK_SIZE);
		      )
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
        

        DBG_CHECK(
                 checkLeft(0,mid);
                 checkRight(mid,N);
                 );
        
        return mid;
      }


      size_t partition_serial(V &leftReduction,
                              V &rightReduction)
      {
        leftReduction = init;
        rightReduction = init;
        const size_t mid = serialPartitioning(0,N,leftReduction,rightReduction);      
        DBG_CHECK(
                 checkLeft(0,mid);
                 checkRight(mid,N);
                 );        
        return mid;
      }

    };

  template<size_t BLOCK_SIZE, typename T, typename V, typename Compare, typename Reduction_T, typename Reduction_V>
    __forceinline size_t parallel_in_place_partitioning(T *array, 
                                                        size_t N, 
                                                        const V &init,
                                                        V &leftReduction,
                                                        V &rightReduction,
                                                        const Compare& cmp, 
                                                        const Reduction_T& reduction_t,
                                                        const Reduction_V& reduction_v)
  {
    parallel_partition<BLOCK_SIZE,T,V,Compare,Reduction_T,Reduction_V> p(array,N,init,cmp,reduction_t,reduction_v);
    return p.partition_parallel(leftReduction,rightReduction);    
  }




  template<typename T, typename V, typename Compare, typename Reduction_T, typename Reduction_V>
    __forceinline size_t serial_in_place_partitioning(T *array, 
                                                      size_t N, 
                                                      const V &init,
                                                      V &leftReduction,
                                                      V &rightReduction,
                                                      const Compare& cmp, 
                                                      const Reduction_T& reduction_t,
                                                      const Reduction_V& reduction_v)
  {
    parallel_partition<1,T,V,Compare,Reduction_T,Reduction_V> p(array,N,init,cmp,reduction_t,reduction_v);
    return p.partition_serial(leftReduction,rightReduction);
  }


  //////////////////////////////////////////////////////////////////////////////////////////////////////////////


  template<typename T, typename Compare, typename ThreadLocalPartition>
  class __aligned(64) parallel_partition_static
    {
    private:

      const Compare& cmp;
      const ThreadLocalPartition& threadLocalPartition;
      
      size_t N;
      size_t blocks;
      size_t global_mid;
      T* array;

      unsigned int counter_start[MAX_MIC_THREADS];
      unsigned int counter_left[MAX_MIC_THREADS];     

      struct Range {
	int start;
	int end;
	Range() {}

      Range(int start, int end) : start(start), end(end) {}

	__forceinline void reset() { start = 0; end = -1; } 
	
	__forceinline Range intersect(const Range& r) const
	{
	  return Range (max(start,r.start),min(end,r.end)); // carefull with ssize_t here
	}

	__forceinline bool empty() const { return end < start; } 
	
	__forceinline size_t size() const { 
	  assert(!empty());
	  return end-start+1; 
	}
      };

      __forceinline size_t findStartRange(size_t &index,const Range *const r,const size_t numRanges)
      {
	size_t i = 0;
	while(index >= r[i].size())
	  {
	    assert(i < numRanges);
	    index -= r[i].size();
	    i++;
	  }	    
	return i;
      }
      
      /* check left part of array */
      void checkLeft(T* const t_array,const size_t begin, const size_t end)
      {
        for (size_t i=begin;i<end;i++)
          if (!cmp(t_array[i]))
            {
              DBG_PRINT(i);
              DBG_PRINT(array[i]);
              FATAL("partition error on left side");
            }
      }

      /* check right part of array */
      void checkRight(T* const t_array,const size_t begin, const size_t end)
      {
        for (size_t i=begin;i<end;i++)
          if (cmp(t_array[i]))
            {
              DBG_PRINT(i);
              DBG_PRINT(array[i]);
              FATAL("partition error on right side");
            }
      }

      size_t partition_serial(T* const t_array,
			      const size_t size)
      {
	const size_t mid = threadLocalPartition(t_array,size); 
        DBG_CHECK(
		  checkLeft(t_array,0,mid);
		  checkRight(t_array,mid,size);
                 );        
        return mid;
      }

    public:

      /* initialize atomic counters */
      __forceinline parallel_partition_static(T *array, 
					      size_t N, 
					      const Compare& cmp,
					      const ThreadLocalPartition& threadLocalPartition) : array(array), N(N), cmp(cmp), threadLocalPartition(threadLocalPartition) 
      {
	global_mid = (size_t)-1;
      }

      /* each thread neutralizes blocks taken from left and right */

      static void task_thread_partition(void* data, const size_t threadID, const size_t numThreads) {

        parallel_partition_static<T,Compare,ThreadLocalPartition>* p = (parallel_partition_static<T,Compare,ThreadLocalPartition>*)data;

	const size_t startID = (threadID+0)*p->N/numThreads;
	const size_t endID   = (threadID+1)*p->N/numThreads;
	const size_t size    = endID-startID;
	
        const size_t mid = p->partition_serial(&p->array[startID],size);
	p->counter_start[threadID] = startID;
	p->counter_left[threadID]  = mid;
      } 


	__aligned(64) Range leftMisplacedRanges[MAX_MIC_THREADS];
	__aligned(64) Range rightMisplacedRanges[MAX_MIC_THREADS];
	size_t numMisplacedRangesLeft;
	size_t numMisplacedRangesRight;
	size_t numMisplacedItems;

      void computeMisplacedRanges(const size_t numThreads)
      {
#if defined(__MIC__)
	for (size_t i=0;i<(numThreads+15)/16;i++)
	  prefetch<PFHINT_L2>(&counter_left[i*16]);
#endif

	numMisplacedRangesLeft  = 0;
	numMisplacedRangesRight = 0;
	size_t numMisplacedItemsLeft   = 0;
	size_t numMisplacedItemsRight  = 0;

	counter_start[numThreads] = N;
	counter_left[numThreads]  = 0;

	unsigned int mid = counter_left[0];
	for (unsigned int i=1;i<numThreads;i++)
	  {
#if defined(__MIC__)
	    prefetch<PFHINT_L1>(&counter_left[i+16]);
#endif
	    mid += counter_left[i];
	  }

	global_mid = mid;

#if defined(__MIC__)
	for (size_t i=0;i<(numThreads+15)/16;i++)
	  prefetch<PFHINT_L2>(&counter_start[i*16]);
#endif

	const Range globalLeft (0,global_mid-1);
	const Range globalRight(global_mid,N-1);

	// without pragma the compiler makes a mess out of this loop
#pragma novector
	for (size_t i=0;i<numThreads;i++)
	  {	    
#if defined(__MIC__)
	    prefetch<PFHINT_L1>(&counter_start[i+16]);
#endif

	    const unsigned int left_start  = counter_start[i];
	    const unsigned int left_end    = counter_start[i] + counter_left[i]-1;
	    const unsigned int right_start = counter_start[i] + counter_left[i];
	    const unsigned int right_end   = counter_start[i+1]-1;

	    Range left_range (left_start,left_end); // counter[i].start,counter[i].start+counter[i].left-1);
	    Range right_range(right_start,right_end); // counter[i].start+counter[i].left,counter[i].start+counter[i].size-1);

	    Range left_misplaced = globalLeft.intersect(right_range);
	    Range right_misplaced = globalRight.intersect(left_range);

	    if (!left_misplaced.empty())  
	      {
		numMisplacedItemsLeft  += left_misplaced.size();
		leftMisplacedRanges[numMisplacedRangesLeft++] = left_misplaced;
#if defined(__MIC__)
		prefetch<PFHINT_L1EX>(&leftMisplacedRanges[numMisplacedRangesLeft+8]);
#endif

	      }

	    if (!right_misplaced.empty()) 
	      {
		numMisplacedItemsRight += right_misplaced.size();
		rightMisplacedRanges[numMisplacedRangesRight++] = right_misplaced;
#if defined(__MIC__)
		prefetch<PFHINT_L1EX>(&rightMisplacedRanges[numMisplacedRangesRight+8]);
#endif
	      }
	  }

	DBG_PART(
		 DBG_PRINT( global_mid );
		 DBG_PRINT( numMisplacedItemsLeft );
		 DBG_PRINT( numMisplacedItemsRight );
		 DBG_PRINT( numMisplacedRangesLeft );
		 DBG_PRINT( numMisplacedRangesRight );
		 );

	assert( numMisplacedItemsLeft == numMisplacedItemsRight );
	
	numMisplacedItems = numMisplacedItemsLeft;
      }

      void move_misplaced(const size_t threadID, const size_t numThreads) 
      {
	//const size_t numPartitionThreads = 64;

	if ( !numMisplacedItems ) return;

#if defined(__MIC__)
	for (size_t i=0;i<(numMisplacedRangesLeft+7)/8;i++)
	  prefetch<PFHINT_L2>(&leftMisplacedRanges[i*8]);
	for (size_t i=0;i<(numMisplacedRangesRight+7)/8;i++)
	  prefetch<PFHINT_L2>(&rightMisplacedRanges[i*8]);
#endif

	//size_t numTotalThreads = small ? numPartitionThreads : numThreads;
	const size_t numTotalThreads = numThreads;
	const size_t startID = (threadID+0)*numMisplacedItems/numTotalThreads;
	const size_t endID   = (threadID+1)*numMisplacedItems/numTotalThreads;
	swapItemsInMisplacedRanges(leftMisplacedRanges,
				   numMisplacedRangesLeft,
				   rightMisplacedRanges,
				   numMisplacedRangesRight,
				   startID,
				   endID);	    
      }

      static void task_thread_move_misplaced(void* data, const size_t threadID, const size_t numThreads) {

        parallel_partition_static<T,Compare,ThreadLocalPartition>* p = (parallel_partition_static<T,Compare,ThreadLocalPartition>*)data;

	p->move_misplaced(threadID,numThreads);
      } 

      /* main function for parallel in-place partitioning */
      size_t partition_parallel()
      {    
        LockStepTaskScheduler* scheduler = LockStepTaskScheduler::instance();
        const size_t numThreads = scheduler->getNumThreads();
    
        if (N <= 4 * numThreads)
          {
	    DBG_PRINT("SERIAL FALLBACK");
	    DBG_PART(
		     DBG_PRINT(numThreads);
		     );

            size_t mid = partition_serial(array,N);
            DBG_CHECK(
		      checkLeft(array,0,mid);
		      checkRight(array,mid,N);
                     );
            return mid;
          }

        DBG_PART2(
                 DBG_PRINT("PARALLEL MODE");
                 );
#define TIME_PHASES 0
#if TIME_PHASES == 1
	double t0 = getSeconds();
#endif
        scheduler->dispatchTask(task_thread_partition,this,0,numThreads);
#if TIME_PHASES == 1
	t0 = getSeconds() - t0;
	std::cout << std::endl << " phase0 = " << 1000.0f*t0 << "ms, perf = " << 1E-6*double(N)/t0 << " Mprim/s" << std::endl;
#endif
        /* ------------------------------------ */
        /* ------ parallel cleanup phase ------ */
        /* ------------------------------------ */
#if  TIME_PHASES == 1
	double t1 = getSeconds();
#endif
                

	computeMisplacedRanges(numThreads);
	
#if  TIME_PHASES == 1
	t1 = getSeconds() - t1;
	std::cout << " phase1 = " << 1000.0f*t1 << "ms, perf = " << 1E-6*double(N)/t1 << " Mprim/s" <<  " misplaced : " << numMisplacedItems << std::endl;
#endif

#if  TIME_PHASES == 1
	double t2 = getSeconds();
#endif
	scheduler->dispatchTask(task_thread_move_misplaced,this,0,numThreads);
#if  TIME_PHASES == 1
	t2 = getSeconds() - t2;
	std::cout << " phase2 = " << 1000.0f*t2 << "ms, perf = " << 1E-6*double(N)/t2 << " Mprim/s" << std::endl;
#endif
        DBG_CHECK(
		  checkLeft(array,0,global_mid);
		  checkRight(array,global_mid,N);
                 );
        
        return global_mid;
      }

      __forceinline void swapItemsInMisplacedRanges(const Range * const leftMisplacedRanges,
						    const size_t numLeftMisplacedRanges,
						    const Range * const rightMisplacedRanges,
						    const size_t numRightMisplacedRanges,
						    const size_t startID,
						    const size_t endID)
      {
	const size_t size = endID - startID;

	size_t leftLocalIndex  = startID;
	size_t rightLocalIndex = startID;

	const size_t leftRangeIndex  = findStartRange(leftLocalIndex,leftMisplacedRanges,numLeftMisplacedRanges);
	const size_t rightRangeIndex = findStartRange(rightLocalIndex,rightMisplacedRanges,numRightMisplacedRanges);

	const Range* l_range = &leftMisplacedRanges[leftRangeIndex];
	const Range* r_range = &rightMisplacedRanges[rightRangeIndex];

	for (size_t i=0;i<size;)
	  {
	    if (unlikely(leftLocalIndex) >= l_range->size())
	      {
		leftLocalIndex = 0;
		l_range++;
	      }

	    if (unlikely(rightLocalIndex) >= r_range->size())
	      {
		rightLocalIndex = 0;
		r_range++;		
	      }

	    const size_t l_size = l_range->size()-leftLocalIndex;
	    const size_t r_size = r_range->size()-rightLocalIndex;
	    const size_t lr_size = min(l_size,r_size);
	    
	    const size_t leftGlobalIndex  = l_range->start + leftLocalIndex;
	    const size_t rightGlobalIndex = r_range->start + rightLocalIndex;

	    const size_t items = min(lr_size,size-i);
	    T *__restrict__ const l = &array[leftGlobalIndex];
	    T *__restrict__ const r = &array[rightGlobalIndex];

#pragma nounroll
	    for (size_t j=0;j<items;j++)
	      {
#if defined(__MIC__)
		prefetch<PFHINT_L1EX>(((char*)&l[j]) + 4*64);
		prefetch<PFHINT_L1EX>(((char*)&r[j]) + 4*64);	  
#endif
		assert( !cmp(l[j]) );
		assert(  cmp(r[j]) );
		xchg(l[j],r[j]);
	      }
	    leftLocalIndex += items;
	    rightLocalIndex += items;
	    i += items;
	  }
      }
						    

    };

  template<typename T, typename Compare, typename ThreadLocalPartition>
    __forceinline size_t parallel_in_place_partitioning_static(T *array, 
							       size_t N, 
							       const Compare& cmp,
							       const ThreadLocalPartition& threadLocalPartition)
  {
    parallel_partition_static<T,Compare,ThreadLocalPartition> p(array,N,cmp,threadLocalPartition);
    return p.partition_parallel();    
  }


 };
