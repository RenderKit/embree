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
  template<typename T, size_t BLOCK_SIZE>
  class __aligned(64) parallel_partition
    {
    private:

      static const size_t SERIAL_THRESHOLD = 16;

      size_t N;
      size_t blocks;
      T task_pivot;
      T* array;
     

      AlignedAtomicCounter64 blockID;

      AlignedAtomicCounter64 numLeftRemainderBlocks;
      AlignedAtomicCounter64 numRightRemainderBlocks;
      AlignedAtomicCounter32 maxLeftBlockID;
      AlignedAtomicCounter32 maxRightBlockID;
      
      unsigned int  leftRemainderBlockIDs[MAX_MIC_THREADS];
      unsigned int rightRemainderBlockIDs[MAX_MIC_THREADS];


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
      size_t serialPartitioning(const size_t begin, const size_t end, const T pivot)
      {
        T* l = array + begin;
        T* r = array + end - 1;

        while(1)
          {
            while (likely(l <= r && *l < pivot)) 
              ++l;
            while (likely(l <= r && *r >= pivot)) 
              --r;
            if (r<l) break;

            std::swap(*l,*r);
            l++; r--;
          }
      
        return l - array;        
      }

      /* neutralize left and right block */
      size_t neutralizeBlocks(size_t &left_begin,
                               const size_t &left_end,
                               size_t &right_begin,
                               const size_t &right_end,
                               const T pivot)
      {
        while(left_begin < left_end && right_begin < right_end)
          {
            while(array[left_begin] < pivot)
              {
                left_begin++;
                if (left_begin >= left_end) break;
              }

            while(array[right_begin] >= pivot)
              {
                right_begin++;
                if (right_begin >= right_end) break;
              }

            if (unlikely(left_begin == left_end || right_begin == right_end)) break;

            std::swap(array[left_begin++],array[right_begin++]);
          }

        size_t mode = 0;
        if (unlikely(left_begin == left_end))
          mode |= NEED_LEFT_BLOCK;

        if (unlikely(right_begin == right_end))
          mode |= NEED_RIGHT_BLOCK;

        assert(mode != 0);
        return mode;
      }

      /* check left part of array */
      void checkLeft(const size_t begin, const size_t end, const T pivot)
      {
        for (size_t i=begin;i<end;i++)
          if (array[i] >= pivot)
            {
              DBG_PRINT(i);
              DBG_PRINT(array[i]);
              DBG_PRINT(pivot);
              FATAL("partition error on left side");
            }
      }

      /* check right part of array */
      void checkRight(const size_t begin, const size_t end, const T pivot)
      {
        for (size_t i=begin;i<end;i++)
          if (array[i] < pivot)
            {
              DBG_PRINT(i);
              DBG_PRINT(array[i]);
              DBG_PRINT(pivot);
              FATAL("partition error on right side");
            }
      }

    public:

      /* initialize atomic counters */
    parallel_partition(T *array, size_t N) : array(array), N(N)
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
      void thread_partition(const T pivot)
      {
        size_t mode = NEED_LEFT_BLOCK | NEED_RIGHT_BLOCK;
        
        size_t left_begin  = (size_t)-1;
        size_t left_end    = (size_t)-1;
        size_t right_begin = (size_t)-1;
        size_t right_end   = (size_t)-1;

        size_t currentLeftBlock  = (size_t)-1;
        size_t currentRightBlock = (size_t)-1;

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

            mode = neutralizeBlocks(left_begin,left_end,right_begin,right_end,pivot);
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
        parallel_partition<T,BLOCK_SIZE>* p = (parallel_partition<T,BLOCK_SIZE>*)data;
        p->thread_partition(p->task_pivot);
      } 

      /* main function for parallel in-place partitioning */
      size_t parition(const T pivot)
      {

        if (N <= SERIAL_THRESHOLD)
          {
            size_t mid = serialPartitioning(0,N,pivot);
            DBG_PART(
                     DBG_PRINT( mid );
                     checkLeft(0,mid,pivot);
                     checkRight(mid,N,pivot);
                     );
            return mid;
          }

        DBG_PART2(
                 DBG_PRINT("PARALLEL MODE");
                 );

        LockStepTaskScheduler* scheduler = LockStepTaskScheduler::instance();
        const size_t numThreads = scheduler->getNumThreads();

        task_pivot = pivot;
        scheduler->dispatchTask(task_thread_partition,this,0,numThreads);

        //thread_partition(pivot);

        /* ---------------------------------- */
        /* ------ serial cleanup phase ------ */
        /* ---------------------------------- */
        
        /* sort remainder blocks */
        insertionsort_ascending<T>(leftRemainderBlockIDs,numLeftRemainderBlocks);
        insertionsort_ascending<T>(rightRemainderBlockIDs,numRightRemainderBlocks);

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
              {
                //DBG_PART2(std::cout << "SWAP " << index0 << " " << index1 << std::endl);
                swapTwoLeftBlocks(index0,index1);
              }
          }
        assert(numLeftRemainderBlocks-1 <= maxLeftBlockID);

        const size_t left_border_index = maxLeftBlockID-(numLeftRemainderBlocks-1);

        size_t left_begin = (size_t)-1;
        size_t left_end   = (size_t)-1;

        getLeftArrayIndex(left_border_index,left_begin,left_end);

        DBG_CHECK( checkLeft(0,left_begin,pivot) );

        /* compact right remaining blocks */


        for (size_t i=0;i<numRightRemainderBlocks;i++)
          {
            assert(i<=maxRightBlockID);
            const unsigned int index0 = rightRemainderBlockIDs[numRightRemainderBlocks-1-i];
            const unsigned int index1 = maxRightBlockID-i;
            if (index0 != index1)
              {
                //DBG_PART2(std::cout << "SWAP " << index0 << " " << index1 << std::endl);
                swapTwoRightBlocks(index0,index1);
              }
          }
        assert(numRightRemainderBlocks-1 <= maxRightBlockID);

        const size_t right_border_index = maxRightBlockID-(numRightRemainderBlocks-1);

        size_t right_begin = (size_t)-1;
        size_t right_end   = (size_t)-1;

        getRightArrayIndex(right_border_index,right_begin,right_end);

        DBG_CHECK( checkRight(right_end,N,pivot) );

        /* size_t left_begin = (size_t)-1; */
        /* size_t left_end   = (size_t)-1; */

        /* getLeftArrayIndex(maxLeftBlockID,left_begin,left_end); */
        
        /* if (numLeftRemainderBlocks) */
        /*   getLeftArrayIndex(leftRemainderBlockIDs[0],left_begin,left_end); */
        /* else */
        /*   left_begin += BLOCK_SIZE; */

        /* size_t right_begin = (size_t)-1; */
        /* size_t right_end   = (size_t)-1; */

        /* getRightArrayIndex(maxRightBlockID,right_begin,right_end); */
        
        /* if (numRightRemainderBlocks) */
        /*   getRightArrayIndex(rightRemainderBlockIDs[0],right_begin,right_end); */
        /* else */
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
                 assert(left_begin != (size_t)-1);
                 assert(right_end  != (size_t)-1);
                 
                 checkLeft(0,left_begin,pivot);
                 checkRight(right_end,N,pivot);
                 );


        
        DBG_CHECK(
                  DBG_PRINT(right_end - left_begin);
                  assert( right_end - left_begin <= numThreads*3*BLOCK_SIZE);
                  )
          
        const size_t mid = serialPartitioning(left_begin,right_end,pivot);

        
        DBG_CHECK(
                 checkLeft(0,mid,pivot);
                 checkRight(mid,N,pivot);
                 );
        
        return mid;
      }


      size_t partition_serial(const T pivot)
      {
        return serialPartitioning(0,N,pivot);      
      }

    };

};
