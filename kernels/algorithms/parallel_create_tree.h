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

#include "parallel_continue.h"

namespace embree
{
  template<size_t threshold1, size_t threshold2, typename CreateThreadAllocator, typename Continuation, typename Recurse1, typename Recurse2>
    void parallel_create_tree(const Continuation& br, CreateThreadAllocator& createAlloc, const Recurse1& recurseParallel, const Recurse2& recurseSequential)
  {
    typedef decltype(createAlloc()) Allocator;
    
    Allocator alloc = createAlloc();
    
    LockStepTaskScheduler* scheduler = LockStepTaskScheduler::instance();
    const size_t threadCount = scheduler->getNumThreads();
    
    vector_t<Continuation > heap;
    heap.push_back(br);
    
    class Push : public ParallelContinue<Continuation> {
    public:
      vector_t<Continuation >& heap;
      __forceinline Push(vector_t<Continuation >& heap) : heap(heap) {}
      void operator() (const Continuation& br) { 
        heap.push_back(br);
        std::push_heap(heap.begin(),heap.end());
      }
    } push(heap);
    
    /* work in multithreaded toplevel mode until sufficient subtasks got generated */
    while (heap.size() < 2*threadCount)
    {
      Continuation br;
      
      /* terminate if heap got empty */
      if (heap.size() == 0) 
        break;
      
      /* pop largest item for better load balancing */
      br = heap.front();
      std::pop_heap(heap.begin(),heap.end());
      heap.pop_back();
      
      /* guarantees to create no leaves in this stage */
      if (br.size() <= threshold1) { // FIXME: max(minLeafSize,threshold)
        push(br);
        break;
      }
      
      recurseParallel(br,alloc,push);
    }
    _mm_sfence(); // make written leaves globally visible
    
    std::sort(heap.begin(),heap.end(),typename Continuation::Greater());
    
    parallel_continue<threshold2>( heap.begin(), heap.size(), [&](const Continuation& br, Allocator& alloc, ParallelContinue<Continuation >& cont) {
        recurseSequential(br,alloc,cont);
      },createAlloc);
  };
  
  template<size_t threshold1, size_t threshold2, typename CreateThreadAllocator, typename Continuation, typename Recurse>
    void parallel_create_tree(const Continuation& br, CreateThreadAllocator& createAlloc, const Recurse& recurse)
  {
    parallel_create_tree<threshold1,threshold2>(br,createAlloc,recurse,recurse);
  }

  template<typename CreateThreadAllocator, typename Continuation, typename Recurse>
    void sequential_create_tree(const Continuation& br, CreateThreadAllocator& createAlloc, const Recurse& recurse)
  {
    typedef decltype(createAlloc()) Allocator;
    
    struct Recursion : public ParallelContinue<Continuation> { 
      const Recurse& recurse;
      Allocator& alloc;
      __forceinline Recursion(const Recurse& recurse, Allocator& alloc) : recurse(recurse), alloc(alloc) {}
      __forceinline void operator() (const Continuation& br) { recurse(br,alloc,*this); } 
    };

    Allocator alloc = createAlloc();
    Recursion recursion(recurse,alloc); 
    recursion(br);
  }
}
