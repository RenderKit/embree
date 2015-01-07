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

#include "range.h"
#include "builders/workstack.h"

namespace embree
{
  template<typename Continuation, typename Index, typename Func>
    class ParallelContinueTask
  {
    static const size_t SIZE_WORK_STACK = 64;

  public:
    __forceinline ParallelContinueTask (Continuation* continations, const Index taskCount, const Func& func)
      : continuations(continuations), cntr(0), taskCount(taskCount), func(func)
    {
      LockStepTaskScheduler* scheduler = LockStepTaskScheduler::instance();
      size_t threadCount = scheduler->getNumThreads();
      threadStack = new WorkStack<BuildRecord<NodeRef>,SIZE_WORK_STACK>[threadCount]; 
      scheduler->dispatchTask(_task,this);
      delete[] threadStack;
    }

    void task(const size_t threadIndex, const size_t threadCount) 
    {
      struct Recurse 
      {
        Func& func;
        __forceinline Recurse(Func& func) : func(func) {}
        __forceinline void operator() (Continuation& c) { func(c,*this); } 
      };
    
      struct Select 
      {
        Func& func;
        __forceinline Select(Func& func) : func(func) {}
        __forceinline void operator() (Continuation& c) 
        {
          if (c.final() || !state->threadStack[threadID].push(c)) {
            Recurse r(func); r(c); 
          }
          else func(c,*this);
        }
      };
      
      while (true) 
      {
        Continuation cont;
        Index i = atomic_add(&cntr,1);
        if (i <= taskCount) 
          cont = continuations[taskIndex];
        
        /* global work queue empty => try to steal from neighboring queues */	 
        else
        {
          bool success = false;
          for (size_t i=0; i<threadCount; i++)
          {
            if (state->threadStack[(threadIndex+i)%threadCount].pop(cont)) {
              success = true;
              break;
            }
          }
          /* found nothing to steal ? */
          if (!success) return; // FIXME: may loose threads
        }
        
        Select select(func); func(cont,select);
	while (state->threadStack[threadIndex].pop(cont)) {
          Select select(func); func(cont,select);
        }
      }
    }

    static void _task(void* data, const size_t threadIndex, const size_t threadCount) {
      ((ParallelContinueTask*)data)->task(threadIndex,threadCount);
    }
    
  private:
    Continuation* continations;
    atomic_t cntr;
    Index taskCount;
    const Func& func;
  private:
    __aligned(64) WorkStack<Continuation,SIZE_WORK_STACK>* threadStack;
  };
  
  /* parallel continue */
  template<typename Continuation, typename Index, typename Func>
    __forceinline void parallel_continue( Continuation* continations, const Index N, const Func& func)
  {
    ParallelContinueTask task(continations,N,func);
  }
}
