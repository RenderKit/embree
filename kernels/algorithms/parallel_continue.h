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

#include "range.h"
#include "builders/workstack.h"
#include "parallel_for.h"

namespace embree
{
  template<typename Continuation>
  class ParallelContinue {
  public:
    virtual void operator() (const Continuation& c) = 0;
  };

#if TASKING_TBB || TASKING_TBB_INTERNAL

  template<typename Continuation, typename Func, typename ThreadLocal>
    struct TBBRecurse : public ParallelContinue<Continuation>
  {
    const Func& func;
    ThreadLocal& threadLocal;
    __forceinline TBBRecurse(const Func& func, ThreadLocal& threadLocal) 
      : func(func), threadLocal(threadLocal) {}
    void operator() (const Continuation& c) { 
      func(c,threadLocal,*this); 
    } 
  };
  
  /* parallel continue */
  template<size_t threshold, typename Continuation, typename Index, typename Func, typename CreateThreadLocal>
    __forceinline void parallel_continue( Continuation* continuations, const Index N, const Func& func, const CreateThreadLocal& createThreadLocal)
  {
    parallel_for(N,[&] (const Index i) 
    {
	auto threadLocal = createThreadLocal();
	TBBRecurse<Continuation,Func,decltype(threadLocal)> recurse(func,threadLocal);
	recurse(continuations[i]);
      });
  }

#endif

#if TASKING_LOCKSTEP

  template<size_t threshold, typename Continuation, typename Index, typename Func, typename ThreadLocal, typename CreateThreadLocal>
    class ParallelContinueTask
  {
    static const size_t SIZE_WORK_STACK = 64;

  public:
    __forceinline ParallelContinueTask (Continuation* continuations, const Index taskCount, const Func& func, const CreateThreadLocal& createThreadLocal)
      : continuations(continuations), cntr(0), taskCount(taskCount), func(func), createThreadLocal(createThreadLocal)
    {
      LockStepTaskScheduler* scheduler = LockStepTaskScheduler::instance();
      size_t threadCount = scheduler->getNumThreads();
      threadStack = new WorkStack<Continuation,SIZE_WORK_STACK>[threadCount]; 
      scheduler->dispatchTask(_task,this);
      delete[] threadStack;
    }

    void task(const size_t threadIndex, const size_t threadCount) 
    {
      struct Recurse : public ParallelContinue<Continuation>
      {
        ParallelContinueTask* parent;
        ThreadLocal& threadLocal;
        __forceinline Recurse(ParallelContinueTask* parent, ThreadLocal& threadLocal) 
          : parent(parent), threadLocal(threadLocal) {}
        void operator() (const Continuation& c) { parent->func(c,threadLocal,*this); } 
      };
    
      struct Split : public ParallelContinue<Continuation>
      {
        ParallelContinueTask* parent;
        ThreadLocal& threadLocal;
        __forceinline Split(ParallelContinueTask* parent, ThreadLocal& threadLocal) 
          : parent(parent), threadLocal(threadLocal) {}
        void operator() (const Continuation& c) 
        {
          const size_t threadIndex = LockStepTaskScheduler::threadIndex();
          if (parent->threadStack[threadIndex].push(c)) return;
          Recurse r(parent,threadLocal); r(c); // fallback if push was not possible
        }
      };

      ThreadLocal threadLocal = createThreadLocal();

      while (true) 
      {
        Continuation cont;
        Index taskIndex = atomic_add(&cntr,1);
        if (taskIndex < taskCount) 
          cont = continuations[taskIndex];

        /* global work queue empty => try to steal from neighboring queues */	 
        else
        {
          bool success = false;
          for (size_t i=0; i<threadCount; i++)
          {
            if (threadStack[(threadIndex+i)%threadCount].pop(cont)) {
              success = true;
              break;
            }
          }
          /* found nothing to steal ? */
          if (!success) return; // FIXME: may loose threads
        }

        Recurse recurse(this,threadLocal); Split split(this,threadLocal); 
        func(cont,threadLocal,cont.size() < threshold ? (ParallelContinue<Continuation>&)recurse : (ParallelContinue<Continuation>&)split);
	while (threadStack[threadIndex].pop(cont)) {
          func(cont,threadLocal,cont.size() < threshold ? (ParallelContinue<Continuation>&)recurse : (ParallelContinue<Continuation>&)split);
        }
      }
    }

    static void _task(void* data, const size_t threadIndex, const size_t threadCount) {
      ((ParallelContinueTask*)data)->task(threadIndex,threadCount);
    }
    
  private:
    Continuation* continuations;
    atomic_t cntr;
    Index taskCount;
    const Func& func;
    const CreateThreadLocal& createThreadLocal;
  private:
    __aligned(64) WorkStack<Continuation,SIZE_WORK_STACK>* threadStack;
  };
  
  /* parallel continue */
  template<size_t threshold, typename Continuation, typename Index, typename Func, typename CreateThreadLocal>
    __forceinline void parallel_continue( Continuation* continuations, const Index N, const Func& func, const CreateThreadLocal& createThreadLocal)
  {
    ParallelContinueTask<threshold,Continuation,Index,Func,decltype(createThreadLocal()),CreateThreadLocal> task(continuations,N,func,createThreadLocal);
  }
#endif
}
