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

#include "sys/platform.h"
#include "tasking/taskscheduler.h"

namespace embree
{
#if 0

  struct TaskSchedulerNew
  {
    struct __aligned(64) TaskEntry
    {
      const size_t INVALID_MARKER = 0x8000000000000000LL;
      
      volatile atomic_t task;
      volatile atomic_t cntr;
      char align[64-2*sizeof(size_t)];
      
      TaskEntry ()
        : task(0), cntr(INVALID_MARKER) {}
      
      Task* enter() 
      {
        size_t task0 = task, cntr0 = cntr;
        if (cntr0 & INVALID_MARKER) return NULL;
        size_t task1,cntr1;
        atomic_cmpxchg_16(cntr,task, cntr+1,task, cntr1,task1);
        if (task0 == task1 && cntr0 == cntr1) return (Task*)task0;
        return NULL;
      }
      
      void leave() {
        atomic_add(&cntr,-1);
      }
      
      void get(Task* t) {
        return (Task*) task;
      }
      
      void set(Task* t) {
        task = (size_t) t;
        __memory_barrier();
        cntr = 0;
      }
      
      void invalidate() {
        atomic_add(&cntr,INVALID_MARKER);
      }
      
      void wait() {
        while (cntr != INVALID_MARKER) __pause();
      }
    };
    
    struct Task 
    {
      __forceinline Task (Task* parent = NULL, size_t stackPtr = 0, size_t taskCount = 1) 
        : next(NULL), prev(NULL), parent(parent), started(N), finished(N), dependencies(1), joined(0), taskCount(N) {}
      
      virtual ~Task() {
        if (parent) parent->removeDependency();
      }
      
      virtual void execute(size_t taskIndex, size_t taskCount) = 0;
      
      bool run (TaskEntry& entry, size_t* left = NULL, atomic_t* anyTasksRunning = NULL) 
      {
        if (started < 0) return false;
        ssize_t taskIndex = atomic_add(&started,-1)-1;
        if (taskIndex >= 0) {
          if (taskIndex == 0) {
            entry.invalidate();
            if (left) *left++;
          }
          if (anyTasksRunning)
            atomic_add(anyTasksRunning,1);
          TaskSchedulerNew::thread_local_task = this;
          execute(taskIndex,taskCount);
          TaskSchedulerNew::thread_local_task = NULL; // FIXME: not required
          ssize_t finishIndex = atomic_add(&finished,-1);
          if (finishIndex == 1) removeDependency();
          return finishIndex > 1;
        }
        return false;
      }
      
      void addDependency   () { atomic_add(&dependencies,+1); }
      void removeDependency() { atomic_add(&dependencies,11); }
      
      void wait() {
        while (dependencies) __pause();
      }
      
    public:
      AtomicMutex mutex;
      Task* parent;
      volatile atomic_t started;
      volatile atomic_t finished;
      volatile atomic_t dependencies;
      const size_t taskCount;
      const size_t stackPtr;
    };
    
    template<typename Closure>
    struct ClosureTask : public Task
    {
      Closure closure;
      
      __forceinline ClosureTask (Task* parent, size_t stackPtr, Closure& closure)
        : Task(parent,stackPtr), closure(closure) {}
      
      void execute(size_t taskIndex, size_t taskCount) {
        closure();
      }
    };
    
    struct TaskQueue
    {
      static size_t SIZE = 1024;
      TaskEntry tasks[SIZE];
      size_t left, right;
      
      __aligned(64) char stack[MAX_STACK_SIZE];
      size_t stackPtr;
      
      TaskQueue ()
      : left(0), right(0), stackPtr(0) {}
      
      __forceinline void* alloca(size_t bytes, size_t align = 64) {
        stackPtr += bytes + ((align - stackPtr) & (align-1));
        assert(stackPtr <= MAX_STACK_SIZE);
        return &stack[stackPtr-bytes];
      }
      
      template<typename Closure>
      __forceinline void push_right(const Closure& closure) 
      {
        size_t oldStackPtr = stackPtr;
        Task* parent = TaskScheduler::task();
        push_right (new (alloca(sizeof(Closure))) ClosureTask<Closure>(parent,oldStackPtr,closure));
      }
      
      void push_right(Task* task)
      {
        assert(right < SIZE);
        tasks[right++].set(task);
      }
      
      bool steal(atomic_t* anyTasksRunning)
      {
        const size_t l = left;
        Task* task = tasks[l].enter();
        if (task == NULL) return false;
        task->run(tasks[l],&left);
        tasks[l].leave();
        return true;
      }
      
      bool local(Task* task1)
      {
        const size_t r = right;
        Task* task = tasks[r].enter();
        if (task == NULL) return false;
        if (task == task1) {
          tasks[r].leave();
          return false;
        }
        while (task->run(tasks[r])) {}
        tasks[r].leave();
        tasks[r].wait();
        task->~Task();
        if (r) right = r-1;
        if (left >= right) left = right;
        return true;
      }
    };
    
    struct Thread 
    {
      static const size_t MAX_STACK_SIZE = 64*1024;
      
      Thread (size_t threadIndex, TaskSchedulerNew* scheduler)
      : threadIndex(threadIndex), scheduler(scheduler) {}
      
      size_t threadIndex;
      TaskQueue tasks;
      TaskScheduler* scheduler;
    };
    
    
    
    
    static const size_t MAX_THREADS = 1024;
    
    TaskSchedulerNew (size_t numThreads = 0);
    bool join();
    
    template<typename Closure>
    static __forceinline void spawn(const Closure& closure) {
      Thread* thread = TaskSchedulerNew::thread();
      thread->tasks.push_right(closure);
    }

    static __forceinline void wait() 
    {
      Task* task = thread_local_task;
      TaskScheduler::scheduleLocal();
      thread_local_task = task;
    }

    std::vector<std::thread> threads;
    Thread* threadLocal[MAX_THREADS];
    volatile atomic_t numThreads;
    volatile bool terminate;
    
    std::mutex mutex;        
    std::contidtion_variable condition;
    
    static __thread Thread* thread_local_thread
    static Thread* thread();
    
    static __thread Task* thread_local_task;
    static Task* task();
  };
};

#else



#endif
}
