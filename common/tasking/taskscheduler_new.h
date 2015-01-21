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
#include <thread>

namespace embree
{
  struct TaskSchedulerNew
  {
    struct TaskFunction {
      virtual void execute(size_t begin, size_t end) = 0;
    };

    template<typename Closure>
    struct ClosureFunction : public TaskFunction
    {
      Closure closure;
      
      __forceinline ClosureFunction (const Closure& closure)
        : closure(closure) {}
      
      void execute(size_t begin, size_t end) {
        closure(begin,end);
      };
    };
    
    struct __aligned(64) Task 
    {
      __forceinline Task()
        : valid(false) {}

      __forceinline Task (TaskFunction* closure, Task* parent, size_t oldStackPtr, size_t begin, size_t end, size_t block) 
        : closure(closure), parent(parent), oldStackPtr(oldStackPtr), begin(begin), end(end), block(block), dependencies(1) 
      {
        validate(true);
        if (parent) parent->addDependency();
      }

      __forceinline void validate(bool valid0) {
        __memory_barrier();
        valid = valid0;
      }

      __forceinline size_t size() const {
        return end-begin;
      }

      __forceinline bool isLeaf() const {
        return size() <= block;
      }
      
      bool split(Task& child)
      {
        Lock<AtomicMutex> lock(mutex);
        if (isLeaf()) return false;
        size_t center = (begin+end)/2;
        new (&child) Task(closure, this, -1, center, end, block);
        end = center;
        atomic_add(&dependencies,1);
        return true;
      }

      int steal(Task& child)
      {
        if (!mutex.tryLock())
          return 0;
        
        if (!valid) {
          mutex.unlock();
          return 0;
        }

        if (isLeaf()) { 
          new (&child) Task(closure, this, -1, begin, end, block);
          begin = end = 0;
          atomic_add(&dependencies,1);
          mutex.unlock();
          return 1;
        } 
        
        else {
          size_t center = (begin+end)/2;
          new (&child) Task(closure, this, -1, center, end, block);
          end = center;
          atomic_add(&dependencies,1);
          mutex.unlock();
          return 2;
        }
      }

      void addDependency      () { atomic_add(&dependencies,+1); }
      void removeDependency   () { atomic_add(&dependencies,-1); }
      void waitForDependencies() { while (dependencies) __pause_cpu(); }
      
      void run () 
      {
        mutex.lock();
        validate(false);
        if (size()) {
          Task* prevTask = TaskSchedulerNew::thread_local_task;
          TaskSchedulerNew::thread_local_task = this;
          closure->execute(begin,end);
          TaskSchedulerNew::thread_local_task = prevTask;
        }
        mutex.unlock();
        removeDependency();
        waitForDependencies(); // FIXME: maybe steal here
        if (parent) parent->removeDependency();
      }

    public:
      volatile bool valid;
      TaskFunction* closure;
      AtomicMutex mutex;
      Task* parent;
      volatile size_t begin, end, block;
      size_t oldStackPtr;
      volatile atomic_t dependencies;
    };
    
    struct TaskQueue
    {
      static const size_t SIZE = 1024;
      static const size_t MAX_STACK_SIZE = 256*1024;
      Task tasks[SIZE];
      volatile atomic_t left, right;
      
      __aligned(64) char stack[MAX_STACK_SIZE];
      size_t stackPtr;
      
      TaskQueue ()
      : left(0), right(0), stackPtr(0) {}
      
      __forceinline void* alloc(size_t bytes, size_t align = 64) {
        stackPtr += bytes + ((align - stackPtr) & (align-1));
        assert(stackPtr <= MAX_STACK_SIZE);
        return &stack[stackPtr-bytes];
      }
      
      template<typename Closure>
      __forceinline void push_right(const Closure& closure, size_t begin, size_t end, size_t block) 
      {
        assert(right < SIZE);
        size_t oldStackPtr = stackPtr;
        Task* parent = TaskSchedulerNew::task();
        TaskFunction* func = new (alloc(sizeof(Closure))) ClosureFunction<Closure>(closure);
        new (&tasks[right++]) Task(func,parent,oldStackPtr,begin,end,block);
      }
      
      bool execute_local()
      {
        if (right == 0)
          return false;

        if (&tasks[right-1] == TaskSchedulerNew::thread_local_task)
          return false;
        
        while (tasks[right-1].split(tasks[right])) {
          right++;
        }
        tasks[right-1].run();
        if (tasks[right-1].oldStackPtr != -1)
          stackPtr = tasks[right-1].oldStackPtr;
        right--;
        if (right == 0) {
          left = 0;
          return false;
        }

        if (left >= right) left = right;
        return true;
      }

      bool steal(volatile atomic_t* anyTasksRunning)
      {
        const size_t l = left;
        if (!tasks[l].valid) return false;
        Thread* thread = TaskSchedulerNew::thread();
        Task& dst = thread->tasks.tasks[thread->tasks.right];
        int op = tasks[l].steal(dst);
        if (op == 0) return false;
        if (op == 1) atomic_add(&left,1);
        if (anyTasksRunning) atomic_add(anyTasksRunning,1);
        thread->tasks.right++;
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
      TaskSchedulerNew* scheduler;
    };
    
    static const size_t MAX_THREADS = 1024;
    
    TaskSchedulerNew (size_t numThreads = 0);
    ~TaskSchedulerNew ();

    void schedule(size_t threadIndex);
    void schedule_on_thread(Thread& thread);

    /* spawn a new task at the top of the threads task stack */
    template<typename Closure>
    static __forceinline void spawn(const Closure& closure, const size_t begin = 0, const size_t end = 1, const size_t block = 1) 
    {
      Thread* thread = TaskSchedulerNew::thread();
      thread->tasks.push_right(closure,begin,end,block);
    }

    /* spawn a new task at the top of the threads task stack */
    template<typename Closure>
    __forceinline void spawn_root(const Closure& closure, const size_t begin = 0, const size_t end = 1, const size_t block = 1) 
    {
      /* allocate thread structure */
      Thread thread(0,this);
      threadLocal[0] = &thread;
      thread_local_thread = &thread;
      spawn(closure,begin,end,block);
      atomic_add(&anyTasksRunning,+1);
      condition.notify_all();
      schedule_on_thread(thread);
    }

    /* work on spawned subtasks and wait until all have finished */
    static __forceinline void wait() 
    {
      Thread* thread = TaskSchedulerNew::thread();
      while (thread->tasks.execute_local()) {
      };
    }

    std::vector<std::thread> threads;
    Thread* threadLocal[MAX_THREADS];
    volatile atomic_t numThreads;
    volatile bool terminate;
    volatile atomic_t anyTasksRunning;
    
    std::mutex mutex;        
    std::condition_variable condition;

    static __thread Thread* thread_local_thread;
    static Thread* thread();
    
    static __thread Task* thread_local_task;
    static Task* task();
  };
};

