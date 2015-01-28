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
#include "../../kernels/algorithms/range.h"

#include <thread>
#include <mutex>
#include <condition_variable>

namespace embree
{
  struct TaskSchedulerNew
  {
    struct Thread;
    
    /*! virtual interface for all tasks */
    struct TaskFunction {
      virtual void execute() = 0;
    };

    /*! builds a task interface from a closure */
    template<typename Closure>
    struct ClosureTaskFunction : public TaskFunction
    {
    public:
      __forceinline ClosureTaskFunction (const Closure& closure)
        : closure(closure) {}
      
      void execute() {
        closure();
      };

    public:
      Closure closure;
    };
    
    struct __aligned(64) Task 
    {
      /*! states a task can be in */
      enum { DONE, INITIALIZED };

      /*! switch from one state to another */
      __forceinline void switch_state(int from, int to) 
      {
	__memory_barrier();
	bool success = atomic_cmpxchg(&state,from,to) == from;
	assert(success);
      }

      /*! try to switch from one state to another */
      __forceinline bool try_switch_state(int from, int to) {
	__memory_barrier();
	return atomic_cmpxchg(&state,from,to) == from;
      }

       /*! increment/decrement dependency counter */
      void add_dependencies(int n) { 
	atomic_add(&dependencies,n); 
      }

      /*! initialize all tasks to DONE state by default */
      __forceinline Task()
	: state(DONE) {} 

      /*! construction of new task */
      __forceinline Task (TaskFunction* closure, Task* parent, size_t stackPtr) 
        : closure(closure), parent(parent), stackPtr(stackPtr), dependencies(1) 
      {
        if (parent) parent->add_dependencies(+1);
	switch_state(DONE,INITIALIZED);
      }

      /*! construction of stolen task, stealing thread will decrement initial dependency */
      __forceinline Task (TaskFunction* closure, Task* parent) 
        : closure(closure), parent(parent), stackPtr(-1), dependencies(1) 
      {
	switch_state(DONE,INITIALIZED);
      }

      /*! try to steal this task */
      bool try_steal(Task& child)
      {
	if (!try_switch_state(INITIALIZED,DONE)) return false;
	new (&child) Task(closure, this);
        return true;
      } 
      
      /*! run this task */
      void run (Thread& thread) 
      {
	/* try to run if not already stolen */
	if (try_switch_state(INITIALIZED,DONE))
	{
	  Task* prevTask = thread.task; 
          thread.task = this;
          closure->execute();
          thread.task = prevTask;
	  add_dependencies(-1);
	}
        
	/* steal until all dependencies have completed */
        while (dependencies) {
          if (thread.scheduler->schedule_steal(thread))
            while (thread.tasks.execute_local(thread,this));
        }

	/* now signal our parent task that we are finished */
        if (parent) 
	  parent->add_dependencies(-1);
      }

    public:
      volatile atomic32_t state;         //!< state this task is in
      volatile atomic32_t dependencies;  //!< dependencies to wait for
      TaskFunction* closure;             //!< the closure to execute
      Task* parent;                      //!< parent task to signal when we are finished
      size_t stackPtr;                   //!< stack location where closure is stored
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
      __forceinline void push_right(Thread& thread, const Closure& closure) 
      {
        assert(right < SIZE);
        size_t oldStackPtr = stackPtr;
        TaskFunction* func = new (alloc(sizeof(Closure))) ClosureTaskFunction<Closure>(closure);
        new (&tasks[right++]) Task(func,thread.task,oldStackPtr);

	/* also move left pointer */
	if (left >= right-1) left = right-1;
      }
      
      bool execute_local(Thread& thread, Task* parent)
      {
	/* stop if we run out of local tasks or reach the waiting task */
        if (right == 0 || &tasks[right-1] == parent)
          return false;

	/* execute task */
        tasks[right-1].run(thread);

	/* pop task and closure from stack */
	right--;
        if (tasks[right].stackPtr != -1)
          stackPtr = tasks[right].stackPtr;

	/* also move left pointer */
	if (left >= right) left = right;

	return right != 0;
      }

      bool steal(Thread& thread) 
      {
	size_t l = left;
	if (l < right) l = atomic_add(&left,1);
	
        if (!tasks[l].try_steal(thread.tasks.tasks[thread.tasks.right]))
          return false;
        
        thread.tasks.right++;
        return true;
      }
    };
    
    struct Thread 
    {
      static const size_t MAX_STACK_SIZE = 64*1024;
      
      Thread (size_t threadIndex, TaskSchedulerNew* scheduler)
      : threadIndex(threadIndex), scheduler(scheduler), task(NULL) {}
      
      size_t threadIndex;
      TaskQueue tasks;
      Task* task;
      TaskSchedulerNew* scheduler;
    };
    
    static const size_t MAX_THREADS = 1024;
    
    TaskSchedulerNew (size_t numThreads = 0);
    ~TaskSchedulerNew ();

    void schedule(size_t threadIndex);
    void schedule_on_thread(Thread& thread);
    bool schedule_steal(Thread& thread);

    /* spawn a new task at the top of the threads task stack */
    template<typename Closure>
    static __forceinline void spawn(const Closure& closure) 
    {
      Thread* thread = TaskSchedulerNew::thread();
      thread->tasks.push_right(*thread,closure);
    }

    /* spawn a new task at the top of the threads task stack */
    template<typename Closure>
    __forceinline void spawn_root(const Closure& closure) 
    {
      /* allocate thread structure */
      Thread thread(0,this);
      threadLocal[0] = &thread;
      thread_local_thread = &thread;
      spawn(closure);
      atomic_add(&anyTasksRunning,+1);
      condition.notify_all();
      schedule_on_thread(thread);
    }

    /* work on spawned subtasks and wait until all have finished */
    static __forceinline void wait() 
    {
      Thread* thread = TaskSchedulerNew::thread();
      while (thread->tasks.execute_local(*thread,thread->task)) {};
    }

    std::vector<std::thread> threads;
    Thread* threadLocal[MAX_THREADS];
    volatile atomic_t numThreads;
    volatile bool terminate;
    volatile atomic_t anyTasksRunning;
    
    std::mutex mutex;        
    std::condition_variable condition;
    volatile atomic_t numThreadsRunning;

    static __thread Thread* thread_local_thread;
    static Thread* thread();
  };
};

