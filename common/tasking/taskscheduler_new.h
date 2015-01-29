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
    static const size_t MAX_THREADS = 1024;
    static const size_t TASK_STACK_SIZE = 1024;
    static const size_t CLOSURE_STACK_SIZE = 256*1024;

    struct Thread;
    
    /*! virtual interface for all tasks */
    struct TaskFunction {
      virtual void execute() = 0;
    };

    /*! virtual interface for all task sets */
    struct TaskSetFunction {
      virtual void execute(const range<size_t>& range) = 0;
    };

    /*! builds a task interface from a closure */
    template<typename Closure>
    struct ClosureTaskFunction : public TaskFunction
    {
      Closure closure;
      __forceinline ClosureTaskFunction (const Closure& closure) : closure(closure) {}
      void execute() { closure(); };
    };

    /*! builds a task interface from a closure */
    template<typename Closure>
    struct ClosureTaskSetFunction : public TaskFunction
    {
      Closure closure;
      __forceinline ClosureTaskSetFunction (const Closure& closure) : closure(closure) {}
      void execute(const range<size_t>& range) { closure(range); };
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
          if (thread.scheduler->steal_from_other_threads(thread))
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
      TaskQueue ()
      : left(0), right(0), stackPtr(0) {}
      
      __forceinline void* alloc(size_t bytes, size_t align = 64) {
        stackPtr += bytes + ((align - stackPtr) & (align-1));
        assert(stackPtr <= CLOSURE_STACK_SIZE);
        return &stack[stackPtr-bytes];
      }
      
      template<typename Closure>
      __forceinline void push_right(Thread& thread, const Closure& closure) 
      {
        assert(right < TASK_STACK_SIZE);

	/* allocate new task on right side of stack */
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
	size_t oldRight = right;
        tasks[right-1].run(thread);
	if (right != oldRight) {
	  THROW_RUNTIME_ERROR("you have to wait for spawned subtasks");
	}

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
	if (l < right) 
	  l = atomic_add(&left,1);
	else 
	  return false;
	
        if (!tasks[l].try_steal(thread.tasks.tasks[thread.tasks.right]))
          return false;
        
        thread.tasks.right++;
        return true;
      }

    public:

      /* task stack */
      Task tasks[TASK_STACK_SIZE];
      __aligned(64) volatile atomic_t left;   //!< threads steal from left
      __aligned(64) volatile atomic_t right;           //!< new tasks are added to the right
      
      /* closure stack */
      __aligned(64) char stack[CLOSURE_STACK_SIZE];
      size_t stackPtr;
    };
    
    /*! thread local structure for each thread */
    struct Thread 
    {
      Thread (size_t threadIndex, TaskSchedulerNew* scheduler)
      : threadIndex(threadIndex), scheduler(scheduler), task(NULL) {}
      
      size_t threadIndex;              //!< ID of this thread
      TaskQueue tasks;                 //!< local task queue
      Task* task;                      //!< current active task
      TaskSchedulerNew* scheduler;     //!< pointer to task scheduler
    };
    
    TaskSchedulerNew (size_t numThreads = 0, bool spinning = false);
    ~TaskSchedulerNew ();
    
    static TaskSchedulerNew* g_instance;

    static void create(size_t numThreads);
    static void destroy();
    
    /*! lets new worker threads join the tasking system */
    void join();

    /*! wait for some number of threads available (threadCount includes main thread) */
    void wait_for_threads(size_t threadCount);

    void startThreads();
    void terminateThreadLoop();
    void destroyThreads();

    void thread_loop(size_t threadIndex);
    bool steal_from_other_threads(Thread& thread);

    /* spawn a new task at the top of the threads task stack */
    template<typename Closure>
    static __forceinline void spawn(const Closure& closure) 
    {
      Thread* thread = TaskSchedulerNew::thread();
      thread->tasks.push_right(*thread,closure);
    }

    /* spawn a new task set  */
    template<typename Closure>
    __forceinline void spawn(const size_t begin, const size_t end, const size_t blockSize, const Closure& closure) 
    {
      spawn([=,&closure]() {
	  if (end-begin <= blockSize) {
	    return closure(range<size_t>(begin,end));
	  }
	  const size_t center = (begin+end)/2;
	  spawn(begin,center,blockSize,closure);
	  spawn(center,end  ,blockSize,closure);
	  wait();
	});
    }

    /* spawn a new task at the top of the threads task stack */
    template<typename Closure>
    __forceinline void spawn_root(const Closure& closure) 
    {
      if (createThreads)
	startThreads();

      assert(!active);
      active = true;
      Thread thread(0,this);
      threadLocal[0] = &thread;
      thread_local_thread = &thread;
      spawn(closure);
      {
	std::unique_lock<std::mutex> lock(mutex);
	atomic_add(&anyTasksRunning,+1);
      }
      condition.notify_all();
      while (thread.tasks.execute_local(thread,NULL));
      atomic_add(&anyTasksRunning,-1);
      active = false;
    }

    /* work on spawned subtasks and wait until all have finished */
    static __forceinline void wait() 
    {
      Thread* thread = TaskSchedulerNew::thread();
      while (thread->tasks.execute_local(*thread,thread->task)) {};
    }

    /* work on spawned subtasks and wait until all have finished */
#if TASKING_LOCKSTEP
    static __forceinline size_t threadCount() {
      return LockStepTaskScheduler::instance()->getNumThreads();
    }
#endif

#if TASKING_TBB
    static __forceinline size_t threadCount() {
      return tbb::task_scheduler_init::default_num_threads(); // FIXME: should return number of really created threads !!
    }
#endif

#if TASKING_TBB_INTERNAL
    static __forceinline size_t threadCount() {
      return TaskSchedulerNew::thread()->scheduler->threadCounter;
    }
#endif

    std::vector<std::thread> threads;
    Thread* threadLocal[MAX_THREADS];
    volatile atomic_t threadCounter;
    volatile bool terminate;
    volatile atomic_t anyTasksRunning;
    volatile bool active;
    bool createThreads;
    bool spinning;

    std::mutex mutex;        
    std::condition_variable condition;

    static __thread Thread* thread_local_thread;
    static Thread* thread();

    __forceinline static TaskSchedulerNew* instance() {
      return thread()->scheduler;
    }
  };
};

