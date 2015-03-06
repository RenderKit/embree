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

#if defined(__MIC__)
#define TASKSCHEDULER_STATIC_LOAD_BALANCING 1
#else
#define TASKSCHEDULER_STATIC_LOAD_BALANCING 0
#endif

namespace embree
{
#if defined(TASKING_TBB)
#  define SPAWN_BEGIN tbb::task_group __internal_task_group
#  define SPAWN(closure) __internal_task_group.run(closure)
#  define SPAWN_END __internal_task_group.wait()
#endif

#if defined(TASKING_TBB_INTERNAL)
#  define SPAWN_BEGIN 
#  define SPAWN(closure) TaskSchedulerNew::spawn(closure)
#  define SPAWN_END TaskSchedulerNew::wait();
#endif

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

    /*! builds a task interface from a closure */
    template<typename Closure>
    struct ClosureTaskFunction : public TaskFunction
    {
      Closure closure;
      __forceinline ClosureTaskFunction (const Closure& closure) : closure(closure) {}
      void execute() { closure(); };
    };

    /*! virtual interface for all tasks */
    struct TaskSetFunction 
    {
      __forceinline TaskSetFunction(size_t begin, size_t end, size_t blockSize) 
        : begin(begin), end(end), blockSize(blockSize) {}
      virtual void execute(const range<size_t>& r) = 0;
      size_t begin,end,blockSize;
    };

    /*! builds a task interface from a closure */
    template<typename Closure>
    struct ClosureTaskSetFunction : public TaskSetFunction
    {
      Closure closure;
      __forceinline ClosureTaskSetFunction (const Closure& closure, size_t begin, size_t end, size_t blockSize) 
        : closure(closure), TaskSetFunction(begin,end,blockSize) {}
      void execute(const range<size_t>& r) { closure(r); };
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
      __forceinline Task (TaskFunction* closure, Task* parent, size_t stackPtr, size_t N) 
        : closure(closure), parent(parent), stackPtr(stackPtr), dependencies(1), N(N)
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
      void run (Thread& thread);

    public:
      volatile atomic32_t state;         //!< state this task is in
      volatile atomic32_t dependencies;  //!< dependencies to wait for
      TaskFunction* closure;             //!< the closure to execute
      Task* parent;                      //!< parent task to signal when we are finished
      size_t stackPtr;                   //!< stack location where closure is stored
      size_t N;                          //!< approximative size of task
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
      __forceinline void push_right(Thread& thread, const size_t size, const Closure& closure) 
      {
        assert(right < TASK_STACK_SIZE);
        
	/* allocate new task on right side of stack */
        size_t oldStackPtr = stackPtr;
        TaskFunction* func = new (alloc(sizeof(ClosureTaskFunction<Closure>))) ClosureTaskFunction<Closure>(closure);
        new (&tasks[right++]) Task(func,thread.task,oldStackPtr,size);

	/* also move left pointer */
	if (left >= right-1) left = right-1;
      }
      
      bool execute_local(Thread& thread, Task* parent);
      bool steal(Thread& thread);
      size_t getTaskSizeAtLeft();

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

      __forceinline size_t threadCount() {
        return scheduler->threadCounter;
      }
      
      size_t threadIndex;              //!< ID of this thread
      TaskQueue tasks;                 //!< local task queue
      Task* task;                      //!< current active task
      TaskSchedulerNew* scheduler;     //!< pointer to task scheduler
    };
    
    TaskSchedulerNew (size_t numThreads = 0, bool spinning = false);
    ~TaskSchedulerNew ();

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
    bool executeTaskSet(Thread& thread);

    template<typename Predicate, typename Body>
      static void steal_loop(Thread& thread, const Predicate& pred, const Body& body);

    /* spawn a new task at the top of the threads task stack */
    template<typename Closure>
    __noinline void spawn_root(const Closure& closure, size_t size = 1) // important: has to be noinline as it allocates thread structure on stack
    {
      if (createThreads)
	startThreads();

      assert(!active);
      active = true;
      Thread thread(0,this);
      threadLocal[0] = &thread;
      thread_local_thread = &thread;
      thread.tasks.push_right(thread,size,closure);
      {
	std::unique_lock<std::mutex> lock(mutex);
	atomic_add(&anyTasksRunning,+1);
      }
      if (!spinning) condition.notify_all();
      while (thread.tasks.execute_local(thread,NULL));
      atomic_add(&anyTasksRunning,-1);

      threadLocal[0] = NULL;
      thread_local_thread = NULL;
      active = false;
    }

    /* spawn a new task at the top of the threads task stack */
    template<typename Closure>
    __noinline void spawn_root(const Closure& closure, size_t begin, size_t end, size_t blockSize) // important: has to be noinline as it allocates thread structure on stack
    {
      if (createThreads)
	startThreads();

      assert(!active);
      active = true;
      Thread thread(0,this);
      threadLocal[0] = &thread;
      thread_local_thread = &thread;

      ClosureTaskSetFunction<Closure> func(closure,begin,end,blockSize);
      task_set_function = &func;
      __memory_barrier();
      
      {
	std::unique_lock<std::mutex> lock(mutex);
	atomic_add(&anyTasksRunning,+1);
      }
      if (!spinning) condition.notify_all();
      executeTaskSet(thread);
      while (thread.tasks.execute_local(thread,NULL));
      atomic_add(&anyTasksRunning,-1);

      threadLocal[0] = NULL;
      thread_local_thread = NULL;
      active = false;
    }

    /* spawn a new task at the top of the threads task stack */
    template<typename Closure>
    static __forceinline void spawn(size_t size, const Closure& closure) 
    {
      Thread* thread = TaskSchedulerNew::thread();
      if (likely(thread != nullptr)) thread->tasks.push_right(*thread,size,closure);
      else                           g_instance->spawn_root(closure,size);
    }

    /* spawn a new task at the top of the threads task stack */
    template<typename Closure>
    static __forceinline void spawn(const Closure& closure) {
      spawn(1,closure);
    }

    /* spawn a new task set  */
    template<typename Closure>
    static void spawn(const size_t begin, const size_t end, const size_t blockSize, const Closure& closure) 
    {
#if TASKSCHEDULER_STATIC_LOAD_BALANCING
      Thread* thread = TaskSchedulerNew::thread();
      if (thread == nullptr) {
        g_instance->spawn_root(closure,begin,end,blockSize);
      }
#endif

      spawn(end-begin, [=,&closure]() 
        {
	  if (end-begin <= blockSize) {
	    return closure(range<size_t>(begin,end));
	  }
	  const size_t center = (begin+end)/2;
	  spawn(begin,center,blockSize,closure);
	  spawn(center,end  ,blockSize,closure);
	  wait();
	});
    }

    /* work on spawned subtasks and wait until all have finished */
    static void wait();

    /* work on spawned subtasks and wait until all have finished */
    static size_t threadCount();

    static Thread* thread();
    
    __forceinline static TaskSchedulerNew* instance() {
      return thread()->scheduler;
    }

  private:
    //std::vector<std::thread> threads;
    std::vector<thread_t> threads;
    Thread* threadLocal[MAX_THREADS];
    volatile atomic_t threadCounter;
    volatile bool terminate;
    volatile atomic_t anyTasksRunning;
    volatile bool active;
    bool createThreads;
    bool spinning;

    std::mutex mutex;        
    std::condition_variable condition;

    /* special toplevel taskset optimization */
  private:
#if defined(__MIC__)
    __aligned(64) QuadTreeBarrier task_set_barrier;
#else
    __aligned(64) LinearBarrierActive task_set_barrier;
#endif
    TaskSetFunction* volatile task_set_function;
    
  private:
    static TaskSchedulerNew* g_instance;
    static __thread Thread* thread_local_thread;
  };
};

