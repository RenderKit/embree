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

#include "../sys/platform.h"
#include "../sys/alloc.h"
#include "../sys/barrier.h"
#include "../sys/thread.h"
#include "../sys/mutex.h"
#include "../sys/condition.h"
#include "../sys/ref.h"
#include "../../kernels/algorithms/range.h"

#include <list>

#if !defined(TASKING_TBB_INTERNAL) && !defined(__MIC__)
#define NOMINMAX
#define __TBB_NO_IMPLICIT_LINKAGE 1
//#define is_trivially_copyable has_trivial_copy_constructor
#include "tbb/tbb.h"
#endif

namespace embree
{
#if !defined(TASKING_TBB_INTERNAL)
#  define SPAWN_BEGIN tbb::task_group __internal_task_group
#  define SPAWN(closure) __internal_task_group.run(closure)
#  define SPAWN_END __internal_task_group.wait();                       \
  if (tbb::task::self().is_cancelled())        \
    throw std::runtime_error("task group cancelled");
#else
#  define SPAWN_BEGIN 
#  define SPAWN(closure) TaskSchedulerTBB::spawn(closure)
#  define SPAWN_END TaskSchedulerTBB::wait();
#endif

  struct TaskSchedulerTBB : public RefCount
  {
    ALIGNED_STRUCT;

    static const size_t TASK_STACK_SIZE = 1024;           //!< task structure stack
    static const size_t CLOSURE_STACK_SIZE = 256*1024;    //!< stack for task closures

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
      __dllexport void run(Thread& thread);

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
      
      __dllexport bool execute_local(Thread& thread, Task* parent);
      bool steal(Thread& thread);
      size_t getTaskSizeAtLeft();

      bool empty() { return right == 0; }

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
      Thread (size_t threadIndex, const Ref<TaskSchedulerTBB>& scheduler)
      : threadIndex(threadIndex), scheduler(scheduler), task(nullptr) {}

      __forceinline size_t threadCount() {
        return scheduler->threadCounter;
      }
      
      size_t threadIndex;              //!< ID of this thread
      TaskQueue tasks;                 //!< local task queue
      Task* task;                      //!< current active task
      Ref<TaskSchedulerTBB> scheduler;     //!< pointer to task scheduler
    };

    /*! pool of worker threads */
    struct ThreadPool
    {
      ThreadPool (size_t numThreads = 0, bool set_affinity = false);
      ~ThreadPool ();

      /*! starts the threads */
      __dllexport void startThreads();

      /*! adds a task scheduler object for scheduling */
      __dllexport void add(const Ref<TaskSchedulerTBB>& scheduler);

      /*! remove the task scheduler object again */
      __dllexport void remove(const Ref<TaskSchedulerTBB>& scheduler);

      /*! returns number of threads of the thread pool */
      size_t size() const { return numThreads; }

      /*! main loop for all threads */
      void thread_loop();
      
    private:
      size_t numThreads;
      bool set_affinity;
      bool running;
      volatile bool terminate;
      std::vector<thread_t> threads;

    private:
      MutexSys mutex;
      ConditionSys condition;
      std::list<Ref<TaskSchedulerTBB> > schedulers;
    };

    TaskSchedulerTBB ();
    ~TaskSchedulerTBB ();

    /*! initializes the task scheduler */
    static void create(size_t numThreads, bool set_affinity);

    /*! destroys the task scheduler again */
    static void destroy();
    
    /*! lets new worker threads join the tasking system */
    void join();
    void reset();

    /*! let a worker thread allocate a thread index */
    __dllexport ssize_t allocThreadIndex();

    /*! wait for some number of threads available (threadCount includes main thread) */
    void wait_for_threads(size_t threadCount);

    /*! thread loop for all worker threads */
    void thread_loop(size_t threadIndex);

    /*! steals a task from a different thread */
    bool steal_from_other_threads(Thread& thread);

    template<typename Predicate, typename Body>
      static void steal_loop(Thread& thread, const Predicate& pred, const Body& body);

    /* spawn a new task at the top of the threads task stack */
    template<typename Closure>
    __noinline void spawn_root(const Closure& closure, size_t size = 1, bool useThreadPool = true) // important: has to be noinline as it allocates thread structure on stack
    {
      if (useThreadPool) startThreads();
      
      size_t threadIndex = allocThreadIndex();
      Thread thread(threadIndex,this);
      assert(threadLocal[threadIndex] == nullptr);
      threadLocal[threadIndex] = &thread;
      Thread* oldThread = swapThread(&thread);
      thread.tasks.push_right(thread,size,closure);
      {
        Lock<MutexSys> lock(mutex);
	atomic_add(&anyTasksRunning,+1);
        hasRootTask = true;
        condition.notify_all();
      }
      
      if (useThreadPool) addScheduler(this);

      while (thread.tasks.execute_local(thread,nullptr));
      atomic_add(&anyTasksRunning,-1);
      if (useThreadPool) removeScheduler(this);
      
      threadLocal[threadIndex] = nullptr;
      swapThread(oldThread);

      /* wait for all threads to terminate */
      atomic_add(&threadCounter,-1);
      while (threadCounter > 0) {
        yield();
      }

      //assert(anyTasksRunning == -1);
      //anyTasksRunning = 0;
    }

    /* spawn a new task at the top of the threads task stack */
    template<typename Closure>
    static __forceinline void spawn(size_t size, const Closure& closure) 
    {
      Thread* thread = TaskSchedulerTBB::thread();
      if (likely(thread != nullptr)) thread->tasks.push_right(*thread,size,closure);
      else                           instance()->spawn_root(closure,size);
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
    __dllexport static void wait();

    /* returns the index of the current thread */
    __dllexport static size_t threadIndex();

    /* returns the total number of threads */
    __dllexport static size_t threadCount();

  private:

    /* returns the thread local task list of this worker thread */
    __dllexport static Thread* thread();

    /* sets the thread local task list of this worker thread */
    __dllexport static Thread* swapThread(Thread* thread);

    /*! returns the taskscheduler object to be used by the master thread */
    __dllexport static TaskSchedulerTBB* instance();

    /*! starts the threads */
    __dllexport static void startThreads();

    /*! adds a task scheduler object for scheduling */
    __dllexport static void addScheduler(const Ref<TaskSchedulerTBB>& scheduler);

    /*! remove the task scheduler object again */
    __dllexport static void removeScheduler(const Ref<TaskSchedulerTBB>& scheduler);

  private:
    Thread* threadLocal[MAX_THREADS]; // FIXME: thread should be no maximal number of threads
    volatile atomic_t threadCounter;
    volatile atomic_t anyTasksRunning;
    volatile bool hasRootTask;
    MutexSys mutex;
    ConditionSys condition;

  private:
    static __thread TaskSchedulerTBB* g_instance;
    static __thread Thread* thread_local_thread;
    static ThreadPool* threadPool;
  };
};

