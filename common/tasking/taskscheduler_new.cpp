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

#include "taskscheduler_new.h"
#include "math/math.h"

#include <thread>
#include <mutex>
#include <condition_variable>

#if USE_TBB
#include <tbb/tbb.h>
#endif

namespace embree
{
#if 1
  

#else

  TaskSchedulerNew::TaskSchedulerNew(size_t numThreads)
    : numThreads(numThreads), terminate(false)
  {
    if (!numThreads) numThreads = getNumberOfLogicalThreads();
    if ( numThreads) numThreads--;

    for (size_t i=0; i<MAX_THREADS; i++)
      threadLocal[i] = NULL;
    
    for (size_t i=0; i<numThreads; i++) {
      threads.push_back(std::thread([i]() { schedule(i); }));
    }
  }
  
  TaskSchedulerNew::~TaskSchedulerNew() 
  {
    /* signal threads to terminate */
    mutex.lock();
    terminate = true;
    mutex.unlock();
    condition.signal_all();

    /* wait for threads to terminate */
    for (size_t i=0; i<threads.size(); i++) 
      join(threads[i]);
  }

  __thread TaskSchedulerNew::Thread* TaskSchedulerNew::thread_local_thread = NULL;

  TaskSchedulerNew::Thread* TaskSchedulerNew::thread() {
    return thread_local_thread;
  }

  __thread TaskSchedulerNew::Task* TaskSchedulerNew::thread_local_task = NULL;

  TaskSchedulerNew::Task* TaskSchedulerNew::task() {
    return thread_local_task;
  }

#if 0
  ssize_t TaskSchedulerNew::allocThreadIndex()
  {
    while (true) {
      size_t N = numThreads;
      if (N >= MAX_THREADS) break;
      if (atomic_cmpxchg(&numThreads,N,N+1) == N)
        return N;
    }
    return -1;
  }

  bool TaskSchedulerNew::join()
  {
    /* try to get a new thread index */
    ssize_t threadIndex = allocThreadIndex();
    if (threadIndex < 0) return false;

  }
#endif

  void TaskSchedulerNew::schedule(size_t threadIndex) try 
  {
    /* allocate thread structure */
    Thread thread(threadIndex,this);
    threadLocals[threadIndex] = &thread;
    thread_local_thread = &thread;

    /* main thread loop */
    while (!terminate)
    {
      /* wait for tasks to enter the tasking system */
      {
        std::unique_lock<std::mutex> lock(mutex);
        condition.wait(lock);
      }

      /* continue until there are some running tasks */
      while (anyTasksRunning)
      {
        /* first try executing local tasks */
        while (thread.tasks.local()) {
          if (terminate) return;
        }
        atomic_add(&anyTasksRunning,-1);
        
        /* second try to steal tasks */
        for (size_t i=0; i<threadCount; i++) 
        {
          const size_t otherThreadIndex = (threadIndex+i)%threadCount; // FIXME: optimize %
          if (!threadLocals[otherThreadIndex])
            continue;
          
          if (threadLocals[otherThreadIndex]->tasks.steal(&anyTasksRunning))
            break;
        }
      }
    }
  }
  catch (const std::exception& e) {
    std::cout << "Error: " << e.what() << std::endl; // FIXME: propagate to main thread
    exit(1);
  }

  void TaskSchedulerNew::schedule_local(Thread* thread)
  {
    Task* task = TaskSchedulerNew::task();
    while (thread->tasks.local(task)) {}
  }

  void TaskSchedulerNew::scheduleLocal()
  {
    Thread* thread = TaskSchedulerNew::thread();
    thread->scheduler->scheduler_local(thread);
  }

  /* regression testing */
  struct task_scheduler_regression_test : public RegressionTest
  {
    task_scheduler_regression_test(const char* name) : name(name) {
      registerRegressionTest(this);
    }
    
    bool operator() ()
    {
      bool passed = true;
      printf("%s::%s ... ",TOSTRING(isa),name);
      fflush(stdout);

      /* create task scheduler */
      TaskSchedulerNew* scheduler = new TaskSchedulerNew;

      struct Fib
      {
        size_t& r;
        size_t i;
        
        Fib(size_t& r, size_t i) : r(r), i(i) {}
        
        void operator(size_t i) const
        {
          if (i == 0) return 0;
          else if (i == 1) return 1;
          else {
            size_t r0; const Fib fib0(r0, i-1);
            size_t r1; const Fib fib1(r1, i-2);
            TaskScheduler::spawn(fib0);
            TaskScheduler::spawn(fib1);
            TaskScheduelr::wait();
            r = r0+r1;
          }
        }
      };

      /* parallel calculation of sum of fibonacci number */
      double t0 = getSeconds();
      size_t r; Fib fib(r,1000);
      TaskSchedulerNew::spawn_root(fib);
      double t1 = getSeconds();
      printf("fib(1000) = %z\n",r);
      //printf("%zu/%3.2fM ",N,1E-6*double(N*M)/(t1-t0));
            
      /* output if test passed or not */
      if (passed) printf("[passed]\n");
      else        printf("[failed]\n");
      
      return passed;
    }

    const char* name;
  };

  task_scheduler_regression_test task_scheduler_regression("task_scheduler_regression_test");

#endif
}
