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
#include "sys/sysinfo.h"

namespace embree
{
  TaskSchedulerNew::TaskSchedulerNew(size_t numThreads_in)
    : numThreads(numThreads_in), createThreads(true), terminate(false), anyTasksRunning(0), numThreadsRunning(0), active(false)
  {
    if (!numThreads_in)
      numThreads = getNumberOfLogicalThreads();
    
    for (size_t i=0; i<MAX_THREADS; i++)
      threadLocal[i] = NULL;

    startThreads();
  }
  
  TaskSchedulerNew::~TaskSchedulerNew() 
  {
    /* let all threads leave the thread loop */
    terminateThreadLoop();

    /* destroy all threads that we created */
    destroyThreads();
  }

  void TaskSchedulerNew::startThreads()
  {
    createThreads = false;
    for (size_t i=1; i<numThreads; i++) {
      atomic_add(&numThreadsRunning,1);
      threads.push_back(std::thread([i,this]() { thread_loop(i); }));
    }
  }

  void TaskSchedulerNew::terminateThreadLoop()
  {
    /* signal threads to terminate */
    mutex.lock();
    terminate = true;
    mutex.unlock();

    while (numThreadsRunning)
      condition.notify_all();
  }

  void TaskSchedulerNew::destroyThreads() 
  {
    /* wait for threads to terminate */
    for (size_t i=0; i<threads.size(); i++) 
      threads[i].join();
  }

#if 0
  void TaskSchedulerNew::join()
  {
    size_t threadIndex = atomic_add(&numThreads,1);
    assert(threadIndex < MAX_THREADS);
  
    /* allocate thread structure */
    Thread thread(threadIndex,this);
    threadLocal[threadIndex] = &thread;
    thread_local_thread = &thread;

    /* process tasks */
    while (anyTasksRunning)
      task_loop(thread);
  }
#endif

  __thread TaskSchedulerNew::Thread* TaskSchedulerNew::thread_local_thread = NULL;

  TaskSchedulerNew::Thread* TaskSchedulerNew::thread() {
    return thread_local_thread;
  }

  void TaskSchedulerNew::thread_loop(size_t threadIndex) try 
  {
    /* allocate thread structure */
    Thread thread(threadIndex,this);
    threadLocal[threadIndex] = &thread;
    thread_local_thread = &thread;

    /* main thread loop */
    while (!terminate)
    {
      /* wait for tasks to enter the tasking system */
      while (!anyTasksRunning && !terminate) {
        std::unique_lock<std::mutex> lock(mutex);
        condition.wait(lock);
      }
      if (terminate) break;
      
      /* work on available task */
      while (anyTasksRunning) {
	if (thread.scheduler->steal_from_other_threads(thread)) {
	  atomic_add(&anyTasksRunning,+1);
	  while (thread.tasks.execute_local(thread,NULL));
	  atomic_add(&anyTasksRunning,-1);
	}
      }
    }

    /* decrement thread counter */
    atomic_add(&numThreadsRunning,-1);
  }
  catch (const std::exception& e) {
    std::cout << "Error: " << e.what() << std::endl; // FIXME: propagate to main thread
    exit(1);
  }

  bool TaskSchedulerNew::steal_from_other_threads(Thread& thread)
  {
    const size_t threadIndex = thread.threadIndex;
    const size_t threadCount = threads.size()+1;

    for (size_t i=1; i<threadCount; i++) 
    //for (size_t i=1; i<5; i++) 
    {
      __pause_cpu();
      size_t otherThreadIndex = threadIndex+i;
      if (otherThreadIndex >= threadCount) otherThreadIndex -= threadCount;

      if (!threadLocal[otherThreadIndex])
        continue;
      
      if (threadLocal[otherThreadIndex]->tasks.steal(thread)) {
        return true;
      }
    }

    return false;
  }
}
