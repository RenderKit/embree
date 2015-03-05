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
#include <algorithm>

#define SORTED_STEALING 0

namespace embree
{
  size_t g_numThreads = 0;                              //!< number of threads to use in builders

  /*! run this task */
  __dllexport void TaskSchedulerNew::Task::run (Thread& thread) // FIXME: avoid as many __dllexports as possible
  {
    /* try to run if not already stolen */
    if (try_switch_state(INITIALIZED,DONE))
    {
      Task* prevTask = thread.task; 
      
      thread.task = this;
      /* set estimate working size here */
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
  
  TaskSchedulerNew::TaskSchedulerNew(size_t numThreads, bool spinning)
    : threadCounter(numThreads), createThreads(true), terminate(false), anyTasksRunning(0), active(false), spinning(spinning)
  {
    for (size_t i=0; i<MAX_THREADS; i++)
      threadLocal[i] = NULL;

    if (numThreads == -1) {
      threadCounter = 1;
      createThreads = false;
    }
    else if (numThreads == 0) {
      threadCounter = getNumberOfLogicalThreads();
    }
  }
  
  TaskSchedulerNew::~TaskSchedulerNew() 
  {
    /* let all threads leave the thread loop */
    terminateThreadLoop();

    /* destroy all threads that we created */
    destroyThreads();
  }

#if TASKING_LOCKSTEP
  __dllexport size_t TaskSchedulerNew::threadCount() {
    return LockStepTaskScheduler::instance()->getNumThreads();
  }
#endif

#if TASKING_TBB
  __dllexport size_t TaskSchedulerNew::threadCount()
  {
    return g_numThreads;
    //return tbb::task_scheduler_init::default_num_threads();
  }
#endif

#if TASKING_TBB_INTERNAL
  __dllexport size_t TaskSchedulerNew::threadCount() {
    return TaskSchedulerNew::thread()->scheduler->threadCounter;
  }
#endif

  __dllexport TaskSchedulerNew* TaskSchedulerNew::g_instance = NULL;

  void TaskSchedulerNew::create(size_t numThreads)
  {
    if (g_instance) THROW_RUNTIME_ERROR("Embree threads already running.");
    g_instance = new TaskSchedulerNew(numThreads);
  }

  void TaskSchedulerNew::destroy() {
    delete g_instance; g_instance = NULL;
  }

  __dllexport void TaskSchedulerNew::startThreads()
  {
    createThreads = false;
    for (size_t i=1; i<threadCounter; i++) {
      threads.push_back(std::thread([i,this]() { thread_loop(i); }));
    }
  }

  void TaskSchedulerNew::terminateThreadLoop()
  {
    /* decrement threadCount again */
    atomic_add(&threadCounter,-1);

    /* signal threads to terminate */
    mutex.lock();
    terminate = true;
    mutex.unlock();
    condition.notify_all();

    /* wait for all threads to terminate */
    if (threads.size())
      while (threadCounter > 0)
        yield();

    threadLocal[0] = NULL;
  }

  void TaskSchedulerNew::destroyThreads() 
  {
    /* wait for threads to terminate */
    for (size_t i=0; i<threads.size(); i++) 
      threads[i].join();
  }

  void TaskSchedulerNew::join()
  {
    size_t threadIndex = atomic_add(&threadCounter,1);
    assert(threadIndex < MAX_THREADS);
    thread_loop(threadIndex);
  }

  void TaskSchedulerNew::wait_for_threads(size_t threadCount)
  {
    while (threadCounter < threadCount)
      __pause_cpu();
  }

  __dllexport __thread TaskSchedulerNew::Thread* TaskSchedulerNew::thread_local_thread = NULL;

  __dllexport TaskSchedulerNew::Thread* TaskSchedulerNew::thread() {
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
      auto predicate = [&] () { return anyTasksRunning || terminate; };

      /* all threads are either spinning ... */
      if (spinning) 
      {
	while (!predicate())
	  __pause_cpu();
      }
      
      /* ... or waiting inside some condition variable */
      else
      {
        std::unique_lock<std::mutex> lock(mutex);
        condition.wait(lock, predicate);
      }
      if (terminate) break;
      
      /* work on available task */
      while (anyTasksRunning) 
      {
	if (thread.scheduler->steal_from_other_threads(thread)) 
	{
	  atomic_add(&anyTasksRunning,+1);
	  while (thread.tasks.execute_local(thread,NULL));
	  atomic_add(&anyTasksRunning,-1);
	}
      }
    }

    /* decrement threadCount again */
    atomic_add(&threadCounter,-1);

    /* wait for all threads to terminate */
    while (threadCounter > 0)
      yield();

    threadLocal[threadIndex] = NULL;
  }
  catch (const std::exception& e) 
  {
    std::cout << "Error: " << e.what() << std::endl; // FIXME: propagate to main thread
    threadLocal[threadIndex] = NULL;
    exit(1);
  }

  bool TaskSchedulerNew::steal_from_other_threads(Thread& thread)
  {
    const size_t threadIndex = thread.threadIndex;
    const size_t threadCount = this->threadCounter;

#if SORTED_STEALING == 1
    size_t workingThreads = 0;
    std::pair<size_t,size_t> thread_task_size[MAX_MIC_THREADS];

    /* find thread with largest estimated size left */
    for (size_t i=1; i<threadCount; i++) 
      {
	size_t otherThreadIndex = threadIndex+i;
	if (otherThreadIndex >= threadCount) otherThreadIndex -= threadCount;

	if (!threadLocal[otherThreadIndex])
	  continue;

	const size_t task_size = threadLocal[otherThreadIndex]->tasks.getTaskSizeAtLeft(); /* we steal from the left side */

	thread_task_size[workingThreads++] = std::pair<size_t,size_t>(task_size,otherThreadIndex);
      }

    /* sort thread/size pairs based on size */
    std::sort(thread_task_size,
	      &thread_task_size[workingThreads],
	      [](const std::pair<size_t,size_t> & a, const std::pair<size_t,size_t> & b) -> bool
	      { 
		return a.first > b.first; 
	      });

    /*
    if (threadIndex == 0)
      for (size_t i=0;i<workingThreads;i++)
	std::cout << "thread_task_size " << thread_task_size[i].first << " " << thread_task_size[i].second << std::endl;
    */

    for (size_t i=0; i<workingThreads; i++) 
      {
	const size_t otherThreadIndex = thread_task_size[i].second;
	if (!threadLocal[otherThreadIndex])
	  continue;
      
	if (threadLocal[otherThreadIndex]->tasks.steal(thread))
	  return true;
      }    
    /* nothing found this time, do another round */

#else	      
    for (size_t i=1; i<threadCount; i++) 
    {
      __pause_cpu();
      size_t otherThreadIndex = threadIndex+i;
      if (otherThreadIndex >= threadCount) otherThreadIndex -= threadCount;

      if (!threadLocal[otherThreadIndex])
        continue;
      
      if (threadLocal[otherThreadIndex]->tasks.steal(thread)) 
        return true;      
    }
#endif	      


    return false;
  }
}
