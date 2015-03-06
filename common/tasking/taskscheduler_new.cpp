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
  
  template<typename Predicate, typename Body>
  __forceinline void TaskSchedulerNew::steal_loop(Thread& thread, const Predicate& pred, const Body& body)
  {
    while (true)
    {
      for (size_t i=0; i<32; i++)
      {
        const size_t threadCount = thread.threadCount();
        for (size_t j=0; j<1024; j+=threadCount)
        {
          if (!pred()) return;
          if (thread.scheduler->steal_from_other_threads(thread)) {
            i=j=0;
            body();
          }
        }
#if !defined(__MIC__)
        yield();
#endif
      }
    }
  }

  /*! run this task */
  __dllexport void TaskSchedulerNew::Task::run (Thread& thread) // FIXME: avoid as many __dllexports as possible
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
    steal_loop(thread,
               [&] () { return dependencies > 0; },
               [&] () { while (thread.tasks.execute_local(thread,this)); });
   
    /* now signal our parent task that we are finished */
    if (parent) 
      parent->add_dependencies(-1);
  }

  __dllexport bool TaskSchedulerNew::TaskQueue::execute_local(Thread& thread, Task* parent)
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
  
  bool TaskSchedulerNew::TaskQueue::steal(Thread& thread) 
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
  
  /* we steal from the left */
  size_t TaskSchedulerNew::TaskQueue::getTaskSizeAtLeft() 
  {	
    if (left >= right) return 0;
    return tasks[left].N;
  }
  
  TaskSchedulerNew::TaskSchedulerNew(size_t numThreads, bool spinning)
    : threadCounter(numThreads), createThreads(true), terminate(false), anyTasksRunning(0), active(false), spinning(spinning),
      task_set_function(nullptr)
  {
    for (size_t i=0; i<MAX_THREADS; i++)
      threadLocal[i] = NULL;

    if (numThreads == -1) {
      threadCounter = 1;
      createThreads = false;
    }
    else if (numThreads == 0) {
#if defined(__MIC__)
      threadCounter = getNumberOfLogicalThreads()-4;
#else
      threadCounter = getNumberOfLogicalThreads();
#endif
    }
    task_set_barrier.init(threadCounter);
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
  __dllexport size_t TaskSchedulerNew::threadCount() 
  {
    Thread* thread = TaskSchedulerNew::thread();
    if (thread) return thread->scheduler->threadCounter;
    else        return g_instance->threadCounter;
  }
#endif

  __dllexport TaskSchedulerNew* TaskSchedulerNew::g_instance = NULL;

  void TaskSchedulerNew::create(size_t numThreads)
  {
    if (g_instance) THROW_RUNTIME_ERROR("Embree threads already running.");
#if __MIC__
    g_instance = new TaskSchedulerNew(numThreads,true);
#else
    g_instance = new TaskSchedulerNew(numThreads,false);
#endif
  }

  void TaskSchedulerNew::destroy() {
    delete g_instance; g_instance = NULL;
  }

  struct MyThread
  {
    MyThread (size_t threadIndex, size_t threadCount, TaskSchedulerNew* scheduler)
      : threadIndex(threadIndex), threadCount(threadCount), scheduler(scheduler) {}
    
    size_t threadIndex;
    size_t threadCount;
    TaskSchedulerNew* scheduler;
  };

  void threadFunction(void* ptr) try 
  {
    MyThread thread = *(MyThread*) ptr;
    thread.scheduler->thread_loop(thread.threadIndex);
    delete (MyThread*) ptr;
  }
  catch (const std::exception& e) {
    std::cout << "Error: " << e.what() << std::endl;
    exit(1);
  }

  __dllexport void TaskSchedulerNew::startThreads()
  {
    createThreads = false;
    //for (size_t i=1; i<threadCounter; i++) {
    //  threads.push_back(std::thread([i,this]() { thread_loop(i); }));
    //}
    for (size_t t=1; t<threadCounter; t++) {
      threads.push_back(createThread((thread_func)threadFunction,new MyThread(t,threadCounter,this),4*1024*1024,t));
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
      //threads[i].join();
      embree::join(threads[i]);
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

  /* work on spawned subtasks and wait until all have finished */
  __dllexport void TaskSchedulerNew::wait() 
  {
    Thread* thread = TaskSchedulerNew::thread();
    if (thread == nullptr) return;
    while (thread->tasks.execute_local(*thread,thread->task)) {};
  }

  void TaskSchedulerNew::thread_loop(size_t threadIndex) try 
  {
#if defined(__MIC__)
    setAffinity(threadIndex);
#endif

    /* allocate thread structure */
    Thread thread(threadIndex,this);
    threadLocal[threadIndex] = &thread;
    thread_local_thread = &thread;

#if 0
#if 0
    while (true)
      task_set_barrier.wait(threadIndex,thread.threadCount());
#else
    while (true)
    {
      while (!anyTasksRunning) __pause_cpu(128);
      if (task_set_function) {
        task_set_barrier.wait(threadIndex,thread.threadCount());
        task_set_barrier.wait(threadIndex,thread.threadCount());
      }
    }
#endif
#endif

    /* main thread loop */
    while (!terminate)
    {
      auto predicate = [&] () { return anyTasksRunning || terminate; };

      /* all threads are either spinning ... */
      if (spinning) 
      {
	while (!predicate())
          __pause_cpu(128);
      }
      
      /* ... or waiting inside some condition variable */
      else
      {
        std::unique_lock<std::mutex> lock(mutex);
        condition.wait(lock, predicate);
      }
      if (terminate) break;
      //executeTaskSet(thread); 
      //continue;
      
      /* work on available task */
      steal_loop(thread,
                 [&] () { return anyTasksRunning > 0; },
                 [&] () { 
                   atomic_add(&anyTasksRunning,+1);
                   while (thread.tasks.execute_local(thread,NULL));
                   atomic_add(&anyTasksRunning,-1);
                 });
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

  __dllexport bool TaskSchedulerNew::executeTaskSet(Thread& thread)
  {
    if (task_set_function)
    {
      const size_t threadIndex = thread.threadIndex;
      const size_t threadCount = this->threadCounter;
      TaskSetFunction* function = task_set_function;
      task_set_barrier.wait(threadIndex,threadCount);
      if (threadIndex == 0) task_set_function = NULL;
      const size_t task_set_size = function->end-function->begin;
      const size_t begin = function->begin+(threadIndex+0)*task_set_size/threadCount;
      const size_t end   = function->begin+(threadIndex+1)*task_set_size/threadCount;
      const size_t bs = function->blockSize;
      for (size_t i=begin; i<end; i+=bs)
        function->execute(range<size_t>(i,min(i+bs,end)));
      task_set_barrier.wait(threadIndex,threadCount);
      return true;
    }
    return false;
  }

  bool TaskSchedulerNew::steal_from_other_threads(Thread& thread)
  {
    const size_t threadIndex = thread.threadIndex;
    const size_t threadCount = this->threadCounter;

    /*if (task_set_function) {
      task_set_barrier.wait(threadIndex,thread.threadCount());
      task_set_barrier.wait(threadIndex,thread.threadCount());
    }
    return false;*/

    /* special static load balancing for top level task sets */
#if TASKSCHEDULER_STATIC_LOAD_BALANCING
    if (executeTaskSet(thread))
      return false;
#endif
    
#if SORTED_STEALING == 1
    size_t workingThreads = 0;
    std::pair<size_t,size_t> thread_task_size[MAX_THREADS];

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
