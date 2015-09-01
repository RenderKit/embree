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

#include "taskscheduler_tbb.h"
#include "../math/math.h"
#include "../sys/sysinfo.h"
#include <algorithm>

#if TASKING_LOCKSTEP
#include "taskscheduler_mic.h"
#endif

namespace embree
{
  size_t g_numThreads = 0;                              //!< number of threads to use in builders
  __thread TaskSchedulerTBB* TaskSchedulerTBB::g_instance = nullptr;
  __thread TaskSchedulerTBB::Thread* TaskSchedulerTBB::thread_local_thread = nullptr;
  TaskSchedulerTBB::ThreadPool* TaskSchedulerTBB::threadPool = nullptr;

  template<typename Predicate, typename Body>
  __forceinline void TaskSchedulerTBB::steal_loop(Thread& thread, const Predicate& pred, const Body& body)
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
        yield();
      }
    }
  }

  /*! run this task */
  __dllexport void TaskSchedulerTBB::Task::run (Thread& thread) // FIXME: avoid as many __dllexports as possible
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

  __dllexport bool TaskSchedulerTBB::TaskQueue::execute_local(Thread& thread, Task* parent)
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
  
  bool TaskSchedulerTBB::TaskQueue::steal(Thread& thread) 
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
  size_t TaskSchedulerTBB::TaskQueue::getTaskSizeAtLeft() 
  {	
    if (left >= right) return 0;
    return tasks[left].N;
  }

  void threadPoolFunction(void* ptr) try 
  {
    TaskSchedulerTBB::ThreadPool* pool = (TaskSchedulerTBB::ThreadPool*) ptr;
    pool->thread_loop();
  }
  catch (const std::exception& e) {
    std::cout << "Error: " << e.what() << std::endl; // FIXME: propagate to main thread
    exit(1);
  }

  TaskSchedulerTBB::ThreadPool::ThreadPool(size_t numThreads, bool set_affinity)
    : numThreads(numThreads), set_affinity(set_affinity), running(false), terminate(false) 
  {
    if (this->numThreads == 0)
      this->numThreads = getNumberOfLogicalThreads();
  }

  __dllexport void TaskSchedulerTBB::ThreadPool::startThreads()
  {
    if (!running) 
    {
      running = true;
      for (size_t t=1; t<numThreads; t++) {
        threads.push_back(createThread((thread_func)threadPoolFunction,this,4*1024*1024,set_affinity ? t : -1));
      }
    }
  }

  TaskSchedulerTBB::ThreadPool::~ThreadPool()
  {
    /* leave all taskschedulers */
    mutex.lock();
    terminate = true;
    mutex.unlock();
    condition.notify_all();

    /* wait for threads to terminate */
    for (size_t i=0; i<threads.size(); i++) 
      embree::join(threads[i]);
  }

  __dllexport void TaskSchedulerTBB::ThreadPool::add(const Ref<TaskSchedulerTBB>& scheduler)
  {
    mutex.lock();
    schedulers.push_back(scheduler);
    mutex.unlock();
    condition.notify_all();
  }

  __dllexport void TaskSchedulerTBB::ThreadPool::remove(const Ref<TaskSchedulerTBB>& scheduler)
  {
    Lock<MutexSys> lock(mutex);
    for (std::list<Ref<TaskSchedulerTBB> >::iterator it = schedulers.begin(); it != schedulers.end(); it++) {
      if (scheduler == *it) {
        schedulers.erase(it);
        return;
      }
    }
  }

  void TaskSchedulerTBB::ThreadPool::thread_loop()
  {
    while (!terminate)
    {
      Ref<TaskSchedulerTBB> scheduler = NULL;
      ssize_t threadIndex = -1;
      {
        Lock<MutexSys> lock(mutex);
        condition.wait(mutex, [&] () { return terminate || !schedulers.empty(); });
        if (terminate) break;
        scheduler = schedulers.front();
        threadIndex = scheduler->allocThreadIndex();
      }
      scheduler->thread_loop(threadIndex);
    }
  }
  
  TaskSchedulerTBB::TaskSchedulerTBB()
    : threadCounter(0), anyTasksRunning(0), hasRootTask(false)
  {
    for (size_t i=0; i<MAX_THREADS; i++)
      threadLocal[i] = nullptr;
  }
  
  TaskSchedulerTBB::~TaskSchedulerTBB() 
  {
    assert(threadCounter == 0);
  }

#if TASKING_LOCKSTEP
  __dllexport size_t TaskSchedulerTBB::threadCount() {
    return LockStepTaskScheduler::instance()->getNumThreads();
  }
#endif

#if TASKING_TBB
   __dllexport size_t TaskSchedulerTBB::threadIndex() {
#if TBB_INTERFACE_VERSION_MAJOR < 8
     return 0;
#else
     return tbb::task_arena::current_thread_index();
#endif
   }
  __dllexport size_t TaskSchedulerTBB::threadCount() {
    return g_numThreads; // FIXME: possible to return number of thread through TBB call?
    //return tbb::task_scheduler_init::default_num_threads();
  }
#endif

#if TASKING_TBB_INTERNAL
  __dllexport size_t TaskSchedulerTBB::threadIndex() 
  {
    Thread* thread = TaskSchedulerTBB::thread();
    if (thread) return thread->threadIndex;
    else        return 0;
  }
  __dllexport size_t TaskSchedulerTBB::threadCount() {
    return threadPool->size();
  }
#endif

  __dllexport TaskSchedulerTBB* TaskSchedulerTBB::instance() 
  {
    if (g_instance == NULL) {
      g_instance = new TaskSchedulerTBB;
      g_instance->refInc();
    }
    return g_instance;
  }

  void TaskSchedulerTBB::create(size_t numThreads, bool set_affinity)
  {
    if (threadPool) THROW_RUNTIME_ERROR("Embree threads already running.");
    threadPool = new TaskSchedulerTBB::ThreadPool(numThreads,set_affinity);
  }

  void TaskSchedulerTBB::destroy() {
    delete threadPool; threadPool = nullptr;
  }

  __dllexport ssize_t TaskSchedulerTBB::allocThreadIndex()
  {
    size_t threadIndex = atomic_add(&threadCounter,1);
    assert(threadIndex < MAX_THREADS);
    return threadIndex;
  }

  void TaskSchedulerTBB::join()
  {
    mutex.lock();
    size_t threadIndex = atomic_add(&threadCounter,1);
    assert(threadIndex < MAX_THREADS);
    condition.wait(mutex, [&] () { return hasRootTask; });
    mutex.unlock();
    thread_loop(threadIndex);
  }

  void TaskSchedulerTBB::reset() {
    hasRootTask = false;
  }

  void TaskSchedulerTBB::wait_for_threads(size_t threadCount)
  {
    while (threadCounter < threadCount-1)
      __pause_cpu();
  }

  __dllexport TaskSchedulerTBB::Thread* TaskSchedulerTBB::thread() {
    return thread_local_thread;
  }

  __dllexport TaskSchedulerTBB::Thread* TaskSchedulerTBB::swapThread(Thread* thread) 
  {
    Thread* old = thread_local_thread;
    thread_local_thread = thread;
    return old;
  }

  __dllexport void TaskSchedulerTBB::wait() 
  {
    Thread* thread = TaskSchedulerTBB::thread();
    if (thread == nullptr) return;
    while (thread->tasks.execute_local(*thread,thread->task)) {};
  }

  void TaskSchedulerTBB::thread_loop(size_t threadIndex)
  {
    /* allocate thread structure */
    Thread thread(threadIndex,this);
    threadLocal[threadIndex] = &thread;
    Thread* oldThread = swapThread(&thread);

    /* main thread loop */
    while (anyTasksRunning > 0)
    {
      steal_loop(thread,
                 [&] () { return anyTasksRunning > 0; },
                 [&] () { 
                   atomic_add(&anyTasksRunning,+1);
                   while (thread.tasks.execute_local(thread,nullptr));
                   atomic_add(&anyTasksRunning,-1);
                 });
    }

    threadLocal[threadIndex] = nullptr;
    swapThread(oldThread);

    /* wait for all threads to terminate */
    atomic_add(&threadCounter,-1);
    while (threadCounter > 0)
      yield();
  }

  bool TaskSchedulerTBB::steal_from_other_threads(Thread& thread)
  {
    const size_t threadIndex = thread.threadIndex;
    const size_t threadCount = this->threadCounter;

    for (size_t i=1; i<threadCount; i++) 
    {
      __pause_cpu(32);
      size_t otherThreadIndex = threadIndex+i;
      if (otherThreadIndex >= threadCount) otherThreadIndex -= threadCount;

      Thread* othread = threadLocal[otherThreadIndex];
      if (!othread)
        continue;

      if (othread->tasks.steal(thread)) 
        return true;      
    }

    return false;
  }

  __dllexport void TaskSchedulerTBB::startThreads() {
    threadPool->startThreads();
  }

  __dllexport void TaskSchedulerTBB::addScheduler(const Ref<TaskSchedulerTBB>& scheduler) {
    threadPool->add(scheduler);
  }

  __dllexport void TaskSchedulerTBB::removeScheduler(const Ref<TaskSchedulerTBB>& scheduler) {
    threadPool->remove(scheduler);
  }
}
