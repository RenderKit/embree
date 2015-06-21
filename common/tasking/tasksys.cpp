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

#include "../sys/platform.h"
#include "../../kernels/algorithms/parallel_for.h" 

#if defined(TASKING_LOCKSTEP)
#include "taskscheduler_mic.h"
#endif

#if defined(TASKING_TBB_INTERNAL) || defined(TASKING_TBB)
#include "taskscheduler_tbb.h"
#endif

namespace embree
{
  /* Signature of ispc-generated 'task' functions */
  typedef void (*TaskFuncType)(void* data, int threadIndex, int threadCount, int taskIndex, int taskCount);

#if defined(TASKING_LOCKSTEP)

  struct ISPCTask
  {
  public:
    ISPCTask (TaskScheduler::Event* event, TaskFuncType func, void* data, int count)
      : task(event,_run,this,count,_finish,this,"ISPCTask"), func(func), data(data) {}

    ~ISPCTask () {
      if (data) _mm_free(data);
    }

    TASK_RUN_FUNCTION(ISPCTask,run);
    TASK_COMPLETE_FUNCTION(ISPCTask,finish);
    
  public:
    TaskScheduler::Task task;
    TaskFuncType func;
    void* data;
  };

  void ISPCTask::run(size_t threadIndex, size_t threadCount, size_t taskIndex, size_t taskCount, TaskScheduler::Event* event) {
    func(data,threadIndex,/* threadCount */ TaskScheduler::getNumThreads() ,taskIndex,taskCount);
  }

  void ISPCTask::finish(size_t threadIndex, size_t threadCount, TaskScheduler::Event* event) {
    delete this;
  }

  extern "C" __dllexport void* ISPCAlloc(void** taskPtr, int64_t size, int32_t alignment) {
    if (*taskPtr == nullptr) *taskPtr = new TaskScheduler::EventSync;
    return (char*)_mm_malloc(size,alignment); 
  }

  extern "C" __dllexport void ISPCLaunch(void** taskPtr, void* func, void* data, int count) {      
    ISPCTask* ispcTask = new ISPCTask((TaskScheduler::Event*)(*taskPtr),(TaskFuncType)func,data,count);
    TaskScheduler::addTask(-1, TaskScheduler::GLOBAL_BACK, &ispcTask->task);
  }
  
  extern "C" __dllexport void ISPCSync(void* task) { // FIXME: for join other tasks would need threadIndex here
    ((TaskScheduler::EventSync*)task)->sync(); 
    delete (TaskScheduler::EventSync*)task;
  }

#endif

#if defined(TASKING_TBB) || defined(TASKING_TBB_INTERNAL)

  extern "C" __dllexport void* ISPCAlloc(void** taskPtr, int64_t size, int32_t alignment) 
  {
    if (*taskPtr == nullptr) *taskPtr = new std::vector<void*>;
    std::vector<void*>* lst = (std::vector<void*>*)(*taskPtr);
    void* ptr = _mm_malloc(size,alignment);
    lst->push_back(ptr);
    return ptr;
  }

 extern "C" __dllexport void ISPCSync(void* task) 
  {
    std::vector<void*>* lst = (std::vector<void*>*)task;
    for (size_t i=0; i<lst->size(); i++) _mm_free((*lst)[i]);
    delete lst;
  }

#endif

#if defined(TASKING_TBB)

  extern "C" __dllexport void ISPCLaunch(void** taskPtr, void* func, void* data, int count) 
  {      
    parallel_for(size_t(0), size_t(count),[&] (const range<size_t>& r) {
        const size_t threadIndex = TaskSchedulerTBB::threadIndex();
        const size_t threadCount = TaskSchedulerTBB::threadCount();
        for (size_t i=r.begin(); i<r.end(); i++) ((TaskFuncType)func)(data,threadIndex,threadCount,i,count);
      });
  }
#endif  
 

#if defined(TASKING_TBB_INTERNAL)

  extern "C" __dllexport void ISPCLaunch(void** taskPtr, void* func, void* data, int count) 
  {      
    parallel_for(size_t(0), size_t(count), [&] (const range<size_t>& r) {
        const size_t threadIndex = TaskSchedulerTBB::threadIndex();
        const size_t threadCount = TaskSchedulerTBB::threadCount();
        for (size_t i=r.begin(); i<r.end(); i++) 
          ((TaskFuncType)func)(data,threadIndex,threadCount,i,count);
      });
  }

#endif
}
