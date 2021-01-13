// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../../../common/algorithms/parallel_for.h"

namespace embree
{
  /* Signature of ispc-generated 'task' functions */
  typedef void (*ISPCTaskFunc)(void* data, int threadIndex, int threadCount, int taskIndex, int taskCount);

  extern "C" void* ISPCAlloc(void** taskPtr, int64_t size, int32_t alignment)
  {
    if (*taskPtr == nullptr) *taskPtr = new std::vector<void*>;
    std::vector<void*>* lst = (std::vector<void*>*)(*taskPtr);
    void* ptr = alignedMalloc((size_t)size,alignment);
    lst->push_back(ptr);
    return ptr;
  }

  extern "C" void ISPCSync(void* task)
  {
    std::vector<void*>* lst = (std::vector<void*>*)task;
    for (size_t i=0; i<lst->size(); i++) alignedFree((*lst)[i]);
    delete lst;
  }

  extern "C" void ISPCLaunch(void** taskPtr, void* func, void* data, int count)
  {
    parallel_for(0, count,[&] (const range<int>& r) {
        const int threadIndex = (int) TaskScheduler::threadIndex();
        const int threadCount = (int) TaskScheduler::threadCount();
        for (int i=r.begin(); i<r.end(); i++)
          ((ISPCTaskFunc)func)(data,threadIndex,threadCount,i,count);
      });
  }
}
