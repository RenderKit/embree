// ======================================================================== //
// Copyright 2009-2016 Intel Corporation                                    //
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

#if !defined(__WIN32__)
#error PPL tasking system only available under windows
#endif

#include <ppl.h>

namespace embree
{  
  struct TaskScheduler
  {
    /*! initializes the task scheduler */
    static void create(size_t numThreads, bool set_affinity);

    /*! destroys the task scheduler again */
    static void destroy();
    
    /* returns the index of the current thread */
    static __forceinline size_t threadIndex()
    {
      size_t ID = GetCurrentThreadId();
	  //PRINT(ID);
	  return ID;
    }
  
    /* returns the total number of threads */
    static __forceinline size_t threadCount() {
	  size_t num = GetMaximumProcessorCount(ALL_PROCESSOR_GROUPS);
	  //PRINT(num);
      return num;
    }

  private:
    static size_t g_numThreads;
  };
};

