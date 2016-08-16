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

#if defined(__WIN32__)
#  define NOMINMAX
#  if defined(__clang__) && !defined(__INTEL_COMPILER) 
#    define __MINGW64__ 1
#  endif
#endif

#define TBB_IMPLEMENT_CPP0X 0
#define __TBB_NO_IMPLICIT_LINKAGE 1
#define __TBBMALLOC_NO_IMPLICIT_LINKAGE 1
#include "tbb/tbb_config.h"
#undef TBB_USE_CAPTURED_EXCEPTION
#define TBB_USE_CAPTURED_EXCEPTION 0
#undef __TBB_EXCEPTION_PTR_PRESENT
#define __TBB_EXCEPTION_PTR_PRESENT 1
#include "tbb/tbb.h"

namespace embree
{
#  define SPAWN_BEGIN tbb::task_group __internal_task_group
#  define SPAWN(closure) __internal_task_group.run(closure)
#  define SPAWN_END __internal_task_group.wait();                       \
  if (tbb::task::self().is_cancelled())                                 \
    throw std::runtime_error("task group cancelled");
  
  struct TaskScheduler
  {
    /*! initializes the task scheduler */
    static void create(size_t numThreads, bool set_affinity);

    /*! destroys the task scheduler again */
    static void destroy();
    
    /* returns the index of the current thread */
    static __forceinline size_t threadIndex()
    {
#if TBB_INTERFACE_VERSION_MAJOR < 8
      return 0;
#else
      return tbb::task_arena::current_thread_index();
#endif
    }
  
    /* returns the total number of threads */
    static __forceinline size_t threadCount() {
      return tbb::task_scheduler_init::default_num_threads();
    }

  private:
    static size_t g_numThreads;
  };
};

