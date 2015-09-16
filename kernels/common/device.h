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

#include "default.h"
#include "state.h"

namespace embree
{
  class Device : public State, public MemoryMonitorInterface
  {
    ALIGNED_CLASS;

  public:

    /*! Device construction */
    Device (const char* cfg, bool singledevice);

    /*! Device destruction */
    ~Device ();

    /*! prints info about the device */
    void print();

    /*! processes error codes, do not call directly */
    void process_error(RTCError error, const char* str);

    /*! invokes the memory monitor callback */
    void memoryMonitor(ssize_t bytes, bool post);

    /*! sets the size of the software cache. */
    void setCacheSize(size_t bytes);

    /*! sets the cache size to the maximum requested by any device */
    void updateCacheSize();
    
    /*! configures some parameter */
    void setParameter1i(const RTCParameter parm, ssize_t val);

  public:

#if USE_TASK_ARENA
  tbb::task_arena* arena;
#endif

  bool singledevice;
  };
}
