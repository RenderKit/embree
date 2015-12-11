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
  class BVH4Factory;
  class BVH8Factory;
  class InstanceFactory;

  class Device : public State, public MemoryMonitorInterface
  {
    ALIGNED_CLASS;

  public:

    /*! Device construction */
    Device (const char* cfg, bool singledevice);

    /*! Device destruction */
    virtual ~Device ();

    /*! prints info about the device */
    void print();

    /*! sets the error code */
    void setErrorCode(RTCError error);

    /*! returns and clears the error code */
    RTCError getErrorCode();

    /*! returns thread local error code storage location */
    RTCError* getError();

    /*! processes error codes, do not call directly */
    void process_error(RTCError error, const char* str);

    /*! invokes the memory monitor callback */
    void memoryMonitor(ssize_t bytes, bool post);

    /*! sets the size of the software cache. */
    void setCacheSize(size_t bytes);

    /*! configures some parameter */
    void setParameter1i(const RTCParameter parm, ssize_t val);

  private:

    /*! initializes the tasking system */
    void initTaskingSystem(size_t numThreads);

    /*! configures tasking system with maximal number of thread set by any device */
    void configureTaskingSystem();

    /*! shuts down the tasking system */
    void exitTaskingSystem();

  public:
    bool singledevice;      //!< true if this is the device created implicitely through rtcInit

    InstanceFactory* instance_factory;

#if !defined(__MIC__)
    BVH4Factory* bvh4_factory;
#endif

#if defined(__TARGET_AVX__)
    BVH8Factory* bvh8_factory;
#endif

#if USE_TASK_ARENA
  tbb::task_arena* arena;
#endif
  };
}
