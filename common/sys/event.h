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

#include "mutex.h"
#include "condition.h"

namespace embree
{
  class EventSys
  {
  public:
    __forceinline EventSys() 
      : event(false) {}
    
    __forceinline void reset() 
    {
      mutex.lock();
      event = false;
      mutex.unlock();
    }

    __forceinline void signal() 
    {
      mutex.lock();
      event = true;
      condition.broadcast(); // this broadcast has to be protected!
      mutex.unlock();
    }

    __forceinline void wait() 
    {
      mutex.lock();
      while (!event) condition.wait(mutex);
      mutex.unlock();

    }

  protected:
    volatile bool event;
    MutexSys mutex;
    ConditionSys condition;
  };

  class EventActive
  {
  public:
    __forceinline EventActive () 
      : event(false) {}
    
    __forceinline void reset() 
    {
      __memory_barrier();
      event = false;
      __memory_barrier();
    }

    __forceinline void signal() 
    {
      __memory_barrier();
      event = true;
      __memory_barrier();
    }

    __forceinline void wait() {
      while (!event) __pause_cpu(1024);
    }

  protected:
    volatile bool event;
  };

#if defined(__MIC__)
  typedef EventActive Event;
#else
  typedef EventSys Event;
#endif
}
