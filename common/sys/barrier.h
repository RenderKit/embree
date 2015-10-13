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

#include "intrinsics.h"
#include "sysinfo.h"

namespace embree
{
  /*! system barrier using operating system */
  class BarrierSys
  {
  public:

    /*! construction / destruction */
    BarrierSys (size_t N = 0);
    ~BarrierSys ();

    /*! intializes the barrier with some number of threads */
    void init(size_t count);

    /*! lets calling thread wait in barrier */
    void wait();

  private:
    void* opaque;
  };

  /*! fast active barrier using atomitc counter */
  struct __aligned(64) BarrierActive 
  {
  public:
    BarrierActive () 
      : cntr(0) {}
    
    void reset() {
      cntr = 0;
    }

    void wait (size_t numThreads) {
      atomic_add((atomic_t*)&cntr,1);
      while (cntr != numThreads) __pause_cpu();
    }

  private:
    volatile atomic_t cntr;
  };

  /*! fast active barrier that does not require initialization to some number of threads */
  struct __aligned(64) BarrierActiveAutoReset
  {
  public:
    BarrierActiveAutoReset () 
      : cntr0(0), cntr1(0) {}

    void wait (size_t threadCount) 
    {
      atomic_add((atomic_t*)&cntr0,1);
      while (cntr0 != threadCount) __pause_cpu();
      atomic_add((atomic_t*)&cntr1,1);
      while (cntr1 != threadCount) __pause_cpu();
      atomic_add((atomic_t*)&cntr0,-1);
      while (cntr0 != 0) __pause_cpu();
      atomic_add((atomic_t*)&cntr1,-1);
      while (cntr1 != 0) __pause_cpu();
    }

  private:
    volatile atomic_t cntr0;
    volatile atomic_t cntr1;
  };

  class __aligned(64) LinearBarrierActive
  {
  public:

    /*! construction and destruction */
    LinearBarrierActive (size_t threadCount = 0);
    ~LinearBarrierActive();
    
    /*! intializes the barrier with some number of threads */
    void init(size_t threadCount);
    
    /*! thread with threadIndex waits in the barrier */
    void wait (const size_t threadIndex);
    
  private:
    volatile unsigned char* count0;
    volatile unsigned char* count1; 
    volatile unsigned int mode;
    volatile unsigned int flag0;
    volatile unsigned int flag1;
    volatile unsigned int threadCount;
  };
  



#if defined (__MIC__)

  class __aligned(64) QuadTreeBarrier
  {
  public:

    class __aligned(64) CoreSyncData {
    public:
      volatile unsigned char threadState[2][4];
      volatile unsigned int mode;
      volatile unsigned int data[16-3];

      void init();

      void pause(unsigned int &cycles);

      __forceinline void prefetchEx() { 
	prefetchL1EX((char*)threadState); 
      }

      __forceinline void prefetch() { 
	prefetchL1((char*)threadState); 
      }
    
      void switchModeAndSendRunSignal(const unsigned int m);

      void setThreadStateToDone(const unsigned int m, const unsigned int threadID);

      bool allThreadsDone(const unsigned int m, const unsigned int orMask = 0);

      bool threadDone(const unsigned int m, const unsigned int threadID);
 

      void waitForAllThreadsOnCore(const unsigned int m);

      void waitForAllOtherThreadsOnCore(const unsigned int m, const unsigned int threadID);

      void waitForThreadReceivesRunSignal(const unsigned int m, const unsigned int threadID);
    };

    QuadTreeBarrier();

    void init(size_t cntr);

    void wait(const size_t threadID, const size_t MAX_THREADS_SYNC);

    void syncWithReduction(const size_t threadID, 
                           const size_t MAX_THREADS_SYNC,
                           void (* reductionFct)(const size_t currentThreadID,
                                                 const size_t childThreadID,
                                                 void *ptr),
                           void *ptr);
    

  public:  
    __aligned(64) CoreSyncData data[MAX_MIC_CORES]; // == one cacheline per core ==
  };
#endif
}
