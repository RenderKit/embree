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

#include "mutex.h"

#if defined(__WIN32__) && !defined(PTHREADS_WIN32)

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

namespace embree
{
  MutexSys::MutexSys( void ) { mutex = new CRITICAL_SECTION; InitializeCriticalSection((CRITICAL_SECTION*)mutex); }
  MutexSys::~MutexSys( void ) { DeleteCriticalSection((CRITICAL_SECTION*)mutex); delete (CRITICAL_SECTION*)mutex; }
  void MutexSys::lock( void ) { EnterCriticalSection((CRITICAL_SECTION*)mutex); }
  bool MutexSys::try_lock( void ) { return TryEnterCriticalSection((CRITICAL_SECTION*)mutex); }
  void MutexSys::unlock( void ) { LeaveCriticalSection((CRITICAL_SECTION*)mutex); }
}
#endif

#if defined(__UNIX__) || defined(PTHREADS_WIN32)
#include <pthread.h>
namespace embree
{
  /*! system mutex using pthreads */
  MutexSys::MutexSys( void ) 
  { 
    mutex = new pthread_mutex_t; 
    if (pthread_mutex_init((pthread_mutex_t*)mutex, nullptr) != 0)
      THROW_RUNTIME_ERROR("pthread_mutex_init failed");
  }
  
  MutexSys::~MutexSys( void ) 
  { 
    if (pthread_mutex_destroy((pthread_mutex_t*)mutex) != 0)
      THROW_RUNTIME_ERROR("pthread_mutex_destroy failed");
    
    delete (pthread_mutex_t*)mutex; 
  }
  
  void MutexSys::lock( void ) 
  { 
    if (pthread_mutex_lock((pthread_mutex_t*)mutex) != 0) 
      THROW_RUNTIME_ERROR("pthread_mutex_lock failed");
  }

  bool MutexSys::try_lock( void ) { 
    return pthread_mutex_trylock((pthread_mutex_t*)mutex) == 0;
  }
  
  void MutexSys::unlock( void ) 
  { 
    if (pthread_mutex_unlock((pthread_mutex_t*)mutex) != 0)
      THROW_RUNTIME_ERROR("pthread_mutex_unlock failed");
  }
};
#endif

namespace embree
{
  // ========== RW MUTEX =============
  void RWMutex::read_lock()
  {
    while(1)
      {
#if defined(__MIC__)
	prefetchL1EX((void*)&data);
#endif

        unsigned int d = getData();
        if (!busy_w(d))
          {
            unsigned int r = update_atomic_add(SINGLE_READER);
            if (!busy_w(r)) break; // -request
            update_atomic_add(-SINGLE_READER);
          }
        pause();
      }
  }

  void RWMutex::read_unlock()
  {
#if defined(__MIC__)
    prefetchL1EX((void*)&data);
#endif
    update_atomic_add(-SINGLE_READER);            
  }
  
  void RWMutex::write_lock()
  {
    while(1)
      {
#if defined(__MIC__)
	prefetchL1EX((void*)&data);
#endif
        unsigned int d = getData();
        if (!busy_rw(d))
          {
            if (update_atomic_cmpxchg(d,WRITER)==d) break;           
          }
        else if ( !(d & WRITER_REQUEST) )
          {
            update_atomic_or(WRITER_REQUEST);
          }
        pause();
      }
  }
  
  void RWMutex::write_unlock()
  {
#if defined(__MIC__)
    prefetchL1EX((void*)&data);
#endif
    update_atomic_and(READERS);
  }
  
  void RWMutex::upgrade_read_to_write_lock()
  {
    read_unlock();
    write_lock();
  }

  void RWMutex::upgrade_write_to_read_lock()
  {
    update_atomic_add(SINGLE_READER-WRITER);            
  }

  void RWMutex::pause()
   {
     unsigned int DELAY_CYCLES = 64;

#if !defined(__MIC__)
     _mm_pause(); 
     _mm_pause();
#else
     _mm_delay_32(DELAY_CYCLES); 
#endif      
   }
 
  bool RWMutex::try_write_lock()
  {
    unsigned int d = getData();
    if (busy_rw(d)) return false;
    if (update_atomic_cmpxchg(d,WRITER)==d) return true;
    return false;
  }

  bool RWMutex::try_read_lock()
  {
    unsigned int d = getData();
    if (busy_w(d)) return false;

    unsigned int r = update_atomic_add(SINGLE_READER);
    if (!busy_w(r)) return true; // -request
    update_atomic_add(-SINGLE_READER);
    return false;
  }
}
