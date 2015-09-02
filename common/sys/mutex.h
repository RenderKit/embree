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

#include "platform.h"
#include "intrinsics.h"

namespace embree
{
  /*! system mutex */
  class MutexSys {
    friend struct ConditionImplementation;
  public:
    MutexSys( void );
    ~MutexSys( void );

    void lock( void );
    bool try_lock( void );
    void unlock( void );

  protected:
    void* mutex;
  };

  /*! spinning mutex */
  class AtomicMutex
  {
  public:
 
    static const size_t MAX_MIC_WAIT_CYCLES = 256;
    AtomicMutex ()
      : flag(0) {}

    __forceinline bool isLocked() {
      return flag == 1;
    }

    __forceinline void lock()
    {
      unsigned int wait = 128;
      while(1) {
        __memory_barrier();
	while (flag == 1) { // read without atomic op first
#if !defined(__MIC__)
	  _mm_pause(); 
	  _mm_pause();
#else
	  _mm_delay_32(wait); 
	  wait += wait;  
	  if (wait > MAX_MIC_WAIT_CYCLES) wait = MAX_MIC_WAIT_CYCLES;  
#endif
	}
        __memory_barrier();
	if (atomic_cmpxchg(&flag,0,1) == 0) break;
#if defined(__MIC__)
	_mm_delay_32(wait); 
#endif
      }
    }

    __forceinline bool try_lock()
    {
      if (flag == 1) return false;
      return atomic_cmpxchg(&flag,0,1) == 0;
    }

    __forceinline void unlock() 
    {
      __memory_barrier();
      flag = 0;
      __memory_barrier();
    }


    __forceinline void wait_until_unlocked() 
    {
      unsigned int wait = 128;
      __memory_barrier();
      while(flag == 1)
	{
#if !defined(__MIC__)
	  _mm_pause(); 
	  _mm_pause();
#else
	  _mm_delay_32(wait); 
#endif	
	}
      __memory_barrier();
    }

    __forceinline void reset(int i = 0) 
    {
      __memory_barrier();
      flag = i;
      __memory_barrier();
    }

  public:
    volatile int flag;
  };

  class __aligned(64) AlignedAtomicMutex : public AtomicMutex
  {
  public:
    volatile unsigned int index;
    atomic32_t m_counter;
    volatile char align[64-3*sizeof(int)]; 
  
    AlignedAtomicMutex() {
      m_counter = 0;
      index = 0;
    }

    __forceinline void resetCounter(unsigned int i = 0) {
        __memory_barrier();
      *(volatile unsigned int*)&m_counter = i;
        __memory_barrier();
    }

    __forceinline unsigned int inc() {
      return atomic_add(&m_counter,1);
    }

    __forceinline unsigned int dec() {
      return atomic_add(&m_counter,-1);
    }

    __forceinline unsigned int val() {
      __memory_barrier();
      return m_counter;
    };
  };

  /*! safe mutex lock and unlock helper */
  template<typename Mutex> class Lock {
  public:
    Lock (Mutex& mutex) : mutex(mutex) { mutex.lock(); }
    ~Lock() { mutex.unlock(); }
  protected:
    Mutex& mutex;
  };

  /*! safe mutex try_lock and unlock helper */
  template<typename Mutex> class TryLock {
  public:
    TryLock (Mutex& mutex) : mutex(mutex), locked(mutex.try_lock()) {}
    ~TryLock() { if (locked) mutex.unlock(); }
    __forceinline bool isLocked() const { return locked; }
  protected:
    Mutex& mutex;
    bool locked;
  };

  /*! safe mutex try_lock and unlock helper */
  template<typename Mutex> class AutoUnlock {
  public:
    AutoUnlock (Mutex& mutex) : mutex(mutex), locked(false) {}
    ~AutoUnlock() { if (locked) mutex.unlock(); }
    __forceinline void lock() { locked = true; mutex.lock(); }
    __forceinline bool isLocked() const { return locked; }
  protected:
    Mutex& mutex;
    bool locked;
  };

  class TicketMutex
  {
  public:
 
    static const size_t MAX_MIC_WAIT_CYCLES = 1024;
    TicketMutex () { reset(); }

    __forceinline bool isLocked() {
      return tickets != threads;
    }

    __forceinline void lock()
    {
      const int16_t i = atomic_add(&threads, 1);

      unsigned int wait = 128;	
      while (tickets != i) 
	{
#if !defined(__MIC__)
	  _mm_pause(); 
	  _mm_pause();
#else
	  _mm_delay_32(wait); 
	  wait += wait;  
	  if (wait > MAX_MIC_WAIT_CYCLES) wait = MAX_MIC_WAIT_CYCLES;  
#endif	  
	}
    }

    __forceinline void unlock() 
    {
      __memory_barrier();
      tickets++;
      __memory_barrier();
    }

    __forceinline void reset() 
    {
      assert(sizeof(TicketMutex) == 4);
      __memory_barrier();
      threads = 0;
      tickets = 0;
      __memory_barrier();
    }

  public:
    volatile int16_t threads;     
    volatile int16_t tickets;
  };

 class MultipleReaderSingleWriterMutex
 {
 private:
#if defined(__WIN32__)
   AtomicMutex writer_mtx;
#else
   TicketMutex writer_mtx;
#endif
   volatile int readers;

 public:

 MultipleReaderSingleWriterMutex() : readers(0) {
     assert(sizeof(MultipleReaderSingleWriterMutex) == 8);
   }

   static const unsigned int DELAY_CYCLES = 1024;

   __forceinline void reset()
   {
     readers = 0;
     writer_mtx.reset();
   }
   __forceinline void pause()
   {
#if !defined(__MIC__)
     _mm_pause(); 
     _mm_pause();
#else
     _mm_delay_32(DELAY_CYCLES); 
#endif      
   }
    
   __forceinline void read_lock()
   {
     while(1)
       {
         atomic_add(&readers,1);
         if (likely(!writer_mtx.isLocked())) break;
         atomic_add(&readers,-1);
         while(writer_mtx.isLocked())
           pause();
       }
   }

   __forceinline void read_unlock()
   {
     atomic_add(&readers,-1);      
   }

   __forceinline void write_lock()
   {
     writer_mtx.lock();
     while(readers)
       pause();
   }

   __forceinline void write_unlock()
   {
     writer_mtx.unlock();
   }

   __forceinline void upgrade_write_to_read_lock()
   {
     atomic_add(&readers,1);     
     writer_mtx.unlock();
   }

 };

 class RWMutex
 {
 private:
   volatile unsigned int data;

   static const unsigned int WRITER          = 1;
   static const unsigned int WRITER_REQUEST  = 2;
   static const unsigned int SINGLE_READER   = 4;
   static const unsigned int READERS         = ~(WRITER|WRITER_REQUEST);   
   
   static __forceinline bool busy_rw(const unsigned int d) { return d & (WRITER|READERS); }
   static __forceinline bool busy_w (const unsigned int d) { return d & (WRITER|WRITER_REQUEST); }
   static __forceinline bool busy_r (const unsigned int d) { return d & READERS; }

   __forceinline unsigned int getData() { return data; }
   
   __forceinline int update_atomic_cmpxchg(const int old_data, const int new_data) {
     return atomic_cmpxchg((int*)&data,old_data,new_data);
   }

   __forceinline int update_atomic_or(int new_data) {
     return atomic_or((int*)&data,new_data);
   }

   __forceinline int update_atomic_and(int new_data) {
     return atomic_and((int*)&data,new_data);
   }

   __forceinline int update_atomic_add(int new_data) {
     return atomic_add((int*)&data,new_data);
   }



 public:

 RWMutex() : data(0) {}
   
   __forceinline void reset()
   {
     data = 0;
   }

   __forceinline bool hasInitialState() { return data == 0; }

   void pause();
    
   void read_lock();
   void read_unlock();
   void write_lock();
   void write_unlock();

   bool try_write_lock();
   bool try_read_lock();

   void upgrade_read_to_write_lock();
   void upgrade_write_to_read_lock();

   __forceinline unsigned int num_readers() {
     return (getData() & READERS) >> 2;
   }

 };

}
