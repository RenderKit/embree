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
#include "mutex.h"

#include <vector>

namespace embree
{
  /*! type for thread */
  typedef struct opaque_thread_t* thread_t;

  /*! signature of thread start function */
  typedef void (*thread_func)(void*);

  /*! creates a hardware thread running on specific logical thread */
  thread_t createThread(thread_func f, void* arg, size_t stack_size = 0, ssize_t threadID = -1);

  /*! set affinity of the calling thread */
  void setAffinity(ssize_t affinity);

  /*! the thread calling this function gets yielded */
  void yield();

  /*! waits until the given thread has terminated */
  void join(thread_t tid);

  /*! destroy handle of a thread */
  void destroyThread(thread_t tid);

  /*! type for handle to thread local storage */
  typedef struct opaque_tls_t* tls_t;

  /*! creates thread local storage */
  tls_t createTls();

  /*! set the thread local storage pointer */
  void setTls(tls_t tls, void* const ptr);

  /*! return the thread local storage pointer */
  void* getTls(tls_t tls);

  /*! destroys thread local storage identifier */
  void destroyTls(tls_t tls);

  /*! manages thread local variables */
  template<typename Type>
  struct ThreadLocalData
  {
  public:

    __forceinline ThreadLocalData (void* init) 
      : ptr(nullptr), init(init) {}

    __forceinline ~ThreadLocalData () {
      clear();
    }

    __forceinline void clear() 
    {
      if (ptr) destroyTls(ptr); ptr = nullptr;
      for (size_t i=0; i<threads.size(); i++)
	delete threads[i];
      threads.clear();
    }

    /*! disallow copy */
    //ThreadLocalData(const ThreadLocalData&) = delete;
    //ThreadLocalData& operator=(const ThreadLocalData&) = delete;

    __forceinline void reset()
    {
      for (size_t i=0; i<threads.size(); i++)
	threads[i]->reset();
    }
    
    __forceinline Type* get() const
    {
      if (ptr == nullptr) {
	Lock<AtomicMutex> lock(mutex);
	if (ptr == nullptr) ptr = createTls();
      }
      Type* lptr = (Type*) getTls(ptr);
      if (unlikely(lptr == nullptr)) {
	setTls(ptr,lptr = new Type(init));
	Lock<AtomicMutex> lock(mutex);
	threads.push_back(lptr);
      }
      return lptr;
    }

    __forceinline const Type& operator  *( void ) const { return *get(); }
    __forceinline       Type& operator  *( void )       { return *get(); }
    __forceinline const Type* operator ->( void ) const { return  get(); }
    __forceinline       Type* operator ->( void )       { return  get(); }
    
    
  private:
    mutable tls_t ptr;
    void* init;
    mutable AtomicMutex mutex;
  public:
    mutable std::vector<Type*> threads;
  };

#if defined(__MIC__)
  void printThreadInfo();
#endif
}
