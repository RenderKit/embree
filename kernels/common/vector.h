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

#include "default.h"

namespace embree
{
  /*! invokes the memory monitor callback */
  void memoryMonitor(ssize_t bytes, bool post);

  /*! allocator that performs aligned monitored allocations */
  template<typename T, size_t alignment = 64>
    struct aligned_monitored_allocator
    {
      typedef T value_type;
      typedef T* pointer;
      typedef const T* const_pointer;
      typedef T& reference;
      typedef const T& const_reference;
      typedef std::size_t size_type; // FIXME: also use std::size_t type under windows if available
      typedef std::ptrdiff_t difference_type;

      __forceinline pointer allocate( size_type n ) 
      {
        memoryMonitor(n*sizeof(T),false);
        return (pointer) alignedMalloc(n*sizeof(value_type),alignment);
      }

      __forceinline void deallocate( pointer p, size_type n ) 
      {
        alignedFree(p);
        memoryMonitor(-n*sizeof(T),true);
      }

      __forceinline void construct( pointer p, const_reference val ) {
        new (p) T(val);
      }

      __forceinline void destroy( pointer p ) {
        p->~T();
      }
    };
}

/*! instantiate vector using monitored aligned allocations */
#define vector_t mvector
#define allocator_t aligned_monitored_allocator<T,std::alignment_of<T>::value>
#include "../../common/sys/vector_t.h"
#undef vector_t
#undef allocator_t

namespace embree
{
  /*! monitored vector */
  //template<typename T> // FIXME: unfortunately not supported in VS2012
  //using mvector = vector_t<T,aligned_monitored_allocator<T,std::alignment_of<T>::value> >;
}
