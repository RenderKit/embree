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

#include "embree2/rtcore.h"

namespace embree
{
  extern size_t g_numThreads;
  extern size_t g_verbose;

  extern std::string g_tri_accel;
  extern std::string g_tri_builder;
  extern std::string g_tri_traverser;
  extern double g_tri_builder_replication_factor;

  extern std::string g_tri_accel_mb;
  extern std::string g_tri_builder_mb;
  extern std::string g_tri_traverser_mb;

  extern std::string g_hair_accel;
  extern std::string g_hair_builder;
  extern std::string g_hair_traverser;
  extern double g_hair_builder_replication_factor;

  extern std::string g_subdiv_accel;

  extern int g_scene_flags;
  extern size_t g_benchmark;
  extern float g_memory_preallocation_factor;

  /*! processes an error */
  void process_error(RTCError error, const char* code);

  typedef void (*ErrorFunc) ();

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

  /*! monitored vector */
  template<typename T>
    using mvector = vector_t<T,aligned_allocator<T,std::alignment_of<T>::value> >;

  /*! used to throw embree API errors */
  struct my_runtime_error : public std::exception
  {
    __forceinline my_runtime_error(RTCError error, const std::string& str)
      : error(error), str(str) {}
    
    ~my_runtime_error() throw() {}
    
    const char* what () const throw () {
      return str.c_str();
    }
    
    RTCError error;
    std::string str;
  };
  
#define THROW_MY_RUNTIME_ERROR(error,str)                               \
  throw my_runtime_error(error,std::string(__FILE__) + " (" + std::to_string((long long)__LINE__) + "): " + std::string(str));

   /* we consider floating point numbers in that range as valid input numbers */
#define VALID_FLOAT_RANGE  1.844E18f

  __forceinline bool inFloatRange(const float v) {
    return (v > -VALID_FLOAT_RANGE) && (v < +VALID_FLOAT_RANGE);
  };
  __forceinline bool inFloatRange(const Vec3fa& v) {
    return all(gt_mask(v,Vec3fa_t(-VALID_FLOAT_RANGE)) & lt_mask(v,Vec3fa_t(+VALID_FLOAT_RANGE)));
  };
#if defined(__SSE2__)
  __forceinline bool inFloatRange(const ssef& v) {
    return all((v > ssef(-VALID_FLOAT_RANGE)) & (v < ssef(+VALID_FLOAT_RANGE)));
  };
#endif
  __forceinline bool inFloatRange(const BBox3fa& v) {
    return all(gt_mask(v.lower,Vec3fa_t(-VALID_FLOAT_RANGE)) & lt_mask(v.upper,Vec3fa_t(+VALID_FLOAT_RANGE)));
  };

#define MODE_HIGH_QUALITY (1<<8)
#define LeafMode 0 // FIXME: remove
}
