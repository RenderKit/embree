// ======================================================================== //
// Copyright 2009-2019 Intel Corporation                                    //
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

#if defined(EMBREE_DPCPP_SUPPORT)
#define CL_TARGET_OPENCL_VERSION 220
#define SYCL_SIMPLE_SWIZZLES
#include <CL/sycl.hpp>
#include <CL/sycl/builtins.hpp>


#ifdef __SYCL_DEVICE_ONLY__
#define GLOBAL __attribute__((ocl_global))
#define LOCAL  __attribute__((ocl_local))

extern int   work_group_reduce_add(int x);
extern float work_group_reduce_min(float x);
extern float work_group_reduce_max(float x);

//extern uint cl_intel_sub_group_ballot(bool valid);

extern float atomic_min(volatile GLOBAL float *p, float val);
extern float atomic_min(volatile LOCAL  float *p, float val);
extern float atomic_max(volatile GLOBAL float *p, float val);
extern float atomic_max(volatile LOCAL  float *p, float val);

extern "C" uint intel_sub_group_ballot(bool valid);

#else
#define GLOBAL 
#define LOCAL 

/* dummy functions for host */
inline int   work_group_reduce_add(int x) { return x; }
inline float work_group_reduce_min(float x) { return x; }
inline float work_group_reduce_max(float x) { return x; }

inline float atomic_min(volatile float *p, float val) { return val; };
inline float atomic_max(volatile float *p, float val) { return val; };

inline uint intel_sub_group_ballot(bool valid) { return 0; }

#endif


constexpr cl::sycl::access::target sycl_local    = cl::sycl::access::target::local;
constexpr cl::sycl::access::mode sycl_read_write = cl::sycl::access::mode::read_write;
constexpr cl::sycl::access::mode sycl_read       = cl::sycl::access::mode::read;
constexpr cl::sycl::access::mode sycl_write      = cl::sycl::access::mode::write;

using cl::sycl::float16;
using cl::sycl::float8;
using cl::sycl::float4;
using cl::sycl::float3;
using cl::sycl::float2;
using cl::sycl::int16;
using cl::sycl::int8;
using cl::sycl::int4;
using cl::sycl::int3;
using cl::sycl::int2;
using cl::sycl::uint16;
using cl::sycl::uint8;
using cl::sycl::uint4;
using cl::sycl::uint3;
using cl::sycl::uint2;
using cl::sycl::uchar16;
using cl::sycl::uchar8;
using cl::sycl::uchar4;
using cl::sycl::uchar3;
using cl::sycl::uchar2;
using cl::sycl::uchar;
using cl::sycl::ushort16;
using cl::sycl::ushort8;
using cl::sycl::ushort4;
using cl::sycl::ushort3;
using cl::sycl::ushort2;

using cl::sycl::fmax;
using cl::sycl::fmin;

namespace embree
{
  namespace gpu
  {
    template<typename T, typename M>
      __forceinline T cselect(const M &mask, const T &a, const T &b)
    {
      return cl::sycl::select(b,a,mask);
    }

    template<typename T>
      __forceinline T cfma(const T &a, const T &b, const T &c)
      {
	return cl::sycl::fma(a,b,c);
      }
    
    template<typename T>
      __forceinline T lerp(const T &a, const T &b, const float t) {      
      return cl::sycl::fma(T(1.0f-t),a,T(t)*b);
    }
    
      
    __forceinline float halfarea(const cl::sycl::float3 &d)
    {
      return cl::sycl::fma((float)d.x(),((float)d.y()+(float)d.z()),(float)d.y()*(float)d.z());
    }

    __forceinline float halfarea(const cl::sycl::float4 &d)
    {
      return cl::sycl::fma((float)d.x(),((float)d.y()+(float)d.z()),(float)d.y()*(float)d.z());
    }
    
    __forceinline float area(const cl::sycl::float3 &d)
    {
      return halfarea(d) * 2.0f;
    }

    __forceinline float area(const cl::sycl::float4 &d)
    {
      return halfarea(d) * 2.0f;
    }

    __forceinline float dot3(const float3 &a,
			     const float3 &b)
    {
      return cl::sycl::dot(a,b);
    }    
    
    template<typename T, cl::sycl::access::address_space space>
      static __forceinline uint atomic_add(T *dest, const T count=1)
      {
	cl::sycl::multi_ptr<T,space> ptr(dest);
	cl::sycl::atomic<T,space> counter(ptr);
	return atomic_fetch_add(counter,count);      
      }

    template<typename T>
      static __forceinline uint as_uint(T t)
      {
	return __builtin_bit_cast(uint,t);
      }

    template<typename T>
      static __forceinline uint as_int(T t)
      {
	return __builtin_bit_cast(int,t);
      }
    
    template<typename T>
      static __forceinline float as_float(T t)
      {
	return __builtin_bit_cast(float,t);
      }

    __forceinline uint bitInterleave(const uint4 in)
    {
      uint x = in.x(), y = in.y(), z = in.z();
      x = (x | (x << 16)) & 0x030000FF;
      x = (x | (x <<  8)) & 0x0300F00F;
      x = (x | (x <<  4)) & 0x030C30C3;
      x = (x | (x <<  2)) & 0x09249249;

      y = (y | (y << 16)) & 0x030000FF;
      y = (y | (y <<  8)) & 0x0300F00F;
      y = (y | (y <<  4)) & 0x030C30C3;
      y = (y | (y <<  2)) & 0x09249249;

      z = (z | (z << 16)) & 0x030000FF;
      z = (z | (z <<  8)) & 0x0300F00F;
      z = (z | (z <<  4)) & 0x030C30C3;
      z = (z | (z <<  2)) & 0x09249249;

      return x | (y << 1) | (z << 2);
    }
    
  };
};


#endif
