// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "../../../../common/sys/sycl.h"

constexpr sycl::access::target sycl_local    = sycl::access::target::local;
constexpr sycl::access::mode sycl_read_write = sycl::access::mode::read_write;
constexpr sycl::access::mode sycl_read       = sycl::access::mode::read;
constexpr sycl::access::mode sycl_write      = sycl::access::mode::write;

using sycl::float16;
using sycl::float8;
using sycl::float4;
using sycl::float3;
using sycl::float2;
using sycl::int16;
using sycl::int8;
using sycl::int4;
using sycl::int3;
using sycl::int2;
using sycl::uint16;
using sycl::uint8;
using sycl::uint4;
using sycl::uint3;
using sycl::uint2;
using sycl::uchar16;
using sycl::uchar8;
using sycl::uchar4;
using sycl::uchar3;
using sycl::uchar2;
using sycl::uchar;
using sycl::ushort16;
using sycl::ushort8;
using sycl::ushort4;
using sycl::ushort3;
using sycl::ushort2;

using sycl::fmax;
using sycl::fmin;

namespace embree
{
  namespace gpu
  {
    __forceinline size_t alignTo(const size_t size, const size_t alignment)
    {
      return ((size+alignment-1)/alignment)*alignment;
    }

    
    template<typename T>
      __forceinline T cfma(const T &a, const T &b, const T &c)
      {
	return sycl::fma(a,b,c);
      }

    template<typename T>
      __forceinline T lerp(const T &a, const T &b, const float t) {      
      return sycl::fma(T(1.0f-t),a,T(t)*b);
    }
      
    __forceinline float halfarea(const sycl::float3 &d)
    {
#if 1      
      return sycl::fma((float)d.x(),((float)d.y()+(float)d.z()),(float)d.y()*(float)d.z());
#else
      return sycl::fma(d.x(),d.y(),sycl::fma(d.x(),d.z(),d.y()*d.z()));
      
#endif      
    }

    __forceinline float halfarea(const sycl::float4 &d)
    {
      return sycl::fma((float)d.x(),((float)d.y()+(float)d.z()),(float)d.y()*(float)d.z());
    }
    
    __forceinline float area(const sycl::float3 &d)
    {
      return halfarea(d) * 2.0f;
    }

    __forceinline float area(const sycl::float4 &d)
    {
      return halfarea(d) * 2.0f;
    }

    __forceinline float dot3(const float3 &a,
			     const float3 &b)
    {
      return sycl::dot(a,b);
    }    


#if 1
    template<typename T>
      static __forceinline uint atomic_add_global(T *dest, const T count=1)
      {
        sycl::atomic_ref<T, sycl::memory_order::relaxed, sycl::memory_scope::device,sycl::access::address_space::global_space> counter(*dest);        
        return counter.fetch_add(count);      
      }

    template<typename T>
      static __forceinline uint atomic_min_global(T *dest, const T count=1)
      {
        sycl::atomic_ref<T, sycl::memory_order::relaxed, sycl::memory_scope::device,sycl::access::address_space::global_space> counter(*dest);        
        return counter.fetch_min(count);      
      }

    template<typename T>
      static __forceinline uint atomic_max_global(T *dest, const T count=1)
      {
        sycl::atomic_ref<T, sycl::memory_order::relaxed, sycl::memory_scope::device,sycl::access::address_space::global_space> counter(*dest);        
        return counter.fetch_max(count);      
      }

    template<typename T>
      static __forceinline uint atomic_add_local(T *dest, const T count=1)
      {
        sycl::atomic_ref<T, sycl::memory_order::relaxed, sycl::memory_scope::work_group,sycl::access::address_space::local_space> counter(*dest);        
        return counter.fetch_add(count);      
      }

    template<typename T>
      static __forceinline uint atomic_min_local(T *dest, const T count=1)
      {
        sycl::atomic_ref<T, sycl::memory_order::relaxed, sycl::memory_scope::work_group,sycl::access::address_space::local_space> counter(*dest);        
        return counter.fetch_min(count);      
      }

    template<typename T>
      static __forceinline uint atomic_max_local(T *dest, const T count=1)
      {
        sycl::atomic_ref<T, sycl::memory_order::relaxed, sycl::memory_scope::work_group,sycl::access::address_space::local_space> counter(*dest);        
        return counter.fetch_max(count);      
      }

    template<typename T>
      static __forceinline uint atomic_or_global(T *dest, const T count=1)
      {
        sycl::atomic_ref<T, sycl::memory_order::relaxed, sycl::memory_scope::device,sycl::access::address_space::global_space> counter(*dest);        
        return counter.fetch_or(count);      
      }
    


#else    
    template<typename T, sycl::access::address_space space>
      static __forceinline uint atomic_add(T *dest, const T count=1)
      {
        sycl::multi_ptr<T,space> ptr(dest);
        sycl::atomic<T,space> counter(ptr);
        return atomic_fetch_add(counter,count);      
      }


    template<typename T, sycl::access::address_space space>
      static __forceinline uint atomic_maximum(T *dest, const T count)
      {
        sycl::multi_ptr<T,space> ptr(dest);
        sycl::atomic<T,space> counter(ptr);
        return atomic_fetch_max(counter,count);      
      }

    template<typename T, sycl::access::address_space space>
      static __forceinline uint atomic_minimum(T *dest, const T count)
      {
        sycl::multi_ptr<T,space> ptr(dest);
        sycl::atomic<T,space> counter(ptr);
        return atomic_fetch_min(counter,count);      
      }
#endif    
    
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

    template<typename T>
      static __forceinline void* as_void(T t)
      {
	return __builtin_bit_cast(void*,t);
      }
    
    __forceinline uint bitInterleave3D(const uint3 &in)
    {
      uint x = in.x(), y = in.y(), z = in.z();
      x = (x | (x << 16)) & 0x030000FF;
      x = (x | (x << 8)) & 0x0300F00F;
      x = (x | (x << 4)) & 0x030C30C3;
      x = (x | (x << 2)) & 0x09249249;

      y = (y | (y << 16)) & 0x030000FF;
      y = (y | (y << 8)) & 0x0300F00F;
      y = (y | (y << 4)) & 0x030C30C3;
      y = (y | (y << 2)) & 0x09249249;

      z = (z | (z << 16)) & 0x030000FF;
      z = (z | (z << 8)) & 0x0300F00F;
      z = (z | (z << 4)) & 0x030C30C3;
      z = (z | (z << 2)) & 0x09249249;

      return x | (y << 1) | (z << 2);
    }


    __forceinline uint bitInterleave4D(const uint4 &in)
    {
      uint x = in.x(), y = in.y(), z = in.z(), w = in.w();

      x = x & 0x000000ff;
      x = (x ^ (x << 16)) & 0x00c0003f;
      x = (x ^ (x << 8)) & 0x00c03807;
      x = (x ^ (x << 4)) & 0x08530853;
      x = (x ^ (x << 2)) & 0x09090909;
      x = (x ^ (x << 1)) & 0x11111111;

      y = y & 0x000000ff;
      y = (y ^ (y << 16)) & 0x00c0003f;
      y = (y ^ (y << 8)) & 0x00c03807;
      y = (y ^ (y << 4)) & 0x08530853;
      y = (y ^ (y << 2)) & 0x09090909;
      y = (y ^ (y << 1)) & 0x11111111;

      z = z & 0x000000ff;
      z = (z ^ (z << 16)) & 0x00c0003f;
      z = (z ^ (z << 8)) & 0x00c03807;
      z = (z ^ (z << 4)) & 0x08530853;
      z = (z ^ (z << 2)) & 0x09090909;
      z = (z ^ (z << 1)) & 0x11111111;

      w = w & 0x000000ff;
      w = (w ^ (w << 16)) & 0x00c0003f;
      w = (w ^ (w << 8)) & 0x00c03807;
      w = (w ^ (w << 4)) & 0x08530853;
      w = (w ^ (w << 2)) & 0x09090909;
      w = (w ^ (w << 1)) & 0x11111111;

      return (x | (y << 1) | (z << 2) | (w << 3));
    }

    __forceinline uint64_t splitBy3(const uint a){
      uint64_t x = a & 0x1fffff; 
      x = (x | x << 32) & 0x1f00000000ffff; 
      x = (x | x << 16) & 0x1f0000ff0000ff; 
      x = (x | x << 8) & 0x100f00f00f00f00f;
      x = (x | x << 4) & 0x10c30c30c30c30c3;
      x = (x | x << 2) & 0x1249249249249249;
      return x;
    }

    __forceinline uint64_t bitInterleave3D_64bits(const uint3 v){
      return splitBy3(v.x()) | splitBy3(v.y()) << 1 | splitBy3(v.z()) << 2;
    }

    __forceinline uint64_t bitInterleave4D_64bits(const uint4 &v){
      const uint low  = bitInterleave4D(v>>0);
      const uint high = bitInterleave4D(v>>8);
      return (((uint64_t)high)<<32) | low;      
    }
    
    
    __forceinline uint sub_group_shared_global_atomic(sycl::atomic_ref<uint, sycl::memory_order::relaxed, sycl::memory_scope::device,sycl::access::address_space::global_space> &counter, const uint add)
    {
      const uint subgroupLocalID = get_sub_group_local_id();    
      const uint ballot = sub_group_ballot(true);
      const uint first  = sycl::ctz(ballot);
      const uint popc   = sycl::popcount(ballot);
      const uint prefix = sycl::popcount(ballot & (((uint)1 << subgroupLocalID)-1));
      uint index = 0;
      if (subgroupLocalID == first)
        index = counter.fetch_add(popc * add);
      index = sub_group_broadcast(index,first);
      return index + prefix;
    }

    __forceinline uint sub_group_shared_local_atomic(sycl::atomic_ref<uint, sycl::memory_order::relaxed, sycl::memory_scope::work_group,sycl::access::address_space::local_space> &counter, const uint add)
    {
      const uint subgroupLocalID = get_sub_group_local_id();    
      const uint ballot = sub_group_ballot(true);
      const uint first  = sycl::ctz(ballot);
      const uint popc   = sycl::popcount(ballot);
      const uint prefix = sycl::popcount(ballot & (((uint)1 << subgroupLocalID)-1));
      uint index = 0;
      if (subgroupLocalID == first)
        index = counter.fetch_add(popc * add);
      index = sub_group_broadcast(index,first);
      return index + prefix;
    }

    template<typename T>        
    __forceinline uint sub_group_shared_varying_global_atomic(sycl::atomic_ref<T, sycl::memory_order::relaxed, sycl::memory_scope::device,sycl::access::address_space::global_space> &counter, const T add)
    {
      const uint subgroupLocalID = get_sub_group_local_id();    
      const uint ballot = sub_group_ballot(true);
      const uint first  = sycl::ctz(ballot);
      const T total  = sub_group_reduce(add, SYCL_EXT_ONEAPI::plus<T>());
      const T prefix = sub_group_exclusive_scan(add, SYCL_EXT_ONEAPI::plus<T>());
      uint index = 0;
      if (subgroupLocalID == first)
        index = counter.fetch_add(total);
      index = sub_group_broadcast(index,first);
      return index + prefix;
    }

    template<typename T>    
    static __forceinline T sub_group_shared_varying_atomic_add_global(T *dest, const T count=1)
    {
      sycl::atomic_ref<T, sycl::memory_order::relaxed, sycl::memory_scope::device,sycl::access::address_space::global_space> counter(*dest);
      return sub_group_shared_varying_global_atomic(counter,count);
    }

    template<typename T>        
    __forceinline uint sub_group_shared_varying_local_atomic(sycl::atomic_ref<T, sycl::memory_order::relaxed, sycl::memory_scope::device,sycl::access::address_space::local_space> &counter, const T add)
    {
      const uint subgroupLocalID = get_sub_group_local_id();    
      const uint ballot = sub_group_ballot(true);
      const uint first  = sycl::ctz(ballot);
      const T total  = sub_group_reduce(add, SYCL_EXT_ONEAPI::plus<T>());
      const T prefix = sub_group_exclusive_scan(add, SYCL_EXT_ONEAPI::plus<T>());
      uint index = 0;
      if (subgroupLocalID == first)
        index = counter.fetch_add(total);
      index = sub_group_broadcast(index,first);
      return index + prefix;
    }

    template<typename T>    
    static __forceinline T sub_group_shared_varying_atomic_add_local(T *dest, const T count=1)
    {
      sycl::atomic_ref<T, sycl::memory_order::relaxed, sycl::memory_scope::device,sycl::access::address_space::local_space> counter(*dest);
      return sub_group_shared_varying_local_atomic(counter,count);
    }

    

    __forceinline void waitOnQueueAndCatchException(sycl::queue &gpu_queue)
    {
      try {
        gpu_queue.wait_and_throw();
      } catch (sycl::exception const& e) {
        std::cout << "Caught synchronous SYCL exception:\n"
                  << e.what() << std::endl;
        FATAL("OpenCL Exception");     
      }      
    }

    __forceinline double getDeviceExecutionTiming(sycl::event &queue_event)
    {
      const auto t0 = queue_event.template get_profiling_info<sycl::info::event_profiling::command_start>();
      const auto t1 = queue_event.template get_profiling_info<sycl::info::event_profiling::command_end>();
      return (t1-t0)*1E-6;      
    }

    
    struct TraversalStats
    {
      uint nrays;
      uint tsteps;
      uint isteps;
      uint stack_pushs;
      uint stack_pops;
      uint stack_pushs_parent;
      uint stack_pops_parent;
      uint stack_max_depth;
      uint numHitsPerNode[16];
      uint dummy[4];
      
      inline void reset() {
        nrays  = 0;
        tsteps = 0;
        isteps = 0;
        stack_pushs = 0;
        stack_pops = 0;
        stack_pushs_parent = 0;
        stack_pops_parent = 0;
        stack_max_depth = 0;
        for (int i=0;i<16;i++)
          numHitsPerNode[i] = 0;
        for (int i=0;i<4;i++)        
          dummy[i] = 0;
      }

      inline void _nrays_inc(uint v=1)  {
        v = get_sub_group_local_id() == sycl::ctz(intel_sub_group_ballot(true)) ? v : 0;
        gpu::atomic_add_global<uint>(&nrays,v);
      }
      inline void _tsteps_inc(uint v=1) {
        v = get_sub_group_local_id() == sycl::ctz(intel_sub_group_ballot(true)) ? v : 0;
        gpu::atomic_add_global<uint>(&tsteps,v);
      }
      inline void _isteps_inc(uint v=1) {
        v = get_sub_group_local_id() == sycl::ctz(intel_sub_group_ballot(true)) ? v : 0;
        gpu::atomic_add_global<uint>(&isteps,v);
      }

      inline void _stack_pushs_inc(uint v=1) {
        v = get_sub_group_local_id() == sycl::ctz(intel_sub_group_ballot(true)) ? v : 0;
        gpu::atomic_add_global<uint>(&stack_pushs,v);
      }      
      inline void _stack_pops_inc(uint v=1) {
        v = get_sub_group_local_id() == sycl::ctz(intel_sub_group_ballot(true)) ? v : 0;
        gpu::atomic_add_global<uint>(&stack_pops,v);
      }      

      inline void _stack_pushs_parent_inc(uint v=1) {
        v = get_sub_group_local_id() == sycl::ctz(intel_sub_group_ballot(true)) ? v : 0;
        gpu::atomic_add_global<uint>(&stack_pushs_parent,v);
      }      
      inline void _stack_pops_parent_inc(uint v=1) {
        v = get_sub_group_local_id() == sycl::ctz(intel_sub_group_ballot(true)) ? v : 0;
        gpu::atomic_add_global<uint>(&stack_pops_parent,v);
      }      

      inline void _numHitsPerNode_inc(uint index, uint v=1) {
        v = get_sub_group_local_id() == sycl::ctz(intel_sub_group_ballot(true)) ? v : 0;
        gpu::atomic_add_global<uint>(&numHitsPerNode[index],v);
      }      

      inline void _stack_max_depth_set(uint sindex, uint v=1) {
        v = get_sub_group_local_id() == sycl::ctz(intel_sub_group_ballot(true)) ? v : 0;
        gpu::atomic_max_global<uint>(&stack_max_depth,sindex);
      }      

      inline void _dummy_inc(uint slot,uint v=1)  {
        v = get_sub_group_local_id() == sycl::ctz(intel_sub_group_ballot(true)) ? v : 0;
        gpu::atomic_add_global<uint>(&dummy[slot],v);
      }
    };

    inline std::ostream &operator<<(std::ostream &cout, const TraversalStats& s)
    {
      uint numHitsTotal = 0;
      for (uint i=0;i<16;i++) numHitsTotal += s.numHitsPerNode[i];      
      uint numHits4plus = 0;
      for (uint i=4;i<16;i++) numHits4plus += s.numHitsPerNode[i];
      
      return cout << "rays " << s.nrays << std::endl
                  << " tsteps " << s.tsteps << std::endl
                  << " isteps " << s.isteps << std::endl
                  << " tsteps/ray " << (float)s.tsteps / s.nrays << std::endl
                  << " isteps/ray " << (float)s.isteps / s.nrays << std::endl
                  << " stack_pushs/ray " << (float)s.stack_pushs / s.nrays << std::endl
                  << " stack_pops/ray " << (float)s.stack_pops / s.nrays << std::endl
                  << " stack_pushs_parent/ray " << (float)s.stack_pushs_parent / s.nrays << std::endl
                  << " stack_pops_parent/ray " << (float)s.stack_pops_parent / s.nrays << std::endl
                  << " numHits [0] " << (float)s.numHitsPerNode[0] / numHitsTotal << std::endl
                  << " numHits [1] " << (float)s.numHitsPerNode[1] / numHitsTotal << std::endl
                  << " numHits [2] " << (float)s.numHitsPerNode[2] / numHitsTotal << std::endl
                  << " numHits [3] " << (float)s.numHitsPerNode[3] / numHitsTotal << std::endl
                  << " numHits [4]+ " << (float)numHits4plus / numHitsTotal << std::endl
                  << " max stack depth " <<  s.stack_max_depth << std::endl
                  << " dummy " << s.dummy[0] << " " << s.dummy[1] << " " << s.dummy[2] << " " << s.dummy[3] << std::endl; 
      
    };
    
  };
};

