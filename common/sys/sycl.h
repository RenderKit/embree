// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "platform.h"

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
using sycl::uint;
using sycl::ulong;
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
using sycl::ushort;

using sycl::fmax;
using sycl::fmin;

#ifdef __SYCL_DEVICE_ONLY__
#define GLOBAL __attribute__((opencl_global))
#define LOCAL  __attribute__((opencl_local))

SYCL_EXTERNAL extern int   work_group_reduce_add(int x);
SYCL_EXTERNAL extern float work_group_reduce_min(float x);
SYCL_EXTERNAL extern float work_group_reduce_max(float x);

SYCL_EXTERNAL extern float atomic_min(volatile GLOBAL float *p, float val);
SYCL_EXTERNAL extern float atomic_min(volatile LOCAL  float *p, float val);
SYCL_EXTERNAL extern float atomic_max(volatile GLOBAL float *p, float val);
SYCL_EXTERNAL extern float atomic_max(volatile LOCAL  float *p, float val);

SYCL_EXTERNAL extern "C" uint intel_sub_group_ballot(bool valid);

SYCL_EXTERNAL extern "C" void __builtin_IB_assume_uniform(void *p);

// Load message caching control

  enum LSC_LDCC {
    LSC_LDCC_DEFAULT,
    LSC_LDCC_L1UC_L3UC,     // Override to L1 uncached and L3 uncached
    LSC_LDCC_L1UC_L3C,      // Override to L1 uncached and L3 cached
    LSC_LDCC_L1C_L3UC,      // Override to L1 cached and L3 uncached
    LSC_LDCC_L1C_L3C,       // Override to L1 cached and L3 cached
    LSC_LDCC_L1S_L3UC,      // Override to L1 streaming load and L3 uncached
    LSC_LDCC_L1S_L3C,       // Override to L1 streaming load and L3 cached
    LSC_LDCC_L1IAR_L3C,     // Override to L1 invalidate-after-read, and L3 cached
  };

 

// Store message caching control (also used for atomics)

  enum LSC_STCC {
    LSC_STCC_DEFAULT,
    LSC_STCC_L1UC_L3UC,     // Override to L1 uncached and L3 uncached
    LSC_STCC_L1UC_L3WB,     // Override to L1 uncached and L3 written back
    LSC_STCC_L1WT_L3UC,     // Override to L1 written through and L3 uncached
    LSC_STCC_L1WT_L3WB,     // Override to L1 written through and L3 written back
    LSC_STCC_L1S_L3UC,      // Override to L1 streaming and L3 uncached
    LSC_STCC_L1S_L3WB,      // Override to L1 streaming and L3 written back
    LSC_STCC_L1WB_L3WB,     // Override to L1 written through and L3 written back
  };

 

///////////////////////////////////////////////////////////////////////

// LSC Loads

///////////////////////////////////////////////////////////////////////

SYCL_EXTERNAL /* extern "C" */ uint          __builtin_IB_lsc_load_global_uchar_to_uint (const GLOBAL sycl::uchar  *base, int elemOff, enum LSC_LDCC cacheOpt);     //D8U32
SYCL_EXTERNAL /* extern "C" */ uint          __builtin_IB_lsc_load_global_ushort_to_uint(const GLOBAL sycl::ushort *base, int elemOff, enum LSC_LDCC cacheOpt);   //D16U32
SYCL_EXTERNAL /* extern "C" */ uint          __builtin_IB_lsc_load_global_uint  (const GLOBAL sycl::uint   *base, int elemOff, enum LSC_LDCC cacheOpt);       //D32V1
SYCL_EXTERNAL /* extern "C" */ sycl::uint2   __builtin_IB_lsc_load_global_uint2 (const GLOBAL sycl::uint2  *base, int elemOff, enum LSC_LDCC cacheOpt);     //D32V2
SYCL_EXTERNAL /* extern "C" */ sycl::uint3   __builtin_IB_lsc_load_global_uint3 (const GLOBAL sycl::uint3  *base, int elemOff, enum LSC_LDCC cacheOpt);     //D32V3
SYCL_EXTERNAL /* extern "C" */ sycl::uint4   __builtin_IB_lsc_load_global_uint4 (const GLOBAL sycl::uint4  *base, int elemOff, enum LSC_LDCC cacheOpt);     //D32V4
SYCL_EXTERNAL /* extern "C" */ sycl::uint8   __builtin_IB_lsc_load_global_uint8 (const GLOBAL sycl::uint8  *base, int elemOff, enum LSC_LDCC cacheOpt);     //D32V8
SYCL_EXTERNAL /* extern "C" */ sycl::ulong   __builtin_IB_lsc_load_global_ulong (const GLOBAL sycl::ulong  *base, int elemOff, enum LSC_LDCC cacheOpt);    //D64V1
SYCL_EXTERNAL /* extern "C" */ sycl::ulong2  __builtin_IB_lsc_load_global_ulong2(const GLOBAL sycl::ulong2 *base, int elemOff, enum LSC_LDCC cacheOpt);  //D64V2
SYCL_EXTERNAL /* extern "C" */ sycl::ulong3  __builtin_IB_lsc_load_global_ulong3(const GLOBAL sycl::ulong3 *base, int elemOff, enum LSC_LDCC cacheOpt);  //D64V3
SYCL_EXTERNAL /* extern "C" */ sycl::ulong4  __builtin_IB_lsc_load_global_ulong4(const GLOBAL sycl::ulong4 *base, int elemOff, enum LSC_LDCC cacheOpt);  //D64V4
SYCL_EXTERNAL /* extern "C" */ sycl::ulong8  __builtin_IB_lsc_load_global_ulong8(const GLOBAL sycl::ulong8 *base, int elemOff, enum LSC_LDCC cacheOpt);  //D64V8
  
//     global address space
SYCL_EXTERNAL extern "C"  void  __builtin_IB_lsc_store_global_uchar_from_uint (GLOBAL sycl::uchar  *base, int immElemOff, uint val, enum LSC_STCC cacheOpt);     //D8U32
SYCL_EXTERNAL extern "C"  void  __builtin_IB_lsc_store_global_ushort_from_uint(GLOBAL sycl::ushort *base, int immElemOff, uint val, enum LSC_STCC cacheOpt);  //D16U32
SYCL_EXTERNAL extern "C"  void  __builtin_IB_lsc_store_global_uint  (GLOBAL sycl::uint   *base, int immElemOff,       uint   val, enum LSC_STCC cacheOpt);        //D32V1
SYCL_EXTERNAL extern "C"  void  __builtin_IB_lsc_store_global_uint2 (GLOBAL sycl::uint2  *base, int immElemOff, sycl::uint2  val, enum LSC_STCC cacheOpt);     //D32V2
SYCL_EXTERNAL extern "C"  void  __builtin_IB_lsc_store_global_uint3 (GLOBAL sycl::uint3  *base, int immElemOff, sycl::uint3  val, enum LSC_STCC cacheOpt);     //D32V3
SYCL_EXTERNAL extern "C"  void  __builtin_IB_lsc_store_global_uint4 (GLOBAL sycl::uint4  *base, int immElemOff, sycl::uint4  val, enum LSC_STCC cacheOpt);     //D32V4
SYCL_EXTERNAL extern "C"  void  __builtin_IB_lsc_store_global_uint8 (GLOBAL sycl::uint8  *base, int immElemOff, sycl::uint8  val, enum LSC_STCC cacheOpt);     //D32V8
SYCL_EXTERNAL extern "C"  void  __builtin_IB_lsc_store_global_ulong (GLOBAL sycl::ulong  *base, int immElemOff, sycl::ulong  val, enum LSC_STCC cacheOpt);     //D64V1
SYCL_EXTERNAL extern "C"  void  __builtin_IB_lsc_store_global_ulong2(GLOBAL sycl::ulong2 *base, int immElemOff, sycl::ulong2 val, enum LSC_STCC cacheOpt);  //D64V2
SYCL_EXTERNAL extern "C"  void  __builtin_IB_lsc_store_global_ulong3(GLOBAL sycl::ulong3 *base, int immElemOff, sycl::ulong3 val, enum LSC_STCC cacheOpt);  //D64V3
SYCL_EXTERNAL extern "C"  void  __builtin_IB_lsc_store_global_ulong4(GLOBAL sycl::ulong4 *base, int immElemOff, sycl::ulong4 val, enum LSC_STCC cacheOpt);  //D64V4
SYCL_EXTERNAL extern "C"  void  __builtin_IB_lsc_store_global_ulong8(GLOBAL sycl::ulong8 *base, int immElemOff, sycl::ulong8 val, enum LSC_STCC cacheOpt);  //D64V8

///////////////////////////////////////////////////////////////////////
// prefetching
///////////////////////////////////////////////////////////////////////
//
// LSC Pre-Fetch Load functions with CacheControls
//     global address space
SYCL_EXTERNAL extern "C"  void __builtin_IB_lsc_prefetch_global_uchar (const GLOBAL sycl::uchar  *base, int immElemOff, enum LSC_LDCC cacheOpt); //D8U32
SYCL_EXTERNAL extern "C"  void __builtin_IB_lsc_prefetch_global_ushort(const GLOBAL sycl::ushort *base, int immElemOff, enum LSC_LDCC cacheOpt); //D16U32
SYCL_EXTERNAL extern "C"  void __builtin_IB_lsc_prefetch_global_uint  (const GLOBAL sycl::uint   *base, int immElemOff, enum LSC_LDCC cacheOpt); //D32V1
SYCL_EXTERNAL extern "C"  void __builtin_IB_lsc_prefetch_global_uint2 (const GLOBAL sycl::uint2  *base, int immElemOff, enum LSC_LDCC cacheOpt); //D32V2
SYCL_EXTERNAL extern "C"  void __builtin_IB_lsc_prefetch_global_uint3 (const GLOBAL sycl::uint3  *base, int immElemOff, enum LSC_LDCC cacheOpt); //D32V3
SYCL_EXTERNAL extern "C"  void __builtin_IB_lsc_prefetch_global_uint4 (const GLOBAL sycl::uint4  *base, int immElemOff, enum LSC_LDCC cacheOpt); //D32V4
SYCL_EXTERNAL extern "C"  void __builtin_IB_lsc_prefetch_global_uint8 (const GLOBAL sycl::uint8  *base, int immElemOff, enum LSC_LDCC cacheOpt); //D32V8
SYCL_EXTERNAL extern "C"  void __builtin_IB_lsc_prefetch_global_ulong (const GLOBAL sycl::ulong  *base, int immElemOff, enum LSC_LDCC cacheOpt); //D64V1
SYCL_EXTERNAL extern "C"  void __builtin_IB_lsc_prefetch_global_ulong2(const GLOBAL sycl::ulong2 *base, int immElemOff, enum LSC_LDCC cacheOpt); //D64V2
SYCL_EXTERNAL extern "C"  void __builtin_IB_lsc_prefetch_global_ulong3(const GLOBAL sycl::ulong3 *base, int immElemOff, enum LSC_LDCC cacheOpt); //D64V3
SYCL_EXTERNAL extern "C"  void __builtin_IB_lsc_prefetch_global_ulong4(const GLOBAL sycl::ulong4 *base, int immElemOff, enum LSC_LDCC cacheOpt); //D64V4
SYCL_EXTERNAL extern "C"  void __builtin_IB_lsc_prefetch_global_ulong8(const GLOBAL sycl::ulong8 *base, int immElemOff, enum LSC_LDCC cacheOpt); //D64V8

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

/* creates a temporary that is enforced to be uniform */
#define SYCL_UNIFORM_VAR(Ty,tmp,k)					\
  Ty tmp##_data;							\
  Ty* p##tmp##_data = (Ty*) sub_group_broadcast((ulong)&tmp##_data,k);	\
  Ty& tmp = *p##tmp##_data;

#if !defined(__forceinline)
#define __forceinline          inline __attribute__((always_inline))
#endif

#if __SYCL_COMPILER_VERSION < 20210801
#define all_of_group all_of
#define any_of_group any_of
#define none_of_group none_of
#define group_broadcast broadcast
#define reduce_over_group reduce
#define exclusive_scan_over_group exclusive_scan
#define inclusive_scan_over_group inclusive_scan
#endif

namespace embree
{
  template<typename T>
  __forceinline T cselect(const bool mask, const T &a, const T &b)
  {
    return sycl::select(b,a,(int)mask);
  }
  
  template<typename T, typename M>
  __forceinline T cselect(const M &mask, const T &a, const T &b)
  {
    return sycl::select(b,a,mask);
  }
  
  __forceinline const SYCL_EXT_ONEAPI::sub_group this_sub_group() {
    return SYCL_EXPERIMENTAL::this_sub_group(); 
  }
  
  __forceinline const uint get_sub_group_local_id() {
    return this_sub_group().get_local_id()[0];
  }

  __forceinline const uint get_sub_group_size() {
    return this_sub_group().get_max_local_range().size();
  }

  __forceinline const uint get_sub_group_id() {
    return this_sub_group().get_group_id()[0];
  }
  
  __forceinline const uint get_num_sub_groups() {
    return this_sub_group().get_group_range().size();
  }
  
  __forceinline uint sub_group_ballot(bool pred) {
    return intel_sub_group_ballot(pred);
  }

  __forceinline bool sub_group_all_of(bool pred) {
    return SYCL_ONEAPI::all_of_group(this_sub_group(),pred);
  }

  __forceinline bool sub_group_any_of(bool pred) {
    return SYCL_ONEAPI::any_of_group(this_sub_group(),pred);
  }
  
  __forceinline bool sub_group_none_of(bool pred) {
    return SYCL_ONEAPI::none_of_group(this_sub_group(),pred);
  }

  template <typename T> __forceinline T sub_group_broadcast(T x, sycl::id<1> local_id) {
    return SYCL_SUBGROUP::group_broadcast<SYCL_EXT_ONEAPI::sub_group>(this_sub_group(),x,local_id);
  }
  
  template <typename T> __forceinline T sub_group_make_uniform(T x) {
    return sub_group_broadcast(x,SYCL_CTZ::ctz(intel_sub_group_ballot(true)));
  }

  __forceinline void assume_uniform_array(void* ptr) {
#ifdef __SYCL_DEVICE_ONLY__
    __builtin_IB_assume_uniform(ptr);
#endif
  }

  template <typename T, class BinaryOperation> __forceinline T sub_group_reduce(T x, BinaryOperation binary_op) {
    return SYCL_SUBGROUP::reduce_over_group<SYCL_EXT_ONEAPI::sub_group>(this_sub_group(),x,binary_op);
  }

  template <typename T, class BinaryOperation> __forceinline T sub_group_reduce(T x, T init, BinaryOperation binary_op) {
    return SYCL_SUBGROUP::reduce_over_group<SYCL_EXT_ONEAPI::sub_group>(this_sub_group(),x,init,binary_op);
  }
  
  template <typename T> __forceinline T sub_group_reduce_min(T x, T init) {
    return sub_group_reduce(x, init, SYCL_EXT_ONEAPI::minimum<T>());
  }

  template <typename T> __forceinline T sub_group_reduce_min(T x) {
    return sub_group_reduce(x, SYCL_EXT_ONEAPI::minimum<T>());
  }

  template <typename T> __forceinline T sub_group_reduce_max(T x) {
    return sub_group_reduce(x, SYCL_EXT_ONEAPI::maximum<T>());
  }
  
  template <typename T> __forceinline T sub_group_reduce_add(T x) {
    return sub_group_reduce(x, SYCL_EXT_ONEAPI::plus<T>());
  }

  template <typename T, class BinaryOperation> __forceinline T sub_group_exclusive_scan(T x, BinaryOperation binary_op) {
    return SYCL_SUBGROUP::exclusive_scan_over_group(this_sub_group(),x,binary_op);
  }

  template <typename T, class BinaryOperation> __forceinline T sub_group_exclusive_scan_min(T x) {
    return sub_group_exclusive_scan(x,SYCL_EXT_ONEAPI::minimum<T>());
  }

  template <typename T, class BinaryOperation> __forceinline T sub_group_exclusive_scan(T x, T init, BinaryOperation binary_op) {
    return SYCL_SUBGROUP::exclusive_scan_over_group(this_sub_group(),x,init,binary_op);
  }

  template <typename T, class BinaryOperation> __forceinline T sub_group_inclusive_scan(T x, BinaryOperation binary_op) {
    return SYCL_SUBGROUP::inclusive_scan_over_group(this_sub_group(),x,binary_op);
  }

  template <typename T, class BinaryOperation> __forceinline T sub_group_inclusive_scan(T x, BinaryOperation binary_op, T init) {
    return SYCL_SUBGROUP::inclusive_scan_over_group(this_sub_group(),x,binary_op,init);
  }

  template <typename T> __forceinline T sub_group_shuffle(T x, sycl::id<1> local_id) {
    return this_sub_group().shuffle(x, local_id);
  }

  template <typename T> __forceinline T sub_group_shuffle_down(T x, uint32_t delta) {
    return this_sub_group().shuffle_down(x, delta);
  }
  
  template <typename T> __forceinline T sub_group_shuffle_up(T x, uint32_t delta) {
    return this_sub_group().shuffle_up(x, delta);
  }

  template <typename T> __forceinline T sub_group_load(const void* src) {
    return this_sub_group().load(sycl::multi_ptr<T,sycl::access::address_space::global_space>((T*)src));
  }

  template <typename T> __forceinline void sub_group_store(void* dst, const T& x) {
    this_sub_group().store(sycl::multi_ptr<T,sycl::access::address_space::global_space>((T*)dst),x);
  }
}

#if __SYCL_COMPILER_VERSION < 20210801
#undef all_of_group
#undef any_of_group
#undef none_of_group
#undef group_broadcast
#undef reduce_over_group
#undef exclusive_scan_over_group
#undef inclusive_scan_over_group
#endif
