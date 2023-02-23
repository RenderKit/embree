// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#if defined(EMBREE_SYCL_GPU_BVH_BUILDER)

#include "../gpu/common.h"
#include "../gpu/AABB3f.h"

namespace embree
{
  namespace gpu
  {
    class AABB3f;
    
    class __aligned(32) AABB {
    public:
      float4 lower;
      float4 upper;

      __forceinline AABB() = default;

      __forceinline AABB(const AABB &aabb) : lower(aabb.lower), upper(aabb.upper) {}

      __forceinline AABB(const float4 &v) : lower(v), upper(v) { }

      __forceinline AABB(const float4 &lower, const float4 &upper) : lower(lower), upper(upper) {}

      __forceinline AABB(const BBox3fa &aabb) : lower(aabb.lower.x, aabb.lower.y, aabb.lower.z, 0) , upper(aabb.upper.x, aabb.upper.y, aabb.upper.z, 0) {}

      __forceinline AABB(const Vec3fa &l, const Vec3fa &u) : lower(l.x, l.y, l.z, 0), upper(u.x, u.y, u.z, 0) {}

      __forceinline AABB(const Vec3fa &v) : lower(v.x, v.y, v.z, 0), upper(v.x, v.y, v.z, 0) {}

      __forceinline void init()
      {
        const float _pos_inf =  INFINITY;
        const float _neg_inf = -INFINITY;
        lower = float4(_pos_inf,_pos_inf,_pos_inf,0.0f);
        upper = float4(_neg_inf,_neg_inf,_neg_inf,0.0f);	
      }

      __forceinline bool empty()
      {
        if (lower.x() > upper.x() ||
            lower.y() > upper.y() ||
            lower.z() > upper.z()) return true;
        return false;
      }
      
      
      __forceinline void extend(const AABB &aabb)
      {
        lower = min(lower,aabb.lower);
        upper = max(upper,aabb.upper);	
      }

      __forceinline void extend(const float4 &v)
      {
        lower = min(lower,v);
        upper = max(upper,v);	
      }

      __forceinline void extend(const BBox3fa &aabb)
      {
        lower = min(lower, float4(aabb.lower.x, aabb.lower.y, aabb.lower.z, 0));
        upper = max(upper, float4(aabb.upper.x, aabb.upper.y, aabb.upper.z, 0));
      }

      __forceinline void extend(const Vec3fa &v)
      {
        lower = min(lower, float4(v.x, v.y, v.z, 0));
        upper = max(upper, float4(v.x, v.y, v.z, 0));
      }

      __forceinline void enlarge(const float4 &v)
      {
        lower -= v;
        upper += v;	
      }

      __forceinline float4 size() const
      {
        return upper - lower;
      }

      __forceinline float3 diag3() const
      {
        float4 d = upper - lower;
        return float3(d.x(),d.y(),d.z());
      }

      __forceinline uint maxDiagDimIndex3() const
      {
        const float3 d = diag3();
        uint64_t dx = (((uint64_t)as_uint(d.x())) << 32) | 0;
        uint64_t dy = (((uint64_t)as_uint(d.y())) << 32) | 1;
        uint64_t dz = (((uint64_t)as_uint(d.z())) << 32) | 2;
        return max(max(dx,dy),dz);        
      }

      __forceinline float maxDiagDim3() const
      {
        const float3 d = diag3();
        return max(max(d.x(),d.y()),d.z());        
      }
      

      __forceinline float4 centroid2() const
      {
        return upper + lower;
      }

      __forceinline float4 centroid() const
      {
        return centroid2() * 0.5f;
      }


      __forceinline float length2() const
      {
        const float4 diag = size();
        return  sycl::fma(diag.x(),diag.x(),
                          sycl::fma(diag.y(),diag.y(),
                                    diag.z() * diag.z()));
        
      }
      
      __forceinline float length() const
      {
        return sqrtf(length2());
      }
      

      __forceinline bool encloses(const AABB &other)
      {
        if ( other.lower.x() < lower.x() ||
             other.lower.y() < lower.y() ||
             other.lower.z() < lower.z() )
          return false;
        if ( other.upper.x() > upper.x() ||
             other.upper.y() > upper.y() ||
             other.upper.z() > upper.z() )
          return false;
        return true;
      }
      
      __forceinline void atomic_merge_global(AABB &dest) const
      {
        sycl::atomic_ref<float, sycl::memory_order::relaxed, sycl::memory_scope::device,sycl::access::address_space::global_space> lower_x(((float*)&dest)[0]);
        sycl::atomic_ref<float, sycl::memory_order::relaxed, sycl::memory_scope::device,sycl::access::address_space::global_space> lower_y(((float*)&dest)[1]);
        sycl::atomic_ref<float, sycl::memory_order::relaxed, sycl::memory_scope::device,sycl::access::address_space::global_space> lower_z(((float*)&dest)[2]);

        sycl::atomic_ref<float, sycl::memory_order::relaxed, sycl::memory_scope::device,sycl::access::address_space::global_space> upper_x(((float*)&dest)[4]);
        sycl::atomic_ref<float, sycl::memory_order::relaxed, sycl::memory_scope::device,sycl::access::address_space::global_space> upper_y(((float*)&dest)[5]);
        sycl::atomic_ref<float, sycl::memory_order::relaxed, sycl::memory_scope::device,sycl::access::address_space::global_space> upper_z(((float*)&dest)[6]);
        
        lower_x.fetch_min(lower.x());
        lower_y.fetch_min(lower.y());
        lower_z.fetch_min(lower.z());

        upper_x.fetch_max(upper.x());
        upper_y.fetch_max(upper.y());
        upper_z.fetch_max(upper.z());        
      }

      __forceinline void atomic_merge_local(AABB &dest) const
      {
        //sycl::atomic_ref<float, sycl::memory_order::relaxed, sycl::memory_scope::work_group,sycl::access::address_space::local_space> lower_x(((float*)&dest)[0]);
        sycl::atomic_ref<float, sycl::memory_order::relaxed, sycl::memory_scope::work_group,sycl::access::address_space::local_space> lower_x(dest.lower.x());
        sycl::atomic_ref<float, sycl::memory_order::relaxed, sycl::memory_scope::work_group,sycl::access::address_space::local_space> lower_y(((float*)&dest)[1]);
        sycl::atomic_ref<float, sycl::memory_order::relaxed, sycl::memory_scope::work_group,sycl::access::address_space::local_space> lower_z(((float*)&dest)[2]);

        sycl::atomic_ref<float, sycl::memory_order::relaxed, sycl::memory_scope::work_group,sycl::access::address_space::local_space> upper_x(((float*)&dest)[4]);
        sycl::atomic_ref<float, sycl::memory_order::relaxed, sycl::memory_scope::work_group,sycl::access::address_space::local_space> upper_y(((float*)&dest)[5]);
        sycl::atomic_ref<float, sycl::memory_order::relaxed, sycl::memory_scope::work_group,sycl::access::address_space::local_space> upper_z(((float*)&dest)[6]);
        
        lower_x.fetch_min(lower.x());
        lower_y.fetch_min(lower.y());
        lower_z.fetch_min(lower.z());

        upper_x.fetch_max(upper.x());
        upper_y.fetch_max(upper.y());
        upper_z.fetch_max(upper.z());                
      }
      
      __forceinline AABB sub_group_reduce() const
      {
        AABB result;
        result.lower.x() = embree::sub_group_reduce_min(lower.x());
        result.lower.y() = embree::sub_group_reduce_min(lower.y());
        result.lower.z() = embree::sub_group_reduce_min(lower.z());
        result.lower.w() = 0.0f;
        result.upper.x() = embree::sub_group_reduce_max(upper.x());
        result.upper.y() = embree::sub_group_reduce_max(upper.y());
        result.upper.z() = embree::sub_group_reduce_max(upper.z());
        result.upper.w() = 0.0f;
        return result;	
      }

      __forceinline AABB sub_group_shuffle(const sycl::id<1> &localID) const
      {
        AABB result;
        result.lower.x() = embree::sub_group_shuffle<float>(lower.x(),localID);
        result.lower.y() = embree::sub_group_shuffle<float>(lower.y(),localID);
        result.lower.z() = embree::sub_group_shuffle<float>(lower.z(),localID);
        result.lower.w() = 0.0f;
        result.upper.x() = embree::sub_group_shuffle<float>(upper.x(),localID);
        result.upper.y() = embree::sub_group_shuffle<float>(upper.y(),localID);
        result.upper.z() = embree::sub_group_shuffle<float>(upper.z(),localID);
        result.upper.w() = 0.0f;	
        return result;	
      }

      __forceinline AABB sub_group_scan_exclusive_min_max() const
      {
        AABB result;
        result.lower.x() = embree::sub_group_exclusive_scan(lower.x(), SYCL_EXT_ONEAPI::minimum<float>());
        result.lower.y() = embree::sub_group_exclusive_scan(lower.y(), SYCL_EXT_ONEAPI::minimum<float>());
        result.lower.z() = embree::sub_group_exclusive_scan(lower.z(), SYCL_EXT_ONEAPI::minimum<float>());
        result.lower.w() = 0.0f;
        result.upper.x() = embree::sub_group_exclusive_scan(upper.x(), SYCL_EXT_ONEAPI::maximum<float>());
        result.upper.y() = embree::sub_group_exclusive_scan(upper.y(), SYCL_EXT_ONEAPI::maximum<float>());
        result.upper.z() = embree::sub_group_exclusive_scan(upper.z(), SYCL_EXT_ONEAPI::maximum<float>());
        result.upper.w() = 0.0f;        
        return result;	
      }

      __forceinline AABB sub_group_scan_inclusive_min_max() const 
      {
        AABB result;
        result.lower.x() = embree::sub_group_inclusive_scan(lower.x(), SYCL_EXT_ONEAPI::minimum<float>());
        result.lower.y() = embree::sub_group_inclusive_scan(lower.y(), SYCL_EXT_ONEAPI::minimum<float>());
        result.lower.z() = embree::sub_group_inclusive_scan(lower.z(), SYCL_EXT_ONEAPI::minimum<float>());
        result.lower.w() = 0.0f;
        result.upper.x() = embree::sub_group_inclusive_scan(upper.x(), SYCL_EXT_ONEAPI::maximum<float>());
        result.upper.y() = embree::sub_group_inclusive_scan(upper.y(), SYCL_EXT_ONEAPI::maximum<float>());
        result.upper.z() = embree::sub_group_inclusive_scan(upper.z(), SYCL_EXT_ONEAPI::maximum<float>());
        result.upper.w() = 0.0f;        
        return result;	
      }

      __forceinline AABB work_group_reduce() const
      {
        AABB result;
        result.lower.x() = work_group_reduce_min(lower.x());
        result.lower.y() = work_group_reduce_min(lower.y());
        result.lower.z() = work_group_reduce_min(lower.z());
        result.lower.w() = 0.0f;
        result.upper.x() = work_group_reduce_max(upper.x());
        result.upper.y() = work_group_reduce_max(upper.y());
        result.upper.z() = work_group_reduce_max(upper.z());
        result.upper.w() = 0.0f;	
        return result;	
      }

      __forceinline float     area() { return gpu::area(size()); }
      __forceinline float halfArea() { return gpu::halfarea(size()); }

      __forceinline uint getPrimitiveCost() const { return 1; }

      __forceinline AABB intersect(const AABB &a, const AABB &b)
      {
        AABB bounds;
        bounds.lower.x() = max(a.lower.x(),b.lower.x());
        bounds.lower.y() = max(a.lower.y(),b.lower.y());
        bounds.lower.z() = max(a.lower.z(),b.lower.z());        
        bounds.upper.x() = min(a.upper.x(),b.upper.x());
        bounds.upper.y() = min(a.upper.y(),b.upper.y());
        bounds.upper.z() = min(a.upper.z(),b.upper.z());
        return bounds;
      }
      
      friend __forceinline embree_ostream operator<<(embree_ostream cout, const AABB &aabb)
      {
        cout << "AABB { "; // << embree_endl;
        cout << "  lower = (" << aabb.lower.x() << ", " << aabb.lower.y() << ", " << aabb.lower.z() << ") "; // << embree_endl;
        cout << "  upper = (" << aabb.upper.x() << ", " << aabb.upper.y() << ", " << aabb.upper.z() << ") "; // << embree_endl;
        return cout << "}";
      }
    };

    __forceinline bool outsideAABBTest(gpu::AABB &big, gpu::AABB &small)
    {
      int4 b0 = small.lower < big.lower;
      int4 b1 = small.upper > big.upper;
      int4 b = b0 | b1;
      return b.x() | b.y() | b.z();
    }
        
  }
}

#endif
