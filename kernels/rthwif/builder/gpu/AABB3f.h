// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#if defined(EMBREE_SYCL_SUPPORT)

#include "../../builder/gpu/common.h"

namespace embree
{
  namespace gpu
  {    
    class AABB3f {
    public:
      float lower_x,lower_y,lower_z; 
      float upper_x,upper_y,upper_z;
      
      __forceinline AABB3f() = default;

      __forceinline AABB3f(const AABB3f &aabb) : lower_x(aabb.lower_x),
                                                 lower_y(aabb.lower_y),
                                                 lower_z(aabb.lower_z),
                                                 upper_x(aabb.upper_x),
                                                 upper_y(aabb.upper_y),
                                                 upper_z(aabb.upper_z)
      {}

      __forceinline AABB3f(const float3 &v) : lower_x(v.x()),
                                              lower_y(v.y()),
                                              lower_z(v.z()),
                                              upper_x(v.x()),
                                              upper_y(v.y()),
                                              upper_z(v.z())                                              
      {}

      __forceinline AABB3f(const float3 &lower, const float3 &upper) : lower_x(lower.x()),
                                                                       lower_y(lower.y()),
                                                                       lower_z(lower.z()),
                                                                       upper_x(upper.x()),
                                                                       upper_y(upper.y()),
                                                                       upper_z(upper.z())
                                                                       
      {}

      __forceinline AABB3f(const BBox3fa &aabb) : lower_x(aabb.lower.x),
                                                  lower_y(aabb.lower.y),
                                                  lower_z(aabb.lower.z),
                                                  upper_x(aabb.upper.x),
                                                  upper_y(aabb.upper.y),
                                                  upper_z(aabb.upper.z)
      {}

      __forceinline AABB3f(const float lower_x, const float lower_y, const float lower_z, const float upper_x, const float upper_y, const float upper_z) : lower_x(lower_x),
                                                                                                                                                           lower_y(lower_y),
                                                                                                                                                           lower_z(lower_z),
                                                                                                                                                           upper_x(upper_x),
                                                                                                                                                           upper_y(upper_y),
                                                                                                                                                           upper_z(upper_z)
      {}
      
      __forceinline bool empty()
      {
        if (lower_x > upper_x ||
            lower_y > upper_y ||
            lower_z > upper_z) return true;
        return false;
      }
      
      __forceinline void init()
      {
        const float _pos_inf =  INFINITY;
        const float _neg_inf = -INFINITY;
        lower_x = _pos_inf;
        lower_y = _pos_inf;
        lower_z = _pos_inf;        
        upper_x = _neg_inf;
        upper_y = _neg_inf;
        upper_z = _neg_inf;        
      }

      __forceinline void setZero()
      {
        lower_x = 0.0f;
        lower_y = 0.0f;
        lower_z = 0.0f;
        upper_x = 0.0f;
        upper_y = 0.0f;
        upper_z = 0.0f;
      }
      
      __forceinline bool encloses(const AABB3f &other)
      {
        if ( other.lower_x < lower_x ||
             other.lower_y < lower_y ||
             other.lower_z < lower_z )
          return false;
        if ( other.upper_x > upper_x ||
             other.upper_y > upper_y ||
             other.upper_z > upper_z )
          return false;
        return true;
      }      
      
      __forceinline void extend(const class AABB3f &aabb)
      {
        lower_x = min(lower_x,aabb.lower_x);
        lower_y = min(lower_y,aabb.lower_y);
        lower_z = min(lower_z,aabb.lower_z);        
        upper_x = max(upper_x,aabb.upper_x);
        upper_y = max(upper_y,aabb.upper_y);
        upper_z = max(upper_z,aabb.upper_z);        
      }

      
      __forceinline void extend(const class AABB3f &aabb, const uint slot)
      {
        const uint subgroupLocalID = get_sub_group_local_id();
        const bool cmp = subgroupLocalID == slot;        
        lower_x = cselect(cmp,min(lower_x,aabb.lower_x),lower_x);
        lower_y = cselect(cmp,min(lower_y,aabb.lower_y),lower_y);
        lower_z = cselect(cmp,min(lower_z,aabb.lower_z),lower_z);        
        upper_x = cselect(cmp,max(upper_x,aabb.upper_x),upper_x);
        upper_y = cselect(cmp,max(upper_y,aabb.upper_y),upper_y);
        upper_z = cselect(cmp,max(upper_z,aabb.upper_z),upper_z);        
      }
      

      __forceinline void extend(const float3 &v)
      {
        lower_x = min(lower_x,v.x());
        lower_y = min(lower_y,v.y());
        lower_z = min(lower_z,v.z());        
        upper_x = max(upper_x,v.x());
        upper_y = max(upper_y,v.y());
        upper_z = max(upper_z,v.z());        
      }

      __forceinline void extend(const float4 &v)
      {
        lower_x = min(lower_x,v.x());
        lower_y = min(lower_y,v.y());
        lower_z = min(lower_z,v.z());        
        upper_x = max(upper_x,v.x());
        upper_y = max(upper_y,v.y());
        upper_z = max(upper_z,v.z());        
      }
      
       __forceinline void extend(const BBox3fa &aabb)
       {
        lower_x = min(lower_x,aabb.lower.x);
        lower_y = min(lower_y,aabb.lower.y);
        lower_z = min(lower_z,aabb.lower.z);        
        upper_x = max(upper_x,aabb.upper.x);
        upper_y = max(upper_y,aabb.upper.y);
        upper_z = max(upper_z,aabb.upper.z);        
       }

      __forceinline void enlarge(const float &v)
      {
        lower_x -= v;
        lower_y -= v;
        lower_z -= v;
        upper_x += v;
        upper_y += v;
        upper_z += v;        
      }      

      __forceinline void add(const class AABB3f &aabb)
      {
        lower_x += aabb.lower_x;
        lower_y += aabb.lower_y;
        lower_z += aabb.lower_z;        
        upper_x += aabb.upper_x;
        upper_y += aabb.upper_y;
        upper_z += aabb.upper_z;        
      }
      
      __forceinline void mul(const float f)
      {
        lower_x *= f;
        lower_y *= f;
        lower_z *= f;        
        upper_x *= f;
        upper_y *= f;
        upper_z *= f;                
      }
      
      __forceinline float3 lower() const { return float3(lower_x,lower_y,lower_z); }
      __forceinline float3 upper() const { return float3(upper_x,upper_y,upper_z); }
      
      __forceinline float3 size() const
      {
        return upper() - lower();
      }

      __forceinline float3 diag() const
      {
        return upper() - lower();
      }
      
      __forceinline float3 centroid2() const
      {
        return upper() + lower();
      }

      __forceinline float3 centroid() const
      {
        return (upper() + lower())*0.5f;
      }

      __forceinline float length2() const
      {
        const float3 diag = size();
        return  sycl::fma(diag.x(),diag.x(),
                          sycl::fma(diag.y(),diag.y(),
                                    diag.z() * diag.z()));
        
      }
      
      __forceinline float maxDiagDim() const
      {
        const float3 d = diag();
        return max(max(d.x(),d.y()),d.z());        
      }      
      
      __forceinline void atomic_merge_global(AABB3f &dest) const
      {
        sycl::atomic_ref<float, sycl::memory_order::relaxed, sycl::memory_scope::device,sycl::access::address_space::global_space> _lower_x(dest.lower_x);
        sycl::atomic_ref<float, sycl::memory_order::relaxed, sycl::memory_scope::device,sycl::access::address_space::global_space> _lower_y(dest.lower_y);
        sycl::atomic_ref<float, sycl::memory_order::relaxed, sycl::memory_scope::device,sycl::access::address_space::global_space> _lower_z(dest.lower_z);

        sycl::atomic_ref<float, sycl::memory_order::relaxed, sycl::memory_scope::device,sycl::access::address_space::global_space> _upper_x(dest.upper_x);
        sycl::atomic_ref<float, sycl::memory_order::relaxed, sycl::memory_scope::device,sycl::access::address_space::global_space> _upper_y(dest.upper_y);
        sycl::atomic_ref<float, sycl::memory_order::relaxed, sycl::memory_scope::device,sycl::access::address_space::global_space> _upper_z(dest.upper_z);
        
        _lower_x.fetch_min(lower_x);
        _lower_y.fetch_min(lower_y);
        _lower_z.fetch_min(lower_z);

        _upper_x.fetch_max(upper_x);
        _upper_y.fetch_max(upper_y);
        _upper_z.fetch_max(upper_z);        
      }

      __forceinline void atomic_merge_local(AABB3f &dest) const
      {
        sycl::atomic_ref<float, sycl::memory_order::relaxed, sycl::memory_scope::work_group,sycl::access::address_space::local_space> _lower_x(dest.lower_x);
        sycl::atomic_ref<float, sycl::memory_order::relaxed, sycl::memory_scope::work_group,sycl::access::address_space::local_space> _lower_y(dest.lower_y);
        sycl::atomic_ref<float, sycl::memory_order::relaxed, sycl::memory_scope::work_group,sycl::access::address_space::local_space> _lower_z(dest.lower_z);

        sycl::atomic_ref<float, sycl::memory_order::relaxed, sycl::memory_scope::work_group,sycl::access::address_space::local_space> _upper_x(dest.upper_x);
        sycl::atomic_ref<float, sycl::memory_order::relaxed, sycl::memory_scope::work_group,sycl::access::address_space::local_space> _upper_y(dest.upper_y);
        sycl::atomic_ref<float, sycl::memory_order::relaxed, sycl::memory_scope::work_group,sycl::access::address_space::local_space> _upper_z(dest.upper_z);

        _lower_x.fetch_min(lower_x);
        _lower_y.fetch_min(lower_y);
        _lower_z.fetch_min(lower_z);

        _upper_x.fetch_max(upper_x);
        _upper_y.fetch_max(upper_y);
        _upper_z.fetch_max(upper_z);        
      }
      
      __forceinline AABB3f sub_group_reduce() const
      {
        AABB3f result;
        result.lower_x = embree::sub_group_reduce(lower_x, SYCL_EXT_ONEAPI::minimum<float>());
        result.lower_y = embree::sub_group_reduce(lower_y, SYCL_EXT_ONEAPI::minimum<float>());
        result.lower_z = embree::sub_group_reduce(lower_z, SYCL_EXT_ONEAPI::minimum<float>());
        result.upper_x = embree::sub_group_reduce(upper_x, SYCL_EXT_ONEAPI::maximum<float>());
        result.upper_y = embree::sub_group_reduce(upper_y, SYCL_EXT_ONEAPI::maximum<float>());
        result.upper_z = embree::sub_group_reduce(upper_z, SYCL_EXT_ONEAPI::maximum<float>());        
        return result;	
      }

      __forceinline AABB3f sub_group_broadcast(const sycl::id<1> &localID) const
      {
        AABB3f result;
        result.lower_x = embree::sub_group_broadcast<float>(lower_x,localID);
        result.lower_y = embree::sub_group_broadcast<float>(lower_y,localID);
        result.lower_z = embree::sub_group_broadcast<float>(lower_z,localID);
        result.upper_x = embree::sub_group_broadcast<float>(upper_x,localID);
        result.upper_y = embree::sub_group_broadcast<float>(upper_y,localID);
        result.upper_z = embree::sub_group_broadcast<float>(upper_z,localID);
        return result;	
      }

      __forceinline AABB3f sub_group_shuffle(const sycl::id<1> &localID) const
      {
        AABB3f result;
        result.lower_x = embree::sub_group_shuffle<float>(lower_x,localID);
        result.lower_y = embree::sub_group_shuffle<float>(lower_y,localID);
        result.lower_z = embree::sub_group_shuffle<float>(lower_z,localID);
        result.upper_x = embree::sub_group_shuffle<float>(upper_x,localID);
        result.upper_y = embree::sub_group_shuffle<float>(upper_y,localID);
        result.upper_z = embree::sub_group_shuffle<float>(upper_z,localID);
        return result;	
      }
      

      __forceinline AABB3f sub_group_scan_exclusive_min_max() const
      {
        AABB3f result;
        result.lower_x = embree::sub_group_exclusive_scan(lower_x, SYCL_EXT_ONEAPI::minimum<float>());
        result.lower_y = embree::sub_group_exclusive_scan(lower_y, SYCL_EXT_ONEAPI::minimum<float>());
        result.lower_z = embree::sub_group_exclusive_scan(lower_z, SYCL_EXT_ONEAPI::minimum<float>());
        result.upper_x = embree::sub_group_exclusive_scan(upper_x, SYCL_EXT_ONEAPI::maximum<float>());
        result.upper_y = embree::sub_group_exclusive_scan(upper_y, SYCL_EXT_ONEAPI::maximum<float>());
        result.upper_z = embree::sub_group_exclusive_scan(upper_z, SYCL_EXT_ONEAPI::maximum<float>());
        return result;	
      }

      __forceinline AABB3f sub_group_scan_inclusive_min_max() const
	
      {
        AABB3f result;
        result.lower_x = embree::sub_group_inclusive_scan(lower_x, SYCL_EXT_ONEAPI::minimum<float>());
        result.lower_y = embree::sub_group_inclusive_scan(lower_y, SYCL_EXT_ONEAPI::minimum<float>());
        result.lower_z = embree::sub_group_inclusive_scan(lower_z, SYCL_EXT_ONEAPI::minimum<float>());
        result.upper_x = embree::sub_group_inclusive_scan(upper_x, SYCL_EXT_ONEAPI::maximum<float>());
        result.upper_y = embree::sub_group_inclusive_scan(upper_y, SYCL_EXT_ONEAPI::maximum<float>());
        result.upper_z = embree::sub_group_inclusive_scan(upper_z, SYCL_EXT_ONEAPI::maximum<float>());
        return result;	
      }

      __forceinline AABB3f work_group_reduce() const
      {
        AABB3f result;
        result.lower_x = work_group_reduce_min(lower_x);
        result.lower_y = work_group_reduce_min(lower_y);
        result.lower_z = work_group_reduce_min(lower_z);
        result.upper_x = work_group_reduce_max(upper_x);
        result.upper_y = work_group_reduce_max(upper_y);
        result.upper_z = work_group_reduce_max(upper_z);
        return result;	
      }

      __forceinline bool isValid()
      {
        if (lower_x > upper_x) return false;
        if (lower_y > upper_y) return false;
        if (lower_z > upper_z) return false;        
        return true;
      }

      __forceinline float halfArea() const
      {
        const float3 d = size();
        return halfarea(d);
      }

      __forceinline float area() const
      {
        return halfArea() * 2.0f;
      }
      
      __forceinline float maxAbsElement() const
      {
        const float3 abs_l = abs(lower());
        const float3 abs_h = abs(upper());
        const float max_abs_l = max(max(abs_l.x(),abs_l.y()),abs_l.z());
        const float max_abs_h = max(max(abs_h.x(),abs_h.y()),abs_h.z());
        return max(max_abs_l,max_abs_h);
      }

      __forceinline AABB3f sub_group_masked_reduce(const bool cmp) const
      {
        AABB3f result;
        const float _pos_inf =  INFINITY;
        const float _neg_inf = -INFINITY;
        result.lower_x = cselect(cmp,lower_x,_pos_inf);
        result.lower_y = cselect(cmp,lower_y,_pos_inf);
        result.lower_z = cselect(cmp,lower_z,_pos_inf);
        result.upper_x = cselect(cmp,upper_x,_neg_inf);
        result.upper_y = cselect(cmp,upper_y,_neg_inf);
        result.upper_z = cselect(cmp,upper_z,_neg_inf);
        return result.sub_group_reduce();
      }
      
      __forceinline AABB3f sub_group_invalidate_reduce_left(const uint pos) const
      {
        AABB3f result;
        const uint subgroupLocalID = get_sub_group_local_id();
        const bool cmp = subgroupLocalID <= pos;
        const float _pos_inf =  INFINITY;
        const float _neg_inf = -INFINITY;
        result.lower_x = cselect(cmp,lower_x,_pos_inf);
        result.lower_y = cselect(cmp,lower_y,_pos_inf);
        result.lower_z = cselect(cmp,lower_z,_pos_inf);
        result.upper_x = cselect(cmp,upper_x,_neg_inf);
        result.upper_y = cselect(cmp,upper_y,_neg_inf);
        result.upper_z = cselect(cmp,upper_z,_neg_inf);
        return result.sub_group_reduce();
      }

      __forceinline AABB3f sub_group_invalidate_reduce_right(const uint pos) const
      {
        AABB3f result;
        const uint subgroupLocalID = get_sub_group_local_id();
        const bool cmp = subgroupLocalID > pos;
        const float _pos_inf =  INFINITY;
        const float _neg_inf = -INFINITY;        
        result.lower_x = cselect(cmp,lower_x,_pos_inf);
        result.lower_y = cselect(cmp,lower_y,_pos_inf);
        result.lower_z = cselect(cmp,lower_z,_pos_inf);
        result.upper_x = cselect(cmp,upper_x,_neg_inf);
        result.upper_y = cselect(cmp,upper_y,_neg_inf);
        result.upper_z = cselect(cmp,upper_z,_neg_inf);        
        return result.sub_group_reduce();
      }      

      __forceinline AABB3f conservativeBounds() const {
        const float ulps = 1.0f;
        const float3 abs_l = abs(lower());
        const float3 abs_h = abs(upper());
        const float max_abs_l = max(max(abs_l.x(),abs_l.y()),abs_l.z());
        const float max_abs_h = max(max(abs_h.x(),abs_h.y()),abs_h.z());             
        const float err = ulps*std::numeric_limits<float>::epsilon() * max(max_abs_l,max_abs_h);
        AABB3f bounds(lower(),upper());
        bounds.enlarge(err);
        return bounds;
      }
      
      __forceinline BBox3f convert() const
      {
        return BBox3f(Vec3f(lower_x,lower_y,lower_z),Vec3f(upper_x,upper_y,upper_z));
      }

      __forceinline bool checkNumericalBounds() const
      {
        const static float FLT_LARGE = 1.844E18f;
        if (lower_x < -FLT_LARGE || upper_x > FLT_LARGE) return false;
        if (lower_y < -FLT_LARGE || upper_y > FLT_LARGE) return false;
        if (lower_z < -FLT_LARGE || upper_z > FLT_LARGE) return false;
        return true;
      }

      __forceinline uint numEqualDims() const
      {
        uint equal_dims = lower_x == upper_x ? 1 : 0;
        equal_dims += lower_y == upper_y ? 1 : 0;
        equal_dims += lower_z == upper_z ? 1 : 0;
        return equal_dims;
      }

      
      friend __forceinline embree_ostream operator<<(embree_ostream cout, const AABB3f &aabb)
      {
        cout << "AABB3f { ";
        cout << "  lower = (" << aabb.lower_x << ", " << aabb.lower_y << ", " << aabb.lower_z << ") ";
        cout << "  upper = (" << aabb.upper_x << ", " << aabb.upper_y << ", " << aabb.upper_z << ") ";
        return cout << "}";
      }
    };

      __forceinline AABB3f merge(const AABB3f &a, const AABB3f &b)
      {
        AABB3f bounds;
        bounds.lower_x = min(a.lower_x,b.lower_x);
        bounds.lower_y = min(a.lower_y,b.lower_y);
        bounds.lower_z = min(a.lower_z,b.lower_z);        
        bounds.upper_x = max(a.upper_x,b.upper_x);
        bounds.upper_y = max(a.upper_y,b.upper_y);
        bounds.upper_z = max(a.upper_z,b.upper_z);
        return bounds;
      }


      __forceinline AABB3f intersect(const AABB3f &a, const AABB3f &b)
      {
        AABB3f bounds;
        bounds.lower_x = max(a.lower_x,b.lower_x);
        bounds.lower_y = max(a.lower_y,b.lower_y);
        bounds.lower_z = max(a.lower_z,b.lower_z);        
        bounds.upper_x = min(a.upper_x,b.upper_x);
        bounds.upper_y = min(a.upper_y,b.upper_y);
        bounds.upper_z = min(a.upper_z,b.upper_z);
        return bounds;
      }




      __forceinline float distance(const AABB3f &a, const AABB3f &b)
      {
        const float3 dist_lower = a.lower() - b.lower();
        const float3 dist_upper = a.upper() - b.upper();
        return sycl::fma(dist_lower.x(),dist_lower.x(),
                         sycl::fma(dist_lower.y(),dist_lower.y(),
                                   sycl::fma(dist_lower.z(),dist_lower.z(),
                                             sycl::fma(dist_upper.x(),dist_upper.x(),
                                                       sycl::fma(dist_upper.y(), dist_upper.y(), dist_upper.z() * dist_upper.z())))));        
      }
    
    __forceinline bool operator ==(const AABB3f& a, const AABB3f& b) {
        if (a.lower_x != b.lower_x ||
            a.lower_y != b.lower_y ||
            a.lower_z != b.lower_z ||
            a.upper_x != b.upper_x ||
            a.upper_y != b.upper_y ||
            a.upper_z != b.upper_z)
          return false;
        return true;
    }
    
    __forceinline bool operator !=(const AABB3f& a, const AABB3f& b) {
      return !(a==b);
    }
    
    
  }
}

#endif
