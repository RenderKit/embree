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

#include "../gpu/common.h"

namespace embree
{
  namespace gpu
  {
    class AABB {
    public:
      cl::sycl::float4 lower;
      cl::sycl::float4 upper;

      AABB() = default;

      AABB(const AABB &aabb) : lower(aabb.lower),upper(aabb.upper) {} // enabling this causes a compile error ???

      AABB(const cl::sycl::float4 &v) : lower(v),upper(v) {}
      
      inline void init()
      {
	const float pos_inf =  INFINITY;
	const float neg_inf = -INFINITY;
	lower = { pos_inf,pos_inf,pos_inf,0 };
	upper = { neg_inf,neg_inf,neg_inf,0 };	
      }
      
      inline void extend(const AABB &aabb)
      {
	lower = min(lower,aabb.lower);
	upper = max(upper,aabb.upper);	
      }

      inline void extend(const cl::sycl::float4 &v)
      {
	lower = min(lower,v);
	upper = max(upper,v);	
      }
      
      inline void enlarge(const cl::sycl::float4 &v)
      {
	lower -= v;
	upper += v;	
      }

      inline cl::sycl::float4 size() const
      {
	return upper - lower;
      }

      inline cl::sycl::float4 centroid2() const
      {
	return upper + lower;
      }

      inline void atomic_merge_global(GLOBAL AABB &dest) const
      {
	atomic_min(((volatile GLOBAL float *)&dest) + 0,lower.x());
	atomic_min(((volatile GLOBAL float *)&dest) + 1,lower.y());
	atomic_min(((volatile GLOBAL float *)&dest) + 2,lower.z());

	atomic_max(((volatile GLOBAL float *)&dest) + 4,upper.x());
	atomic_max(((volatile GLOBAL float *)&dest) + 5,upper.y());
	atomic_max(((volatile GLOBAL float *)&dest) + 6,upper.z());	
      }

      inline void atomic_merge_local(GLOBAL AABB &dest) const
      {
	atomic_min(((volatile LOCAL float *)&dest) + 0,lower.x());
	atomic_min(((volatile LOCAL float *)&dest) + 1,lower.y());
	atomic_min(((volatile LOCAL float *)&dest) + 2,lower.z());

	atomic_max(((volatile LOCAL float *)&dest) + 4,upper.x());
	atomic_max(((volatile LOCAL float *)&dest) + 5,upper.y());
	atomic_max(((volatile LOCAL float *)&dest) + 6,upper.z());	
      }
      
      inline AABB sub_group_reduce(cl::sycl::intel::sub_group& sg) const
      {
	AABB result;
	result.lower.x() = sg.reduce<float,cl::sycl::intel::minimum>(lower.x());
	result.lower.y() = sg.reduce<float,cl::sycl::intel::minimum>(lower.y());
	result.lower.z() = sg.reduce<float,cl::sycl::intel::minimum>(lower.z());
	result.lower.w() = 0.0f;
	result.upper.x() = sg.reduce<float,cl::sycl::intel::maximum>(upper.x());
	result.upper.y() = sg.reduce<float,cl::sycl::intel::maximum>(upper.y());
	result.upper.z() = sg.reduce<float,cl::sycl::intel::maximum>(upper.z());
	result.upper.w() = 0.0f;	
	return result;	
      }

      inline AABB sub_group_broadcast(cl::sycl::intel::sub_group& sg, const cl::sycl::id<1> &localID) const
      {
	AABB result;
	result.lower.x() = sg.broadcast<float>(lower.x(),localID);
	result.lower.y() = sg.broadcast<float>(lower.y(),localID);
	result.lower.z() = sg.broadcast<float>(lower.z(),localID);
	result.lower.w() = 0.0f;
	result.upper.x() = sg.broadcast<float>(upper.x(),localID);
	result.upper.y() = sg.broadcast<float>(upper.y(),localID);
	result.upper.z() = sg.broadcast<float>(upper.z(),localID);
	result.upper.w() = 0.0f;	
	return result;	
      }

      inline AABB sub_group_scan_exclusive_min_max(cl::sycl::intel::sub_group& sg) const
      {
	AABB result;
	result.lower.x() = sg.exclusive_scan<float,cl::sycl::intel::minimum>(lower.x());
	result.lower.y() = sg.exclusive_scan<float,cl::sycl::intel::minimum>(lower.y());
	result.lower.z() = sg.exclusive_scan<float,cl::sycl::intel::minimum>(lower.z());
	result.lower.w() = 0.0f;
	result.upper.x() = sg.exclusive_scan<float,cl::sycl::intel::maximum>(upper.x());
	result.upper.y() = sg.exclusive_scan<float,cl::sycl::intel::maximum>(upper.y());
	result.upper.z() = sg.exclusive_scan<float,cl::sycl::intel::maximum>(upper.z());
	result.upper.w() = 0.0f;	
	return result;	
      }

      inline AABB sub_group_scan_inclusive_min_max(cl::sycl::intel::sub_group& sg) const 
      {
	AABB result;
	result.lower.x() = sg.inclusive_scan<float,cl::sycl::intel::minimum>(lower.x());
	result.lower.y() = sg.inclusive_scan<float,cl::sycl::intel::minimum>(lower.y());
	result.lower.z() = sg.inclusive_scan<float,cl::sycl::intel::minimum>(lower.z());
	result.lower.w() = 0.0f;
	result.upper.x() = sg.inclusive_scan<float,cl::sycl::intel::maximum>(upper.x());
	result.upper.y() = sg.inclusive_scan<float,cl::sycl::intel::maximum>(upper.y());
	result.upper.z() = sg.inclusive_scan<float,cl::sycl::intel::maximum>(upper.z());
	result.upper.w() = 0.0f;	
	return result;	
      }

      inline AABB work_group_reduce() const
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
            
    };

    inline const cl::sycl::stream &operator<<(const cl::sycl::stream &out, const AABB& aabb) {
      return out << "lower " << aabb.lower << "  upper " << aabb.upper;
    }

  };
};

#endif
