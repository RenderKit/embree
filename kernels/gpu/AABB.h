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
      
      inline void extend(AABB &aabb)
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

      inline void atomic_merge_global(GLOBAL AABB &dest)
      {
	atomic_min(((volatile GLOBAL float *)&dest) + 0,lower.x());
	atomic_min(((volatile GLOBAL float *)&dest) + 1,lower.y());
	atomic_min(((volatile GLOBAL float *)&dest) + 2,lower.z());

	atomic_max(((volatile GLOBAL float *)&dest) + 4,upper.x());
	atomic_max(((volatile GLOBAL float *)&dest) + 5,upper.y());
	atomic_max(((volatile GLOBAL float *)&dest) + 6,upper.z());	
      }
      
      static inline AABB sub_group_reduce(cl::sycl::intel::sub_group& sg, const AABB &aabb)
      {
	AABB result;
	result.lower.x() = sg.reduce<float,cl::sycl::intel::minimum>(aabb.lower.x());
	result.lower.y() = sg.reduce<float,cl::sycl::intel::minimum>(aabb.lower.y());
	result.lower.z() = sg.reduce<float,cl::sycl::intel::minimum>(aabb.lower.z());
	result.lower.w() = 0.0f;
	result.upper.x() = sg.reduce<float,cl::sycl::intel::maximum>(aabb.upper.x());
	result.upper.y() = sg.reduce<float,cl::sycl::intel::maximum>(aabb.upper.y());
	result.upper.z() = sg.reduce<float,cl::sycl::intel::maximum>(aabb.upper.z());
	result.upper.w() = 0.0f;	
	return result;	
      }

      static inline AABB sub_group_broadcast(cl::sycl::intel::sub_group& sg, const AABB &aabb, const cl::sycl::id<1> &localID)
      {
	AABB result;
	result.lower.x() = sg.broadcast<float>(aabb.lower.x(),localID);
	result.lower.y() = sg.broadcast<float>(aabb.lower.y(),localID);
	result.lower.z() = sg.broadcast<float>(aabb.lower.z(),localID);
	result.lower.w() = 0.0f;
	result.upper.x() = sg.broadcast<float>(aabb.upper.x(),localID);
	result.upper.y() = sg.broadcast<float>(aabb.upper.y(),localID);
	result.upper.z() = sg.broadcast<float>(aabb.upper.z(),localID);
	result.upper.w() = 0.0f;	
	return result;	
      }

      static inline AABB sub_group_scan_exclusive_min_max(cl::sycl::intel::sub_group& sg, const AABB &aabb, const cl::sycl::id<1> &localID)
      {
	AABB result;
	result.lower.x() = sg.exclusive_scan<float,cl::sycl::intel::minimum>(aabb.lower.x());
	result.lower.y() = sg.exclusive_scan<float,cl::sycl::intel::minimum>(aabb.lower.y());
	result.lower.z() = sg.exclusive_scan<float,cl::sycl::intel::minimum>(aabb.lower.z());
	result.lower.w() = 0.0f;
	result.upper.x() = sg.exclusive_scan<float,cl::sycl::intel::maximum>(aabb.upper.x());
	result.upper.y() = sg.exclusive_scan<float,cl::sycl::intel::maximum>(aabb.upper.y());
	result.upper.z() = sg.exclusive_scan<float,cl::sycl::intel::maximum>(aabb.upper.z());
	result.upper.w() = 0.0f;	
	return result;	
      }

      static inline AABB sub_group_scan_inclusive_min_max(cl::sycl::intel::sub_group& sg, const AABB &aabb, const cl::sycl::id<1> &localID)
      {
	AABB result;
	result.lower.x() = sg.inclusive_scan<float,cl::sycl::intel::minimum>(aabb.lower.x());
	result.lower.y() = sg.inclusive_scan<float,cl::sycl::intel::minimum>(aabb.lower.y());
	result.lower.z() = sg.inclusive_scan<float,cl::sycl::intel::minimum>(aabb.lower.z());
	result.lower.w() = 0.0f;
	result.upper.x() = sg.inclusive_scan<float,cl::sycl::intel::maximum>(aabb.upper.x());
	result.upper.y() = sg.inclusive_scan<float,cl::sycl::intel::maximum>(aabb.upper.y());
	result.upper.z() = sg.inclusive_scan<float,cl::sycl::intel::maximum>(aabb.upper.z());
	result.upper.w() = 0.0f;	
	return result;	
      }

      static inline AABB work_group_reduce(const AABB &aabb)
      {
	AABB result;
	result.lower.x() = work_group_reduce_min(aabb.lower.x());
	result.lower.y() = work_group_reduce_min(aabb.lower.y());
	result.lower.z() = work_group_reduce_min(aabb.lower.z());
	result.lower.w() = 0.0f;
	result.upper.x() = work_group_reduce_max(aabb.upper.x());
	result.upper.y() = work_group_reduce_max(aabb.upper.y());
	result.upper.z() = work_group_reduce_max(aabb.upper.z());
	result.upper.w() = 0.0f;	
	return result;	
      }
      
      inline void print()
      {
	printf("AABB: lower %f %f %f upper %f %f %f \n",(float)lower.x(),(float)lower.y(),(float)lower.z(),(float)upper.x(),(float)upper.y(),(float)upper.z());	
      }
    };
  };
};

#endif
