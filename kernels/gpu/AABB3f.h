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

#include "../common/device.h"
#include "common.h"

#if defined(EMBREE_DPCPP_SUPPORT)

#include <CL/sycl.hpp>

namespace embree
{
  namespace gpu
  {
    class AABB3f {
    public:
      cl::sycl::float3 lower;
      cl::sycl::float3 upper;
      
      AABB3f() = default;

      inline void init()
      {
	const float pos_inf =  INFINITY;
	const float neg_inf = -INFINITY;
	lower = { pos_inf,pos_inf,pos_inf };
	upper = { neg_inf,neg_inf,neg_inf };	
      }
      
      inline void extend(class AABB3f &aabb)
      {
	lower = min(lower,aabb.lower);
	upper = max(upper,aabb.upper);	
      }

      inline void extend(const cl::sycl::float3 &v)
      {
	lower = min(lower,v);
	upper = max(upper,v);	
      }

      inline void enlarge(const cl::sycl::float3 &v)
      {
	lower -= v;
	upper += v;	
      }

      inline cl::sycl::float3 size()
      {
	return upper - lower;
      }

      inline cl::sycl::float3 centroid2()
      {
	return upper + lower;
      }

      inline void atomic_merge_global(AABB3f &dest)
      {
	atomic_min(((volatile GLOBAL float *)&dest) + 0,lower.x());
	atomic_min(((volatile GLOBAL float *)&dest) + 1,lower.y());
	atomic_min(((volatile GLOBAL float *)&dest) + 2,lower.z());
	
	atomic_max(((volatile GLOBAL float *)&dest) + 4,upper.x());
	atomic_max(((volatile GLOBAL float *)&dest) + 5,upper.y());
	atomic_max(((volatile GLOBAL float *)&dest) + 6,upper.z());	
      }

      inline void atomic_merge_local(AABB3f &dest)
      {
	atomic_min(((volatile LOCAL float *)&dest) + 0,lower.x());
	atomic_min(((volatile LOCAL float *)&dest) + 1,lower.y());
	atomic_min(((volatile LOCAL float *)&dest) + 2,lower.z());
	
	atomic_max(((volatile LOCAL float *)&dest) + 4,upper.x());
	atomic_max(((volatile LOCAL float *)&dest) + 5,upper.y());
	atomic_max(((volatile LOCAL float *)&dest) + 6,upper.z());	
      }
      
      static inline AABB3f sub_group_reduce(cl::sycl::intel::sub_group& sg, const AABB3f &aabb)
      {
	AABB3f result;
	result.lower.x() = sg.reduce<float,cl::sycl::intel::minimum>(aabb.lower.x());
	result.lower.y() = sg.reduce<float,cl::sycl::intel::minimum>(aabb.lower.y());
	result.lower.z() = sg.reduce<float,cl::sycl::intel::minimum>(aabb.lower.z());
	result.upper.x() = sg.reduce<float,cl::sycl::intel::maximum>(aabb.upper.x());
	result.upper.y() = sg.reduce<float,cl::sycl::intel::maximum>(aabb.upper.y());
	result.upper.z() = sg.reduce<float,cl::sycl::intel::maximum>(aabb.upper.z());
	return result;	
      }

      static inline AABB3f sub_group_broadcast(cl::sycl::intel::sub_group& sg, const AABB3f &aabb, const cl::sycl::id<1> &localID)
      {
	AABB3f result;
	result.lower.x() = sg.broadcast<float>(aabb.lower.x(),localID);
	result.lower.y() = sg.broadcast<float>(aabb.lower.y(),localID);
	result.lower.z() = sg.broadcast<float>(aabb.lower.z(),localID);
	result.upper.x() = sg.broadcast<float>(aabb.upper.x(),localID);
	result.upper.y() = sg.broadcast<float>(aabb.upper.y(),localID);
	result.upper.z() = sg.broadcast<float>(aabb.upper.z(),localID);
	return result;	
      }

      static inline AABB3f sub_group_scan_exclusive_min_max(cl::sycl::intel::sub_group& sg, const AABB3f &aabb, const cl::sycl::id<1> &localID)
      {
	AABB3f result;
	result.lower.x() = sg.exclusive_scan<float,cl::sycl::intel::minimum>(aabb.lower.x());
	result.lower.y() = sg.exclusive_scan<float,cl::sycl::intel::minimum>(aabb.lower.y());
	result.lower.z() = sg.exclusive_scan<float,cl::sycl::intel::minimum>(aabb.lower.z());
	result.upper.x() = sg.exclusive_scan<float,cl::sycl::intel::maximum>(aabb.upper.x());
	result.upper.y() = sg.exclusive_scan<float,cl::sycl::intel::maximum>(aabb.upper.y());
	result.upper.z() = sg.exclusive_scan<float,cl::sycl::intel::maximum>(aabb.upper.z());
	return result;	
      }

      static inline AABB3f sub_group_scan_inclusive_min_max(cl::sycl::intel::sub_group& sg, const AABB3f &aabb, const cl::sycl::id<1> &localID)
      {
	AABB3f result;
	result.lower.x() = sg.inclusive_scan<float,cl::sycl::intel::minimum>(aabb.lower.x());
	result.lower.y() = sg.inclusive_scan<float,cl::sycl::intel::minimum>(aabb.lower.y());
	result.lower.z() = sg.inclusive_scan<float,cl::sycl::intel::minimum>(aabb.lower.z());
	result.upper.x() = sg.inclusive_scan<float,cl::sycl::intel::maximum>(aabb.upper.x());
	result.upper.y() = sg.inclusive_scan<float,cl::sycl::intel::maximum>(aabb.upper.y());
	result.upper.z() = sg.inclusive_scan<float,cl::sycl::intel::maximum>(aabb.upper.z());
	return result;	
      }

      static inline AABB3f work_group_reduce(const AABB3f &aabb)
      {
	AABB3f result;
	result.lower.x() = work_group_reduce_min(aabb.lower.x());
	result.lower.y() = work_group_reduce_min(aabb.lower.y());
	result.lower.z() = work_group_reduce_min(aabb.lower.z());
	result.upper.x() = work_group_reduce_max(aabb.upper.x());
	result.upper.y() = work_group_reduce_max(aabb.upper.y());
	result.upper.z() = work_group_reduce_max(aabb.upper.z());
	return result;	
      }

      
    };
  };
};

#endif
