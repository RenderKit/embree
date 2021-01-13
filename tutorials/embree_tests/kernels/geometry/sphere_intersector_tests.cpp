// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../../../external/catch.hpp"
#include "../../../../kernels/geometry/sphere_intersector.h"
#include "../../../../common/simd/sse.cpp"

using namespace embree;

namespace __sphere_unit_tests_internal {
  
  struct fakeEpilog {
    
    fakeEpilog (Ray& ray)
      : ray_(ray) {}
    
    bool operator ()(const vbool<4> & , 
                     const isa::SphereIntersectorHitM<4>& hit) const {
      
      ray_.id = -1;
      for (auto i=0; i<4; ++i) {
        if (hit.vt[i] > 1.e-5f && hit.vt[i] < ray_.tfar) {
          ray_.tfar = hit.vt[i];
          ray_.id = i;
        }
      }
      return ray_.id != -1;
    }
    
    Ray& ray_;
  };
  
}

TEST_CASE ("Overlapping spheres with filtering - Issue 676 fix-intersection-epilog-handling", "[spheres]") 
{
  isa::CurvePrecalculations1 pre;
  vbool<4> valid {true, true, false, false};
  Vec3fa org (14.8001127f, -9.01768494f, 3.47012758f);
  Vec3fa dir (-0.989340246f, -0.0190101117f, -0.144376263f);
  Ray ray (org, dir);
  Vec4vf<4> v0;
  v0.x = vfloat<4>{ 9.66870880f,  10.0441875f, 0.f, 0.f};
  v0.y = vfloat<4>{-16.3965702f, -9.69345284f, 0.f, 0.f};
  v0.z = vfloat<4>{ 3.93995930f,  3.94893074f, 0.f, 0.f};
  v0.w = vfloat<4>{9.f, 9.f, 0.f, 0.f};
  
  __sphere_unit_tests_internal::fakeEpilog epilog (ray);
  
  isa::SphereIntersector1<4>::intersect (valid, ray, pre, v0, epilog);
  int id = ray.id;
  REQUIRE (id == 0);
  REQUIRE (ray.tfar == Approx (10.2983));
}
