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

#pragma once

#include "../default.h"

namespace embree
{  
  struct AmbientLight
  {
  public:
    AmbientLight () {}
    
    AmbientLight (const Vec3fa& L) 
    : L(L) {}
    
    const AmbientLight transform(const AffineSpace3fa& space) const {
      return AmbientLight(L);
    }
    
  public:
    Vec3fa L;                  //!< radiance of ambient light
  };
  
  struct PointLight
  {
  public:
    PointLight() {}
    
    PointLight (const Vec3fa& P, const Vec3fa& I) 
    : P(P), I(I) {}
    
    const PointLight transform(const AffineSpace3fa& space) const {
      return PointLight(xfmPoint(space,P),I);
    }
    
  public:
    Vec3fa P;                  //!< position of point light
    Vec3fa I;                  //!< radiant intensity of point light
  };
  
  struct DirectionalLight
  {
  public:
    DirectionalLight () {}
    
    DirectionalLight (const Vec3fa& D, const Vec3fa& E) 
    : D(D), E(E) {}
    
    const DirectionalLight transform(const AffineSpace3fa& space) const {
      return DirectionalLight(xfmVector(space,D),E);
    }
    
  public:
    Vec3fa D;                  //!< Light direction
    Vec3fa E;                  //!< Irradiance (W/m^2)
  };
  
  struct SpotLight
  {
    SpotLight () {}
    
    SpotLight (const Vec3fa& P, const Vec3fa& D, const Vec3fa& I, const float angleMin, const float angleMax)
    : P(P), D(D), I(I), angleMin(angleMin), angleMax(angleMax) {}
    
    const SpotLight transform(const AffineSpace3fa& space) const {
      return SpotLight(xfmPoint(space,P),xfmVector(space,D),I,angleMin,angleMax);
    }
    
  public:
    Vec3fa P;                 //!< Position of the spot light
    Vec3fa D;                 //!< Light direction of the spot light
    Vec3fa I;                 //!< Radiant intensity (W/sr)
    float angleMin, angleMax; //!< Linear falloff region
  };
  
  struct DistantLight
  {
  public:
    DistantLight () {}
    
    DistantLight (const Vec3fa& D, const Vec3fa& L, const float halfAngle) 
    : D(D), L(L), halfAngle(halfAngle), radHalfAngle(deg2rad(halfAngle)), cosHalfAngle(cos(deg2rad(halfAngle))) {}
    
    const DistantLight transform(const AffineSpace3fa& space) const {
      return DistantLight(xfmVector(space,D),L,halfAngle);
    }
    
  public:
    Vec3fa D;             //!< Light direction
    Vec3fa L;             //!< Radiance (W/(m^2*sr))
    float halfAngle;     //!< Half illumination angle
    float radHalfAngle;  //!< Half illumination angle in radians
    float cosHalfAngle;  //!< Cosine of half illumination angle
  };
  
  struct TriangleLight
  {
    TriangleLight() {}
    
    TriangleLight (const Vec3fa& v0, const Vec3fa& v1, const Vec3fa& v2, const Vec3fa& L) 
    : v0(v0), v1(v1), v2(v2), L(L) {}
    
    const TriangleLight transform(const AffineSpace3fa& space) const {
      return TriangleLight(xfmPoint(space,v0),xfmPoint(space,v1),xfmPoint(space,v2),L);
    }
    
  public:
    Vec3fa v0;
    Vec3fa v1;
    Vec3fa v2;
    Vec3fa L;
  };
  
  struct QuadLight
  {
    QuadLight() {}
    
    QuadLight (const Vec3fa& v0, const Vec3fa& v1, const Vec3fa& v2, const Vec3fa& v3, const Vec3fa& L) 
    : v0(v0), v1(v1), v2(v2), v3(v3), L(L) {}
    
    const QuadLight transform(const AffineSpace3fa& space) const {
      return QuadLight(xfmPoint(space,v0),xfmPoint(space,v1),xfmPoint(space,v2),xfmPoint(space,v3),L);
    }
    
  public:
    Vec3fa v0;
    Vec3fa v1;
    Vec3fa v2;
    Vec3fa v3;
    Vec3fa L;
  };
}
