// ======================================================================== //
// Copyright 2009-2018 Intel Corporation                                    //
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
  namespace SceneGraph
  {
    enum LightType
    {
      LIGHT_AMBIENT,
      LIGHT_POINT,
      LIGHT_DIRECTIONAL,
      LIGHT_SPOT,
      LIGHT_DISTANT,
      LIGHT_TRIANGLE,
      LIGHT_QUAD,
    };

    class Light : public RefCount
    {
      ALIGNED_CLASS_(16)

    public:
      Light(LightType type) : type(type) {}

      LightType getType() const { return type; }
      virtual Ref<Light> transform(const AffineSpace3fa& space) const = 0;

    private:
      LightType type;
    };

    class AmbientLight : public Light
    {
    public:
      AmbientLight (const Vec3fa& L)
        : Light(LIGHT_AMBIENT), L(L) {}

      Ref<Light> transform(const AffineSpace3fa& space) const {
        return new AmbientLight(L);
      }

    public:
      Vec3fa L;                  //!< radiance of ambient light
    };

    class PointLight : public Light
    {
    public:
      PointLight (const Vec3fa& P, const Vec3fa& I)
        : Light(LIGHT_POINT), P(P), I(I) {}

      Ref<Light> transform(const AffineSpace3fa& space) const {
        return new PointLight(xfmPoint(space,P),I);
      }

    public:
      Vec3fa P;                  //!< position of point light
      Vec3fa I;                  //!< radiant intensity of point light
    };

    class DirectionalLight : public Light
    {
    public:
      DirectionalLight (const Vec3fa& D, const Vec3fa& E)
        : Light(LIGHT_DIRECTIONAL), D(D), E(E) {}

      Ref<Light> transform(const AffineSpace3fa& space) const {
        return new DirectionalLight(xfmVector(space,D),E);
      }

    public:
      Vec3fa D;                  //!< Light direction
      Vec3fa E;                  //!< Irradiance (W/m^2)
    };

    class SpotLight : public Light
    {
    public:
      SpotLight (const Vec3fa& P, const Vec3fa& D, const Vec3fa& I, float angleMin, float angleMax)
        : Light(LIGHT_SPOT), P(P), D(D), I(I), angleMin(angleMin), angleMax(angleMax) {}

      Ref<Light> transform(const AffineSpace3fa& space) const {
        return new SpotLight(xfmPoint(space,P),xfmVector(space,D),I,angleMin,angleMax);
      }

    public:
      Vec3fa P;                 //!< Position of the spot light
      Vec3fa D;                 //!< Light direction of the spot light
      Vec3fa I;                 //!< Radiant intensity (W/sr)
      float angleMin, angleMax; //!< Linear falloff region
    };

    class DistantLight : public Light
    {
    public:
      DistantLight (const Vec3fa& D, const Vec3fa& L, const float halfAngle)
        : Light(LIGHT_DISTANT), D(D), L(L), halfAngle(halfAngle), radHalfAngle(deg2rad(halfAngle)), cosHalfAngle(cos(deg2rad(halfAngle))) {}

      Ref<Light> transform(const AffineSpace3fa& space) const {
        return new DistantLight(xfmVector(space,D),L,halfAngle);
      }

    public:
      Vec3fa D;            //!< Light direction
      Vec3fa L;            //!< Radiance (W/(m^2*sr))
      float halfAngle;     //!< Half illumination angle
      float radHalfAngle;  //!< Half illumination angle in radians
      float cosHalfAngle;  //!< Cosine of half illumination angle
    };

    class TriangleLight : public Light
    {
    public:
      TriangleLight (const Vec3fa& v0, const Vec3fa& v1, const Vec3fa& v2, const Vec3fa& L)
        : Light(LIGHT_TRIANGLE), v0(v0), v1(v1), v2(v2), L(L) {}

      Ref<Light> transform(const AffineSpace3fa& space) const {
        return new TriangleLight(xfmPoint(space,v0),xfmPoint(space,v1),xfmPoint(space,v2),L);
      }

    public:
      Vec3fa v0;
      Vec3fa v1;
      Vec3fa v2;
      Vec3fa L;
    };

    class QuadLight : public Light
    {
    public:
      QuadLight (const Vec3fa& v0, const Vec3fa& v1, const Vec3fa& v2, const Vec3fa& v3, const Vec3fa& L)
        : Light(LIGHT_QUAD), v0(v0), v1(v1), v2(v2), v3(v3), L(L) {}

      Ref<Light> transform(const AffineSpace3fa& space) const {
        return new QuadLight(xfmPoint(space,v0),xfmPoint(space,v1),xfmPoint(space,v2),xfmPoint(space,v3),L);
      }

    public:
      Vec3fa v0;
      Vec3fa v1;
      Vec3fa v2;
      Vec3fa v3;
      Vec3fa L;
    };
  }
}
