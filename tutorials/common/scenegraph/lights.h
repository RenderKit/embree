// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

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

    class Light
    {
      ALIGNED_CLASS_(16)

    public:
      Light(LightType type) : type(type) {}

      LightType getType() const { return type; }

    private:
      LightType type;
    };

    class AmbientLight : public Light
    {
    public:
      AmbientLight (const Vec3fa& L)
        : Light(LIGHT_AMBIENT), L(L) {}

      AmbientLight transform(const AffineSpace3fa& space) const {
        return AmbientLight(L);
      }

      static AmbientLight lerp(const AmbientLight& light0, const AmbientLight& light1, const float f) {
        return AmbientLight(embree::lerp(light0.L,light1.L,f));
      }

    public:
      Vec3fa L;                  //!< radiance of ambient light
    };

    class PointLight : public Light
    {
    public:
      PointLight (const Vec3fa& P, const Vec3fa& I)
        : Light(LIGHT_POINT), P(P), I(I) {}

      PointLight transform(const AffineSpace3fa& space) const {
        return PointLight(xfmPoint(space,P),I);
      }

      static PointLight lerp(const PointLight& light0, const PointLight& light1, const float f)
      {
        return PointLight(embree::lerp(light0.P,light1.P,f),
                          embree::lerp(light0.I,light1.I,f));
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

      DirectionalLight transform(const AffineSpace3fa& space) const {
        return DirectionalLight(xfmVector(space,D),E);
      }

      static DirectionalLight lerp(const DirectionalLight& light0, const DirectionalLight& light1, const float f)
      {
        return DirectionalLight(embree::lerp(light0.D,light1.D,f),
                                embree::lerp(light0.E,light1.E,f));
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

      SpotLight transform(const AffineSpace3fa& space) const {
        return SpotLight(xfmPoint(space,P),xfmVector(space,D),I,angleMin,angleMax);
      }

      static SpotLight lerp(const SpotLight& light0, const SpotLight& light1, const float f)
      {
        return SpotLight(embree::lerp(light0.P,light1.P,f),
                         embree::lerp(light0.D,light1.D,f),
                         embree::lerp(light0.I,light1.I,f),
                         embree::lerp(light0.angleMin,light1.angleMin,f),
                         embree::lerp(light0.angleMax,light1.angleMax,f));
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

      DistantLight transform(const AffineSpace3fa& space) const {
        return DistantLight(xfmVector(space,D),L,halfAngle);
      }

      static DistantLight lerp(const DistantLight& light0, const DistantLight& light1, const float f)
      {
        return DistantLight(embree::lerp(light0.D,light1.D,f),
                            embree::lerp(light0.L,light1.L,f),
                            embree::lerp(light0.halfAngle,light1.halfAngle,f));
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

      TriangleLight transform(const AffineSpace3fa& space) const {
        return TriangleLight(xfmPoint(space,v0),xfmPoint(space,v1),xfmPoint(space,v2),L);
      }

      static TriangleLight lerp(const TriangleLight& light0, const TriangleLight& light1, const float f)
      {
        return TriangleLight(embree::lerp(light0.v0,light1.v0,f),
                             embree::lerp(light0.v1,light1.v1,f),
                             embree::lerp(light0.v2,light1.v2,f),
                             embree::lerp(light0.L,light1.L,f));
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

      QuadLight transform(const AffineSpace3fa& space) const {
        return QuadLight(xfmPoint(space,v0),xfmPoint(space,v1),xfmPoint(space,v2),xfmPoint(space,v3),L);
      }

      static QuadLight lerp(const QuadLight& light0, const QuadLight& light1, const float f)
      {
        return QuadLight(embree::lerp(light0.v0,light1.v0,f),
                         embree::lerp(light0.v1,light1.v1,f),
                         embree::lerp(light0.v2,light1.v2,f),
                         embree::lerp(light0.v3,light1.v3,f),
                         embree::lerp(light0.L,light1.L,f));
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
