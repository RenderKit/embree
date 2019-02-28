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

/*! \brief utility library containing sampling functions */

// convention is to return the sample (Vec3fa) generated from given Vec2f 's'ample as last parameter
// sampling functions often come in pairs: sample and pdf (needed later for MIS)
// good reference is "Total Compendium" by Philip Dutre http://people.cs.kuleuven.be/~philip.dutre/GI/

#include "../math/vec.h"
#include "../math/linearspace.h"

namespace embree {

struct Sample3f
{
  Vec3fa v;
  float pdf;
};

inline Sample3f make_Sample3f(const Vec3fa& v, const float pdf) {
  Sample3f s; s.v = v; s.pdf = pdf; return s;
}

#if defined(ISPC)
inline Sample3f make_Sample3f(const Vec3fa& v, const float pdf) {
  Sample3f s; s.v = v; s.pdf = pdf; return s;
}
#endif

inline Vec3fa cartesian(const float phi, const float sinTheta, const float cosTheta)
{
  float sinPhi, cosPhi;
  sincosf(phi, &sinPhi, &cosPhi);
  return Vec3fa(cosPhi * sinTheta,
                    sinPhi * sinTheta,
                    cosTheta);
}

inline Vec3fa cartesian(const float phi, const float cosTheta)
{
  return cartesian(phi, cos2sin(cosTheta), cosTheta);
}


/// cosine-weighted sampling of hemisphere oriented along the +z-axis
////////////////////////////////////////////////////////////////////////////////

inline Vec3fa cosineSampleHemisphere(const Vec2f s)
{
  const float phi =float(two_pi) * s.x;
  const float cosTheta = sqrt(s.y);
  const float sinTheta = sqrt(1.0f - s.y);
  return cartesian(phi, sinTheta, cosTheta);
}

inline float cosineSampleHemispherePDF(const Vec3fa &dir)
{
  return dir.z / float(pi);
}

inline float cosineSampleHemispherePDF(float cosTheta)
{
  return cosTheta / float(pi);
}

/*! Cosine weighted hemisphere sampling. Up direction is provided as argument. */
inline Sample3f cosineSampleHemisphere(const float  u, const float  v, const Vec3fa& N)
{
  Vec3fa localDir = cosineSampleHemisphere(Vec2f(u,v));
  Sample3f s;
  s.v = frame(N) * localDir;
  s.pdf = cosineSampleHemispherePDF(localDir);
  return s;
}

/// power cosine-weighted sampling of hemisphere oriented along the +z-axis
////////////////////////////////////////////////////////////////////////////////

inline Vec3fa powerCosineSampleHemisphere(const float n, const Vec2f &s)
{
  const float phi =float(two_pi) * s.x;
  const float cosTheta = pow(s.y, 1.0f / (n + 1.0f));
  return cartesian(phi, cosTheta);
}

inline float powerCosineSampleHemispherePDF(const float cosTheta, const float n) // TODO: order of arguments
{
  return (n + 1.0f) * (0.5f / float(pi)) * pow(cosTheta, n);
}

inline float powerCosineSampleHemispherePDF(const Vec3fa& dir, const float n) // TODO: order of arguments
{
  return (n + 1.0f) * (0.5f / float(pi)) * pow(dir.z, n);
}

/// sampling of cone of directions oriented along the +z-axis
////////////////////////////////////////////////////////////////////////////////

inline Vec3fa uniformSampleCone(const float cosAngle, const Vec2f &s)
{
  const float phi =float(two_pi) * s.x;
  const float cosTheta = 1.0f - s.y * (1.0f - cosAngle);
  return cartesian(phi, cosTheta);
}

inline float uniformSampleConePDF(const float cosAngle)
{
    return rcp(float(two_pi)*(1.0f - cosAngle));
}

inline float _uniformSampleConePDF(const float cosAngle)
{
    return rcp(float(two_pi)*(1.0f - cosAngle));
}


/// sampling of disk
////////////////////////////////////////////////////////////////////////////////

inline Vec3fa uniformSampleDisk(const float radius, const Vec2f &s)
{
  const float r = sqrtf(s.x) * radius;
  const float phi =float(two_pi) * s.y;
  float sinPhi, cosPhi;
  sincosf(phi, &sinPhi, &cosPhi);
  return Vec3fa(r * cosPhi, r * sinPhi, 0.f);
}

inline float uniformSampleDiskPDF(const float radius)
{
  return rcp(float(pi) * sqr(radius));
}

inline float _uniformSampleDiskPDF(const float radius)
{
  return rcp(float(pi) * sqr(radius));
}


/// sampling of triangle abc
////////////////////////////////////////////////////////////////////////////////

inline Vec3fa uniformSampleTriangle(const Vec3fa &a, const Vec3fa &b, const Vec3fa &c, const Vec2f &s)
{
  const float su = sqrtf(s.x);
  return c + (1.0f - su) * (a-c) + (s.y*su) * (b-c);
}

inline float uniformSampleTrianglePDF(const Vec3fa &a, const Vec3fa &b, const Vec3fa &c)
{
  return 2.0f * rcp(abs(length(cross(a-c, b-c))));
}

} // namespace embree
