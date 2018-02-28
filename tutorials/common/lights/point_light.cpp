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

#include "light.h"
#include "../math/sampling.h"
#include "../math/linearspace.h"

namespace embree {

struct PointLight
{
  Light super;    //!< inherited light fields

  Vec3fa position; //!< light position
  Vec3fa power;    //!< RGB color and intensity of light
  float radius;   //!< defines the size of the SphereLight
};


// Implementation
//////////////////////////////////////////////////////////////////////////////

Light_SampleRes PointLight_sample(const Light* super,
                                  const DifferentialGeometry& dg,
                                  const Vec2f& s)
{
  const PointLight* self = (PointLight*)super;
  Light_SampleRes res;

  // extant light vector from the hit point
  const Vec3fa dir = self->position - dg.P;
  const float dist2 = dot(dir, dir);
  const float invdist = rsqrt(dist2);

  // normalized light vector
  res.dir = dir * invdist;
  res.dist = dist2 * invdist;

  res.pdf = inf; // per default we always take this res

  // convert from power to radiance by attenuating by distance^2
  res.weight = self->power * sqr(invdist);
  const float sinTheta = self->radius * invdist;

  if ((self->radius > 0.f) & (sinTheta > 0.005f)) {
    // res surface of sphere as seen by hit point -> cone of directions
    // for very small cones treat as point light, because float precision is not good enough
    if (sinTheta < 1.f) {
      const float cosTheta = sqrt(1.f - sinTheta * sinTheta);
      const Vec3fa localDir = uniformSampleCone(cosTheta, s);
      res.dir = frame(res.dir) * localDir;
      res.pdf = uniformSampleConePDF(cosTheta);
      const float c = localDir.z;
      res.dist = c*res.dist - sqrt(sqr(self->radius) - (1.f - c*c) * dist2);
      // TODO scale radiance by actual distance
    } else { // inside sphere
      const Vec3fa localDir = cosineSampleHemisphere(s);
      res.dir = frame(dg.Ns) * localDir;
      res.pdf = cosineSampleHemispherePDF(localDir);
      // TODO:
      res.weight = self->power * rcp(sqr(self->radius));
      res.dist = self->radius;
    }
  }

  return res;
}

Light_EvalRes PointLight_eval(const Light* super,
                              const DifferentialGeometry& dg,
                              const Vec3fa& dir)
{
  const PointLight* self = (PointLight*)super;
  Light_EvalRes res;
  res.value = Vec3fa(0.f);
  res.dist = inf;
  res.pdf = 0.f;

  if (self->radius > 0.f) {
    const Vec3fa A = self->position - dg.P;
    const float a = dot(dir, dir);
    const float b = 2.f * dot(dir, A);
    const float centerDist2 = dot(A, A);
    const float c = centerDist2 - sqr(self->radius);
    const float radical = sqr(b) - 4.f*a*c;

    if (radical > 0.f) {
      const float t_near = (b - sqrt(radical)) / (2.f*a);
      const float t_far = (b + sqrt(radical)) / (2.f*a);

      if (t_far > 0.0f) {
        // TODO: handle interior case
        res.dist = t_near;
        const float sinTheta2 = sqr(self->radius) * rcp(centerDist2);
        const float cosTheta = sqrt(1.f - sinTheta2);
        res.pdf = uniformSampleConePDF(cosTheta);
        const float invdist = rcp(t_near);
        res.value = self->power * res.pdf * sqr(invdist);
      }
    }
  }

  return res;
}

// Exports (called from C++)
//////////////////////////////////////////////////////////////////////////////

//! Set the parameters of an ispc-side PointLight object
extern "C" void PointLight_set(void* super,
                           const Vec3fa& position,
                           const Vec3fa& power,
                           float radius)
{
  PointLight* self = (PointLight*)super;
  self->position = position;
  self->power = power;
  self->radius = radius;
}

//! Create an ispc-side PointLight object
extern "C" void* PointLight_create()
{
  PointLight* self = (PointLight*) alignedMalloc(sizeof(PointLight),16);
  Light_Constructor(&self->super);
  self->super.sample = PointLight_sample;
  self->super.eval = PointLight_eval;

  PointLight_set(self, Vec3fa(0.f), Vec3fa(1.f), 0.f);
  return self;
}

} // namespace embree
