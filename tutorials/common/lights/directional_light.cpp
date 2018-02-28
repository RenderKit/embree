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

struct DirectionalLight
{
  Light super;      //!< inherited light fields

  LinearSpace3fa frame;   //!< coordinate frame, with vz == direction *towards* the light source
  Vec3fa radiance;   //!< RGB color and intensity of light
  float cosAngle;   //!< Angular limit of the cone light in an easier to use form: cosine of the half angle in radians
  float pdf;        //!< Probability to sample a direction to the light
};

// for very small cones treat as singular light, because float precision is not good enough
#define COS_ANGLE_MAX 0.99999988f


// Implementation
//////////////////////////////////////////////////////////////////////////////

Light_SampleRes DirectionalLight_sample(const Light* super,
                                        const DifferentialGeometry& dg,
                                        const Vec2f& s)
{
  const DirectionalLight* self = (DirectionalLight*)super;
  Light_SampleRes res;

  res.dir = self->frame.vz;
  res.dist = inf;
  res.pdf = self->pdf;

  if (self->cosAngle < COS_ANGLE_MAX)
    res.dir = self->frame * uniformSampleCone(self->cosAngle, s);

  res.weight = self->radiance; // *pdf/pdf cancel

  return res;
}

Light_EvalRes DirectionalLight_eval(const Light* super,
                                    const DifferentialGeometry&,
                                    const Vec3fa& dir)
{
  DirectionalLight* self = (DirectionalLight*)super;
  Light_EvalRes res;
  res.dist = inf;

  if (self->cosAngle < COS_ANGLE_MAX && dot(self->frame.vz, dir) > self->cosAngle) {
    res.value = self->radiance * self->pdf;
    res.pdf = self->pdf;
  } else {
    res.value = Vec3fa(0.f);
    res.pdf = 0.f;
  }

  return res;
}


// Exports (called from C++)
//////////////////////////////////////////////////////////////////////////////

//! Set the parameters of an ispc-side DirectionalLight object
extern "C" void DirectionalLight_set(void* super,
                                 const Vec3fa& direction,
                                 const Vec3fa& radiance,
                                 float cosAngle)
{
  DirectionalLight* self = (DirectionalLight*)super;
  self->frame = frame(direction);
  self->radiance = radiance;
  self->cosAngle = cosAngle;
  self->pdf = cosAngle < COS_ANGLE_MAX ? uniformSampleConePDF(cosAngle) : inf;
}

//! Create an ispc-side DirectionalLight object
extern "C" void* DirectionalLight_create()
{
  DirectionalLight* self = (DirectionalLight*) alignedMalloc(sizeof(DirectionalLight),16);
  Light_Constructor(&self->super);
  self->super.sample = DirectionalLight_sample;
  self->super.eval = DirectionalLight_eval;

  DirectionalLight_set(self, Vec3fa(0.f, 0.f, 1.f), Vec3fa(1.f), 1.f);
  return self;
}

} // namespace embree
