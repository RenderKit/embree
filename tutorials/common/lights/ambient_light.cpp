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

struct AmbientLight
{
  Light super;      //!< inherited light fields

  Vec3fa radiance;   //!< RGB color and intensity of light
};


// Implementation
//////////////////////////////////////////////////////////////////////////////

// XXX importance sampling is only done into the positive hemisphere
// ==> poor support for translucent materials
Light_SampleRes AmbientLight_sample(const Light* super,
                                    const DifferentialGeometry& dg,
                                    const Vec2f& s)
{
  AmbientLight* self = (AmbientLight*)super;
  Light_SampleRes res;

  const Vec3fa localDir = cosineSampleHemisphere(s);
  res.dir = frame(dg.Ns) * localDir;
  res.pdf = cosineSampleHemispherePDF(localDir);
  res.dist = inf;
  res.weight = self->radiance * rcp(res.pdf);

  return res;
}

Light_EvalRes AmbientLight_eval(const Light* super,
                                const DifferentialGeometry& dg,
                                const Vec3fa& dir)
{
  AmbientLight* self = (AmbientLight*)super;
  Light_EvalRes res;

  res.value = self->radiance;
  res.dist = inf;
  res.pdf = cosineSampleHemispherePDF(max(dot(dg.Ns, dir), 0.f));

  return res;
}


void AmbientLight_Constructor(AmbientLight* self,
                              const Vec3fa& radiance)
{
  Light_Constructor(&self->super);
  self->radiance = radiance;
  self->super.sample = AmbientLight_sample;
  self->super.eval = AmbientLight_eval;
}


// Exports (called from C++)
//////////////////////////////////////////////////////////////////////////////

//! Create an ispc-side AmbientLight object
extern "C" void *AmbientLight_create()
{
  AmbientLight* self = (AmbientLight*) alignedMalloc(sizeof(AmbientLight),16);
  AmbientLight_Constructor(self, Vec3fa(1.f));
  return self;
}

//! Set the parameters of an ispc-side AmbientLight object
extern "C" void AmbientLight_set(void* super,
                             const Vec3fa& radiance)
{
  AmbientLight* self = (AmbientLight*)super;
  self->radiance = radiance;
}

} // namespace embree
