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

#include "light.isph"
#include "../math/sampling.isph"
#include "../math/linearspace.isph"

struct AmbientLight
{
  Light super;      //!< inherited light fields

  Vec3f radiance;   //!< RGB color and intensity of light
};


// Implementation
//////////////////////////////////////////////////////////////////////////////

// XXX importance sampling is only done into the positive hemisphere
// ==> poor support for translucent materials
Light_SampleRes AmbientLight_sample(const uniform Light* uniform super,
                                    const DifferentialGeometry& dg,
                                    const Vec2f& s)
{
  uniform AmbientLight* uniform self = (uniform AmbientLight* uniform)super;
  Light_SampleRes res;

  const Vec3f localDir = cosineSampleHemisphere(s);
  res.dir = frame(dg.Ns) * localDir;
  res.pdf = cosineSampleHemispherePDF(localDir);
  res.dist = inf;
  res.weight = self->radiance * rcp(res.pdf);

  return res;
}

Light_EvalRes AmbientLight_eval(const uniform Light* uniform super,
                                const DifferentialGeometry& dg,
                                const Vec3f& dir)
{
  uniform AmbientLight* uniform self = (uniform AmbientLight* uniform)super;
  Light_EvalRes res;

  res.value = self->radiance;
  res.dist = inf;
  res.pdf = cosineSampleHemispherePDF(max(dot(dg.Ns, dir), 0.f));

  return res;
}


void AmbientLight_Constructor(uniform AmbientLight* uniform self,
                              const uniform Vec3f& radiance)
{
  Light_Constructor(&self->super);
  self->radiance = radiance;
  self->super.sample = AmbientLight_sample;
  self->super.eval = AmbientLight_eval;
}


// Exports (called from C++)
//////////////////////////////////////////////////////////////////////////////

//! Create an ispc-side AmbientLight object
export void *uniform AmbientLight_create()
{
  uniform AmbientLight* uniform self = uniform new uniform AmbientLight;
  AmbientLight_Constructor(self, make_Vec3f(1.f));
  return self;
}

//! Set the parameters of an ispc-side AmbientLight object
export void AmbientLight_set(void* uniform super,
                             const uniform Vec3f& radiance)
{
  uniform AmbientLight* uniform self = (uniform AmbientLight* uniform)super;
  self->radiance = radiance;
}
