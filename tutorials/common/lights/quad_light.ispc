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

struct QuadLight
{
  Light super;            //!< inherited light fields

  Vec3f position;         //!< world-space corner position of the light
  Vec3f edge1;            //!< vectors to adjacent corners
  Vec3f edge2;            //!< vectors to adjacent corners
  Vec3f radiance;         //!< RGB color and intensity of the QuadLight

  Vec3f nnormal;          //!< negated normal, the direction that the QuadLight is not emitting; normalized
  float ppdf;             // probability to sample point on light = 1/area
};


// Implementation
//////////////////////////////////////////////////////////////////////////////

Light_SampleRes QuadLight_sample(const uniform Light* uniform super,
                                 const DifferentialGeometry& dg,
                                 const Vec2f& s)
{
  const QuadLight* uniform self = (QuadLight* uniform)super;
  Light_SampleRes res;

  // res position on light with density ppdf = 1/area
  // TODO: use solid angle sampling
  const Vec3f pos = self->position + self->edge1 * s.x + self->edge2 * s.y;

  // extant light vector from the hit point
  const Vec3f dir = pos - dg.P;
  const float dist = length(dir);

  // normalized light vector
  res.dir = dir / dist;
  res.dist = dist;

  // convert to pdf wrt. solid angle
  const float cosd = dot(self->nnormal, res.dir);
  res.pdf = self->ppdf * (dist * dist) / abs(cosd);

  // emit only to one side
  res.weight = cosd > 0.f ? self->radiance * rcp(res.pdf) : make_Vec3f(0.f);

  return res;
}


// Exports (called from C++)
//////////////////////////////////////////////////////////////////////////////

//! Set the parameters of an ispc-side QuadLight object
export void QuadLight_set(void* uniform super,
                          const uniform Vec3f& position,
                          const uniform Vec3f& edge2,
                          const uniform Vec3f& edge1,
                          const uniform Vec3f& radiance)
{
  uniform QuadLight* uniform self = (uniform QuadLight* uniform)super;
  self->position = position;
  self->edge1    = edge1;
  self->edge2    = edge2;
  self->radiance = radiance;

  const uniform Vec3f ndirection = cross(edge2, edge1);
  self->ppdf = rcp(length(ndirection));
  self->nnormal = ndirection * self->ppdf;
}

//! Create an ispc-side QuadLight object
export void* uniform QuadLight_create()
{
  uniform QuadLight* uniform self = uniform new uniform QuadLight;

  Light_Constructor(&self->super);
  self->super.sample = QuadLight_sample;

  QuadLight_set(self,
                make_Vec3f(0.f),
                make_Vec3f(1.f, 0.f, 0.f),
                make_Vec3f(0.f, 1.f, 0.f),
                make_Vec3f(1.f));

  return self;
}
