// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "../core/differential_geometry.h"

namespace embree {

struct Light;

struct Light_SampleRes
{
  Vec3fa weight;  //!< radiance that arrives at the given point divided by pdf
  Vec3fa dir;     //!< direction towards the light source
  float dist;    //!< largest valid t_far value for a shadow ray
  float pdf;     //!< probability density that this sample was taken
};

//! compute the weighted radiance at a point caused by a sample on the light source
// by convention, giving (0, 0) as "random" numbers should sample the "center"
// of the light source (used by the raytracing renderers such as the OBJ renderer)
typedef Light_SampleRes (*Light_SampleFunc)(const Light* self,
                                            const DifferentialGeometry& dg, /*! point to generate the sample for >*/
                                            const Vec2f& s);                /*! random numbers to generate the sample >*/


struct Light_EvalRes
{
  Vec3fa value;     //!< radiance that arrives at the given point (not weighted by pdf)
  float dist;
  float pdf;       //!< probability density that the direction would have been sampled
};

//! compute the radiance, distance and pdf caused by the light source (pointed to by the given direction)
typedef Light_EvalRes (*Light_EvalFunc)(const Light* self,
                                        const DifferentialGeometry& dg, /*! point to evaluate illumination for >*/
                                        const Vec3fa& dir);              /*! direction towards the light source >*/


struct Light
{
  Light_SampleFunc sample;
  Light_EvalFunc eval;
};

Light_EvalRes Light_eval(const Light* self, const DifferentialGeometry& dg, const Vec3fa& dir);

inline void Light_Constructor(Light* self)
{
  self->eval = Light_eval;
}

} // namespace embree
