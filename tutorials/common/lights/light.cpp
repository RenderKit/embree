// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "light.h"

namespace embree {

Light_EvalRes Light_eval(const Light* uniform,
                         const DifferentialGeometry&,
                         const Vec3fa&)
{
  Light_EvalRes res;
  res.value = Vec3fa(0.f);
  res.dist = inf;
  res.pdf = 0.f;
  return res;
}

extern "C" void Light_destroy(Light* light)
{
  alignedFree(light);
}

extern "C" void dummy() {} // just to avoid linker warning under MacOSX

} // namespace embree
