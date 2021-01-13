// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "light.isph"

Light_EvalRes Light_eval(const uniform Light* uniform,
                         const DifferentialGeometry&,
                         const Vec3f&)
{
  Light_EvalRes res;
  res.value = make_Vec3f(0.f);
  res.dist = inf;
  res.pdf = 0.f;
  return res;
}

export void Light_destroy(Light* uniform light)
{
  delete light;
}

export void dummy() {} // just to avoid linker warning under MacOSX
