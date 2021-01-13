// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "../math/vec.h"

namespace embree {

struct DifferentialGeometry
{
  unsigned int instID;
  unsigned int geomID;
  unsigned int primID;
  float u,v;
  Vec3fa P;
  Vec3fa Ng;
  Vec3fa Ns;
  Vec3fa Tx; //direction along hair
  Vec3fa Ty;
  float eps;
};

} // namespace embree
