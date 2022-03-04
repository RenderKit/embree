// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "../../../include/embree3/rtcore.h"
#include "../math/vec.h"

namespace embree {

struct DifferentialGeometry
{
  unsigned int instIDs[RTC_MAX_INSTANCE_LEVEL_COUNT];
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
