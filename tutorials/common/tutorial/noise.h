// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "../math/vec.h"

namespace embree
{
  /* noise functions */
  float noise(const Vec3fa& p);
  Vec3fa noise3D(const Vec3fa& p);
}
