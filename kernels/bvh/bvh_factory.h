// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "kernels/bvh/bvh.h"
#include "kernels/common/isa.h"
#include "kernels/common/accel.h"
#include "kernels/common/scene.h"
#include "kernels/geometry/curve_intersector_virtual.h"

namespace embree
{
  /*! BVH instantiations */
  class BVHFactory
  {
  public:
    enum class BuildVariant     { STATIC, DYNAMIC, HIGH_QUALITY };
    enum class IntersectVariant { FAST, ROBUST };
  };
}
