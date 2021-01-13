// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "../math/vec.h"

namespace embree 
{
  extern "C" void* QuadLight_create();
  
  extern "C" void QuadLight_set(void* super,
                                const Vec3fa& position,
                                const Vec3fa& edge2,
                                const Vec3fa& edge1,
                                const Vec3fa& radiance);
}

