// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "../math/vec.h"

namespace embree 
{
  extern "C" void* PointLight_create();
  
  extern "C" void PointLight_set(void* super,
                                 const Vec3fa& position,
                                 const Vec3fa& power,
                                 float radius);
}
