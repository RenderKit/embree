// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "../math/vec.h"

namespace embree 
{
  extern "C" void* SpotLight_create();
  
  extern "C" void SpotLight_set(void* super,
                                const Vec3fa& position,
                                const Vec3fa& direction,
                                const Vec3fa& power,
                                float cosAngleMax,
                                float cosAngleScale,
                                float radius);
}
