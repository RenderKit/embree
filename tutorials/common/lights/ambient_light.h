// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "../math/vec.h"

namespace embree
{
  extern "C" void* AmbientLight_create();

  extern "C" void AmbientLight_set(void* super,
                                   const Vec3fa& radiance);

  extern "C" void Light_destroy(Light* light);
}
