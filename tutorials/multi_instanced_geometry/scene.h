// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "../common/tutorial/tutorial_device.h"
#include "../common/math/linearspace.h"
#include "../../include/embree4/rtcore.h"

namespace embree {

extern "C" struct InstanceLevels
{
  unsigned int numLevels;
  const unsigned int* numInstancesOnLevel;
  LinearSpace3fa* * normalTransforms;
};

extern "C" RTCScene initializeScene(RTCDevice device,
                                            struct InstanceLevels* levels);

extern "C" void cleanupScene();

} // namespace embree
