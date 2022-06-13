// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#if defined(__aarch64__)
#include <arm_neon.h>
#endif

#include "constants.h"

namespace embree
{
#if defined(__aarch64__)
const uint32x4_t movemask_mask = { 1, 2, 4, 8 };
const float32x4_t vOne = { 1.0f, 1.0f, 1.0f, 1.0f };
const float32x4_t vmOne = { -1.0f, -1.0f, -1.0f, -1.0f };
const float32x4_t vInf = { INFINITY, INFINITY, INFINITY, INFINITY };
const float32x4_t vmInf = { -INFINITY, -INFINITY, -INFINITY, -INFINITY };
#endif
}
