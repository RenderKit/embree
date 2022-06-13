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
const uint32x4_t v0x80000000 = { 0x80000000, 0x80000000, 0x80000000, 0x80000000 };
const uint32x4_t v0x7fffffff = { 0x7fffffff, 0x7fffffff, 0x7fffffff, 0x7fffffff };
const uint8x16_t v0022 = {0,1,2,3, 0,1,2,3, 8,9,10,11, 8,9,10,11};
const uint8x16_t v1133 = {4,5,6,7, 4,5,6,7, 12,13,14,15, 12,13,14,15};
const uint8x16_t v0101 = {0,1,2,3, 4,5,6,7, 0,1,2,3, 4,5,6,7};
const float32x4_t vOne = { 1.0f, 1.0f, 1.0f, 1.0f };
const float32x4_t vmOne = { -1.0f, -1.0f, -1.0f, -1.0f };
const float32x4_t vInf = { INFINITY, INFINITY, INFINITY, INFINITY };
const float32x4_t vmInf = { -INFINITY, -INFINITY, -INFINITY, -INFINITY };
#endif
}
