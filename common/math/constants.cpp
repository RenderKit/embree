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
#endif
}
