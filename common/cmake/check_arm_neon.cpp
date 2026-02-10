// Copyright 2009-2020 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#if !defined(__ARM_NEON) && !defined(_M_ARM64)
#error "No ARM Neon support"
#endif

#include <arm_neon.h>

int main()
{
  return vaddvq_s32(vdupq_n_s32(1));
}
