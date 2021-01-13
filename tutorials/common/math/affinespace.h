// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "../../../common/math/affinespace.h"

namespace embree 
{
  __forceinline bool   eq (const AffineSpace3fa& a, const AffineSpace3fa& b) { return a == b; }
}

