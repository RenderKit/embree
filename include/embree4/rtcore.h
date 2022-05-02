// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "rtcore_config.h"

#if defined(EMBREE_DPCPP_SUPPORT) && defined(__cplusplus)
#  define __SYCL_USE_NON_VARIADIC_SPIRV_OCL_PRINTF__
#  include <CL/sycl.hpp>
#endif

#include "rtcore_common.h"
#include "rtcore_device.h"
#include "rtcore_buffer.h"
#include "rtcore_ray.h"
#include "rtcore_geometry.h"
#include "rtcore_scene.h"
#include "rtcore_builder.h"
#include "rtcore_quaternion.h"
