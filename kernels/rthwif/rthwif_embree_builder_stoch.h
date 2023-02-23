#pragma once

#include "rthwif_embree_builder.h"
#include "builder/gpu/AABB.h"
#include "../common/device.h"
#include "../builders/priminfo.h"

namespace embree {
    BBox3fa rthwifBuildStoch(DeviceGPU* deviceGPU, sycl::queue &gpu_queue, PrimInfo pinfo, const uint numPrimitives, gpu::AABB *aabb);
}