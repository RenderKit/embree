#pragma once

#include "rthwif_embree_builder.h"
#include "builder/gpu/AABB.h"
#include "../common/device.h"
#include "../builders/priminfo.h"

namespace embree {
    namespace gpu {
        struct __aligned(8) BVH2BuildRecord
        {
        gpu::AABB3f bounds;        // 24 bytes
        unsigned int start, end;   // 8 bytes
        unsigned int left;         // 4 bytes
        unsigned int right;
        unsigned int parent;       // 4 bytes

        __forceinline BVH2BuildRecord() {}
        
        __forceinline void init(const uint _start, const uint _end, const uint _parent, const gpu::AABB3f &_bounds)
        {
            bounds = _bounds;
            start = _start;
            end   = _end;
            parent = _parent;
            left = right = -1;
        }      

        __forceinline unsigned int size() const { return end - start; }

        __forceinline operator const gpu::AABB3f &() const { return bounds; }

        __forceinline bool isLeaf() const { return left == -1; }

        friend __forceinline embree_ostream operator<<(embree_ostream cout, const BVH2BuildRecord &br)
        {
            cout << "start " << br.start << " end " << br.end << " left " << br.left << " right " << br.right << " parent " << br.parent << " bounds " << br.bounds << " ";
            return cout;
        }
        
        };
    }

    struct StochReturn {
        gpu::BVH2BuildRecord *bvh2;
        uint *indices;
        uint rootIndex;
    };


    StochReturn rthwifBuildStoch(DeviceGPU* deviceGPU, sycl::queue &gpu_queue, const uint numPrimitives, gpu::AABB *aabb);
}