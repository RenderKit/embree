// ======================================================================== //
// Copyright 2009-2013 Intel Corporation                                    //
//                                                                          //
// Licensed under the Apache License, Version 2.0 (the "License");          //
// you may not use this file except in compliance with the License.         //
// You may obtain a copy of the License at                                  //
//                                                                          //
//     http://www.apache.org/licenses/LICENSE-2.0                           //
//                                                                          //
// Unless required by applicable law or agreed to in writing, software      //
// distributed under the License is distributed on an "AS IS" BASIS,        //
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. //
// See the License for the specific language governing permissions and      //
// limitations under the License.                                           //
// ======================================================================== //

#ifndef __EMBREE_BVH4_INTERSECTOR16_CHUNK_H__
#define __EMBREE_BVH4_INTERSECTOR16_CHUNK_H__

#include "bvh4.h"
#include "../common/stack_item.h"
#include "../common/registry_intersector.h"
#include "../common/ray16.h"

namespace embree
{
  namespace isa
  {
    /*! BVH4 Traverser. Packet traversal implementation for a Quad BVH. */
    template<typename TriangleIntersector>
      class BVH4Intersector16Chunk
    {
      /* shortcuts for frequently used types */
      typedef typename TriangleIntersector::Primitive Triangle;
      typedef typename BVH4::NodeRef NodeRef;
      typedef typename BVH4::Node Node;
      
    public:
      static Accel::Intersector16 create() { 
        return Accel::Intersector16((RTCIntersectFunc16)intersect, (RTCOccludedFunc16)occluded);
      }
      
      static void intersect(mic_i* valid, BVH4* bvh, Ray16& ray);
      static void occluded (mic_i* valid, BVH4* bvh, Ray16& ray);
    };
  }
}

#endif
  
