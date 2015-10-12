// ======================================================================== //
// Copyright 2009-2015 Intel Corporation                                    //
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

#pragma once

#include "../../common/accel.h"
#include "../../common/scene.h"

namespace embree
{
  /*! BVH4i instantiations */
  struct BVH4iFactory
  {
    static Accel* BVH4iTriangle1ObjectSplitBinnedSAH(Scene* scene,bool robust);
    static Accel* BVH4iTriangle1ObjectSplitMorton(Scene* scene,bool robust);
    static Accel* BVH4iTriangle1ObjectSplitEnhancedMorton(Scene* scene,bool robust);
    static Accel* BVH4iTriangle1PreSplitsBinnedSAH(Scene* scene,bool robust);
    static Accel* BVH4iVirtualGeometryBinnedSAH(Scene* scene,bool robust);
    static Accel* BVH4iTriangle1MemoryConservativeBinnedSAH(Scene* scene,bool robust);
    static Accel* BVH4iTriangle1ObjectSplitMorton64Bit(Scene* scene,bool robust);
    static Accel* BVH4iSubdivMeshBinnedSAH(Scene* scene,bool robust);
  };
}
