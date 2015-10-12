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
  /*! BVH4 instantiations */
  struct BVH4Factory
  {
    static Accel* BVH4Triangle4vMB(Scene* scene);

    static Accel* BVH4Bezier1v(Scene* scene);
    static Accel* BVH4Bezier1i(Scene* scene);

    static Accel* BVH4OBBBezier1v(Scene* scene, bool highQuality);
    static Accel* BVH4OBBBezier1i(Scene* scene, bool highQuality);
    static Accel* BVH4OBBBezier1iMB(Scene* scene, bool highQuality);

    static Accel* BVH4Triangle4(Scene* scene);
    static Accel* BVH4Triangle8(Scene* scene);
    static Accel* BVH4Triangle4v(Scene* scene);
    static Accel* BVH4Triangle4i(Scene* scene);
    static Accel* BVH4TrianglePairs4ObjectSplit(Scene* scene);
    static Accel* BVH4SubdivPatch1(Scene* scene);
    static Accel* BVH4SubdivPatch1Cached(Scene* scene);
    static Accel* BVH4SubdivGridEager(Scene* scene);
    static Accel* BVH4UserGeometry(Scene* scene);
    static Accel* BVH4InstancedBVH4Triangle4ObjectSplit(Scene* scene);

    static Accel* BVH4BVH4Triangle4ObjectSplit(Scene* scene);
    static Accel* BVH4BVH4Triangle8ObjectSplit(Scene* scene);
    static Accel* BVH4BVH4Triangle4vObjectSplit(Scene* scene);
    static Accel* BVH4BVH4Triangle4iObjectSplit(Scene* scene);
    static Accel* BVH4BVH4TrianglePairs4ObjectSplit(Scene* scene);

    static Accel* BVH4Triangle4SpatialSplit(Scene* scene);
    static Accel* BVH4Triangle8SpatialSplit(Scene* scene);
    static Accel* BVH4Triangle4ObjectSplit(Scene* scene);
    static Accel* BVH4Triangle8ObjectSplit(Scene* scene);
    static Accel* BVH4Triangle4vObjectSplit(Scene* scene);
    static Accel* BVH4Triangle4iObjectSplit(Scene* scene);

    static Accel* BVH4Triangle4ObjectSplit(TriangleMesh* mesh);
    static Accel* BVH4Triangle4vObjectSplit(TriangleMesh* mesh);
    static Accel* BVH4Triangle4Refit(TriangleMesh* mesh);
    static Accel* BVH4TrianglePairs4ObjectSplit(TriangleMesh* mesh);
  };
}
