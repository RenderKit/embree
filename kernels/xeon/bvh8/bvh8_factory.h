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
  /*! BVH8 instantiations */
  struct BVH8Factory
  {
    static Accel* BVH8Triangle4(Scene* scene);
    static Accel* BVH8Triangle4ObjectSplit(Scene* scene);
    static Accel* BVH8Triangle4SpatialSplit(Scene* scene);

    static Accel* BVH8Triangle8(Scene* scene);
    //static Accel* BVH8Triangle8v(Scene* scene);
    static Accel* BVH8TrianglePairs4ObjectSplit(Scene* scene);
    static Accel* BVH8Triangle8ObjectSplit(Scene* scene);
    static Accel* BVH8Triangle8SpatialSplit(Scene* scene);
    //static Accel* BVH8Triangle8vObjectSplit(Scene* scene);
    //static Accel* BVH8Triangle8vSpatialSplit(Scene* scene);
  };
}
