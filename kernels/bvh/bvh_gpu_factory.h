// ======================================================================== //
// Copyright 2009-2018 Intel Corporation                                    //
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

#include "bvh_factory.h"

namespace embree
{
#if defined(EMBREE_DPCPP_SUPPORT)	    

  /*! BVHGPU instantiations */
  class BVHGPUFactory : public BVHFactory
  {
  public:
    BVHGPUFactory();

  public:
    Accel* BVHGPUTriangle1v   (Scene* scene, BuildVariant bvariant);
    Accel::Intersectors BVHGPUTriangle1vIntersectors(BVH4* bvh);

    Accel* BVHGPUQuad1v   (Scene* scene, BuildVariant bvariant);
    Accel::Intersectors BVHGPUQuad1vIntersectors(BVH4* bvh);

  private:
    void selectBuilders(int features);
    void selectIntersectors(int features);

  private:
    DEFINE_ISA_FUNCTION(Builder*,BVHGPUQuad1vSceneBuilderSAH,void* COMMA Scene* COMMA size_t);
    DEFINE_ISA_FUNCTION(Builder*,BVHGPUTriangle1vSceneBuilderSAH,void* COMMA Scene* COMMA size_t);
    DEFINE_SYMBOL2(Accel::Intersector1,BVHGPUTriangle1vIntersector1);
    DEFINE_SYMBOL2(Accel::Intersector4,BVHGPUTriangle1vIntersector4);    
    DEFINE_SYMBOL2(Accel::IntersectorN,BVHGPUTriangle1vIntersectorStream);
    DEFINE_SYMBOL2(Accel::Intersector1,BVHGPUQuad1vIntersector1);
    DEFINE_SYMBOL2(Accel::Intersector4,BVHGPUQuad1vIntersector4);    
    DEFINE_SYMBOL2(Accel::IntersectorN,BVHGPUQuad1vIntersectorStream);

  };
#endif
  
}
