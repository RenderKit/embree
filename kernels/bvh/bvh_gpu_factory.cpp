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

#include "../common/isa.h" 
#include "bvh_gpu_factory.h"
#include "../bvh/bvh.h"
#include "../geometry/triangle1v.h"
#include "../geometry/quad1v.h"
#include "../geometry/triangle1vmb.h"
#include "../geometry/quad1vmb.h"
#include "../common/accelinstance.h"

namespace embree
{
#if defined(EMBREE_DPCPP_SUPPORT)	    
  
  DECLARE_ISA_FUNCTION(Builder*,BVHGPUTriangle1vSceneBuilderSAH,void* COMMA Scene* COMMA size_t);
  DECLARE_ISA_FUNCTION(Builder*,BVHGPUQuad1vSceneBuilderSAH,void* COMMA Scene* COMMA size_t);

  DECLARE_ISA_FUNCTION(Builder*,BVHGPUTriangle1vMBSceneBuilderSAH,void* COMMA Scene*);
  DECLARE_ISA_FUNCTION(Builder*,BVHGPUQuad1vMBSceneBuilderSAH,void* COMMA Scene*);
  
  DECLARE_SYMBOL2(Accel::Intersector1,BVHGPUTriangle1vIntersector1);
  DECLARE_SYMBOL2(Accel::Intersector4,BVHGPUTriangle1vIntersector4);
  DECLARE_SYMBOL2(Accel::IntersectorN,BVHGPUTriangle1vIntersectorStream);

  DECLARE_SYMBOL2(Accel::Intersector1,BVHGPUTriangle1vMBIntersector1);
  DECLARE_SYMBOL2(Accel::Intersector4,BVHGPUTriangle1vMBIntersector4);
  DECLARE_SYMBOL2(Accel::IntersectorN,BVHGPUTriangle1vMBIntersectorStream);
  
  DECLARE_SYMBOL2(Accel::Intersector1,BVHGPUQuad1vIntersector1);
  DECLARE_SYMBOL2(Accel::Intersector4,BVHGPUQuad1vIntersector4);
  DECLARE_SYMBOL2(Accel::IntersectorN,BVHGPUQuad1vIntersectorStream);

  DECLARE_SYMBOL2(Accel::Intersector1,BVHGPUQuad1vMBIntersector1);
  DECLARE_SYMBOL2(Accel::Intersector4,BVHGPUQuad1vMBIntersector4);
  DECLARE_SYMBOL2(Accel::IntersectorN,BVHGPUQuad1vMBIntersectorStream);
  
  BVHGPUFactory::BVHGPUFactory()
  {
    int features = 0;
    selectBuilders(features);
    selectIntersectors(features);
  }

  void BVHGPUFactory::selectBuilders(int features)
  {
    IF_ENABLED_TRIS(SELECT_SYMBOL_DEFAULT(features,BVHGPUTriangle1vSceneBuilderSAH));
    IF_ENABLED_QUADS(SELECT_SYMBOL_DEFAULT(features,BVHGPUQuad1vSceneBuilderSAH));

    IF_ENABLED_TRIS(SELECT_SYMBOL_DEFAULT(features,BVHGPUTriangle1vMBSceneBuilderSAH));
    IF_ENABLED_QUADS(SELECT_SYMBOL_DEFAULT(features,BVHGPUQuad1vMBSceneBuilderSAH));        
  }

  void BVHGPUFactory::selectIntersectors(int features)
  {
    SELECT_SYMBOL_DEFAULT_SSE42_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVHGPUTriangle1vIntersector1);
    SELECT_SYMBOL_DEFAULT_SSE42_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVHGPUTriangle1vIntersector4);
    SELECT_SYMBOL_DEFAULT_SSE42_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVHGPUTriangle1vIntersectorStream);

    SELECT_SYMBOL_DEFAULT_SSE42_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVHGPUTriangle1vMBIntersector1);
    SELECT_SYMBOL_DEFAULT_SSE42_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVHGPUTriangle1vMBIntersector4);
    SELECT_SYMBOL_DEFAULT_SSE42_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVHGPUTriangle1vMBIntersectorStream);
    
    SELECT_SYMBOL_DEFAULT_SSE42_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVHGPUQuad1vIntersector1);
    SELECT_SYMBOL_DEFAULT_SSE42_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVHGPUQuad1vIntersector4);
    SELECT_SYMBOL_DEFAULT_SSE42_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVHGPUQuad1vIntersectorStream);

    SELECT_SYMBOL_DEFAULT_SSE42_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVHGPUQuad1vMBIntersector1);
    SELECT_SYMBOL_DEFAULT_SSE42_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVHGPUQuad1vMBIntersector4);
    SELECT_SYMBOL_DEFAULT_SSE42_AVX_AVX2_AVX512KNL_AVX512SKX(features,BVHGPUQuad1vMBIntersectorStream);        
  }

  Accel::Intersectors BVHGPUFactory::BVHGPUTriangle1vIntersectors(BVH4* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1    = BVHGPUTriangle1vIntersector1();
#if defined (EMBREE_RAY_PACKETS)
    intersectors.intersector4    = BVHGPUTriangle1vIntersector4();
    intersectors.intersector8    = NULL;
    intersectors.intersector16   = NULL;
    intersectors.intersectorN    = BVHGPUTriangle1vIntersectorStream();
#endif
    return intersectors;  
  }

  Accel::Intersectors BVHGPUFactory::BVHGPUTriangle1vMBIntersectors(BVH4* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1    = BVHGPUTriangle1vMBIntersector1();
#if defined (EMBREE_RAY_PACKETS)
    intersectors.intersector4    = BVHGPUTriangle1vMBIntersector4();
    intersectors.intersector8    = NULL;
    intersectors.intersector16   = NULL;
    intersectors.intersectorN    = BVHGPUTriangle1vMBIntersectorStream();
#endif
    return intersectors;  
  }
  
  Accel::Intersectors BVHGPUFactory::BVHGPUQuad1vIntersectors(BVH4* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1    = BVHGPUQuad1vIntersector1();
#if defined (EMBREE_RAY_PACKETS)
    intersectors.intersector4    = BVHGPUQuad1vIntersector4();
    intersectors.intersector8    = NULL;
    intersectors.intersector16   = NULL;
    intersectors.intersectorN    = BVHGPUQuad1vIntersectorStream();
#endif
    return intersectors;  
  }

  Accel::Intersectors BVHGPUFactory::BVHGPUQuad1vMBIntersectors(BVH4* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1    = BVHGPUQuad1vMBIntersector1();
#if defined (EMBREE_RAY_PACKETS)
    intersectors.intersector4    = BVHGPUQuad1vMBIntersector4();
    intersectors.intersector8    = NULL;
    intersectors.intersector16   = NULL;
    intersectors.intersectorN    = BVHGPUQuad1vMBIntersectorStream();
#endif
    return intersectors;  
  }  
  

  Accel* BVHGPUFactory::BVHGPUTriangle1v(Scene* scene, BuildVariant bvariant)
  {
    BVH4* accel = new BVH4(Triangle1v::type,scene);
    Accel::Intersectors intersectors = BVHGPUTriangle1vIntersectors(accel);
    Builder* builder = BVHGPUTriangle1vSceneBuilderSAH(accel,scene,(size_t)bvariant);
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVHGPUFactory::BVHGPUTriangle1vMB(Scene* scene)
  {
    BVH4* accel = new BVH4(Triangle1vMB::type,scene);
    Accel::Intersectors intersectors = BVHGPUTriangle1vMBIntersectors(accel);
    Builder* builder = BVHGPUTriangle1vMBSceneBuilderSAH(accel,scene);
    return new AccelInstance(accel,builder,intersectors);
  }
  
  Accel* BVHGPUFactory::BVHGPUQuad1v(Scene* scene, BuildVariant bvariant)
  {
    BVH4* accel = new BVH4(Quad1v::type,scene);
    Accel::Intersectors intersectors = BVHGPUQuad1vIntersectors(accel); 
    Builder* builder = BVHGPUQuad1vSceneBuilderSAH(accel,scene,(size_t)bvariant);
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVHGPUFactory::BVHGPUQuad1vMB(Scene* scene)
  {
    BVH4* accel = new BVH4(Quad1vMB::type,scene);
    Accel::Intersectors intersectors = BVHGPUQuad1vMBIntersectors(accel); 
    Builder* builder = BVHGPUQuad1vMBSceneBuilderSAH(accel,scene);
    return new AccelInstance(accel,builder,intersectors);
  }  
  
#endif
}

