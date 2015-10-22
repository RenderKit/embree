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

#if defined (__TARGET_AVX__)

#include "bvh8_factory.h"
#include "../bvh/bvh.h"

#include "../geometry/triangle.h"
#include "../geometry/trianglepairsv.h"
#include "../../common/accelinstance.h"

namespace embree
{
  DECLARE_SYMBOL2(Accel::Intersector1,BVH8Triangle4Intersector1Moeller);
  DECLARE_SYMBOL2(Accel::Intersector4,BVH8Triangle4Intersector4HybridMoeller);
  DECLARE_SYMBOL2(Accel::Intersector4,BVH8Triangle4Intersector4HybridMoellerNoFilter);
  DECLARE_SYMBOL2(Accel::Intersector8,BVH8Triangle4Intersector8ChunkMoeller);
  DECLARE_SYMBOL2(Accel::Intersector8,BVH8Triangle4Intersector8HybridMoeller);
  DECLARE_SYMBOL2(Accel::Intersector8,BVH8Triangle4Intersector8HybridMoellerNoFilter);

  DECLARE_SYMBOL2(Accel::Intersector1,BVH8Triangle8Intersector1Moeller);
  DECLARE_SYMBOL2(Accel::Intersector4,BVH8Triangle8Intersector4HybridMoeller);
  DECLARE_SYMBOL2(Accel::Intersector4,BVH8Triangle8Intersector4HybridMoellerNoFilter);
  DECLARE_SYMBOL2(Accel::Intersector8,BVH8Triangle8Intersector8ChunkMoeller);
  DECLARE_SYMBOL2(Accel::Intersector8,BVH8Triangle8Intersector8HybridMoeller);
  DECLARE_SYMBOL2(Accel::Intersector8,BVH8Triangle8Intersector8HybridMoellerNoFilter);

  DECLARE_SYMBOL2(Accel::Intersector1,BVH8TrianglePairs4Intersector1Moeller);
  DECLARE_SYMBOL2(Accel::Intersector4,BVH8TrianglePairs4Intersector4HybridMoeller);
  DECLARE_SYMBOL2(Accel::Intersector4,BVH8TrianglePairs4Intersector4HybridMoellerNoFilter);
  DECLARE_SYMBOL2(Accel::Intersector8,BVH8TrianglePairs4Intersector8HybridMoeller);
  DECLARE_SYMBOL2(Accel::Intersector8,BVH8TrianglePairs4Intersector8HybridMoellerNoFilter);

  DECLARE_SYMBOL2(Accel::Intersector16,BVH8Triangle4Intersector16ChunkMoeller);
  DECLARE_SYMBOL2(Accel::Intersector16,BVH8Triangle4Intersector16HybridMoeller);
  DECLARE_SYMBOL2(Accel::Intersector16,BVH8Triangle4Intersector16HybridMoellerNoFilter);
  DECLARE_SYMBOL2(Accel::Intersector16,BVH8Triangle8Intersector16ChunkMoeller);
  DECLARE_SYMBOL2(Accel::Intersector16,BVH8Triangle8Intersector16HybridMoeller);
  DECLARE_SYMBOL2(Accel::Intersector16,BVH8Triangle8Intersector16HybridMoellerNoFilter);

  DECLARE_SYMBOL2(Accel::Intersector16,BVH8TrianglePairs4Intersector16HybridMoeller);
  DECLARE_SYMBOL2(Accel::Intersector16,BVH8TrianglePairs4Intersector16HybridMoellerNoFilter);

  DECLARE_BUILDER2(void,Scene,size_t,BVH8Triangle4SceneBuilderSAH);
  DECLARE_BUILDER2(void,Scene,size_t,BVH8Triangle8SceneBuilderSAH);
  DECLARE_BUILDER2(void,Scene,size_t,BVH8TrianglePairs4SceneBuilderSAH);

  DECLARE_BUILDER2(void,Scene,size_t,BVH8Triangle4SceneBuilderSpatialSAH);
  DECLARE_BUILDER2(void,Scene,size_t,BVH8Triangle8SceneBuilderSpatialSAH);

  BVH8Factory::BVH8Factory (int features)
  {
    /* select builders */
    SELECT_SYMBOL_INIT_AVX_AVX512(features,BVH8Triangle4SceneBuilderSAH);
    SELECT_SYMBOL_INIT_AVX_AVX512(features,BVH8Triangle8SceneBuilderSAH);
    SELECT_SYMBOL_INIT_AVX_AVX512(features,BVH8TrianglePairs4SceneBuilderSAH);

    SELECT_SYMBOL_INIT_AVX(features,BVH8Triangle4SceneBuilderSpatialSAH);
    SELECT_SYMBOL_INIT_AVX(features,BVH8Triangle8SceneBuilderSpatialSAH);

    /* select intersectors1 */
    SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8Triangle4Intersector1Moeller);
    SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8Triangle8Intersector1Moeller);
    SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8TrianglePairs4Intersector1Moeller);

#if defined (RTCORE_RAY_PACKETS)

    /* select intersectors4 */
    SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8Triangle4Intersector4HybridMoeller);
    SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8Triangle4Intersector4HybridMoellerNoFilter);
    SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8Triangle8Intersector4HybridMoeller);
    SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8Triangle8Intersector4HybridMoellerNoFilter);
    SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8TrianglePairs4Intersector4HybridMoeller);
    SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8TrianglePairs4Intersector4HybridMoellerNoFilter);

    /* select intersectors8 */
    SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8Triangle4Intersector8ChunkMoeller);
    SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8Triangle4Intersector8HybridMoeller);
    SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8Triangle4Intersector8HybridMoellerNoFilter);
    SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8Triangle8Intersector8ChunkMoeller);
    SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8Triangle8Intersector8HybridMoeller);
    SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8Triangle8Intersector8HybridMoellerNoFilter);
    SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8TrianglePairs4Intersector8HybridMoeller);
    SELECT_SYMBOL_INIT_AVX_AVX2(features,BVH8TrianglePairs4Intersector8HybridMoellerNoFilter);

    /* select intersectors16 */
    SELECT_SYMBOL_INIT_AVX512(features,BVH8Triangle4Intersector16ChunkMoeller);
    SELECT_SYMBOL_INIT_AVX512(features,BVH8Triangle4Intersector16HybridMoeller);
    SELECT_SYMBOL_INIT_AVX512(features,BVH8Triangle4Intersector16HybridMoellerNoFilter);

    SELECT_SYMBOL_INIT_AVX512(features,BVH8Triangle8Intersector16ChunkMoeller);
    SELECT_SYMBOL_INIT_AVX512(features,BVH8Triangle8Intersector16HybridMoeller);
    SELECT_SYMBOL_INIT_AVX512(features,BVH8Triangle8Intersector16HybridMoellerNoFilter);

    SELECT_SYMBOL_INIT_AVX512(features,BVH8TrianglePairs4Intersector16HybridMoeller);
    SELECT_SYMBOL_INIT_AVX512(features,BVH8TrianglePairs4Intersector16HybridMoellerNoFilter);

#endif
  }

  Accel::Intersectors BVH8Factory::BVH8Triangle4Intersectors(BVH8* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1           = BVH8Triangle4Intersector1Moeller;
    intersectors.intersector4_filter    = BVH8Triangle4Intersector4HybridMoeller;
    intersectors.intersector4_nofilter  = BVH8Triangle4Intersector4HybridMoellerNoFilter;
    intersectors.intersector8_filter    = BVH8Triangle4Intersector8HybridMoeller;
    intersectors.intersector8_nofilter  = BVH8Triangle4Intersector8HybridMoellerNoFilter;
    intersectors.intersector16_filter   = BVH8Triangle4Intersector16HybridMoeller;
    intersectors.intersector16_nofilter = BVH8Triangle4Intersector16HybridMoellerNoFilter;
    return intersectors;
  }

  Accel::Intersectors BVH8Factory::BVH8Triangle8Intersectors(BVH8* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1           = BVH8Triangle8Intersector1Moeller;
    intersectors.intersector4_filter    = BVH8Triangle8Intersector4HybridMoeller;
    intersectors.intersector4_nofilter  = BVH8Triangle8Intersector4HybridMoellerNoFilter;
    intersectors.intersector8_filter    = BVH8Triangle8Intersector8HybridMoeller;
    intersectors.intersector8_nofilter  = BVH8Triangle8Intersector8HybridMoellerNoFilter;
    intersectors.intersector16_filter   = BVH8Triangle8Intersector16HybridMoeller;
    intersectors.intersector16_nofilter = BVH8Triangle8Intersector16HybridMoellerNoFilter;
    return intersectors;
  }

  Accel::Intersectors BVH8Factory::BVH8TrianglePairs4Intersectors(BVH8* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1           = BVH8TrianglePairs4Intersector1Moeller;
    intersectors.intersector4_filter    = BVH8TrianglePairs4Intersector4HybridMoeller;
    intersectors.intersector4_nofilter  = BVH8TrianglePairs4Intersector4HybridMoellerNoFilter;
    intersectors.intersector8_filter    = BVH8TrianglePairs4Intersector8HybridMoeller;
    intersectors.intersector8_nofilter  = BVH8TrianglePairs4Intersector8HybridMoellerNoFilter;
    intersectors.intersector16_filter   = BVH8TrianglePairs4Intersector16HybridMoeller;
    intersectors.intersector16_nofilter = BVH8TrianglePairs4Intersector16HybridMoellerNoFilter;
    return intersectors;
  }

  Accel* BVH8Factory::BVH8Triangle4(Scene* scene)
  {
    BVH8* accel = new BVH8(Triangle4::type,scene);
    Accel::Intersectors intersectors= BVH8Triangle4Intersectors(accel);

    Builder* builder = nullptr;
    if      (scene->device->tri_builder == "default"     )  builder = BVH8Triangle4SceneBuilderSAH(accel,scene,0);
    else if (scene->device->tri_builder == "sah"         )  builder = BVH8Triangle4SceneBuilderSAH(accel,scene,0);
    else if (scene->device->tri_builder == "sah_spatial" )  builder = BVH8Triangle4SceneBuilderSpatialSAH(accel,scene,0);
    else if (scene->device->tri_builder == "sah_presplit") builder = BVH8Triangle4SceneBuilderSAH(accel,scene,MODE_HIGH_QUALITY);
    else throw_RTCError(RTC_INVALID_ARGUMENT,"unknown builder "+scene->device->tri_builder+" for BVH8<Triangle4>");

    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH8Factory::BVH8Triangle4ObjectSplit(Scene* scene)
  {
    BVH8* accel = new BVH8(Triangle4::type,scene);
    Accel::Intersectors intersectors= BVH8Triangle4Intersectors(accel);
    Builder* builder = BVH8Triangle4SceneBuilderSAH(accel,scene,0);
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH8Factory::BVH8Triangle4SpatialSplit(Scene* scene)
  {
    BVH8* accel = new BVH8(Triangle4::type,scene);
    Accel::Intersectors intersectors= BVH8Triangle4Intersectors(accel);
    Builder* builder = BVH8Triangle4SceneBuilderSpatialSAH(accel,scene,0);
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH8Factory::BVH8Triangle8(Scene* scene)
  {
    BVH8* accel = new BVH8(Triangle8::type,scene);
    Accel::Intersectors intersectors= BVH8Triangle8Intersectors(accel);

    Builder* builder = nullptr;
    if      (scene->device->tri_builder == "default"     ) builder = BVH8Triangle8SceneBuilderSAH(accel,scene,0);
    else if (scene->device->tri_builder == "sah"         ) builder = BVH8Triangle8SceneBuilderSAH(accel,scene,0);
    else if (scene->device->tri_builder == "sah_spatial" ) builder = BVH8Triangle8SceneBuilderSpatialSAH(accel,scene,0);
    else if (scene->device->tri_builder == "sah_presplit") builder = BVH8Triangle8SceneBuilderSAH(accel,scene,MODE_HIGH_QUALITY);
    else throw_RTCError(RTC_INVALID_ARGUMENT,"unknown builder "+scene->device->tri_builder+" for BVH8<Triangle8>");

    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH8Factory::BVH8TrianglePairs4ObjectSplit(Scene* scene)
  {
    BVH8* accel = new BVH8(TrianglePairs4v::type,scene);
    Accel::Intersectors intersectors = BVH8TrianglePairs4Intersectors(accel);

    Builder* builder = nullptr;
    if      (scene->device->tri_builder == "default"     ) builder = BVH8TrianglePairs4SceneBuilderSAH(accel,scene,0);
    else throw_RTCError(RTC_INVALID_ARGUMENT,"unknown builder "+scene->device->tri_builder+" for BVH8<TrianglePairs4v>");

    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH8Factory::BVH8Triangle8ObjectSplit(Scene* scene)
  {
    BVH8* accel = new BVH8(Triangle8::type,scene);
    Accel::Intersectors intersectors= BVH8Triangle8Intersectors(accel);
    Builder* builder = BVH8Triangle8SceneBuilderSAH(accel,scene,0);
    return new AccelInstance(accel,builder,intersectors);
  }

  Accel* BVH8Factory::BVH8Triangle8SpatialSplit(Scene* scene)
  {
    BVH8* accel = new BVH8(Triangle8::type,scene);
    Accel::Intersectors intersectors= BVH8Triangle8Intersectors(accel);
    Builder* builder = BVH8Triangle8SceneBuilderSpatialSAH(accel,scene,0);
    return new AccelInstance(accel,builder,intersectors);
  }
}

#endif
