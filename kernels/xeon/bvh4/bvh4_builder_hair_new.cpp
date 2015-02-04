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

#include "bvh4.h"
#include "bvh4_builder_hair_new.h"
#include "builders_new/primrefgen.h"

#include "geometry/bezier1v.h"
#include "geometry/bezier1i.h"

#include "common/profile.h"

namespace embree
{
  namespace isa
  {
    template<typename Primitive>
    struct BVH4HairBuilderBinnedSAH : public Builder
    {
      BVH4* bvh;
      Scene* scene;
      vector_t<BezierPrim> prims;

      BVH4HairBuilderBinnedSAH (BVH4* bvh, Scene* scene)
        : bvh(bvh), scene(scene) {}
      
      void build(size_t, size_t) 
      {
        /* fast path for empty BVH */
        const size_t numPrimitives = scene->getNumPrimitives<BezierCurves,1>();
        if (numPrimitives == 0) {
          prims.resize(0,true);
          bvh->set(BVH4::emptyNode,empty,0);
          return;
        }
        
        double t0 = 0.0;
        if (g_verbose >= 2) {
          std::cout << "building BVH4<" + bvh->primTy.name + "> using " << TOSTRING(isa) << "::BVH4BuilderHairNew ..." << std::flush;
          t0 = getSeconds();
        }
        
        //profile(1,5,numPrimitives,[&] (ProfileTimer& timer) {
        
        /* create primref array */
        bvh->alloc2.init(numPrimitives*sizeof(Primitive));
        prims.resize(numPrimitives);
        
        const PrimInfo pinfo = createBezierRefArray<1>(scene,prims);
        
        BVH4::NodeRef root = bvh_obb_builder_binned_sah_internal
          (
            [&] () { return bvh->alloc2.threadLocal2(); },
            [&] (size_t depth, const PrimInfo& pinfo, FastAllocator::ThreadLocal2* alloc) -> BVH4::NodeRef
            {
              size_t items = pinfo.size();
              size_t start = pinfo.begin;
              Primitive* accel = (Primitive*) alloc->alloc1.malloc(items*sizeof(Primitive));
              BVH4::NodeRef node = bvh->encodeLeaf((char*)accel,items);
              for (size_t i=0; i<items; i++) {
                accel[i].fill(prims.data(),start,pinfo.end,bvh->scene,false);
              }
              return node;
            },
            prims.data(),pinfo); //,BVH4::N,BVH4::maxBuildDepthLeaf,sahBlockSize,minLeafSize,maxLeafSize);
        
        bvh->set(root,pinfo.geomBounds,pinfo.size());
        
        // timer("BVH4BuilderHairNew");
        //});
        
        /* clear temporary data for static geometry */
        const bool staticGeom = scene->isStatic();
        if (staticGeom) prims.resize(0,true);
        bvh->alloc2.cleanup();
        
        if (g_verbose >= 2) {
          double t1 = getSeconds();
          std::cout << " [DONE]" << std::endl;
          std::cout << "  dt = " << 1000.0f*(t1-t0) << "ms, perf = " << 1E-6*double(numPrimitives)/(t1-t0) << " Mprim/s" << std::endl;
          bvh->printStatistics();
        }
      }
    };
    
    /*! entry functions for the builder */
    Builder* BVH4Bezier1vBuilder_OBB_New (void* bvh, Scene* scene, size_t mode) { return new BVH4HairBuilderBinnedSAH<Bezier1v>((BVH4*)bvh,scene); }
    Builder* BVH4Bezier1iBuilder_OBB_New (void* bvh, Scene* scene, size_t mode) { return new BVH4HairBuilderBinnedSAH<Bezier1i>((BVH4*)bvh,scene); }
  }
}
