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
#include "builders/primrefgen.h"

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
      vector<BezierPrim> prims;

      BVH4HairBuilderBinnedSAH (BVH4* bvh, Scene* scene)
        : bvh(bvh), scene(scene) {}
      
      void build(size_t, size_t) 
      {
        /* fast path for empty BVH */
        const size_t numPrimitives = scene->getNumPrimitives<BezierCurves,1>();
        if (numPrimitives == 0) {
          prims.clear();
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

        auto progress = [&] (size_t dn) { bvh->scene->progressMonitor(dn); };
        auto virtualprogress = BuildProgressMonitorFromClosure(progress);
        const PrimInfo pinfo = createBezierRefArray<1>(scene,prims,virtualprogress);
        
        BVH4::NodeRef root = bvh_obb_builder_binned_sah_internal
          (
            [&] () { return bvh->alloc2.threadLocal2(); },

            [&] (const PrimInfo* children, const size_t numChildren, HeuristicArrayBinningSAH<BezierPrim> alignedHeuristic, FastAllocator::ThreadLocal2* alloc) -> BVH4::Node* 
            {
              BVH4::Node* node = (BVH4::Node*) alloc->alloc0.malloc(sizeof(BVH4::Node),16); node->clear();
              for (size_t i=0; i<numChildren; i++)
                node->set(i,children[i].geomBounds);
              return node;
            },
            
            [&] (const PrimInfo* children, const size_t numChildren, UnalignedHeuristicArrayBinningSAH<BezierPrim> unalignedHeuristic, FastAllocator::ThreadLocal2* alloc) -> BVH4::UnalignedNode*
            {
              BVH4::UnalignedNode* node = (BVH4::UnalignedNode*) alloc->alloc0.malloc(sizeof(BVH4::UnalignedNode),16); node->clear();
              for (size_t i=0; i<numChildren; i++) 
              {
                const LinearSpace3fa space = unalignedHeuristic.computeAlignedSpace(children[i]); 
                const PrimInfo       sinfo = unalignedHeuristic.computePrimInfo(children[i],space);
                node->set(i,NAABBox3fa(space,sinfo.geomBounds));
              }
              return node;
            },

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
            progress,
            prims.data(),pinfo,BVH4::N,BVH4::maxBuildDepthLeaf,1,1,BVH4::maxLeafBlocks);
        
        bvh->set(root,pinfo.geomBounds,pinfo.size());
        
        // timer("BVH4BuilderHairNew");
        //});
        
        /* clear temporary data for static geometry */
        const bool staticGeom = scene->isStatic();
        if (staticGeom) prims.clear();
        bvh->alloc2.cleanup();
        
        if (g_verbose >= 2) {
          double t1 = getSeconds();
          std::cout << " [DONE]" << std::endl;
          std::cout << "  dt = " << 1000.0f*(t1-t0) << "ms, perf = " << 1E-6*double(numPrimitives)/(t1-t0) << " Mprim/s" << std::endl;
          bvh->printStatistics();
        }
      }

      void clear() {
        prims.clear();
      }
    };


    template<typename Primitive>
    struct BVH4HairMBBuilderBinnedSAH : public Builder
    {
      BVH4* bvh;
      Scene* scene;
      vector<BezierPrim> prims;

      BVH4HairMBBuilderBinnedSAH (BVH4* bvh, Scene* scene)
        : bvh(bvh), scene(scene) {}
      
      void build(size_t, size_t) 
      {
        /* fast path for empty BVH */
        const size_t numPrimitives = scene->getNumPrimitives<BezierCurves,2>();
        if (numPrimitives == 0) {
          prims.resize(0,true);
          bvh->set(BVH4::emptyNode,empty,0);
          return;
        }
        
        double t0 = 0.0;
        if (g_verbose >= 2) {
          std::cout << "building BVH4<" + bvh->primTy.name + "> using " << TOSTRING(isa) << "::BVH4BuilderHairNewMB ..." << std::flush;
          t0 = getSeconds();
        }
        
        //profile(1,5,numPrimitives,[&] (ProfileTimer& timer) {
        
        /* create primref array */
        bvh->alloc2.init(numPrimitives*sizeof(Primitive));
        prims.resize(numPrimitives);

        auto progress = [&] (size_t dn) { bvh->scene->progressMonitor(dn); };
        auto virtualprogress = BuildProgressMonitorFromClosure(progress);
        const PrimInfo pinfo = createBezierRefArray<2>(scene,prims,virtualprogress);
        
        BVH4::NodeRef root = bvh_obb_builder_binned_sah_internal
          (
            [&] () { return bvh->alloc2.threadLocal2(); },

            [&] (const PrimInfo* children, const size_t numChildren, HeuristicArrayBinningSAH<BezierPrim> alignedHeuristic, FastAllocator::ThreadLocal2* alloc) -> BVH4::NodeMB*
            {
              BVH4::NodeMB* node = (BVH4::NodeMB*) alloc->alloc0.malloc(sizeof(BVH4::NodeMB),16); node->clear();
              for (size_t i=0; i<numChildren; i++) 
              {
                std::pair<BBox3fa,BBox3fa> bounds = alignedHeuristic.computePrimInfoMB(scene,children[i]);
                node->set(i,bounds.first,bounds.second);
              }
              return node;
            },
            
            [&] (const PrimInfo* children, const size_t numChildren, UnalignedHeuristicArrayBinningSAH<BezierPrim> unalignedHeuristic, FastAllocator::ThreadLocal2* alloc) -> BVH4::UnalignedNodeMB*
            {
              BVH4::UnalignedNodeMB* node = (BVH4::UnalignedNodeMB*) alloc->alloc0.malloc(sizeof(BVH4::UnalignedNodeMB),16); node->clear();
              for (size_t i=0; i<numChildren; i++) 
              {
                std::pair<AffineSpace3fa,AffineSpace3fa> spaces = unalignedHeuristic.computeAlignedSpaceMB(scene,children[i]); 
                
#if BVH4HAIR_MB_VERSION == 0
                Vec3fa axis = normalize(spaces.first.l.row2()+spaces.second.l.row2());
                spaces.first = spaces.second = frame(axis).transposed();
                UnalignedHeuristicArrayBinningSAH<BezierPrim>::PrimInfoMB pinfo = unalignedHeuristic.computePrimInfoMB(scene,children[i],spaces);
                node->set(i,spaces.first,pinfo.s0t0,pinfo.s1t1);
#elif BVH4HAIR_MB_VERSION == 1
                UnalignedHeuristicArrayBinningSAH<BezierPrim>::PrimInfoMB pinfo1 = unalignedHeuristic.computePrimInfoMB(scene,children[i],spaces);
                spaces.first = BVH4::UnalignedNodeMB::normalizeSpace(spaces.first,pinfo1.s0t0);
                spaces.second = BVH4::UnalignedNodeMB::normalizeSpace(spaces.second,pinfo1.s1t1);
                unalignedHeuristic.PrimInfoMB pinfo = unalignedHeuristic.computePrimInfoMB(scene,children[i],spaces);
                node->set(i,spaces.first,spaces.second);
                node->set(i,pinfo.s0t0,pinfo.s0t1_s1t0,pinfo.s1t1);
#elif BVH4HAIR_MB_VERSION == 2
                
                UnalignedHeuristicArrayBinningSAH<BezierPrim>::PrimInfoMB pinfo1 = unalignedHeuristic.computePrimInfoMB(scene,children[i],spaces);
                
                Vec3fa k0 = 0.5f*(pinfo1.s0t0.lower+pinfo1.s0t0.upper);
                Vec3fa k1 = 0.5f*(pinfo1.s1t1.lower+pinfo1.s1t1.upper);
                Vec3fa d0(k0.x,k0.y,pinfo1.s0t0.lower.z);
                Vec3fa d1(k1.x,k1.y,pinfo1.s1t1.lower.z);
                spaces.first.p  -= d0; pinfo1.s0t0.lower -= d0; pinfo1.s0t0.upper -= d0;
                spaces.second.p -= d1; pinfo1.s1t1.lower -= d1; pinfo1.s1t1.upper -= d1;
                
                UnalignedHeuristicArrayBinningSAH<BezierPrim>::PrimInfoMB pinfo = unalignedHeuristic.computePrimInfoMB(scene,children[i],spaces);
                
                Vec3fa a0 = xfmVector(spaces.first.l.transposed(),-spaces.first.p);
                Vec3fa a1 = xfmVector(spaces.second.l.transposed(),-spaces.second.p);
                Vec3fa b0 = xfmVector(spaces.first .l.transposed(),Vec3fa(0.0f,0.0f,pinfo1.s0t0.upper.z)-spaces.first .p);
                Vec3fa b1 = xfmVector(spaces.second.l.transposed(),Vec3fa(0.0f,0.0f,pinfo1.s1t1.upper.z)-spaces.second.p);
                float r0 = max(abs(pinfo.s0t0.lower.x),abs(pinfo.s0t0.lower.y),abs(pinfo.s0t0.upper.x),abs(pinfo.s0t0.upper.y));
                float r1 = max(abs(pinfo.s1t1.lower.x),abs(pinfo.s1t1.lower.y),abs(pinfo.s1t1.upper.x),abs(pinfo.s1t1.upper.y));
                node->set(i,a0,a1,b0,b1,r0,r1);
#endif
              }
              return node;
            },

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
            progress,
            prims.data(),pinfo,BVH4::N,BVH4::maxBuildDepthLeaf,1,1,BVH4::maxLeafBlocks);
        
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

      void clear() {
        prims.clear();
      }
    };
    
    /*! entry functions for the builder */
    Builder* BVH4Bezier1vBuilder_OBB_New (void* bvh, Scene* scene, size_t mode) { return new BVH4HairBuilderBinnedSAH<Bezier1v>((BVH4*)bvh,scene); }
    Builder* BVH4Bezier1iBuilder_OBB_New (void* bvh, Scene* scene, size_t mode) { return new BVH4HairBuilderBinnedSAH<Bezier1i>((BVH4*)bvh,scene); }
    Builder* BVH4Bezier1iMBBuilder_OBB_New (void* bvh, Scene* scene, size_t mode) { return new BVH4HairMBBuilderBinnedSAH<Bezier1iMB> ((BVH4*)bvh,scene); }
  }
}
