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

namespace embree
{
  namespace isa
  {
    static const size_t MAX_BRANCHING_FACTOR = 16;  //!< maximal supported BVH branching factor
    static const size_t MIN_LARGE_LEAF_LEVELS = 8;  //!< create balanced tree of we are that many levels before the maximal tree depth

    template<typename Primitive>
    BVH4BuilderHairNew<Primitive>::BVH4BuilderHairNew (BVH4* bvh, Scene* scene, size_t mode)
      : scene(scene), maxDepth(BVH4::maxBuildDepthLeaf), minLeafSize(1), maxLeafSize(BVH4::maxLeafBlocks), bvh(bvh) {}
    
    template<typename Primitive>
    BVH4::NodeRef BVH4BuilderHairNew<Primitive>::createLeaf(Allocator& alloc, size_t depth, const PrimInfo& pinfo)
    {
      /*if (g_verbose >= 2) {
        static atomic_t N = 0;
        size_t n = atomic_add(&N,pinfo.size());
        if (n>1024) { std::cout << "." << std::flush; N = 0; }
        }*/

      size_t items = pinfo.size();
      size_t start = pinfo.begin;
      Primitive* accel = (Primitive*) alloc.malloc(items*sizeof(Primitive));
      BVH4::NodeRef node = bvh->encodeLeaf((char*)accel,items);
      for (size_t i=0; i<items; i++) {
        accel[i].fill(prims.data(),start,pinfo.end,bvh->scene,false);
      }
      return node;
    }

    template<typename Primitive>
    BVH4::NodeRef BVH4BuilderHairNew<Primitive>::createLargeLeaf(Allocator& alloc, size_t depth, const PrimInfo& pinfo)
    {
      //if (current.depth > maxDepth) 
      //  THROW_RUNTIME_ERROR("depth limit reached");
      
      /* create leaf for few primitives */
      if (pinfo.size() <= maxLeafSize)
        return createLeaf(alloc,depth,pinfo);
      
      /* fill all children by always splitting the largest one */
      //ReductionTy values[MAX_BRANCHING_FACTOR];
      //PrimInfo* pchildren[MAX_BRANCHING_FACTOR];
      PrimInfo children[BVH4::N]; //MAX_BRANCHING_FACTOR];
      size_t numChildren = 1;
      children[0] = pinfo;
      //pchildren[0] = &children[0];
      
      do {
        
        /* find best child with largest bounding box area */
        int bestChild = -1;
        int bestSize = 0;
        for (size_t i=0; i<numChildren; i++)
        {
          /* ignore leaves as they cannot get split */
          if (children[i].size() <= maxLeafSize)
            continue;
          
          /* remember child with largest size */
          if (children[i].size() > bestSize) { 
            bestSize = children[i].size();
            bestChild = i;
          }
        }
        if (bestChild == -1) break;
        
        /*! split best child into left and right child */
        __aligned(64) PrimInfo left, right;
        alignedHeuristic.splitFallback(children[bestChild],left,right);
        
        /* add new children left and right */
        //left.init(pinfo.depth+1); 
        //right.init(pinfo.depth+1);
        children[bestChild] = children[numChildren-1];
        children[numChildren-1] = left;
        children[numChildren+0] = right;
        //pchildren[numChildren] = &children[numChildren];
        numChildren++;
        
      } while (numChildren < BVH4::N); // branchingFactor);
      
      /* create node */
      //auto node = createNode(pinfo,pchildren,numChildren,alloc);
      BVH4::Node* node = bvh->allocNode(alloc);
      for (size_t i=0; i<numChildren; i++) {
        node->set(i,children[i].geomBounds);
        node->set(i,createLargeLeaf(alloc,depth+1,children[i]));
      }
      return BVH4::encodeNode(node);
      
      /* recurse into each child */
      //for (size_t i=0; i<numChildren; i++)
      //values[i] = createLargeLeaf(children[i],alloc);
      
      /* perform reduction */
      //return updateNode(node,values,numChildren);
    }

    template<typename Primitive>
    bool BVH4BuilderHairNew<Primitive>::split(const PrimInfo& pinfo, PrimInfo& linfo, PrimInfo& rinfo)
    {
      /* variable to track the SAH of the best splitting approach */
      float bestSAH = inf;
      const float leafSAH = BVH4::intCost*float(pinfo.size())*halfArea(pinfo.geomBounds);
      
      /* perform standard binning in aligned space */
      float alignedObjectSAH = inf;
      HeuristicArrayBinningSAH<BezierPrim>::Split alignedObjectSplit;
      //if (pinfo.size() > 100) {
        alignedObjectSplit = alignedHeuristic.find(pinfo,0);
        alignedObjectSAH = BVH4::travCostAligned*halfArea(pinfo.geomBounds) + BVH4::intCost*alignedObjectSplit.splitSAH();
        bestSAH = min(alignedObjectSAH,bestSAH);
        //}

      /* perform standard binning in unaligned space */
      UnalignedHeuristicArrayBinningSAH<BezierPrim>::Split unalignedObjectSplit;
      LinearSpace3fa uspace;
      float unalignedObjectSAH = inf;
      //if (alignedObjectSAH > 0.7f*leafSAH) {
      //if (pinfo.size() <= 100) {
        uspace = unalignedHeuristic.computeAlignedSpace(pinfo); 
        const PrimInfo       sinfo = unalignedHeuristic.computePrimInfo(pinfo,uspace);
        unalignedObjectSplit = unalignedHeuristic.find(sinfo,0,uspace);    	
        unalignedObjectSAH = BVH4::travCostUnaligned*halfArea(pinfo.geomBounds) + BVH4::intCost*unalignedObjectSplit.splitSAH();
        bestSAH = min(unalignedObjectSAH,bestSAH);
        //}
      
      /* perform best split */
      if (bestSAH == float(inf)) {
        alignedHeuristic.splitFallback(pinfo,linfo,rinfo);
        return true;
      }
      else if (bestSAH == alignedObjectSAH) {
        alignedHeuristic.split(alignedObjectSplit,pinfo,linfo,rinfo);
        return true;
      }
      else if (bestSAH == unalignedObjectSAH) {
        unalignedHeuristic.split(unalignedObjectSplit,uspace,pinfo,linfo,rinfo);
        return false;
      }
      else {
        THROW_RUNTIME_ERROR("bvh4hair_builder: internal error"); // FIXME: remove
        return true;
      }
    }
    
    template<typename Primitive>
    BVH4::NodeRef BVH4BuilderHairNew<Primitive>::recurse(Allocator& alloc, size_t depth, const PrimInfo& pinfo)
    {
      //if (alloc == NULL) 
      //  alloc = createAlloc();
	
      //ReductionTy values[MAX_BRANCHING_FACTOR];
      //PrimInfo children[MAX_BRANCHING_FACTOR];
      PrimInfo children[BVH4::N];
      //__aligned(64) PrimInfo children[MAX_BRANCHING_FACTOR];
        
      /* create leaf node */
      if (depth+MIN_LARGE_LEAF_LEVELS >= maxDepth || pinfo.size() <= minLeafSize) {
        //heuristic.deterministic_order(current.prims);
        return createLargeLeaf(alloc,depth,pinfo);
      }
                
      /* fill all children by always splitting the one with the largest surface area */
      size_t numChildren = 1;
      children[0] = pinfo;
      //pchildren[0] = &children[0];
      bool aligned = true;

      do {
        
        /* find best child with largest bounding box area */
        int bestChild = -1;
        float bestArea = neg_inf;
        for (size_t i=0; i<numChildren; i++)
        {
          /* ignore leaves as they cannot get split */
          if (children[i].size() <= minLeafSize)
            continue;
          
          /* remember child with largest area */
          if (area(children[i].geomBounds) > bestArea) { 
            bestArea = area(children[i].geomBounds);
            bestChild = i;
          }
        }
        if (bestChild == -1) break;
        
        /*! split best child into left and right child */
        PrimInfo left, right;
        //if (children[bestChild].size() > 10000) splitParallel  (children[bestChild],left,right);
        //else                                    splitSequential(children[bestChild],left,right);
        aligned &= split(children[bestChild],left,right);
        
        /* add new children left and right */
        //left.init(current.depth+1); 
        //right.init(current.depth+1);
        children[bestChild] = children[numChildren-1];
        children[numChildren-1] = left;
        children[numChildren+0] = right;
        //pchildren[numChildren] = &children[numChildren];
        numChildren++;
          
      } while (numChildren < BVH4::N); //branchingFactor);
      assert(numChildren > 1);
	
      /* sort buildrecords for optimal cache locality */
      //std::sort(&children[0],&children[numChildren]);
      
      /* create node */
      //auto node = createNode(current,pchildren,numChildren,alloc);
      
      /* create aligned node */
      if (aligned) 
      {
        BVH4::Node* node = bvh->allocNode(alloc);
        for (size_t i=0; i<numChildren; i++) {
          node->set(i,children[i].geomBounds);
          node->child(i) = recurse(alloc,depth+1,children[i]);
        }
        return BVH4::encodeNode(node);
      }
      
      /* create unaligned node */
      else 
      {
        BVH4::UnalignedNode* node = bvh->allocUnalignedNode(alloc);
        for (size_t i=0; i<numChildren; i++) 
        {
          const LinearSpace3fa space = unalignedHeuristic.computeAlignedSpace(children[i]); 
          const PrimInfo       sinfo = unalignedHeuristic.computePrimInfo(children[i],space);
          node->set(i,NAABBox3fa(space,sinfo.geomBounds));
          node->child(i) = recurse(alloc,depth+1,children[i]);
        }
        return BVH4::encodeNode(node);
      }
    }

    template<typename Primitive>
    void BVH4BuilderHairNew<Primitive>::build(size_t threadIndex, size_t threadCount) 
    {
      /* fast path for empty BVH */
      const size_t numPrimitives = scene->getNumPrimitives<BezierCurves,1>();
      bvh->init(sizeof(BVH4::UnalignedNode),numPrimitives,threadCount);
      if (numPrimitives == 0) return;
            
      double t0 = 0.0;
      if (g_verbose >= 2) {
        std::cout << "building BVH4<" + bvh->primTy.name + "> using " << TOSTRING(isa) << "::BVH4BuilderHairNew ..." << std::flush;
        t0 = getSeconds();
      }
      
      /* create primref array */
      bvh->alloc2.init(numPrimitives*sizeof(Primitive),numPrimitives*sizeof(BVH4::Node));
      prims.resize(numPrimitives);
      new (  &alignedHeuristic)          HeuristicArrayBinningSAH<BezierPrim>(prims.data());
      new (&unalignedHeuristic) UnalignedHeuristicArrayBinningSAH<BezierPrim>(prims.data());

      const PrimInfo pinfo = createBezierRefArray<1>(scene,prims);
      Allocator alloc(&bvh->alloc);
      BVH4::NodeRef root = recurse(alloc,1,pinfo);
      bvh->set(root,pinfo.geomBounds,pinfo.size());

      /* clear temporary data for static geometry */
      bool staticGeom = scene->isStatic();
      if (staticGeom) prims.resize(0,true);
      bvh->alloc2.cleanup();

      if (g_verbose >= 2) {
        double t1 = getSeconds();
        std::cout << " [DONE]" << std::endl;
        std::cout << "  dt = " << 1000.0f*(t1-t0) << "ms, perf = " << 1E-6*double(numPrimitives)/(t1-t0) << " Mprim/s" << std::endl;
        bvh->printStatistics();
        //std::cout << BVH4Statistics(bvh).str();
      }
    }
    
    /*! entry functions for the builder */
    Builder* BVH4Bezier1vBuilder_OBB_New (void* bvh, Scene* scene, size_t mode) { return new BVH4BuilderHairNew<Bezier1v>((BVH4*)bvh,scene,mode); }
    Builder* BVH4Bezier1iBuilder_OBB_New (void* bvh, Scene* scene, size_t mode) { return new BVH4BuilderHairNew<Bezier1i>((BVH4*)bvh,scene,mode); }
  }
}
