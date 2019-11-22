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

#include "bvh.h"
#include "bvh_builder.h"
#include "../builders/bvh_builder_msmblur.h"

#include "../builders/primrefgen.h"
#include "../builders/splitter.h"

#include "../geometry/linei.h"
#include "../geometry/triangle.h"
#include "../geometry/trianglev.h"
#include "../geometry/trianglev_mb.h"
#include "../geometry/trianglei.h"
#include "../geometry/quadv.h"
#include "../geometry/quadi.h"
#include "../geometry/object.h"
#include "../geometry/instance.h"
#include "../geometry/subgrid.h"

#include "../common/state.h"

namespace embree
{
  namespace isa
  {

    template<int N, typename Mesh, typename Primitive>
    struct CreateMSMBlurLeaf
    {
      typedef BVHN<N> BVH;
      typedef typename BVH::NodeRef NodeRef;
      typedef typename BVH::NodeRecordMB4D NodeRecordMB4D;

      __forceinline CreateMSMBlurLeaf (BVH* bvh) : bvh(bvh) {}

      __forceinline const NodeRecordMB4D operator() (const BVHBuilderMSMBlur::BuildRecord& current, const FastAllocator::CachedAllocator& alloc) const
      {
        size_t items = Primitive::blocks(current.prims.size());
        size_t start = current.prims.begin();
        size_t end   = current.prims.end();
        for (size_t i=start; i<end; i++) assert((*current.prims.prims)[start].geomID() == (*current.prims.prims)[i].geomID()); // assert that all geomIDs are identical
        Primitive* accel = (Primitive*) alloc.malloc1(items*sizeof(Primitive),BVH::byteNodeAlignment);
        NodeRef node = bvh->encodeLeaf((char*)accel,items);
        LBBox3fa allBounds = empty;
        for (size_t i=0; i<items; i++)
          allBounds.extend(accel[i].fillMB(current.prims.prims->data(), start, current.prims.end(), bvh->scene, current.prims.time_range));
        return NodeRecordMB4D(node,allBounds,current.prims.time_range);
      }

      BVH* bvh;
    };

    /* Motion blur BVH with 4D nodes and internal time splits */
    template<int N, typename Mesh, typename Primitive>
    struct BVHGPUBuilderMBlurSAH : public Builder
    {
      typedef BVHN<N> BVH;
      typedef typename BVHN<N>::NodeRef NodeRef;
      typedef typename BVHN<N>::NodeRecordMB NodeRecordMB;
      typedef typename BVHN<N>::AlignedNodeMB AlignedNodeMB;
      typedef typename BVHN<N>::AlignedNodeMB4D AlignedNodeMB4D;
      typedef typename BVHN<N>::AlignedNode AlignedNode;
      typedef typename BVHN<N>::UnalignedNode UnalignedNode;
      typedef typename BVHN<N>::UnalignedNodeMB UnalignedNodeMB;
      typedef typename BVHN<N>::QuantizedNode QuantizedNode;

      BVH* bvh;
      Scene* scene;
      const size_t sahBlockSize;
      const float intCost;
      const size_t minLeafSize;
      const size_t maxLeafSize;
      const Geometry::GTypeMask gtype_;

      BVHGPUBuilderMBlurSAH (BVH* bvh, Scene* scene, const size_t sahBlockSize, const float intCost, const size_t minLeafSize, const size_t maxLeafSize, const Geometry::GTypeMask gtype)
        : bvh(bvh), scene(scene), sahBlockSize(sahBlockSize), intCost(intCost), minLeafSize(minLeafSize), maxLeafSize(min(maxLeafSize,Primitive::max_size()*BVH::maxLeafBlocks)), gtype_(gtype) {}

      void convertToGPULayout(NodeRef node)
      {
	if (node.isAlignedNode())
	  {
	    PRINT("ALIGNED NODE");
	    AlignedNode* n = node.alignedNode();
	    for (size_t i=0;i<N;i++)
	      convertToGPULayout(n->child(i));
	  }
	else if (node.isUnalignedNode())
	  {
	    PRINT("UNALIGNED NODE");
	    UnalignedNode* n = node.unalignedNode();
	    for (size_t i=0;i<N;i++)
	      convertToGPULayout(n->child(i));	    
	  }
	else if (node.isAlignedNodeMB())
	  {
	    PRINT("ALIGNED NODE MB");
	    AlignedNodeMB* n = node.alignedNodeMB();
	    for (size_t i=0;i<N;i++)
	      convertToGPULayout(n->child(i));
	  }
	else if (node.isAlignedNodeMB4D())
	  {
	    PRINT("ALIGNED NODE MB 4D");
	    AlignedNodeMB4D* n = node.alignedNodeMB4D();
	    PRINT(*n);
	    for (size_t i=0;i<N;i++)
	      convertToGPULayout(n->child(i));	    
	  }
	else if (node.isUnalignedNodeMB())
	  {
	    PRINT("UNALIGNED NODE MB");	    
	    UnalignedNodeMB* n = node.unalignedNodeMB();
	    for (size_t i=0;i<N;i++)
	      convertToGPULayout(n->child(i));	    
	  }
	else if (node.isQuantizedNode())
	  {
	    PRINT("QUANTIZED NODE");	    
	    QuantizedNode* n = node.quantizedNode();
	    for (size_t i=0;i<N;i++)
	      convertToGPULayout(n->child(i));	    	    
	  }
	else if (node.isLeaf())
	  {
	    size_t num; const char* tri = node.leaf(num);
	    if (num)
	      {
		for (size_t i=0; i<num; i++)
		  {
		    const size_t bytes = bvh->primTy->getBytes(tri);
		  }
	      }
	  }
	else {
	  throw std::runtime_error("not supported node type in bvh_statistics");
	}
	
      }
      
      void build()
      {
	/* skip build for empty scene */
        const size_t numPrimitives = scene->getNumPrimitives(gtype_,true);
        if (numPrimitives == 0) { bvh->clear(); return; }

        double t0 = bvh->preBuild(TOSTRING(isa) "::BVH" + toString(N) + "BuilderMBlurSAH");

	buildMultiSegment(numPrimitives);

	/* clear temporary data for static geometry */
        if (scene->isStaticAccel()) bvh->shrink();
	bvh->cleanup();
        bvh->postBuild(t0);

	convertToGPULayout(bvh->root);
	PRINT("BVH MB BUILDING DONE");      
	exit(0);	

      }

      void buildMultiSegment(size_t numPrimitives)
      {
        /* create primref array */
        mvector<PrimRefMB> prims(scene->device,numPrimitives);
	PrimInfoMB pinfo = createPrimRefArrayMSMBlur(scene,gtype_,numPrimitives,prims,bvh->scene->progressInterface);

        /* early out if no valid primitives */
        if (pinfo.size() == 0) { bvh->clear(); return; }

        /* estimate acceleration structure size */
        const size_t node_bytes = pinfo.num_time_segments*sizeof(AlignedNodeMB)/(4*N);
        const size_t leaf_bytes = size_t(1.2*Primitive::blocks(pinfo.num_time_segments)*sizeof(Primitive));
        bvh->alloc.init_estimate(node_bytes+leaf_bytes);

        /* settings for BVH build */
        BVHBuilderMSMBlur::Settings settings;
        settings.branchingFactor = N;
        settings.maxDepth = BVH::maxDepth;
        settings.logBlockSize = bsr(sahBlockSize);
        settings.minLeafSize = minLeafSize;
        settings.maxLeafSize = maxLeafSize;
        settings.travCost = travCost;
        settings.intCost = intCost;
        settings.singleLeafTimeSegment = Primitive::singleTimeSegment;
        settings.singleThreadThreshold = bvh->alloc.fixSingleThreadThreshold(N,DEFAULT_SINGLE_THREAD_THRESHOLD,pinfo.size(),node_bytes+leaf_bytes);
        
        /* build hierarchy */
        auto root =
          BVHBuilderMSMBlur::build<NodeRef>(prims,pinfo,scene->device,
                                            RecalculatePrimRef<Mesh>(scene),
                                            typename BVH::CreateAlloc(bvh),
                                            typename BVH::AlignedNodeMB4D::Create(),
                                            typename BVH::AlignedNodeMB4D::Set(),
                                            CreateMSMBlurLeaf<N,Mesh,Primitive>(bvh),
                                            bvh->scene->progressInterface,
                                            settings);

        bvh->set(root.ref,root.lbounds,pinfo.num_time_segments);
      }

      
      void clear() {
      }
    };

#if defined(EMBREE_GEOMETRY_TRIANGLE)
    Builder* BVHGPUTriangle1vMBSceneBuilderSAH (void* bvh, Scene* scene) { return new BVHGPUBuilderMBlurSAH<4,TriangleMesh,Triangle4i>((BVH4*)bvh,scene,4,1.0f,4,inf,TriangleMesh::geom_type); }
#endif

#if defined(EMBREE_GEOMETRY_QUAD)
    Builder* BVHGPUQuad1vMBSceneBuilderSAH (void* bvh, Scene* scene) { return new BVHGPUBuilderMBlurSAH<4,QuadMesh,Quad4i>((BVH4*)bvh,scene,4,1.0f,4,inf,QuadMesh::geom_type); }    
#endif
    
  } 
}
