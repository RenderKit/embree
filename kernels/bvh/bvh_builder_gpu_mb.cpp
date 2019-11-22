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
#include "../geometry/triangle1v.h"
#include "../geometry/triangle1vmb.h"
#include "../geometry/quadv.h"
#include "../geometry/quadi.h"
#include "../geometry/object.h"
#include "../geometry/instance.h"
#include "../geometry/subgrid.h"

#include "../common/state.h"

#include "../gpu/AABB.h"
#include "../gpu/AABB3f.h"
#include "../gpu/builder.h"


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

      inline void convertToQBVHNodeNMB(gpu::QBVHNodeNMB *gpu_node, const AlignedNodeMB4D *const n)
      {
	for (size_t i=0;i<N;i++)
	  {
	    gpu_node->offset[i] = 0;
	    gpu_node->lower_t[i]  = n->lower_t[i];
	    gpu_node->upper_t[i]  = n->upper_t[i];	    	    
	    gpu_node->lower_x[i]  = n->lower_x[i];
	    gpu_node->upper_x[i]  = n->upper_x[i];
	    gpu_node->lower_y[i]  = n->lower_y[i];
	    gpu_node->upper_y[i]  = n->upper_y[i];
	    gpu_node->lower_z[i]  = n->lower_z[i];
	    gpu_node->upper_z[i]  = n->upper_z[i];
	    gpu_node->lower_dx[i] = n->lower_dx[i];
	    gpu_node->upper_dx[i] = n->upper_dx[i];
	    gpu_node->lower_dy[i] = n->lower_dy[i];
	    gpu_node->upper_dy[i] = n->upper_dy[i];
	    gpu_node->lower_dz[i] = n->lower_dz[i];
	    gpu_node->upper_dz[i] = n->upper_dz[i];
	  }
      }

      inline void convertToTriangle1vMB(gpu::Triangle1vMB &tri1v,Triangle4vMB &tri_block, const size_t slot)
      {
	tri1v.v0.x() = tri_block.v0.x[slot];
	tri1v.v0.y() = tri_block.v0.y[slot];
	tri1v.v0.z() = tri_block.v0.z[slot];
	tri1v.v0.w() = gpu::as_float(tri_block.primID()[slot]);
	
	tri1v.v1.x() = tri_block.v1.x[slot];
	tri1v.v1.y() = tri_block.v1.y[slot];
	tri1v.v1.z() = tri_block.v1.z[slot];
	tri1v.v1.w() = gpu::as_float(tri_block.geomID()[slot]);

	tri1v.v2.x() = tri_block.v2.x[slot];
	tri1v.v2.y() = tri_block.v2.y[slot];
	tri1v.v2.z() = tri_block.v2.z[slot];
	tri1v.v2.w() = 0.0f;

	tri1v.d0.x() = tri_block.dv0.x[slot];
	tri1v.d0.y() = tri_block.dv0.y[slot];
	tri1v.d0.z() = tri_block.dv0.z[slot];
	
	tri1v.d1.x() = tri_block.dv1.x[slot];
	tri1v.d1.y() = tri_block.dv1.y[slot];
	tri1v.d1.z() = tri_block.dv1.z[slot];

	tri1v.d2.x() = tri_block.dv2.x[slot];
	tri1v.d2.y() = tri_block.dv2.y[slot];
	tri1v.d2.z() = tri_block.dv2.z[slot];


      }
      
      void convertToGPULayout(NodeRef node,
			      char* bvh_mem,
			      gpu::Triangle1vMB *leaf_ptr,
			      size_t parent_node_offset,
			      size_t slot,
			      std::atomic<size_t> &gpu_node_allocator,
			      std::atomic<size_t> &gpu_leaf_allocator)
      {
	if (node.isAlignedNode())
	  {
	    PRINT("ALIGNED NODE");
	    AlignedNode* n = node.alignedNode();
	    for (size_t i=0;i<N;i++)
	      if (n->child(i) != BVH::emptyNode)
		convertToGPULayout(n->child(i),bvh_mem,leaf_ptr,0,i,gpu_node_allocator,gpu_leaf_allocator);
	  }
	else if (node.isUnalignedNode())
	  {
	    PRINT("UNALIGNED NODE");
	    UnalignedNode* n = node.unalignedNode();	    
	    for (size_t i=0;i<N;i++)
	      if (n->child(i) != BVH::emptyNode)	      
		convertToGPULayout(n->child(i),bvh_mem,leaf_ptr,0,i,gpu_node_allocator,gpu_leaf_allocator);	    
	  }
	else if (node.isAlignedNodeMB())
	  {
	    PRINT("ALIGNED NODE MB");
	    AlignedNodeMB* n = node.alignedNodeMB();
	    for (size_t i=0;i<N;i++)
	      if (n->child(i) != BVH::emptyNode)	      
		convertToGPULayout(n->child(i),bvh_mem,leaf_ptr,0,i,gpu_node_allocator,gpu_leaf_allocator);
	  }
	else if (node.isAlignedNodeMB4D())
	  {
	    PRINT("ALIGNED NODE MB 4D");
	    AlignedNodeMB4D* n = node.alignedNodeMB4D();
	    PRINT(*n);

	    const size_t offset = gpu_node_allocator.fetch_add(sizeof(gpu::QBVHNodeNMB));
	    PRINT(offset);

	    /* set parent offset */
	    if (parent_node_offset)
	      {
		gpu::QBVHNodeNMB *parent_gpu_node = (gpu::QBVHNodeNMB*)(bvh_mem + parent_node_offset);
		parent_gpu_node->offset[slot] = offset - parent_node_offset;
		PRINT( parent_gpu_node->offset[slot] );
	      }

	    /* create new gpu node and convert layout */
	    gpu::QBVHNodeNMB *gpu_node = (gpu::QBVHNodeNMB*)(bvh_mem + offset);
	    gpu_node->clear();
	    convertToQBVHNodeNMB(gpu_node,n);
	      
	    for (size_t i=0;i<N;i++)
	      if (n->child(i) != BVH::emptyNode)	      
		convertToGPULayout(n->child(i),bvh_mem,leaf_ptr,offset,i,gpu_node_allocator,gpu_leaf_allocator);
	  }
	else if (node.isUnalignedNodeMB())
	  {
	    PRINT("UNALIGNED NODE MB");	    
	    UnalignedNodeMB* n = node.unalignedNodeMB();
	    for (size_t i=0;i<N;i++)
	      if (n->child(i) != BVH::emptyNode)	      
		convertToGPULayout(n->child(i),bvh_mem,leaf_ptr,0,i,gpu_node_allocator,gpu_leaf_allocator);	    
	  }
	else if (node.isQuantizedNode())
	  {
	    PRINT("QUANTIZED NODE");	    
	    QuantizedNode* n = node.quantizedNode();
	    for (size_t i=0;i<N;i++)
	      if (n->child(i) != BVH::emptyNode)	      
		convertToGPULayout(n->child(i),bvh_mem,leaf_ptr,0,i,gpu_node_allocator,gpu_leaf_allocator);
	  }
	else if (node.isLeaf())
	  {
	    PRINT("LEAF");
	    size_t num; const char* tri = node.leaf(num);
	    PRINT(num);
	    
	    if (num)
	      {
		Triangle4vMB *tri_mb = (Triangle4vMB*)tri;

		size_t numTris=0;
		for (size_t i=0; i<num; i++)
		  numTris += tri_mb[i].size();

		const size_t leaf_index = gpu_leaf_allocator.fetch_add(numTris);		    

		size_t index=0;
		for (size_t i=0; i<num; i++)
		  {
		    for (size_t j=0;j<tri_mb[i].size();j++)
		      convertToTriangle1vMB(leaf_ptr[index++],tri_mb[i],j);
		    
		    PRINT(i);
		    PRINT(tri_mb[i].v0);
		    PRINT(tri_mb[i].v1);
		    PRINT(tri_mb[i].v2);
		    PRINT(tri_mb[i].dv0);
		    PRINT(tri_mb[i].dv1);
		    PRINT(tri_mb[i].dv2);
		    PRINT(tri_mb[i].geomID());
		    PRINT(tri_mb[i].primID());		    
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

	const size_t totalBytesAllocated = bvh->alloc.getUsedBytes();
	PRINT(totalBytesAllocated);

#if defined(EMBREE_DPCPP_SUPPORT)
	DeviceGPU* deviceGPU = (DeviceGPU*)scene->device;

	const uint leaf_primitive_size = sizeof(Triangle1vMB);
	const uint node_size       = totalBytesAllocated;
	const uint leaf_size       = numPrimitives * leaf_primitive_size;
	const uint totalSize       = sizeof(gpu::BVHBase) + node_size + leaf_size; 
	const uint node_data_start = sizeof(gpu::BVHBase);
	const uint leaf_data_start = sizeof(gpu::BVHBase) + node_size;
	
	char *bvh_mem = (char*)cl::sycl::aligned_alloc(64,totalSize,deviceGPU->getGPUDevice(),deviceGPU->getGPUContext(),cl::sycl::usm::alloc::shared);
	assert(bvh_mem);

	std::atomic<size_t> gpu_node_allocator;
	std::atomic<size_t> gpu_leaf_allocator;
	
	gpu_node_allocator.store(node_data_start);
	gpu_leaf_allocator.store(0);

	convertToGPULayout(bvh->root,bvh_mem,(gpu::Triangle1vMB*)(bvh_mem+leaf_data_start),0,0,gpu_node_allocator,gpu_leaf_allocator);

	assert(gpu_node_allocator.load() <= leaf_data_start);
	assert(gpu_leaf_allocator.load() == numPrimitives);
	
	PRINT("BVH MB BUILDING DONE");      
	exit(0);	
#endif	

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
    Builder* BVHGPUTriangle1vMBSceneBuilderSAH (void* bvh, Scene* scene) { return new BVHGPUBuilderMBlurSAH<4,TriangleMesh,Triangle4vMB>((BVH4*)bvh,scene,4,1.0f,4,inf,TriangleMesh::geom_type); }
#endif

#if defined(EMBREE_GEOMETRY_QUAD)
    Builder* BVHGPUQuad1vMBSceneBuilderSAH (void* bvh, Scene* scene) { return new BVHGPUBuilderMBlurSAH<4,QuadMesh,Quad4i>((BVH4*)bvh,scene,4,1.0f,4,inf,QuadMesh::geom_type); }    
#endif
    
  } 
}
