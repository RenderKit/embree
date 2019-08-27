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
#include "../builders/primrefgen.h"
#include "../builders/splitter.h"
#include "../geometry/triangle1v.h"


#include "../common/state.h"
#include "../../common/algorithms/parallel_for_for.h"
#include "../../common/algorithms/parallel_for_for_prefix_sum.h"

#include "../gpu/AABB.h"
#include "../gpu/AABB3f.h"
#include "../gpu/builder.h"

#define PROFILE 0
#define PROFILE_RUNS 20

namespace embree
{
#if defined(EMBREE_DPCPP_SUPPORT)

  inline cl::sycl::float4 Vec3fa_to_float4(const Vec3fa& v)
  {
    return (cl::sycl::float4){v.x,v.y,v.z,v.w};
  }
  
  [[cl::intel_reqd_sub_group_size(BVH_NODE_N)]] inline void atomicUpdateLocalBinInfo(cl::sycl::intel::sub_group &subgroup, const gpu::BinMapping &binMapping, gpu::BinInfo &binInfo, const gpu::AABB &primref,const cl::sycl::stream &out)
  {
    const cl::sycl::float4 p = primref.centroid2();
    const cl::sycl::float4 bin4 = (p-binMapping.ofs)*binMapping.scale;
    const cl::sycl::uint4 i = bin4.convert<cl::sycl::uint,cl::sycl::rounding_mode::rtz>();
  
    gpu::AABB3f bounds = convert_AABB3f(primref);

    bounds.atomic_merge_local(binInfo.boundsX[i.x()]);
    bounds.atomic_merge_local(binInfo.boundsY[i.y()]);
    bounds.atomic_merge_local(binInfo.boundsZ[i.z()]);

    gpu::atomic_add<uint,cl::sycl::access::address_space::local_space>((uint *)&binInfo.counts[i.x()] + 0,1);
    gpu::atomic_add<uint,cl::sycl::access::address_space::local_space>((uint *)&binInfo.counts[i.y()] + 1,1);
    gpu::atomic_add<uint,cl::sycl::access::address_space::local_space>((uint *)&binInfo.counts[i.z()] + 2,1);        
  }


  
  [[cl::intel_reqd_sub_group_size(BVH_NODE_N)]] inline void serial_find_split(cl::sycl::intel::sub_group &subgroup,
									      const gpu::BuildRecord &record,
									      const gpu::AABB *const primref,
									      gpu::BinMapping &binMapping,
									      gpu::BinInfo &binInfo,
									      uint *primref_index0,
									      uint *primref_index1,
									      const cl::sycl::stream &out)
  {
    const uint startID = record.start;
    const uint endID   = record.end;
  
    binInfo.init(subgroup);

    const uint subgroupLocalID = subgroup.get_local_id()[0];
    const uint subgroupSize    = subgroup.get_local_range().size();

    for (uint t=startID+subgroupLocalID;t<endID;t+=subgroupSize)
      {
	const uint index = primref_index0[t];
	
	primref_index1[t] = index;	
	atomicUpdateLocalBinInfo(subgroup,binMapping,binInfo,primref[index],out);      
      }
  }


  inline bool is_left(const gpu::BinMapping &binMapping, const gpu::Split &split, const gpu::AABB &primref)
  {
    const uint   dim  = split.dim;    
#if 1 
    const float lower = ((float*)&primref.lower)[dim]; // FIXME    
    const float upper = ((float*)&primref.upper)[dim];
    const float c     = lower+upper;
    const uint pos    = (uint)cl::sycl::floor((c-((float*)&binMapping.ofs)[dim])*((float*)&binMapping.scale)[dim]); // FIXME
#else
    // ORG OCL CODE    
    const float lower = primref.lower[dim];    
    const float upper = primref.upper[dim];
    const float c     = lower+upper;
    const uint pos    = (uint)cl::sycl::floor((c-binMapping.ofs[dim])*binMapping.scale[dim]);
#endif
    return pos < split.pos;    
  }


  [[cl::intel_reqd_sub_group_size(BVH_NODE_N)]] inline void serial_partition_index(cl::sycl::intel::sub_group &sg,
										   const gpu::AABB *const primref,
										   const gpu::BinMapping &binMapping,
										   const gpu::BuildRecord &current,
										   gpu::Split &split,
										   gpu::BuildRecord &outLeft,
										   gpu::BuildRecord &outRight,
										   gpu::AABB &outGeometryBoundsLeft,
										   gpu::AABB &outGeometryBoundsRight,
										   uint *primref_index0,
										   uint *primref_index1,
										   const cl::sycl::stream &out)
  {
    const uint subgroupLocalID = sg.get_local_id()[0];
    const uint subgroupSize    = sg.get_local_range().size();

    const uint start = current.start;
    const uint end   = current.end;  
    
    gpu::AABB leftCentroid;
    gpu::AABB rightCentroid;
  
    leftCentroid .init();
    rightCentroid.init();

    gpu::AABB leftAABB;
    gpu::AABB rightAABB;

    leftAABB.init();
    rightAABB.init();
  
    uint* l = primref_index0 + start;
    uint* r = primref_index0 + end;

    /* no valid split, just split in the middle */
    if (split.sah == (float)(INFINITY))
      {      
	for (uint i=start + subgroupLocalID;i<split.pos;i+=subgroupSize)
	  {
	    const uint index       = primref_index1[i];
	    const uint count       = sg.reduce<uint,cl::sycl::intel::plus>(1);
	    leftCentroid.extend(primref[index].centroid2());
	    leftAABB.extend(primref[index]);
	    l[subgroupLocalID] = index;	  
	    l+=count;
	  }

	for (uint i=split.pos + subgroupLocalID;i<end;i+=subgroupSize)
	  {
	    const uint index       = primref_index1[i];
	    const uint count       = sg.reduce<uint,cl::sycl::intel::plus>(1);
	    rightCentroid.extend(primref[index].centroid2());
	    rightAABB.extend(primref[index]);	  
	    r-=count;
	    r[subgroupLocalID] = index;	  
	  }      
      }
  
    else
      {
	for (uint i=start + subgroupLocalID;i<end;i+=subgroupSize)
	  {
	    const uint index       = primref_index1[i];
	    const uint isLeft      = is_left(binMapping, split,primref[index]) ? 1 : 0;
	    const uint isRight     = 1 - isLeft;
	    const uint countLeft   = sg.reduce<uint,cl::sycl::intel::plus>(isLeft );
	    const uint countRight  = sg.reduce<uint,cl::sycl::intel::plus>(isRight);
	    const uint prefixLeft  = sg.exclusive_scan<uint,cl::sycl::intel::plus>(isLeft);
	    const uint prefixRight = sg.exclusive_scan<uint,cl::sycl::intel::plus>(isRight);
          
	    r -= countRight;
      
	    if (isLeft)
	      {
		leftCentroid.extend(primref[index].centroid2());
		leftAABB.extend(primref[index]);	      
		l[prefixLeft] = index;
	      }
	    else
	      {
		rightCentroid.extend(primref[index].centroid2());
		rightAABB.extend(primref[index]);	  	      
		r[prefixRight] = index;
	      }
	    l += countLeft;
	  }
      }

    leftCentroid  = leftCentroid.sub_group_reduce(sg);
    rightCentroid = rightCentroid.sub_group_reduce(sg);
    leftAABB  = leftAABB.sub_group_reduce(sg);
    rightAABB = rightAABB.sub_group_reduce(sg);
  
    if (subgroupLocalID == 0)
      {
	uint pos =  l - primref_index0;  // single lane needs to compute "pos"
	//out << "pos = " << pos << cl::sycl::endl;
	
	outLeft.init(start,pos,leftCentroid);
	outRight.init(pos,end,rightCentroid);
      
	const uint sizeLeft  = outLeft.size();
	const uint sizeRight = outRight.size();
            
	leftAABB.upper.w() = gpu::as_float(sizeLeft);
	rightAABB.upper.w() = gpu::as_float(sizeRight);

	outGeometryBoundsLeft   = leftAABB;
	outGeometryBoundsRight  = rightAABB;
      }
  
  }
  

  /* ======================================== */  
  /* === build bvh for single buildrecord === */
  /* ======================================== */
  
     
  [[cl::intel_reqd_sub_group_size(BVH_NODE_N)]] inline void bvh_build_serial(cl::sycl::intel::sub_group &subgroup,
									     gpu::BuildRecord &record,
									     gpu::Globals &globals,
									     char *bvh_mem,
									     const gpu::AABB *const primref,
									     uint *primref_index0,
									     uint *primref_index1,
									     gpu::BinInfo &binInfo,
									     gpu::BuildRecord &current,
									     gpu::BuildRecord &brecord,
									     gpu::AABB childrenAABB[BVH_NODE_N],
									     gpu::BuildRecord stack[BUILDRECORD_STACK_SIZE],
									     const cl::sycl::stream &out)
  {
    const uint subgroupLocalID = subgroup.get_local_id()[0];
    //const uint subgroupSize    = subgroup.get_local_range().size();

    
    const uint cfg_minLeafSize = BVH_LEAF_N_MIN;

    uint sindex = 1;
    stack[0] = record;
        
    while(sindex) 
      {
	/* next element from stack */
	sindex--;      
	current = stack[sindex];

	if (subgroupLocalID == 0)
	  out << "sindex = " << sindex << " current " << current << cl::sycl::endl;
	
	gpu::BinMapping binMapping;

	const uint items = current.size();
	gpu::Split split;
	
	/*! create a leaf node when #items < threshold */
	if (items <= cfg_minLeafSize)
	  {
	    if (subgroup.get_local_id() == 0)
	      {
		const uint leaf_offset = createLeaf(globals,current.start,items,sizeof(gpu::Quad1));
		out << "leaf_offset " << leaf_offset << cl::sycl::endl;
		*current.parent = gpu::encodeOffset(bvh_mem,current.parent,leaf_offset);
	      }
	  }
	else
	  {
		
	    uint numChildren = 2;
	    struct gpu::BuildRecord *children = &stack[sindex];

	    
	    binMapping.init(current.centroidBounds,BINS);
	    
	    serial_find_split(subgroup,current,primref,binMapping,binInfo,primref_index0,primref_index1,out);
	    
	    split = binInfo.reduceBinsAndComputeBestSplit16(subgroup,binMapping.scale,current.start,current.end,out);

#if 1
	    if (subgroup.get_local_id() == 0)
	      out << "split " << split << cl::sycl::endl;
#endif
	    
	    serial_partition_index(subgroup,primref,binMapping,current,split,children[0],children[1],childrenAABB[0],childrenAABB[1],primref_index0,primref_index1,out);
	    
	    while (numChildren < BVH_NODE_N)
	      {
		/*! find best child to split */
		float bestArea = -(float)INFINITY;
		int bestChild = -1;
		for (int i=0; i<numChildren; i++)
		  {
		    /* ignore leaves as they cannot get split */
		    if (children[i].size() <= cfg_minLeafSize) continue;

		    /* find child with largest surface area */
		    if (gpu::halfarea(childrenAABB[i].size()) > bestArea) {
		      bestChild = i;
		      bestArea = gpu::halfarea(childrenAABB[i].size());
		    }
		  }
		if (bestChild == -1) break;

		/* perform best found split */
		brecord = children[bestChild];
		gpu::BuildRecord &lrecord = children[bestChild];
		gpu::BuildRecord &rrecord = children[numChildren];	

		binMapping.init(brecord.centroidBounds,BINS);
		serial_find_split(subgroup,brecord,primref,binMapping,binInfo,primref_index0,primref_index1,out);
		split = binInfo.reduceBinsAndComputeBestSplit16(subgroup,binMapping.scale,brecord.start,brecord.end,out);

		if (subgroup.get_local_id() == 0)
		  out << "split " << split << cl::sycl::endl;
		
		serial_partition_index(subgroup,primref,binMapping,brecord,split,lrecord,rrecord,childrenAABB[bestChild],childrenAABB[numChildren],primref_index0,primref_index1,out);		    
		numChildren++;		
	      }

	    /* sort children based on range size */
	    const float _sortID = childrenAABB[subgroupLocalID].upper.w();
	    const uint sortID = gpu::as_uint(_sortID);
	    const uint numPrimsIDs = cl::sycl::select((uint)0,(sortID << BVH_NODE_N_LOG) | subgroupLocalID, (int)(subgroupLocalID < numChildren));
	    const uint IDs = subgroupLocalID; //sortBVHChildrenIDs(numPrimsIDs) & (BVH_NODE_N-1);
	    
	    uint node_offset = gpu::createNode(subgroup,globals,IDs,childrenAABB,numChildren,bvh_mem,out);

	    /* set parent pointer in child build records */
	    struct gpu::BVHNodeN *node = (struct gpu::BVHNodeN*)(bvh_mem + node_offset);
	    if (subgroupLocalID < numChildren)
	      {
		children[IDs].parent = ((uint *)&node->offset[0]) + subgroupLocalID;
	      }

	    /* update parent pointer */
	    *current.parent = gpu::encodeOffset(bvh_mem,current.parent,node_offset);

	    sindex += numChildren;
	  }
      }			      
  }
#endif

  namespace isa
  {    
    template<int N, typename Mesh, typename Primitive>
    struct BVHGPUBuilderSAH : public Builder
    {
      typedef BVHN<N> BVH;
      typedef typename BVHN<N>::NodeRef NodeRef;

      BVH* bvh;
      Scene* scene;
      Mesh* mesh;
      mvector<PrimRef> prims;
      GeneralBVHBuilder::Settings settings;
      bool primrefarrayalloc;

      BVHGPUBuilderSAH (BVH* bvh, Scene* scene, const size_t sahBlockSize, const float intCost, const size_t minLeafSize, const size_t maxLeafSize,
			const size_t mode, bool primrefarrayalloc = false)
        : bvh(bvh), scene(scene), mesh(nullptr), prims(scene->device,0),
          settings(sahBlockSize, minLeafSize, maxLeafSize, travCost, intCost, DEFAULT_SINGLE_THREAD_THRESHOLD), primrefarrayalloc(primrefarrayalloc) {}

      BVHGPUBuilderSAH (BVH* bvh, Mesh* mesh, const size_t sahBlockSize, const float intCost, const size_t minLeafSize, const size_t maxLeafSize, const size_t mode)
        : bvh(bvh), scene(nullptr), mesh(mesh), prims(bvh->device,0), settings(sahBlockSize, minLeafSize, maxLeafSize, travCost, intCost, DEFAULT_SINGLE_THREAD_THRESHOLD), primrefarrayalloc(false) {}

      void build()
      {
        /* we reset the allocator when the mesh size changed */
        if (mesh && mesh->numPrimitivesChanged) {
          bvh->alloc.clear();
        }

        /* if we use the primrefarray for allocations we have to take it back from the BVH */
        if (settings.primrefarrayalloc != size_t(inf))
          bvh->alloc.unshare(prims);

	/* skip build for empty scene */
        const size_t numPrimitives = mesh ? mesh->size() : scene->getNumPrimitives<Mesh,false>();
        if (numPrimitives == 0) {
          bvh->clear();
          prims.clear();
          return;
        }

        double t0 = bvh->preBuild(mesh ? "" : TOSTRING(isa) "::BVH" + toString(N) + "BuilderSAH");

#if PROFILE
        profile(2,PROFILE_RUNS,numPrimitives,[&] (ProfileTimer& timer) {
#endif

            /* create primref array */
            if (primrefarrayalloc) {
              settings.primrefarrayalloc = numPrimitives/1000;
              if (settings.primrefarrayalloc < 1000)
                settings.primrefarrayalloc = inf;
            }

            /* enable os_malloc for two level build */
            if (mesh)
              bvh->alloc.setOSallocation(true);

            /* initialize allocator */
            const size_t node_bytes = numPrimitives*sizeof(typename BVH::AlignedNodeMB)/(4*N);
            const size_t leaf_bytes = 64;
            bvh->alloc.init_estimate(node_bytes+leaf_bytes);
            settings.singleThreadThreshold = bvh->alloc.fixSingleThreadThreshold(N,DEFAULT_SINGLE_THREAD_THRESHOLD,numPrimitives,node_bytes+leaf_bytes);
            prims.resize(numPrimitives); 

            PrimInfo pinfo = mesh ?
              createPrimRefArray(mesh,prims,bvh->scene->progressInterface) :
              createPrimRefArray(scene,Mesh::geom_type,false,prims,bvh->scene->progressInterface);


	    
	    
            /* pinfo might has zero size due to invalid geometry */
            if (unlikely(pinfo.size() == 0))
	      {
		bvh->clear();
		prims.clear();
		return;
	      }
	    PRINT(pinfo.size());

#if defined(EMBREE_DPCPP_SUPPORT)
	      
	    DeviceGPU* deviceGPU = (DeviceGPU*)scene->device;
	    cl::sycl::queue &gpu_queue = deviceGPU->getQueue();
	    
	    PRINT(deviceGPU->getMaxWorkGroupSize());

	    /* --- estimate size of the BVH --- */
	    unsigned int totalSize       = 64 + numPrimitives * 2 * 64;
	    unsigned int node_data_start = 64;
	    unsigned int leaf_data_start = numPrimitives * 64;

	    /* --- allocate buffers --- */

	    gpu::AABB *aabb = (gpu::AABB*)cl::sycl::aligned_alloc(64,sizeof(gpu::AABB)*numPrimitives,deviceGPU->getDevice(),deviceGPU->getContext(),cl::sycl::usm::alloc::shared);
	    assert(aabb);

	    char *bvh_mem = (char*)cl::sycl::aligned_alloc(64,totalSize,deviceGPU->getDevice(),deviceGPU->getContext(),cl::sycl::usm::alloc::shared);
	    assert(bvh_mem);

	    uint *primref_index = (uint*)cl::sycl::aligned_alloc(64,sizeof(uint)*2*numPrimitives,deviceGPU->getDevice(),deviceGPU->getContext(),cl::sycl::usm::alloc::shared);
	    assert(primref_index);

	    gpu::Globals *globals = (gpu::Globals*)cl::sycl::aligned_alloc(64,sizeof(gpu::Globals),deviceGPU->getDevice(),deviceGPU->getContext(),cl::sycl::usm::alloc::shared);
	    assert(globals);
	    
	    /* copy primrefs for now */
	    
	    for (size_t i=0;i<numPrimitives;i++)
	      ((PrimRef*)aabb)[i] = prims[i];
	    	    
	    /* --- init globals --- */
	    {
	      cl::sycl::event queue_event =  gpu_queue.submit([&](cl::sycl::handler &cgh) {
		  cgh.single_task<class init_first_kernel>([=]() {
		      globals->init(bvh_mem,numPrimitives,node_data_start,leaf_data_start,totalSize);
		    });
		});
	      try {
		gpu_queue.wait_and_throw();
	      } catch (cl::sycl::exception const& e) {
		std::cout << "Caught synchronous SYCL exception:\n"
			  << e.what() << std::endl;
	      }
	    }
	    
	    const int sizeWG = deviceGPU->getMaxWorkGroupSize();
	    //const cl::sycl::nd_range<1> nd_range1(cl::sycl::range<1>((int)pinfo.size()),cl::sycl::range<1>(sizeWG));
	    const cl::sycl::nd_range<1> nd_range1(cl::sycl::range<1>((int)sizeWG),cl::sycl::range<1>(sizeWG));	    
	    {	      
	      cl::sycl::event queue_event = gpu_queue.submit([&](cl::sycl::handler &cgh) {
		  cl::sycl::stream out(DBG_PRINT_BUFFER_SIZE, DBG_PRINT_LINE_SIZE, cgh);
		  cgh.parallel_for<class init_bounds0>(nd_range1,[=](cl::sycl::nd_item<1> item)
		{
		  const uint startID = item.get_global_id(0);
		  const uint step    = item.get_global_range().size();

		  gpu::AABB local_geometry_aabb;
		  gpu::AABB local_centroid_aabb;
		  local_geometry_aabb.init();
		  local_centroid_aabb.init();
		  
		  for (uint i=startID;i<numPrimitives;i+=step)
		    {
		      const gpu::AABB aabb_geom = aabb[i];
		      const gpu::AABB aabb_centroid(aabb_geom.centroid2());
		      primref_index[i] = i;
		      local_geometry_aabb.extend(aabb_geom);
		      local_centroid_aabb.extend(aabb_centroid);		      
		    }
		  cl::sycl::multi_ptr<gpu::Globals,cl::sycl::access::address_space::global_space> ptr(globals);
		  local_geometry_aabb.atomic_merge_global(ptr.get()->geometryBounds);
		  local_centroid_aabb.atomic_merge_global(ptr.get()->centroidBounds);		  
		});
		  
		});
	      try {
		gpu_queue.wait_and_throw();
	      } catch (cl::sycl::exception const& e) {
		std::cout << "Caught synchronous SYCL exception:\n"
			  << e.what() << std::endl;
	      }
	    }

	    /* --- init bvh sah builder --- */
	    {
	      cl::sycl::event queue_event = gpu_queue.submit([&](cl::sycl::handler &cgh) {
		  cl::sycl::stream out(DBG_PRINT_BUFFER_SIZE, DBG_PRINT_LINE_SIZE, cgh);
		  cgh.single_task<class init_builder>([=]() {
		      gpu::BuildRecord *record = (gpu::BuildRecord*)(bvh_mem + globals->leaf_mem_allocator_start);
		      record->init(0,numPrimitives,globals->centroidBounds);
		      globals->numBuildRecords = 1;
		      out << "geometryBounds: " << globals->geometryBounds << cl::sycl::endl;
		      out << "centroiBounds : " << globals->centroidBounds << cl::sycl::endl;
		    });
		});
	      try {
		gpu_queue.wait_and_throw();
	      } catch (cl::sycl::exception const& e) {
		std::cout << "Caught synchronous SYCL exception:\n"
			  << e.what() << std::endl;
	      }
	    }
	    

	    /* --- single HW thread recursive build --- */
	    {
	      cl::sycl::event queue_event = gpu_queue.submit([&](cl::sycl::handler &cgh) {

		  cl::sycl::stream out(DBG_PRINT_BUFFER_SIZE, DBG_PRINT_LINE_SIZE, cgh);
		  const cl::sycl::nd_range<1> nd_range(cl::sycl::range<1>(BVH_NODE_N),cl::sycl::range<1>(BVH_NODE_N));
		  
		  /* local variables */
		  cl::sycl::accessor< gpu::BinInfo    , 0, sycl_read_write, sycl_local> binInfo(cgh);
		  cl::sycl::accessor< gpu::BuildRecord, 0, sycl_read_write, sycl_local> current(cgh);
		  cl::sycl::accessor< gpu::BuildRecord, 0, sycl_read_write, sycl_local> brecord(cgh);
		  cl::sycl::accessor< gpu::AABB       , 1, sycl_read_write, sycl_local> childrenAABB(cl::sycl::range<1>(BVH_NODE_N),cgh);
		  cl::sycl::accessor< gpu::BuildRecord, 1, sycl_read_write, sycl_local> stack(cl::sycl::range<1>(BUILDRECORD_STACK_SIZE),cgh);
		  		  		  
		  cgh.parallel_for<class serial_build>(nd_range,[=](cl::sycl::nd_item<1> item) {
		      const uint groupID   = item.get_group(0);
		      const uint numGroups = item.get_group_range(0);
		      cl::sycl::intel::sub_group subgroup = item.get_sub_group();
		      
		      gpu::AABB *primref     = aabb;
		      uint *primref_index0   = primref_index + 0;
		      uint *primref_index1   = primref_index + globals->numPrimitives;		      
		      gpu::BuildRecord *record = (gpu::BuildRecord*)(bvh_mem + globals->leaf_mem_allocator_start);
		      
		      const uint numRecords = globals->numBuildRecords;
		      
		      for (uint recordID = groupID;recordID<numRecords;recordID+=numGroups)
			bvh_build_serial(subgroup,record[recordID],*globals,bvh_mem,primref,primref_index0,primref_index1,binInfo,current,brecord,childrenAABB.get_pointer(),stack.get_pointer(),out);
		    });		  
		});
	      try {
		gpu_queue.wait_and_throw();
	      } catch (cl::sycl::exception const& e) {
		std::cout << "Caught synchronous SYCL exception:\n"
			  << e.what() << std::endl;
	      }
	    }

	    /* --- convert primrefs to primitives --- */
#if 0	    
	    for (size_t i=0;i<numPrimitives;i++)
	      std::cout << "i " << i << " index " << primref_index[i] << " aabb[primref_index[i]] " << ((PrimRef*)aabb)[primref_index[i]] << std::endl;
#endif

	    gpu::Quad1 *quad1 = (gpu::Quad1 *)(bvh_mem + globals->leaf_mem_allocator_start);
	    PRINT(globals->leaf_mem_allocator_start);
	    for (size_t i=0;i<numPrimitives;i++)
	      {
		const uint index = primref_index[i];
		const uint geomID = gpu::as_uint((float)aabb[index].lower.w());
		const uint primID = gpu::as_uint((float)aabb[index].upper.w());
		//std::cout << " i " << i << " index " << index << " geomID " << geomID << " primID " << primID << std::endl;
		TriangleMesh* mesh = (TriangleMesh*)scene->get(geomID);
		const TriangleMesh::Triangle &tri = mesh->triangle(primID);
		const Vec3fa v0 = mesh->vertex(tri.v[0]);
		const Vec3fa v1 = mesh->vertex(tri.v[1]);
		const Vec3fa v2 = mesh->vertex(tri.v[2]);

		quad1[i].init(Vec3fa_to_float4(v0),
			      Vec3fa_to_float4(v1),
			      Vec3fa_to_float4(v2),
			      Vec3fa_to_float4(v2),
			      geomID,
			      primID,
			      primID);
		
		// PRINT(mesh);
		// PRINT(v0);
		// PRINT(v1);
		// PRINT(v2);
		
	      }

	    /* --- deallocate temporary data structures --- */
	    cl::sycl::free(aabb         ,deviceGPU->getContext());
	    cl::sycl::free(primref_index,deviceGPU->getContext());
	    cl::sycl::free(globals      ,deviceGPU->getContext());

	    std::cout << "BVH GPU Builder DONE" << std::endl << std::flush;
	    
            /* call BVH builder */
            NodeRef root(0); // = BVHNBuilderVirtual<N>::build(&bvh->alloc,CreateLeaf<N,Primitive>(bvh),bvh->scene->progressInterface,prims.data(),pinfo,settings);

#endif	    
            //bvh->set(root,LBBox3fa(pinfo.geomBounds),pinfo.size());
	    bvh->clear();
#if PROFILE
          });
#endif

        /* if we allocated using the primrefarray we have to keep it alive */
        if (settings.primrefarrayalloc != size_t(inf))
          bvh->alloc.share(prims);

        /* for static geometries we can do some cleanups */
        else if (scene && scene->isStaticAccel()) {
          bvh->shrink();
          prims.clear();
        }
	bvh->cleanup();
        bvh->postBuild(t0);
      }

      void clear() {
        prims.clear();
      }
    };

    /************************************************************************************/
    /************************************************************************************/
    /************************************************************************************/
    /************************************************************************************/
#if defined(EMBREE_GEOMETRY_TRIANGLE)
    Builder* BVHGPUTriangle1vSceneBuilderSAH (void* bvh, Scene* scene, size_t mode) { return new BVHGPUBuilderSAH<4,TriangleMesh,Triangle1v>((BVH4*)bvh,scene,1,1.0f,1,inf,mode,true); }
#endif
    
  }
}
