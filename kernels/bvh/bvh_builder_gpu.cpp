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

  [[cl::intel_reqd_sub_group_size(BVH_NODE_N)]] inline void atomicUpdateLocalBinInfo(const gpu::BinMapping &binMapping, gpu::BinInfo &binInfo, const gpu::AABB &primref)
  {
    const cl::sycl::float4 p = primref.centroid2();
    const cl::sycl::float4 bin4 = (p-binMapping.ofs)*binMapping.scale;
    cl::sycl::uint4 i;

#if 1
    i.x() = (uint)bin4.x();
    i.y() = (uint)bin4.y();
    i.z() = (uint)bin4.z();
#else
    i = bin4.convert<cl::sycl::uint,cl::sycl::rounding_mode::rtz>();
#endif
    
    assert(i.x() < BINS);
    assert(i.y() < BINS);
    assert(i.z() < BINS);
  
    gpu::AABB3f bounds = convert_AABB3f(primref);
    bounds.atomic_merge_local(binInfo.boundsX[i.x()]);
    bounds.atomic_merge_local(binInfo.boundsX[i.y()]);
    bounds.atomic_merge_local(binInfo.boundsX[i.z()]);

    gpu::atomic_add<uint,cl::sycl::access::address_space::local_space>((uint *)&binInfo.counts[i.x()] + 0,1);
    gpu::atomic_add<uint,cl::sycl::access::address_space::local_space>((uint *)&binInfo.counts[i.y()] + 1,1);
    gpu::atomic_add<uint,cl::sycl::access::address_space::local_space>((uint *)&binInfo.counts[i.z()] + 2,1);        
  }


  
  [[cl::intel_reqd_sub_group_size(BVH_NODE_N)]] inline void serial_find_split(cl::sycl::intel::sub_group &subgroup,
									      const gpu::BuildRecord &record,
									      const gpu::AABB *const primref,
									      gpu::BinMapping &binMapping,			      
									      gpu::Split &split,
									      gpu::BinInfo &binInfo,
									      uint *primref_index0,
									      uint *primref_index1)
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
	atomicUpdateLocalBinInfo(binMapping,binInfo,primref[index]);      
      }
  }


  inline bool is_left(const gpu::BinMapping &binMapping, const gpu::Split &split, const gpu::AABB &primref)
  {
#if 1    
    const uint   dim  = split.dim;
    const float lower = ((float*)&primref.lower)[dim]; // FIXME    
    const float upper = ((float*)&primref.upper)[dim];
    const float c     = lower+upper;
    const uint pos    = (uint)cl::sycl::floor((c-((float*)&binMapping.ofs)[dim])*((float*)&binMapping.scale)[dim]); // FIXME
#else
    // ORG OCL CODE    
    const float lower = primref.lower[dim];    
    const float upper = primref.upper[dim];
    const float c     = lower+upper;
    const uint pos    = convert_uint_rtz((c-binMapping.ofs[dim])*binMapping.scale[dim]);
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
										   uint *primref_index1)
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
	  //extendBinBounds(&left,&primref[index]);
	  //extendAABBlu(&leftAABB,primref[index].lower,primref[index].upper);
	  l[subgroupLocalID] = index;	  
	  l+=count;
	}

      for (uint i=split.pos + subgroupLocalID;i<end;i+=subgroupSize)
	{
	  const uint index       = primref_index1[i];
	  const uint count       = sg.reduce<uint,cl::sycl::intel::plus>(1);
	  rightCentroid.extend(primref[index].centroid2());
	  rightAABB.extend(primref[index]);	  
	  //extendBinBounds(&right,&primref[index]);
	  //extendAABBlu(&rightAABB,primref[index].lower,primref[index].upper);
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
	      // extendBinBounds(&left,&primref[index]);
	      // extendAABBlu(&leftAABB,primref[index].lower,primref[index].upper);
	      leftCentroid.extend(primref[index].centroid2());
	      leftAABB.extend(primref[index]);	      
	      l[prefixLeft] = index;
	    }
	  else
	    {
	      // extendBinBounds(&right,&primref[index]);
	      // extendAABBlu(&rightAABB,primref[index].lower,primref[index].upper);
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
									     gpu::Split &split,
									     gpu::AABB childrenAABB[BVH_NODE_N],
									     gpu::BuildRecord stack[BUILDRECORD_STACK_SIZE])
  {
    const uint subgroupLocalID = subgroup.get_local_id()[0];
    const uint subgroupSize    = subgroup.get_local_range().size();
    
    const uint cfg_minLeafSize = BVH_LEAF_N_MIN;
    
    uint sindex = 1;
    stack[0] = record;	  

    record.print();
    
    while(sindex) 
      {
	/* next element from stack */
	sindex--;      
	current = stack[sindex];
	gpu::BinMapping binMapping;

	const uint items = current.size();
	  
	    /*! create a leaf node when #items < threshold */
	    if (items <= cfg_minLeafSize)
	      {
		if (subgroup.get_local_id() == 0)
		  {
		    const uint leaf_offset = createLeaf(globals,current.start,items,sizeof(gpu::Quad1));  
		    *current.parent = gpu::encodeOffset(bvh_mem,current.parent,leaf_offset);
		  }
	      }
	    else
	      {
		
		uint numChildren = 2;
		struct gpu::BuildRecord *children = &stack[sindex];
		binMapping.init(current.centroidBounds,BINS);
		serial_find_split(subgroup,current,primref,binMapping,split,binInfo,primref_index0,primref_index1);
		split = binInfo.reduceBinsAndComputeBestSplit16(subgroup,binMapping.scale,current.start,current.end);	      
		serial_partition_index(subgroup,primref,binMapping,current,split,children[0],children[1],childrenAABB[0],childrenAABB[1],primref_index0,primref_index1);

			      	      
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
		    serial_find_split(subgroup,brecord,primref,binMapping,split,binInfo,primref_index0,primref_index1);
		    split = binInfo.reduceBinsAndComputeBestSplit16(subgroup,binMapping.scale,brecord.start,brecord.end);	      
		    serial_partition_index(subgroup,primref,binMapping,brecord,split,lrecord,rrecord,childrenAABB[bestChild],childrenAABB[numChildren],primref_index0,primref_index1);		    
		    numChildren++;
		  }

		/* sort children based on range size */
		const float _sortID = childrenAABB[subgroupLocalID].upper.w();
		const uint sortID = gpu::as_uint(_sortID);
		const uint numPrimsIDs = select((uint)0,(sortID << BVH_NODE_N_LOG) | subgroupLocalID, subgroupLocalID < numChildren);
		const uint IDs = subgroupLocalID; //sortBVHChildrenIDs(numPrimsIDs) & (BVH_NODE_N-1);

		uint node_offset = gpu::createNode(subgroup,globals,IDs,childrenAABB,numChildren,bvh_mem);
	  
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

	    /* --- estimate size of the BVH --- */
	    unsigned int totalSize       = 64 + numPrimitives * 2 * 64;
	    unsigned int node_data_start = 64;
	    unsigned int leaf_data_start = numPrimitives * 64;

	    /* --- allocate buffers --- */
	    
	    cl::sycl::buffer<char> bvh_buffer(totalSize);
	    cl::sycl::buffer<gpu::AABB> aabb_buffer((gpu::AABB*)prims.data(),numPrimitives);
	    cl::sycl::buffer<uint> primref_index(2*numPrimitives);	    
	    cl::sycl::buffer<gpu::Globals> globals_buffer(1);	    
	    
	    /* --- init globals --- */
	    {
	      cl::sycl::event queue_event =  gpu_queue.submit([&](cl::sycl::handler &cgh) {
		  auto accessor_globals = globals_buffer.get_access<sycl_read_write>(cgh);
		  auto accessor_bvh     = bvh_buffer.get_access<sycl_read_write>(cgh);
		  cgh.single_task<class init_first_kernel>([=]() {
		      gpu::Globals *g  = accessor_globals.get_pointer();
		      char *bvh_mem    = accessor_bvh.get_pointer();
		      g->init(bvh_mem,numPrimitives,node_data_start,leaf_data_start,totalSize);
		    });
		});
	      queue_event.wait();
	    }
	    
	    PRINT(deviceGPU->getMaxWorkGroupSize());
	    const int sizeWG = deviceGPU->getMaxWorkGroupSize();
	    const cl::sycl::nd_range<1> nd_range1(cl::sycl::range<1>((int)pinfo.size()),cl::sycl::range<1>(sizeWG));	      	    
	    {
	      
	      cl::sycl::event queue_event = gpu_queue.submit([&](cl::sycl::handler &cgh) {
		  auto accessor_globals = globals_buffer.get_access<sycl_read_write>(cgh);		  
		  auto accessor_aabb    = aabb_buffer.get_access<sycl_read>(cgh);		  
		  cgh.parallel_for<class init_bounds0>(nd_range1,[=](cl::sycl::nd_item<1> item)
		                                     {
						       const gpu::AABB aabb_geom = accessor_aabb[item.get_global_id(0)];
						       const gpu::AABB aabb_centroid(aabb_geom.centroid2());						       
						       const gpu::AABB reduced_geometry_aabb = aabb_geom.work_group_reduce();
						       const gpu::AABB reduced_centroid_aabb = aabb_centroid.work_group_reduce();						       
						       cl::sycl::multi_ptr<gpu::Globals,cl::sycl::access::address_space::global_space> ptr(accessor_globals.get_pointer());
						       reduced_geometry_aabb.atomic_merge_global(ptr.get()->geometryBounds);
						       reduced_centroid_aabb.atomic_merge_global(ptr.get()->centroidBounds);						       
						     });
		  
		});
	      queue_event.wait();
	    }

	    /* --- init bvh sah builder --- */
	    {
	      cl::sycl::event queue_event = gpu_queue.submit([&](cl::sycl::handler &cgh) {
		  auto accessor_globals = globals_buffer.get_access<sycl_read_write>(cgh);
		  auto accessor_bvh     = bvh_buffer.get_access<sycl_read_write>(cgh);
		  cgh.single_task<class init_builder>([=]() {
		      gpu::Globals *globals    = accessor_globals.get_pointer();
		      char *bvh_mem            = accessor_bvh.get_pointer();
		      gpu::BuildRecord *record = (gpu::BuildRecord*)(bvh_mem + globals->leaf_mem_allocator_start);
		      record->init(0,numPrimitives,globals->centroidBounds);
		      globals->numBuildRecords = 1;
		      globals->geometryBounds.print();
		      globals->centroidBounds.print();
		    });
		});
	      queue_event.wait();
	    }

	    

	    /* --- single HW thread recursive build --- */
	    {
	      cl::sycl::event queue_event = gpu_queue.submit([&](cl::sycl::handler &cgh) {
		  auto accessor_globals       = globals_buffer.get_access<sycl_read_write>(cgh);
		  auto accessor_bvh           = bvh_buffer.get_access<sycl_read_write>(cgh);
		  auto accessor_aabb          = aabb_buffer.get_access<sycl_read>(cgh);		  		  
		  auto accessor_primref_index = primref_index.get_access<sycl_read_write>(cgh);
		  const cl::sycl::nd_range<1> nd_range(cl::sycl::range<1>(BVH_NODE_N),cl::sycl::range<1>(BVH_NODE_N));
		  
		  /* local variables */
		  cl::sycl::accessor< gpu::BinInfo    , 0, sycl_read_write, sycl_local> binInfo(cgh);
		  cl::sycl::accessor< gpu::BuildRecord, 0, sycl_read_write, sycl_local> current(cgh);
		  cl::sycl::accessor< gpu::BuildRecord, 0, sycl_read_write, sycl_local> brecord(cgh);
		  cl::sycl::accessor< gpu::Split      , 0, sycl_read_write, sycl_local> split(cgh);		  
		  cl::sycl::accessor< gpu::AABB       , 1, sycl_read_write, sycl_local> childrenAABB(cl::sycl::range<1>(BVH_NODE_N),cgh);

		  cl::sycl::accessor< gpu::BuildRecord, 1, sycl_read_write, sycl_local> stack(cl::sycl::range<1>(BUILDRECORD_STACK_SIZE),cgh);
		  
		  		  
		  cgh.parallel_for<class serial_build>(nd_range,[=](cl::sycl::nd_item<1> item) {
		      const uint groupID   = item.get_group(0);
		      const uint numGroups = item.get_group_range(0);
		      cl::sycl::intel::sub_group subgroup = item.get_sub_group();
		      
		      //printf("groupID %d numGroups %d \n",groupID,numGroups);
		      
		      gpu::Globals *globals  = accessor_globals.get_pointer();
		      char *bvh_mem          = accessor_bvh.get_pointer();
		      gpu::AABB *primref     = accessor_aabb.get_pointer();
		      uint *primref_index0   = accessor_primref_index.get_pointer() + 0;
		      uint *primref_index1   = accessor_primref_index.get_pointer() + globals->numPrimitives;		      
		      gpu::BuildRecord *record = (gpu::BuildRecord*)(bvh_mem + globals->leaf_mem_allocator_start);
		      
		      const uint numRecords = globals->numBuildRecords;
		      
		      for (uint recordID = groupID;recordID<numRecords;recordID+=numGroups)
			bvh_build_serial(subgroup,record[recordID],*globals,bvh_mem,primref,primref_index0,primref_index1,binInfo,current,brecord,split,childrenAABB.get_pointer(),stack.get_pointer());
		    });
		});
	      queue_event.wait();
	    }
	    

	    //global struct BuildRecord *record = (global struct BuildRecord*)(bvh_mem + globals->leaf_mem_allocator[1]);

	    
            /* call BVH builder */
            NodeRef root(0); // = BVHNBuilderVirtual<N>::build(&bvh->alloc,CreateLeaf<N,Primitive>(bvh),bvh->scene->progressInterface,prims.data(),pinfo,settings);
	    PING;
	    //std::cout << "bounds " << (float)bounds.lower.x() << std::endl;
	    exit(0);

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
