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

      // FIXME: shrink bvh->alloc in destructor here and in other builders too

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
	    cl::sycl::buffer<gpu::Globals> globals_buffer(1);	    
	    
	    /* --- init globals --- */
	    {
	      const cl::sycl::nd_range<1> nd_range1(cl::sycl::range<1>(1),cl::sycl::range<1>(1));      
	      cl::sycl::event queue_event =  gpu_queue.submit([&](cl::sycl::handler &cgh) {
		  auto accessor_globals = globals_buffer.get_access<cl::sycl::access::mode::read_write>(cgh);
		  auto accessor_bvh = bvh_buffer.get_access<cl::sycl::access::mode::read_write>(cgh);

		  cgh.single_task<class init_first_kernel>([=]() {
		      gpu::Globals *g  = accessor_globals.get_pointer();
		      char *bvh_mem    = accessor_bvh.get_pointer();
		      g->init(bvh_mem,numPrimitives,node_data_start,leaf_data_start,totalSize);
		      g->geometryBounds.print();
		    });
		});
	      queue_event.wait();
	    }
	    	    
	    PRINT(deviceGPU->getMaxWorkGroupSize());
	    {
	      const int sizeWG = deviceGPU->getMaxWorkGroupSize();
	      const cl::sycl::nd_range<1> nd_range1(cl::sycl::range<1>((int)pinfo.size()),cl::sycl::range<1>(sizeWG));	      
	      
	      cl::sycl::event queue_event =  gpu_queue.submit([&](cl::sycl::handler &cgh) {

		  auto accessor_aabb    = aabb_buffer.get_access<cl::sycl::access::mode::read>(cgh);
		  auto accessor_globals = globals_buffer.get_access<cl::sycl::access::mode::read_write>(cgh);
		  
		  cgh.parallel_for<class init_bounds>(nd_range1,[=](cl::sycl::nd_item<1> item)
		                                     {//kernel code
						       gpu::AABB aabb         = accessor_aabb[item.get_global_id(0)];
						       gpu::AABB reduced_aabb = gpu::AABB::work_group_reduce(aabb);
						       cl::sycl::multi_ptr<gpu::Globals,cl::sycl::access::address_space::global_space> ptr(accessor_globals.get_pointer());
						       reduced_aabb.atomic_merge_global(&ptr.get()->geometryBounds);
						       if (item.get_global_id(0) == 0)
							 accessor_globals.get_pointer()->geometryBounds.print();
						     });//end of parallel_for
		  
		});
	      queue_event.wait();
	    }
	    
	    //printf("geometryBounds "); globals.geometryBounds.print();
	    //printf("centroidBounds "); globals.centroidBounds.print();

	      //gpu_queue.wait_and_throw();
	    
	    
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
