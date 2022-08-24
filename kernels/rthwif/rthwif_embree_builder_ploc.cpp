#include "rthwif_embree.h"
#include "rthwif_embree_builder.h"
#include "rthwif_internal.h"
#include "rthwif_embree_builder_ploc.h"
#include "builder/qbvh6.h"

#define SINGLE_WG_SWITCH_THRESHOLD 8*1024
#define FAST_MC_THRESHOLD          1024*1024
#define MAX_WGS                    64
#define SMALL_SORT_THRESHOLD       1024*4

namespace embree
{
  using namespace embree::isa;
  
  
  void checkBVH2PlocHW(BVH2Ploc *bvh2, uint index,uint &nodes,uint &leaves,float &nodeSAH, float &leafSAH, const uint numPrimitives, const uint bvh2_max_allocations)
  {
    if (!bvh2[index].bounds.isValid()) {
      PRINT2(index,bvh2[index]);
      FATAL("invalid bounds in BVH2");
    }
    if (!bvh2[index].bounds.checkNumericalBounds())
    {
      PRINT2(index,bvh2[index]);      
      FATAL("Numerical Bounds in BVH2");
    }

    if (BVH2Ploc::getIndex(index) < numPrimitives) //isLeaf 
    {
      leaves++;
      leafSAH +=  bvh2[index].bounds.area();
      assert(bvh2[index].getLeafIndex() < numPrimitives);
    }
    else
    {
      uint indices[BVH_BRANCHING_FACTOR];
      const uint numChildren = openBVH2MaxAreaSortChildren(BVH2Ploc::getIndex(index),indices,bvh2);
      for (uint i=0;i<numChildren;i++)
        if (BVH2Ploc::getIndex(indices[i]) > bvh2_max_allocations)
          FATAL("OPENING ERROR");

      nodes++;              
      nodeSAH += bvh2[index].bounds.area();
      
      assert(bvh2[index].bounds.encloses( bvh2[ bvh2[index].leftIndex() ].bounds ));        
      checkBVH2PlocHW(bvh2,bvh2[index].leftIndex(),nodes,leaves,nodeSAH,leafSAH,numPrimitives,bvh2_max_allocations);

      assert(bvh2[index].bounds.encloses( bvh2[ bvh2[index].rightIndex() ].bounds ));        
      checkBVH2PlocHW(bvh2,bvh2[index].rightIndex(),nodes,leaves,nodeSAH,leafSAH,numPrimitives,bvh2_max_allocations);
    }
  }

  struct BuildTimer {
    enum Type {
      PRE_PROCESS  = 0,
      BUILD        = 1,
      POST_PROCESS = 2,
      ALLOCATION   = 3,            
      TOTAL        = 4
    };

    double host_timers[TOTAL];
    double device_timers[TOTAL];
    
    double t0,t1;

    inline void reset()
    {
      for (uint i=0;i<TOTAL;i++)
      {
        host_timers[i] = 0.0;
        device_timers[i] = 0.0;        
      }        
    }
    
    inline void start(const Type type)
    {
      t0 = getSeconds();
    }

    inline void stop(const Type type)
    {
      t1 = getSeconds();
      host_timers[(int)type] += 1000.0*(t1-t0);
    }

    inline void add_to_device_timer(const Type type, double t)
    {
      device_timers[(int)type] += t;
    }
    
    inline float get_accum_device_timer(const Type type) { return device_timers[(int)type]; }
    inline float get_accum_host_timer  (const Type type) { return host_timers[(int)type]; }    
    inline float get_host_timer() { return 1000.0*(t1-t0); }

    inline float get_total_device_time()
    {
      double sum = 0.0;
      for (uint i=0;i<ALLOCATION;i++) sum += device_timers[i];
      return sum;
    }

    inline float get_total_host_time()
    {
      double sum = 0.0;
      for (uint i=0;i<ALLOCATION;i++) sum += host_timers[i];
      return sum;
    }
    
    
  };
  
  BBox3fa rthwifBuildPloc(Scene* scene, RTCBuildQuality quality_flags, Device::avector<char,64>& accel, const bool two_level=false)
  {
    BuildTimer timer;
    timer.reset();
      
    DeviceGPU* deviceGPU = dynamic_cast<DeviceGPU*>(scene->device);
    assert(deviceGPU);
    const bool verbose = deviceGPU->verbosity(2);
    
    // ===============================================================================================================
  
    sycl::queue &gpu_queue = deviceGPU->getGPUQueue();

    const uint gpu_maxWorkGroupSize = deviceGPU->getGPUDevice().get_info<sycl::info::device::max_work_group_size>();
    const uint gpu_maxComputeUnits  = deviceGPU->getGPUDevice().get_info<sycl::info::device::max_compute_units>();    
    const uint gpu_maxLocalMemory   = deviceGPU->getGPUDevice().get_info<sycl::info::device::local_mem_size>();
    const uint gpu_maxSubgroups     = gpu_maxComputeUnits * 8;

    if (unlikely(deviceGPU->verbosity(2)))
    {    
      PRINT( deviceGPU->getGPUDevice().get_info<sycl::info::device::global_mem_size>() );
      PRINT(gpu_maxWorkGroupSize);
      PRINT(gpu_maxComputeUnits);
      PRINT(gpu_maxLocalMemory);
      PRINT(gpu_maxSubgroups);
    }

    const uint numGeoms = scene->size();
        
    // ===============================================================================================================
    
    size_t sizeTotalAllocations = 0;
    
    timer.start(BuildTimer::ALLOCATION);

    uint *host_device_tasks = (uint*)sycl::aligned_alloc(64,sizeof(uint)*4,deviceGPU->getGPUDevice(),deviceGPU->getGPUContext(),sycl::usm::alloc::host); // FIXME
    assert(host_device_tasks);

    PLOCGlobals *globals  = (PLOCGlobals*)sycl::aligned_alloc(64,sizeof(PLOCGlobals),deviceGPU->getGPUDevice(),deviceGPU->getGPUContext(),sycl::usm::alloc::shared);
    assert(globals);

    sizeTotalAllocations += sizeof(PLOCGlobals);
    
    const size_t alloc_TriMeshes = sizeof(TriMesh)*(numGeoms+1);
    const size_t alloc_GeomPrefixSums = sizeof(uint)*(numGeoms+1);

    char *tmpMem0 = (char*)sycl::aligned_alloc(64,alloc_GeomPrefixSums+alloc_TriMeshes,deviceGPU->getGPUDevice(),deviceGPU->getGPUContext(),sycl::usm::alloc::shared);
    gpu_queue.prefetch(tmpMem0,alloc_GeomPrefixSums+alloc_TriMeshes);

    sizeTotalAllocations += alloc_GeomPrefixSums+alloc_TriMeshes;
    
    //PRINT(alloc_GeomPrefixSums+alloc_TriMeshes);
    
    uint *const quads_per_geom_prefix_sum  = (uint*)tmpMem0;
    TriMesh *const triMesh                 = (TriMesh*)(tmpMem0 + alloc_GeomPrefixSums);
    
    timer.stop(BuildTimer::ALLOCATION);
	
    if (unlikely(deviceGPU->verbosity(2)))
      std::cout << "USM allocation time for globals, tri meshes and prefix sums " << timer.get_host_timer() << " ms for " << (double)(alloc_TriMeshes+alloc_GeomPrefixSums) / (1024*1024) << " MBs " << std::endl;     	

    // ==============================================
    // === compute prefix sum over geometry sizes === 
    // ==============================================
    
    uint org_numPrimitives = 0;
    for (uint  geomID = 0; geomID < numGeoms; geomID++)
    {
      const uint current = scene->get(geomID)->size();
      org_numPrimitives += current;

      // TODO: use BufferView
      TriangleMesh* mesh = scene->get<TriangleMesh>(geomID);
      triMesh[geomID].numTriangles = mesh->size();
      triMesh[geomID].numVertices  = mesh->numVertices();
      triMesh[geomID].triangles    =  (TriangleMesh::Triangle*)mesh->triangles.getPtr(); 
      triMesh[geomID].vertices     =  (Vec3fa*)mesh->vertices0.getPtr();

      gpu_queue.prefetch(triMesh[geomID].triangles, triMesh[geomID].numTriangles * sizeof(TriangleMesh::Triangle));
      gpu_queue.prefetch(triMesh[geomID].vertices , triMesh[geomID].numVertices * sizeof(Vec3fa));      
    }
    
    if (unlikely(deviceGPU->verbosity(2))) PRINT(numGeoms);
    

    double first_kernel_time0 = getSeconds();
    // === DUMMY KERNEL TO TRIGGER USM TRANSFER ===
    {	  
      sycl::event queue_event =  gpu_queue.submit([&](sycl::handler &cgh) {
                                                    cgh.single_task([=]() {
                                                                    });
                                                  });
      gpu::waitOnQueueAndCatchException(gpu_queue);
    }
    double first_kernel_time1 = getSeconds();

    if (unlikely(deviceGPU->verbosity(2))) std::cout << "Dummy first kernel launch (should trigger all USM transfers) " << (first_kernel_time1-first_kernel_time0)*1000.0f << " ms " << std::endl;
        
    // =============================
    // === count quads per block === 
    // =============================

    timer.start(BuildTimer::PRE_PROCESS);

    double device_quadification_time = 0.0f;
    countQuadsPerGeometry(gpu_queue,triMesh,numGeoms,quads_per_geom_prefix_sum,device_quadification_time,verbose);

    timer.stop(BuildTimer::PRE_PROCESS);
    timer.add_to_device_timer(BuildTimer::PRE_PROCESS,device_quadification_time);
                              
    if (unlikely(deviceGPU->verbosity(2))) std::cout << "Count quads: " << timer.get_host_timer() << " ms (host) " << device_quadification_time << " ms (device) " << std::endl;      

    /* ----------------------------- */    
    /* --- prefix sum over quads --- */
    /* ----------------------------- */
    
    timer.start(BuildTimer::PRE_PROCESS);    
    {
      static const uint GEOM_PREFIX_SUB_GROUP_WIDTH = 16;
      static const uint GEOM_PREFIX_WG_SIZE  = 1024;

      sycl::event queue_event =  gpu_queue.submit([&](sycl::handler &cgh) {
                                                    sycl::accessor< uint       , 1, sycl_read_write, sycl_local> counts(sycl::range<1>((GEOM_PREFIX_WG_SIZE/GEOM_PREFIX_SUB_GROUP_WIDTH)),cgh);
                                                    sycl::accessor< uint       , 1, sycl_read_write, sycl_local> counts_prefix_sum(sycl::range<1>((GEOM_PREFIX_WG_SIZE/GEOM_PREFIX_SUB_GROUP_WIDTH)),cgh);
                                                    const sycl::nd_range<1> nd_range(GEOM_PREFIX_WG_SIZE,sycl::range<1>(GEOM_PREFIX_WG_SIZE));		  
                                                    cgh.parallel_for(nd_range,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(GEOM_PREFIX_SUB_GROUP_WIDTH) {
                                                        const uint subgroupID      = get_sub_group_id();
                                                        const uint subgroupLocalID = get_sub_group_local_id();                                                        
                                                        const uint localID        = item.get_local_id(0);
                                                        const uint localSize       = item.get_local_range().size();            
                                                        const uint aligned_numGeoms = gpu::alignTo(numGeoms,GEOM_PREFIX_WG_SIZE);

                                                        uint total_offset = 0;
                                                        for (uint t=localID;t<aligned_numGeoms;t+=localSize)
                                                        {
                                                          item.barrier(sycl::access::fence_space::local_space);
                                                          
                                                          uint count = 0;
                                                          if (t < numGeoms)
                                                            count = quads_per_geom_prefix_sum[t];

                                                          const uint exclusive_scan = sub_group_exclusive_scan(count, std::plus<uint>());
                                                          const uint reduction = sub_group_reduce(count, std::plus<uint>());                                                     
                                                          counts[subgroupID] = reduction;                                                             

                                                          item.barrier(sycl::access::fence_space::local_space);

                                                          uint total_reduction = 0;
                                                          for (uint j=subgroupLocalID;j<GEOM_PREFIX_WG_SIZE/GEOM_PREFIX_SUB_GROUP_WIDTH;j+=GEOM_PREFIX_SUB_GROUP_WIDTH)
                                                          {
                                                            const uint subgroup_counts = counts[j];
                                                            const uint sums_exclusive_scan = sub_group_exclusive_scan(subgroup_counts, std::plus<uint>());
                                                            const uint reduction = sub_group_broadcast(subgroup_counts,GEOM_PREFIX_SUB_GROUP_WIDTH-1) + sub_group_broadcast(sums_exclusive_scan,GEOM_PREFIX_SUB_GROUP_WIDTH-1);
                                                            counts_prefix_sum[j] = sums_exclusive_scan + total_reduction;
                                                            total_reduction += reduction;
                                                          }
                                                          item.barrier(sycl::access::fence_space::local_space);

                                                          const uint sums_prefix_sum = counts_prefix_sum[subgroupID];                                                                 
                                                          const uint p_sum = total_offset + sums_prefix_sum + exclusive_scan;
                                                          total_offset += total_reduction;
                                                          
                                                          if (t < numGeoms)
                                                            quads_per_geom_prefix_sum[t] = p_sum;

                                                        }
                                                        
                                                        if (localID == 0)
                                                        {
                                                          quads_per_geom_prefix_sum[numGeoms] = total_offset;                                                          
                                                          *host_device_tasks = total_offset;
                                                        }
                                                        
                                                      });
                                                  });
      gpu::waitOnQueueAndCatchException(gpu_queue);
      double dt = gpu::getDeviceExecutionTiming(queue_event);
      timer.add_to_device_timer(BuildTimer::PRE_PROCESS,dt);
      
      if (unlikely(deviceGPU->verbosity(2)))
        std::cout << "Prefix sum over quad counts over geometries " << dt << " ms" << std::endl;
    }
    
    timer.stop(BuildTimer::PRE_PROCESS);
    
    const uint numPrimitives = *host_device_tasks;
    
    if (unlikely(deviceGPU->verbosity(1)))    
      PRINT2(org_numPrimitives,numPrimitives);

    // ==========================================================
    // ==========================================================

    const bool fastMCMode = numPrimitives < FAST_MC_THRESHOLD;
            
    /* --- estimate size of the BVH --- */
    const uint header              = 128;
    const uint node_size           = numPrimitives * 64; 
    const uint leaf_size           = numPrimitives * 64; 
    const uint totalSize           = header + node_size + leaf_size; 
    const uint node_data_start     = header;
    const uint leaf_data_start     = header + node_size;

    assert( (leaf_data_start % 64) == 0 );
    
    if (unlikely(deviceGPU->verbosity(2)))
    {
      PRINT2( node_size , node_size/64);
      PRINT2( leaf_size , leaf_size/64);	
      PRINT2( totalSize , totalSize/64);
      PRINT( numPrimitives );
      PRINT(fastMCMode);
    }
    
    // ================================    
    // === allocate and set buffers ===
    // ================================
    
    timer.start(BuildTimer::ALLOCATION);    

    
    if (accel.size() < totalSize) accel = std::move(Device::avector<char,64>(scene->device,totalSize));    
    gpu_queue.prefetch((char*)accel.data(),totalSize);

    sizeTotalAllocations += totalSize;
    
    QBVH6* qbvh   = (QBVH6*)accel.data();
    assert(qbvh);
    char *bvh_mem = (char*)accel.data() + header;
    assert(bvh_mem);
    const size_t conv_mem_size = sizeof(LeafGenerationData)*numPrimitives;
    
    LeafGenerationData *leafGenData = (LeafGenerationData*)sycl::aligned_alloc(64,conv_mem_size,deviceGPU->getGPUDevice(),deviceGPU->getGPUContext(),sycl::usm::alloc::device); // FIXME
    assert(conversionState);

    sizeTotalAllocations += conv_mem_size;
    

    uint *scratch_mem = (uint*)leafGenData;


    
    char *const leaf_mem = (char*)accel.data() + leaf_data_start;
    BVH2Ploc *const bvh2          = (BVH2Ploc*)(leaf_mem);

    typedef gpu::MortonCodePrimitive64Bit_2x MCPrim;

    // =================================    
    // === MCodes stored in leaf_mem ===
    // =================================
    
    MCPrim *const mc0 = (MCPrim*)(bvh2 + numPrimitives);
    MCPrim *const mc1 = mc0 + numPrimitives; 
    
    MCPrim *const morton_codes[2] = { mc0, mc1 }; 
        
    const size_t totalUSMAllocations = totalSize + conv_mem_size + sizeof(PLOCGlobals);
                
    // ====================================================================================
    // === cluster_index, bvh2_subtree_size stored in node_mem, needs 16 bytes per prim ===
    // ====================================================================================
    
    
    uint *const cluster_index     = (uint*) (bvh_mem + 0 * numPrimitives * sizeof(uint)); // * 2
    uint *const bvh2_subtree_size = (uint*) (bvh_mem + 2 * numPrimitives * sizeof(uint)); // * 2
        
    uint *cluster_i[2] = { cluster_index + 0, cluster_index + numPrimitives };        

    uint *const cluster_index_source = cluster_i[0];
    uint *const   cluster_index_dest = cluster_i[1];


    timer.stop(BuildTimer::ALLOCATION);    
          
    if (unlikely(deviceGPU->verbosity(2)))
      std::cout << "USM allocation time for BVH and additional data " << timer.get_host_timer() << " ms for " << (double)totalUSMAllocations / (1024*1024) << " MBs " << std::endl;     	


    
    // ======================          
    // ==== init globals ====
    // ======================
    {	  
      sycl::event queue_event =  gpu_queue.submit([&](sycl::handler &cgh) {
                                                    cgh.single_task([=]() {
                                                                      globals->reset();
                                                                      globals->numPrimitives              = numPrimitives;
                                                                      globals->totalAllocatedMem          = totalUSMAllocations;
                                                                      globals->numOrgPrimitives           = org_numPrimitives;                                                                      
                                                                      globals->node_mem_allocator_cur     = node_data_start/64;
                                                                      globals->node_mem_allocator_start   = node_data_start/64;
                                                                      globals->leaf_mem_allocator_cur     = leaf_data_start/64;
                                                                      globals->leaf_mem_allocator_start   = leaf_data_start/64;
                                                                      globals->bvh2_index_allocator       = numPrimitives; 
                                                                      globals->numBuildRecords            = numPrimitives;
                                                                    });
                                                  });
      gpu::waitOnQueueAndCatchException(gpu_queue);
      double dt = gpu::getDeviceExecutionTiming(queue_event);
      timer.add_to_device_timer(BuildTimer::PRE_PROCESS,dt);
      
      if (unlikely(deviceGPU->verbosity(2)))
        std::cout << "Init globals " << dt << " ms" << std::endl;
    }	    

    // ===========================================================          
    // ==== merge triangles to quads, write out PLOC primrefs ====
    // ===========================================================

    timer.start(BuildTimer::PRE_PROCESS);        
    
    device_quadification_time = 0.0f;

    mergeTriangleToQuads_initPLOCPrimRefs(gpu_queue,triMesh,numGeoms,quads_per_geom_prefix_sum,bvh2,device_quadification_time,verbose);

    timer.stop(BuildTimer::PRE_PROCESS);
    timer.add_to_device_timer(BuildTimer::PRE_PROCESS,device_quadification_time);
    
    if (unlikely(deviceGPU->verbosity(2))) std::cout << "Merge triangles to quads, write out quads: " << timer.get_host_timer() << " ms (host) " << device_quadification_time << " ms (device) " << std::endl;
    

#if 0    
     for (uint i=0;i<numPrimitives;i++)
       if (!bvh2[i].bounds.isValid() || !bvh2[i].bounds.checkNumericalBounds() )
       {
         PRINT2(i,bvh2[i]);      
         FATAL("Numerical Bounds in BVH2");
       }
#endif
            
        
    // ==========================================          
    // ==== get centroid and geometry bounds ====
    // ==========================================
        
     timer.start(BuildTimer::PRE_PROCESS);        
     double device_compute_centroid_bounds_time = 0.0f;
     
     computeCentroidGeometryBounds(gpu_queue, &globals->geometryBounds, &globals->centroidBounds, bvh2, numPrimitives, device_compute_centroid_bounds_time, verbose);
        
     timer.stop(BuildTimer::PRE_PROCESS);
     timer.add_to_device_timer(BuildTimer::PRE_PROCESS,device_compute_centroid_bounds_time);
        
    if (unlikely(deviceGPU->verbosity(2)))
      std::cout << "Get Geometry and Centroid Bounds Phase " << timer.get_host_timer() << " ms (host) " << device_compute_centroid_bounds_time << " ms (device) " << std::endl;		
       
    // ==============================          
    // ==== compute morton codes ====
    // ==============================

    timer.start(BuildTimer::PRE_PROCESS);        
    double device_compute_mc_time = 0.0f;

    if (!fastMCMode)
      computeMortonCodes64Bit_SaveMSBBits(gpu_queue,&globals->centroidBounds,mc0,bvh2,bvh2_subtree_size,numPrimitives,device_compute_mc_time,verbose);
    else
      computeMortonCodes64Bit(gpu_queue,&globals->centroidBounds,(gpu::MortonCodePrimitive40x24Bits3D*)mc1,bvh2,numPrimitives,0,(uint64_t)-1,device_compute_mc_time,verbose);
    
        
    timer.stop(BuildTimer::PRE_PROCESS);
     
    if (unlikely(deviceGPU->verbosity(2)))
      std::cout << "Compute Morton Codes " << timer.get_host_timer() << " ms (host) " << device_compute_mc_time << " ms (device) " << std::endl;		
    
    // ===========================          
    // ==== sort morton codes ====
    // ===========================


    timer.start(BuildTimer::PRE_PROCESS);        

    double sort_time = 0.0;
        
    //char *const radix_sort_scratch_mem = (char*)(mc1 + numPrimitives);

    if (!fastMCMode)
    {
      const uint scratchMemWGs = gpu::getNumWGsScratchSize(conv_mem_size);
      const uint nextPowerOf2 =  1 << (32 - sycl::clz(numPrimitives) - 1);
      const uint sortWGs = min(max(min((int)nextPowerOf2/8192,(int)gpu_maxComputeUnits/4 /*RADIX_SORT_MAX_NUM_DSS*/ ),1),(int)scratchMemWGs);
      if (unlikely(deviceGPU->verbosity(2)))      
        PRINT2(scratchMemWGs,sortWGs);

      for (uint i=4;i<8;i++) 
        gpu::sort_iteration_type<false,MCPrim>(gpu_queue, morton_codes[i%2], morton_codes[(i+1)%2], numPrimitives, scratch_mem, i, sort_time, sortWGs);
      gpu::waitOnQueueAndCatchException(gpu_queue);      
      
      restoreMSBBits(gpu_queue,mc0,bvh2_subtree_size,numPrimitives,sort_time,verbose);      

      for (uint i=4;i<8;i++) 
        gpu::sort_iteration_type<false,MCPrim>(gpu_queue, morton_codes[i%2], morton_codes[(i+1)%2], numPrimitives, scratch_mem, i, sort_time, sortWGs);
    }
    else
    {
      if (numPrimitives < SMALL_SORT_THRESHOLD)
      {
        gpu::radix_sort_single_workgroup(gpu_queue, (uint64_t *)mc0, (uint64_t *)mc1, numPrimitives, 3,8, sort_time);
      }
      else
      {
        const uint scratchMemWGs = gpu::getNumWGsScratchSize(conv_mem_size);        
        const uint nextPowerOf2 =  1 << (32 - sycl::clz(numPrimitives) - 1);          
        const uint sortWGs = min(max(min((int)nextPowerOf2/1024,(int)gpu_maxComputeUnits/4),1),(int)scratchMemWGs);
        if (unlikely(deviceGPU->verbosity(2)))        
          PRINT3(conv_mem_size,scratchMemWGs,sortWGs);
        for (uint i=3;i<8;i++) 
          gpu::sort_iteration_type<false,gpu::MortonCodePrimitive40x24Bits3D>(gpu_queue, (gpu::MortonCodePrimitive40x24Bits3D*)morton_codes[i%2], (gpu::MortonCodePrimitive40x24Bits3D*)morton_codes[(i+1)%2], numPrimitives, scratch_mem, i, sort_time, sortWGs);        
      }      
    }
      
    gpu::waitOnQueueAndCatchException(gpu_queue);

    timer.stop(BuildTimer::PRE_PROCESS);        
    timer.add_to_device_timer(BuildTimer::PRE_PROCESS,timer.get_host_timer());
            
    if (unlikely(deviceGPU->verbosity(2)))
      std::cout << "Sort Morton Codes " << timer.get_host_timer() << " ms (host and device)" << std::endl;
        
              
    // ===========================          
    // ====== init clusters ======
    // ===========================

    timer.start(BuildTimer::PRE_PROCESS);        
    double device_init_clusters_time = 0.0f;

    if (!fastMCMode)
      initClusters(gpu_queue,mc0,bvh2,cluster_index,bvh2_subtree_size,numPrimitives,device_init_clusters_time,verbose);
    else
      initClusters(gpu_queue,(gpu::MortonCodePrimitive40x24Bits3D*)mc0,bvh2,cluster_index,bvh2_subtree_size,numPrimitives,device_init_clusters_time,verbose); 
    
    timer.stop(BuildTimer::PRE_PROCESS);        
    timer.add_to_device_timer(BuildTimer::PRE_PROCESS,device_init_clusters_time);
        
    if (unlikely(deviceGPU->verbosity(2)))
      std::cout << "Init Clusters " << timer.get_host_timer() << " ms (host) " << device_init_clusters_time << " ms (device) " << std::endl;		

    uint numPrims = numPrimitives;

    
    // ===================================================================================================================================================
    // ===================================================================================================================================================
    // ===================================================================================================================================================
        
    double device_ploc_iteration_time = 0.0f;
        
    uint iteration = 0;

    timer.start(BuildTimer::BUILD);        

    if (two_level)
    {
      static const uint PARALLEL_WG_NUM = MAX_WGS;        
      static const uint RANGE_THRESHOLD = numPrimitives/(4*PARALLEL_WG_NUM);
      static const uint BOTTOM_UP_THRESHOLD = 16;
      static const uint SEARCH_RADIUS_TOP_LEVEL = 32;
                    
      gpu::Range *ranges = (gpu::Range*)scratch_mem; //FIXME: need to store ranges somewhere else
        
      extractRanges(gpu_queue, host_device_tasks, mc0, ranges, numPrims, RANGE_THRESHOLD , device_ploc_iteration_time, verbose);

      const uint numRanges = *host_device_tasks;
      
      gpu::radix_sort_single_workgroup(gpu_queue, (uint64_t*)ranges, (uint64_t*)ranges + numRanges, numRanges,0,8,device_ploc_iteration_time);
      
      parallelWGBuild(gpu_queue, globals, bvh2, cluster_index_source, cluster_index_dest, bvh2_subtree_size, ranges, numRanges, BOTTOM_UP_THRESHOLD, device_ploc_iteration_time, verbose);
      
      singleWGTopLevelBuild(gpu_queue, globals, bvh2, cluster_index_source, cluster_index_dest, bvh2_subtree_size, ranges, numRanges, SEARCH_RADIUS_TOP_LEVEL, device_ploc_iteration_time, verbose);
      timer.add_to_device_timer(BuildTimer::BUILD,device_ploc_iteration_time);
    }
    else
      for (;numPrims>1;iteration++)
      {          
        // ==================================================            
        // ==== single kernel path if #prims < threshold ====
        // ==================================================

        if (numPrims < SINGLE_WG_SWITCH_THRESHOLD)
        {
          double singleWG_time = 0.0f;
          singleWGBuild(gpu_queue, globals, bvh2, cluster_index_source, cluster_index_dest, bvh2_subtree_size, numPrims, singleWG_time, verbose);
          timer.add_to_device_timer(BuildTimer::BUILD,singleWG_time);

          numPrims = 1;
        }
        else  
        {            
          // ===================================================================================
          // ==== nearest neighbor search, merge clusters and create bvh2 nodes (fast path) ====
          // ===================================================================================

          const uint MERGED_KERNEL_WG_NUM = min((numPrims+1024-1)/1024,(uint)MAX_WGS);
          const uint radius = SEARCH_RADIUS;
          
          device_ploc_iteration_time = 0.0f;
          iteratePLOC(gpu_queue,globals,bvh2,cluster_index_source,cluster_index_dest,bvh2_subtree_size,scratch_mem,numPrims,radius,MERGED_KERNEL_WG_NUM,host_device_tasks,device_ploc_iteration_time, false);
          timer.add_to_device_timer(BuildTimer::BUILD,device_ploc_iteration_time);

          const uint new_numPrims = *host_device_tasks;
          assert(new_numPrims < numPrims);          
          numPrims = new_numPrims;          
            
          // ==========================            
        }        
        if (unlikely(deviceGPU->verbosity(2)))
          PRINT4(iteration,numPrims,(float)device_ploc_iteration_time,(float)timer.get_accum_device_timer(BuildTimer::BUILD));
      }

    timer.stop(BuildTimer::BUILD);        
            
    
                
    /* --- check and convert BVH2 (host) --- */        
          
    if (unlikely(deviceGPU->verbosity(2)))
    {
      if (globals->bvh2_index_allocator >= 2*numPrimitives)
        FATAL("BVH2 construction, allocator");
      
      PRINT(globals->rootIndex);
      PRINT(globals->bvh2_index_allocator);
      uint nodes = 0;
      uint leaves = 0;
      float nodeSAH = 0;
      float leafSAH = 0;
      checkBVH2PlocHW(bvh2,globals->rootIndex,nodes,leaves,nodeSAH,leafSAH,numPrimitives,globals->bvh2_index_allocator);
      nodeSAH /= globals->geometryBounds.area();
      leafSAH /= globals->geometryBounds.area();                
      PRINT4(nodes,leaves,nodeSAH,leafSAH);
 
      /* --- dummy kernel to trigger USM transfer again to not screw up device timings --- */
      sycl::event queue_event =  gpu_queue.submit([&](sycl::handler &cgh) {
                                                    cgh.single_task([=]() {
                                                                    });
                                                  });
      gpu::waitOnQueueAndCatchException(gpu_queue);
    }

    timer.start(BuildTimer::POST_PROCESS);        
   
    /* --- convert BVH2 to QBVH6 --- */    
    const float conversion_device_time = convertBVH2toQBVH6(gpu_queue,globals,host_device_tasks,triMesh,qbvh,bvh2,leafGenData,numPrimitives,node_size/64,verbose);

    /* --- init final QBVH6 header --- */        
    {     
      sycl::event queue_event =  gpu_queue.submit([&](sycl::handler &cgh) {
                                                    cgh.single_task([=]() {
                                                                      /* init qbvh */
                                                                      qbvh->bounds.lower.x = globals->geometryBounds.lower_x;
                                                                      qbvh->bounds.lower.y = globals->geometryBounds.lower_y;
                                                                      qbvh->bounds.lower.z = globals->geometryBounds.lower_z;
                                                                      qbvh->bounds.upper.x = globals->geometryBounds.upper_x;
                                                                      qbvh->bounds.upper.y = globals->geometryBounds.upper_y;
                                                                      qbvh->bounds.upper.z = globals->geometryBounds.upper_z;
                                                                      //qbvh->rootNodeOffset = 128;
                                                                      qbvh->numPrims       = org_numPrimitives;                                                                        
                                                                      qbvh->nodeDataStart  = globals->node_mem_allocator_start;
                                                                      qbvh->nodeDataCur    = globals->node_mem_allocator_cur;
                                                                      qbvh->leafDataStart  = globals->leaf_mem_allocator_start;
                                                                      qbvh->leafDataCur    = globals->leaf_mem_allocator_cur;
                                                                    });
                                                  });
      gpu::waitOnQueueAndCatchException(gpu_queue);
    }	    

    timer.stop(BuildTimer::POST_PROCESS);
    timer.add_to_device_timer(BuildTimer::POST_PROCESS,conversion_device_time);

    if (unlikely(deviceGPU->verbosity(2)))
    {
      std::cout << "BVH2 -> QBVH6 Flattening DONE in " <<  timer.get_host_timer() << " ms (host) " << conversion_device_time << " ms (device) " << std::endl << std::flush;
    }
    
    // ==========================================================
    // ==========================================================

    BBox3fa geomBounds(Vec3fa(globals->geometryBounds.lower_x,globals->geometryBounds.lower_y,globals->geometryBounds.lower_z),
                       Vec3fa(globals->geometryBounds.upper_x,globals->geometryBounds.upper_y,globals->geometryBounds.upper_z));
    
    if (deviceGPU) {
      HWAccel* hwaccel = (HWAccel*) accel.data();
      hwaccel->dispatchGlobalsPtr = (uint64_t) deviceGPU->dispatchGlobalsPtr;
    }
    
    timer.start(BuildTimer::ALLOCATION);        

    if (tmpMem0)           sycl::free(tmpMem0,deviceGPU->getGPUContext());    
    if (globals)           sycl::free(globals,deviceGPU->getGPUContext());
    if (leafGenData)       sycl::free(leafGenData,deviceGPU->getGPUContext());
    if (host_device_tasks) sycl::free(host_device_tasks,deviceGPU->getGPUContext());

    timer.stop(BuildTimer::ALLOCATION);        
    
    if (unlikely(deviceGPU->verbosity(2)))
      std::cout << "Time freeing temporary data " << timer.get_host_timer()  << " ms " << std::endl << std::flush;


    if (unlikely(deviceGPU->verbosity(1)))
    {
      const float total_host   = timer.get_total_host_time();
      const float total_device = timer.get_total_device_time();
      
      std::cout << "BVH2 GPU Ploc Builder DONE in " << total_host << " ms (host), " << total_device << " ms (device) => Quads Build : " << numPrimitives*0.001f/total_host << " MPrims/s (host) " << numPrimitives*0.001f/total_device << " MPrims/s (device) / Original Tris : " << org_numPrimitives*0.001f/total_host << " MPrims/s (host) " <<  org_numPrimitives*0.001f/total_device << " MPrims/s (device) " << std::endl << std::flush;
      std::cout << "Allocation    " << timer.get_accum_host_timer(BuildTimer::ALLOCATION) << " ms (host) for " << (float)sizeTotalAllocations / (1024*1024) << " MB => " << (float)sizeTotalAllocations / (1024*1024) * 1000 / timer.get_accum_host_timer(BuildTimer::ALLOCATION) <<  " MB/s " << std::endl;
      std::cout << "Pre-process   " << timer.get_accum_host_timer(BuildTimer::PRE_PROCESS) << " ms (host) " << timer.get_accum_device_timer(BuildTimer::PRE_PROCESS) << " ms (device) , ratio " << timer.get_accum_host_timer(BuildTimer::PRE_PROCESS) / timer.get_accum_device_timer(BuildTimer::PRE_PROCESS) << std::endl;
      std::cout << "Build         " << timer.get_accum_host_timer(BuildTimer::BUILD) << " ms (host) " << timer.get_accum_device_timer(BuildTimer::BUILD) << " ms (device) , ratio " << timer.get_accum_host_timer(BuildTimer::BUILD) / timer.get_accum_device_timer(BuildTimer::BUILD) << std::endl;
      std::cout << "Post-process  " << timer.get_accum_host_timer(BuildTimer::POST_PROCESS) << " ms (host) " << timer.get_accum_device_timer(BuildTimer::POST_PROCESS) << " ms (device) , ratio " << timer.get_accum_host_timer(BuildTimer::POST_PROCESS) / timer.get_accum_device_timer(BuildTimer::POST_PROCESS) << std::endl;
      
      
      //exit(0);
    }
    
    if (unlikely(deviceGPU->verbosity(2)))
    {
      //qbvh->print(std::cout,qbvh->root(),0,6);
      BVHStatistics stats = qbvh->computeStatistics();      
      stats.print(std::cout);
      stats.print_raw(std::cout);
    }
      
    
    return geomBounds;
  }
 
}


