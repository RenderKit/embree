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


  __forceinline uint estimateSizeInternalNodes(const uint numActiveQuads, const uint numInstances, const uint numUserGeometries)
  {
    const uint N = numActiveQuads + numInstances + numUserGeometries;
    // === conservative estimate ===
    const uint numFatLeaves = ceilf( (float)N/2 );
    const uint numInnerNodes = ceilf( (float)numFatLeaves/5 );
    return std::max( (numFatLeaves + numInnerNodes) * 64, N * 16);
  }

  __forceinline uint estimateSizeLeafNodes(const uint numActiveQuads, const uint numInstances, const uint numUserGeometries)
  {
    return (numActiveQuads + numUserGeometries + 2 * numInstances) * 64;  
  }
  
  
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
      const uint numChildren = openBVH2MaxAreaSortChildren(BVH2Ploc::getIndex(index),indices,bvh2,numPrimitives);
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

  __forceinline uint32_t getNumPrimitives(const RTHWIF_GEOMETRY_DESC* geom)
  {
    switch (geom->geometryType) {
    case RTHWIF_GEOMETRY_TYPE_TRIANGLES  : return ((RTHWIF_GEOMETRY_TRIANGLES_DESC*)  geom)->triangleCount;
    case RTHWIF_GEOMETRY_TYPE_AABBS_FPTR : return ((RTHWIF_GEOMETRY_AABBS_FPTR_DESC*) geom)->primCount;
    case RTHWIF_GEOMETRY_TYPE_QUADS      : return ((RTHWIF_GEOMETRY_QUADS_DESC*)      geom)->quadCount;
    case RTHWIF_GEOMETRY_TYPE_INSTANCE   : return 1;
    default                              : return 0;
    };
  }

  /* fill all arg members that app did not know of yet */
  RTHWIF_BUILD_ACCEL_ARGS rthwifPrepareBuildAccelArgs(const RTHWIF_BUILD_ACCEL_ARGS& args_i)
  {
    RTHWIF_BUILD_ACCEL_ARGS args;
    memset(&args,0,sizeof(RTHWIF_BUILD_ACCEL_ARGS));
    memcpy(&args,&args_i,std::min(sizeof(RTHWIF_BUILD_ACCEL_ARGS),args_i.structBytes));
    args.structBytes = sizeof(RTHWIF_BUILD_ACCEL_ARGS);
    return args;
  }
  

  RTHWIF_API RTHWIF_ERROR rthwifGetAccelSizeGPU(const RTHWIF_BUILD_ACCEL_ARGS& args_i, RTHWIF_ACCEL_SIZE& size_o)
  {
#if defined(EMBREE_DPCPP_GPU_BVH_BUILDER)
    
    RTHWIF_BUILD_ACCEL_ARGS args = rthwifPrepareBuildAccelArgs(args_i);
    const RTHWIF_GEOMETRY_DESC** geometries = args.geometries;
    const uint numGeometries = args.numGeometries;

    auto getSize = [&](uint32_t geomID) -> size_t {
      const RTHWIF_GEOMETRY_DESC* geom = geometries[geomID];
      if (geom == nullptr) return 0;
      return getNumPrimitives(geom);
    };
    
    auto getType = [&](unsigned int geomID)
    {
      const RTHWIF_GEOMETRY_DESC* geom = geometries[geomID];
      assert(geom);
      return geom->geometryType;
    };

    uint numTriangles   = 0;
    uint numQuads       = 0;
    uint numProcedurals = 0;
    uint numInstances   = 0;
    
    for (size_t geomID=0; geomID<numGeometries; geomID++)
    {
      uint32_t numPrimitives = getSize(geomID);
      if (numPrimitives == 0) continue;          
      switch (getType(geomID)) {
      case RTHWIF_GEOMETRY_TYPE_TRIANGLES  : numTriangles   += numPrimitives; break;
      case RTHWIF_GEOMETRY_TYPE_QUADS      : numQuads       += numPrimitives; break;
      case RTHWIF_GEOMETRY_TYPE_AABBS_FPTR : numProcedurals += numPrimitives; break;
      case RTHWIF_GEOMETRY_TYPE_INSTANCE   : numInstances   += numPrimitives; break;
      default: assert(false); break;        
      };
    }

    // ============================================================================================================================================================================
    // ============================================================================================================================================================================
    // ============================================================================================================================================================================
    DeviceGPU* deviceGPU = (DeviceGPU*)args.deviceGPU;

    sycl::queue &gpu_queue = deviceGPU->getGPUQueue();
    
    const bool verbose2 = deviceGPU->verbosity(2);

    PRINT4(numTriangles,numQuads,numProcedurals,numInstances);

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

    if (unlikely(verbose2)) std::cout << "Dummy first kernel launch (should trigger all USM transfers) " << (first_kernel_time1-first_kernel_time0)*1000.0f << " ms " << std::endl;

    uint *quadsPerGeometry  = (uint*)sycl::aligned_alloc(64,numGeometries*sizeof(uint),deviceGPU->getGPUDevice(),deviceGPU->getGPUContext(),sycl::usm::alloc::shared);

    double device_coundQuadsTime = 0.0f;
    countQuadsPerGeometry(gpu_queue,args.geometries,numGeometries,quadsPerGeometry,device_coundQuadsTime,verbose2);
    

    uint numActiveQuads = 0;
    for (uint i=0;i<numGeometries;i++)
      numActiveQuads += quadsPerGeometry[i];

    PRINT( numActiveQuads );

    sycl::free(quadsPerGeometry,deviceGPU->getGPUContext());

    const uint numPrimitives = numActiveQuads + numInstances + numProcedurals;
    PRINT( numPrimitives );


    const uint header              = 128;
    const uint node_size           = estimateSizeInternalNodes(numActiveQuads,numInstances,numProcedurals);
    const uint leaf_size           = estimateSizeLeafNodes(numActiveQuads,numInstances,numProcedurals); 
    const uint totalSize           = header + node_size + leaf_size; 
    
    /* query memory requirements from builder */
    size_t expectedBytes = totalSize;
    size_t worstCaseBytes = totalSize;
    size_t scratchBytes = std::max(numPrimitives,MAX_WGS) * sizeof(LeafGenerationData) + sizeof(PLOCGlobals);

    PRINT3(expectedBytes,worstCaseBytes,scratchBytes);
    
    // ============================================================================================================================================================================
    // ============================================================================================================================================================================
    // ============================================================================================================================================================================
    
    /* return size to user */
    RTHWIF_ACCEL_SIZE size;
    memset(&size,0,sizeof(RTHWIF_ACCEL_SIZE));
    size.accelBufferExpectedBytes = expectedBytes;
    size.accelBufferWorstCaseBytes = worstCaseBytes;
    size.scratchBufferBytes = scratchBytes;
    size.numQuads = numActiveQuads;
    size.numInstances = numInstances;
    size.numProcedurals = numProcedurals;
    size_t bytes_o = size_o.structBytes;
    memset(&size_o,0,bytes_o);
    memcpy(&size_o,&size,bytes_o);
    size_o.structBytes = bytes_o;
#endif    
    return RTHWIF_ERROR_NONE;
  }
  

  RTHWIF_API RTHWIF_ERROR rthwifBuildAccelGPU(const RTHWIF_BUILD_ACCEL_ARGS& args, const RTHWIF_ACCEL_SIZE& sizeIn)
  {
#if defined(EMBREE_DPCPP_GPU_BVH_BUILDER)    
    BuildTimer timer;
    timer.reset();
    
    DeviceGPU* deviceGPU = (DeviceGPU*)args.deviceGPU;
    sycl::queue &gpu_queue = deviceGPU->getGPUQueue();    
    const bool verbose2 = deviceGPU->verbosity(2);

    /* -------------------------------- */    
    /* --- estimate size of the BVH --- */
    /* -------------------------------- */
    
    const uint header              = 128;
    const uint node_size           = estimateSizeInternalNodes(sizeIn.numQuads,sizeIn.numInstances,sizeIn.numProcedurals);
    const uint leaf_size           = estimateSizeLeafNodes(sizeIn.numQuads,sizeIn.numInstances,sizeIn.numProcedurals); 
    const uint totalSize           = header + node_size + leaf_size; 
    const uint node_data_start     = header;
    const uint leaf_data_start     = header + node_size;
    
    const uint numGeometries = args.numGeometries;
    const uint numPrimitives = sizeIn.numQuads + sizeIn.numInstances + sizeIn.numProcedurals;
    const uint numInstances = sizeIn.numInstances;
    const bool fastMCMode = numPrimitives < FAST_MC_THRESHOLD;
    const size_t conv_mem_size = sizeof(numPrimitives)*numPrimitives;

    /* --------------------------- */    
    /* --- set up all pointers --- */
    /* --------------------------- */
    
    uint *scratch = (uint*)((char*)args.scratchBuffer + sizeof(PLOCGlobals));
    PLOCGlobals *globals = (PLOCGlobals *)args.scratchBuffer;
    uint *host_device_tasks = (uint*)args.hostDeviceCommPtr;
    uint *prims_per_geom_prefix_sum = (uint*)scratch;
    QBVH6* qbvh   = (QBVH6*)args.accelBuffer;
    char *bvh_mem = (char*)qbvh + header;
    char *const leaf_mem = (char*)qbvh + leaf_data_start;
    BVH2Ploc *const bvh2 = (BVH2Ploc*)(leaf_mem);
    typedef gpu::MortonCodePrimitive64Bit_2x MCPrim;
    MCPrim *const mc0 = (MCPrim*)(bvh2 + numPrimitives);
    MCPrim *const mc1 = mc0 + numPrimitives;     
    MCPrim *const morton_codes[2] = { mc0, mc1 }; 
    uint *const cluster_index     = (uint*) (bvh_mem + 0 * numPrimitives * sizeof(uint)); // * 2
    uint *const bvh2_subtree_size = (uint*) (bvh_mem + 2 * numPrimitives * sizeof(uint)); // * 2        
    uint *cluster_i[2] = { cluster_index + 0, cluster_index + numPrimitives };        
    uint *const cluster_index_source = cluster_i[0];
    uint *const   cluster_index_dest = cluster_i[1];
    LeafGenerationData *leafGenData = (LeafGenerationData*)scratch;

    const uint gpu_maxWorkGroupSize = deviceGPU->getGPUDevice().get_info<sycl::info::device::max_work_group_size>();
    const uint gpu_maxComputeUnits  = deviceGPU->getGPUDevice().get_info<sycl::info::device::max_compute_units>();    
    const uint gpu_maxLocalMemory   = deviceGPU->getGPUDevice().get_info<sycl::info::device::local_mem_size>();
    
    if (unlikely(verbose2))
    {
      PRINT("PLOC++ GPU BVH BUILDER");            
      PRINT( deviceGPU->getGPUDevice().get_info<sycl::info::device::global_mem_size>() );
      PRINT(gpu_maxWorkGroupSize);
      PRINT(gpu_maxComputeUnits);
      PRINT(gpu_maxLocalMemory);
    }

    // ======================          
    // ==== init globals ====
    // ======================
    {	  
      sycl::event queue_event =  gpu_queue.submit([&](sycl::handler &cgh) {
                                                    cgh.single_task([=]() {
                                                                      globals->reset();
                                                                      globals->numPrimitives              = numPrimitives;
                                                                      globals->totalAllocatedMem          = totalSize;
                                                                      globals->numOrgPrimitives           = numPrimitives;                                                                      
                                                                      globals->node_mem_allocator_cur     = node_data_start/64;
                                                                      globals->node_mem_allocator_start   = node_data_start/64;
                                                                      globals->leaf_mem_allocator_cur     = leaf_data_start/64;
                                                                      globals->leaf_mem_allocator_start   = leaf_data_start/64;
                                                                      globals->bvh2_index_allocator       = numPrimitives; 
                                                                      globals->numBuildRecords             = 0;
                                                                    });
                                                  });
      gpu::waitOnQueueAndCatchException(gpu_queue);
      double dt = gpu::getDeviceExecutionTiming(queue_event);
      timer.add_to_device_timer(BuildTimer::PRE_PROCESS,dt);
      
      if (unlikely(verbose2))
        std::cout << "Init globals " << dt << " ms" << std::endl;
    }	    
    
    // =============================
    // === count quads per block === 
    // =============================

    timer.start(BuildTimer::PRE_PROCESS);

    double device_quadification_time = 0.0f;
    
    countQuadsPerGeometry(gpu_queue,args.geometries,numGeometries,prims_per_geom_prefix_sum,device_quadification_time,verbose2);

    timer.stop(BuildTimer::PRE_PROCESS);
    timer.add_to_device_timer(BuildTimer::PRE_PROCESS,device_quadification_time);
                              
    if (unlikely(verbose2)) std::cout << "Count quads: " << timer.get_host_timer() << " ms (host) " << device_quadification_time << " ms (device) " << std::endl;      

    // =============================      
    // === prefix sum over quads ===
    // =============================      
    
    timer.start(BuildTimer::PRE_PROCESS);
    double geom_prefix_sum_time = 0.0f;
     
    prefixsumOverGeometryCounts(gpu_queue,numGeometries,prims_per_geom_prefix_sum,host_device_tasks,geom_prefix_sum_time,verbose2);
    
    timer.stop(BuildTimer::PRE_PROCESS);
    timer.add_to_device_timer(BuildTimer::PRE_PROCESS,geom_prefix_sum_time);
    const uint numActiveQuads = *host_device_tasks;
    
    if (numActiveQuads != sizeIn.numQuads) FATAL("QUADS");

    // ===================================================          
    // ==== merge triangles to quads, create primrefs ====
    // ===================================================

    timer.start(BuildTimer::PRE_PROCESS);        
    
    double create_primref_time = 0.0f;
      
    createQuads_initPLOCPrimRefs(gpu_queue,args.geometries,numGeometries,prims_per_geom_prefix_sum,bvh2,0,create_primref_time,verbose2);

    const GeometryTypeRanges geometryTypeRanges(sizeIn.numQuads,sizeIn.numProcedurals,sizeIn.numInstances);        
      
    timer.stop(BuildTimer::PRE_PROCESS);
    timer.add_to_device_timer(BuildTimer::PRE_PROCESS,create_primref_time);
    
    if (unlikely(verbose2)) std::cout << "create quads/userGeometries/instances etc, init primrefs: " << timer.get_host_timer() << " ms (host) " << create_primref_time << " ms (device) " << std::endl;
      

    // ==========================================          
    // ==== get centroid and geometry bounds ====
    // ==========================================
        
    timer.start(BuildTimer::PRE_PROCESS);        
    double device_compute_centroid_bounds_time = 0.0f;
     
    computeCentroidGeometryBounds(gpu_queue, &globals->geometryBounds, &globals->centroidBounds, bvh2, numPrimitives, device_compute_centroid_bounds_time, verbose2);
    
    timer.stop(BuildTimer::PRE_PROCESS);
    timer.add_to_device_timer(BuildTimer::PRE_PROCESS,device_compute_centroid_bounds_time);


    if (unlikely(verbose2))
      std::cout << "Get Geometry and Centroid Bounds Phase " << timer.get_host_timer() << " ms (host) " << device_compute_centroid_bounds_time << " ms (device) " << std::endl;		
       
    // ==============================          
    // ==== compute morton codes ====
    // ==============================

    timer.start(BuildTimer::PRE_PROCESS);        
    double device_compute_mc_time = 0.0f;

    if (!fastMCMode)
      computeMortonCodes64Bit_SaveMSBBits(gpu_queue,&globals->centroidBounds,mc0,bvh2,bvh2_subtree_size,numPrimitives,device_compute_mc_time,verbose2);
    else
      computeMortonCodes64Bit(gpu_queue,&globals->centroidBounds,(gpu::MortonCodePrimitive40x24Bits3D*)mc1,bvh2,numPrimitives,0,(uint64_t)-1,device_compute_mc_time,verbose2);
    
        
    timer.stop(BuildTimer::PRE_PROCESS);
     
    if (unlikely(verbose2))
      std::cout << "Compute Morton Codes " << timer.get_host_timer() << " ms (host) " << device_compute_mc_time << " ms (device) " << std::endl;		
    
    // ===========================          
    // ==== sort morton codes ====
    // ===========================

    timer.start(BuildTimer::PRE_PROCESS);        

    double sort_time = 0.0;

    const bool sync_sort = false;
    
    if (!fastMCMode)
    {
      const uint scratchMemWGs = gpu::getNumWGsScratchSize(conv_mem_size);
      const uint nextPowerOf2 =  1 << (32 - sycl::clz(numPrimitives) - 1);
      const uint sortWGs = min(max(min((int)nextPowerOf2/8192,(int)gpu_maxComputeUnits/4 /*RADIX_SORT_MAX_NUM_DSS*/ ),1),(int)scratchMemWGs);
      if (unlikely(verbose2))      
        PRINT2(scratchMemWGs,sortWGs);
     
      for (uint i=4;i<8;i++) 
        gpu::sort_iteration_type<MCPrim>(gpu_queue, morton_codes[i%2], morton_codes[(i+1)%2], numPrimitives, (uint*)scratch, i, sort_time, sortWGs, sync_sort);
      gpu::waitOnQueueAndCatchException(gpu_queue);      
      
      restoreMSBBits(gpu_queue,mc0,bvh2_subtree_size,numPrimitives,sort_time,verbose2);      

      for (uint i=4;i<8;i++) 
        gpu::sort_iteration_type<MCPrim>(gpu_queue, morton_codes[i%2], morton_codes[(i+1)%2], numPrimitives, (uint*)scratch, i, sort_time, sortWGs, sync_sort);
    }
    else
    {
      if (numPrimitives < SMALL_SORT_THRESHOLD)
        gpu::radix_sort_single_workgroup(gpu_queue, (uint64_t *)mc0, (uint64_t *)mc1, numPrimitives, 3,8, sort_time);
      else
      {
        const uint scratchMemWGs = gpu::getNumWGsScratchSize(conv_mem_size);        
        const uint nextPowerOf2 =  1 << (32 - sycl::clz(numPrimitives) - 1);          
        const uint sortWGs = min(max(min((int)nextPowerOf2/1024,(int)gpu_maxComputeUnits/4),1),(int)scratchMemWGs);
        if (unlikely(verbose2))        
          PRINT3(conv_mem_size,scratchMemWGs,sortWGs);
        for (uint i=3;i<8;i++) 
          gpu::sort_iteration_type<gpu::MortonCodePrimitive40x24Bits3D>(gpu_queue, (gpu::MortonCodePrimitive40x24Bits3D*)morton_codes[i%2], (gpu::MortonCodePrimitive40x24Bits3D*)morton_codes[(i+1)%2], numPrimitives, (uint*)scratch, i, sort_time, sortWGs, sync_sort);        
      }      
    }
    
    gpu::waitOnQueueAndCatchException(gpu_queue);

    timer.stop(BuildTimer::PRE_PROCESS);        
    timer.add_to_device_timer(BuildTimer::PRE_PROCESS,timer.get_host_timer());
            
    if (unlikely(verbose2))
      std::cout << "Sort Morton Codes " << timer.get_host_timer() << " ms (host and device)" << std::endl;
        
              
    // ===========================          
    // ====== init clusters ======
    // ===========================

    
    timer.start(BuildTimer::PRE_PROCESS);        
    double device_init_clusters_time = 0.0f;

    if (!fastMCMode)
      initClusters(gpu_queue,mc0,bvh2,cluster_index,bvh2_subtree_size,numPrimitives,device_init_clusters_time,verbose2);
    else
      initClusters(gpu_queue,(gpu::MortonCodePrimitive40x24Bits3D*)mc0,bvh2,cluster_index,bvh2_subtree_size,numPrimitives,device_init_clusters_time,verbose2); 
    
    timer.stop(BuildTimer::PRE_PROCESS);        
    timer.add_to_device_timer(BuildTimer::PRE_PROCESS,device_init_clusters_time);
        
    if (unlikely(verbose2))
      std::cout << "Init Clusters " << timer.get_host_timer() << " ms (host) " << device_init_clusters_time << " ms (device) " << std::endl;		

    uint numPrims = numPrimitives;

    // ===================================================================================================================================================
    // ===================================================================================================================================================
    // ===================================================================================================================================================
        
    double device_ploc_iteration_time = 0.0f;
        
    uint iteration = 0;

    timer.start(BuildTimer::BUILD);        

    // ===========================            
    // ==== clear scratch mem ====
    // ===========================      
    {
      const sycl::nd_range<1> nd_range1(MAX_WGS,sycl::range<1>(MAX_WGS)); 
      sycl::event queue_event = gpu_queue.submit([&](sycl::handler &cgh) {
                                                   cgh.parallel_for(nd_range1,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(16)
                                                                    {
                                                                      const uint globalID     = item.get_global_id(0);
                                                                      scratch[globalID] = 0;
                                                                    });                                                         
                                                 });
    }
      
    for (;numPrims>1;iteration++)
    {          
      // ==================================================            
      // ==== single kernel path if #prims < threshold ====
      // ==================================================

      if (numPrims < SINGLE_WG_SWITCH_THRESHOLD)
      {
        double singleWG_time = 0.0f;
        singleWGBuild(gpu_queue, globals, bvh2, cluster_index_source, cluster_index_dest, bvh2_subtree_size, numPrims, singleWG_time, verbose2);
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
        iteratePLOC(gpu_queue,globals,bvh2,cluster_index_source,cluster_index_dest,bvh2_subtree_size,scratch,numPrims,radius,MERGED_KERNEL_WG_NUM,host_device_tasks,device_ploc_iteration_time, false);
        timer.add_to_device_timer(BuildTimer::BUILD,device_ploc_iteration_time);

        const uint new_numPrims = *host_device_tasks;
        assert(new_numPrims < numPrims);          
        numPrims = new_numPrims;          
            
        // ==========================            
      }        
      if (unlikely(verbose2))
        PRINT4(iteration,numPrims,(float)device_ploc_iteration_time,(float)timer.get_accum_device_timer(BuildTimer::BUILD));
    }
    timer.stop(BuildTimer::BUILD);        
            
    
    // =====================================                
    // === check and convert BVH2 (host) ===
    // =====================================

    if (unlikely(verbose2))
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


    // =============================    
    // === convert BVH2 to QBVH6 ===
    // =============================
    
    const float conversion_device_time = convertBVH2toQBVH6(gpu_queue,globals,host_device_tasks,args.geometries,qbvh,bvh2,leafGenData,numPrimitives,numInstances != 0,geometryTypeRanges,verbose2);

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
                                                                      qbvh->numPrims       = numPrimitives;                                                                        
                                                                      qbvh->nodeDataStart  = globals->node_mem_allocator_start;
                                                                      qbvh->nodeDataCur    = globals->node_mem_allocator_cur;
                                                                      qbvh->leafDataStart  = globals->leaf_mem_allocator_start;
                                                                      qbvh->leafDataCur    = globals->leaf_mem_allocator_cur;
                                                                      *(gpu::AABB3f*)host_device_tasks = globals->geometryBounds;
                                                                    });
                                                  });
      gpu::waitOnQueueAndCatchException(gpu_queue);
    }	    

    if (args.boundsOut) *args.boundsOut = *(RTHWIF_AABB*)host_device_tasks;
  
    timer.stop(BuildTimer::POST_PROCESS);
    timer.add_to_device_timer(BuildTimer::POST_PROCESS,conversion_device_time);

    if (unlikely(verbose2))
      std::cout << "BVH2 -> QBVH6 Flattening DONE in " <<  timer.get_host_timer() << " ms (host) " << conversion_device_time << " ms (device) " << std::endl << std::flush;

    // ==========================================================    
    // ==========================================================
    // ==========================================================

    if (unlikely(verbose2))
    {
      // === memory allocation and usage stats ===
      PRINT(globals->node_mem_allocator_start);      
      PRINT(globals->node_mem_allocator_cur);
      PRINT(globals->node_mem_allocator_cur-globals->node_mem_allocator_start);
      PRINT(100.0f * (float)(globals->node_mem_allocator_cur-globals->node_mem_allocator_start) / (node_size/64));
      PRINT(globals->leaf_mem_allocator_start);      
      PRINT(globals->leaf_mem_allocator_cur);
      PRINT(globals->leaf_mem_allocator_cur-globals->leaf_mem_allocator_start);
      PRINT(100.0f * (float)(globals->leaf_mem_allocator_cur-globals->leaf_mem_allocator_start) / (leaf_size/64));      
      PRINT(globals->numLeaves);

    }

    if ((globals->node_mem_allocator_cur-globals->node_mem_allocator_start)*64 > node_size ||
        (globals->leaf_mem_allocator_cur-globals->leaf_mem_allocator_start)*64 > leaf_size) return RTHWIF_ERROR_RETRY; 
    
    if (deviceGPU) {
      HWAccel* hwaccel = (HWAccel*)args.accelBuffer;
#if defined(EMBREE_DPCPP_ALLOC_DISPATCH_GLOBALS)      
      hwaccel->dispatchGlobalsPtr = (uint64_t)args.dispatchGlobalsPtr;
#endif      
    }
#endif    
    return RTHWIF_ERROR_NONE;    
  }
  
  BBox3f rthwifBuildPloc(Scene* scene, RTCBuildQuality quality_flags, AccelBuffer& accel, void *dispatchGlobalsPtr)
  {
    const bool two_level = false;
    
    BuildTimer timer;
    timer.reset();
      
    DeviceGPU* deviceGPU = dynamic_cast<DeviceGPU*>(scene->device);
    assert(deviceGPU);
    const bool verbose1 = deviceGPU->verbosity(1);    
    const bool verbose2 = deviceGPU->verbosity(2);
    
    // ===============================================================================================================
  
    sycl::queue &gpu_queue = deviceGPU->getGPUQueue();

    const uint gpu_maxWorkGroupSize = deviceGPU->getGPUDevice().get_info<sycl::info::device::max_work_group_size>();
    const uint gpu_maxComputeUnits  = deviceGPU->getGPUDevice().get_info<sycl::info::device::max_compute_units>();    
    const uint gpu_maxLocalMemory   = deviceGPU->getGPUDevice().get_info<sycl::info::device::local_mem_size>();
    const uint gpu_maxSubgroups     = gpu_maxComputeUnits * 8;

    if (unlikely(verbose2))
    {
      PRINT("PLOC++ GPU BVH BUILDER");
      PRINT( deviceGPU->getGPUDevice().get_info<sycl::info::device::global_mem_size>() );
      PRINT(gpu_maxWorkGroupSize);
      PRINT(gpu_maxComputeUnits);
      PRINT(gpu_maxLocalMemory);
      PRINT(gpu_maxSubgroups);
    }

    const uint numGeoms = scene->size();
    const uint numTriangles        = scene->getNumPrimitives(TriangleMesh::geom_type,false);
    const uint numQuads            = scene->getNumPrimitives(QuadMesh::geom_type,false);
    const uint numInstances        = scene->getNumPrimitives(Instance::geom_type,false);
    const uint numUserGeometries   = scene->getNumPrimitives(UserGeometry::geom_type,false);
    const bool activeTriQuadMeshes = numTriangles || numQuads;

    if (unlikely(verbose2))
      PRINT5(numGeoms,numTriangles,numQuads,numInstances,numUserGeometries);

    // ===================    
    // === empty scene ===
    // ===================
    
    if (unlikely(!activeTriQuadMeshes && !numInstances && !numUserGeometries))
      {
        const size_t totalSize = 3*64; // just for the header and a single node
        if (accel.size() < totalSize) accel.resize(totalSize); //accel = std::move(Device::avector<char,64>(scene->device,totalSize));    
        QBVH6* qbvh   = (QBVH6*)accel.data();        
        BBox3fa geometryBounds (Vec3fa(0.0f),Vec3fa(0.0f));
        qbvh->bounds = geometryBounds;
        qbvh->numPrims       = 0;                                                                        
        qbvh->nodeDataStart  = 2;
        qbvh->nodeDataCur    = 3;
        qbvh->leafDataStart  = 3;
        qbvh->leafDataCur    = 3;        
        new (qbvh->nodePtr(2)) QBVH6::InternalNode6(NODE_TYPE_INTERNAL);
        return geometryBounds;
      }
        
    // ===============================================================================================================
    
    size_t sizeTotalAllocations = 0;
    
    timer.start(BuildTimer::ALLOCATION);

    // === host-device communication buffer ===
    uint *host_device_tasks = (uint*)sycl::aligned_alloc(64,sizeof(uint)*4,deviceGPU->getGPUDevice(),deviceGPU->getGPUContext(),sycl::usm::alloc::host); // FIXME
    assert(host_device_tasks);

    // === globals ===    
    PLOCGlobals *globals  = (PLOCGlobals*)sycl::aligned_alloc(64,sizeof(PLOCGlobals),deviceGPU->getGPUDevice(),deviceGPU->getGPUContext(),sycl::usm::alloc::shared);
    assert(globals);

    sizeTotalAllocations += sizeof(PLOCGlobals);

    char* scratch_mem0               = nullptr;
    TriQuadMesh *triQuadMesh         = nullptr;
    uint *prims_per_geom_prefix_sum  = nullptr;
    size_t size_scratch_mem0         = 0;

    // === triangles/quads? ===
    if (activeTriQuadMeshes)
    {
      const size_t alloc_TriQuadMeshes = sizeof(TriQuadMesh)*(numGeoms+1);
      const size_t alloc_GeomPrefixSums = sizeof(uint)*(numGeoms+1);
      size_scratch_mem0 = alloc_GeomPrefixSums+alloc_TriQuadMeshes;
        
      scratch_mem0 = (char*)sycl::aligned_alloc(64,size_scratch_mem0,deviceGPU->getGPUDevice(),deviceGPU->getGPUContext(),sycl::usm::alloc::shared);
      gpu_queue.prefetch(scratch_mem0,size_scratch_mem0);
        
      triQuadMesh                = (TriQuadMesh*)(scratch_mem0 + 0);
      prims_per_geom_prefix_sum  =        (uint*)(scratch_mem0 + alloc_TriQuadMeshes);
    }


    sizeTotalAllocations += size_scratch_mem0;    
    timer.stop(BuildTimer::ALLOCATION);
	
    if (unlikely(verbose2))
      std::cout << "USM allocation time for globals, tri meshes/prefix sums or instances " << timer.get_host_timer() << " ms for " << (double)(size_scratch_mem0) / (1024*1024) << " MBs " << std::endl;     	

    // ==============================================
    // === compute prefix sum over geometry sizes === 
    // ==============================================
    
    uint org_numPrimitives = 0;
    
    for (uint  geomID = 0; geomID < numGeoms; geomID++)
    {
      const uint current = scene->get(geomID)->size();
      org_numPrimitives += current;

      if (activeTriQuadMeshes)
      {
        // TODO: use BufferView
        TriangleMesh* tri_mesh = scene->getSafe<TriangleMesh>(geomID);
        QuadMesh*    quad_mesh = scene->getSafe<QuadMesh>(geomID);
      
        if (tri_mesh)
        {
          triQuadMesh[geomID] = TriQuadMesh(tri_mesh->size(),tri_mesh->numVertices(),(TriangleMesh::Triangle*)tri_mesh->triangles.getPtr(),(Vec3fa*)tri_mesh->vertices0.getPtr());
          gpu_queue.prefetch((TriangleMesh::Triangle*)tri_mesh->triangles.getPtr(), tri_mesh->size() * sizeof(TriangleMesh::Triangle));
          gpu_queue.prefetch((Vec3fa*)tri_mesh->vertices0.getPtr()                , tri_mesh->numVertices() * sizeof(Vec3fa));
        }
        else if (quad_mesh)
        {
          triQuadMesh[geomID] = TriQuadMesh(quad_mesh->size(),quad_mesh->numVertices(),(QuadMesh::Quad*)quad_mesh->quads.getPtr(),(Vec3fa*)quad_mesh->vertices0.getPtr(),true);
          gpu_queue.prefetch((QuadMesh::Quad*)quad_mesh->quads.getPtr(), quad_mesh->size() * sizeof(QuadMesh::Quad));
          gpu_queue.prefetch((Vec3fa*)quad_mesh->vertices0.getPtr()    , quad_mesh->numVertices() * sizeof(Vec3fa));        
        }
        else
        {
          triQuadMesh[geomID] = TriQuadMesh(0,0,nullptr,nullptr);
        }
      }
    }
    
    if (unlikely(verbose2)) PRINT2(numGeoms,org_numPrimitives);
    
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

    if (unlikely(verbose2)) std::cout << "Dummy first kernel launch (should trigger all USM transfers) " << (first_kernel_time1-first_kernel_time0)*1000.0f << " ms " << std::endl;
    
    uint numActiveQuads = 0;
    
    if (activeTriQuadMeshes)    
    {      
      // =============================
      // === count quads per block === 
      // =============================

      timer.start(BuildTimer::PRE_PROCESS);

      double device_quadification_time = 0.0f;
    
      countQuadsPerGeometry(gpu_queue,triQuadMesh,numGeoms,prims_per_geom_prefix_sum,device_quadification_time,verbose2);

      timer.stop(BuildTimer::PRE_PROCESS);
      timer.add_to_device_timer(BuildTimer::PRE_PROCESS,device_quadification_time);
                              
      if (unlikely(verbose2)) std::cout << "Count quads: " << timer.get_host_timer() << " ms (host) " << device_quadification_time << " ms (device) " << std::endl;      

      // =============================      
      // === prefix sum over quads ===
      // =============================      
    
      timer.start(BuildTimer::PRE_PROCESS);
      double geom_prefix_sum_time = 0.0f;
     
      prefixsumOverGeometryCounts(gpu_queue,numGeoms,prims_per_geom_prefix_sum,host_device_tasks,geom_prefix_sum_time,verbose2);
    
      timer.stop(BuildTimer::PRE_PROCESS);
      timer.add_to_device_timer(BuildTimer::PRE_PROCESS,geom_prefix_sum_time);
      numActiveQuads = *host_device_tasks;
    }
    
    const uint numPrimitives = numActiveQuads + numInstances + numUserGeometries; 
    
    if (unlikely(verbose1))    
      PRINT5(org_numPrimitives,numPrimitives,numActiveQuads,numUserGeometries,numInstances);

    // ==========================================================
    // ==========================================================
    // ==========================================================
    

    const bool fastMCMode = numPrimitives < FAST_MC_THRESHOLD;
            
    /* --- estimate size of the BVH --- */
    const uint header              = 128;
    const uint node_size           = estimateSizeInternalNodes(numActiveQuads,numInstances,numUserGeometries);
    const uint leaf_size           = estimateSizeLeafNodes(numActiveQuads,numInstances,numUserGeometries); 
    const uint totalSize           = header + node_size + leaf_size; 
    const uint node_data_start     = header;
    const uint leaf_data_start     = header + node_size;

    assert( (leaf_data_start % 64) == 0 );
    
    if (unlikely(verbose2))
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

    
    if (accel.size() < totalSize) accel.resize(totalSize); //accel = std::move(Device::avector<char,64>(scene->device,totalSize));    
    gpu_queue.prefetch((char*)accel.data(),totalSize);

    sizeTotalAllocations += totalSize;
    
    QBVH6* qbvh   = (QBVH6*)accel.data();
    assert(qbvh);
    char *bvh_mem = (char*)accel.data() + header;
    assert(bvh_mem);
    const size_t conv_mem_size = sizeof(LeafGenerationData)*numPrimitives;

    uint *scratch_mem1 = (uint*)sycl::aligned_alloc(64,conv_mem_size,deviceGPU->getGPUDevice(),deviceGPU->getGPUContext(),sycl::usm::alloc::shared); // FIXME device
    LeafGenerationData *leafGenData = (LeafGenerationData*)scratch_mem1;
    assert(conversionState);

    sizeTotalAllocations += conv_mem_size;
        
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
          
    if (unlikely(verbose2))
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
                                                                      globals->numBuildRecords          = 0;
                                                                    });
                                                  });
      gpu::waitOnQueueAndCatchException(gpu_queue);
      double dt = gpu::getDeviceExecutionTiming(queue_event);
      timer.add_to_device_timer(BuildTimer::PRE_PROCESS,dt);
      
      if (unlikely(verbose2))
        std::cout << "Init globals " << dt << " ms" << std::endl;
    }	    

    // ===================================================          
    // ==== merge triangles to quads, create primrefs ====
    // ===================================================

    timer.start(BuildTimer::PRE_PROCESS);        
    
    double create_primref_time = 0.0f;

    if (activeTriQuadMeshes)
      createQuads_initPLOCPrimRefs(gpu_queue,triQuadMesh,numGeoms,prims_per_geom_prefix_sum,bvh2,0,create_primref_time,verbose2);

    if (numUserGeometries)
      createUserGeometries_initPLOCPrimRefs(gpu_queue,scene,numGeoms,bvh2,numActiveQuads,create_primref_time,verbose2);

    if (numInstances)
      createInstances_initPLOCPrimRefs(gpu_queue,scene,numGeoms,bvh2,numActiveQuads + numUserGeometries,create_primref_time,verbose2);
    
    const GeometryTypeRanges geometryTypeRanges(numActiveQuads,numUserGeometries,numInstances);        
    
    timer.stop(BuildTimer::PRE_PROCESS);
    timer.add_to_device_timer(BuildTimer::PRE_PROCESS,create_primref_time);
    
    if (unlikely(verbose2)) std::cout << "create quads/userGeometries/instances etc, init primrefs: " << timer.get_host_timer() << " ms (host) " << create_primref_time << " ms (device) " << std::endl;
                
        
    // ==========================================          
    // ==== get centroid and geometry bounds ====
    // ==========================================
        
     timer.start(BuildTimer::PRE_PROCESS);        
     double device_compute_centroid_bounds_time = 0.0f;
     
     computeCentroidGeometryBounds(gpu_queue, &globals->geometryBounds, &globals->centroidBounds, bvh2, numPrimitives, device_compute_centroid_bounds_time, verbose2);
        
     timer.stop(BuildTimer::PRE_PROCESS);
     timer.add_to_device_timer(BuildTimer::PRE_PROCESS,device_compute_centroid_bounds_time);


    if (unlikely(verbose2))
      std::cout << "Get Geometry and Centroid Bounds Phase " << timer.get_host_timer() << " ms (host) " << device_compute_centroid_bounds_time << " ms (device) " << std::endl;		
       
    // ==============================          
    // ==== compute morton codes ====
    // ==============================

    timer.start(BuildTimer::PRE_PROCESS);        
    double device_compute_mc_time = 0.0f;

    if (!fastMCMode)
      computeMortonCodes64Bit_SaveMSBBits(gpu_queue,&globals->centroidBounds,mc0,bvh2,bvh2_subtree_size,numPrimitives,device_compute_mc_time,verbose2);
    else
      computeMortonCodes64Bit(gpu_queue,&globals->centroidBounds,(gpu::MortonCodePrimitive40x24Bits3D*)mc1,bvh2,numPrimitives,0,(uint64_t)-1,device_compute_mc_time,verbose2);
    
        
    timer.stop(BuildTimer::PRE_PROCESS);
     
    if (unlikely(verbose2))
      std::cout << "Compute Morton Codes " << timer.get_host_timer() << " ms (host) " << device_compute_mc_time << " ms (device) " << std::endl;		
    
    // ===========================          
    // ==== sort morton codes ====
    // ===========================

    timer.start(BuildTimer::PRE_PROCESS);        

    double sort_time = 0.0;

    const bool sync_sort = false;
    
    if (!fastMCMode)
    {
      const uint scratchMemWGs = gpu::getNumWGsScratchSize(conv_mem_size);
      const uint nextPowerOf2 =  1 << (32 - sycl::clz(numPrimitives) - 1);
      const uint sortWGs = min(max(min((int)nextPowerOf2/8192,(int)gpu_maxComputeUnits/4 /*RADIX_SORT_MAX_NUM_DSS*/ ),1),(int)scratchMemWGs);
      if (unlikely(verbose2))      
        PRINT2(scratchMemWGs,sortWGs);
     
      for (uint i=4;i<8;i++) 
        gpu::sort_iteration_type<MCPrim>(gpu_queue, morton_codes[i%2], morton_codes[(i+1)%2], numPrimitives, scratch_mem1, i, sort_time, sortWGs, sync_sort);
      gpu::waitOnQueueAndCatchException(gpu_queue);      
      
      restoreMSBBits(gpu_queue,mc0,bvh2_subtree_size,numPrimitives,sort_time,verbose2);      

      for (uint i=4;i<8;i++) 
        gpu::sort_iteration_type<MCPrim>(gpu_queue, morton_codes[i%2], morton_codes[(i+1)%2], numPrimitives, scratch_mem1, i, sort_time, sortWGs, sync_sort);
    }
    else
    {
      if (numPrimitives < SMALL_SORT_THRESHOLD)
        gpu::radix_sort_single_workgroup(gpu_queue, (uint64_t *)mc0, (uint64_t *)mc1, numPrimitives, 3,8, sort_time);
      else
      {
        const uint scratchMemWGs = gpu::getNumWGsScratchSize(conv_mem_size);        
        const uint nextPowerOf2 =  1 << (32 - sycl::clz(numPrimitives) - 1);          
        const uint sortWGs = min(max(min((int)nextPowerOf2/1024,(int)gpu_maxComputeUnits/4),1),(int)scratchMemWGs);
        if (unlikely(verbose2))        
          PRINT3(conv_mem_size,scratchMemWGs,sortWGs);
        for (uint i=3;i<8;i++) 
          gpu::sort_iteration_type<gpu::MortonCodePrimitive40x24Bits3D>(gpu_queue, (gpu::MortonCodePrimitive40x24Bits3D*)morton_codes[i%2], (gpu::MortonCodePrimitive40x24Bits3D*)morton_codes[(i+1)%2], numPrimitives, scratch_mem1, i, sort_time, sortWGs, sync_sort);        
      }      
    }
      
    gpu::waitOnQueueAndCatchException(gpu_queue);

    timer.stop(BuildTimer::PRE_PROCESS);        
    timer.add_to_device_timer(BuildTimer::PRE_PROCESS,timer.get_host_timer());
            
    if (unlikely(verbose2))
      std::cout << "Sort Morton Codes " << timer.get_host_timer() << " ms (host and device)" << std::endl;
        
              
    // ===========================          
    // ====== init clusters ======
    // ===========================

    
    timer.start(BuildTimer::PRE_PROCESS);        
    double device_init_clusters_time = 0.0f;

    if (!fastMCMode)
      initClusters(gpu_queue,mc0,bvh2,cluster_index,bvh2_subtree_size,numPrimitives,device_init_clusters_time,verbose2);
    else
      initClusters(gpu_queue,(gpu::MortonCodePrimitive40x24Bits3D*)mc0,bvh2,cluster_index,bvh2_subtree_size,numPrimitives,device_init_clusters_time,verbose2); 
    
    timer.stop(BuildTimer::PRE_PROCESS);        
    timer.add_to_device_timer(BuildTimer::PRE_PROCESS,device_init_clusters_time);
        
    if (unlikely(verbose2))
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
                    
      gpu::Range *ranges = (gpu::Range*)scratch_mem1;         
      extractRanges(gpu_queue, host_device_tasks, mc0, ranges, numPrims, RANGE_THRESHOLD , device_ploc_iteration_time, verbose2);
      const uint numRanges = *host_device_tasks;      
      gpu::radix_sort_single_workgroup(gpu_queue, (uint64_t*)ranges, (uint64_t*)ranges + numRanges, numRanges,0,8,device_ploc_iteration_time);      
      parallelWGBuild(gpu_queue, globals, bvh2, cluster_index_source, cluster_index_dest, bvh2_subtree_size, ranges, numRanges, BOTTOM_UP_THRESHOLD, device_ploc_iteration_time, verbose2);      
      singleWGTopLevelBuild(gpu_queue, globals, bvh2, cluster_index_source, cluster_index_dest, bvh2_subtree_size, ranges, numRanges, SEARCH_RADIUS_TOP_LEVEL, device_ploc_iteration_time, verbose2);
      timer.add_to_device_timer(BuildTimer::BUILD,device_ploc_iteration_time);
    }
    else
    {
      // ===========================            
      // ==== clear scratch mem ====
      // ===========================      
      {
        const sycl::nd_range<1> nd_range1(MAX_WGS,sycl::range<1>(MAX_WGS)); 
        sycl::event queue_event = gpu_queue.submit([&](sycl::handler &cgh) {
                                                     cgh.parallel_for(nd_range1,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(16)
                                                                      {
                                                                        const uint globalID     = item.get_global_id(0);
                                                                        scratch_mem1[globalID] = 0;
                                                                      });                                                         
                                                   });
      }
      
      for (;numPrims>1;iteration++)
      {          
        // ==================================================            
        // ==== single kernel path if #prims < threshold ====
        // ==================================================

        if (numPrims < SINGLE_WG_SWITCH_THRESHOLD)
        {
          double singleWG_time = 0.0f;
          singleWGBuild(gpu_queue, globals, bvh2, cluster_index_source, cluster_index_dest, bvh2_subtree_size, numPrims, singleWG_time, verbose2);
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
          iteratePLOC(gpu_queue,globals,bvh2,cluster_index_source,cluster_index_dest,bvh2_subtree_size,scratch_mem1,numPrims,radius,MERGED_KERNEL_WG_NUM,host_device_tasks,device_ploc_iteration_time, false);
          timer.add_to_device_timer(BuildTimer::BUILD,device_ploc_iteration_time);

          const uint new_numPrims = *host_device_tasks;
          assert(new_numPrims < numPrims);          
          numPrims = new_numPrims;          
            
          // ==========================            
        }        
        if (unlikely(verbose2))
          PRINT4(iteration,numPrims,(float)device_ploc_iteration_time,(float)timer.get_accum_device_timer(BuildTimer::BUILD));
      }
    }
    timer.stop(BuildTimer::BUILD);        
            
    
    // =====================================                
    // === check and convert BVH2 (host) ===
    // =====================================

    if (unlikely(verbose2))
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

    // =============================    
    // === convert BVH2 to QBVH6 ===
    // =============================
    
    const float conversion_device_time = convertBVH2toQBVH6(gpu_queue,globals,host_device_tasks,triQuadMesh,scene,qbvh,bvh2,leafGenData,numPrimitives,numInstances != 0,geometryTypeRanges,verbose2);

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

    if (unlikely(verbose2))
      std::cout << "BVH2 -> QBVH6 Flattening DONE in " <<  timer.get_host_timer() << " ms (host) " << conversion_device_time << " ms (device) " << std::endl << std::flush;

    // ==========================================================    
    // ==========================================================
    // ==========================================================

    if (unlikely(verbose2))
    {
      // === memory allocation and usage stats ===
      PRINT(globals->node_mem_allocator_start);      
      PRINT(globals->node_mem_allocator_cur);
      PRINT(globals->node_mem_allocator_cur-globals->node_mem_allocator_start);
      PRINT(100.0f * (float)(globals->node_mem_allocator_cur-globals->node_mem_allocator_start) / (node_size/64));
      PRINT(globals->leaf_mem_allocator_start);      
      PRINT(globals->leaf_mem_allocator_cur);
      PRINT(globals->leaf_mem_allocator_cur-globals->leaf_mem_allocator_start);
      PRINT(100.0f * (float)(globals->leaf_mem_allocator_cur-globals->leaf_mem_allocator_start) / (leaf_size/64));      
      PRINT(globals->numLeaves);

      if ((globals->node_mem_allocator_cur-globals->node_mem_allocator_start)*64 > node_size ||
          (globals->leaf_mem_allocator_cur-globals->leaf_mem_allocator_start)*64 > leaf_size) FATAL("NOT ENOUGH MEMORY FOR INTERNAL NODES ALLOCATED");      
    }
    
    BBox3f geomBounds(Vec3fa(globals->geometryBounds.lower_x,globals->geometryBounds.lower_y,globals->geometryBounds.lower_z),
                      Vec3fa(globals->geometryBounds.upper_x,globals->geometryBounds.upper_y,globals->geometryBounds.upper_z));
    
    if (deviceGPU) {
#if 1      
      HWAccel* hwaccel = (HWAccel*) accel.data();
      hwaccel->dispatchGlobalsPtr = (uint64_t)dispatchGlobalsPtr;
      PRINT2(hwaccel,dispatchGlobalsPtr);
#else
      EmbreeHWAccel* hwaccel = (EmbreeHWAccel*) accel.data();
      hwaccel->numTimeSegments = 1;
      hwaccel->AccelTable[0] = (char*)hwaccel + 128;
#endif      
    }
    
    timer.start(BuildTimer::ALLOCATION);        

    if (scratch_mem0)      sycl::free(scratch_mem0,deviceGPU->getGPUContext());    
    if (globals)           sycl::free(globals,deviceGPU->getGPUContext());
    if (scratch_mem1)      sycl::free(scratch_mem1,deviceGPU->getGPUContext());
    if (host_device_tasks) sycl::free(host_device_tasks,deviceGPU->getGPUContext());

    timer.stop(BuildTimer::ALLOCATION);        

    // ======================    
    // === Verbose Output ===
    // ======================
    
    if (unlikely(verbose2))
      std::cout << "Time freeing temporary data " << timer.get_host_timer()  << " ms " << std::endl << std::flush;

    if (unlikely(verbose1))
    {
      const float total_host   = timer.get_total_host_time();
      const float total_device = timer.get_total_device_time();
      
      std::cout << "BVH2 GPU Ploc Builder DONE in " << total_host << " ms (host), " << total_device << " ms (device) => Quads Build : " << numPrimitives*0.001f/total_host << " MPrims/s (host) " << numPrimitives*0.001f/total_device << " MPrims/s (device) / Original Tris : " << org_numPrimitives*0.001f/total_host << " MPrims/s (host) " <<  org_numPrimitives*0.001f/total_device << " MPrims/s (device) " << std::endl << std::flush;
      std::cout << "Allocation    " << timer.get_accum_host_timer(BuildTimer::ALLOCATION) << " ms (host) for " << (float)sizeTotalAllocations / (1024*1024) << " MB => " << (float)sizeTotalAllocations / (1024*1024) * 1000 / timer.get_accum_host_timer(BuildTimer::ALLOCATION) <<  " MB/s " << std::endl;
      std::cout << "Pre-process   " << timer.get_accum_host_timer(BuildTimer::PRE_PROCESS) << " ms (host) " << timer.get_accum_device_timer(BuildTimer::PRE_PROCESS) << " ms (device) , ratio " << timer.get_accum_host_timer(BuildTimer::PRE_PROCESS) / timer.get_accum_device_timer(BuildTimer::PRE_PROCESS) << std::endl;
      std::cout << "Build         " << timer.get_accum_host_timer(BuildTimer::BUILD) << " ms (host) " << timer.get_accum_device_timer(BuildTimer::BUILD) << " ms (device) , ratio " << timer.get_accum_host_timer(BuildTimer::BUILD) / timer.get_accum_device_timer(BuildTimer::BUILD) << std::endl;
      std::cout << "Post-process  " << timer.get_accum_host_timer(BuildTimer::POST_PROCESS) << " ms (host) " << timer.get_accum_device_timer(BuildTimer::POST_PROCESS) << " ms (device) , ratio " << timer.get_accum_host_timer(BuildTimer::POST_PROCESS) / timer.get_accum_device_timer(BuildTimer::POST_PROCESS) << std::endl;
    }

    // ========================    
    // === Additional Stats ===
    // ========================

#if 0
    if (unlikely(verbose2))
    {
      if (numInstances != 0) qbvh->print(std::cout,qbvh->root(),0,6);
      //BVHStatistics stats = qbvh->computeStatistics();      
      //stats.print(std::cout);
      //stats.print_raw(std::cout);
      //PRINT("VERBOSE STATS DONE");
    }
#endif
    
    return geomBounds;
  }
 
}


