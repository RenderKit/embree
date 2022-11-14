#include "rthwif_embree.h"
#include "rthwif_embree_builder.h"
#include "rthwif_internal.h"
#include "rthwif_embree_builder_ploc.h"
#include "builder/qbvh6.h"
#include "../common/algorithms/parallel_reduce.h"

#define SINGLE_WG_SWITCH_THRESHOLD 8*1024
#define FAST_MC_THRESHOLD          1024*1024
#define SMALL_SORT_THRESHOLD       1024*4

static const float ESTIMATED_QUADIFICATION_FACTOR = 0.58;

#if defined(EMBREE_SYCL_GPU_BVH_BUILDER)      

namespace embree
{
  using namespace embree::isa;

  __forceinline uint estimateSizeInternalNodes(const uint numQuads, const uint numInstances, const uint numProcedurals)
  {
    const uint N = numQuads + numInstances + numProcedurals; 
    // === conservative estimate ===
    const uint numFatLeaves = ceilf( (float)N/2 ) + ceilf( (float)numInstances/2 ); // FIXME : better upper bound for instance case
    const uint numInnerNodes = ceilf( (float)numFatLeaves/5 ); 
    return gpu::alignTo(std::max( ((numFatLeaves + numInnerNodes) * 64) , N * 16),64);
  }

  __forceinline uint estimateSizeLeafNodes(const uint numQuads, const uint numInstances, const uint numProcedurals)
  {
    return (numQuads + numProcedurals + 2 * numInstances) * 64;  
  }

  __forceinline uint estimateAccelBufferSize(const uint numQuads, const uint numInstances, const uint numProcedurals)
  {
    const uint header              = 128;
    const uint node_size           = estimateSizeInternalNodes(numQuads,numInstances,numProcedurals);
    const uint leaf_size           = estimateSizeLeafNodes(numQuads,numInstances,numProcedurals); 
    const uint totalSize           = header + node_size + leaf_size;
    return totalSize;
  }

  __forceinline uint estimateScratchBufferSize(const uint numPrimitives)
  {
    return sizeof(PLOCGlobals) + sizeof(uint)*MAX_WGS + numPrimitives * sizeof(LeafGenerationData);
  }
  
  void checkBVH2PlocHW(BVH2Ploc *bvh2, uint index,uint &nodes,uint &leaves,float &nodeSAH, float &leafSAH, const uint numPrimitives, const uint bvh2_max_allocations)
  {
    if (bvh2[index].bounds.empty()) {
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
      
      if (!bvh2[index].bounds.encloses( bvh2[ bvh2[index].leftIndex() ].bounds )) PRINT2("ENCLOSING ERROR LEFT",index);
      checkBVH2PlocHW(bvh2,bvh2[index].leftIndex(),nodes,leaves,nodeSAH,leafSAH,numPrimitives,bvh2_max_allocations);

      if (!bvh2[index].bounds.encloses( bvh2[ bvh2[index].rightIndex() ].bounds )) PRINT2("ENCLOSING ERROR RIGHT",index);
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
  
  __forceinline PrimitiveCounts countPrimitives(const RTHWIF_GEOMETRY_DESC** geometries, const uint numGeometries)
  {
    auto reduce = [&](const range<size_t>& r) -> PrimitiveCounts
                  {
                    PrimitiveCounts counts;
                    for (size_t geomID = r.begin(); geomID < r.end(); geomID++)
                    {
                      const RTHWIF_GEOMETRY_DESC* geom = geometries[geomID];
                      if (geom == nullptr) continue;    
                      switch (geom->geometryType) {
                      case RTHWIF_GEOMETRY_TYPE_TRIANGLES  : counts.numTriangles   += ((RTHWIF_GEOMETRY_TRIANGLES_DESC*)  geom)->triangleCount; break;
                      case RTHWIF_GEOMETRY_TYPE_QUADS      : counts.numQuads       += ((RTHWIF_GEOMETRY_QUADS_DESC*)      geom)->quadCount; break;
                      case RTHWIF_GEOMETRY_TYPE_AABBS_FPTR : counts.numProcedurals += ((RTHWIF_GEOMETRY_AABBS_FPTR_DESC*) geom)->primCount; break;
                      case RTHWIF_GEOMETRY_TYPE_INSTANCE   : counts.numInstances   += 1; break;
                      default: assert(false); break;        
                      };                    
                    };
                    return counts;
                  };

    const uint COUNT_BLOCK_SIZE = 256;
    const uint COUNT_PARALLEL_THRESHOLD = 256;
    
    const PrimitiveCounts primCounts = parallel_reduce((uint)0, numGeometries, COUNT_BLOCK_SIZE, COUNT_PARALLEL_THRESHOLD, PrimitiveCounts(), reduce,
                                                       [&](const PrimitiveCounts& b0, const PrimitiveCounts& b1) -> PrimitiveCounts { return b0 + b1; });
    return primCounts;
  }

  RTHWIF_ERROR createEmptyBVH(const RTHWIF_BUILD_ACCEL_ARGS& args, sycl::queue  &gpu_queue)
  {
    sycl::event queue_event =  gpu_queue.submit([&](sycl::handler &cgh) {
                                                  cgh.single_task([=]() {
                                                                    QBVH6* qbvh  = (QBVH6*)args.accelBuffer;       
                                                                    qbvh->bounds = BBox3f(empty);
                                                                    qbvh->numPrims       = 0;                                                                        
                                                                    qbvh->nodeDataStart  = 2;
                                                                    qbvh->nodeDataCur    = 3;
                                                                    qbvh->leafDataStart  = 3;
                                                                    qbvh->leafDataCur    = 3;        
                                                                    new (qbvh->nodePtr(2)) QBVH6::InternalNode6(NODE_TYPE_INTERNAL);
                                                                  });
                                                });
    gpu::waitOnEventAndCatchException(queue_event);    
    if (args.accelBufferBytesOut) *args.accelBufferBytesOut = 128+64;
    if (args.boundsOut)           { BBox3f geometryBounds (empty); *args.boundsOut = *(RTHWIF_AABB*)&geometryBounds; };
    return RTHWIF_ERROR_NONE;    
  }
// =================================================================================================================================================================================
// =================================================================================================================================================================================
// =================================================================================================================================================================================
  

  RTHWIF_API RTHWIF_ERROR rthwifGetAccelSizeGPU(const RTHWIF_BUILD_ACCEL_ARGS& args_i, RTHWIF_ACCEL_SIZE& size_o)
  {
    RTHWIF_BUILD_ACCEL_ARGS args = rthwifPrepareBuildAccelArgs(args_i);
    const RTHWIF_GEOMETRY_DESC** geometries = args.geometries;
    const uint numGeometries = args.numGeometries;
    
    const PrimitiveCounts primCounts = countPrimitives(geometries,numGeometries);

    const uint numQuads       = primCounts.numQuads;
    const uint numTriangles   = primCounts.numTriangles;
    const uint numProcedurals = primCounts.numProcedurals;
    const uint numInstances   = primCounts.numInstances;
  
    const uint numPrimitives = numQuads + numTriangles + numProcedurals + numInstances;

    // =============================================    
    // === allocation for empty scene is default ===
    // =============================================
    
    size_t expectedBytes = 3*64; 
    size_t worstCaseBytes = 4*64;

    if (numPrimitives)
    {    
      expectedBytes  = estimateAccelBufferSize(numQuads + ceilf(numTriangles * ESTIMATED_QUADIFICATION_FACTOR), numInstances, numProcedurals);
      worstCaseBytes = estimateAccelBufferSize(numQuads + numTriangles, numInstances, numProcedurals);    
    }

    // ===============================================    
    // === estimate accel and scratch buffer sizes ===
    // ===============================================
    
    const size_t scratchBytes = estimateScratchBufferSize(std::max(numPrimitives,numGeometries));

    if (args_i.verbose == 2)
    {
      PRINT4(numTriangles,numQuads,numProcedurals,numInstances);      
      PRINT2(numTriangles,ceilf(numTriangles * ESTIMATED_QUADIFICATION_FACTOR));          
      PRINT3(expectedBytes,worstCaseBytes,scratchBytes);
    }
        
    /* return size to user */
    RTHWIF_ACCEL_SIZE size;
    memset(&size,0,sizeof(RTHWIF_ACCEL_SIZE));
    size.accelBufferExpectedBytes = expectedBytes;
    size.accelBufferWorstCaseBytes = worstCaseBytes;
    size.scratchBufferBytes = scratchBytes;
    size_t bytes_o = size_o.structBytes;
    memset(&size_o,0,bytes_o);
    memcpy(&size_o,&size,bytes_o);
    size_o.structBytes = bytes_o;
    return RTHWIF_ERROR_NONE;
  }

  RTHWIF_API RTHWIF_ERROR rthwifPrefetchAccelGPU(const RTHWIF_BUILD_ACCEL_ARGS& args)
  {
    double time0 = getSeconds();
    
    sycl::queue  &gpu_queue  = *(sycl::queue*)args.sycl_queue;
    const RTHWIF_GEOMETRY_DESC** geometries = args.geometries;
    const uint numGeometries                = args.numGeometries;  

    // ===================================    
    // === prefetch builder scene data ===
    // ===================================
    
    for (size_t geomID = 0; geomID < numGeometries; geomID++)
    {
      const RTHWIF_GEOMETRY_DESC* geom = geometries[geomID];
      if (geom == nullptr) continue;    
      switch (geom->geometryType) {
      case RTHWIF_GEOMETRY_TYPE_TRIANGLES  :
      {
        RTHWIF_GEOMETRY_TRIANGLES_DESC *t = (RTHWIF_GEOMETRY_TRIANGLES_DESC*)geom;
        if (t->vertexBuffer)   gpu_queue.prefetch(t->vertexBuffer,t->vertexCount*t->vertexStride);
        if (t->triangleBuffer) gpu_queue.prefetch(t->triangleBuffer,t->triangleCount*t->triangleStride);      
        gpu_queue.prefetch(t,sizeof(RTHWIF_GEOMETRY_TRIANGLES_DESC));
        break;
      }
      case RTHWIF_GEOMETRY_TYPE_QUADS      :
      {
        RTHWIF_GEOMETRY_QUADS_DESC *q = (RTHWIF_GEOMETRY_QUADS_DESC*)geom;
        if (q->vertexBuffer) gpu_queue.prefetch(q->vertexBuffer,q->vertexCount*q->vertexStride);
        if (q->quadBuffer) gpu_queue.prefetch(q->quadBuffer,q->quadCount*q->quadStride);      
        gpu_queue.prefetch(q,sizeof(RTHWIF_GEOMETRY_QUADS_DESC));      
        break;
      }
      case RTHWIF_GEOMETRY_TYPE_AABBS_FPTR :
      {
        RTHWIF_GEOMETRY_AABBS_FPTR_DESC *a = (RTHWIF_GEOMETRY_AABBS_FPTR_DESC*)geom;
        gpu_queue.prefetch(a,sizeof(RTHWIF_GEOMETRY_AABBS_FPTR_DESC));
        break;
      }
      case RTHWIF_GEOMETRY_TYPE_INSTANCE   :
      {
        RTHWIF_GEOMETRY_INSTANCE_DESC *i = (RTHWIF_GEOMETRY_INSTANCE_DESC*)geom;
        gpu_queue.prefetch(i->bounds,sizeof(RTHWIF_AABB));      
        gpu_queue.prefetch(i,sizeof(RTHWIF_GEOMETRY_INSTANCE_DESC));
        break;
      }
      default: assert(false); break;        
      };                    
    };

    if (geometries) gpu_queue.prefetch(geometries,sizeof(RTHWIF_GEOMETRY_DESC*)*numGeometries);
    if (args.accelBuffer)   gpu_queue.prefetch(args.accelBuffer  ,args.accelBufferBytes);
    if (args.scratchBuffer) gpu_queue.prefetch(args.scratchBuffer,args.scratchBufferBytes);  
  
    // ======================================================    
    // === DUMMY KERNEL TO TRIGGER REMAINING USM TRANSFER ===
    // ======================================================
    
    sycl::event queue_event =  gpu_queue.submit([&](sycl::handler &cgh) { cgh.single_task([=]() {}); });
    gpu::waitOnEventAndCatchException(queue_event);

    double time1 = getSeconds();
    if (args.verbose == 1)
      std::cout << "Device Prefetch Time " << (float)(time1-time0)*1000.0f << " ms" << std::endl;
    
    return RTHWIF_ERROR_NONE;      
  }

  RTHWIF_API RTHWIF_ERROR rthwifBuildAccelGPU(const RTHWIF_BUILD_ACCEL_ARGS& args)
  {
    BuildTimer timer;
    timer.reset();

    timer.start(BuildTimer::PRE_PROCESS);      
    
    // ================================    
    // === GPU device/queue/context ===
    // ================================
  
    sycl::queue  &gpu_queue  = *(sycl::queue*)args.sycl_queue;
    sycl::device &gpu_device = *(sycl::device*)args.sycl_device;
    const bool verbose1 = args.verbose >= 1;    
    const bool verbose2 = args.verbose >= 2;
    const uint gpu_maxComputeUnits  = gpu_device.get_info<sycl::info::device::max_compute_units>();    
    uint *host_device_tasks = (uint*)args.hostDeviceCommPtr;
  
    if (unlikely(verbose2))
    {
      const uint gpu_maxWorkGroupSize = gpu_device.get_info<sycl::info::device::max_work_group_size>();
      const uint gpu_maxLocalMemory   = gpu_device.get_info<sycl::info::device::local_mem_size>();    
      PRINT("PLOC++ GPU BVH BUILDER");            
      PRINT( gpu_device.get_info<sycl::info::device::global_mem_size>() );
      PRINT(gpu_maxWorkGroupSize);
      PRINT(gpu_maxComputeUnits);
      PRINT(gpu_maxLocalMemory);
    }

    // =============================    
    // === setup scratch pointer ===
    // =============================
    
    PLOCGlobals *globals = (PLOCGlobals *)args.scratchBuffer;
    uint *const sync_mem = (uint*)((char*)args.scratchBuffer + sizeof(PLOCGlobals));
    uint *const scratch  = (uint*)((char*)args.scratchBuffer + sizeof(PLOCGlobals) + sizeof(uint)*MAX_WGS);    
    uint *const prims_per_geom_prefix_sum = (uint*)scratch;    
  
    // ======================          
    // ==== init globals ====
    // ======================
    {
      sycl::event queue_event =  gpu_queue.submit([&](sycl::handler &cgh) {
                                                    cgh.single_task([=]() {
                                                                      globals->reset();
                                                                    });
                                                  });
      gpu::waitOnEventAndCatchException(queue_event);
      double dt = gpu::getDeviceExecutionTiming(queue_event);
      timer.add_to_device_timer(BuildTimer::PRE_PROCESS,dt);
      
      if (unlikely(verbose2))
        std::cout << "Init Globals " << dt << " ms" << std::endl;
    }  
  
    // ================================================    
    // === get primitive type count from geometries ===
    // ================================================
  
    const RTHWIF_GEOMETRY_DESC** geometries = args.geometries;
    const uint numGeometries                = args.numGeometries;
 
    double device_prim_counts_time = 0.0f;
  
    const PrimitiveCounts primCounts = countPrimitives(gpu_queue,geometries,numGeometries,globals,host_device_tasks,device_prim_counts_time,verbose2); 

    // ================================================
    
    timer.stop(BuildTimer::PRE_PROCESS);
    timer.add_to_device_timer(BuildTimer::PRE_PROCESS,device_prim_counts_time);                              
    if (unlikely(verbose2)) std::cout << "Count primitives from geometries: " << timer.get_host_timer() << " ms (host) " << device_prim_counts_time << " ms (device) " << std::endl;      
  
    uint numQuads       = primCounts.numQuads + primCounts.numTriangles; // no quadification taken into account at this point
    uint numProcedurals = primCounts.numProcedurals;
    uint numInstances   = primCounts.numInstances;

    const uint expected_numPrimitives = numQuads + numProcedurals + numInstances;    

    // =================================================    
    // === empty scene before removing invalid prims ===
    // =================================================
    
    if (unlikely(expected_numPrimitives == 0)) createEmptyBVH(args,gpu_queue);
        
    if (numQuads)
    {
      // =====================================
      // === compute correct quadification === 
      // =====================================

      timer.start(BuildTimer::PRE_PROCESS);

      double device_quadification_time = 0.0f;
    
      countQuadsPerGeometry(gpu_queue,args.geometries,numGeometries,prims_per_geom_prefix_sum,device_quadification_time,verbose2);

      timer.stop(BuildTimer::PRE_PROCESS);
      timer.add_to_device_timer(BuildTimer::PRE_PROCESS,device_quadification_time);
                              
      if (unlikely(verbose2)) std::cout << "Count quads: " << timer.get_host_timer() << " ms (host) " << device_quadification_time << " ms (device) " << std::endl;      

      // ================================================      
      // === prefix sum over quad counts per geometry ===
      // ================================================      
    
      timer.start(BuildTimer::PRE_PROCESS);
      double geom_prefix_sum_time = 0.0f;
     
      prefixSumOverGeometryCounts(gpu_queue,numGeometries,prims_per_geom_prefix_sum,host_device_tasks,geom_prefix_sum_time,verbose2);
    
      timer.stop(BuildTimer::PRE_PROCESS);
      timer.add_to_device_timer(BuildTimer::PRE_PROCESS,geom_prefix_sum_time);
      
      numQuads = *host_device_tasks; // numQuads contains now the actual number of quads after quadification
    }


    // ================================
    // === estimate size of the BVH ===
    // ================================

    uint numPrimitives             = numQuads + numInstances + numProcedurals;  // actual #prims can be lower due to invalid instances or procedurals but quads count is accurate at this point
    const uint allocated_size      = args.accelBufferBytes;
    const uint header              = 128;
    const uint leaf_size           = estimateSizeLeafNodes(numQuads,numInstances,numProcedurals);
    const uint node_size           = (header + leaf_size) <= allocated_size ? allocated_size - leaf_size - header : 0; 
    const uint node_data_start     = header;
    const uint leaf_data_start     = header + node_size;
    
    if (unlikely(verbose2))
    {
      PRINT4(numQuads,numProcedurals,numInstances,expected_numPrimitives);
      PRINT3(node_size,leaf_size,args.accelBufferBytes);
      PRINT2(node_size/64,leaf_size/64);      
    }
  
    // =================================================================
    // === if allocated accel buffer is too small, return with error ===
    // =================================================================

    const uint required_size = header + estimateSizeInternalNodes(numQuads,numInstances,numProcedurals) + leaf_size;
    if (allocated_size < required_size)
    {
      if (unlikely(args.verbose))
      {
        PRINT2(required_size,allocated_size);
        PRINT2(node_size,estimateSizeInternalNodes(numQuads,numInstances,numProcedurals));        
        PRINT3("RETRY BVH BUILD DUE BECAUSE OF SMALL ACCEL BUFFER ALLOCATION!!!", args.accelBufferBytes,required_size );
      }
      if (args.accelBufferBytesOut) *args.accelBufferBytesOut = required_size;
      return RTHWIF_ERROR_RETRY;
    }

    const bool fastMCMode = numPrimitives < FAST_MC_THRESHOLD;
    const size_t conv_mem_size = sizeof(numPrimitives)*numPrimitives;
    const uint NUM_ACTIVE_LARGE_WGS = min((numPrimitives+1024-1)/1024,(uint)MAX_WGS);

    // ===========================
    // === set up all pointers ===
    // ===========================
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

    // ==============================          
    // ==== init globals phase 2 ====
    // ==============================
    {
      sycl::event queue_event =  gpu_queue.submit([&](sycl::handler &cgh) {
                                                    cgh.single_task([=]() {
                                                                      globals->numPrimitives              = numPrimitives;
                                                                      globals->node_mem_allocator_cur     = node_data_start/64;
                                                                      globals->node_mem_allocator_start   = node_data_start/64;
                                                                      globals->leaf_mem_allocator_cur     = leaf_data_start/64;
                                                                      globals->leaf_mem_allocator_start   = leaf_data_start/64;
                                                                      globals->bvh2_index_allocator       = numPrimitives; 
                                                                    });
                                                  });
      gpu::waitOnEventAndCatchException(queue_event);
      double dt = gpu::getDeviceExecutionTiming(queue_event);
      timer.add_to_device_timer(BuildTimer::PRE_PROCESS,dt);
      
      if (unlikely(verbose2))
        std::cout << "Init globals " << dt << " ms" << std::endl;
    }	    
    

    timer.start(BuildTimer::PRE_PROCESS);        
    
    double create_primref_time = 0.0f;
    // ===================================================          
    // ==== merge triangles to quads, create primrefs ====
    // ===================================================
         
    if (numQuads)
      createQuads_initPLOCPrimRefs(gpu_queue,args.geometries,numGeometries,prims_per_geom_prefix_sum,bvh2,0,create_primref_time,verbose2);      

    // ====================================          
    // ==== create procedural primrefs ====
    // ====================================

    if (numProcedurals)
      numProcedurals = createProcedurals_initPLOCPrimRefs(gpu_queue,args.geometries,numGeometries,sync_mem,bvh2,numQuads,host_device_tasks,create_primref_time,verbose2);

    // ==================================          
    // ==== create instance primrefs ====
    // ==================================
    
    if (numInstances)
      numInstances = createInstances_initPLOCPrimRefs(gpu_queue,args.geometries,numGeometries,sync_mem,bvh2,numQuads + numProcedurals,host_device_tasks,create_primref_time,verbose2);

    // === recompute actual number of primitives ===
    numPrimitives = numQuads + numInstances + numProcedurals;

    const GeometryTypeRanges geometryTypeRanges(numQuads,numProcedurals,numInstances);        
    
    if (unlikely(verbose2))
      PRINT4(numPrimitives,numQuads,numInstances,numProcedurals);
  
    // =================================================================================    
    // === test for empty scene again after all final primitive counts are available ===
    // =================================================================================

    if (unlikely(numPrimitives == 0)) return createEmptyBVH(args,gpu_queue);

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
    
    if (!fastMCMode) // fastMCMode == 32bit key + 32bit value pairs, !fastMode == 64bit key + 32bit value pairs
    {
      const uint scratchMemWGs = gpu::getNumWGsScratchSize(conv_mem_size);
      const uint nextPowerOf2 =  1 << (32 - sycl::clz(numPrimitives) - 1);
      const uint sortWGs = min(max(min((int)nextPowerOf2/8192,(int)gpu_maxComputeUnits/4),1),(int)scratchMemWGs);
     
      for (uint i=4;i<8;i++) 
        gpu::sort_iteration_type<MCPrim>(gpu_queue, morton_codes[i%2], morton_codes[(i+1)%2], numPrimitives, (uint*)scratch, i, sort_time, sortWGs);
      
      restoreMSBBits(gpu_queue,mc0,bvh2_subtree_size,numPrimitives,sort_time,verbose2);      

      for (uint i=4;i<8;i++) 
        gpu::sort_iteration_type<MCPrim>(gpu_queue, morton_codes[i%2], morton_codes[(i+1)%2], numPrimitives, (uint*)scratch, i, sort_time, sortWGs);
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
        for (uint i=3;i<8;i++) 
          gpu::sort_iteration_type<gpu::MortonCodePrimitive40x24Bits3D>(gpu_queue, (gpu::MortonCodePrimitive40x24Bits3D*)morton_codes[i%2], (gpu::MortonCodePrimitive40x24Bits3D*)morton_codes[(i+1)%2], numPrimitives, (uint*)scratch, i, sort_time, sortWGs);        
      }      
    }
    
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

    // ========================            
    // ==== clear sync mem ====
    // ========================      

    clearFirstScratchMemEntries(gpu_queue,sync_mem,0,NUM_ACTIVE_LARGE_WGS);
  
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

        const uint radius = SEARCH_RADIUS;          
      
        device_ploc_iteration_time = 0.0f;
        iteratePLOC(gpu_queue,globals,bvh2,cluster_index_source,cluster_index_dest,bvh2_subtree_size,sync_mem,numPrims,radius,NUM_ACTIVE_LARGE_WGS,host_device_tasks,device_ploc_iteration_time, false);
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
      gpu::waitOnEventAndCatchException(queue_event);
    }
    
    // =============================    
    // === convert BVH2 to QBVH6 ===
    // =============================
    timer.start(BuildTimer::PRE_PROCESS);    
    float conversion_device_time = 0.0f;
    const bool convert_success = convertBVH2toQBVH6(gpu_queue,globals,host_device_tasks,args.geometries,qbvh,bvh2,leafGenData,numPrimitives,numInstances != 0,geometryTypeRanges,conversion_device_time,verbose2);

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
      gpu::waitOnEventAndCatchException(queue_event);
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
      PRINT4(globals->node_mem_allocator_start,globals->node_mem_allocator_cur,globals->node_mem_allocator_cur-globals->node_mem_allocator_start,100.0f * (float)(globals->node_mem_allocator_cur-globals->node_mem_allocator_start) / (node_size/64));
      PRINT4(globals->leaf_mem_allocator_start,globals->leaf_mem_allocator_cur,globals->leaf_mem_allocator_cur-globals->leaf_mem_allocator_start,100.0f * (float)(globals->leaf_mem_allocator_cur-globals->leaf_mem_allocator_start) / (leaf_size/64));      
      PRINT(globals->numLeaves);
    }

    if (unlikely(convert_success == false))
    {
      if (args.accelBufferBytesOut) *args.accelBufferBytesOut = required_size*2; // should never happen
      return RTHWIF_ERROR_RETRY;
    }
    
#if defined(EMBREE_SYCL_ALLOC_DISPATCH_GLOBALS)
    HWAccel* hwaccel = (HWAccel*)args.accelBuffer;  
    hwaccel->dispatchGlobalsPtr = (uint64_t)args.dispatchGlobalsPtr;
#endif      

    if (args.accelBufferBytesOut)
      *args.accelBufferBytesOut = args.accelBufferBytes;

#if 1
    if (verbose2)
    {
      qbvh->print(std::cout,qbvh->root(),0,6);
      BVHStatistics stats = qbvh->computeStatistics();      
      stats.print(std::cout);
      stats.print_raw(std::cout);
      PRINT("VERBOSE STATS DONE");
    }        
#endif

    if (unlikely(verbose1))
      std::cout << "BVH build time: host = " << timer.get_total_host_time() << " ms, device = " << timer.get_total_device_time() << " ms " << std::endl;
    
    return RTHWIF_ERROR_NONE;    
  }
}

#endif    
