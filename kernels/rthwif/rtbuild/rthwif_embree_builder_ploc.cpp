#include "rtbuild.h"
#include "rthwif_embree_builder_ploc.h"
#include "qbvh6.h"
#include "../common/algorithms/parallel_reduce.h"

// === less than threshold, a single workgroup is used to perform all PLOC iterations in a single kernel launch ===
#define SINGLE_WG_SWITCH_THRESHOLD            4*1024

// === less than threshold, 40bits morton code + 24bits index are used, otherwise 64bit morton code + 32bit index ===
#define FAST_MC_NUM_PRIMS_THRESHOLD           1024*1024

// === max number of primitives fitting in 24bits ===
#define FAST_MC_MAX_NUM_PRIMS                 ((unsigned int)1<<24)

// === less than threshold, a single workgroup is used for all radix sort iterations ===
#define SMALL_SORT_THRESHOLD                  1024*4

// === maximum number of workgroups with 1024 elements, DG2/PVC perform best with 64 ===
#define MAX_LARGE_WGS                         256

// === rebalance if BVH2 subtrees are degenerated ===
#define BVH2_REBALANCE                        1

#define VALIDATE(arg)                           \
  {                                             \
    ze_result_t result = validate(arg);           \
    if (result != ZE_RESULT_SUCCESS) return result;     \
  }

#define VALIDATE_PTR(arg)                       \
  {                                                                     \
    if ((arg) == nullptr) return ZE_RESULT_ERROR_INVALID_NULL_POINTER; \
  }                                                                     \

#if defined(EMBREE_SYCL_GPU_BVH_BUILDER)      

namespace embree
{
  using namespace embree::isa;

  __forceinline size_t estimateSizeInternalNodes(const size_t numQuads, const size_t numInstances, const size_t numProcedurals, const size_t numLossyCompressedGeometries, const bool conservative)
  {
    const size_t N = numQuads + numInstances + numProcedurals + numLossyCompressedGeometries; 
    // === conservative estimate ===
    size_t numFatLeaves = 0;
    if (conservative)
      numFatLeaves = ceilf( (float)N/2 ) + ceilf( (float)numInstances/2 ); // FIXME : better upper bound for instance case
    else
      numFatLeaves = ceilf( (float)N/3 ) + ceilf( (float)numInstances/2 ); // FIXME : better upper bound for instance case        
    const size_t numInnerNodes = ceilf( (float)numFatLeaves/4 ); 
    return gpu::alignTo(std::max( ((numFatLeaves + numInnerNodes) * 64) , N * 16),64);
  }

  __forceinline size_t estimateSizeLeafNodes(const size_t numQuads, const size_t numInstances, const size_t numProcedurals, const size_t numLossyCompressedGeometries)
  {
    return (numQuads + numProcedurals + numLossyCompressedGeometries + 2*numInstances) * 64;  
  }

  __forceinline size_t estimateAccelBufferSize(const size_t numQuads, const size_t numInstances, const size_t numProcedurals, const size_t numLossyCompressedGeometries, const bool conservative)
  {
    const size_t header              = 128;
    const size_t node_size           = estimateSizeInternalNodes(numQuads,numInstances,numProcedurals,numLossyCompressedGeometries,conservative);
    const size_t leaf_size           = estimateSizeLeafNodes(numQuads,numInstances,numProcedurals,numLossyCompressedGeometries);
    const size_t lcg_size            = estimateLossyCompressedGeometriesSize(numLossyCompressedGeometries);    
    const size_t totalSize           = header + node_size + leaf_size + lcg_size;
    return totalSize;
  }

  __forceinline size_t estimateScratchBufferSize(const size_t numPrimitives)
  {
    // === sizeof(size_t)*MAX_LARGE_WGS for prefix sums across large work groups ===
    return sizeof(PLOCGlobals) + sizeof(size_t)*MAX_LARGE_WGS + numPrimitives * sizeof(LeafGenerationData);
  }

  unsigned int getBVH2Depth(BVH2Ploc *bvh2, unsigned int index, const unsigned int numPrimitives)
  {
    if (BVH2Ploc::getIndex(index) < numPrimitives) //isLeaf 
      return 1;
    else
      return 1 + std::max(getBVH2Depth(bvh2,bvh2[index].leftIndex(),numPrimitives),getBVH2Depth(bvh2,bvh2[index].rightIndex(),numPrimitives));
  }

  unsigned int getNumLeaves(BVH2Ploc *bvh2, unsigned int index, const unsigned int numPrimitives)
  {
    if (BVH2Ploc::getIndex(index) < numPrimitives) //isLeaf 
      return 1;
    else
      return getNumLeaves(bvh2,bvh2[index].leftIndex(),numPrimitives) + getNumLeaves(bvh2,bvh2[index].rightIndex(),numPrimitives);
  }


  unsigned int getNumFatLeaves(BVH2Ploc *bvh2, unsigned int index, const unsigned int numPrimitives)
  {
    if (BVH2Ploc::isFatLeaf(index,numPrimitives)) //isLeaf 
      return 1;
    else
      return getNumFatLeaves(bvh2,bvh2[index].leftIndex(),numPrimitives) + getNumFatLeaves(bvh2,bvh2[index].rightIndex(),numPrimitives);
  }
  


  void printBVH2Path(BVH2Ploc *bvh2, unsigned int index, const unsigned int numPrimitives)
  {
    if (BVH2Ploc::getIndex(index) < numPrimitives) //isLeaf
    {
      PRINT2(index,"LEAF");
    }
    else
    {
      const unsigned int depth = getBVH2Depth(bvh2,index,numPrimitives);
      const unsigned int leftIndex = bvh2[index].leftIndex();
      const unsigned int rightIndex = bvh2[index].rightIndex();
      const bool isFatLeafLeft = BVH2Ploc::isFatLeaf(bvh2[index].left,numPrimitives);
      const bool isFatLeafRight = BVH2Ploc::isFatLeaf(bvh2[index].right,numPrimitives);
      const unsigned int numLeavesLeft = getNumLeaves(bvh2,leftIndex,numPrimitives);
      const unsigned int numLeavesRight = getNumLeaves(bvh2,rightIndex,numPrimitives);      
      PRINT6(index,depth,leftIndex,rightIndex,isFatLeafLeft,isFatLeafRight);
      PRINT2(leftIndex,numLeavesLeft);
      PRINT2(rightIndex,numLeavesRight);
      
      if (!isFatLeafLeft)
        printBVH2Path(bvh2, leftIndex,numPrimitives);
      if (!isFatLeafRight)      
        printBVH2Path(bvh2,rightIndex,numPrimitives);      
    }
  }  
  
  
  void checkBVH2PlocHW(BVH2Ploc *bvh2, unsigned int index,unsigned int &nodes,unsigned int &leaves,float &nodeSAH, float &leafSAH, unsigned int &maxDepth,const unsigned int numPrimitives, const unsigned int bvh2_max_allocations, const unsigned int depth)
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
      maxDepth = max(maxDepth,depth+1);
      unsigned int indices[BVH_BRANCHING_FACTOR];
      const unsigned int numChildren = openBVH2MaxAreaSortChildren(BVH2Ploc::getIndex(index),indices,bvh2,numPrimitives);
      for (unsigned int i=0;i<numChildren;i++)
        if (BVH2Ploc::getIndex(indices[i]) > bvh2_max_allocations)
          FATAL("OPENING ERROR");

      nodes++;              
      nodeSAH += bvh2[index].bounds.area();
      
      if (!bvh2[index].bounds.encloses( bvh2[ bvh2[index].leftIndex() ].bounds )) PRINT2("ENCLOSING ERROR LEFT",index);
      checkBVH2PlocHW(bvh2,bvh2[index].leftIndex(),nodes,leaves,nodeSAH,leafSAH,maxDepth,numPrimitives,bvh2_max_allocations,depth+1);

      if (!bvh2[index].bounds.encloses( bvh2[ bvh2[index].rightIndex() ].bounds )) PRINT2("ENCLOSING ERROR RIGHT",index);
      checkBVH2PlocHW(bvh2,bvh2[index].rightIndex(),nodes,leaves,nodeSAH,leafSAH,maxDepth,numPrimitives,bvh2_max_allocations,depth+1);
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
      for (unsigned int i=0;i<TOTAL;i++)
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
      for (unsigned int i=0;i<ALLOCATION;i++) sum += device_timers[i];
      return sum;
    }

    inline float get_total_host_time()
    {
      double sum = 0.0;
      for (unsigned int i=0;i<ALLOCATION;i++) sum += host_timers[i];
      return sum;
    }
    
    
  };

  __forceinline uint32_t getNumPrimitives(const ze_rtas_builder_geometry_info_exp_t* geom)
  {
    switch (geom->geometryType) {
    case ZE_RTAS_BUILDER_GEOMETRY_TYPE_EXP_TRIANGLES  : return ((ze_rtas_builder_triangles_geometry_info_exp_t*) geom)->triangleCount;
    case ZE_RTAS_BUILDER_GEOMETRY_TYPE_EXP_QUADS      : return ((ze_rtas_builder_quads_geometry_info_exp_t*) geom)->quadCount;      
    case ZE_RTAS_BUILDER_GEOMETRY_TYPE_EXP_PROCEDURAL : return ((ze_rtas_builder_procedural_geometry_info_exp_t*) geom)->primCount;
    case ZE_RTAS_BUILDER_GEOMETRY_TYPE_EXP_INSTANCE   : return 1;
    default                              : return 0;
    };
  }

  /* fill all arg members that app did not know of yet */
  // __forceinline RTHWIF_BUILD_ACCEL_ARGS rthwifPrepareBuildAccelArgs(const RTHWIF_BUILD_ACCEL_ARGS& args_i)
  // {
  //   RTHWIF_BUILD_ACCEL_ARGS args;
  //   memset(&args,0,sizeof(RTHWIF_BUILD_ACCEL_ARGS));
  //   memcpy(&args,&args_i,std::min(sizeof(RTHWIF_BUILD_ACCEL_ARGS),args_i.structBytes));
  //   args.structBytes = sizeof(RTHWIF_BUILD_ACCEL_ARGS);
  //   return args;
  // } 
  
  __forceinline PrimitiveCounts countPrimitives(const ze_rtas_builder_geometry_info_exp_t** geometries, const unsigned int numGeometries)
  {
    auto reduce = [&](const range<size_t>& r) -> PrimitiveCounts
                  {
                    PrimitiveCounts counts;
                    for (size_t geomID = r.begin(); geomID < r.end(); geomID++)
                    {
                      const ze_rtas_builder_geometry_info_exp_t* geom = geometries[geomID];
                      if (geom == nullptr) continue;    
                      switch (geom->geometryType) {
                      case ZE_RTAS_BUILDER_GEOMETRY_TYPE_EXP_TRIANGLES  :
                      {
                        counts.numTriangles   += ((ze_rtas_builder_triangles_geometry_info_exp_t*)  geom)->triangleCount;
                        counts.numQuadBlocks  += (((ze_rtas_builder_triangles_geometry_info_exp_t *)geom)->triangleCount+TRIANGLE_QUAD_BLOCK_SIZE-1)/TRIANGLE_QUAD_BLOCK_SIZE;
                        break;
                      }
                      case ZE_RTAS_BUILDER_GEOMETRY_TYPE_EXP_QUADS      :
                      {
                        counts.numQuads       += ((ze_rtas_builder_quads_geometry_info_exp_t*)  geom)->quadCount;
                        counts.numQuadBlocks  += (((ze_rtas_builder_quads_geometry_info_exp_t *)geom)->quadCount+TRIANGLE_QUAD_BLOCK_SIZE-1)/TRIANGLE_QUAD_BLOCK_SIZE;                        
                        break;
                      }
                      case ZE_RTAS_BUILDER_GEOMETRY_TYPE_EXP_PROCEDURAL : counts.numProcedurals += ((ze_rtas_builder_procedural_geometry_info_exp_t*) geom)->primCount; break;
                      case ZE_RTAS_BUILDER_GEOMETRY_TYPE_EXP_INSTANCE   : counts.numInstances   += 1; break;
                      default: assert(false); break;        
                      };                    
                    };
                    return counts;
                  };

    const unsigned int COUNT_BLOCK_SIZE = 256;
    const unsigned int COUNT_PARALLEL_THRESHOLD = 256;
    
    const PrimitiveCounts primCounts = parallel_reduce((unsigned int)0, numGeometries, COUNT_BLOCK_SIZE, COUNT_PARALLEL_THRESHOLD, PrimitiveCounts(), reduce,
                                                       [&](const PrimitiveCounts& b0, const PrimitiveCounts& b1) -> PrimitiveCounts { return b0 + b1; });
    return primCounts;
  }

  ze_result_t createEmptyBVH(void *pRtasBuffer, ze_rtas_aabb_exp_t *pBounds, size_t *pRtasBufferSizeBytes, sycl::queue  &gpu_queue)
  {
    sycl::event queue_event =  gpu_queue.submit([&](sycl::handler &cgh) {
                                                  cgh.single_task([=]() {
                                                                    QBVH6* qbvh  = (QBVH6*)pRtasBuffer;       
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
    if (pRtasBufferSizeBytes) *pRtasBufferSizeBytes = 128+64;
    if (pBounds)           { BBox3f geometryBounds (empty); *pBounds = *(ze_rtas_aabb_exp_t*)&geometryBounds; };
    return ZE_RESULT_SUCCESS;    
  }
// =================================================================================================================================================================================
// =================================================================================================================================================================================
// =================================================================================================================================================================================

ZE_APIEXPORT ze_result_t ZE_APICALL zeRTASGetAccelSizeGPUExp( const ze_rtas_builder_build_op_exp_desc_t* args, ze_rtas_builder_exp_properties_t* size_o, void *sycl_queue, unsigned int verbose_level )    
  {
    double time0 = getSeconds();
    
    //RTHWIF_BUILD_ACCEL_ARGS args = rthwifPrepareBuildAccelArgs(args_i);
    const ze_rtas_builder_geometry_info_exp_t** geometries = args->ppGeometries; 
    const unsigned int numGeometries = args->numGeometries;
    sycl::queue  &gpu_queue  = *(sycl::queue*)sycl_queue;

    // =============================================================================    
    // === GPU-based primitive count estimation including triangle quadification ===
    // =============================================================================
    
    const PrimitiveCounts primCounts = getEstimatedPrimitiveCounts(gpu_queue,geometries,numGeometries,verbose_level >= 2);            

    const unsigned int numTriangles       = primCounts.numTriangles; // === original number of triangles ===
    const unsigned int numMergedTrisQuads = primCounts.numMergedTrisQuads;
    const unsigned int numQuads           = primCounts.numQuads;
    const unsigned int numProcedurals     = primCounts.numProcedurals;
    const unsigned int numInstances       = primCounts.numInstances;
    const unsigned int numLossyCompressedGeometries = primCounts.numLossyCompressedGeometries;

    const unsigned int numPrimitives = numMergedTrisQuads + numProcedurals + numInstances + numLossyCompressedGeometries;

    // =============================================    
    // === allocation for empty scene is default ===
    // =============================================
    
    size_t expectedBytes = 3*64; 
    size_t worstCaseBytes = 4*64;

    if (numPrimitives)
    {    
      expectedBytes  = estimateAccelBufferSize(     numMergedTrisQuads, numInstances, numProcedurals, numLossyCompressedGeometries, false);
      worstCaseBytes = estimateAccelBufferSize(numQuads + numTriangles, numInstances, numProcedurals, numLossyCompressedGeometries, true);    
    }

    // ===============================================    
    // === estimate accel and scratch buffer sizes ===
    // ===============================================
    
    const size_t scratchBytes = estimateScratchBufferSize(std::max(numPrimitives,numGeometries));

    if (verbose_level >= 2)
    {
      PRINT6(numMergedTrisQuads,numTriangles,numQuads,numProcedurals,numInstances,numLossyCompressedGeometries);      
      PRINT3(expectedBytes,worstCaseBytes,scratchBytes);
    }
        
    /* return size to user */
    size_o->rtasBufferSizeBytesExpected = expectedBytes;
    size_o->rtasBufferSizeBytesMaxRequired = worstCaseBytes;
    size_o->scratchBufferSizeBytes = scratchBytes;

    double time1 = getSeconds();
    if (verbose_level >= 1)
      std::cout << "zeRTASGetAccelSizeGPUExp time = " << (float)(time1-time0)*1000.0f << " ms" << std::endl;
    
    return ZE_RESULT_SUCCESS;
  }

ZE_APIEXPORT ze_result_t ZE_APICALL zeRTASPrefetchAccelGPUExp( const ze_rtas_builder_build_op_exp_desc_t* args, void *sycl_queue, unsigned int verbose_level )  
  {
    double time0 = getSeconds();
    
    sycl::queue  &gpu_queue  = *(sycl::queue*)sycl_queue;

    // ======================================================    
    // === DUMMY KERNEL TO TRIGGER REMAINING USM TRANSFER ===
    // ======================================================
    
    sycl::event queue_event =  gpu_queue.submit([&](sycl::handler &cgh) { cgh.single_task([=]() {}); });
    gpu::waitOnEventAndCatchException(queue_event);

    double time1 = getSeconds();
    if (verbose_level >= 1)
      std::cout << "zeRTASPrefetchAccelGPUExp time = " << (float)(time1-time0)*1000.0f << " ms" << std::endl;
    
    return ZE_RESULT_SUCCESS;      
  }

  ZE_APIEXPORT ze_result_t ZE_APICALL zeRTASBuildAccelGPUExp( const ze_rtas_builder_build_op_exp_desc_t* args,
                                                            void *pScratchBuffer, size_t scratchBufferSizeBytes,
                                                            void *pRtasBuffer, size_t rtasBufferSizeBytes,
                                                            void *pBuildUserPtr, ze_rtas_aabb_exp_t *pBounds, size_t *pRtasBufferSizeBytes,
                                                            void *sycl_queue, unsigned int verbose_level=0)    
  {
    BuildTimer timer;
    timer.reset();

    timer.start(BuildTimer::PRE_PROCESS);      
    
    // ================================    
    // === GPU device/queue/context ===
    // ================================
  
    sycl::queue  &gpu_queue  = *(sycl::queue*)sycl_queue;
    const bool verbose1 = verbose_level >= 1;    
    const bool verbose2 = verbose_level >= 2;
    const unsigned int gpu_maxComputeUnits  = gpu_queue.get_device().get_info<sycl::info::device::max_compute_units>();
    const unsigned int MAX_WGS = gpu_maxComputeUnits / 8;
    
    unsigned int *host_device_tasks = (unsigned int*)sycl::aligned_alloc(64,HOST_DEVICE_COMM_BUFFER_SIZE,gpu_queue.get_device(),gpu_queue.get_context(),sycl::usm::alloc::host);

    
    if (unlikely(verbose2))
    {
      const unsigned int gpu_maxWorkGroupSize = gpu_queue.get_device().get_info<sycl::info::device::max_work_group_size>();
      const unsigned int gpu_maxLocalMemory   = gpu_queue.get_device().get_info<sycl::info::device::local_mem_size>();    
      PRINT("PLOC++ GPU BVH BUILDER");            
      PRINT( gpu_queue.get_device().get_info<sycl::info::device::global_mem_size>() );
      PRINT(gpu_maxWorkGroupSize);
      PRINT(gpu_maxComputeUnits);
      PRINT(gpu_maxLocalMemory);
    }

    // =============================    
    // === setup scratch pointer ===
    // =============================
    
    PLOCGlobals *globals = (PLOCGlobals *)pScratchBuffer;
    unsigned int *const sync_mem = (unsigned int*)((char*)pScratchBuffer + sizeof(PLOCGlobals));
    unsigned int *const scratch  = (unsigned int*)((char*)pScratchBuffer + sizeof(PLOCGlobals) + sizeof(unsigned int)*MAX_LARGE_WGS);    
  
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
      if (unlikely(verbose1))
      {
        double dt = gpu::getDeviceExecutionTiming(queue_event);
        timer.add_to_device_timer(BuildTimer::PRE_PROCESS,dt);
        if (unlikely(verbose2)) std::cout << "=> Init Globals I: " << dt << " ms" << std::endl;        
      }      
    }  
  
    // ==============================================================================    
    // === get primitive type count from geometries, compute quad blocks per geom ===
    // ==============================================================================
  
    const ze_rtas_builder_geometry_info_exp_t** geometries = args->ppGeometries;
    unsigned int numGeometries                = args->numGeometries;
 
    double device_prim_counts_time = 0.0f;
  
    const PrimitiveCounts primCounts = countPrimitives(gpu_queue,geometries,numGeometries,globals,scratch,host_device_tasks,device_prim_counts_time,verbose1); 

    // ================================================
    
    timer.stop(BuildTimer::PRE_PROCESS);
    timer.add_to_device_timer(BuildTimer::PRE_PROCESS,device_prim_counts_time);                              
    if (unlikely(verbose2)) std::cout << "=> Count Primitives from Geometries: " << timer.get_host_timer() << " ms (host) " << device_prim_counts_time << " ms (device) " << std::endl;      
  
    unsigned int numQuads            = primCounts.numQuads + primCounts.numTriangles; // no quadification taken into account at this point
    unsigned int numProcedurals      = primCounts.numProcedurals;
    unsigned int numInstances        = primCounts.numInstances;
    const unsigned int numQuadBlocks = primCounts.numQuadBlocks;
    unsigned int numLossyCompressedGeometries = primCounts.numLossyCompressedGeometries;
    
    const unsigned int expected_numPrimitives = numQuads + numProcedurals + numInstances + numLossyCompressedGeometries;    

    // =================================================    
    // === empty scene before removing invalid prims ===
    // =================================================

    if (unlikely(expected_numPrimitives == 0)) createEmptyBVH(pRtasBuffer,pBounds,pRtasBufferSizeBytes,gpu_queue);
        
    if (numQuads)
    {
      // ==================================================
      // === compute correct quadification using blocks === 
      // ==================================================

      timer.start(BuildTimer::PRE_PROCESS);      
      double device_quadification_time = 0.0;
      numQuads = countQuadsPerGeometryUsingBlocks(gpu_queue,globals,args->ppGeometries,numGeometries,numQuadBlocks,scratch,scratch+numGeometries,host_device_tasks,device_quadification_time,verbose1);
      timer.stop(BuildTimer::PRE_PROCESS);
      timer.add_to_device_timer(BuildTimer::PRE_PROCESS,device_quadification_time);
      if (unlikely(verbose2)) std::cout << "=> Count " << numQuads << " Quads " << timer.get_host_timer() << " ms (host) " << (float)device_quadification_time << " ms (device) " << std::endl;
    }
    
    // ================================
    // === estimate size of the BVH ===
    // ================================

    size_t numPrimitives             = numQuads + numInstances + numProcedurals + numLossyCompressedGeometries;  // actual #prims can be lower due to invalid instances or procedurals but quads count is accurate at this point
    const size_t allocated_size      = rtasBufferSizeBytes; //args->accelBufferBytes;

    const size_t header              = 128;
    const size_t leaf_size           = estimateSizeLeafNodes(numQuads,numInstances,numProcedurals,numLossyCompressedGeometries);
    const size_t lcg_size            = estimateLossyCompressedGeometriesSize(numLossyCompressedGeometries);
    const size_t node_size           = (header + leaf_size + lcg_size) <= allocated_size ? allocated_size - lcg_size - leaf_size - header : 0; 
    const size_t node_data_start     = header;
    const size_t leaf_data_start     = header + node_size;
    const size_t lcg_data_start      = numLossyCompressedGeometries ? (leaf_data_start + leaf_size) : 0;
    
    if (unlikely(verbose2))
      PRINT3( node_data_start, leaf_data_start, lcg_data_start );
      
    // =================================================================
    // === if allocated accel buffer is too small, return with error ===
    // =================================================================
    const uint32_t required_size = header + estimateSizeInternalNodes(numQuads,numInstances,numProcedurals,numLossyCompressedGeometries,false) + leaf_size;
    //const unsigned int required_size = header + estimateSizeInternalNodes(numQuads,numInstances,numProcedurals,false) + leaf_size;
    if (unlikely(allocated_size < required_size))
    {
      if (unlikely(verbose2))
      {
        PRINT2(required_size,allocated_size);
        PRINT2(node_size,estimateSizeInternalNodes(numQuads,numInstances,numProcedurals,numLossyCompressedGeometries,false));        
        PRINT3("RETRY BVH BUILD DUE BECAUSE OF SMALL ACCEL BUFFER ALLOCATION!!!", required_size,allocated_size);
      }
      if (pRtasBufferSizeBytes) *pRtasBufferSizeBytes = required_size;
      if (host_device_tasks) sycl::free(host_device_tasks,gpu_queue.get_context());
      return ZE_RESULT_ERROR_OUT_OF_HOST_MEMORY;
    }

    const size_t conv_mem_size = sizeof(numPrimitives)*numPrimitives;
    const size_t NUM_ACTIVE_LARGE_WGS = min((numPrimitives+LARGE_WG_SIZE-1)/LARGE_WG_SIZE,(size_t)MAX_WGS);

    // ===========================
    // === set up all pointers ===
    // ===========================
    QBVH6* qbvh   = (QBVH6*)pRtasBuffer;
    char *bvh_mem = (char*)qbvh + header;
    char *const leaf_mem = (char*)qbvh + leaf_data_start;
    BVH2Ploc *const bvh2 = (BVH2Ploc*)(leaf_mem);
    typedef gpu::MortonCodePrimitive64Bit_2x MCPrim;
    MCPrim *const mc0 = (MCPrim*)(bvh2 + numPrimitives);
    MCPrim *const mc1 = mc0 + numPrimitives;     
    MCPrim *const morton_codes[2] = { mc0, mc1 }; 
    unsigned int *const cluster_index     = (unsigned int*) (bvh_mem + 0 * numPrimitives * sizeof(unsigned int)); // * 2
    BVH2SubTreeState *const bvh2_subtree_size = (BVH2SubTreeState*) (bvh_mem + 2 * numPrimitives * sizeof(unsigned int)); // * 2        
    unsigned int *cluster_i[2] = { cluster_index + 0, cluster_index + numPrimitives };        
    unsigned int *const cluster_index_source = cluster_i[0];
    unsigned int *const   cluster_index_dest = cluster_i[1];
    LeafGenerationData *leafGenData = (LeafGenerationData*)scratch;
    char *lcg_bvh_mem = (char*)qbvh + lcg_data_start;

    // ==============================          
    // ==== init globals phase 2 ====
    // ==============================
    {
      sycl::event queue_event =  gpu_queue.submit([&](sycl::handler &cgh) {
                                                    cgh.single_task([=]() {
                                                                      globals->node_mem_allocator_cur     = node_data_start/64;
                                                                      globals->node_mem_allocator_start   = node_data_start/64;
                                                                      globals->leaf_mem_allocator_cur     = leaf_data_start/64;
                                                                      globals->leaf_mem_allocator_start   = leaf_data_start/64;
                                                                      globals->bvh2_index_allocator       = numPrimitives; 
                                                                    });
                                                  });
      gpu::waitOnEventAndCatchException(queue_event);
      if (unlikely(verbose1))
      {
        double dt = gpu::getDeviceExecutionTiming(queue_event);
        timer.add_to_device_timer(BuildTimer::PRE_PROCESS,dt);
        if (unlikely(verbose2)) std::cout << "=> Init Globals II: " << dt << " ms" << std::endl;        
      }      
    }	    
    timer.start(BuildTimer::PRE_PROCESS);        
    
    double create_primref_time = 0.0f;
    // ===================================================          
    // ==== merge triangles to quads, create primrefs ====
    // ===================================================

    if (numQuads)
      createQuads_initPLOCPrimRefs(gpu_queue,globals,args->ppGeometries,numGeometries,numQuadBlocks,scratch,bvh2,0,create_primref_time,verbose1);
    
    // ====================================          
    // ==== create procedural primrefs ====
    // ====================================
    
    if (numProcedurals)
      numProcedurals = createProcedurals_initPLOCPrimRefs(gpu_queue,args->ppGeometries,numGeometries,sync_mem,NUM_ACTIVE_LARGE_WGS,bvh2,numQuads,pBuildUserPtr,host_device_tasks,create_primref_time,verbose1);

    // ==================================          
    // ==== create instance primrefs ====
    // ==================================
    
    if (numInstances)
      numInstances = createInstances_initPLOCPrimRefs(gpu_queue,args->ppGeometries,numGeometries,sync_mem,NUM_ACTIVE_LARGE_WGS,bvh2,numQuads + numProcedurals,host_device_tasks,create_primref_time,verbose1);


    // ===================================================          
    // ==== create lossy compressed geometry primrefs ====
    // ===================================================

    if (numLossyCompressedGeometries)
      numLossyCompressedGeometries = createLossyCompressedGeometries_initPLOCPrimRefs(gpu_queue,args->ppGeometries,numGeometries,sync_mem,NUM_ACTIVE_LARGE_WGS,bvh2,numQuads + numProcedurals + numInstances,host_device_tasks,lcg_bvh_mem,&globals->numLeaves,create_primref_time,verbose1);
    
    
    // =================================================================================================    
    // === recompute actual number of primitives after quadification and removing of invalid entries ===
    // =================================================================================================
    
    numPrimitives = numQuads + numInstances + numProcedurals + numLossyCompressedGeometries;

    const GeometryTypeRanges geometryTypeRanges(numQuads,numProcedurals,numInstances,numLossyCompressedGeometries);        
    
    if (unlikely(verbose2))
    {
      PRINT5(numPrimitives,numQuads,numInstances,numProcedurals,numLossyCompressedGeometries);
      PRINT2(node_size,leaf_size);
      PRINT2(node_size/64,leaf_size/64);      
    }      
    
    // =================================================================================    
    // === test for empty scene again after all final primitive counts are available ===
    // =================================================================================

    if (unlikely(numPrimitives == 0))
    {
      if (host_device_tasks) sycl::free(host_device_tasks,gpu_queue.get_context());      
      return createEmptyBVH(pRtasBuffer,pBounds,pRtasBufferSizeBytes,gpu_queue);
    }

    timer.stop(BuildTimer::PRE_PROCESS);
    timer.add_to_device_timer(BuildTimer::PRE_PROCESS,create_primref_time);    
    if (unlikely(verbose2)) std::cout << "=> Create Quads/Procedurals/Instances etc, Init PrimRefs: " << timer.get_host_timer() << " ms (host) " << create_primref_time << " ms (device) " << std::endl;
      
    // ==========================================          
    // ==== get centroid and geometry bounds ====
    // ==========================================
        
    timer.start(BuildTimer::PRE_PROCESS);        
    double device_compute_centroid_bounds_time = 0.0f;
     
    computeCentroidGeometryBounds(gpu_queue, &globals->geometryBounds, &globals->centroidBounds, bvh2, numPrimitives, device_compute_centroid_bounds_time, verbose1);
    
    timer.stop(BuildTimer::PRE_PROCESS);
    timer.add_to_device_timer(BuildTimer::PRE_PROCESS,device_compute_centroid_bounds_time);


    if (unlikely(verbose2))
      std::cout << "=> Get Geometry and Centroid Bounds Phase: " << timer.get_host_timer() << " ms (host) " << device_compute_centroid_bounds_time << " ms (device) " << std::endl;		
  
    // ==============================          
    // ==== compute morton codes ====
    // ==============================

    const bool fastMCMode = numPrimitives < FAST_MC_NUM_PRIMS_THRESHOLD || (args->buildQuality == ZE_RTAS_BUILDER_BUILD_QUALITY_HINT_EXP_LOW && numPrimitives < FAST_MC_MAX_NUM_PRIMS);    
    
    timer.start(BuildTimer::PRE_PROCESS);        
    double device_compute_mc_time = 0.0f;

    if (!fastMCMode)
      computeMortonCodes64Bit_SaveMSBBits(gpu_queue,&globals->centroidBounds,mc0,bvh2,(unsigned int*)bvh2_subtree_size,numPrimitives,device_compute_mc_time,verbose1);
    else
      computeMortonCodes64Bit(gpu_queue,&globals->centroidBounds,(gpu::MortonCodePrimitive40x24Bits3D*)mc1,bvh2,numPrimitives,0,(uint64_t)-1,device_compute_mc_time,verbose1);
            
    timer.stop(BuildTimer::PRE_PROCESS);
     
    if (unlikely(verbose2))
      std::cout << "=> Compute Morton Codes: " << timer.get_host_timer() << " ms (host) " << device_compute_mc_time << " ms (device) " << std::endl;		
    
    // ===========================          
    // ==== sort morton codes ====
    // ===========================

    timer.start(BuildTimer::PRE_PROCESS);        
    
    if (!fastMCMode) // fastMCMode == 32bit key + 32bit value pairs, !fastMode == 64bit key + 32bit value pairs
    {
      const unsigned int scratchMemWGs = gpu::getNumWGsScratchSize(conv_mem_size);
      const unsigned int nextPowerOf2 =  1 << (32 - sycl::clz(numPrimitives) - 1);
      const unsigned int sortWGs = min(max(min((int)nextPowerOf2/8192,(int)gpu_maxComputeUnits/4),1),(int)scratchMemWGs);

      sycl::event initial = sycl::event();
      sycl::event block0  = gpu::radix_sort_Nx8Bit(gpu_queue, morton_codes[0], morton_codes[1], numPrimitives, (unsigned int*)scratch, 4, 8, initial, sortWGs);      
      sycl::event restore = restoreMSBBits(gpu_queue,mc0,(unsigned int*)bvh2_subtree_size,numPrimitives,block0,verbose1);      
      sycl::event block1  = gpu::radix_sort_Nx8Bit(gpu_queue, morton_codes[0], morton_codes[1], numPrimitives, (unsigned int*)scratch, 4, 8, restore, sortWGs);
      gpu::waitOnEventAndCatchException(block1);      
    }
    else
    {
      if (numPrimitives < SMALL_SORT_THRESHOLD)
        gpu::radix_sort_single_workgroup(gpu_queue, (uint64_t *)mc0, (uint64_t *)mc1, numPrimitives, 3,8);
      else
      {
        const unsigned int scratchMemWGs = gpu::getNumWGsScratchSize(conv_mem_size);        
        const unsigned int nextPowerOf2 =  1 << (32 - sycl::clz(numPrimitives) - 1);          
        const unsigned int sortWGs = min(max(min((int)nextPowerOf2/LARGE_WG_SIZE,(int)gpu_maxComputeUnits/4),1),(int)scratchMemWGs);
        sycl::event initial = sycl::event();
        sycl::event block0  = gpu::radix_sort_Nx8Bit(gpu_queue, (gpu::MortonCodePrimitive40x24Bits3D*)morton_codes[1], (gpu::MortonCodePrimitive40x24Bits3D*)morton_codes[0], numPrimitives, (unsigned int*)scratch, 3, 8, initial, sortWGs);
        gpu::waitOnEventAndCatchException(block0);              
      }      
    }
    
    timer.stop(BuildTimer::PRE_PROCESS);        
    timer.add_to_device_timer(BuildTimer::PRE_PROCESS,timer.get_host_timer());
            
    if (unlikely(verbose2))
      std::cout << "=> Sort Morton Codes: " << timer.get_host_timer() << " ms (host and device)" << std::endl;
                      
    // ===========================          
    // ====== init clusters ======
    // ===========================

    
    timer.start(BuildTimer::PRE_PROCESS);        
    double device_init_clusters_time = 0.0f;

    if (!fastMCMode)
      initClusters(gpu_queue,mc0,bvh2,cluster_index,bvh2_subtree_size,numPrimitives,device_init_clusters_time,verbose1);
    else
      initClusters(gpu_queue,(gpu::MortonCodePrimitive40x24Bits3D*)mc0,bvh2,cluster_index,bvh2_subtree_size,numPrimitives,device_init_clusters_time,verbose1); 
    
    timer.stop(BuildTimer::PRE_PROCESS);        
    timer.add_to_device_timer(BuildTimer::PRE_PROCESS,device_init_clusters_time);
        
    if (unlikely(verbose2))
      std::cout << "=> Init Clusters: " << timer.get_host_timer() << " ms (host) " << device_init_clusters_time << " ms (device) " << std::endl;		

    unsigned int numPrims = numPrimitives;
    // ===================================================================================================================================================
    // ===================================================================================================================================================
    // ===================================================================================================================================================

    // === 8 or 16-wide search radius dependening on compiler flags ===
    const unsigned int SEARCH_RADIUS_SHIFT = args->buildQuality == ZE_RTAS_BUILDER_BUILD_QUALITY_HINT_EXP_LOW ? 3 : 4;
    
    double device_ploc_iteration_time = 0.0f;
        
    unsigned int iteration = 0;
  
    timer.start(BuildTimer::BUILD);        
    
    // ========================            
    // ==== clear sync mem ====
    // ========================      

    clearScratchMem(gpu_queue,sync_mem,0,NUM_ACTIVE_LARGE_WGS,device_ploc_iteration_time,verbose1);

    float ratio = 100.0f;
    for (;numPrims>1;iteration++)
    {
      // ==================================================            
      // ==== single kernel path if #prims < threshold ====
      // ==================================================
      
      if (numPrims < SINGLE_WG_SWITCH_THRESHOLD)
      {
        double singleWG_time = 0.0f;
        singleWGBuild(gpu_queue, globals, bvh2, cluster_index_source, cluster_index_dest, bvh2_subtree_size, numPrims, SEARCH_RADIUS_SHIFT, singleWG_time, verbose1);
        timer.add_to_device_timer(BuildTimer::BUILD,singleWG_time);
        numPrims = 1;
      }
      else  
      {            
        // ===================================================================================
        // ==== nearest neighbor search, merge clusters and create bvh2 nodes (fast path) ====
        // ===================================================================================
        device_ploc_iteration_time = 0.0f;
        iteratePLOC(gpu_queue,globals,bvh2,cluster_index_source,cluster_index_dest,bvh2_subtree_size,sync_mem,numPrims,NUM_ACTIVE_LARGE_WGS,host_device_tasks,SEARCH_RADIUS_SHIFT,device_ploc_iteration_time,ratio < BOTTOM_LEVEL_RATIO,verbose1);
        timer.add_to_device_timer(BuildTimer::BUILD,device_ploc_iteration_time);
        const unsigned int new_numPrims = *host_device_tasks;
        assert(new_numPrims < numPrims);
        ratio = (float)(numPrims-new_numPrims) / numPrims * 100.0f;
        numPrims = new_numPrims;                      
        // ==========================            
      }        
      if (unlikely(verbose2))
        PRINT5(iteration,numPrims,ratio,(float)device_ploc_iteration_time,(float)timer.get_accum_device_timer(BuildTimer::BUILD));
    }
    timer.stop(BuildTimer::BUILD);        

    if (unlikely(verbose2))
      std::cout << "=> PLOC phase: " <<  timer.get_host_timer() << " ms (host) " << (float)timer.get_accum_device_timer(BuildTimer::BUILD) << " ms (device) " << std::endl;    

#if BVH2_REBALANCE == 1
        // ===============================================================================================================
        // ========================================== rebalance BVH2 if degenerated ======================================
        // ===============================================================================================================
        
        double rebalanceBVH2_time = 0.0f;
        rebalanceBVH2(gpu_queue,bvh2,bvh2_subtree_size,numPrimitives,rebalanceBVH2_time,verbose1);
        if (unlikely(verbose2))
          PRINT(rebalanceBVH2_time);
        timer.add_to_device_timer(BuildTimer::BUILD,rebalanceBVH2_time);

        // ===============================================================================================================
        // ===============================================================================================================
        // ===============================================================================================================
#endif        
                
    // =====================================                
    // === check and convert BVH2 (host) ===
    // =====================================
    
    if (unlikely(verbose2))
    {
      PRINT2(globals->bvh2_index_allocator,2*numPrimitives);              
      if (globals->bvh2_index_allocator >= 2*numPrimitives)
        FATAL("BVH2 construction, allocator");
      PRINT(globals->rootIndex);      
      unsigned int nodes = 0;
      unsigned int leaves = 0;
      float nodeSAH = 0;
      float leafSAH = 0;
      unsigned int maxDepth = 0;
      checkBVH2PlocHW(bvh2,globals->rootIndex,nodes,leaves,nodeSAH,leafSAH,maxDepth,numPrimitives,globals->bvh2_index_allocator,0);
      nodeSAH /= globals->geometryBounds.area();
      leafSAH /= globals->geometryBounds.area();                
      PRINT5(nodes,leaves,nodeSAH,leafSAH,maxDepth);
 
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
    const bool convert_success = convertBVH2toQBVH6(gpu_queue,globals,host_device_tasks,args->ppGeometries,qbvh,bvh2,leafGenData,numPrimitives,numInstances != 0,geometryTypeRanges,lcg_bvh_mem,conversion_device_time,verbose1);
    //const bool convert_success = convertBVH2toQBVH6(gpu_queue,globals,host_device_tasks,args->ppGeometries,qbvh,bvh2,leafGenData,numPrimitives,numInstances != 0,geometryTypeRanges,conversion_device_time,verbose1);
    
    /* dispatch globals ptr for debugging purposes */
    void* dispatchGlobalsPtr = nullptr;
#if defined(EMBREE_SYCL_ALLOC_DISPATCH_GLOBALS)
    dispatchGlobalsPtr = args->dispatchGlobalsPtr;
#endif
    
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
                                                                      qbvh->dispatchGlobalsPtr = (uint64_t) dispatchGlobalsPtr;

                                                                    });
                                                  });
      gpu::waitOnEventAndCatchException(queue_event);
    }	    
    
    if (pBounds) *pBounds = *(ze_rtas_aabb_exp_t*)host_device_tasks;
    
    timer.stop(BuildTimer::POST_PROCESS);
    timer.add_to_device_timer(BuildTimer::POST_PROCESS,conversion_device_time);

    if (unlikely(verbose2))
      std::cout << "=> BVH2 -> QBVH6 Flattening: " <<  timer.get_host_timer() << " ms (host) " << conversion_device_time << " ms (device) " << std::endl;

    // ==========================================================    
    // ==========================================================
    // ==========================================================

    if (unlikely(verbose2))
    {
      // === memory allocation and usage stats ===
      const unsigned int nodes_used   = globals->node_mem_allocator_cur-globals->node_mem_allocator_start;
      const unsigned int leaves_used  = globals->leaf_mem_allocator_cur-globals->leaf_mem_allocator_start;
      const float nodes_util  = 100.0f * (float)(globals->node_mem_allocator_cur-globals->node_mem_allocator_start) / (node_size/64);
      const float leaves_util = 100.0f * (float)(globals->leaf_mem_allocator_cur-globals->leaf_mem_allocator_start) / (leaf_size/64);
      PRINT4(globals->node_mem_allocator_start,globals->node_mem_allocator_cur,nodes_used,nodes_util);
      PRINT4(globals->leaf_mem_allocator_start,globals->leaf_mem_allocator_cur,leaves_used,leaves_util);      
      PRINT(globals->numLeaves);
    }
    
    if (unlikely(convert_success == false))
    {
      if (pRtasBufferSizeBytes) *pRtasBufferSizeBytes = estimateAccelBufferSize(numQuads, numInstances, numProcedurals, numLossyCompressedGeometries, true); 
      if (host_device_tasks) sycl::free(host_device_tasks,gpu_queue.get_context());      
      return ZE_RESULT_ERROR_OUT_OF_HOST_MEMORY;
    }
    
#if defined(EMBREE_SYCL_ALLOC_DISPATCH_GLOBALS)
    {
      sycl::event queue_event =  gpu_queue.submit([&](sycl::handler &cgh) {
                                                    cgh.single_task([=]() {
                                                                      HWAccel* hwaccel = (HWAccel*)args->accelBuffer;  
                                                                      hwaccel->dispatchGlobalsPtr = (uint64_t)args->dispatchGlobalsPtr;                                                                      
                                                                    });
                                                  });
      gpu::waitOnEventAndCatchException(queue_event);
    }
#endif      

    if (pRtasBufferSizeBytes)
      *pRtasBufferSizeBytes = rtasBufferSizeBytes;
#if 1
    if (verbose2)
    {
      gpu::waitOnQueueAndCatchException(gpu_queue);      
      BVHStatistics stats = qbvh->computeStatistics();      
      stats.print(std::cout);
      stats.print_raw(std::cout);
      qbvh->print();      
      PRINT("VERBOSE STATS DONE");
    }        
#endif
    
    if (host_device_tasks) sycl::free(host_device_tasks,gpu_queue.get_context());      
    
    if (unlikely(verbose1))
      std::cout << "=> BVH build time: host = " << timer.get_total_host_time() << " ms , device = " << timer.get_total_device_time() << " ms , numPrimitives (original) = " << expected_numPrimitives << " , numPrimitives (build) = " << numPrimitives << std::endl;
    return ZE_RESULT_SUCCESS;    
  }
}

#endif    
