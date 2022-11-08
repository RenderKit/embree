#include "rthwif_embree.h"
#include "rthwif_embree_builder.h"
#include "rthwif_internal.h"
#include "rthwif_embree_builder_ploc.h"
#include "builder/qbvh6.h"
#include "../common/algorithms/parallel_reduce.h"

// FIXME: compute MAX_WGS at run-time
// FIXME: leaf data generation without flags

#define SINGLE_WG_SWITCH_THRESHOLD 8*1024
#define FAST_MC_THRESHOLD          1024*1024
#define MAX_WGS                    64
#define SMALL_SORT_THRESHOLD       1024*4

static const float ESTIMATED_QUADIFICATION_FACTOR = 0.58;
static const float ESTIMATED_INTERNAL_NODE_FACTOR = 0.6;

namespace embree
{
  using namespace embree::isa;

  __forceinline uint estimateSizeInternalNodes(const uint numQuads, const uint numInstances, const uint numProcedurals)
  {
    const uint N = numQuads + numInstances + numProcedurals;
    // === conservative estimate ===
    const uint numFatLeaves = ceilf( (float)N/2 );
    const uint numInnerNodes = ceilf( (float)numFatLeaves/4 ); // 4 instead of 5 due to mixed nodes: 2 leaf refs + 4 inner node refs per inner node
    return std::max( (numFatLeaves + numInnerNodes) * 64, N * 16);
  }

  __forceinline uint estimateSizeLeafNodes(const uint numQuads, const uint numInstances, const uint numProcedurals)
  {
    return (numQuads + numProcedurals + 2 * numInstances) * 64;  
  }

  __forceinline uint estimateAccelBufferSize(const uint numQuads, const uint numInstances, const uint numProcedurals, const float internalNodeFactor = 1.0f)
  {
    const uint header              = 128;
    const uint node_size           = gpu::alignTo(ceilf(internalNodeFactor*estimateSizeInternalNodes(numQuads,numInstances,numProcedurals)),64);
    const uint leaf_size           = estimateSizeLeafNodes(numQuads,numInstances,numProcedurals); 
    const uint totalSize           = header + node_size + leaf_size; 
    return totalSize;
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

  struct PrimitiveCounts {
    uint numTriangles;
    uint numQuads;
    uint numProcedurals;
    uint numInstances;

    __forceinline PrimitiveCounts() : numTriangles(0), numQuads(0), numProcedurals(0), numInstances(0)
    {
    }
  };

  __forceinline PrimitiveCounts operator +(const PrimitiveCounts& a, const PrimitiveCounts& b) {
    PrimitiveCounts c;
    c.numTriangles   = a.numTriangles   + b.numTriangles;
    c.numQuads       = a.numQuads       + b.numQuads;
    c.numProcedurals = a.numProcedurals + b.numProcedurals;
    c.numInstances   = a.numInstances   + b.numInstances;
    return c;
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

// ===============================================================================================================
// ===============================================================================================================

#if defined(EMBREE_SYCL_GPU_BVH_BUILDER)      
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

  if (args_i.verbose)
    PRINT4(numTriangles,numQuads,numProcedurals,numInstances);
  // ============================================================================================================================================================================
  // ============================================================================================================================================================================
  // ============================================================================================================================================================================
        
  size_t expectedBytes = 3*64; // empty scene is default
  size_t worstCaseBytes = 2*expectedBytes;

  if (numPrimitives)
  {    
    if (args_i.verbose)
      PRINT2(numTriangles,ceilf(numTriangles * ESTIMATED_QUADIFICATION_FACTOR));
    
    expectedBytes  = estimateAccelBufferSize(numQuads + ceilf(numTriangles * ESTIMATED_QUADIFICATION_FACTOR), numInstances, numProcedurals, ESTIMATED_INTERNAL_NODE_FACTOR);
    worstCaseBytes = estimateAccelBufferSize(numQuads + numTriangles, numInstances, numProcedurals);    
  }
  size_t scratchBytes = std::max(numPrimitives * sizeof(LeafGenerationData) + sizeof(PLOCGlobals),sizeof(uint)*MAX_WGS);

  if (args_i.verbose)  
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
  size_t bytes_o = size_o.structBytes;
  memset(&size_o,0,bytes_o);
  memcpy(&size_o,&size,bytes_o);
  size_o.structBytes = bytes_o;
  return RTHWIF_ERROR_NONE;
}
#endif    
  

#if defined(EMBREE_SYCL_GPU_BVH_BUILDER)
RTHWIF_API RTHWIF_ERROR rthwifBuildAccelGPU(const RTHWIF_BUILD_ACCEL_ARGS& args)
{
  BuildTimer timer;
  timer.reset();

  // ================================    
  // === GPU device/queue/context ===
  // ================================
  
  sycl::queue  &gpu_queue  = *(sycl::queue*)args.sycl_queue;
  sycl::device &gpu_device = *(sycl::device*)args.sycl_device;  
  const bool verbose2 = args.verbose;
  const uint gpu_maxWorkGroupSize = gpu_device.get_info<sycl::info::device::max_work_group_size>();
  const uint gpu_maxComputeUnits  = gpu_device.get_info<sycl::info::device::max_compute_units>();    
  const uint gpu_maxLocalMemory   = gpu_device.get_info<sycl::info::device::local_mem_size>();
  uint *host_device_tasks = (uint*)args.hostDeviceCommPtr;
  
  if (unlikely(verbose2))
  {
    PRINT("PLOC++ GPU BVH BUILDER");            
    PRINT( gpu_device.get_info<sycl::info::device::global_mem_size>() );
    PRINT(gpu_maxWorkGroupSize);
    PRINT(gpu_maxComputeUnits);
    PRINT(gpu_maxLocalMemory);
  }

  // =================================    
  // === get primitive type counts ===
  // =================================
  const RTHWIF_GEOMETRY_DESC** geometries = args.geometries;
  const uint numGeometries                = args.numGeometries;  
  const PrimitiveCounts primCounts        = countPrimitives(geometries,numGeometries); // FIXME by GPU version
  
  uint numQuads       = primCounts.numQuads + primCounts.numTriangles;
  uint numProcedurals = primCounts.numProcedurals;
  uint numInstances   = primCounts.numInstances;

  const uint expected_numPrimitives = numQuads + numProcedurals + numInstances;    

  // ===================    
  // === empty scene ===
  // ===================
    
  if (unlikely(expected_numPrimitives == 0))
  {
    QBVH6* qbvh  = (QBVH6*)args.accelBuffer;       
    BBox3f geometryBounds (empty);
    qbvh->bounds = geometryBounds;
    qbvh->numPrims       = 0;                                                                        
    qbvh->nodeDataStart  = 2;
    qbvh->nodeDataCur    = 3;
    qbvh->leafDataStart  = 3;
    qbvh->leafDataCur    = 3;        
    new (qbvh->nodePtr(2)) QBVH6::InternalNode6(NODE_TYPE_INTERNAL);
    if (args.accelBufferBytesOut)
      *args.accelBufferBytesOut = 3*64;
    if (args.boundsOut) *args.boundsOut = *(RTHWIF_AABB*)&geometryBounds;
    return RTHWIF_ERROR_NONE;
  }    
    
  // ============================================    
  // === DUMMY KERNEL TO TRIGGER USM TRANSFER ===
  // ============================================
    
  {
    double first_kernel_time0 = getSeconds();          
    sycl::event queue_event =  gpu_queue.submit([&](sycl::handler &cgh) { cgh.single_task([=]() {}); });
    gpu::waitOnQueueAndCatchException(gpu_queue);
    double first_kernel_time1 = getSeconds();
    if (unlikely(verbose2)) std::cout << "Dummy first kernel launch (should trigger all USM transfers) " << (first_kernel_time1-first_kernel_time0)*1000.0f << " ms " << std::endl;      
  }

  // =============================    
  // === setup scratch pointer ===
  // =============================
    
  PLOCGlobals *globals = (PLOCGlobals *)args.scratchBuffer;
  uint *const scratch = (uint*)((char*)args.scratchBuffer + sizeof(PLOCGlobals));
  uint *prims_per_geom_prefix_sum = (uint*)scratch;
        
  if (numQuads)
  {
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
    numQuads = *host_device_tasks; // update with actual quads count
  }


  // ================================
  // === estimate size of the BVH ===
  // ================================
  
  uint numPrimitives             = numQuads + numInstances + numProcedurals;  // can be lower due to invalid instances or procedurals  
  const uint header              = 128;
  const uint leaf_size           = estimateSizeLeafNodes(numQuads,numInstances,numProcedurals);
  const uint node_size           = args.accelBufferBytes - leaf_size - header; 
  const uint totalSize           = header + node_size + leaf_size; 
  const uint node_data_start     = header;
  const uint leaf_data_start     = header + node_size;
    
  if (unlikely(verbose2))
  {
    PRINT4(numQuads,numProcedurals,numInstances,expected_numPrimitives);
    PRINT4(node_size,leaf_size,totalSize,args.accelBufferBytes);
  }
  
  // =================================================================
  // === if allocated accel buffer is too small, return with error ===
  // =================================================================
    
  if (leaf_size+header > args.accelBufferBytes || node_size < numPrimitives*16)
  {
    const uint estimate = header + leaf_size + estimateSizeInternalNodes(numQuads,numInstances,numProcedurals);    
    if (unlikely(args.verbose))
      PRINT3("RETRY!!!", args.accelBufferBytes,estimate);
    if (args.accelBufferBytesOut) *args.accelBufferBytesOut = estimate;
    return RTHWIF_ERROR_RETRY;
  }

  
  const bool fastMCMode = numPrimitives < FAST_MC_THRESHOLD;
  const size_t conv_mem_size = sizeof(numPrimitives)*numPrimitives;

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
  
  // ======================          
  // ==== init globals ====
  // ======================
  {
    timer.start(BuildTimer::PRE_PROCESS);      
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
    
  double create_primref_time = 0.0f;

  timer.start(BuildTimer::PRE_PROCESS);        
    
  // ===================================================          
  // ==== merge triangles to quads, create primrefs ====
  // ===================================================
         
  if (numQuads)
    createQuads_initPLOCPrimRefs(gpu_queue,args.geometries,numGeometries,prims_per_geom_prefix_sum,bvh2,0,create_primref_time,verbose2);      

  // ====================================          
  // ==== create procedural primrefs ====
  // ====================================

  if (numProcedurals)
    numProcedurals = createProcedurals_initPLOCPrimRefs(gpu_queue,args.geometries,numGeometries,bvh2,numQuads,host_device_tasks,create_primref_time,verbose2);

  // ==================================          
  // ==== create instance primrefs ====
  // ==================================
    
  if (numInstances)
    numInstances = createInstances_initPLOCPrimRefs(gpu_queue,args.geometries,numGeometries,prims_per_geom_prefix_sum,MAX_WGS,bvh2,numQuads + numProcedurals,host_device_tasks,create_primref_time,verbose2);

  // === recompute actual number of primitives ===
  numPrimitives = numQuads + numInstances + numProcedurals;

  if (unlikely(verbose2))
    PRINT4(numPrimitives,numQuads,numInstances,numProcedurals);
  
  // ===========================    
  // === empty scene again ? ===
  // ===========================

  if (unlikely(numPrimitives == 0))
  {
    QBVH6* qbvh  = (QBVH6*)args.accelBuffer;       
    BBox3f geometryBounds (empty);
    qbvh->bounds = geometryBounds;
    qbvh->numPrims       = 0;                                                                        
    qbvh->nodeDataStart  = 2;
    qbvh->nodeDataCur    = 3;
    qbvh->leafDataStart  = 3;
    qbvh->leafDataCur    = 3;        
    new (qbvh->nodePtr(2)) QBVH6::InternalNode6(NODE_TYPE_INTERNAL);
    if (args.accelBufferBytesOut)
      *args.accelBufferBytesOut = 3*64;
    if (args.boundsOut) *args.boundsOut = *(RTHWIF_AABB*)&geometryBounds;
    return RTHWIF_ERROR_NONE;
  }      

  timer.stop(BuildTimer::PRE_PROCESS);
  timer.add_to_device_timer(BuildTimer::PRE_PROCESS,create_primref_time);    
  if (unlikely(verbose2)) std::cout << "create quads/userGeometries/instances etc, init primrefs: " << timer.get_host_timer() << " ms (host) " << create_primref_time << " ms (device) " << std::endl;
    
  const GeometryTypeRanges geometryTypeRanges(numQuads,numProcedurals,numInstances);        

  
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

  if (convert_success == false)
  {
    if (args.accelBufferBytesOut) *args.accelBufferBytesOut = estimateAccelBufferSize(numQuads, numInstances, numProcedurals); // re-compute with INTERNAL_NODE_FACTOR = 1.0f;
    return RTHWIF_ERROR_RETRY;
  }
    
#if defined(EMBREE_SYCL_ALLOC_DISPATCH_GLOBALS)
    HWAccel* hwaccel = (HWAccel*)args.accelBuffer;  
    hwaccel->dispatchGlobalsPtr = (uint64_t)args.dispatchGlobalsPtr;
#endif      

    if (args.accelBufferBytesOut)
      *args.accelBufferBytesOut = totalSize;

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
  return RTHWIF_ERROR_NONE;    
}

#endif      


}

