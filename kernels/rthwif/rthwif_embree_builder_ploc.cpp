#include "rthwif_embree.h"
#include "rthwif_embree_builder.h"
#include "rthwif_internal.h"
#include "rthwif_embree_builder_ploc.h"
#include "builder/qbvh6.h"
#include "../common/algorithms/parallel_reduce.h"
#include "rthwif_embree_builder_stoch.h"

// === less than threshold, a single workgroup is used to perform all PLOC iterations in a single kernel launch ===
#define SINGLE_WG_SWITCH_THRESHOLD            4*1024

// === less than threshold, 40bits morton code + 24bits index are used, otherwise 64bit morton code + 32bit index ===
#define FAST_MC_NUM_PRIMS_THRESHOLD           1024*1024

// === max number of primitives fitting in 24bits ===
#define FAST_MC_MAX_NUM_PRIMS                 ((uint)1<<24)

// === less than threshold, a single workgroup is used for all radix sort iterations ===
#define SMALL_SORT_THRESHOLD                  1024*4

// === maximum number of workgroups with 1024 elements, DG2/PVC perform best with 64 ===
#define MAX_LARGE_WGS                         256

// === rebalance if BVH2 subtrees are degenerated ===
#define BVH2_REBALANCE                        1

#if defined(EMBREE_SYCL_GPU_BVH_BUILDER)      

namespace embree
{
    __forceinline gpu::AABB3f convert_AABB3f(const gpu::AABB &aabb)
    {
      return gpu::AABB3f(aabb.lower.x(), aabb.lower.y(), aabb.lower.z(), aabb.upper.x(), aabb.upper.y(), aabb.upper.z());
    }

    __forceinline gpu::AABB convert_AABB(const gpu::AABB3f &aabb)
    {
      return gpu::AABB(float4(aabb.lower_x,aabb.lower_y,aabb.lower_z,0.0f),
                  float4(aabb.upper_x,aabb.upper_y,aabb.upper_z,0.0f));
    }

  using namespace embree::isa;

  __forceinline size_t estimateSizeInternalNodes(const size_t numQuads, const size_t numInstances, const size_t numProcedurals, const bool conservative)
  {
    const size_t N = numQuads + numInstances + numProcedurals; 
    // === conservative estimate ===
    size_t numFatLeaves = 0;
    if (conservative)
      numFatLeaves = ceilf( (float)N/2 ) + ceilf( (float)numInstances/2 ); // FIXME : better upper bound for instance case
    else
      numFatLeaves = ceilf( (float)N/3 ) + ceilf( (float)numInstances/2 ); // FIXME : better upper bound for instance case        
    const size_t numInnerNodes = ceilf( (float)numFatLeaves/4 ); 
    return gpu::alignTo(std::max( ((numFatLeaves + numInnerNodes) * 64) , N * 16),64);
  }

  __forceinline size_t estimateSizeLeafNodes(const size_t numQuads, const size_t numInstances, const size_t numProcedurals)
  {
    return (numQuads + numProcedurals + 2 * numInstances) * 64;  
  }

  __forceinline size_t estimateAccelBufferSize(const size_t numQuads, const size_t numInstances, const size_t numProcedurals, const bool conservative)
  {
    const size_t header              = 128;
    const size_t node_size           = estimateSizeInternalNodes(numQuads,numInstances,numProcedurals,conservative);
    const size_t leaf_size           = estimateSizeLeafNodes(numQuads,numInstances,numProcedurals); 
    const size_t totalSize           = header + node_size + leaf_size;
    return totalSize;
  }

  __forceinline size_t estimateScratchBufferSize(const size_t numPrimitives)
  {
    // === sizeof(size_t)*MAX_LARGE_WGS for prefix sums across large work groups ===
    return sizeof(PLOCGlobals) + sizeof(size_t)*MAX_LARGE_WGS + numPrimitives * sizeof(LeafGenerationData);
  }

  uint getBVH2Depth(BVH2Ploc *bvh2, uint index, const uint numPrimitives)
  {
    if (BVH2Ploc::getIndex(index) < numPrimitives) //isLeaf 
      return 1;
    else
      return 1 + std::max(getBVH2Depth(bvh2,bvh2[index].leftIndex(),numPrimitives),getBVH2Depth(bvh2,bvh2[index].rightIndex(),numPrimitives));
  }

  uint getNumLeaves(BVH2Ploc *bvh2, uint index, const uint numPrimitives)
  {
    if (BVH2Ploc::getIndex(index) < numPrimitives) //isLeaf 
      return 1;
    else
      return getNumLeaves(bvh2,bvh2[index].leftIndex(),numPrimitives) + getNumLeaves(bvh2,bvh2[index].rightIndex(),numPrimitives);
  }


  uint getNumFatLeaves(BVH2Ploc *bvh2, uint index, const uint numPrimitives)
  {
    if (BVH2Ploc::isFatLeaf(index,numPrimitives)) //isLeaf 
      return 1;
    else
      return getNumFatLeaves(bvh2,bvh2[index].leftIndex(),numPrimitives) + getNumFatLeaves(bvh2,bvh2[index].rightIndex(),numPrimitives);
  }
  


  void printBVH2Path(BVH2Ploc *bvh2, uint index, const uint numPrimitives)
  {
    if (BVH2Ploc::getIndex(index) < numPrimitives) //isLeaf
    {
      PRINT2(index,"LEAF");
    }
    else
    {
      const uint depth = getBVH2Depth(bvh2,index,numPrimitives);
      const uint leftIndex = bvh2[index].leftIndex();
      const uint rightIndex = bvh2[index].rightIndex();
      const bool isFatLeafLeft = BVH2Ploc::isFatLeaf(bvh2[index].left,numPrimitives);
      const bool isFatLeafRight = BVH2Ploc::isFatLeaf(bvh2[index].right,numPrimitives);
      const uint numLeavesLeft = getNumLeaves(bvh2,leftIndex,numPrimitives);
      const uint numLeavesRight = getNumLeaves(bvh2,rightIndex,numPrimitives);      
      PRINT6(index,depth,leftIndex,rightIndex,isFatLeafLeft,isFatLeafRight);
      PRINT2(leftIndex,numLeavesLeft);
      PRINT2(rightIndex,numLeavesRight);
      
      if (!isFatLeafLeft)
        printBVH2Path(bvh2, leftIndex,numPrimitives);
      if (!isFatLeafRight)      
        printBVH2Path(bvh2,rightIndex,numPrimitives);      
    }
  }  
  
  
  void checkBVH2PlocHW(BVH2Ploc *bvh2, uint index,uint &nodes,uint &leaves,float &nodeSAH, float &leafSAH, uint &maxDepth,const uint numPrimitives, const uint bvh2_max_allocations, const uint depth)
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
      uint indices[BVH_BRANCHING_FACTOR];
      const uint numChildren = openBVH2MaxAreaSortChildren(BVH2Ploc::getIndex(index),indices,bvh2,numPrimitives);
      for (uint i=0;i<numChildren;i++)
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
  __forceinline RTHWIF_BUILD_ACCEL_ARGS rthwifPrepareBuildAccelArgs(const RTHWIF_BUILD_ACCEL_ARGS& args_i)
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
                      case RTHWIF_GEOMETRY_TYPE_TRIANGLES  :
                      {
                        counts.numTriangles   += ((RTHWIF_GEOMETRY_TRIANGLES_DESC*)  geom)->triangleCount;
                        counts.numQuadBlocks  += (((RTHWIF_GEOMETRY_TRIANGLES_DESC *)geom)->triangleCount+TRIANGLE_QUAD_BLOCK_SIZE-1)/TRIANGLE_QUAD_BLOCK_SIZE;
                        break;
                      }
                      case RTHWIF_GEOMETRY_TYPE_QUADS      :
                      {
                        counts.numQuads       += ((RTHWIF_GEOMETRY_QUADS_DESC*)  geom)->quadCount;
                        counts.numQuadBlocks  += (((RTHWIF_GEOMETRY_QUADS_DESC *)geom)->quadCount+TRIANGLE_QUAD_BLOCK_SIZE-1)/TRIANGLE_QUAD_BLOCK_SIZE;                        
                        break;
                      }
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

  static uint createLeafPrimitives(PLOCGlobals *const globals, BVH2Ploc* bvh2Ploc, uint *indices, uint start, uint end, uint depth = 0) {
    //PRINT4("leaf", depth, start, end);
    if (start == end - 1) {
      return indices[start];
    }

    auto idxPloc = globals->bvh2_index_allocator++;
    //PRINT(idxPloc);
    auto &plocNode = bvh2Ploc[idxPloc];

    uint mid = start + (end - start) / 2;
    plocNode.left = createLeafPrimitives(globals, bvh2Ploc, indices, start, mid, depth + 1);
    plocNode.right = createLeafPrimitives(globals, bvh2Ploc, indices, mid, end, depth + 1);

    //PRINT2(plocNode.leftIndex(), plocNode.rightIndex());
    const auto &leftNode = bvh2Ploc[plocNode.leftIndex()];
    const auto &rightNode = bvh2Ploc[plocNode.rightIndex()];

    plocNode.bounds = gpu::merge(leftNode.bounds, rightNode.bounds);

    return BVH2Ploc::makeFatLeaf(idxPloc, 0);
  }

  static uint convertStochBVH2toPlocBVH2(PLOCGlobals *const globals, BVH2Ploc* bvh2Ploc, gpu::BVH2BuildRecord *bvh2Stoch, uint *indices, gpu::AABB* aabb, uint idxStoch, uint depth = 0) {
    //PRINT2(depth, idxStoch);
    auto &stochNode = bvh2Stoch[idxStoch];
    
    if (stochNode.isLeaf()) {
      gpu::AABB3f bounds;
      bounds.init();
      for (auto i = stochNode.start; i < stochNode.end; i++) {
        if (!stochNode.bounds.encloses(convert_AABB3f(aabb[indices[i]])))
          FATAL("prim not enclosed!");
        if (!stochNode.bounds.encloses(bvh2Ploc[indices[i]].bounds))
          FATAL("prim not enclosed 2!");
          bounds.extend(convert_AABB3f(aabb[indices[i]]));
      }
      if (bounds != stochNode.bounds) {
        PRINT2(stochNode.bounds, bounds);
        FATAL("invalid bounds");
      }
      return createLeafPrimitives(globals, bvh2Ploc, indices, stochNode.start, stochNode.end, depth);
    }

    auto idxPloc = globals->bvh2_index_allocator++;
    auto &plocNode = bvh2Ploc[idxPloc];

    plocNode.left = convertStochBVH2toPlocBVH2(globals, bvh2Ploc, bvh2Stoch, indices, aabb, stochNode.left, depth + 1);
    plocNode.right = convertStochBVH2toPlocBVH2(globals, bvh2Ploc, bvh2Stoch, indices, aabb, stochNode.right, depth + 1);

    const auto &leftNode = bvh2Ploc[plocNode.leftIndex()];
    const auto &rightNode = bvh2Ploc[plocNode.rightIndex()];

    plocNode.bounds = gpu::merge(leftNode.bounds, rightNode.bounds);

    if (stochNode.bounds != gpu::merge(bvh2Stoch[stochNode.left].bounds, bvh2Stoch[stochNode.right].bounds)) {
      FATAL("invalid bounds");
    }

    if (plocNode.bounds != stochNode.bounds) {
      PRINT2(plocNode.bounds, stochNode.bounds);
      FATAL("invalid bounds");
    }


    return idxPloc;
  }

  RTHWIF_API RTHWIF_ERROR rthwifGetAccelSizeGPU(const RTHWIF_BUILD_ACCEL_ARGS& args_i, RTHWIF_ACCEL_SIZE& size_o, void *sycl_queue, uint verbose_level=0)
  {
    double time0 = getSeconds();
    
    RTHWIF_BUILD_ACCEL_ARGS args = rthwifPrepareBuildAccelArgs(args_i);
    const RTHWIF_GEOMETRY_DESC** geometries = args.geometries;
    const uint numGeometries = args.numGeometries;
    sycl::queue  &gpu_queue  = *(sycl::queue*)sycl_queue;

    // =============================================================================    
    // === GPU-based primitive count estimation including triangle quadification ===
    // =============================================================================
    
    const PrimitiveCounts primCounts = getEstimatedPrimitiveCounts(gpu_queue,geometries,numGeometries,verbose_level >= 2);            

    const uint numTriangles       = primCounts.numTriangles; // === original number of triangles ===
    const uint numMergedTrisQuads = primCounts.numMergedTrisQuads;
    const uint numQuads           = primCounts.numQuads;
    const uint numProcedurals     = primCounts.numProcedurals;
    const uint numInstances       = primCounts.numInstances;
  
    const uint numPrimitives = numMergedTrisQuads + numProcedurals + numInstances;

    // =============================================    
    // === allocation for empty scene is default ===
    // =============================================
    
    size_t expectedBytes = 3*64; 
    size_t worstCaseBytes = 4*64;

    if (numPrimitives)
    {    
      expectedBytes  = estimateAccelBufferSize(     numMergedTrisQuads, numInstances, numProcedurals, false);
      worstCaseBytes = estimateAccelBufferSize(numQuads + numTriangles, numInstances, numProcedurals, true);    
    }

    // ===============================================    
    // === estimate accel and scratch buffer sizes ===
    // ===============================================
    
    const size_t scratchBytes = estimateScratchBufferSize(std::max(numPrimitives,numGeometries));

    if (verbose_level >= 2)
    {
      PRINT5(numMergedTrisQuads,numTriangles,numQuads,numProcedurals,numInstances);      
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

    double time1 = getSeconds();
    if (verbose_level >= 1)
      std::cout << "rthwifGetAccelSizeGPU time = " << (float)(time1-time0)*1000.0f << " ms" << std::endl;
    
    return RTHWIF_ERROR_NONE;
  }

  RTHWIF_API RTHWIF_ERROR rthwifPrefetchAccelGPU(const RTHWIF_BUILD_ACCEL_ARGS& args, void *sycl_queue, uint verbose_level=0)
  {
    double time0 = getSeconds();
    
    sycl::queue  &gpu_queue  = *(sycl::queue*)sycl_queue;

#if 0
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
#endif  
    // ======================================================    
    // === DUMMY KERNEL TO TRIGGER REMAINING USM TRANSFER ===
    // ======================================================
    
    sycl::event queue_event =  gpu_queue.submit([&](sycl::handler &cgh) { cgh.single_task([=]() {}); });
    gpu::waitOnEventAndCatchException(queue_event);

    double time1 = getSeconds();
    if (verbose_level >= 1)
      std::cout << "rthwifPrefetchAccelGPU time = " << (float)(time1-time0)*1000.0f << " ms" << std::endl;
    
    return RTHWIF_ERROR_NONE;      
  }

  RTHWIF_API RTHWIF_ERROR rthwifBuildAccelGPU(const RTHWIF_BUILD_ACCEL_ARGS& args, DeviceGPU* device, void *sycl_queue, uint verbose_level=0)
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
    const uint gpu_maxComputeUnits  = gpu_queue.get_device().get_info<sycl::info::device::max_compute_units>();
    const uint MAX_WGS = gpu_maxComputeUnits / 8;
    
    uint *host_device_tasks = (uint*)sycl::aligned_alloc(64,HOST_DEVICE_COMM_BUFFER_SIZE,gpu_queue.get_device(),gpu_queue.get_context(),sycl::usm::alloc::host);

    
    if (unlikely(verbose2))
    {
      const uint gpu_maxWorkGroupSize = gpu_queue.get_device().get_info<sycl::info::device::max_work_group_size>();
      const uint gpu_maxLocalMemory   = gpu_queue.get_device().get_info<sycl::info::device::local_mem_size>();    
      PRINT("PLOC++ GPU BVH BUILDER");            
      PRINT( gpu_queue.get_device().get_info<sycl::info::device::global_mem_size>() );
      PRINT(gpu_maxWorkGroupSize);
      PRINT(gpu_maxComputeUnits);
      PRINT(gpu_maxLocalMemory);
    }

    // =============================    
    // === setup scratch pointer ===
    // =============================
    
    PLOCGlobals *globals = (PLOCGlobals *)args.scratchBuffer;
    uint *const sync_mem = (uint*)((char*)args.scratchBuffer + sizeof(PLOCGlobals));
    uint *const scratch  = (uint*)((char*)args.scratchBuffer + sizeof(PLOCGlobals) + sizeof(uint)*MAX_LARGE_WGS);    
  
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
  
    const RTHWIF_GEOMETRY_DESC** geometries = args.geometries;
    uint numGeometries                = args.numGeometries;
 
    double device_prim_counts_time = 0.0f;
  
    const PrimitiveCounts primCounts = countPrimitives(gpu_queue,geometries,numGeometries,globals,scratch,host_device_tasks,device_prim_counts_time,verbose1); 

    // ================================================
    
    timer.stop(BuildTimer::PRE_PROCESS);
    timer.add_to_device_timer(BuildTimer::PRE_PROCESS,device_prim_counts_time);                              
    if (unlikely(verbose2)) std::cout << "=> Count Primitives from Geometries: " << timer.get_host_timer() << " ms (host) " << device_prim_counts_time << " ms (device) " << std::endl;      
  
    uint numQuads            = primCounts.numQuads + primCounts.numTriangles; // no quadification taken into account at this point
    uint numProcedurals      = primCounts.numProcedurals;
    uint numInstances        = primCounts.numInstances;
    const uint numQuadBlocks = primCounts.numQuadBlocks;

    const uint expected_numPrimitives = numQuads + numProcedurals + numInstances;    

    // =================================================    
    // === empty scene before removing invalid prims ===
    // =================================================
    
    if (unlikely(expected_numPrimitives == 0)) createEmptyBVH(args,gpu_queue);
        
    if (numQuads)
    {
      // ==================================================
      // === compute correct quadification using blocks === 
      // ==================================================

      timer.start(BuildTimer::PRE_PROCESS);      
      double device_quadification_time = 0.0;
      numQuads = countQuadsPerGeometryUsingBlocks(gpu_queue,globals,args.geometries,numGeometries,numQuadBlocks,scratch,scratch+numGeometries,host_device_tasks,device_quadification_time,verbose1);
      timer.stop(BuildTimer::PRE_PROCESS);
      timer.add_to_device_timer(BuildTimer::PRE_PROCESS,device_quadification_time);
      if (unlikely(verbose2)) std::cout << "=> Count " << numQuads << " Quads " << timer.get_host_timer() << " ms (host) " << (float)device_quadification_time << " ms (device) " << std::endl;
    }
    
    // ================================
    // === estimate size of the BVH ===
    // ================================

    size_t numPrimitives             = numQuads + numInstances + numProcedurals;  // actual #prims can be lower due to invalid instances or procedurals but quads count is accurate at this point
    const size_t allocated_size      = args.accelBufferBytes;
    const size_t header              = 128;
    const size_t leaf_size           = estimateSizeLeafNodes(numQuads,numInstances,numProcedurals);
    const size_t node_size           = (header + leaf_size) <= allocated_size ? allocated_size - leaf_size - header : 0; 
    const size_t node_data_start     = header;
    const size_t leaf_data_start     = header + node_size;
      
    // =================================================================
    // === if allocated accel buffer is too small, return with error ===
    // =================================================================

    const uint required_size = header + estimateSizeInternalNodes(numQuads,numInstances,numProcedurals,false) + leaf_size;
    if (unlikely(allocated_size < required_size))
    {
      if (unlikely(verbose2))
      {
        PRINT2(required_size,allocated_size);
        PRINT2(node_size,estimateSizeInternalNodes(numQuads,numInstances,numProcedurals,false));        
        PRINT3("RETRY BVH BUILD DUE BECAUSE OF SMALL ACCEL BUFFER ALLOCATION!!!", args.accelBufferBytes,required_size );
      }
      if (args.accelBufferBytesOut) *args.accelBufferBytesOut = required_size;
      if (host_device_tasks) sycl::free(host_device_tasks,gpu_queue.get_context());
      return RTHWIF_ERROR_RETRY;
    }

    const size_t conv_mem_size = sizeof(numPrimitives)*numPrimitives;
    const size_t NUM_ACTIVE_LARGE_WGS = min((numPrimitives+LARGE_WG_SIZE-1)/LARGE_WG_SIZE,(size_t)MAX_WGS);

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
    BVH2SubTreeState *const bvh2_subtree_size = (BVH2SubTreeState*) (bvh_mem + 2 * numPrimitives * sizeof(uint)); // * 2        
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
      createQuads_initPLOCPrimRefs(gpu_queue,globals,args.geometries,numGeometries,numQuadBlocks,scratch,bvh2,0,create_primref_time,verbose1);
    
    // ====================================          
    // ==== create procedural primrefs ====
    // ====================================

    if (numProcedurals)
      numProcedurals = createProcedurals_initPLOCPrimRefs(gpu_queue,args.geometries,numGeometries,sync_mem,NUM_ACTIVE_LARGE_WGS,bvh2,numQuads,host_device_tasks,create_primref_time,verbose1);

    // ==================================          
    // ==== create instance primrefs ====
    // ==================================
    
    if (numInstances)
      numInstances = createInstances_initPLOCPrimRefs(gpu_queue,args.geometries,numGeometries,sync_mem,NUM_ACTIVE_LARGE_WGS,bvh2,numQuads + numProcedurals,host_device_tasks,create_primref_time,verbose1);

    // =================================================================================================    
    // === recompute actual number of primitives after quadification and removing of invalid entries ===
    // =================================================================================================
    
    numPrimitives = numQuads + numInstances + numProcedurals;
    const GeometryTypeRanges geometryTypeRanges(numQuads,numProcedurals,numInstances);        

#if 1
    gpu::AABB* aabb = (gpu::AABB*)sycl::aligned_alloc(64,sizeof(gpu::AABB)*numPrimitives,device->getGPUDevice(),device->getGPUContext(),sycl::usm::alloc::shared);
    {
      const uint wgSize = 32;
      sycl::event queue_event = gpu_queue.submit([&](sycl::handler &cgh) { //7
                                              const sycl::nd_range<1> nd_range(sycl::range<1>(gpu::alignTo(numPrimitives,wgSize)),sycl::range<1>(wgSize));                                                        
                                              cgh.parallel_for(nd_range,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(32) {
                                                uint i = item.get_global_id(0);
                                                if (i < numPrimitives)
                                                  aabb[i] = convert_AABB(bvh2[i].bounds);
                                              });
                                          });
    }
    gpu::waitOnQueueAndCatchException(gpu_queue);

    StochReturn res = rthwifBuildStoch(device, gpu_queue, numPrimitives, aabb);
    
    globals->rootIndex = convertStochBVH2toPlocBVH2(globals, bvh2, res.bvh2, res.indices, aabb, res.rootIndex);
    globals->geometryBounds = bvh2[globals->rootIndex].bounds;
#else    
    if (unlikely(verbose2))
    {
      PRINT4(numPrimitives,numQuads,numInstances,numProcedurals);
      PRINT3(node_size,leaf_size,args.accelBufferBytes);
      PRINT2(node_size/64,leaf_size/64);      
    }      
  
    // =================================================================================    
    // === test for empty scene again after all final primitive counts are available ===
    // =================================================================================

    if (unlikely(numPrimitives == 0))
    {
      if (host_device_tasks) sycl::free(host_device_tasks,gpu_queue.get_context());      
      return createEmptyBVH(args,gpu_queue);
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

    const bool fastMCMode = numPrimitives < FAST_MC_NUM_PRIMS_THRESHOLD || (args.quality == RTHWIF_BUILD_QUALITY_LOW && numPrimitives < FAST_MC_MAX_NUM_PRIMS);    
    
    timer.start(BuildTimer::PRE_PROCESS);        
    double device_compute_mc_time = 0.0f;

    if (!fastMCMode)
      computeMortonCodes64Bit_SaveMSBBits(gpu_queue,&globals->centroidBounds,mc0,bvh2,(uint*)bvh2_subtree_size,numPrimitives,device_compute_mc_time,verbose1);
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
      const uint scratchMemWGs = gpu::getNumWGsScratchSize(conv_mem_size);
      const uint nextPowerOf2 =  1 << (32 - sycl::clz(numPrimitives) - 1);
      const uint sortWGs = min(max(min((int)nextPowerOf2/8192,(int)gpu_maxComputeUnits/4),1),(int)scratchMemWGs);

      sycl::event initial = sycl::event();
      sycl::event block0  = gpu::radix_sort_Nx8Bit(gpu_queue, morton_codes[0], morton_codes[1], numPrimitives, (uint*)scratch, 4, 8, initial, sortWGs);      
      sycl::event restore = restoreMSBBits(gpu_queue,mc0,(uint*)bvh2_subtree_size,numPrimitives,block0,verbose1);      
      sycl::event block1  = gpu::radix_sort_Nx8Bit(gpu_queue, morton_codes[0], morton_codes[1], numPrimitives, (uint*)scratch, 4, 8, restore, sortWGs);
      gpu::waitOnEventAndCatchException(block1);      
    }
    else
    {
      if (numPrimitives < SMALL_SORT_THRESHOLD)
        gpu::radix_sort_single_workgroup(gpu_queue, (uint64_t *)mc0, (uint64_t *)mc1, numPrimitives, 3,8);
      else
      {
        const uint scratchMemWGs = gpu::getNumWGsScratchSize(conv_mem_size);        
        const uint nextPowerOf2 =  1 << (32 - sycl::clz(numPrimitives) - 1);          
        const uint sortWGs = min(max(min((int)nextPowerOf2/LARGE_WG_SIZE,(int)gpu_maxComputeUnits/4),1),(int)scratchMemWGs);
        sycl::event initial = sycl::event();
        sycl::event block0  = gpu::radix_sort_Nx8Bit(gpu_queue, (gpu::MortonCodePrimitive40x24Bits3D*)morton_codes[1], (gpu::MortonCodePrimitive40x24Bits3D*)morton_codes[0], numPrimitives, (uint*)scratch, 3, 8, initial, sortWGs);
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

    uint numPrims = numPrimitives;
    // ===================================================================================================================================================
    // ===================================================================================================================================================
    // ===================================================================================================================================================

    // === 8 or 16-wide search radius dependening on compiler flags ===
    const uint SEARCH_RADIUS_SHIFT = args.quality == RTHWIF_BUILD_QUALITY_LOW ? 3 : 4;
    
    double device_ploc_iteration_time = 0.0f;
        
    uint iteration = 0;
  
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
      
        const uint new_numPrims = *host_device_tasks;
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

#endif

#if BVH2_REBALANCE == 1
        // ===============================================================================================================
        // ========================================== rebalance BVH2 if degenerated ======================================
        // ===============================================================================================================
#if 0
        uint maxDepth = 0;
        for (uint i=0;i<numPrims;i++)
        {
          const uint depth = getBVH2Depth(bvh2,cluster_index_source[i],numPrimitives);
          maxDepth = max(maxDepth,depth);
        }
        PRINT(maxDepth);
#endif
        
        double rebalanceBVH2_time = 0.0f;
        rebalanceBVH2(gpu_queue,bvh2,bvh2_subtree_size,numPrimitives,rebalanceBVH2_time,verbose1);
        if (unlikely(verbose2))
          PRINT(rebalanceBVH2_time);
        timer.add_to_device_timer(BuildTimer::BUILD,rebalanceBVH2_time);

#if 0
        maxDepth = 0;
        for (uint i=0;i<numPrims;i++)
        {
          const uint depth = getBVH2Depth(bvh2,cluster_index_source[i],numPrimitives);
          maxDepth = max(maxDepth,depth);
        }
        PRINT(maxDepth);        
#endif
        // ===============================================================================================================
        // ===============================================================================================================
        // ===============================================================================================================
#endif        
                
    // =====================================                
    // === check and convert BVH2 (host) ===
    // =====================================

    if (unlikely(verbose2))
        PRINT2(globals->bvh2_index_allocator,2*numPrimitives);        
    
    if (unlikely(verbose2))
    {
      PRINT2(globals->bvh2_index_allocator,2*numPrimitives);              
      if (globals->bvh2_index_allocator >= 2*numPrimitives)
        FATAL("BVH2 construction, allocator");
      PRINT(globals->rootIndex);      
      uint nodes = 0;
      uint leaves = 0;
      float nodeSAH = 0;
      float leafSAH = 0;
      uint maxDepth = 0;
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
    const bool convert_success = convertBVH2toQBVH6(gpu_queue,globals,host_device_tasks,args.geometries,qbvh,bvh2,leafGenData,numPrimitives,numInstances != 0,geometryTypeRanges,conversion_device_time,verbose1);

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
      std::cout << "=> BVH2 -> QBVH6 Flattening: " <<  timer.get_host_timer() << " ms (host) " << conversion_device_time << " ms (device) " << std::endl;

    // ==========================================================    
    // ==========================================================
    // ==========================================================

    if (unlikely(verbose2))
    {
      // === memory allocation and usage stats ===
      const uint nodes_used   = globals->node_mem_allocator_cur-globals->node_mem_allocator_start;
      const uint leaves_used  = globals->leaf_mem_allocator_cur-globals->leaf_mem_allocator_start;
      const float nodes_util  = 100.0f * (float)(globals->node_mem_allocator_cur-globals->node_mem_allocator_start) / (node_size/64);
      const float leaves_util = 100.0f * (float)(globals->leaf_mem_allocator_cur-globals->leaf_mem_allocator_start) / (leaf_size/64);
      PRINT4(globals->node_mem_allocator_start,globals->node_mem_allocator_cur,nodes_used,nodes_util);
      PRINT4(globals->leaf_mem_allocator_start,globals->leaf_mem_allocator_cur,leaves_used,leaves_util);      
      PRINT(globals->numLeaves);
    }

    if (unlikely(convert_success == false))
    {
      if (args.accelBufferBytesOut) *args.accelBufferBytesOut = estimateAccelBufferSize(numQuads, numInstances, numProcedurals, true); 
      if (host_device_tasks) sycl::free(host_device_tasks,gpu_queue.get_context());      
      return RTHWIF_ERROR_RETRY;
    }
    
#if defined(EMBREE_SYCL_ALLOC_DISPATCH_GLOBALS)
    {
      sycl::event queue_event =  gpu_queue.submit([&](sycl::handler &cgh) {
                                                    cgh.single_task([=]() {
                                                                      HWAccel* hwaccel = (HWAccel*)args.accelBuffer;  
                                                                      hwaccel->dispatchGlobalsPtr = (uint64_t)args.dispatchGlobalsPtr;                                                                      
                                                                    });
                                                  });
      gpu::waitOnEventAndCatchException(queue_event);
    }
#endif      

    if (args.accelBufferBytesOut)
      *args.accelBufferBytesOut = args.accelBufferBytes;

#if 0
    if (verbose2)
    {
      gpu::waitOnQueueAndCatchException(gpu_queue);
      
      qbvh->print(std::cout,qbvh->root(),0,6);
      BVHStatistics stats = qbvh->computeStatistics();      
      stats.print(std::cout);
      stats.print_raw(std::cout);
      PRINT("VERBOSE STATS DONE");
    }        
#endif

    if (host_device_tasks) sycl::free(host_device_tasks,gpu_queue.get_context());      
    
    if (unlikely(verbose1))
      std::cout << "=> BVH build time: host = " << timer.get_total_host_time() << " ms , device = " << timer.get_total_device_time() << " ms , numPrimitives (original) = " << expected_numPrimitives << " , numPrimitives (build) = " << numPrimitives << std::endl;

    return RTHWIF_ERROR_NONE;    
  }
}

#endif    
