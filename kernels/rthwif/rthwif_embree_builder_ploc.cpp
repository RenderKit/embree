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
  
  
  BBox3fa rthwifBuildPloc(Scene* scene, RTCBuildQuality quality_flags, Device::avector<char,64>& accel, const bool two_level=false)
  {
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
    
    // ===============================================================================================================
    const uint numGeoms = scene->size();
    

    double alloc_time0 = getSeconds(); //FIXME free

    PLOCGlobals *globals  = (PLOCGlobals*)sycl::aligned_alloc(64,sizeof(PLOCGlobals),deviceGPU->getGPUDevice(),deviceGPU->getGPUContext(),sycl::usm::alloc::shared);
    assert(globals);
    
    const size_t alloc_TriMeshes = sizeof(TriMesh)*(numGeoms+1);
    TriMesh *triMesh  = (TriMesh*)sycl::aligned_alloc(64,alloc_TriMeshes,deviceGPU->getGPUDevice(),deviceGPU->getGPUContext(),sycl::usm::alloc::shared); //FIXME: change to device
    const size_t alloc_GeomPrefixSums = sizeof(uint)*(numGeoms+1);
    uint *quads_per_geom_prefix_sum  = (uint*)sycl::aligned_alloc(64,alloc_GeomPrefixSums,deviceGPU->getGPUDevice(),deviceGPU->getGPUContext(),sycl::usm::alloc::shared);
    
    double alloc_time1 = getSeconds();
	
    if (unlikely(deviceGPU->verbosity(2)))
      std::cout << "USM allocation time for globals, tri meshes and prefix sums " << 1000 * (alloc_time1 - alloc_time0) << " ms for " << (double)(alloc_TriMeshes+alloc_GeomPrefixSums) / (1024*1024) << " MBs " << std::endl;     	

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
    }
    
    if (unlikely(deviceGPU->verbosity(2))) PRINT(numGeoms);

    
    
    // === DUMMY KERNEL TO TRIGGER USM TRANSFER ===
#if 1
    {	  
      sycl::event queue_event =  gpu_queue.submit([&](sycl::handler &cgh) {
                                                    cgh.single_task([=]() {
                                                                    });
                                                  });
      gpu::waitOnQueueAndCatchException(gpu_queue);
    }
#endif    


    // =============================
    // === count quads per block === 
    // =============================
    double quadification_time = 0.0f;
    countQuadsPerGeometry(gpu_queue,triMesh,numGeoms,quads_per_geom_prefix_sum,quadification_time,verbose);
   
    /* --- prefix sum over quads --- */
    uint numQuadsPerGeom = 0;
    for (uint ID=0;ID<numGeoms;ID++)
    {    
      const uint current = quads_per_geom_prefix_sum[ID];
      //PRINT2(ID,current);
      quads_per_geom_prefix_sum[ID] = numQuadsPerGeom;
      numQuadsPerGeom += current;

    }
    quads_per_geom_prefix_sum[numGeoms] = numQuadsPerGeom;
    const uint numPrimitives = numQuadsPerGeom;
  
    if (unlikely(deviceGPU->verbosity(2)))    
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
    
    alloc_time0 = getSeconds(); 
    
    if (accel.size() < totalSize) accel = std::move(Device::avector<char,64>(scene->device,totalSize));    
	
    QBVH6* qbvh   = (QBVH6*)accel.data();
    assert(qbvh);
    char *bvh_mem = (char*)accel.data() + header;
    assert(bvh_mem);
    const size_t conv_mem_size = sizeof(LeafGenerationData)*numPrimitives;    
    LeafGenerationData *leafGenData = (LeafGenerationData*)sycl::aligned_alloc(64,conv_mem_size,deviceGPU->getGPUDevice(),deviceGPU->getGPUContext(),sycl::usm::alloc::shared); // FIXME
    assert(conversionState);

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

    
    alloc_time1 = getSeconds();
	
    if (unlikely(deviceGPU->verbosity(2)))
      std::cout << "USM allocation time " << 1000 * (alloc_time1 - alloc_time0) << " ms for " << (double)totalUSMAllocations / (1024*1024) << " MBs " << std::endl;     	
      
    // ======================          
    // ==== init globals ====
    // ======================

    double t1,t2,preprocess_time = 0.0f;
          
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
    }	    

    // ===========================================================          
    // ==== merge triangles to quads, write out PLOC primrefs ====
    // ===========================================================

    mergeTriangleToQuads_initPLOCPrimRefs(gpu_queue,triMesh,numGeoms,quads_per_geom_prefix_sum,bvh2,quadification_time,verbose);

#if 0    
     for (uint i=0;i<numPrimitives;i++)
       if (!bvh2[i].bounds.isValid() || !bvh2[i].bounds.checkNumericalBounds() )
       {
         PRINT2(i,bvh2[i]);      
         FATAL("Numerical Bounds in BVH2");
       }
#endif
      
    double total_diff = 0;
    double total0 = getSeconds();	
      
        
    // ==========================================          
    // ==== get centroid and geometry bounds ====
    // ==========================================
        
    t1 = getSeconds();

    computeCentroidGeometryBounds(gpu_queue, &globals->geometryBounds, &globals->centroidBounds, bvh2, numPrimitives, verbose);
        
    t2 = getSeconds();
        
    if (unlikely(deviceGPU->verbosity(2)))
      std::cout << "Get Geometry and Centroid Bounds Phase " << 1000 * (t2 - t1) << " ms" << std::endl;		
   
    preprocess_time += 1000 * (t2 - t1);
    
    // ==============================          
    // ==== compute morton codes ====
    // ==============================

    t1 = getSeconds();

    if (!fastMCMode)
      computeMortonCodes64Bit_SaveMSBBits(gpu_queue,&globals->centroidBounds,mc0,bvh2,bvh2_subtree_size,numPrimitives,preprocess_time,verbose);
    else
      computeMortonCodes64Bit(gpu_queue,&globals->centroidBounds,(gpu::MortonCodePrimitive40x24Bits3D*)mc1,bvh2,numPrimitives,0,(uint64_t)-1,preprocess_time,verbose);
    
    t2 = getSeconds();
        
    if (unlikely(deviceGPU->verbosity(2)))
      std::cout << "Compute Morton Codes " << 1000 * (t2 - t1) << " ms" << std::endl;		
    
    // ===========================          
    // ==== sort morton codes ====
    // ===========================
        
    t1 = getSeconds();

    double sort_time = 0.0;
        

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
                                
    t2 = getSeconds();

        
    if (unlikely(deviceGPU->verbosity(2)))
      std::cout << "Sort Morton Codes " << 1000 * (t2 - t1) << " ms" << std::endl;
        
    preprocess_time += 1000 * (t2 - t1);


              
    // ===========================          
    // ====== init clusters ======
    // ===========================

    t1 = getSeconds();

    if (!fastMCMode)
      initClusters(gpu_queue,mc0,bvh2,cluster_index,bvh2_subtree_size,numPrimitives,preprocess_time,verbose);
    else
      initClusters(gpu_queue,(gpu::MortonCodePrimitive40x24Bits3D*)mc0,bvh2,cluster_index,bvh2_subtree_size,numPrimitives,preprocess_time,verbose);
    
    t2 = getSeconds();
        
    if (unlikely(deviceGPU->verbosity(2)))
      std::cout << "Init Clusters " << 1000 * (t2 - t1) << " ms" << std::endl;		

    uint numPrims = numPrimitives;

      
    // ===================================================================================================================================================
    // ===================================================================================================================================================
    // ===================================================================================================================================================
        
    double ploc_iteration_time = 0.0f;
        
    uint iteration = 0;

    double ploc_host_time0 = getSeconds();

    if (two_level)
    {
      static const uint PARALLEL_WG_NUM = MAX_WGS;        
      static const uint RANGE_THRESHOLD = numPrimitives/(4*PARALLEL_WG_NUM);
      static const uint BOTTOM_UP_THRESHOLD = 16;
      static const uint SEARCH_RADIUS_TOP_LEVEL = 32;
                    
      gpu::Range *ranges = (gpu::Range*)scratch_mem; //FIXME: need to store ranges somewhere else
        
      extractRanges(gpu_queue, &globals->numBuildRecords, mc0, ranges, numPrims, RANGE_THRESHOLD , ploc_iteration_time, verbose);

      
      gpu::radix_sort_single_workgroup(gpu_queue, (uint64_t*)ranges, (uint64_t*)ranges + globals->numBuildRecords, globals->numBuildRecords,0,8,ploc_iteration_time);
      
      parallelWGBuild(gpu_queue, &globals->bvh2_index_allocator, bvh2, cluster_index_source, cluster_index_dest, bvh2_subtree_size, ranges, globals->numBuildRecords, BOTTOM_UP_THRESHOLD, ploc_iteration_time, verbose);
      
      singleWGTopLevelBuild(gpu_queue, &globals->bvh2_index_allocator, bvh2, cluster_index_source, cluster_index_dest, bvh2_subtree_size, ranges, globals->numBuildRecords, SEARCH_RADIUS_TOP_LEVEL, ploc_iteration_time, verbose);
    }
    else
      for (;numPrims>1;iteration++)
      {
          
        double iteration_time = 0.0f;

        // ==================================================            
        // ==== single kernel path if #prims < threshold ====
        // ==================================================

        if (numPrims < SINGLE_WG_SWITCH_THRESHOLD)
        {
          double singleWG_time = 0.0f;
          singleWGBuild(gpu_queue, &globals->bvh2_index_allocator, bvh2, cluster_index_source, cluster_index_dest, bvh2_subtree_size, numPrims, singleWG_time, verbose);
          iteration_time += singleWG_time;
          numPrims = 1;
        }
        else  
        {            
          // ===================================================================================
          // ==== nearest neighbor search, merge clusters and create bvh2 nodes (fast path) ====
          // ===================================================================================

          const uint MERGED_KERNEL_WG_NUM = MAX_WGS;           
          const uint radius = SEARCH_RADIUS;          

          searchNearestNeighborMergeClustersCreateBVH2(gpu_queue,&globals->bvh2_index_allocator,bvh2,cluster_index_source,cluster_index_dest,bvh2_subtree_size,scratch_mem,numPrims,radius,MERGED_KERNEL_WG_NUM,iteration_time, false);


          // ===================================================            
          // ==== prefix_sum and compact cluster references ====
          // ===================================================
            
          computePrefixSum_compactClusterReferences(gpu_queue,&globals->numBuildRecords,cluster_index_dest,cluster_index_source,scratch_mem,numPrims,MERGED_KERNEL_WG_NUM,iteration_time,false);                        

          const uint new_numPrims = globals->numBuildRecords;            
          assert(new_numPrims < numPrims);          
          numPrims = new_numPrims;          
            
          // ==========================            
        }        
        ploc_iteration_time += iteration_time;
        if (unlikely(deviceGPU->verbosity(2)))
          PRINT4(iteration,numPrims,(float)ploc_iteration_time,(float)iteration_time);
      }

    const double ploc_host_time1 = getSeconds();
    const double ploc_host_time = (ploc_host_time1 - ploc_host_time0)*1000;
    if (unlikely(deviceGPU->verbosity(2)))    
      PRINT6(iteration,(float)ploc_iteration_time,(float)preprocess_time,(float)(ploc_iteration_time + preprocess_time),(float)ploc_host_time,(float)(ploc_host_time+preprocess_time));          
      
    double total1 = getSeconds();
    total_diff += (total1-total0); 
            
    if (unlikely(deviceGPU->verbosity(1)))
    {
      std::cout << "BVH2 GPU Ploc Builder DONE in " << 1000.*total_diff << " ms (host), " << (float)(ploc_iteration_time + preprocess_time) << " ms (device) : " << numPrimitives*0.000001f/total_diff << " Build MPrims/s (host), " << org_numPrimitives*0.000001f/total_diff << " Original MPrims/s (host) " << std::endl << std::flush;      
    }
    
    assert(bvh2_subtree_size[globals->bvh2_index_allocator-1] == numPrimitives);
    globals->rootIndex = globals->bvh2_index_allocator-1;
    if (globals->bvh2_index_allocator >= 2*numPrimitives)
      FATAL("BVH2 construction, allocator");
                
    /* --- check and convert BVH2 (host) --- */        
          
    if (unlikely(deviceGPU->verbosity(2)))
    {
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

      /* --- dummy kernel to trigger USM transfer --- */
      sycl::event queue_event =  gpu_queue.submit([&](sycl::handler &cgh) {
                                                    cgh.single_task([=]() {
                                                                    });
                                                  });
      gpu::waitOnQueueAndCatchException(gpu_queue);
    }

    double time_convert0 = getSeconds();
    
    /* --- convert BVH2 to QBVH6 --- */    
    convertBVH2toQBVH6(gpu_queue,globals,triMesh,qbvh,bvh2,globals->rootIndex,leafGenData,numPrimitives,node_size/64,verbose);

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
                                                                      qbvh->rootNodeOffset = 128;
                                                                      qbvh->numPrims       = org_numPrimitives;                                                                        
                                                                      qbvh->nodeDataStart  = globals->node_mem_allocator_start;
                                                                      qbvh->nodeDataCur    = globals->node_mem_allocator_cur;
                                                                      qbvh->leafDataStart  = globals->leaf_mem_allocator_start;
                                                                      qbvh->leafDataCur    = globals->leaf_mem_allocator_cur;
                                                                    });
                                                  });
      gpu::waitOnQueueAndCatchException(gpu_queue);
    }	    

    double time_convert1 = getSeconds();    
    if (unlikely(deviceGPU->verbosity(1)))
    {
      const double host_bvh2_qbvh6_conversion_time = (time_convert1 - time_convert0)*1000;
      std::cout << "BVH2 -> QBVH6 Flattening DONE in " <<  host_bvh2_qbvh6_conversion_time << " ms " << std::endl << std::flush;                  
    }
    
    // ==========================================================
    // ==========================================================

    BBox3fa geomBounds(Vec3fa(globals->geometryBounds.lower_x,globals->geometryBounds.lower_y,globals->geometryBounds.lower_z),
                       Vec3fa(globals->geometryBounds.upper_x,globals->geometryBounds.upper_y,globals->geometryBounds.upper_z));
    
    if (unlikely(deviceGPU->verbosity(2)))
    {    
      BVHStatistics stats = qbvh->computeStatistics();      
      stats.print(std::cout);
      stats.print_raw(std::cout);
    }
    if (deviceGPU) {
      HWAccel* hwaccel = (HWAccel*) accel.data();
      hwaccel->dispatchGlobalsPtr = (uint64_t) deviceGPU->dispatchGlobalsPtr;
    }

    if (quads_per_geom_prefix_sum) sycl::free(quads_per_geom_prefix_sum,deviceGPU->getGPUContext());
    
    if (globals)        sycl::free(globals,deviceGPU->getGPUContext());
    if (leafGenData)    sycl::free(leafGenData,deviceGPU->getGPUContext());
    
    return geomBounds;
  }
 
}


