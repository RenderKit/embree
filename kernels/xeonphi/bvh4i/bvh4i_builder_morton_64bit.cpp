// ======================================================================== //
// Copyright 2009-2014 Intel Corporation                                    //
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

#include "bvh4i_builder_morton.h"
#include "builders/builder_util.h"

#define MORTON_BVH4I_NODE_PREALLOC_FACTOR   0.8f
#define NUM_MORTON_IDS_PER_BLOCK            8
#define SINGLE_THREADED_BUILD_THRESHOLD     (MAX_MIC_THREADS*64)

//#define PROFILE
#define PROFILE_ITERATIONS 200

#define TIMER(x) 
#define DBG(x) 

#define L1_PREFETCH_ITEMS 8
#define L2_PREFETCH_ITEMS 44

namespace embree 
{
#if defined(DEBUG)
  extern AtomicMutex mtx;
#endif



  // =======================================================================================================
  // =======================================================================================================
  // =======================================================================================================

  __aligned(64) static double dt = 0.0f;

  BVH4iBuilderMorton64Bit::BVH4iBuilderMorton64Bit(BVH4i* bvh, void* geometry)
    : bvh(bvh), scene((Scene*)geometry), morton(NULL), node(NULL), accel(NULL), numPrimitives(0), numGroups(0),numNodes(0), numAllocatedNodes(0), size_morton(0), size_node(0), size_accel(0), numPrimitivesOld(-1)
  {
  }

  BVH4iBuilderMorton64Bit::~BVH4iBuilderMorton64Bit()
  {
    if (morton) {
      assert(size_morton > 0);
      os_free(morton,size_morton);
    }
  }


  void BVH4iBuilderMorton64Bit::allocateData(size_t threadCount)
  {
    /* preallocate arrays */
    const size_t additional_size = 16 * CACHELINE_SIZE;
    if (numPrimitivesOld != numPrimitives)
    {
      DBG(
	  DBG_PRINT( numPrimitivesOld );
	  DBG_PRINT( numPrimitives );
	  );

      numPrimitivesOld = numPrimitives;
      /* free previously allocated memory */
      if (morton) {
	assert(size_morton > 0);
	os_free(morton,size_morton);
      }
      if (node) { 
	assert(size_node > 0);
	os_free(node  ,size_node);
      }
      if (accel ) {
	assert(size_accel > 0);
	os_free(accel ,size_accel);
      }
      
      /* allocated memory for primrefs,nodes, and accel */
      const size_t minAllocNodes = (threadCount+1) * 2* ALLOCATOR_NODE_BLOCK_SIZE;


      const size_t numPrims      = numPrimitives+4;
      const size_t numNodes      = (size_t)((numPrimitives+3)/4);


      const size_t sizeNodeInBytes = sizeof(BVH4i::Node);
      const size_t sizeAccelInBytes = sizeof(Triangle1);


      const size_t size_morton_tmp = numPrims * sizeof(MortonID64Bit) + additional_size;

      size_node         = (numNodes * MORTON_BVH4I_NODE_PREALLOC_FACTOR + minAllocNodes) * sizeNodeInBytes + additional_size;
      size_accel        = numPrims * sizeAccelInBytes + additional_size;
      numAllocatedNodes = size_node / sizeof(BVHNode);

      morton = (MortonID64Bit* ) os_malloc(size_morton_tmp); 
      node   = (mic_i*)          os_malloc(size_node  );     
      accel  = (Triangle1*)      os_malloc(size_accel );     

      assert(morton != 0);
      assert(node   != 0);
      assert(accel  != 0);

      size_morton = size_morton_tmp;

#if DEBUG
      DBG_PRINT( minAllocNodes );
      DBG_PRINT( numNodes );
      DBG_PRINT(bvh->size_node);
      DBG_PRINT(bvh->size_accel);
      DBG_PRINT(numAllocatedNodes);
#endif

    }

    bvh->accel = accel;
    bvh->qbvh  = (BVH4i::Node*)node;
    bvh->size_node  = size_node;
    bvh->size_accel = size_accel;
  }

  void BVH4iBuilderMorton64Bit::build(size_t threadIndex, size_t threadCount) 
  {
    if (unlikely(g_verbose >= 2))
      {
	std::cout << "building BVH4i with 64Bit Morton builder (MIC)... " << std::flush;
      }
    
    /* do some global inits first */
    size_t numPrimitives = 0;       
    for (size_t i=0;i<scene->size();i++)
      {
	if (unlikely(scene->get(i) == NULL)) continue;
	if (unlikely((scene->get(i)->type != TRIANGLE_MESH))) continue;
	if (unlikely(!scene->get(i)->isEnabled())) continue;
	const TriangleMesh* __restrict__ const mesh = scene->getTriangleMesh(i);
	if (unlikely(mesh->numTimeSteps != 1)) continue;
	numGroups++;
	numPrimitives += mesh->numTriangles;
      }

    if (likely(numPrimitives == 0))
      {
	DBG(std::cout << "EMPTY SCENE BUILD" << std::endl);
	bvh->root = BVH4i::invalidNode;
	bvh->bounds = empty;
	bvh->qbvh = NULL;
	bvh->accel = NULL;
	return;
      }

    /* allocate memory arrays */
    allocateData(TaskScheduler::getNumThreads());

#if defined(PROFILE)
    size_t numTotalPrimitives = numPrimitives;
    std::cout << "STARTING PROFILE MODE" << std::endl << std::flush;
    std::cout << "primitives = " << numTotalPrimitives << std::endl;

    double dt_min = pos_inf;
    double dt_avg = 0.0f;
    double dt_max = neg_inf;
    size_t iterations = PROFILE_ITERATIONS;
    for (size_t i=0; i<iterations; i++) 
    {
      TaskScheduler::executeTask(threadIndex,threadCount,_build_parallel_morton,this,TaskScheduler::getNumThreads(),"build_parallel_morton");

      dt_min = min(dt_min,dt);
      dt_avg = dt_avg + dt;
      dt_max = max(dt_max,dt);
    }
    dt_avg /= double(iterations);

    std::cout << "[DONE]" << std::endl;
    std::cout << "  min = " << 1000.0f*dt_min << "ms (" << numTotalPrimitives/dt_min*1E-6 << " Mtris/s)" << std::endl;
    std::cout << "  avg = " << 1000.0f*dt_avg << "ms (" << numTotalPrimitives/dt_avg*1E-6 << " Mtris/s)" << std::endl;
    std::cout << "  max = " << 1000.0f*dt_max << "ms (" << numTotalPrimitives/dt_max*1E-6 << " Mtris/s)" << std::endl;
    std::cout << BVH4iStatistics<BVH4i::Node>(bvh).str();

#else
    DBG(DBG_PRINT(numPrimitives));


    if (likely(numPrimitives > SINGLE_THREADED_BUILD_THRESHOLD && TaskScheduler::getNumThreads() > 1))
      {
#if DEBUG
	DBG_PRINT( TaskScheduler::getNumThreads() );
	std::cout << "PARALLEL BUILD" << std::endl << std::flush;
#endif
	TaskScheduler::executeTask(threadIndex,threadCount,_build_parallel_morton,this,TaskScheduler::getNumThreads(),"build_parallel");
      }
    else
      {
	/* number of primitives is small, just use single threaded mode */
#if DEBUG
	std::cout << "SERIAL BUILD" << std::endl << std::flush;
#endif
	build_parallel_morton(0,1,0,0,NULL);
      }

    if (g_verbose >= 2) {
      double perf = numPrimitives/dt*1E-6;
      std::cout << "[DONE] " << 1000.0f*dt << "ms (" << perf << " Mtris/s), primitives " << numPrimitives << std::endl;
      std::cout << BVH4iStatistics<BVH4i::Node>(bvh).str();
    }
#endif
    
  }

    
  // =======================================================================================================
  // =======================================================================================================
  // =======================================================================================================

  void BVH4iBuilderMorton64Bit::initThreadState(const size_t threadID, const size_t numThreads)
  {
    const size_t numBlocks = (numPrimitives+NUM_MORTON_IDS_PER_BLOCK-1) / NUM_MORTON_IDS_PER_BLOCK;
    const size_t startID   =      ((threadID+0)*numBlocks/numThreads) * NUM_MORTON_IDS_PER_BLOCK;
    const size_t endID     = min( ((threadID+1)*numBlocks/numThreads) * NUM_MORTON_IDS_PER_BLOCK ,numPrimitives) ;
    
    assert(startID % NUM_MORTON_IDS_PER_BLOCK == 0);

    /* find first group containing startID */
    size_t group = 0, skipped = 0;
    for (; group<numGroups; group++) 
    {       
      if (unlikely(scene->get(group) == NULL)) continue;
      if (scene->get(group)->type != TRIANGLE_MESH) continue;
      const TriangleMesh* __restrict__ const mesh = scene->getTriangleMesh(group);
      if (unlikely(!mesh->isEnabled())) continue;
      if (unlikely(mesh->numTimeSteps != 1)) continue;

      const size_t numTriangles = mesh->numTriangles;	
      if (skipped + numTriangles > startID) break;
      skipped += numTriangles;
    }

    /* store start group and offset */
    thread_startGroup[threadID] = group;
    thread_startGroupOffset[threadID] = startID - skipped;
  }


  void BVH4iBuilderMorton64Bit::computeBounds(const size_t threadID, const size_t numThreads) 
  {
    const size_t numBlocks = (numPrimitives+NUM_MORTON_IDS_PER_BLOCK-1) / NUM_MORTON_IDS_PER_BLOCK;
    const size_t startID   =      ((threadID+0)*numBlocks/numThreads) * NUM_MORTON_IDS_PER_BLOCK;
    const size_t endID     = min( ((threadID+1)*numBlocks/numThreads) * NUM_MORTON_IDS_PER_BLOCK ,numPrimitives) ;
    assert(startID % NUM_MORTON_IDS_PER_BLOCK == 0);

    __aligned(64) Centroid_Scene_AABB bounds;
    bounds.reset();

    size_t currentID = startID;

    size_t startGroup = thread_startGroup[threadID];
    size_t offset = thread_startGroupOffset[threadID];

    mic_f bounds_centroid_min((float)pos_inf);
    mic_f bounds_centroid_max((float)neg_inf);

    for (size_t group = startGroup; group<numGroups; group++) 
    {       
      if (unlikely(scene->get(group) == NULL)) continue;
      if (unlikely(scene->get(group)->type != TRIANGLE_MESH)) continue;
      const TriangleMesh* __restrict__ const mesh = scene->getTriangleMesh(group);
      if (unlikely(!mesh->isEnabled())) continue;
      if (unlikely(mesh->numTimeSteps != 1)) continue;


      const char* __restrict__ cptr_tri = (char*)&mesh->triangle(offset);
      const unsigned int stride = mesh->triangles.getBufferStride();
      
      for (size_t i=offset; i<mesh->numTriangles && currentID < endID; i++, currentID++,cptr_tri+=stride)	 
	{
	  const TriangleMesh::Triangle& tri = *(TriangleMesh::Triangle*)cptr_tri;
	  prefetch<PFHINT_L1>(&tri + L1_PREFETCH_ITEMS);
	  prefetch<PFHINT_L2>(&tri + L2_PREFETCH_ITEMS);

	  const mic3f v = mesh->getTriangleVertices<PFHINT_L2>(tri);
	  const mic_f bmin  = min(min(v[0],v[1]),v[2]);
	  const mic_f bmax  = max(max(v[0],v[1]),v[2]);

	  const mic_f centroid2 = bmin+bmax;
	  bounds_centroid_min = min(bounds_centroid_min,centroid2);
	  bounds_centroid_max = max(bounds_centroid_max,centroid2);
	}
      
      if (unlikely(currentID == endID)) break;
      offset = 0;
    }

    store4f(&bounds.centroid2.lower,bounds_centroid_min);
    store4f(&bounds.centroid2.upper,bounds_centroid_max);
    
    global_bounds.extend_centroid_bounds_atomic(bounds); 
  }

  void BVH4iBuilderMorton64Bit::computeMortonCodes(const size_t threadID, const size_t numThreads)
  {
    const size_t numBlocks = (numPrimitives+NUM_MORTON_IDS_PER_BLOCK-1) / NUM_MORTON_IDS_PER_BLOCK;
    const size_t startID   =      ((threadID+0)*numBlocks/numThreads) * NUM_MORTON_IDS_PER_BLOCK;
    const size_t endID     = min( ((threadID+1)*numBlocks/numThreads) * NUM_MORTON_IDS_PER_BLOCK ,numPrimitives) ;
    assert(startID % NUM_MORTON_IDS_PER_BLOCK == 0);

    /* store the morton codes in 'morton' memory */
    MortonID64Bit* __restrict__ dest = morton + startID; 

    /* compute mapping from world space into 3D grid */
    const mic_f base     = broadcast4to16f((float*)&global_bounds.centroid2.lower);
    const mic_f diagonal = 
      broadcast4to16f((float*)&global_bounds.centroid2.upper) - 
      broadcast4to16f((float*)&global_bounds.centroid2.lower);
    const mic_f scale    = select(diagonal != 0, rcp(diagonal) * mic_f(LATTICE_SIZE_PER_DIM * 0.99f),mic_f(0.0f));

    size_t currentID = startID;
    size_t offset = thread_startGroupOffset[threadID];



    for (size_t group = thread_startGroup[threadID]; group<numGroups; group++) 
    {       
      if (unlikely(scene->get(group) == NULL)) continue;
      if (unlikely(scene->get(group)->type != TRIANGLE_MESH)) continue;
      const TriangleMesh* const mesh = scene->getTriangleMesh(group);
      if (unlikely(!mesh->isEnabled())) continue;
      if (unlikely(mesh->numTimeSteps != 1)) continue;

      const size_t numTriangles = min(mesh->numTriangles-offset,endID-currentID);
       
      const char* __restrict__ cptr_tri = (char*)&mesh->triangle(offset);
      const unsigned int stride = mesh->triangles.getBufferStride();
      
      for (size_t i=0; i<numTriangles; i++,cptr_tri+=stride)	  
      {
	prefetch<PFHINT_NTEX>(dest);

	const TriangleMesh::Triangle& tri = *(TriangleMesh::Triangle*)cptr_tri;

	prefetch<PFHINT_NT>(&tri + 16);

	const mic3f v = mesh->getTriangleVertices<PFHINT_L2>(tri);
	const mic_f bmin  = min(min(v[0],v[1]),v[2]);
	const mic_f bmax  = max(max(v[0],v[1]),v[2]);

	const mic_f cent  = bmin+bmax;
	const mic_i binID = convert_uint32((cent-base)*scale);

	dest->primID  = offset+i;
	dest->groupID = group;

	const size_t binIDx = binID[0];
	const size_t binIDy = binID[1];
	const size_t binIDz = binID[2];

	const size_t code  = bitInterleave(binIDx,binIDy,binIDz); // check
	dest->code = code;
	dest++;
	prefetch<PFHINT_L2EX>(dest + 4*4);

        currentID++;
      }

      offset = 0;
      if (currentID == endID) break;
    }

  }
  
  void BVH4iBuilderMorton64Bit::radixsort(const size_t threadID, const size_t numThreads)
  {
    const size_t numBlocks = (numPrimitives+NUM_MORTON_IDS_PER_BLOCK-1) / NUM_MORTON_IDS_PER_BLOCK;
    const size_t startID   = ((threadID+0)*numBlocks/numThreads) * NUM_MORTON_IDS_PER_BLOCK;
    const size_t endID     = ((threadID+1)*numBlocks/numThreads) * NUM_MORTON_IDS_PER_BLOCK;
    assert(startID % NUM_MORTON_IDS_PER_BLOCK == 0);
    assert(endID % NUM_MORTON_IDS_PER_BLOCK == 0);

    assert(((numThreads)*numBlocks/numThreads) * NUM_MORTON_IDS_PER_BLOCK == ((numPrimitives+7)&(-8)));

    MortonID64Bit* __restrict__ mortonID[2];
    mortonID[0] = (MortonID64Bit*) morton; 
    mortonID[1] = (MortonID64Bit*) node;


    /* we need 4 iterations to process all 32 bits */
    for (size_t b=0; b<4; b++)
    {
      const MortonID64Bit* __restrict__ const src = (MortonID64Bit*)mortonID[((b+0)%2)];
      MortonID64Bit*       __restrict__ const dst = (MortonID64Bit*)mortonID[((b+1)%2)];

      __assume_aligned(&radixCount[threadID][0],64);
      
      /* count how many items go into the buckets */

#pragma unroll(16)
      for (size_t i=0; i<16; i++)
	store16i(&radixCount[threadID][i*16],mic_i::zero());


      for (size_t i=startID; i<endID; i+=NUM_MORTON_IDS_PER_BLOCK) {
	prefetch<PFHINT_NT>(&src[i+L1_PREFETCH_ITEMS]);
	prefetch<PFHINT_L2>(&src[i+L2_PREFETCH_ITEMS]);
	
#pragma unroll(NUM_MORTON_IDS_PER_BLOCK)
	for (unsigned long j=0;j<NUM_MORTON_IDS_PER_BLOCK;j++)
	  {
	    const unsigned int index = src[i+j].getByte(b);
	    radixCount[threadID][index]++;
	  }
      }

      LockStepTaskScheduler::syncThreads(threadID,numThreads);


      /* calculate total number of items for each bucket */


      mic_i count[16];
#pragma unroll(16)
      for (size_t i=0; i<16; i++)
	count[i] = mic_i::zero();


      for (size_t i=0; i<threadID; i++)
#pragma unroll(16)
	for (size_t j=0; j<16; j++)
	  count[j] += load16i((int*)&radixCount[i][j*16]);
      
      __aligned(64) unsigned int inner_offset[RADIX_BUCKETS];

#pragma unroll(16)
      for (size_t i=0; i<16; i++)
	store16i(&inner_offset[i*16],count[i]);

#pragma unroll(16)
      for (size_t i=0; i<16; i++)
	count[i] = load16i((int*)&inner_offset[i*16]);

      for (size_t i=threadID; i<numThreads; i++)
#pragma unroll(16)
	for (size_t j=0; j<16; j++)
	  count[j] += load16i((int*)&radixCount[i][j*16]);	  

     __aligned(64) unsigned int total[RADIX_BUCKETS];

#pragma unroll(16)
      for (size_t i=0; i<16; i++)
	store16i(&total[i*16],count[i]);

      __aligned(64) unsigned int offset[RADIX_BUCKETS];

      /* calculate start offset of each bucket */
      offset[0] = 0;
      for (size_t i=1; i<RADIX_BUCKETS; i++)    
        offset[i] = offset[i-1] + total[i-1];
      
      /* calculate start offset of each bucket for this thread */

#pragma unroll(RADIX_BUCKETS)
	for (size_t j=0; j<RADIX_BUCKETS; j++)
          offset[j] += inner_offset[j];

      /* copy items into their buckets */
      for (size_t i=startID; i<endID; i+=NUM_MORTON_IDS_PER_BLOCK) {
	prefetch<PFHINT_NT>(&src[i+L1_PREFETCH_ITEMS]);
	prefetch<PFHINT_L2>(&src[i+L2_PREFETCH_ITEMS]);

#pragma nounroll
	for (unsigned long j=0;j<NUM_MORTON_IDS_PER_BLOCK;j++)
	  {
	    const unsigned int index = src[i+j].getByte(b);
	    assert(index < RADIX_BUCKETS);
	    dst[offset[index]] = src[i+j];
	    prefetch<PFHINT_L2EX>(&dst[offset[index]+L1_PREFETCH_ITEMS]);
	    offset[index]++;
	  }
	evictL2(&src[i]);
      }

      if (b<3) LockStepTaskScheduler::syncThreads(threadID,numThreads);

    }
  }


  void BVH4iBuilderMorton64Bit::build_main (const size_t threadIndex, const size_t threadCount)
  { 
    DBG(PING);
    TIMER(std::cout << std::endl);
    TIMER(double msec = 0.0);

    /* compute scene bounds */
    TIMER(msec = getSeconds());
    global_bounds.reset();
    LockStepTaskScheduler::dispatchTask( task_computeBounds, this, threadIndex, threadCount );
    TIMER(msec = getSeconds()-msec);    
    TIMER(std::cout << "task_computeBounds " << 1000. * msec << " ms" << std::endl << std::flush);
    TIMER(DBG_PRINT(global_bounds));



    /* compute morton codes */
    TIMER(msec = getSeconds());
    LockStepTaskScheduler::dispatchTask( task_computeMortonCodes, this, threadIndex, threadCount );   

    /* padding */
    MortonID64Bit* __restrict__ const dest = (MortonID64Bit*)morton;
    
    for (size_t i=numPrimitives; i<((numPrimitives+7)&(-8)); i++) {
      dest[i].code    = (size_t)-1; 
      dest[i].groupID = 0;
      dest[i].primID  = 0;
    }

    TIMER(msec = getSeconds()-msec);    
    TIMER(std::cout << "task_computeMortonCodes " << 1000. * msec << " ms" << std::endl << std::flush);

 
    /* sort morton codes */
    TIMER(msec = getSeconds());
    LockStepTaskScheduler::dispatchTask( task_radixsort, this, threadIndex, threadCount );

#if defined(DEBUG)
    for (size_t i=1; i<((numPrimitives+7)&(-8)); i++)
      assert(morton[i-1].code <= morton[i].code);

    for (size_t i=numPrimitives; i<((numPrimitives+7)&(-8)); i++) {
      assert(dest[i].code  == 0xffffffff); 
      assert(dest[i].groupID == 0);
      assert(dest[i].primID == 0);
    }

#endif	    

    TIMER(msec = getSeconds()-msec);    
    TIMER(std::cout << "task_radixsort " << 1000. * msec << " ms" << std::endl << std::flush);

    TIMER(msec = getSeconds());

    /* build and extract top-level tree */

    exit(0);
  }

  void BVH4iBuilderMorton64Bit::build_parallel_morton(size_t threadIndex, size_t threadCount, size_t taskIndex, size_t taskCount, TaskScheduler::Event* event) 
  {
    TIMER(double msec = 0.0);

    /* initialize thread state */
    initThreadState(threadIndex,threadCount);
    
    /* let all thread except for control thread wait for work */
    if (threadIndex != 0) {
      LockStepTaskScheduler::dispatchTaskMainLoop(threadIndex,threadCount);
      return;
    }

    /* start measurement */
    double t0 = 0.0f;

#if !defined(PROFILE)
    if (g_verbose >= 2) 
#endif
      t0 = getSeconds();

    /* performs build of tree */
    build_main(threadIndex,threadCount);

    FATAL("HERE");
    bvh->root = bvh->qbvh[0].lower[0].child; 
    bvh->bounds = global_bounds.geometry;

    /* end task */
    LockStepTaskScheduler::releaseThreads(threadCount);
    
    /* stop measurement */
#if !defined(PROFILE)
    if (g_verbose >= 2) 
#endif
      dt = getSeconds()-t0;

  }
}


