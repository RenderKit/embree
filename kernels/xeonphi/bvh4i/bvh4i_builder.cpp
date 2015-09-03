// ======================================================================== //
// Copyright 2009-2015 Intel Corporation                                    //
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

#include "bvh4i/bvh4i.h"
#include "bvh4i/bvh4i_builder.h"
#include "bvh4i/bvh4i_statistics.h"
#include "bvh4i/bvh4i_rotate.h"
#include "../../algorithms/parallel_partition.h"


#define THRESHOLD_FOR_SUBTREE_RECURSION         64
#define BUILD_RECORD_PARALLEL_SPLIT_THRESHOLD 1024
#define SINGLE_THREADED_BUILD_THRESHOLD        512

#define L1_PREFETCH_ITEMS 2
#define L2_PREFETCH_ITEMS 16

#define TIMER(x) 
#define DBG(x) 

//#define PROFILE
#define PROFILE_ITERATIONS 20

#define MEASURE_MEMORY_ALLOCATION_TIME 0

//#define CHECK_BUILD_RECORD_IN_DEBUG_MODE

namespace embree
{
  extern AtomicMutex mtx;

  static double dt = 0.0f;
  static double total_partition_time = 0.0;

  // =============================================================================================
  // =============================================================================================
  // =============================================================================================


  Builder* BVH4iBuilder::create (void* accel, void* geometry, size_t mode ) 
  { 
    DBG(PING);
    DBG(PRINT(mode));

    Builder* builder = nullptr;

    switch( mode )
      {

      case BVH4I_BUILDER_DEFAULT:
	builder = new BVH4iBuilder((BVH4i*)accel,geometry);
	break;

      case BVH4I_BUILDER_PRESPLITS:
	builder = new BVH4iBuilderPreSplits((BVH4i*)accel,geometry);
	break;

      case BVH4I_BUILDER_VIRTUAL_GEOMETRY:
	builder = new BVH4iBuilderVirtualGeometry((BVH4i*)accel,geometry);
	break;

      case BVH4I_BUILDER_MEMORY_CONSERVATIVE:
	builder = new BVH4iBuilderMemoryConservative((BVH4i*)accel,geometry);
	break;

      case BVH4I_BUILDER_SUBDIV_MESH:
	builder = new BVH4iBuilderSubdivMesh((BVH4i*)accel,geometry);
	break;

      default:
	THROW_RUNTIME_ERROR("ERROR: unknown BVH4iBuilder mode selected");	
      }
    return builder;
  }

  // =======================================================================================================
  // =======================================================================================================
  // =======================================================================================================


  
  BVH4iBuilder::BVH4iBuilder (BVH4i* bvh, void* geometry, const size_t bvh4iNodeSize)
    : ParallelBinnedSAHBuilder(geometry),
      bvh(bvh),       
      prims(nullptr), 
      node(nullptr), 
      accel(nullptr), 
      size_prims(0),
      num64BytesBlocksPerNode(bvh4iNodeSize / 64),
      leafItemThreshold(BVH4i::N)
  {
    DBG(PING);
  }

  
  BVH4iBuilder::~BVH4iBuilder()
  {
    DBG(PING);
    if (prims)  {
      assert(size_prims > 0);
      os_free(prims,size_prims);
      prims = nullptr;
    }    
  }

  
  size_t BVH4iBuilder::getNumPrimitives()
  {
    /* count total number of triangles */
    size_t primitives = 0;       
    for (size_t i=0;i<scene->size();i++)
      {
	if (unlikely(scene->get(i) == nullptr)) continue;
	if (unlikely((scene->get(i)->getType() != Geometry::TRIANGLE_MESH))) continue;
	if (unlikely(!scene->get(i)->isEnabled())) continue;
	const TriangleMesh* __restrict__ const mesh = scene->getTriangleMesh(i);
	if (unlikely(mesh->numTimeSteps != 1)) continue;
	primitives += mesh->size();
      }
    return primitives;	
  
  }

  
  void BVH4iBuilder::allocateMemoryPools(const size_t numPrims, 
					 const size_t numNodes,
					 const size_t sizeNodeInBytes,
					 const size_t sizeAccelInBytes,
					 const float  bvh4iNodePreallocFactor)
  {
#if MEASURE_MEMORY_ALLOCATION_TIME == 1
    double msec = 0.0;
    msec = getSeconds();
#endif


    const size_t additional_size = 16 * CACHELINE_SIZE;

    /* free previously allocated memory */

    if (prims)  {
      assert(size_prims > 0);
      os_free(prims,size_prims);
    }
    if (node  ) {
      assert(bvh->size_node > 0);
      os_free(node ,bvh->size_node);
    }
    if (accel ) {
      assert(bvh->size_accel > 0);
      os_free(accel,bvh->size_accel);
    }
      
    // === allocated memory for primrefs,nodes, and accel ===
    const size_t size_primrefs = numPrims * sizeof(PrimRef) + additional_size;
    const size_t size_node     = (double)(numNodes * bvh4iNodePreallocFactor * sizeNodeInBytes + additional_size) * scene->device->memory_preallocation_factor;
    const size_t size_accel    = numPrims * sizeAccelInBytes + additional_size;

    numAllocated64BytesBlocks = size_node / sizeof(float16);

    DBG(
	PRINT(numPrims);
	PRINT(numNodes);
	PRINT(sizeNodeInBytes);
	PRINT(sizeAccelInBytes);
	PRINT(numAllocated64BytesBlocks);
	PRINT(size_primrefs);
	PRINT(size_node);
	PRINT(size_accel);
	);

    prims = (PrimRef  *) os_malloc(size_primrefs); 
    node  = (int16    *) os_malloc(size_node);
    accel = (Triangle1*) os_malloc(size_accel);

    assert(prims  != 0);
    assert(node   != 0);
    assert(accel  != 0);

    // memset(prims,0,size_primrefs);
    // memset(node,0,size_node);
    // memset(accel,0,size_accel);

    memset((char*)accel + numPrims * sizeAccelInBytes,0,additional_size); // clear out as a 4-wide access is possible

    bvh->accel      = accel;
    bvh->qbvh       = (BVH4i::Node*)node;
    bvh->size_node  = size_node;
    bvh->size_accel = size_accel;

    size_prims = size_primrefs;    
    size_t total = size_primrefs+size_node+size_accel;
#if MEASURE_MEMORY_ALLOCATION_TIME == 1
    msec = getSeconds()-msec;    
    std::cout << "allocation time " << 1000. * msec << " ms for " << (float)(total) / 1024.0f / 1024.0f << " MB " << std::endl << std::flush;
#endif
  }

    
  void BVH4iBuilder::allocateData(const size_t threadCount, const size_t totalNumPrimitives)
  {
    DBG(PING);
    size_t numPrimitivesOld = numPrimitives;
    numPrimitives = totalNumPrimitives;
    if (numPrimitivesOld != numPrimitives)
      {
	const size_t numPrims = numPrimitives+BVH4i::N;
	const size_t minAllocNodes = (threadCount+1) * ALLOCATOR_NODE_BLOCK_SIZE; 
	const size_t numNodes = (size_t)((numPrims+3)/4) + minAllocNodes;
	allocateMemoryPools(numPrims,numNodes,sizeof(BVH4i::Node),sizeof(Triangle1));
      }
  }

  
  void BVH4iBuilder::printBuilderName()
  {
    std::cout << "building BVH4i with SAH builder (MIC) ... " << std::endl;        
  }


  
  void BVH4iBuilder::build(const size_t threadIndex, const size_t threadCount) 
  {
    if (threadIndex != 0) {
      THROW_RUNTIME_ERROR("threadIndex != 0");
    }
    const size_t totalNumPrimitives = getNumPrimitives();

    /* print builder name */
    if (unlikely(scene->device->verbosity(2))) {
      printBuilderName();

      DBG(
	  PRINT(totalNumPrimitives);
	  PRINT(threadIndex);
	  PRINT(threadCount);
	  );
    }

    if (likely(totalNumPrimitives == 0))
      {
	DBG(std::cout << "EMPTY SCENE BUILD" << std::endl);
	bvh->root = BVH4i::invalidNode;
	bvh->bounds = empty;
	bvh->qbvh = nullptr;
	bvh->accel = nullptr;
	return;
      }

    /* allocate BVH data */
    allocateData(threadCount ,totalNumPrimitives);
    if (likely(numPrimitives > SINGLE_THREADED_BUILD_THRESHOLD &&  threadCount > 1) )
    //if (likely(threadCount > 1) )

      {
	DBG(std::cout << "PARALLEL BUILD" << std::endl);

#if defined(PROFILE)

	std::cout << "STARTING PROFILE MODE" << std::endl << std::flush;
	std::cout << "primitives = " << totalNumPrimitives << std::endl;
	double dt_min = pos_inf;
	double dt_avg = 0.0f;
	double dt_max = neg_inf;
	size_t iterations = PROFILE_ITERATIONS;
	for (size_t i=0; i<iterations; i++) 
	  {
	    build_main(threadIndex,threadCount);
	    dt_min = min(dt_min,dt);
	    dt_avg = dt_avg + dt;
	    dt_max = max(dt_max,dt);
	  }
	dt_avg /= double(iterations);

	std::cout << "[DONE]" << std::endl;
	std::cout << "  min = " << 1000.0f*dt_min << "ms (" << totalNumPrimitives/dt_min*1E-6 << " Mtris/s)" << std::endl;
	std::cout << "  avg = " << 1000.0f*dt_avg << "ms (" << totalNumPrimitives/dt_avg*1E-6 << " Mtris/s)" << std::endl;
	std::cout << "  max = " << 1000.0f*dt_max << "ms (" << totalNumPrimitives/dt_max*1E-6 << " Mtris/s)" << std::endl;
	std::cout << "---" << std::endl << std::flush;

#else

	build_main(threadIndex,threadCount);

#endif
      }
    else
      {
	assert( numPrimitives > 0 );
	/* number of primitives is small, just use single threaded mode */
	DBG(std::cout << "SERIAL BUILD" << std::endl);
	build_main(0,1);
      }

    if (scene->device->verbosity(2)) {
      double perf = totalNumPrimitives/dt*1E-6;
      std::cout << "[DONE] " << 1000.0f*dt << "ms (" << perf << " Mtris/s), primitives " << numPrimitives << std::endl;
      std::cout << getStatistics();
    }

  }


  std::string BVH4iBuilder::getStatistics()
  {
    return BVH4iStatistics<BVH4i::Node>(bvh).str();
  }

  // =======================================================================================================
  // =======================================================================================================
  // =======================================================================================================

  void BVH4iBuilder::storeNodeDataUpdateParentPtrs(void *ptr,
						   BuildRecord *__restrict__ const br,
						   const size_t numChildren)
  {
    BVH4i::Node *__restrict__ n = (BVH4i::Node*)ptr;
    for (size_t i=0;i<numChildren;i++)
      br[i].parentPtr = &n->child(i);

    storeNode(n,br,numChildren);    
  }

  
  void BVH4iBuilder::computePrimRefsTriangles(const size_t threadID, const size_t numThreads) 
  {
    const size_t numGroups = scene->size();
    const size_t startID = ((threadID+0)*numPrimitives)/numThreads;
    const size_t endID   = ((threadID+1)*numPrimitives)/numThreads;
    
    PrimRef *__restrict__ const prims     = this->prims;

    // === find first group containing startID ===
    unsigned int g=0, numSkipped = 0;
    for (; g<numGroups; g++) {       
      if (unlikely(scene->get(g) == nullptr)) continue;
      if (unlikely(scene->get(g)->getType() != Geometry::TRIANGLE_MESH)) continue;
      const TriangleMesh* __restrict__ const mesh = scene->getTriangleMesh(g);
      if (unlikely(!mesh->isEnabled())) continue;
      if (unlikely(mesh->numTimeSteps != 1)) continue;

      const size_t numTriangles = mesh->size();
      if (numSkipped + numTriangles >= startID) break;
      numSkipped += numTriangles;
    }

    // === start with first group containing startID ===

    CentroidGeometryAABB bounds_cs;
    bounds_cs.reset();

    unsigned int num = 0;
    unsigned int currentID = startID;
    unsigned int offset = startID - numSkipped;

    __aligned(64) PrimRef local_prims[2];
    size_t numLocalPrims = 0;
    PrimRef *__restrict__ dest = &prims[currentID];

    for (; g<numGroups; g++) 
      {
	if (unlikely(scene->get(g) == nullptr)) continue;
	if (unlikely(scene->get(g)->getType() != Geometry::TRIANGLE_MESH)) continue;
	const TriangleMesh* __restrict__ const mesh = scene->getTriangleMesh(g);
	if (unlikely(!mesh->isEnabled())) continue;
	if (unlikely(mesh->numTimeSteps != 1)) continue;

	if (offset < mesh->size())
	  {
	    const char *__restrict cptr = (char*)&mesh->triangle(offset);
	    const size_t stride = mesh->getTriangleBufferStride();
	
	    for (unsigned int i=offset; i<mesh->size() && (currentID < endID); i++, currentID++,cptr+=stride)	 
	      { 			    
		const TriangleMesh::Triangle& tri = *(TriangleMesh::Triangle*)cptr;
		prefetch<PFHINT_L2>(cptr + L2_PREFETCH_ITEMS);
		prefetch<PFHINT_L1>(cptr + L1_PREFETCH_ITEMS);

		assert( tri.v[0] < mesh->numVertices() );
		assert( tri.v[1] < mesh->numVertices() );
		assert( tri.v[2] < mesh->numVertices() );

#if DEBUG
		for (size_t k=0;k<3;k++)
		  if (!(std::isfinite( mesh->vertex( tri.v[k] ).x) && std::isfinite( mesh->vertex( tri.v[k] ).y) && std::isfinite( mesh->vertex( tri.v[k] ).z)))
		    THROW_RUNTIME_ERROR("!isfinite in vertex for tri.v[k]");

#endif

		const Vec3f16 v = mesh->getTriangleVertices(tri);

		const float16 bmin  = min(min(v[0],v[1]),v[2]);
		const float16 bmax  = max(max(v[0],v[1]),v[2]);

		bounds_cs.extend(bmin,bmax);	

		store4f(&local_prims[numLocalPrims].lower,bmin);
		store4f(&local_prims[numLocalPrims].upper,bmax);	
		local_prims[numLocalPrims].lower.a = g;
		local_prims[numLocalPrims].upper.a = i;

		numLocalPrims++;
		if (unlikely(((size_t)dest % 64) != 0) && numLocalPrims == 1)
		  {
		    *dest = local_prims[0];
		    dest++;
		    numLocalPrims--;
		  }
		else
		  {
		    const float16 twoAABBs = load16f(local_prims);
		    if (numLocalPrims == 2)
		      {
			numLocalPrims = 0;
			store16f_ngo(dest,twoAABBs);
			dest+=2;
		      }
		  }	
	      }
	  }
	if (currentID == endID) break;
	offset = 0;
      }

    /* is there anything left in the local queue? */
    if (numLocalPrims % 2 != 0)
      *dest = local_prims[0];

    /* update global bounds */
    Centroid_Scene_AABB bounds ( bounds_cs );
    
    global_bounds.extend_atomic(bounds);    
  }

  
  void BVH4iBuilder::finalize(const size_t threadIndex, const size_t threadCount)
  {

  }

  __forceinline void computeAccelerationData(const unsigned int &geomID,
					     const unsigned int &primID,     
					     const Scene *__restrict__ const scene,
					     Triangle1 * __restrict__ const acc)
  {
    const TriangleMesh* __restrict__ const mesh = scene->getTriangleMesh(geomID);
    const TriangleMesh::Triangle & tri = mesh->triangle(primID);

    const int16 pID(primID);
    const int16 gID(geomID);

    const Vec3f16 v = mesh->getTriangleVertices<PFHINT_L1>(tri);

#if DEBUG
    for (size_t k=0;k<3;k++)
      if (!(std::isfinite( mesh->vertex( tri.v[k] ).x) && std::isfinite( mesh->vertex( tri.v[k] ).y) && std::isfinite( mesh->vertex( tri.v[k] ).z)))
	THROW_RUNTIME_ERROR("!isfinite in vertex for tri.v[k]");
#endif

    const float16 tri_accel = initTriangle1(v[0],v[1],v[2],gID,pID,int16(mesh->mask));
    store16f_ngo(acc,tri_accel);
  }

  void BVH4iBuilder::createTriangle1AccelRange(const size_t startID, const size_t endID)
  {
    Triangle1    * __restrict__  acc  = accel + startID;
    const PrimRef* __restrict__  bptr = prims + startID;

    for (size_t j=startID; j<endID; j++, bptr++, acc++)
      {
	prefetch<PFHINT_NT>(bptr + L1_PREFETCH_ITEMS);
	prefetch<PFHINT_L2>(bptr + L2_PREFETCH_ITEMS);
	assert(bptr->geomID() < scene->size() );
	assert(bptr->primID() < scene->get( bptr->geomID() )->numPrimitives );

	computeAccelerationData(bptr->geomID(),bptr->primID(),scene,acc);
      }
  }

  
  void BVH4iBuilder::createAccel(const size_t threadIndex, const size_t threadCount)
  {
    //scene->lockstep_scheduler.dispatchTask( task_createTriangle1Accel, this, threadIndex, threadCount );   
    parallel_for(size_t(0), numPrimitives, [&](const range<size_t>& r) { createTriangle1AccelRange(r.begin(),r.end()); });
  }

  
  void BVH4iBuilder::createTriangle1Accel(const size_t threadID, const size_t numThreads)
  {
    const size_t startID = (threadID+0)*numPrimitives/numThreads;
    const size_t endID   = (threadID+1)*numPrimitives/numThreads;   
    createTriangle1AccelRange(startID,endID);    
  }

  
  void BVH4iBuilder::buildSubTree(BuildRecord& current, 
				  NodeAllocator& alloc, 
				  const size_t mode,
				  const size_t threadID, 
				  const size_t numThreads)
  {
    recurseSAH(current,alloc,mode,threadID,numThreads);
  }



  
  void BVH4iBuilder::parallelBinningGlobal(const size_t threadID, const size_t numThreads)
  {
    BuildRecord &current = global_sharedData.rec;

    const unsigned int items = current.items();
    const unsigned int startID = current.begin + ((threadID+0)*items/numThreads);
    const unsigned int endID   = current.begin + ((threadID+1)*items/numThreads);

    const BinMapping mapping(current.bounds);
    fastbin<PrimRef>(prims,startID,endID,mapping,global_bin16[threadID]);    

    scene->lockstep_scheduler.syncThreadsWithReduction( threadID, numThreads, reduceBinsParallel, global_bin16 );
    
    if (threadID == 0)
      {
	global_sharedData.split = global_bin16[0].bestSplit(current,mapping.getValidDimMask());
      }
  }

  

  // =======================================================================================================
  // =======================================================================================================
  // =======================================================================================================

  
  bool BVH4iBuilder::splitSequential(BuildRecord& current, BuildRecord& leftChild, BuildRecord& rightChild)
  {
#if defined(DEBUG)
    checkBuildRecord(current);
#endif

    /* mark as leaf if leaf threshold reached */
    if (current.items() <= leafItemThreshold) {
      current.createLeaf();
      return false;
    }

    const BinMapping mapping(current.bounds);

    float16 leftArea[3];
    float16 rightArea[3];
    int16 leftNum[3];

    fastbin<PrimRef>(prims,current.begin,current.end,mapping,leftArea,rightArea,leftNum);

    const Split split = getBestSplit(current,leftArea,rightArea,leftNum,mapping.getValidDimMask());

    if (unlikely(split.invalid())) 
      split_fallback(prims,current,leftChild,rightChild);
    else 
      {
	leftChild.bounds.reset();
	rightChild.bounds.reset();	

	/* partitioning of items */
	const BinPartitionMapping mapping(split,current.bounds);
	const unsigned int mid = partitionPrimitives<PrimRef,false>(&prims[current.begin],
								    current.size(), 
								    mapping, 
								    leftChild.bounds, 
								    rightChild.bounds);
	assert(area(leftChild.bounds.geometry) >= 0.0f);
	assert(current.begin + mid == current.begin + split.numLeft);

	if (unlikely(current.begin + mid == current.begin || current.begin + mid == current.end)) 
	  {
	    std::cout << "WARNING: mid == current.begin || mid == current.end " << std::endl;
	    split_fallback(prims,current,leftChild,rightChild);	    
	  }
	else
	  {
	    const unsigned int current_mid = current.begin + split.numLeft;
	    leftChild.init(current.begin,current_mid);
	    rightChild.init(current_mid,current.end);
	  }

      }


#if defined(DEBUG)
    checkBuildRecord(leftChild);
    checkBuildRecord(rightChild);
#endif

    if (leftChild.items()  <= leafItemThreshold) leftChild.createLeaf();
    if (rightChild.items() <= leafItemThreshold) rightChild.createLeaf();	
    return true;
  }

  
  bool BVH4iBuilder::splitParallelGlobal( BuildRecord &current,
					  BuildRecord &leftChild,
					  BuildRecord &rightChild,
					  const size_t threadID,
					  const size_t numThreads)
  {
    const unsigned int items = current.end - current.begin;
    assert(items >= BUILD_RECORD_PARALLEL_SPLIT_THRESHOLD);

#if defined(DEBUG)
    checkBuildRecord(current);
#endif
  
    /* mark as leaf if leaf threshold reached */
    if (items <= leafItemThreshold) {
      current.createLeaf();
      return false;
    }

    global_sharedData.rec = current;
    global_sharedData.split.reset();
    global_sharedData.left.reset();
    global_sharedData.right.reset();
     
    scene->lockstep_scheduler.dispatchTask( task_parallelBinningGlobal, this, threadID, numThreads );

    if (unlikely(global_sharedData.split.invalid())) 
      split_fallback(prims,current,leftChild,rightChild);
    else
      {
	// TIMER(double msec = getSeconds());    

	global_sharedData.left.reset();
	global_sharedData.right.reset();

	const BinPartitionMapping mapping(global_sharedData.split,global_sharedData.rec.bounds);

	auto part = [&] (PrimRef* const t_array,
			 const size_t size) 
	  {          
	    return partitionPrimitives<PrimRef,true>(t_array,size,mapping,global_sharedData.left,global_sharedData.right);
	  };

	size_t mid_parallel = parallel_in_place_partitioning_static<PrimRef>(&prims[current.begin],
									     current.size(),
									     part,
									     scene->lockstep_scheduler);
	
	assert( 
	       parallel_in_place_partitioning_static_verify<PrimRef>(&prims[current.begin],
								     current.size(),
								     mid_parallel,
								     [&] (const PrimRef &ref) { 
								       const Vec2f16 b = ref.getBounds();
								       return any(mapping.lt_split(b.x,b.y));
								     }
								     ));

	assert(mid_parallel == global_sharedData.split.numLeft);
	const unsigned int mid = current.begin + mid_parallel;

	// TIMER(
	//       msec = getSeconds()-msec;    
	//       total_partition_time += msec;
	//       std::cout << "partition time " << 1000. * msec << " total " << 1000. * total_partition_time << " items = " << current.size() << std::endl;
	//       );
	
	if (unlikely(current.begin == mid || mid == current.end)) 
	  {
	    std::cout << "WARNING: mid == current.begin || mid == current.end " << std::endl;
	    split_fallback(prims,current,leftChild,rightChild);	    
	  }
	else
	  {
	    leftChild.init(global_sharedData.left,current.begin,mid);
	    rightChild.init(global_sharedData.right,mid,current.end);
	  }	 
      }

#if defined(DEBUG)
    checkBuildRecord(leftChild);
    checkBuildRecord(rightChild);
#endif
     
    if (leftChild.items()  <= leafItemThreshold) leftChild.createLeaf();
    if (rightChild.items() <= leafItemThreshold) rightChild.createLeaf();
    return true;
  }


  
  bool BVH4iBuilder::splitParallelLocal(BuildRecord &current,
					BuildRecord &leftChild,
					BuildRecord &rightChild,
					const size_t threadID)
  {
    const unsigned int items    = current.end - current.begin;
    const size_t globalCoreID   = threadID / 4;
    const size_t localThreadID  = threadID % 4;
    const size_t globalThreadID = threadID;
    
    assert(items >= THRESHOLD_FOR_SUBTREE_RECURSION);

#if defined(DEBUG)
    checkBuildRecord(current);
#endif
  
    /* mark as leaf if leaf threshold reached */
    if (items <= leafItemThreshold) {
      current.createLeaf();
      return false;
    }

    SharedBinningPartitionData &sd = local_sharedData[globalCoreID]; 

    sd.rec = current;
    sd.split.reset();
    sd.left.reset();
    sd.right.reset();

    localTaskScheduler[globalCoreID].dispatchTask( task_parallelBinningLocal, this, localThreadID, globalThreadID );

    if (unlikely(sd.split.invalid())) 
      split_fallback(prims,current,leftChild,rightChild);
    else
      {

	sd.left.reset();
	sd.right.reset();

	const BinPartitionMapping mapping(sd.split,sd.rec.bounds);

	auto part_local = [&] (PrimRef* const t_array,
			 const size_t size) 
	  {                                               
	    return partitionPrimitives<PrimRef,true>(t_array,size,mapping,sd.left,sd.right);
	  };
	size_t mid_parallel = parallel_in_place_partitioning_static<PrimRef>(&prims[sd.rec.begin],
									     sd.rec.size(),
									     part_local,
									     localTaskScheduler[globalCoreID]
									     );
	assert( 
	       parallel_in_place_partitioning_static_verify<PrimRef>(&prims[sd.rec.begin],
								     sd.rec.size(),
								     mid_parallel,
								     [&] (const PrimRef &ref) { 
								       const Vec2f16 b = ref.getBounds();
								       return any(mapping.lt_split(b.x,b.y));
								     }
								     )
		);
	assert(mid_parallel == sd.split.numLeft);
	const unsigned int mid = current.begin + mid_parallel;
	

	if (unlikely(mid == current.begin || mid == current.end)) 
	  {
	    std::cout << "WARNING: mid == current.begin || mid == current.end " << std::endl;
	    split_fallback(prims,current,leftChild,rightChild);	    
	  }
	else
	  {
	    leftChild.init(sd.left,current.begin,mid);
	    rightChild.init(sd.right,mid,current.end);
	  }
	 
      }

#if defined(DEBUG)
    checkBuildRecord(leftChild);
    checkBuildRecord(rightChild);
#endif
     
    if (leftChild.items()  <= leafItemThreshold) leftChild.createLeaf();
    if (rightChild.items() <= leafItemThreshold) rightChild.createLeaf();
    return true;
  }

  
  __forceinline bool BVH4iBuilder::split(BuildRecord& current, BuildRecord& left, BuildRecord& right, const size_t mode, const size_t threadID, const size_t numThreads)
  {
    if (unlikely(mode == BUILD_TOP_LEVEL))
      {
	if (current.items() >= BUILD_RECORD_PARALLEL_SPLIT_THRESHOLD)
	  return splitParallelGlobal(current,left,right,threadID,numThreads);
	else
	  {
	    DBG(std::cout << "WARNING in top-level build: too few items for parallel split " << current.items() << std::endl << std::flush);
	    return splitSequential(current,left,right);
	  }
      }
    else if (unlikely(mode == FILL_LOCAL_QUEUES))
      {
	if (current.items() >= THRESHOLD_FOR_SUBTREE_RECURSION)
	  return splitParallelLocal(current,left,right,threadID);
	else
	  {
	    DBG(std::cout << "WARNING in fill_local_queues build: too few items for parallel split " << current.items() << std::endl << std::flush);
	    return splitSequential(current,left,right);
	  }
	
      }
    else
      return splitSequential(current,left,right);
  }

  
  bool BVH4iBuilder::split_fallback(PrimRef * __restrict__ const primref, BuildRecord& current, BuildRecord& leftChild, BuildRecord& rightChild)
  {
    assert( BVH4i::N == 4 );
    unsigned int blocks4 = (current.items()+BVH4i::N-1)/BVH4i::N;
    unsigned int center = current.begin + (blocks4/2)*BVH4i::N; // (current.begin + current.end)/2;

    if (unlikely(current.items() <= BVH4i::N))
      {
	center = current.begin + 1;
      }
    assert(center != current.begin);
    assert(center != current.end);
    
    Centroid_Scene_AABB left; left.reset();
    for (size_t i=current.begin; i<center; i++)
      left.extend(primref[i].bounds());
    leftChild.init(left,current.begin,center);
    assert(leftChild.items() > 0);
    
    Centroid_Scene_AABB right; right.reset();
    for (size_t i=center; i<current.end; i++)
      right.extend(primref[i].bounds());	
    rightChild.init(right,center,current.end);
    assert(rightChild.items() > 0);
    
    return true;
  }

  
  // =======================================================================================================
  // =======================================================================================================
  // =======================================================================================================

    
  __forceinline void BVH4iBuilder::createLeaf(BuildRecord& current, NodeAllocator& alloc,const size_t threadIndex, const size_t threadCount)
  {
#if defined(DEBUG)
    if (current.depth > BVH4i::maxBuildDepthLeaf) 
      THROW_RUNTIME_ERROR("ERROR: depth limit reached");
#endif
    
    /* create leaf */
    if (current.items() <= leafItemThreshold) {
      createBVH4iLeaf(*(BVH4i::NodeRef*)current.parentPtr,current.begin,current.items());

#if defined(DEBUG)
      checkLeafNode(*(BVH4i::NodeRef*)current.parentPtr,current.bounds.geometry);      
#endif
      return;
    }

    /* first split level */
    BuildRecord record0, record1;
    split_fallback(prims,current,record0,record1);

    /* second split level */
    BuildRecord children[4];
    split_fallback(prims,record0,children[0],children[1]);
    split_fallback(prims,record1,children[2],children[3]);


    /* allocate next four nodes */
    size_t numChildren = BVH4i::N;
    if (current.items() <= 4 )
      numChildren = current.items();

    for (size_t i=0; i<numChildren; i++) 
      children[i].depth = current.depth+1;

    const size_t currentIndex = alloc.get(num64BytesBlocksPerNode);

    createBVH4iNode<2>(*(BVH4i::NodeRef*)current.parentPtr,currentIndex);

    storeNodeDataUpdateParentPtrs(&node[currentIndex],children,numChildren);

    /* recursivly create leaves */
    for (size_t i=0; i<numChildren; i++) 
      createLeaf(children[i],alloc,threadIndex,threadCount);
  }  

  
  __forceinline void BVH4iBuilder::recurse(BuildRecord& current, NodeAllocator& alloc,const size_t mode, const size_t threadID, const size_t numThreads)
  {
    if (mode == BUILD_TOP_LEVEL) {
      global_workStack.push_nolock(current);
    }
    else if (current.items() > THRESHOLD_FOR_SUBTREE_RECURSION || mode == FILL_LOCAL_QUEUES) {
      const size_t coreID = threadID/4;
      if (!local_workStack[coreID].push(current))
        recurseSAH(current,alloc,RECURSE,threadID,numThreads);
    }
    else
      recurseSAH(current,alloc,RECURSE,threadID,numThreads);
  }
  
  
  void BVH4iBuilder::recurseSAH(BuildRecord& current, NodeAllocator& alloc,const size_t mode, const size_t threadID, const size_t numThreads)
  {
#if defined(DEBUG)
    checkBuildRecord(current);
#endif

    __aligned(64) BuildRecord children[BVH4i::N];

    /* create leaf node */
    if (current.depth >= BVH4i::maxBuildDepth || current.isLeaf()) {
      createLeaf(current,alloc,threadID,numThreads);
      return;
    }

    /* fill all 4 children by always splitting the one with the largest surface area */
    unsigned int numChildren = 1;
    children[0] = current;

    do {

      /* find best child with largest bounding box area */
      int bestChild = -1;
      float bestArea = neg_inf;
      for (unsigned int i=0; i<numChildren; i++)
	{
	  /* ignore leaves as they cannot get split */
	  if (children[i].isLeaf())
	    continue;
        
	  /* remember child with largest area */
	  if (children[i].sceneArea() > bestArea) { 
	    bestArea = children[i].sceneArea();
	    bestChild = i;
	  }
	}
      if (bestChild == -1) break;

      /*! split best child into left and right child */
      __aligned(64) BuildRecord left, right;
      if (!split(children[bestChild],left,right,mode,threadID,numThreads)) 
        continue;
      
      /* add new children left and right */
      left.depth = right.depth = current.depth+1;
      children[bestChild] = children[numChildren-1];
      children[numChildren-1] = left;
      children[numChildren+0] = right;
      numChildren++;
      
    } while (numChildren < BVH4i::N);

    /* create leaf node if no split is possible */
    if (numChildren == 1) {
      createLeaf(current,alloc,threadID,numThreads);
      return;
    }

    /* allocate next four nodes */
    const size_t currentIndex = alloc.get(num64BytesBlocksPerNode);

    /* init used/unused nodes */

    createBVH4iNode<2>(*(BVH4i::NodeRef*)current.parentPtr,currentIndex);

    storeNodeDataUpdateParentPtrs(&node[currentIndex],children,numChildren);

    /* recurse into each child */

    for (unsigned int i=0; i<numChildren; i++) 
	recurse(children[i],alloc,mode,threadID,numThreads);

  }

  
  void BVH4iBuilder::checkLeafNode(const BVH4i::NodeRef &ref, const BBox3fa &bounds)
  {
    if (!ref.isLeaf())
      THROW_RUNTIME_ERROR("no leaf");

    unsigned int accel_entries = ref.items();
    unsigned int accel_offset  = ref.offsetIndex();

    BBox3fa leaf_prim_bounds = empty;
    for (size_t i=0;i<accel_entries;i++)
      {
	leaf_prim_bounds.extend( prims[ accel_offset + i ].lower );
	leaf_prim_bounds.extend( prims[ accel_offset + i ].upper );
      }

    if (!(subset(leaf_prim_bounds,bounds))) 
      {
	PRINT(bounds);
	PRINT(leaf_prim_bounds);
	THROW_RUNTIME_ERROR("checkLeafNode");
      }

  }


  
  void BVH4iBuilder::checkBuildRecord(const BuildRecord &current)
  {
#if defined(CHECK_BUILD_RECORD_IN_DEBUG_MODE)
    BBox3fa check_box;
    BBox3fa box;
    check_box = empty;
    box = *(BBox3fa*)&current.bounds.geometry;

    BBox3fa *aabb = (BBox3fa*)prims;

    for (unsigned int i=current.begin;i<current.end;i++) 
      {
	check_box.extend(aabb[i]);
	if (!subset(aabb[i],box))
	  {
	    PRINT(current);
	    PRINT(i);
	    PRINT(prims[i]);
	    THROW_RUNTIME_ERROR("check build record => subset");
	  }
      }

    //if (enablePreSplits) return;
    if (!(subset(check_box,box) && subset(box,check_box))) 
      {
	PRINT(current);
	PRINT(check_box);
	PRINT(box);
	THROW_RUNTIME_ERROR("check build record => subset(check_box,box) && subset(box,check_box)");
      }
#endif
  }

  
  void BVH4iBuilder::parallelBinningLocal(const size_t localThreadID,const size_t globalThreadID)
  {
    const size_t globalCoreID = globalThreadID/4;
    BuildRecord &current = local_sharedData[globalCoreID].rec;

    const unsigned int items   = current.items();
    const unsigned int startID = current.begin + ((localThreadID+0)*items/4);
    const unsigned int endID   = current.begin + ((localThreadID+1)*items/4);
    
    const BinMapping mapping(current.bounds);
    fastbin<PrimRef>(prims,startID,endID,mapping,global_bin16[globalThreadID]);    

    localTaskScheduler[globalCoreID].syncThreads(localThreadID);

    if (localThreadID == 0)
      {
	Bin16 &bin16 = global_bin16[globalThreadID];

	for (size_t i=1;i<4;i++)
	  bin16.merge(global_bin16[globalThreadID+i]);

	local_sharedData[globalCoreID].split = bin16.bestSplit(current,mapping.getValidDimMask());
      }

  }

  

  
  void BVH4iBuilder::computePrimRefs(const size_t threadIndex, const size_t threadCount)
  {    
#if 1
    scene->lockstep_scheduler.dispatchTask( task_computePrimRefsTriangles, this, threadIndex, threadCount );
#else
    Scene::Iterator<TriangleMesh> iter;
    ParallelForForPrefixSumState<PrimInfo> pstate;
#endif    

  }


  // =======================================================================================================
  // =======================================================================================================
  // =======================================================================================================

  void BVH4iBuilder::build_main(size_t threadIndex, size_t threadCount) 
  {

    TIMER(double msec = 0.0; std::cout << std::endl; total_partition_time = 0.0);

    /* start measurement */
    double t0 = 0.0f;
#if !defined(PROFILE)
    if (scene->device->verbosity(2)) 
#endif
      t0 = getSeconds();

    TIMER(msec = getSeconds());
    /* calculate list of primrefs */
    global_bounds.reset();
    computePrimRefs(threadIndex,threadCount);
    TIMER(msec = getSeconds()-msec);    
    TIMER(std::cout << "task_computePrimRefs " << 1000. * msec << " ms" << std::endl << std::flush);
    TIMER(msec = getSeconds());


    /* initialize atomic node counter */
    atomicID.reset(0);

    /* update BVH4i */
    bvh->bounds = global_bounds.geometry;


#if DEBUG
    assert( std::isfinite(bvh->bounds.lower.x) );
    assert( std::isfinite(bvh->bounds.lower.y) );
    assert( std::isfinite(bvh->bounds.lower.z) );

    assert( std::isfinite(bvh->bounds.upper.x) );
    assert( std::isfinite(bvh->bounds.upper.y) );
    assert( std::isfinite(bvh->bounds.upper.z) );

#endif


    /* create initial build record */
    BuildRecord br;
    br.init(global_bounds,0,numPrimitives);
    br.depth = 1;
    br.parentPtr = &bvh->root;
        
    /* push initial build record to global work stack */
    global_workStack.reset();
    global_workStack.push_nolock(br);    

    /* work in multithreaded toplevel mode until sufficient subtasks got generated */    
    NodeAllocator alloc(atomicID,numAllocated64BytesBlocks);

    const size_t coreCount = (threadCount+3)/4;
    while (global_workStack.size() < coreCount &&
	   global_workStack.size()+BVH4i::N <= SIZE_GLOBAL_WORK_STACK) 
      {
	BuildRecord br;
	if (!global_workStack.pop_nolock_largest(br)) break;

	DBG(PRINT(br));
	recurseSAH(br,alloc,BUILD_TOP_LEVEL,threadIndex,threadCount);      
      }

    TIMER(msec = getSeconds()-msec);    
    TIMER(std::cout << "build_top_level " << 1000. * msec << " ms" << std::endl << std::flush);

    /* fill per core work queues */    
    TIMER(msec = getSeconds());    
    scene->lockstep_scheduler.dispatchTask(task_fillLocalWorkQueues, this, threadIndex, threadCount );
    TIMER(msec = getSeconds()-msec);    
    TIMER(std::cout << "task_fillLocalWorkQueues " << 1000. * msec << " ms" << std::endl << std::flush);

    /* now process all created subtasks on multiple threads */    
    TIMER(msec = getSeconds());    
    scene->lockstep_scheduler.dispatchTask(task_buildSubTrees, this, threadIndex, threadCount );
    numNodes = atomicID;
    TIMER(msec = getSeconds()-msec);    
    TIMER(std::cout << "task_buildSubTrees " << 1000. * msec << " ms" << std::endl << std::flush);

    /* create triangle acceleration structure */
    TIMER(msec = getSeconds());        

    createAccel(threadIndex, threadCount );
    TIMER(msec = getSeconds()-msec);    
    TIMER(std::cout << "task_createAccel " << 1000. * msec << " ms" << std::endl << std::flush);
    
    /* finalize build */
    TIMER(msec = getSeconds());     
    finalize(threadIndex, threadCount );
    TIMER(msec = getSeconds()-msec);    
    TIMER(std::cout << "task_finalize " << 1000. * msec << " ms" << std::endl << std::flush);

#if DEBUG
    for (size_t i=0;i<threadCount/4;i++)
      if (!local_workStack[i].isEmpty())
	{
	  PRINT(i);
	  THROW_RUNTIME_ERROR("local_workStack[i].size() != 0");
	}
#endif    

    /* stop measurement */
#if !defined(PROFILE)
    if (scene->device->verbosity(2)) 
#endif
      dt = getSeconds()-t0;
  }

};
