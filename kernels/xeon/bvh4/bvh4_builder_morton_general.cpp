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

#include "bvh4.h"
#include "bvh4_builder_morton_general.h"
#include "bvh4_statistics.h"

#include "geometry/triangle1.h"
#include "geometry/triangle4.h"
#include "geometry/triangle8.h"
#include "geometry/triangle1v.h"
#include "geometry/triangle4v.h"
#include "geometry/triangle4i.h"

#define DBG(x) 
#define PROFILE

namespace embree 
{
  namespace isa
  {
    static double dt = 0.0f;

    BVH4BuilderMortonGeneral::BVH4BuilderMortonGeneral (BVH4* bvh, Scene* scene, TriangleMesh* mesh, size_t listMode, size_t logBlockSize, bool needVertices, size_t primBytes, const size_t minLeafSize, const size_t maxLeafSize)
      : bvh(bvh), state(nullptr), scheduler(&scene->lockstep_scheduler), scene(scene), mesh(mesh), listMode(listMode), logBlockSize(logBlockSize), needVertices(needVertices), primBytes(primBytes), minLeafSize(minLeafSize), maxLeafSize(maxLeafSize),
	topLevelItemThreshold(0), encodeShift(0), encodeMask(-1), morton(NULL), bytesMorton(0), numGroups(0), numPrimitives(0), numAllocatedPrimitives(0), numAllocatedNodes(0)
    {
      needAllThreads = true;
      //if (mesh) needAllThreads = mesh->numTriangles > 50000;
    }
    
    BVH4Triangle4BuilderMortonGeneral::BVH4Triangle4BuilderMortonGeneral (BVH4* bvh, Scene* scene, size_t listMode)
      : BVH4BuilderMortonGeneral(bvh,scene,NULL,listMode,2,false,sizeof(Triangle4),4,inf) {}
        
    BVH4BuilderMortonGeneral::~BVH4BuilderMortonGeneral () 
    {
      if (morton) os_free(morton,bytesMorton);
      bvh->alloc.shrink();
    }
    
    void BVH4BuilderMortonGeneral::build(size_t threadIndex, size_t threadCount) 
    {
      if (g_verbose >= 2)
        std::cout << "building BVH4<" << bvh->primTy.name << "> with " << TOSTRING(isa) << "::BVH4BuilderMortonGeneral ... " << std::flush;
      
      /* calculate size of scene */
      size_t numPrimitivesOld = numPrimitives;
      if (mesh) numPrimitives = mesh->numTriangles;
      else      numPrimitives = scene->numTriangles;
      
      bvh->numPrimitives = numPrimitives;
      numGroups = scene->size();
      size_t maxPrimsPerGroup = 0;
      if (mesh) 
        maxPrimsPerGroup = numPrimitives;
      else {
	for (size_t i=0; i<numGroups; i++) // FIXME: encoding unsafe
        {
          Geometry* geom = scene->get(i);
          if (!geom || geom->type != TRIANGLE_MESH) continue;
          TriangleMesh* mesh = (TriangleMesh*) geom;
	  if (mesh->numTimeSteps != 1) continue;
	  maxPrimsPerGroup = max(maxPrimsPerGroup,mesh->numTriangles);
        }

        /* calculate groupID, primID encoding */
        encodeShift = __bsr(maxPrimsPerGroup) + 1;
        encodeMask = ((size_t)1 << encodeShift)-1;
        size_t maxGroups = ((size_t)1 << (31-encodeShift))-1;
      
        if (maxPrimsPerGroup > encodeMask || numGroups > maxGroups) 
          THROW_RUNTIME_ERROR("encoding error in morton builder");
      }
      
      /* preallocate arrays */
      if (numPrimitivesOld != numPrimitives)
      {
	bvh->init(sizeof(BVH4::Node),numPrimitives,threadCount);
        if (morton) os_free(morton,bytesMorton);
	bytesMorton = ((numPrimitives+7)&(-8)) * sizeof(MortonID32Bit);
        morton = (MortonID32Bit* ) os_malloc(bytesMorton); memset(morton,0,bytesMorton);
      }
         
#if defined(PROFILE)
      
      double dt_min = pos_inf;
      double dt_avg = 0.0f;
      double dt_max = neg_inf;
      for (size_t i=0; i<20; i++) 
      {
        double t0 = getSeconds();
#endif

        state.reset(new MortonBuilderState);
	size_t numActiveThreads = threadCount;
	//size_t numActiveThreads = min(threadCount,getNumberOfCores());
	build_parallel_morton(threadIndex,numActiveThreads,0,1);
        state.reset(NULL);


#if defined(PROFILE)
        double dt = getSeconds()-t0;
        dt_min = min(dt_min,dt);
        if (i != 0) dt_avg = dt_avg + dt;
        dt_max = max(dt_max,dt);
      }
      dt_avg /= double(19);
      
      std::cout << "[DONE]" << std::endl;
      std::cout << "  min = " << 1000.0f*dt_min << "ms (" << numPrimitives/dt_min*1E-6 << " Mtris/s)" << std::endl;
      std::cout << "  avg = " << 1000.0f*dt_avg << "ms (" << numPrimitives/dt_avg*1E-6 << " Mtris/s)" << std::endl;
      std::cout << "  max = " << 1000.0f*dt_max << "ms (" << numPrimitives/dt_max*1E-6 << " Mtris/s)" << std::endl;
      std::cout << BVH4Statistics(bvh).str();
#endif

      if (g_verbose >= 2) {
        std::cout << "[DONE] " << 1000.0f*dt << "ms (" << numPrimitives/dt*1E-6 << " Mtris/s)" << std::endl;
        std::cout << "  bvh4::alloc : "; bvh->alloc.print_statistics();
	std::cout << "  bvh4::alloc2: "; bvh->alloc2.print_statistics();
        std::cout << BVH4Statistics(bvh).str();
      }

      /* benchmark mode */
      if (g_benchmark) {
	BVH4Statistics stat(bvh);
	std::cout << "BENCHMARK_BUILD " << dt << " " << double(numPrimitives)/dt << " " << stat.sah() << " " << stat.bytesUsed() << std::endl;
      }
    }
    
    // =======================================================================================================
    // =======================================================================================================
    // =======================================================================================================
    
    void BVH4BuilderMortonGeneral::initThreadState(const size_t threadID, const size_t numThreads)
    {
      const size_t startID = (threadID+0)*numPrimitives/numThreads;
      const size_t endID   = (threadID+1)*numPrimitives/numThreads;
      
      if (mesh) 
      {
        /* store start group and offset */
        state->startGroup[threadID] = mesh->id;
        state->startGroupOffset[threadID] = startID;
      }
      else
      {
        /* find first group containing startID */
        size_t group = 0, skipped = 0;
        for (; group<numGroups; group++) 
        {       
          Geometry* geom = scene->get(group);
          if (!geom || geom->type != TRIANGLE_MESH) continue;
          TriangleMesh* mesh = (TriangleMesh*) geom;
          if (mesh->numTimeSteps != 1) continue;
          const size_t numTriangles = mesh->numTriangles;
          if (skipped + numTriangles > startID) break; 
         skipped += numTriangles;
        }
        
        /* store start group and offset */
        state->startGroup[threadID] = group;
        state->startGroupOffset[threadID] = startID - skipped;
      }
    }

    CentGeomBBox3fa BVH4BuilderMortonGeneral::computeBounds()
    {
      CentGeomBBox3fa bounds; bounds.reset();

      if (mesh) 
      {
        for (size_t i=0; i<mesh->numTriangles; i++) {
	  const BBox3fa b = mesh->bounds(i);
	  if (!inFloatRange(b)) continue;
          bounds.extend(b);
	}
      }
      else
      {
        for (size_t group=0; group<numGroups; group++) 
        {       
          Geometry* geom = scene->get(group);
          if (!geom || geom->type != TRIANGLE_MESH) continue;
          TriangleMesh* mesh = (TriangleMesh*) geom;
          if (mesh->numTimeSteps != 1) continue;
          for (size_t i=0; i<mesh->numTriangles; i++) {
	    const BBox3fa b = mesh->bounds(i);
	    if (!inFloatRange(b)) continue;
	    bounds.extend(b);
	  }
        }
      }
      return bounds;
    }

    void BVH4BuilderMortonGeneral::computeBounds(const size_t threadID, const size_t numThreads)
    {
      initThreadState(threadID,numThreads);

      const ssize_t start = (threadID+0)*numPrimitives/numThreads;
      const ssize_t end   = (threadID+1)*numPrimitives/numThreads;
      
      CentGeomBBox3fa bounds; bounds.reset();
      
      if (mesh) 
      {
        for (size_t i=start; i<end; i++) {
	  const BBox3fa b = mesh->bounds(i);
	  if (!inFloatRange(b)) continue;
          bounds.extend(b);
	}
      }
      else
      {
	for (ssize_t cur=0, i=0; i<ssize_t(scene->size()); i++) 
	{
	  TriangleMesh* geom = (TriangleMesh*) scene->get(i);
          if (geom == NULL) continue;
	  if (geom->type != TRIANGLE_MESH || geom->numTimeSteps != 1 || !geom->isEnabled()) continue;
	  ssize_t gstart = 0;
	  ssize_t gend = geom->numTriangles;
	  ssize_t s = max(start-cur,gstart);
	  ssize_t e = min(end  -cur,gend  );
	  for (ssize_t j=s; j<e; j++) {
	    const BBox3fa b = geom->bounds(j);
	    if (!inFloatRange(b)) continue;
	    bounds.extend(b);
	  }
	  cur += geom->numTriangles;
	  if (cur >= end) break;  
        }
      }
      global_bounds.extend_atomic(bounds);    
    }

    void BVH4BuilderMortonGeneral::computeMortonCodes(const size_t startID, const size_t endID, size_t& destID,
                                               const size_t startGroup, const size_t startOffset, 
                                               MortonID32Bit* __restrict__ const dest)
    {
      /* compute mapping from world space into 3D grid */
      const ssef base  = (ssef)global_bounds.centBounds.lower;
      const ssef diag  = (ssef)global_bounds.centBounds.upper - (ssef)global_bounds.centBounds.lower;
      const ssef scale = select(diag > ssef(1E-19f), rcp(diag) * ssef(LATTICE_SIZE_PER_DIM * 0.99f),ssef(0.0f));
      
      size_t currentID = destID;
      size_t offset = startOffset;
      
      /* use SSE to calculate morton codes */
      size_t slots = 0;
      ssei ax = 0, ay = 0, az = 0, ai = 0;
      
      for (size_t group = startGroup; group<numGroups; group++) 
      {       
        Geometry* geom = scene->get(group);
        if (!geom || !geom->isEnabled() || geom->type != TRIANGLE_MESH) continue;
        TriangleMesh* mesh = (TriangleMesh*) geom;
        if (mesh->numTimeSteps != 1) continue;
        const size_t numTriangles = min(mesh->numTriangles-offset,endID-currentID);
        
        for (size_t i=0; i<numTriangles; i++)	  
        {
          const BBox3fa b = mesh->bounds(offset+i);
          const ssef lower = (ssef)b.lower;
          const ssef upper = (ssef)b.upper;
          const ssef centroid = lower+upper;
          const ssei binID = ssei((centroid-base)*scale);
          unsigned int index = offset+i;
          if (this->mesh == NULL) index |= group << encodeShift;
          ax[slots] = extract<0>(binID);
          ay[slots] = extract<1>(binID);
          az[slots] = extract<2>(binID);
          ai[slots] = index;
          slots++;
          currentID++;
          
          if (slots == 4)
          {
            const ssei code = bitInterleave(ax,ay,az);
            storeu4i(&dest[currentID-4],unpacklo(code,ai));
            storeu4i(&dest[currentID-2],unpackhi(code,ai));
            slots = 0;
          }
        }
        offset = 0;
        if (currentID == endID) break;
      }
      
      if (slots != 0)
      {
        const ssei code = bitInterleave(ax,ay,az);
        for (size_t i=0; i<slots; i++) {
          dest[currentID-slots+i].index = ai[i];
          dest[currentID-slots+i].code = code[i];
        }
      }
      destID = currentID - destID;
    }
    
    void BVH4BuilderMortonGeneral::computeMortonCodes(const size_t threadID, const size_t numThreads)
    {      
      const size_t startID = (threadID+0)*numPrimitives/numThreads;
      const size_t endID   = (threadID+1)*numPrimitives/numThreads;
      
      /* store the morton codes temporarily in 'node' memory */
      //MortonID32Bit* __restrict__ const dest = (MortonID32Bit*)bvh->alloc.base();
      MortonID32Bit* __restrict__ const dest = (MortonID32Bit*)bvh->alloc2.ptr();
      computeMortonCodes(startID,endID,state->dest[threadID],state->startGroup[threadID],state->startGroupOffset[threadID],dest);
    }
    
    void BVH4BuilderMortonGeneral::recreateMortonCodes(BuildRecord& current) const
    {
      assert(current.size() > 4);
      CentGeomBBox3fa global_bounds;
      global_bounds.reset();
      
      for (size_t i=current.begin; i<current.end; i++)
      {
        const size_t index  = morton[i].index;
        const size_t primID = index & encodeMask; 
        const size_t geomID = index >> encodeShift; 
        const TriangleMesh* mesh = this->mesh ? this->mesh : scene->getTriangleMesh(geomID);
        global_bounds.extend(mesh->bounds(primID));
      }
      
      /* compute mapping from world space into 3D grid */
      const ssef base  = (ssef)global_bounds.centBounds.lower;
      const ssef diag  = (ssef)global_bounds.centBounds.upper - (ssef)global_bounds.centBounds.lower;
      const ssef scale = select(diag > ssef(1E-19f), rcp(diag) * ssef(LATTICE_SIZE_PER_DIM * 0.99f),ssef(0.0f));
      
      for (size_t i=current.begin; i<current.end; i++)
      {
        const size_t index  = morton[i].index;
        const size_t primID = index & encodeMask; 
        const size_t geomID = index >> encodeShift; 
        const TriangleMesh* mesh = this->mesh ? this->mesh : scene->getTriangleMesh(geomID);
        const BBox3fa b = mesh->bounds(primID);
        const ssef lower = (ssef)b.lower;
        const ssef upper = (ssef)b.upper;
        const ssef centroid = lower+upper;
        const ssei binID = ssei((centroid-base)*scale);
        const unsigned int bx = extract<0>(binID);
        const unsigned int by = extract<1>(binID);
        const unsigned int bz = extract<2>(binID);
        const unsigned int code = bitInterleave(bx,by,bz);
        morton[i].code  = code;
      }
      std::sort(morton+current.begin,morton+current.end);
      
#if defined(DEBUG)
      for (size_t i=current.begin; i<current.end-1; i++)
        assert(morton[i].code <= morton[i+1].code);
#endif	    
    }
    
    void BVH4BuilderMortonGeneral::radixsort(const size_t threadID, const size_t numThreads)
    {
      const size_t startID = (threadID+0)*numPrimitives/numThreads;
      const size_t endID   = (threadID+1)*numPrimitives/numThreads;
      
      MortonID32Bit* __restrict__ mortonID[2];
      mortonID[0] = (MortonID32Bit*) morton; 
      //mortonID[1] = (MortonID32Bit*) bvh->alloc.base();
      mortonID[1] = (MortonID32Bit*) bvh->alloc2.ptr();
      MortonBuilderState::ThreadRadixCountTy* radixCount = state->radixCount;
      
      /* we need 3 iterations to process all 32 bits */
      for (size_t b=0; b<3; b++)
      {
        const MortonID32Bit* __restrict src = (MortonID32Bit*) &mortonID[((b+1)%2)][0];
        MortonID32Bit*       __restrict dst = (MortonID32Bit*) &mortonID[((b+0)%2)][0];
        
        /* shift and mask to extract some number of bits */
        const unsigned int mask = RADIX_BUCKETS_MASK;
        const unsigned int shift = b * RADIX_BITS;
        
        /* count how many items go into the buckets */
        for (size_t i=0; i<RADIX_BUCKETS; i++)
          radixCount[threadID][i] = 0;
        
        for (size_t i=startID; i<endID; i++) {
          const size_t index = src[i].get(shift, mask);
          radixCount[threadID][index]++;
        }
        //TaskScheduler::syncThreads(threadID,numThreads);
	barrier.wait(threadID,numThreads);
        
        /* calculate total number of items for each bucket */
        __aligned(64) size_t total[RADIX_BUCKETS];
        for (size_t i=0; i<RADIX_BUCKETS; i++)
          total[i] = 0;
        
        for (size_t i=0; i<numThreads; i++)
          for (size_t j=0; j<RADIX_BUCKETS; j++)
            total[j] += radixCount[i][j];
        
        /* calculate start offset of each bucket */
        __aligned(64) size_t offset[RADIX_BUCKETS];
        offset[0] = 0;
        for (size_t i=1; i<RADIX_BUCKETS; i++)    
          offset[i] = offset[i-1] + total[i-1];
        
        /* calculate start offset of each bucket for this thread */
        for (size_t j=0; j<RADIX_BUCKETS; j++)
          for (size_t i=0; i<threadID; i++)
            offset[j] += radixCount[i][j];
        
        /* copy items into their buckets */
        for (size_t i=startID; i<endID; i++) {
          const size_t index = src[i].get(shift, mask);
          dst[offset[index]++] = src[i];
        }
        if (b < 2) barrier.wait(threadID,numThreads);
	  //TaskScheduler::syncThreads(threadID,numThreads);
      }
    }
    
    // =======================================================================================================
    // =======================================================================================================
    // =======================================================================================================
    
    void BVH4Triangle4BuilderMortonGeneral::createSmallLeaf(BuildRecord& current, Allocator* alloc, BBox3fa& box_o)
    {
      ssef lower(pos_inf);
      ssef upper(neg_inf);
      size_t items = current.size();
      size_t start = current.begin;
      assert(items<=4);
      
      /* allocate leaf node */
      Triangle4* accel = (Triangle4*) alloc->alloc1.malloc(sizeof(Triangle4));
      *current.parent = bvh->encodeLeaf((char*)accel,listMode ? listMode : 1);
      
      ssei vgeomID = -1, vprimID = -1, vmask = -1;
      sse3f v0 = zero, v1 = zero, v2 = zero;
      
      for (size_t i=0; i<items; i++)
      {
        const size_t index = morton[start+i].index;
        const size_t primID = index & encodeMask; 
        const size_t geomID = this->mesh ? this->mesh->id : (index >> encodeShift); 
        const TriangleMesh* mesh = scene->getTriangleMesh(geomID);
        const TriangleMesh::Triangle& tri = mesh->triangle(primID);
        const Vec3fa& p0 = mesh->vertex(tri.v[0]);
        const Vec3fa& p1 = mesh->vertex(tri.v[1]);
        const Vec3fa& p2 = mesh->vertex(tri.v[2]);
        lower = min(lower,(ssef)p0,(ssef)p1,(ssef)p2);
        upper = max(upper,(ssef)p0,(ssef)p1,(ssef)p2);
        vgeomID [i] = geomID;
        vprimID [i] = primID;
        vmask   [i] = mesh->mask;
        v0.x[i] = p0.x; v0.y[i] = p0.y; v0.z[i] = p0.z;
        v1.x[i] = p1.x; v1.y[i] = p1.y; v1.z[i] = p1.z;
        v2.x[i] = p2.x; v2.y[i] = p2.y; v2.z[i] = p2.z;
      }
      Triangle4::store_nt(accel,Triangle4(v0,v1,v2,vgeomID,vprimID,vmask,listMode));
      box_o = BBox3fa((Vec3fa)lower,(Vec3fa)upper);
    }

    
    
    // =======================================================================================================
    // =======================================================================================================
    // =======================================================================================================
    
    void BVH4BuilderMortonGeneral::build_parallel_morton(size_t threadIndex, size_t threadCount, size_t taskIndex, size_t taskCount) 
    {
      /* start measurement */
      double t0 = 0.0f;
      if (g_verbose >= 2) t0 = getSeconds();

      //int numThreads = tbb::task_scheduler_init::default_num_threads();
      //std::cout << "numThreads = " << numThreads << std::endl;
      //tbb::task_scheduler_init init(numThreads);

      //bvh->alloc.init(numPrimitives*sizeof(BVH4::Node),numPrimitives*sizeof(BVH4::Node));
      size_t bytesAllocated = (numPrimitives+7)/8*sizeof(BVH4::Node) + size_t(1.2f*(numPrimitives+3)/4)*sizeof(Triangle4);
      bvh->alloc2.init(bytesAllocated,2*bytesAllocated);

      /* compute scene bounds */
      global_bounds.reset();
      scheduler->dispatchTask( task_computeBounds, this, threadIndex, threadCount );
      bvh->bounds = global_bounds.geomBounds;

      /* calculate initial destination for each thread */
      for (size_t i=0; i<threadCount; i++)
	state->dest[i] = i*numPrimitives/threadCount;

      /* compute morton codes */
      scheduler->dispatchTask( task_computeMortonCodes, this, threadIndex, threadCount );   

      /* calculate new destinations */
      size_t cnt = 0;
      for (size_t i=0; i<threadCount; i++) {
	size_t n = state->dest[i]; state->dest[i] = cnt; cnt += n;
      }
      
      /* if primitive got filtered out, run again */
      if (cnt < numPrimitives) {
	scheduler->dispatchTask( task_computeMortonCodes, this, threadIndex, threadCount );   
	numPrimitives = cnt;
      }
      
      /* padding */
      MortonID32Bit* __restrict__ const dest = (MortonID32Bit*) bvh->alloc2.ptr();
      for (size_t i=numPrimitives; i<( (numPrimitives+7)&(-8) ); i++) {
        dest[i].code  = 0xffffffff; 
        dest[i].index = 0;
      }

      /* sort morton codes */
      barrier.init(threadCount);
      scheduler->dispatchTask( task_radixsort, this, threadIndex, threadCount );

#if defined(DEBUG)
      for (size_t i=1; i<numPrimitives; i++)
        assert(morton[i-1].code <= morton[i].code);
#endif	    

      BuildRecord br;
      br.init(0,numPrimitives);
      br.parent = &bvh->root;
      br.depth = 1;
      
      BBox3fa bounds = empty;
      //LockStepTaskScheduler::execute_tbb([&] { bounds = recurse_tbb(br, NULL); });
      bounds = recurse_tbb(br, NULL);

      /* stop measurement */
      if (g_verbose >= 2) dt = getSeconds()-t0;
    }

    Builder* BVH4Triangle4BuilderMortonGeneral  (void* bvh, Scene* scene, size_t mode) { return new class BVH4Triangle4BuilderMortonGeneral ((BVH4*)bvh,scene,mode); }
  }
}

  
  
