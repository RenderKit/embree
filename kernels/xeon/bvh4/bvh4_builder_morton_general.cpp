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

#define PROFILE_MORTON_GENERAL

#include "bvh4.h"
#include "bvh4_builder_morton_general.h"

#include "geometry/triangle1.h"
#include "geometry/triangle4.h"
#include "geometry/triangle8.h"
#include "geometry/triangle1v.h"
#include "geometry/triangle4v.h"
#include "geometry/triangle4i.h"

#include "algorithms/parallel_for_for.h"

#define DBG(x) 


namespace embree 
{
  namespace isa
  {
    static double dt = 0.0f;
    
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
    
    void BVH4BuilderMortonGeneral::createSmallLeaf(BuildRecord& current, Allocator* alloc, BBox3fa& box_o)
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
      //double T0 = getSeconds();

      Scene::Iterator<TriangleMesh,1> iter1(scene);
      global_bounds = parallel_for_for_reduce( iter1, CentGeomBBox3fa(empty), [&](TriangleMesh* mesh, const range<size_t>& r, size_t k) -> CentGeomBBox3fa
      {
        CentGeomBBox3fa bounds(empty);
        for (size_t i=r.begin(); i<r.end(); i++)
        {
          const BBox3fa b = mesh->bounds(i);
          if (!inFloatRange(b)) continue;
          bounds.extend(b);
        }
        return bounds;
      }, [] (CentGeomBBox3fa a, const CentGeomBBox3fa& b) { a.merge(b); return a; });

      bvh->bounds = global_bounds.geomBounds;
      //double T1 = getSeconds();
      //PRINT(1000.0f*(T1-T0));

      /* calculate initial destination for each thread */
      for (size_t i=0; i<threadCount; i++)
	state->dest[i] = i*numPrimitives/threadCount;

      //double T0 = getSeconds();

      /* compute morton codes */
      MortonID32Bit* __restrict__ const dest = (MortonID32Bit*) bvh->alloc2.ptr();
      //MortonID32Bit* __restrict__ const dest = (MortonID32Bit*) morton;

      /* compute mapping from world space into 3D grid */
      const ssef base  = (ssef)global_bounds.centBounds.lower;
      const ssef diag  = (ssef)global_bounds.centBounds.upper - (ssef)global_bounds.centBounds.lower;
      const ssef scale = select(diag > ssef(1E-19f), rcp(diag) * ssef(LATTICE_SIZE_PER_DIM * 0.99f),ssef(0.0f));
      
      Scene::Iterator<TriangleMesh,1> iter(scene);
      parallel_for_for( iter, [&](TriangleMesh* mesh, const range<size_t>& r, size_t k)
      {
        size_t currentID = k;
        size_t slots = 0;
        ssei ax = 0, ay = 0, az = 0, ai = 0;
                
        for (size_t i=r.begin(); i<r.end(); i++)	  
        {
          const BBox3fa b = mesh->bounds(i);
          const ssef lower = (ssef)b.lower;
          const ssef upper = (ssef)b.upper;
          const ssef centroid = lower+upper;
          const ssei binID = ssei((centroid-base)*scale);
          unsigned int index = i;
          index |= mesh->id << encodeShift;
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
            
        if (slots != 0)
        {
          const ssei code = bitInterleave(ax,ay,az);
          for (size_t i=0; i<slots; i++) {
            dest[currentID-slots+i].index = ai[i];
            dest[currentID-slots+i].code = code[i];
          }
        }
      });

      //double T1 = getSeconds();
      //PRINT(1000.0f*(T1-T0));

      /* padding */     
      for (size_t i=numPrimitives; i<( (numPrimitives+7)&(-8) ); i++) {
        dest[i].code  = 0xffffffff; 
        dest[i].index = 0;
      }

      /* sort morton codes */
      barrier.init(threadCount);
      //double T0 = getSeconds();
      //scheduler->dispatchTask( task_radixsort, this, threadIndex, threadCount );
      //radix_sort_u32(dest,(MortonID32Bit*)bvh->alloc2.ptr(),numPrimitives);
      radix_sort_copy_u32((MortonID32Bit*)bvh->alloc2.ptr(),morton,numPrimitives);
      //double T1 = getSeconds();
      //PRINT(1000.0f*(T1-T0));

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

      Builder* BVH4Triangle4BuilderMortonGeneral  (void* bvh, Scene* scene, size_t mode) { 
        return new class BVH4BuilderMortonGeneral ((BVH4*)bvh,scene,NULL,mode,2,false,sizeof(Triangle4),4,inf); 
      }
  }
}

  
  
