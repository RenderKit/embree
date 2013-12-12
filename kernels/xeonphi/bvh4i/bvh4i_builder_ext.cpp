// ======================================================================== //
// Copyright 2009-2013 Intel Corporation                                    //
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

#include "kernels/xeonphi/bvh4i/bvh4i.h"
#include "kernels/xeonphi/bvh4i/bvh4i_builder.h"
#include "kernels/xeonphi/bvh4i/bvh4i_builder_util_mic.h"

#define PRESPLIT_SPACE_FACTOR     0.5f
#define PRESPLIT_AREA_THRESHOLD  12.0f
#define PRESPLIT_MIN_AREA         0.001f

#define DBG(x) 


namespace embree
{

  /* =================================================================================== */
  /* =================================================================================== */
  /* =================================================================================== */

  void BVH4iBuilderPreSplits::printBuilderName()
  {
    std::cout << "building BVH4i with presplits-based SAH builder (MIC) ... " << std::endl;    
  }

  void BVH4iBuilderPreSplits::allocateData(size_t threadCount,size_t totalNumPrimitives)
  {
    DBG(PING);
    size_t numPrimitivesOld = numPrimitives;
    numPrimitives = totalNumPrimitives;
    DBG(DBG_PRINT(numPrimitives));

    if (numPrimitivesOld != numPrimitives || numPrimitives == 0)
      {
	const size_t preSplitPrims = (size_t)((float)numPrimitives * PRESPLIT_SPACE_FACTOR);
	const size_t numPrims = numPrimitives+preSplitPrims;
	const size_t minAllocNodes = numPrims ? threadCount * ALLOCATOR_NODE_BLOCK_SIZE * 4: 16;
	const size_t numNodes = max((size_t)(numPrims * BVH_NODE_PREALLOC_FACTOR),minAllocNodes);

	numMaxPrimitives = numPrims;
	numMaxPreSplits  = numPrims - numPrimitives;

	DBG_PRINT(numPrimitives);
	DBG_PRINT(numMaxPrimitives);
	DBG_PRINT(numMaxPreSplits);

	allocateMemoryPools(numPrims,numNodes);
      }    
  }


  __forceinline mic_f box_sah( const mic_f &b_min,
			       const mic_f &b_max) 
  { 
    const mic_f d = b_max - b_min;
    const mic_f d_x = swAAAA(d);
    const mic_f d_y = swBBBB(d);
    const mic_f d_z = swCCCC(d);
    return (d_x*(d_y+d_z)+d_y*d_z)*2.0f; 
  }

  __forceinline mic_f box_sah( const PrimRef &r) 
  { 
    const mic_f bmin = broadcast4to16f(&r.lower);
    const mic_f bmax = broadcast4to16f(&r.upper);
    return box_sah(bmin,bmax);
  }
  
  void quicksort_ascending_primrefs(PrimRef *__restrict__ t, 
				    const ssize_t begin, 
				    const ssize_t end)
  {
    if (likely(begin < end)) 
    {      
      const PrimRef pivotvalue = t[begin];
      ssize_t left  = begin - 1;
      ssize_t right = end   + 1;
      
      while(1) 
      {
        while (box_sah(t[--right]) > box_sah(pivotvalue));
        while (box_sah(t[++left]) < box_sah(pivotvalue));
        
        if (left >= right) break;
        
        const PrimRef temp = t[right];
        t[right] = t[left];
        t[left] = temp;
      }
      
      const int pivot = right;
      quicksort_ascending_primrefs(t, begin, pivot);
      quicksort_ascending_primrefs(t, pivot + 1, end);
    }
  }


  void BVH4iBuilderPreSplits::computePrimRefs(size_t threadIndex, size_t threadCount)
  {
    DBG(PING);

    dest0.reset(0);
    dest1.reset(numPrimitives-1);

    LockStepTaskScheduler::dispatchTask( task_computePrimRefsPreSplits, this, threadIndex, threadCount );

    const size_t startFactor = dest0;
    const size_t numFactorTris = numPrimitives - startFactor;

    DBG_PRINT( startFactor );
    DBG_PRINT( numFactorTris );

    quicksort_ascending_primrefs(prims,startFactor, numPrimitives-1);

#if 0
    for (size_t i=dest1 + 1;i<numPrimitives;i++)
      {
	DBG_PRINT(i);
	DBG_PRINT( prims[i] );
	DBG_PRINT(box_sah( prims[i] ));
      }
#endif
    
    PrimRef *presplits = (PrimRef*)accel;

    const size_t step = (numMaxPreSplits+3) / 4;

    DBG_PRINT(step);

    const size_t startPreSplits = numPrimitives - step;
    dest0 = startPreSplits;

    DBG_PRINT(dest0);
    DBG_PRINT(numPrimitives);

    dest1.reset(0);


    for (size_t i=dest0;i<numPrimitives;i++)
      {
	const size_t geomID = prims[i].geomID();
	const size_t primID = prims[i].primID();

	const TriangleMeshScene::TriangleMesh* __restrict__ const mesh = scene->getTriangleMesh(geomID);
	const TriangleMeshScene::TriangleMesh::Triangle & tri = mesh->triangle(primID);

	const float *__restrict__ const vptr0 = (float*)&mesh->vertex(tri.v[0]);
	const float *__restrict__ const vptr1 = (float*)&mesh->vertex(tri.v[1]);
	const float *__restrict__ const vptr2 = (float*)&mesh->vertex(tri.v[2]);

	const mic_f v0 = broadcast4to16f(vptr0);
	const mic_f v1 = broadcast4to16f(vptr1);
	const mic_f v2 = broadcast4to16f(vptr2);
	
	const unsigned int dest_index = dest1.add(4);

	const mic_f v01 = (v0 + v1) * 0.5f;
	const mic_f v12 = (v1 + v2) * 0.5f;
	const mic_f v20 = (v2 + v0) * 0.5f;

	const mic_f bminA = min(min(v0,v01),v20);
	const mic_f bmaxA = max(max(v0,v01),v20);

	const mic_f bminB = min(min(v01,v1),v12);
	const mic_f bmaxB = max(max(v01,v1),v12);

	const mic_f bminC = min(min(v20,v12),v2);
	const mic_f bmaxC = max(max(v20,v12),v2);

	const mic_f bminD = min(min(v01,v12),v20);
	const mic_f bmaxD = max(max(v01,v12),v20);

	PrimRef &refA = presplits[dest_index+0];
	PrimRef &refB = presplits[dest_index+1];
	PrimRef &refC = presplits[dest_index+2];
	PrimRef &refD = presplits[dest_index+3];

	store4f(&refA.lower,bminA);
	store4f(&refA.upper,bmaxA);
	refA.lower.a = geomID; 
	refA.upper.a = primID;

	store4f(&refB.lower,bminB);
	store4f(&refB.upper,bmaxB);
	refB.lower.a = geomID; 
	refB.upper.a = primID;

	store4f(&refC.lower,bminC);
	store4f(&refC.upper,bmaxC);
	refC.lower.a = geomID; 
	refC.upper.a = primID;

	store4f(&refD.lower,bminD);
	store4f(&refD.upper,bmaxD);
	refD.lower.a = geomID; 
	refD.upper.a = primID;
	
      }

    assert( startPreSplits + dest1 < numMaxPrimitives);

    for (size_t i=0;i<dest1;i++)
      prims[startPreSplits + i] = presplits[i];

    numPrimitives = dest1 + startPreSplits;    
    DBG_PRINT( dest1 + startPreSplits );
  }



  __forceinline mic_f tri_sah( const mic_f &v0,
			       const mic_f &v1,
			       const mic_f &v2) 
  {
    const mic_f n = lcross_xyz(v1-v0,v2-v0);
    return sqrt(ldot3_xyz(n,n)) * 0.5f;
  }

  void BVH4iBuilderPreSplits::computePrimRefsPreSplits(const size_t threadID, const size_t numThreads) 
  {
    DBG(PING);
    const size_t numGroups = source->size();
    const size_t startID = (threadID+0)*numPrimitives/numThreads;
    const size_t endID   = (threadID+1)*numPrimitives/numThreads;
    
    PrimRef *__restrict__ const prims     = this->prims;

    // === find first group containing startID ===
    unsigned int g=0, numSkipped = 0;
    for (; g<numGroups; g++) {       
      if (unlikely(scene->get(g) == NULL)) continue;
      if (unlikely(scene->get(g)->type != TRIANGLE_MESH)) continue;
      const TriangleMeshScene::TriangleMesh* __restrict__ const mesh = scene->getTriangleMesh(g);
      if (unlikely(!mesh->isEnabled())) continue;

      const size_t numTriangles = mesh->numTriangles;
      if (numSkipped + numTriangles > startID) break;
      numSkipped += numTriangles;
    }

    // === start with first group containing startID ===
    mic_f bounds_scene_min((float)pos_inf);
    mic_f bounds_scene_max((float)neg_inf);
    mic_f bounds_centroid_min((float)pos_inf);
    mic_f bounds_centroid_max((float)neg_inf);

    unsigned int num = 0;
    unsigned int currentID = startID;
    unsigned int offset = startID - numSkipped;

    for (; g<numGroups; g++) 
    {
      if (unlikely(scene->get(g) == NULL)) continue;
      if (unlikely(scene->get(g)->type != TRIANGLE_MESH)) continue;
      const TriangleMeshScene::TriangleMesh* __restrict__ const mesh = scene->getTriangleMesh(g);
      if (unlikely(!mesh->isEnabled())) continue;

      for (unsigned int i=offset; i<mesh->numTriangles && currentID < endID; i++, currentID++)	 
      { 			    
	const TriangleMeshScene::TriangleMesh::Triangle& tri = mesh->triangle(i);
	prefetch<PFHINT_L2>(&tri + L2_PREFETCH_ITEMS);
	prefetch<PFHINT_L1>(&tri + L1_PREFETCH_ITEMS);

	const float *__restrict__ const vptr0 = (float*)&mesh->vertex(tri.v[0]);
	const float *__restrict__ const vptr1 = (float*)&mesh->vertex(tri.v[1]);
	const float *__restrict__ const vptr2 = (float*)&mesh->vertex(tri.v[2]);

	const mic_f v0 = broadcast4to16f(vptr0);
	const mic_f v1 = broadcast4to16f(vptr1);
	const mic_f v2 = broadcast4to16f(vptr2);

	mic_f bmin = min(min(v0,v1),v2);
	mic_f bmax = max(max(v0,v1),v2);


	const mic_f area_tri = tri_sah(v0,v1,v2);
	const mic_f area_box = box_sah(bmin,bmax);
	const mic_f factor = area_box * rcp(area_tri);

#if 0
	DBG_PRINT(area_tri);
	DBG_PRINT(area_box);
	DBG_PRINT(factor);
#endif
	const mic_m m_factor = factor > PRESPLIT_AREA_THRESHOLD;
	const mic_m m_sah_zero = area_box > PRESPLIT_MIN_AREA;

	if (any(m_factor & m_sah_zero)) 
	  {
	    const unsigned int d_index = dest1.dec();
	    store4f(&prims[d_index].lower,bmin);
	    store4f(&prims[d_index].upper,bmax);	
	    prims[d_index].lower.a = g; 
	    prims[d_index].upper.a = i;
	  }
	else
	  {       
	    const unsigned int d_index = dest0.inc();
	    store4f(&prims[d_index].lower,bmin);
	    store4f(&prims[d_index].upper,bmax);	
	    prims[d_index].lower.a = g; 
	    prims[d_index].upper.a = i;
	  }

	bounds_scene_min = min(bounds_scene_min,bmin);
	bounds_scene_max = max(bounds_scene_max,bmax);
	const mic_f centroid2 = bmin+bmax;
	bounds_centroid_min = min(bounds_centroid_min,centroid2);
	bounds_centroid_max = max(bounds_centroid_max,centroid2);

      }
      if (currentID == endID) break;
      offset = 0;
    }

    /* update global bounds */
    Centroid_Scene_AABB bounds;
    
    store4f(&bounds.centroid2.lower,bounds_centroid_min);
    store4f(&bounds.centroid2.upper,bounds_centroid_max);
    store4f(&bounds.geometry.lower,bounds_scene_min);
    store4f(&bounds.geometry.upper,bounds_scene_max);

    global_bounds.extend_atomic(bounds);    
  }

  /* =================================================================================== */
  /* =================================================================================== */
  /* =================================================================================== */

  void BVH4iBuilderVirtualGeometry::printBuilderName()
  {
    std::cout << "building BVH4i with Virtual Geometry SAH builder (MIC) ... " << std::endl;    
  }

  size_t BVH4iBuilderVirtualGeometry::getNumPrimitives()
  {
    DBG(PING);
    /* count total number of virtual objects */
    size_t numVirtualObjects = 0;       
    for (size_t i=0;i<scene->size();i++)
      {
	if (unlikely(scene->get(i) == NULL)) continue;
	if (unlikely(scene->get(i)->type != USER_GEOMETRY)) continue;
	if (unlikely(!scene->get(i)->isEnabled())) continue;
	numVirtualObjects++;
      }
    return numVirtualObjects;	
  }

  void BVH4iBuilderVirtualGeometry::computePrimRefs(size_t threadIndex, size_t threadCount)
  {
    DBG(PING);
    LockStepTaskScheduler::dispatchTask( task_computePrimRefsVirtualGeometry, this, threadIndex, threadCount );	
  }

  void BVH4iBuilderVirtualGeometry::createAccel(size_t threadIndex, size_t threadCount)
  {
    DBG(PING);
    LockStepTaskScheduler::dispatchTask( task_createVirtualGeometryAccel, this, threadIndex, threadCount );
  }

  void BVH4iBuilderVirtualGeometry::computePrimRefsVirtualGeometry(const size_t threadID, const size_t numThreads) 
  {
    const size_t numTotalGroups = scene->size();

    /* count total number of virtual objects */
    const size_t numVirtualObjects = numPrimitives;
    const size_t startID   = (threadID+0)*numVirtualObjects/numThreads;
    const size_t endID     = (threadID+1)*numVirtualObjects/numThreads;

    DBG(
	DBG_PRINT(numTotalGroups);
	DBG_PRINT(numVirtualObjects);
	DBG_PRINT(startID);
	DBG_PRINT(endID);
	);
    
    PrimRef *__restrict__ const prims     = this->prims;

    /* find group == startID */
    unsigned int g=0;
    for (size_t i=0; i<numTotalGroups; i++) {       
      if (unlikely(scene->get(i) == NULL)) continue;
      if (unlikely(scene->get(i)->type != USER_GEOMETRY)) continue;
      if (unlikely(!scene->get(i)->isEnabled())) continue;
      if (g == startID) break;
      g++;
    }

    /* start with first group containing startID */
    mic_f bounds_scene_min((float)pos_inf);
    mic_f bounds_scene_max((float)neg_inf);
    mic_f bounds_centroid_min((float)pos_inf);
    mic_f bounds_centroid_max((float)neg_inf);

    size_t currentID = startID;

    for (; g<numTotalGroups; g++) 
      {
	if (unlikely(scene->get(g) == NULL)) continue;
	if (unlikely(scene->get(g)->type != USER_GEOMETRY )) continue;
	if (unlikely(!scene->get(g)->isEnabled())) continue;

	const UserGeometryScene::Base *virtual_geometry = (UserGeometryScene::Base *)scene->get(g);

	const mic_f bmin = broadcast4to16f(&virtual_geometry->bounds.lower);
	const mic_f bmax = broadcast4to16f(&virtual_geometry->bounds.upper);

	DBG(
	    DBG_PRINT(currentID);
	    DBG_PRINT(bmin);
	    DBG_PRINT(bmax);
	    );

	bounds_scene_min = min(bounds_scene_min,bmin);
	bounds_scene_max = max(bounds_scene_max,bmax);
	const mic_f centroid2 = bmin+bmax;
	bounds_centroid_min = min(bounds_centroid_min,centroid2);
	bounds_centroid_max = max(bounds_centroid_max,centroid2);

	store4f(&prims[currentID].lower,bmin);
	store4f(&prims[currentID].upper,bmax);	
	prims[currentID].lower.a = g;
	prims[currentID].upper.a = 0;
	currentID++;

	if (currentID == endID) break;
      }

    /* update global bounds */
    Centroid_Scene_AABB bounds;
    
    store4f(&bounds.centroid2.lower,bounds_centroid_min);
    store4f(&bounds.centroid2.upper,bounds_centroid_max);
    store4f(&bounds.geometry.lower,bounds_scene_min);
    store4f(&bounds.geometry.upper,bounds_scene_max);

    global_bounds.extend_atomic(bounds);    
  }


  void BVH4iBuilderVirtualGeometry::createVirtualGeometryAccel(const size_t threadID, const size_t numThreads)
  {
    DBG(PING);

    const size_t startID = (threadID+0)*numPrimitives/numThreads;
    const size_t endID   = (threadID+1)*numPrimitives/numThreads;

    Triangle1    * __restrict__  acc  = accel + startID;
    const PrimRef* __restrict__  bptr = prims + startID;

    for (size_t j=startID; j<endID; j++, bptr++, acc++)
      {
	prefetch<PFHINT_NT>(bptr + L1_PREFETCH_ITEMS);
	prefetch<PFHINT_L2>(bptr + L2_PREFETCH_ITEMS);
	assert(bptr->geomID() < scene->size() );
	Accel *tmp = (Accel*)(UserGeometryScene::Base *)(scene->get( bptr->geomID() ));
	*(void**)acc = tmp;
      }
  }

};
