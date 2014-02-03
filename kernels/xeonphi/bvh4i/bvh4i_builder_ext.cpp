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

#include "bvh4i/bvh4i.h"
#include "bvh4i/bvh4i_builder.h"

#define PRESPLIT_SPACE_FACTOR         0.45f
#define PRESPLIT_AREA_THRESHOLD      20.0f
#define PRESPLIT_MIN_AREA             0.01f
#define NUM_PRESPLITS_PER_TRIANGLE    16
#define PRESPLITS_TREE_DEPTH          4

#define DBG(x) 

#define L1_PREFETCH_ITEMS 2
#define L2_PREFETCH_ITEMS 16

namespace embree
{

  /* =================================================================================== */
  /* =================================================================================== */
  /* =================================================================================== */

  void BVH4iBuilderPreSplits::printBuilderName()
  {
    std::cout << "building BVH4i with presplits-based SAH builder (MIC) ... " << std::endl;    
  }

  void BVH4iBuilderPreSplits::allocateData(const size_t threadCount, const size_t totalNumPrimitives)
  {
    DBG(PING);
    size_t numPrimitivesOld = numPrimitives;
    numPrimitives = totalNumPrimitives;
    DBG(DBG_PRINT(numPrimitives));

    if (numPrimitivesOld != numPrimitives)
      {
	const size_t preSplitPrims = (size_t)((float)numPrimitives * PRESPLIT_SPACE_FACTOR);
	const size_t numPrims = numPrimitives+preSplitPrims;
	const size_t minAllocNodes = numPrims ? threadCount * ALLOCATOR_NODE_BLOCK_SIZE * 4: 16;
	const size_t numNodes = max((size_t)(numPrims * BVH_NODE_PREALLOC_FACTOR),minAllocNodes);

	numMaxPrimitives = numPrims;
	numMaxPreSplits  = numPrims - numPrimitives;

	if (g_verbose >= 2)
	  {
	    DBG_PRINT(numPrimitives);
	    DBG_PRINT(numMaxPrimitives);
	    DBG_PRINT(numMaxPreSplits);
	  };

	allocateMemoryPools(numPrims,numNodes);
      }    
  }


  __forceinline void splitTri(const PrimRef& prim, int dim, float pos, const Vec3fa& a, const Vec3fa& b, const Vec3fa& c, PrimRef& left_o, PrimRef& right_o)
  {
    BBox3fa left = empty, right = empty;
    const Vec3fa v[3] = { a,b,c };

    /* clip triangle to left and right box by processing all edges */
    Vec3fa v1 = v[2];
    for (size_t i=0; i<3; i++)
      {
	Vec3fa v0 = v1; v1 = v[i];
	float v0d = v0[dim], v1d = v1[dim];
      
	if (v0d <= pos) left. extend(v0); // this point is on left side
	if (v0d >= pos) right.extend(v0); // this point is on right side

	if ((v0d < pos && pos < v1d) || (v1d < pos && pos < v0d)) // the edge crosses the splitting location
	  {
	    assert((v1d-v0d) != 0.0f);
	    Vec3fa c = v0 + (pos-v0d)/(v1d-v0d)*(v1-v0);
	    left.extend(c);
	    right.extend(c);
	  }
      }
    assert(!left.empty());  // happens if split does not hit triangle
    assert(!right.empty()); // happens if split does not hit triangle

    /* safe clip against current bounds */
    BBox3fa bounds = prim.bounds();
    BBox3fa cleft(min(max(left.lower,bounds.lower),bounds.upper),
                 max(min(left.upper,bounds.upper),bounds.lower));
    BBox3fa cright(min(max(right.lower,bounds.lower),bounds.upper),
                  max(min(right.upper,bounds.upper),bounds.lower));

    left_o  = PrimRef(cleft, prim.geomID(), prim.primID());
    right_o = PrimRef(cright,prim.geomID(), prim.primID());
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

  __forceinline mic_f tri_sah( const mic_f &v0,
			       const mic_f &v1,
			       const mic_f &v2) 
  {
    const mic_f n = lcross_xyz(v1-v0,v2-v0);
    return sqrt(ldot3_xyz(n,n)) * 0.5f;
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

  __forceinline size_t getMaxDim(const PrimRef &p)
  {
    Vec3fa diag = p.upper - p.lower;
    size_t index = 0;
    for (size_t i=1;i<3;i++)
      if (diag[i] > diag[index])
	index = i;
    return index;
  }

  void subdivideTriangle(const PrimRef &primBounds,
			 const Vec3fa& vtxA,
			 const Vec3fa& vtxB,
			 const Vec3fa& vtxC,
			 const size_t depth,
			 AlignedAtomicCounter32 &counter,
			 PrimRef *__restrict__ prims)
  {
    DBG(
	DBG_PRINT(primBounds);
	DBG_PRINT(depth);
	);

    if (depth == 0) 
      {	    
	const unsigned int index = counter.inc();
	prims[index] = primBounds;
      }
    else
      {
	const size_t dim = getMaxDim(primBounds);
	
	PrimRef left,right;

	const float pos = (primBounds.upper[dim] + primBounds.lower[dim]) * 0.5f;
	splitTri(primBounds,dim,pos,vtxA,vtxB,vtxC,left,right);

	DBG(
	    DBG_PRINT(left);
	    DBG_PRINT(right);
	    );

	subdivideTriangle( left ,vtxA,vtxB,vtxC, depth-1,counter,prims);
	subdivideTriangle( right,vtxA,vtxB,vtxC, depth-1,counter,prims);
      }
  }

  
  void BVH4iBuilderPreSplits::computePrimRefs(const size_t threadIndex, const size_t threadCount)
  {
    DBG(PING);

    dest0.reset(0);
    dest1.reset(numPrimitives-1);

    LockStepTaskScheduler::dispatchTask( task_computePrimRefsPreSplits, this, threadIndex, threadCount );

    const size_t startFactor = dest0;
    const size_t numFactorTris = numPrimitives - startFactor;

    DBG(
	DBG_PRINT( startFactor );
	DBG_PRINT( numFactorTris );
	);

    quicksort_ascending_primrefs(prims,startFactor, numPrimitives-1);
    
    PrimRef *presplits = (PrimRef*)accel;

    const size_t step = (numMaxPreSplits+NUM_PRESPLITS_PER_TRIANGLE-1) / NUM_PRESPLITS_PER_TRIANGLE;


    const size_t startPreSplits = numPrimitives - step;
    dest0 = startPreSplits;

    DBG(
	DBG_PRINT(step);
	DBG_PRINT(dest0);
	DBG_PRINT(numPrimitives);
	);

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

	Vec3fa vtxA = *(Vec3fa*)vptr0;
	Vec3fa vtxB = *(Vec3fa*)vptr1;
	Vec3fa vtxC = *(Vec3fa*)vptr2;

	BBox3fa bounds = empty;
	bounds.extend(vtxA);
	bounds.extend(vtxB);
	bounds.extend(vtxC);
	PrimRef primBounds = PrimRef(bounds,geomID,primID);

	subdivideTriangle(primBounds,vtxA,vtxB,vtxC,PRESPLITS_TREE_DEPTH,dest1,presplits);
	
      }

    assert( startPreSplits + dest1 < numMaxPrimitives);

    for (size_t i=0;i<dest1;i++)
      prims[startPreSplits + i] = presplits[i];

    numPrimitives = dest1 + startPreSplits;    
    //numPrimitives = startPreSplits;    

  }



  void BVH4iBuilderPreSplits::computePrimRefsPreSplits(const size_t threadID, const size_t numThreads) 
  {
    DBG(PING);
    const size_t numGroups = scene->size();
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
      if (unlikely(mesh->numTimeSteps != 1)) continue;

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
      if (unlikely(mesh->numTimeSteps != 1)) continue;

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

	DBG(
	    DBG_PRINT(area_tri);
	    DBG_PRINT(area_box);
	    DBG_PRINT(factor);
	    );

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
    /* count total number of virtual objects */
    size_t numVirtualObjects = 0;       
    for (size_t i=0;i<scene->size();i++)
      {
	if (unlikely(scene->get(i) == NULL)) continue;
	if (unlikely((scene->get(i)->type != USER_GEOMETRY) && (scene->get(i)->type != INSTANCES))) continue;
	if (unlikely(!scene->get(i)->isEnabled())) continue;
        UserGeometryScene::Base* geom = (UserGeometryScene::Base*) scene->get(i);
	numVirtualObjects += geom->size();
      }
    return numVirtualObjects;	
  }

  void BVH4iBuilderVirtualGeometry::computePrimRefs(const size_t threadIndex, const size_t threadCount)
  {
    DBG(PING);
    LockStepTaskScheduler::dispatchTask( task_computePrimRefsVirtualGeometry, this, threadIndex, threadCount );	
  }

  void BVH4iBuilderVirtualGeometry::createAccel(const size_t threadIndex, const size_t threadCount)
  {
    DBG(PING);
    LockStepTaskScheduler::dispatchTask( task_createVirtualGeometryAccel, this, threadIndex, threadCount );
  }

  void BVH4iBuilderVirtualGeometry::computePrimRefsVirtualGeometry(const size_t threadID, const size_t numThreads) 
  {
    DBG(PING);

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

    // === find first group containing startID ===
    unsigned int g=0, numSkipped = 0;
    for (; g<numTotalGroups; g++) {       
      if (unlikely(scene->get(g) == NULL)) continue;
      if (unlikely((scene->get(g)->type != USER_GEOMETRY) && (scene->get(g)->type != INSTANCES))) continue;
      if (unlikely(!scene->get(g)->isEnabled())) continue;
      const UserGeometryScene::Base* const geom = (UserGeometryScene::Base*) scene->get(g);
      const size_t numPrims = geom->size();
      if (numSkipped + numPrims > startID) break;
      numSkipped += numPrims;
    }

    /* start with first group containing startID */
    mic_f bounds_scene_min((float)pos_inf);
    mic_f bounds_scene_max((float)neg_inf);
    mic_f bounds_centroid_min((float)pos_inf);
    mic_f bounds_centroid_max((float)neg_inf);

    unsigned int num = 0;
    unsigned int currentID = startID;
    unsigned int offset = startID - numSkipped;

    for (; g<numTotalGroups; g++) 
      {
	if (unlikely(scene->get(g) == NULL)) continue;
	if (unlikely((scene->get(g)->type != USER_GEOMETRY ) && (scene->get(g)->type != INSTANCES))) continue;
	if (unlikely(!scene->get(g)->isEnabled())) continue;

	UserGeometryScene::Base *virtual_geometry = (UserGeometryScene::Base *)scene->get(g);

        size_t N = virtual_geometry->size();
        for (unsigned int i=offset; i<N && currentID < endID; i++, currentID++)	 
        { 			    
          const BBox3fa bounds = virtual_geometry->bounds(i);
          const mic_f bmin = broadcast4to16f(&bounds.lower); 
          const mic_f bmax = broadcast4to16f(&bounds.upper);

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
          prims[currentID].upper.a = i;
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


  void BVH4iBuilderVirtualGeometry::createVirtualGeometryAccel(const size_t threadID, const size_t numThreads)
  {
    DBG(PING);

    const size_t startID = (threadID+0)*numPrimitives/numThreads;
    const size_t endID   = (threadID+1)*numPrimitives/numThreads;

    AccelSetItem *acc = (AccelSetItem*)accel + startID;

    const PrimRef* __restrict__  bptr = prims + startID;

    for (size_t j=startID; j<endID; j++, bptr++, acc++)
      {
	prefetch<PFHINT_NT>(bptr + L1_PREFETCH_ITEMS);
	prefetch<PFHINT_L2>(bptr + L2_PREFETCH_ITEMS);
	assert(bptr->geomID() < scene->size() );
        AccelSet* _accel = (AccelSet*)(UserGeometryScene::Base *) scene->get( bptr->geomID() );
	acc->accel = _accel;
        acc->item = bptr->primID();
      }
  }

};
