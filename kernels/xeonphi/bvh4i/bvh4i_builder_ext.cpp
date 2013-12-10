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

namespace embree
{

  __forceinline mic_f box_sah( const mic_f &b_min,
			       const mic_f &b_max) 
  { 
    const mic_f d = b_max - b_min;
    const mic_f d_x = swAAAA(d);
    const mic_f d_y = swBBBB(d);
    const mic_f d_z = swCCCC(d);
    return (d_x*(d_y+d_z)+d_y*d_z)*2.0f; 
  }

  __forceinline mic_f tri_sah( const mic_f &v0,
			       const mic_f &v1,
			       const mic_f &v2) 
  {
    const mic_f n = lcross_xyz(v1-v0,v2-v0);
    return sqrt(ldot3_xyz(n,n)) * 0.5f;
  }

  void BVH4iBuilder::computePrimRefsPreSplits(const size_t threadID, const size_t numThreads) 
  {
    const size_t numGroups = source->groups();
    const size_t startID = (threadID+0)*numPrimitives/numThreads;
    const size_t endID   = (threadID+1)*numPrimitives/numThreads;
    
    const Scene* __restrict__ const scene = (Scene*)geometry;
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

    PrimRef *__restrict__ dest = &prims[currentID];

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

	const mic_f bmin = min(min(v0,v1),v2);
	const mic_f bmax = max(max(v0,v1),v2);

	const mic_f area_tri = tri_sah(v0,v1,v2);
	const mic_f area_box = box_sah(bmin,bmax);
	const mic_f factor = area_box * rcp(area_tri);
	if (any(factor > mic_f(20.0f))) atomicID.inc();
#if 0
	DBG_PRINT(area_tri);
	DBG_PRINT(area_box);
	DBG_PRINT(factor);
#endif

	bounds_scene_min = min(bounds_scene_min,bmin);
	bounds_scene_max = max(bounds_scene_max,bmax);
	const mic_f centroid2 = bmin+bmax;
	bounds_centroid_min = min(bounds_centroid_min,centroid2);
	bounds_centroid_max = max(bounds_centroid_max,centroid2);
       
	store4f(&dest->lower,bmin);
	store4f(&dest->upper,bmax);	
	dest->lower.a = g;
	dest->upper.a = i;
	dest++;
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

  void BVH4iBuilder::computePrimRefsVirtual(const size_t threadID, const size_t numThreads) 
  {
    const size_t numGroups = source->groups();
    const size_t startID = (threadID+0)*numPrimitives/numThreads;
    const size_t endID   = (threadID+1)*numPrimitives/numThreads;
    
    const Scene* __restrict__ const scene = (Scene*)geometry;
    PrimRef *__restrict__ const prims     = this->prims;

    // === find first group containing startID ===
    unsigned int g=0, numSkipped = 0;
    for (; g<numGroups; g++) {       
      if (unlikely(scene->get(g) == NULL)) continue;
      if (unlikely(scene->get(g)->type != USER_GEOMETRY)) continue;
      if (unlikely(!scene->get(g)->isEnabled())) continue;

      const size_t numPrimitives = scene->get(g)->numPrimitives;
      if (numSkipped + numPrimitives > startID) break;
      numSkipped += numPrimitives;
    }

    // === start with first group containing startID ===
    mic_f bounds_scene_min((float)pos_inf);
    mic_f bounds_scene_max((float)neg_inf);
    mic_f bounds_centroid_min((float)pos_inf);
    mic_f bounds_centroid_max((float)neg_inf);

    unsigned int num = 0;
    unsigned int currentID = startID;
    unsigned int offset = startID - numSkipped;

    __align(64) PrimRef local_prims[2];
    size_t numLocalPrims = 0;
    PrimRef *__restrict__ dest = &prims[currentID];

    for (; g<numGroups; g++) 
    {
      if (unlikely(scene->get(g) == NULL)) continue;
      if (unlikely(scene->get(g)->type != USER_GEOMETRY )) continue;
      if (unlikely(!scene->get(g)->isEnabled())) continue;
      const Geometry *geometry = scene->get(g);
      for (unsigned int i=offset; i<geometry->numPrimitives && currentID < endID; i++, currentID++)	 
      { 			    
	// const TriangleMeshScene::TriangleMesh::Triangle& tri = mesh->triangle(i);
	// prefetch<PFHINT_L2>(&tri + L2_PREFETCH_ITEMS);
	// prefetch<PFHINT_L1>(&tri + L1_PREFETCH_ITEMS);

	// const float *__restrict__ const vptr0 = (float*)&mesh->vertex(tri.v[0]);
	// const float *__restrict__ const vptr1 = (float*)&mesh->vertex(tri.v[1]);
	// const float *__restrict__ const vptr2 = (float*)&mesh->vertex(tri.v[2]);

	// const mic_f v0 = broadcast4to16f(vptr0);
	// const mic_f v1 = broadcast4to16f(vptr1);
	// const mic_f v2 = broadcast4to16f(vptr2);

	// const mic_f bmin = min(min(v0,v1),v2);
	// const mic_f bmax = max(max(v0,v1),v2);
	// bounds_scene_min = min(bounds_scene_min,bmin);
	// bounds_scene_max = max(bounds_scene_max,bmax);
	// const mic_f centroid2 = bmin+bmax;
	// bounds_centroid_min = min(bounds_centroid_min,centroid2);
	// bounds_centroid_max = max(bounds_centroid_max,centroid2);

	// store4f(&local_prims[numLocalPrims].lower,bmin);
	// store4f(&local_prims[numLocalPrims].upper,bmax);	
	// local_prims[numLocalPrims].lower.a = g;
	// local_prims[numLocalPrims].upper.a = i;
	// numLocalPrims++;
	// if (unlikely(((size_t)dest % 64) != 0) && numLocalPrims == 1)
	//   {
	//     *dest = local_prims[0];
	//     dest++;
	//     numLocalPrims--;
	//   }
	// else
	//   {
	//     const mic_f twoAABBs = load16f(local_prims);
	//     if (numLocalPrims == 2)
	//       {
	// 	numLocalPrims = 0;
	// 	store16f_ngo(dest,twoAABBs);
	// 	dest+=2;
	//       }
	//   }	
      }
      if (currentID == endID) break;
      offset = 0;
    }

    /* is there anything left in the local queue? */

    /* update global bounds */
    Centroid_Scene_AABB bounds;
    
    store4f(&bounds.centroid2.lower,bounds_centroid_min);
    store4f(&bounds.centroid2.upper,bounds_centroid_max);
    store4f(&bounds.geometry.lower,bounds_scene_min);
    store4f(&bounds.geometry.upper,bounds_scene_max);

    global_bounds.extend_atomic(bounds);    
  }

};
