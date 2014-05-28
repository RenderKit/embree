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

//#include "bvh4i_leaf_intersector.h"
#include "bvh4i/bvh4i_traversal.h"
#include "bvh4hair_traversal.h"
#include "bvh4hair_intersector16_single.h"
#include "geometry/bezier1i.h"
#include "geometry/bezier1i_intersector16.h"

#define DBG(x) 

namespace embree
{
  namespace isa
  {
    static unsigned int BVH4I_LEAF_MASK = BVH4i::leaf_mask; // needed due to compiler efficiency bug

    //if (unlikely(pre.mbox.hit(curves[i].geomID,curves[i].primID))) continue;
    //pre.mbox.add(curves[i].geomID,curves[i].primID);


    struct Bezier1iLeafIntersector
    {
      static __forceinline bool intersect(BVH4i::NodeRef curNode,
					  const size_t rayIndex, 
					  const mic_f &dir_xyz,
					  const mic_f &org_xyz,
					  const mic_f &min_dist_xyz,
					  mic_f &max_dist_xyz,
					  Ray16& ray16, 
					  const void *__restrict__ const accel,
					  const Scene*__restrict__ const geometry,
					  Bezier1iIntersector16::Precalculations &pre)
      {
	unsigned int items = curNode.items();
	unsigned int index = curNode.offsetIndex();
	const Bezier1i *__restrict__ const tptr = (Bezier1i*)accel + index;
	bool ret = false;
	prefetch<PFHINT_L1>(tptr + 0);
	prefetch<PFHINT_L1>(tptr + 4);

	for (size_t i=0;i<items;i++)
	  {
	    STAT3(normal.trav_prims,1,1,1);
	    ret |= Bezier1iIntersector16::intersect(pre,ray16,dir_xyz,org_xyz,rayIndex,tptr[i],geometry); // add mailboxing
	  }

	max_dist_xyz = ray16.tfar[rayIndex];

	return ret;
      }

      static __forceinline bool occluded(BVH4i::NodeRef curNode,
					 const size_t rayIndex, 
					 const mic_f &dir_xyz,
					 const mic_f &org_xyz,
					 const mic_f &min_dist_xyz,
					 const mic_f &max_dist_xyz,
					 const Ray16& ray16, 
					 mic_m &m_terminated,
					 const void *__restrict__ const accel,
					 const Scene*__restrict__ const geometry,
					 Bezier1iIntersector16::Precalculations &pre)
      {
	unsigned int items = curNode.items();
	unsigned int index = curNode.offsetIndex();
	const Bezier1i *__restrict__ const tptr = (Bezier1i*)accel + index;
	prefetch<PFHINT_L1>(tptr + 0);
	prefetch<PFHINT_L1>(tptr + 4);

	for (size_t i=0;i<items;i++)
	  {
	    STAT3(shadow.trav_prims,1,1,1);
	    if (Bezier1iIntersector16::occluded(pre,ray16,dir_xyz,org_xyz,rayIndex,tptr[i],geometry))
	      return true;
	  }

	return false;
      }

      static __forceinline bool intersect(BVH4i::NodeRef curNode,
					  const mic_f &dir_xyz,
					  const mic_f &org_xyz,
					  const mic_f &min_dist_xyz,
					  mic_f &max_dist_xyz,
					  Ray& ray, 
					  const void *__restrict__ const accel,
					  const Scene*__restrict__ const geometry,
					  Bezier1iIntersector16::Precalculations &pre)
      {
	unsigned int items = curNode.items();
	unsigned int index = curNode.offsetIndex(); 
	const Bezier1i *__restrict__ const tptr = (Bezier1i*)accel + index;

	int old_primID = ray.primID;

	bool ret = false;
	prefetch<PFHINT_L1>(tptr + 0);
	prefetch<PFHINT_L1>(tptr + 4);

	// for (size_t i=0;i<items;i++)
	//   ret |= Bezier1iIntersector16::intersect(pre,ray16,dir_xyz,org_xyz,rayIndex,tptr[i],geometry); // add mailboxing

	max_dist_xyz = ray.tfar;
	return ret;

	return old_primID != ray.primID;
      }

      static __forceinline bool occluded(BVH4i::NodeRef curNode,
					 const mic_f &dir_xyz,
					 const mic_f &org_xyz,
					 const mic_f &min_dist_xyz,
					 const mic_f &max_dist_xyz,
					 Ray& ray,
					 const void *__restrict__ const accel,
					 const Scene*__restrict__ const geometry,
					 Bezier1iIntersector16::Precalculations &pre)
      {
	unsigned int items = curNode.items();
	unsigned int index = curNode.offsetIndex(); 
	const Bezier1i *__restrict__ const tptr = (Bezier1i*)accel + index;

	prefetch<PFHINT_L1>(tptr + 0);
	prefetch<PFHINT_L1>(tptr + 4);

	// for (size_t i=0;i<items;i++)
	//   if (Bezier1iIntersector16::occluded(pre,ray16,dir_xyz,org_xyz,rayIndex,tptr[i],geometry))
	//     return true;

	return false;
      }

    };



    static unsigned int BVH4HAIR_LEAF_MASK = BVH4Hair::leaf_mask; // needed due to compiler efficiency bug

#if 1

    template<typename LeafIntersector>    
    void BVH4HairIntersector16<LeafIntersector>::intersect(mic_i* valid_i, BVH4Hair* bvh, Ray16& ray16)
    {
      /* near and node stack */
      __aligned(64) float   stack_dist[3*BVH4i::maxDepth+1];
      __aligned(64) BVH4Hair::NodeRef stack_node[3*BVH4i::maxDepth+1];

      LinearSpace_mic3f ray16_space = frame(ray16.dir).transposed();

      /* setup */
      const mic_m m_valid    = *(mic_i*)valid_i != mic_i(0);
      const mic3f rdir16     = rcp_safe(ray16.dir);
      const mic_f inf        = mic_f(pos_inf);
      const mic_f zero       = mic_f::zero();

      store16f(stack_dist,inf);

      const void * __restrict__ accel = (void*)bvh->triPtr();

      stack_node[0] = BVH4Hair::invalidNode;
      long rayIndex = -1;
      while((rayIndex = bitscan64(rayIndex,toInt(m_valid))) != BITSCAN_NO_BIT_SET_64)	    
        {
	  Bezier1iIntersector16::Precalculations pre(ray16_space,rayIndex);
	  
	  stack_node[1] = bvh->unaligned_nodes->child(0);
	  size_t sindex = 2;

	  const mic_f org_xyz      = loadAOS4to16f(rayIndex,ray16.org.x,ray16.org.y,ray16.org.z);
	  const mic_f dir_xyz      = loadAOS4to16f(rayIndex,ray16.dir.x,ray16.dir.y,ray16.dir.z);
	  const mic_f min_dist_xyz = broadcast1to16f(&ray16.tnear[rayIndex]);
	  mic_f       max_dist_xyz = broadcast1to16f(&ray16.tfar[rayIndex]);

	  const mic_f org_xyz1     = select(0x7777,org_xyz,mic_f::one());

	  const size_t leaf_mask = BVH4HAIR_LEAF_MASK;

	  while (1)
	    {

	      BVH4Hair::NodeRef curNode = stack_node[sindex-1];
	      sindex--;

	      traverse_single_intersect(curNode,
					sindex,
					dir_xyz,
					org_xyz1,
					min_dist_xyz,
					max_dist_xyz,
					stack_node,
					stack_dist,
					leaf_mask);

	      /* return if stack is empty */
	      if (unlikely(curNode == BVH4Hair::invalidNode)) break;

	      STAT3(normal.trav_leaves,1,1,1);

	      /* intersect one ray against four triangles */

	      //////////////////////////////////////////////////////////////////////////////////////////////////
	      BVH4i::NodeRef curNode4i = (unsigned int)curNode;
	      const bool hit = LeafIntersector::intersect(curNode4i,
							  rayIndex,
							  dir_xyz,
							  org_xyz,
							  min_dist_xyz,
							  max_dist_xyz,
							  ray16,
							  accel,
							  (Scene*)bvh->geometry,
							  pre);
									   
	      if (hit) 
		{
		  const unsigned int current_dist = *(unsigned int*)&ray16.tfar[rayIndex];
		  compactStack(stack_node,stack_dist,sindex,current_dist,max_dist_xyz);
		}
	      // ------------------------
	    }	  
	}
    }

    template<typename LeafIntersector>    
    void BVH4HairIntersector16<LeafIntersector>::occluded(mic_i* valid_i, BVH4Hair* bvh, Ray16& ray16)
    {
      /* near and node stack */
      __aligned(64) BVH4Hair::NodeRef stack_node[3*BVH4Hair::maxDepth+1];

      /* setup */
      const mic_m m_valid = *(mic_i*)valid_i != mic_i(0);
      const mic3f rdir16  = rcp_safe(ray16.dir);
      mic_m terminated    = !m_valid;
      const mic_f inf     = mic_f(pos_inf);
      const mic_f zero    = mic_f::zero();

      const void * __restrict__ accel = (void*)bvh->triPtr();

      stack_node[0] = BVH4Hair::invalidNode;

      long rayIndex = -1;
      while((rayIndex = bitscan64(rayIndex,toInt(m_valid))) != BITSCAN_NO_BIT_SET_64)	    
        {
	  Bezier1iIntersector16::Precalculations pre(ray16,rayIndex);

	  stack_node[1] = bvh->unaligned_nodes->child(0);
	  size_t sindex = 2;

	  const mic_f org_xyz      = loadAOS4to16f(rayIndex,ray16.org.x,ray16.org.y,ray16.org.z);
	  const mic_f dir_xyz      = loadAOS4to16f(rayIndex,ray16.dir.x,ray16.dir.y,ray16.dir.z);
	  const mic_f min_dist_xyz = broadcast1to16f(&ray16.tnear[rayIndex]);
	  const mic_f max_dist_xyz = broadcast1to16f(&ray16.tfar[rayIndex]);
	  const mic_f org_xyz1     = select(0x7777,org_xyz,mic_f::one());

	  const size_t leaf_mask = BVH4HAIR_LEAF_MASK;

	  while (1)
	    {
	      BVH4Hair::NodeRef curNode = stack_node[sindex-1];
	      sindex--;

	      traverse_single_occluded(curNode,
				       sindex,
				       dir_xyz,
				       org_xyz1,
				       min_dist_xyz,
				       max_dist_xyz,
				       stack_node,
				       leaf_mask);


	      /* return if stack is empty */
	      if (unlikely(curNode == BVH4Hair::invalidNode)) break;

	      STAT3(shadow.trav_leaves,1,1,1);

	      /* intersect one ray against four triangles */

	      //////////////////////////////////////////////////////////////////////////////////////////////////
	      BVH4i::NodeRef curNode4i = (unsigned int)curNode;

	      const bool hit = LeafIntersector::occluded(curNode4i,
							 rayIndex,
							 dir_xyz,
							 org_xyz,
							 min_dist_xyz,
							 max_dist_xyz,
							 ray16,
							 terminated,
							 accel,
							 (Scene*)bvh->geometry,
							 pre);

	      if (unlikely(hit)) break;
	      //////////////////////////////////////////////////////////////////////////////////////////////////

	    }


	  if (unlikely(all(toMask(terminated)))) break;
	}


      store16i(m_valid & toMask(terminated),&ray16.geomID,0);
    }


#else

    template<typename LeafIntersector>    
    void BVH4HairIntersector16<LeafIntersector>::intersect(mic_i* valid_i, BVH4Hair* bvh, Ray16& ray16)
    {
      /* near and node stack */
      __aligned(64) float   stack_dist[3*BVH4i::maxDepth+1];
      __aligned(64) BVH4i::NodeRef stack_node[3*BVH4i::maxDepth+1];

      LinearSpace_mic3f ray16_space = frame(ray16.dir).transposed();
      /* setup */
      const mic_m m_valid    = *(mic_i*)valid_i != mic_i(0);
      const mic3f rdir16     = rcp_safe(ray16.dir);
      const mic_f inf        = mic_f(pos_inf);
      const mic_f zero       = mic_f::zero();

      store16f(stack_dist,inf);

      const BVH4i::Node * __restrict__ nodes = (BVH4i::Node*)bvh->nodePtr();
      const void * __restrict__ accel = (void*)bvh->triPtr();


      stack_node[0] = BVH4i::invalidNode;
      long rayIndex = -1;
      while((rayIndex = bitscan64(rayIndex,toInt(m_valid))) != BITSCAN_NO_BIT_SET_64)	    
        {
	  Bezier1iIntersector16::Precalculations pre(ray16_space,rayIndex);
	  
	  stack_node[1] = bvh->root;
	  size_t sindex = 2;

	  const mic_f org_xyz      = loadAOS4to16f(rayIndex,ray16.org.x,ray16.org.y,ray16.org.z);
	  const mic_f dir_xyz      = loadAOS4to16f(rayIndex,ray16.dir.x,ray16.dir.y,ray16.dir.z);
	  const mic_f rdir_xyz     = loadAOS4to16f(rayIndex,rdir16.x,rdir16.y,rdir16.z);
	  const mic_f org_rdir_xyz = org_xyz * rdir_xyz;
	  const mic_f min_dist_xyz = broadcast1to16f(&ray16.tnear[rayIndex]);
	  mic_f       max_dist_xyz = broadcast1to16f(&ray16.tfar[rayIndex]);

	  const unsigned int leaf_mask = BVH4I_LEAF_MASK;

	  while (1)
	    {

	      BVH4i::NodeRef curNode = stack_node[sindex-1];
	      sindex--;

	      traverse_single_intersect(curNode,
					sindex,
					rdir_xyz,
					org_rdir_xyz,
					min_dist_xyz,
					max_dist_xyz,
					stack_node,
					stack_dist,
					nodes,
					leaf_mask);
		   


	      /* return if stack is empty */
	      if (unlikely(curNode == BVH4i::invalidNode)) break;

	      STAT3(normal.trav_leaves,1,1,1);

	      /* intersect one ray against four triangles */

	      //////////////////////////////////////////////////////////////////////////////////////////////////

	      const bool hit = LeafIntersector::intersect(curNode,
							  rayIndex,
							  dir_xyz,
							  org_xyz,
							  min_dist_xyz,
							  max_dist_xyz,
							  ray16,
							  accel,
							  (Scene*)bvh->geometry,
							  pre);
									   
	      if (hit)
		compactStack(stack_node,stack_dist,sindex,max_dist_xyz);

	      // ------------------------
	    }
	}
    }

    template<typename LeafIntersector>    
    void BVH4HairIntersector16<LeafIntersector>::occluded(mic_i* valid_i, BVH4Hair* bvh, Ray16& ray16)
    {
      /* near and node stack */
      __aligned(64) BVH4i::NodeRef stack_node[3*BVH4i::maxDepth+1];

      /* setup */
      const mic_m m_valid = *(mic_i*)valid_i != mic_i(0);
      const mic3f rdir16  = rcp_safe(ray16.dir);
      mic_m terminated    = !m_valid;
      const mic_f inf     = mic_f(pos_inf);
      const mic_f zero    = mic_f::zero();

      const BVH4i::Node * __restrict__ nodes = (BVH4i::Node*)bvh->nodePtr();
      const void * __restrict__ accel = (void*)bvh->triPtr();

      stack_node[0] = BVH4i::invalidNode;

      long rayIndex = -1;
      while((rayIndex = bitscan64(rayIndex,toInt(m_valid))) != BITSCAN_NO_BIT_SET_64)	    
        {
	  Bezier1iIntersector16::Precalculations pre(ray16,rayIndex);

	  stack_node[1] = bvh->root;
	  size_t sindex = 2;

	  const mic_f org_xyz      = loadAOS4to16f(rayIndex,ray16.org.x,ray16.org.y,ray16.org.z);
	  const mic_f dir_xyz      = loadAOS4to16f(rayIndex,ray16.dir.x,ray16.dir.y,ray16.dir.z);
	  const mic_f rdir_xyz     = loadAOS4to16f(rayIndex,rdir16.x,rdir16.y,rdir16.z);
	  const mic_f org_rdir_xyz = org_xyz * rdir_xyz;
	  const mic_f min_dist_xyz = broadcast1to16f(&ray16.tnear[rayIndex]);
	  const mic_f max_dist_xyz = broadcast1to16f(&ray16.tfar[rayIndex]);
	  const mic_i v_invalidNode(BVH4i::invalidNode);
	  const unsigned int leaf_mask = BVH4I_LEAF_MASK;

	  while (1)
	    {
	      BVH4i::NodeRef curNode = stack_node[sindex-1];
	      sindex--;

	      traverse_single_occluded(curNode,
				       sindex,
				       rdir_xyz,
				       org_rdir_xyz,
				       min_dist_xyz,
				       max_dist_xyz,
				       stack_node,
				       nodes,
				       leaf_mask);

	      /* return if stack is empty */
	      if (unlikely(curNode == BVH4i::invalidNode)) break;

	      STAT3(shadow.trav_leaves,1,1,1);

	      /* intersect one ray against four triangles */

	      //////////////////////////////////////////////////////////////////////////////////////////////////

	      const bool hit = LeafIntersector::occluded(curNode,
							 rayIndex,
							 dir_xyz,
							 org_xyz,
							 min_dist_xyz,
							 max_dist_xyz,
							 ray16,
							 terminated,
							 accel,
							 (Scene*)bvh->geometry,
							 pre);

	      if (unlikely(hit)) break;
	      //////////////////////////////////////////////////////////////////////////////////////////////////

	    }


	  if (unlikely(all(toMask(terminated)))) break;
	}


      store16i(m_valid & toMask(terminated),&ray16.geomID,0);
    }

#endif
    

    template<typename LeafIntersector>        
    void BVH4HairIntersector1<LeafIntersector>::intersect(BVH4Hair* bvh, Ray& ray)
    {

      /* near and node stack */
      __aligned(64) float   stack_dist[3*BVH4i::maxDepth+1];
      __aligned(64) BVH4Hair::NodeRef stack_node[3*BVH4i::maxDepth+1];

      LinearSpace_mic3f ray_space = frame(mic3f(ray.dir)).transposed();

      /* setup */
      const mic_f inf        = mic_f(pos_inf);
      const mic_f zero       = mic_f::zero();

      store16f(stack_dist,inf);

      const void * __restrict__ accel = (void*)bvh->triPtr();

      stack_node[0] = BVH4Hair::invalidNode;

      Bezier1iIntersector16::Precalculations pre(ray_space,0);
	  
      stack_node[1] = bvh->unaligned_nodes->child(0);
      size_t sindex = 2;

      const mic_f org_xyz      = loadAOS4to16f(ray.org.x,ray.org.y,ray.org.z);
      const mic_f dir_xyz      = loadAOS4to16f(ray.dir.x,ray.dir.y,ray.dir.z);
      const mic_f min_dist_xyz = broadcast1to16f(&ray.tnear);
      mic_f       max_dist_xyz = broadcast1to16f(&ray.tfar);

      const mic_f org_xyz1     = select(0x7777,org_xyz,mic_f::one());

      const size_t leaf_mask = BVH4HAIR_LEAF_MASK;

      while (1)
	{

	  BVH4Hair::NodeRef curNode = stack_node[sindex-1];
	  sindex--;

	  traverse_single_intersect(curNode,
				    sindex,
				    dir_xyz,
				    org_xyz1,
				    min_dist_xyz,
				    max_dist_xyz,
				    stack_node,
				    stack_dist,
				    leaf_mask);

	  /* return if stack is empty */
	  if (unlikely(curNode == BVH4Hair::invalidNode)) break;

	  STAT3(normal.trav_leaves,1,1,1);
	  STAT3(normal.trav_prims,4,4,4);

	  /* intersect one ray against four triangles */

	  //////////////////////////////////////////////////////////////////////////////////////////////////
	  BVH4i::NodeRef curNode4i = (unsigned int)curNode;
	  const bool hit = LeafIntersector::intersect(curNode4i,
						      dir_xyz,
						      org_xyz,
						      min_dist_xyz,
						      max_dist_xyz,
						      ray,
						      accel,
						      (Scene*)bvh->geometry,
						      pre);
									   
	  if (hit) 
	    {
	      const unsigned int current_dist = *(unsigned int*)&ray.tfar;
	      compactStack(stack_node,stack_dist,sindex,current_dist,max_dist_xyz);
	    }
	  // ------------------------
	}	         
    }

    template<typename LeafIntersector>    
    void BVH4HairIntersector1<LeafIntersector>::occluded(BVH4Hair* bvh, Ray& ray)
    {
      // FIXME
      FATAL("HERE");
    }
    
    
    DEFINE_INTERSECTOR16   (BVH4HairIntersector16Bezier1i, BVH4HairIntersector16<Bezier1iLeafIntersector>);
    DEFINE_INTERSECTOR1    (BVH4HairIntersector1Bezier1i , BVH4HairIntersector1<Bezier1iLeafIntersector>);

  }
}
