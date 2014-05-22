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
#include "bvh4hair_intersector16_single.h"
#include "geometry/bezier1i.h"
#include "geometry/bezier1i_intersector16.h"

#define DBG(x) 

namespace embree
{
  namespace isa
  {
    static unsigned int BVH4I_LEAF_MASK = BVH4i::leaf_mask; // needed due to compiler efficiency bug


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
	prefetch<PFHINT_L1>(tptr + 2);

	// for (size_t i=0;i<items;i++)
	//    tptr[i].prefetchControlPoints<PFHINT_L2>();

	for (size_t i=0;i<items;i++)
	  ret |= Bezier1iIntersector16::intersect(pre,ray16,dir_xyz,org_xyz,rayIndex,tptr[i],geometry); // add mailboxing

	max_dist_xyz = ray16.tfar[rayIndex];
	//if (unlikely(pre.mbox.hit(curves[i].geomID,curves[i].primID))) continue;
	//pre.mbox.add(curves[i].geomID,curves[i].primID);

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
	prefetch<PFHINT_L1>(tptr + 2);

	for (size_t i=0;i<items;i++)
	  if (Bezier1iIntersector16::occluded(pre,ray16,dir_xyz,org_xyz,rayIndex,tptr[i],geometry))
	    return true;
	return false;
      }

      static __forceinline bool intersect(BVH4i::NodeRef curNode,
					  const mic_f &dir_xyz,
					  const mic_f &org_xyz,
					  const mic_f &min_dist_xyz,
					  mic_f &max_dist_xyz,
					  Ray& ray, 
					  const void *__restrict__ const accel,
					  const Scene*__restrict__ const geometry)
      {
	unsigned int items = curNode.items();
	unsigned int index = curNode.offsetIndex(); /* array of AccelSetItems */
	Bezier1i *accel_ptr = (Bezier1i*)accel + index;
	int old_primID = ray.primID;

	return old_primID != ray.primID;
      }

      static __forceinline bool occluded(BVH4i::NodeRef curNode,
					 const mic_f &dir_xyz,
					 const mic_f &org_xyz,
					 const mic_f &min_dist_xyz,
					 const mic_f &max_dist_xyz,
					 Ray& ray,
					 const void *__restrict__ const accel,
					 const Scene*__restrict__ const geometry)
      {
	unsigned int items = curNode.items();
	unsigned int index = curNode.offsetIndex(); /* array of AccelSetItems */
	Bezier1i *accel_ptr = (Bezier1i*)accel + index;

	return false;
      }

    };


    static __forceinline mic_f xfm(const mic_f &v, const BVH4Hair::UnalignedNode &node)
    {
      const mic_f x = ldot3_xyz(v,node.matrixColumnXYZW[0]);
      const mic_f y = ldot3_xyz(v,node.matrixColumnXYZW[1]);
      const mic_f z = ldot3_xyz(v,node.matrixColumnXYZW[2]);
      const mic_f ret = select(0x8888,mic_f::zero(),select(0x4444,z,select(0x2222,y,x)));
      return ret;
    }

#if 0

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

	  const size_t leaf_mask = BVH4Hair::leaf_mask;

	  while (1)
	    {

	      BVH4Hair::NodeRef curNode = stack_node[sindex-1];
	      sindex--;

	      const mic_m m7777 = 0x7777; 

	      while (1) 
		{
		  if (unlikely(curNode.isLeaf(leaf_mask))) break;
		  
		  STAT3(normal.trav_nodes,1,1,1);
		  const BVH4Hair::UnalignedNode *__restrict__ const u_node = (BVH4Hair::UnalignedNode *)curNode.node();

		  prefetch<PFHINT_L1>((char*)u_node + 0*64);
		  prefetch<PFHINT_L1>((char*)u_node + 1*64);
		  prefetch<PFHINT_L1>((char*)u_node + 2*64);
		  prefetch<PFHINT_L1>((char*)u_node + 3*64);


		  const mic_f xfm_org_xyz = xfm(org_xyz1,*u_node);
		  const mic_f xfm_dir_xyz = xfm(dir_xyz ,*u_node);

		  
		  const mic_f rcp_xfm_dir_xyz = rcp_safe( xfm_dir_xyz );
		  		  
		  mic_f tLowerXYZ = (mic_f::zero() - xfm_org_xyz) * rcp_xfm_dir_xyz;
		  mic_f tUpperXYZ = (mic_f::one()  - xfm_org_xyz) * rcp_xfm_dir_xyz;

		    
		  mic_m hitm = eq(0x1111, xfm_org_xyz,xfm_org_xyz);

		  const mic_f tLower = select(m7777,min(tLowerXYZ,tUpperXYZ),min_dist_xyz);
		  const mic_f tUpper = select(m7777,max(tLowerXYZ,tUpperXYZ),max_dist_xyz);


		  /* early pop of next node */
		  sindex--;
		  curNode = stack_node[sindex];


		  const mic_f tNear = vreduce_max4(tLower);
		  const mic_f tFar  = vreduce_min4(tUpper);  


		  hitm = le(hitm,tNear,tFar);

		  DBG(
		      DBG_PRINT(*u_node);

		      DBG_PRINT(org_xyz);
		      DBG_PRINT(org_xyz1);
		      DBG_PRINT(dir_xyz);
		      DBG_PRINT( xfm_org_xyz );
		      DBG_PRINT( xfm_dir_xyz );
		      DBG_PRINT(tLowerXYZ);
		      DBG_PRINT(tUpperXYZ);
		      DBG_PRINT(tLower);
		      DBG_PRINT(tUpper);
		      DBG_PRINT(tNear);
		      DBG_PRINT(tFar);
		      DBG_PRINT(hitm);
		      );

		  const mic_f tNear_pos = select(hitm,tNear,inf);

		  STAT3(normal.trav_hit_boxes[countbits(hitm)],1,1,1);


		  /* if no child is hit, continue with early popped child */
		  if (unlikely(none(hitm))) continue;

		  
		  sindex++;        
		  const unsigned long hiti = toInt(hitm);
		  const unsigned long pos_first = bitscan64(hiti);
		  const unsigned long num_hitm = countbits(hiti); 
        
		  /* if a single child is hit, continue with that child */
		  curNode = u_node->child(pos_first>>2);
		  assert(curNode != BVH4Hair::emptyNode);

		  if (likely(num_hitm == 1)) continue;
        
		  /* if two children are hit, push in correct order */
		  const unsigned long pos_second = bitscan64(pos_first,hiti);
		  if (likely(num_hitm == 2))
		    {
		      const unsigned int dist_first  = ((unsigned int*)&tNear)[pos_first];
		      const unsigned int dist_second = ((unsigned int*)&tNear)[pos_second];
		      const BVH4Hair::NodeRef node_first  = curNode;
		      const BVH4Hair::NodeRef node_second = u_node->child(pos_second>>2);

		      assert(node_first  != BVH4Hair::emptyNode);
		      assert(node_second != BVH4Hair::emptyNode);
          
		      if (dist_first <= dist_second)
			{
			  
			  stack_node[sindex] = node_second;
			  ((unsigned int*)stack_dist)[sindex] = dist_second;                      
			  sindex++;
			  assert(sindex < 3*BVH4Hair::maxDepth+1);
			  continue;
			}
		      else
			{
			  stack_node[sindex] = node_first;
			  ((unsigned int*)stack_dist)[sindex] = dist_first;
			  curNode = node_second;
			  sindex++;
			  assert(sindex < 3*BVH4Hair::maxDepth+1);
			  continue;
			}
		    }

		  /* continue with closest child and push all others */


		  const mic_f min_dist = set_min_lanes(tNear_pos);
		  //const unsigned int old_sindex = sindex;
		  //sindex += countbits(hiti) - 1;
		  assert(sindex < 3*BVH4i::maxDepth+1);
        
		  const mic_m closest_child = eq(hitm,min_dist,tNear);
		  const unsigned long closest_child_pos = bitscan64(closest_child);
		  const mic_m m_pos = andn(hitm,andn(closest_child,(mic_m)((unsigned int)closest_child - 1)));
		  curNode = u_node->child(closest_child_pos>>2);

		  assert(curNode  != BVH4Hair::emptyNode);

		  long i = -1;
		  while((i = bitscan64(i,m_pos)) != BITSCAN_NO_BIT_SET_64)	    
		    {
		      ((unsigned int*)stack_dist)[sindex] = ((unsigned int*)&tNear)[i];		      
		      stack_node[sindex] = u_node->child(i>>2);
		      assert(stack_node[sindex]  != BVH4Hair::emptyNode);
		      sindex++;
		    }
		}







	      /* return if stack is empty */
	      if (unlikely(curNode == BVH4Hair::invalidNode)) break;

	      STAT3(normal.trav_leaves,1,1,1);
	      STAT3(normal.trav_prims,4,4,4);

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
									   
	      //if (hit) compactStack(stack_node,stack_dist,sindex,max_dist_xyz);

	      // ------------------------
	    }	  
	}
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
	      STAT3(normal.trav_prims,4,4,4);

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
#endif
    
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
	      STAT3(shadow.trav_prims,4,4,4);

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

    template<typename LeafIntersector>        
    void BVH4HairIntersector1<LeafIntersector>::intersect(BVH4i* bvh, Ray& ray)
    {
      /* near and node stack */
      __aligned(64) float   stack_dist[3*BVH4i::maxDepth+1];
      __aligned(64) NodeRef stack_node[3*BVH4i::maxDepth+1];

      if (unlikely(bvh == NULL)) return;

      /* setup */
      const mic3f rdir16     = rcp_safe(mic3f(mic_f(ray.dir.x),mic_f(ray.dir.y),mic_f(ray.dir.z)));
      const mic_f inf        = mic_f(pos_inf);
      const mic_f zero       = mic_f::zero();

      store16f(stack_dist,inf);

      const Node      * __restrict__ nodes = (Node    *)bvh->nodePtr();
      const void * __restrict__ accel = (void*)bvh->triPtr();

      if (unlikely(accel == NULL)) return;

      stack_node[0] = BVH4i::invalidNode;      
      stack_node[1] = bvh->root;

      size_t sindex = 2;

      const mic_f org_xyz      = loadAOS4to16f(ray.org.x,ray.org.y,ray.org.z);
      const mic_f dir_xyz      = loadAOS4to16f(ray.dir.x,ray.dir.y,ray.dir.z);
      const mic_f rdir_xyz     = loadAOS4to16f(rdir16.x[0],rdir16.y[0],rdir16.z[0]);
      const mic_f org_rdir_xyz = org_xyz * rdir_xyz;
      const mic_f min_dist_xyz = broadcast1to16f(&ray.tnear);
      mic_f       max_dist_xyz = broadcast1to16f(&ray.tfar);
	  
      const unsigned int leaf_mask = BVH4I_LEAF_MASK;
	 
      while (1)
	{
	  NodeRef curNode = stack_node[sindex-1];
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

	  //////////////////////////////////////////////////////////////////////////////////////////////////

	  bool hit = LeafIntersector::intersect(curNode,
						dir_xyz,
						org_xyz,
						min_dist_xyz,
						max_dist_xyz,
						ray,
						accel,
						(Scene*)bvh->geometry);
	  if (hit)
	    compactStack(stack_node,stack_dist,sindex,max_dist_xyz);

	  //////////////////////////////////////////////////////////////////////////////////////////////////

	}	  
    }

    template<typename LeafIntersector>    
    void BVH4HairIntersector1<LeafIntersector>::occluded(BVH4i* bvh, Ray& ray)
    {
      /* near and node stack */
      __aligned(64) NodeRef stack_node[3*BVH4i::maxDepth+1];

      if (unlikely(bvh == NULL)) return;

      /* setup */
      const mic3f rdir16      = rcp_safe(mic3f(ray.dir.x,ray.dir.y,ray.dir.z));
      const mic_f inf         = mic_f(pos_inf);
      const mic_f zero        = mic_f::zero();

      const Node      * __restrict__ nodes = (Node     *)bvh->nodePtr();
      const void * __restrict__ accel = (void*)bvh->triPtr();

      if (unlikely(accel == NULL)) return;

      stack_node[0] = BVH4i::invalidNode;
      stack_node[1] = bvh->root;
      size_t sindex = 2;

      const mic_f org_xyz      = loadAOS4to16f(ray.org.x,ray.org.y,ray.org.z);
      const mic_f dir_xyz      = loadAOS4to16f(ray.dir.x,ray.dir.y,ray.dir.z);
      const mic_f rdir_xyz     = loadAOS4to16f(rdir16.x[0],rdir16.y[0],rdir16.z[0]);
      const mic_f org_rdir_xyz = org_xyz * rdir_xyz;
      const mic_f min_dist_xyz = broadcast1to16f(&ray.tnear);
      const mic_f max_dist_xyz = broadcast1to16f(&ray.tfar);

      const unsigned int leaf_mask = BVH4I_LEAF_MASK;
	  
      while (1)
	{
	  NodeRef curNode = stack_node[sindex-1];
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



	  //////////////////////////////////////////////////////////////////////////////////////////////////
	  bool hit = LeafIntersector::occluded(curNode,
					       dir_xyz,
					       org_xyz,
					       min_dist_xyz,
					       max_dist_xyz,
					       ray,
					       accel,
					       (Scene*)bvh->geometry);

	  if (unlikely(hit))
	    {
	      ray.geomID = 0;
	      return;
	    }

	  //////////////////////////////////////////////////////////////////////////////////////////////////

	}
      
    }
    
    
    DEFINE_INTERSECTOR16   (BVH4HairIntersector16Bezier1i, BVH4HairIntersector16<Bezier1iLeafIntersector>);
    DEFINE_INTERSECTOR1    (BVH4HairIntersector1Bezier1i , BVH4HairIntersector1<Bezier1iLeafIntersector>);

  }
}
