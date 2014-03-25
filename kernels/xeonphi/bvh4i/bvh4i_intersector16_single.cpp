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

#include "bvh4i_intersector16_single.h"
#include "geometry/triangle1.h"
#include "geometry/filter.h"

//#define USE_QUANTIZATION


namespace embree
{
  namespace isa
  {

    static unsigned int BVH4I_LEAF_MASK = BVH4i::leaf_mask; // needed due to compiler efficiency bug
    static unsigned int M_LANE_7777 = 0x7777; // needed due to compiler efficiency bug

    static __aligned(64) int zlc4[4] = {0xffffffff,0xffffffff,0xffffffff,0};

    void BVH4iIntersector16Single::intersect(mic_i* valid_i, BVH4i* bvh, Ray16& ray16)
    {
      /* near and node stack */
      __aligned(64) float   stack_dist[3*BVH4i::maxDepth+1];
      __aligned(64) NodeRef stack_node[3*BVH4i::maxDepth+1];

      /* setup */
      const mic_m m_valid    = *(mic_i*)valid_i != mic_i(0);
      const mic3f rdir16     = rcp_safe(ray16.dir);
      const mic_f inf        = mic_f(pos_inf);
      const mic_f zero       = mic_f::zero();

      store16f(stack_dist,inf);

      const Node      * __restrict__ nodes = (Node     *)bvh->nodePtr();
      const Triangle1 * __restrict__ accel = (Triangle1*)bvh->triPtr();

      stack_node[0] = BVH4i::invalidNode;
      long rayIndex = -1;
      while((rayIndex = bitscan64(rayIndex,toInt(m_valid))) != BITSCAN_NO_BIT_SET_64)	    
        {
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

	      STAT3(normal.trav_leaves,1,1,1);
	      STAT3(normal.trav_prims,4,4,4);

	      /* intersect one ray against four triangles */

	      //////////////////////////////////////////////////////////////////////////////////////////////////

	      const Triangle1* tptr  = (Triangle1*) curNode.leaf(accel);

	      
	      prefetch<PFHINT_L1>(tptr + 3);
	      prefetch<PFHINT_L1>(tptr + 2);
	      prefetch<PFHINT_L1>(tptr + 1);
	      prefetch<PFHINT_L1>(tptr + 0); 

	      const mic_i and_mask = broadcast4to16i(zlc4);
	      
	      const mic_f v0 = gather_4f_zlc(and_mask,
					     (float*)&tptr[0].v0,
					     (float*)&tptr[1].v0,
					     (float*)&tptr[2].v0,
					     (float*)&tptr[3].v0);
	      
	      const mic_f v1 = gather_4f_zlc(and_mask,
					     (float*)&tptr[0].v1,
					     (float*)&tptr[1].v1,
					     (float*)&tptr[2].v1,
					     (float*)&tptr[3].v1);
	      
	      const mic_f v2 = gather_4f_zlc(and_mask,
					     (float*)&tptr[0].v2,
					     (float*)&tptr[1].v2,
					     (float*)&tptr[2].v2,
					     (float*)&tptr[3].v2);

	      const mic_f e1 = v1 - v0;
	      const mic_f e2 = v0 - v2;	     
	      const mic_f normal = lcross_zxy(e1,e2);
	      const mic_f org = v0 - org_xyz;
	      const mic_f odzxy = msubr231(org * swizzle(dir_xyz,_MM_SWIZ_REG_DACB), dir_xyz, swizzle(org,_MM_SWIZ_REG_DACB));
	      const mic_f den = ldot3_zxy(dir_xyz,normal);	      
	      const mic_f rcp_den = rcp(den);
	      const mic_f uu = ldot3_zxy(e2,odzxy); 
	      const mic_f vv = ldot3_zxy(e1,odzxy); 
	      const mic_f u = uu * rcp_den;
	      const mic_f v = vv * rcp_den;

#if defined(__BACKFACE_CULLING__)
	      const mic_m m_init = (mic_m)0x1111 & (den > zero);
#else
	      const mic_m m_init = 0x1111;
#endif

	      const mic_m valid_u = ge(m_init,u,zero);
	      const mic_m valid_v = ge(valid_u,v,zero);
	      const mic_m m_aperture = le(valid_v,u+v,mic_f::one()); 

	      const mic_f nom = ldot3_zxy(org,normal);

	      if (unlikely(none(m_aperture))) continue;
	      const mic_f t = rcp_den*nom;

	      mic_m m_final  = lt(lt(m_aperture,min_dist_xyz,t),t,max_dist_xyz);


#if defined(__USE_RAY_MASK__)
	      const mic_i rayMask(ray16.mask[rayIndex]);
	      const mic_i triMask = swDDDD(gather16i_4i_align(&tptr[0].v2,&tptr[1].v2,&tptr[2].v2,&tptr[3].v2));
	      const mic_m m_ray_mask = (rayMask & triMask) != mic_i::zero();
	      m_final &= m_ray_mask;	      
#endif

	      //////////////////////////////////////////////////////////////////////////////////////////////////

              /* did the ray hit one of the four triangles? */
	      if (unlikely(any(m_final)))
		{
		  /* intersection filter test */
#if defined(__INTERSECTION_FILTER__) 
		  mic_f org_max_dist_xyz = max_dist_xyz;

		  /* did the ray hit one of the four triangles? */
		  while (any(m_final)) 
		    {
		      max_dist_xyz  = select(m_final,t,org_max_dist_xyz);
		      const mic_f min_dist = vreduce_min(max_dist_xyz);
		      const mic_m m_dist = eq(min_dist,max_dist_xyz);
		      const size_t vecIndex = bitscan(toInt(m_dist));
		      const size_t triIndex = vecIndex >> 2;
		      const Triangle1  *__restrict__ tri_ptr = tptr + triIndex;
		      const mic_m m_tri = m_dist^(m_dist & (mic_m)((unsigned int)m_dist - 1));
		      const mic_f gnormalx = mic_f(tri_ptr->Ng.x);
		      const mic_f gnormaly = mic_f(tri_ptr->Ng.y);
		      const mic_f gnormalz = mic_f(tri_ptr->Ng.z);
		      const int geomID = tri_ptr->geomID();
		      const int primID = tri_ptr->primID();
                
		      Geometry* geom = ((Scene*)bvh->geometry)->get(geomID);
		      if (likely(!geom->hasIntersectionFilter16())) 
			{

			  compactustore16f_low(m_tri,&ray16.tfar[rayIndex],min_dist);
			  compactustore16f_low(m_tri,&ray16.u[rayIndex],u); 
			  compactustore16f_low(m_tri,&ray16.v[rayIndex],v); 
			  compactustore16f_low(m_tri,&ray16.Ng.x[rayIndex],gnormalx); 
			  compactustore16f_low(m_tri,&ray16.Ng.y[rayIndex],gnormaly); 
			  compactustore16f_low(m_tri,&ray16.Ng.z[rayIndex],gnormalz); 
			  ray16.geomID[rayIndex] = geomID;
			  ray16.primID[rayIndex] = primID;
			  max_dist_xyz = min_dist;
			  break;
			}
                
		      if (runIntersectionFilter16(geom,ray16,rayIndex,u,v,min_dist,gnormalx,gnormaly,gnormalz,m_tri,geomID,primID)) {
			max_dist_xyz = min_dist;
			break;
		      }
		      m_final ^= m_tri;
		    }
		  max_dist_xyz = ray16.tfar[rayIndex];
#else
		  STAT3(normal.trav_prim_hits,1,1,1);
                  max_dist_xyz  = select(m_final,t,max_dist_xyz);
		  const mic_f min_dist = vreduce_min(max_dist_xyz);
		  const mic_m m_dist = eq(min_dist,max_dist_xyz);
                  const size_t vecIndex = bitscan(toInt(m_dist));
		  const size_t triIndex = vecIndex >> 2;

		  const Triangle1  *__restrict__ tri_ptr = tptr + triIndex;

		  const mic_m m_tri = m_dist^(m_dist & (mic_m)((unsigned int)m_dist - 1));

		  const mic_f gnormalx = mic_f(tri_ptr->Ng.x);
		  const mic_f gnormaly = mic_f(tri_ptr->Ng.y);
		  const mic_f gnormalz = mic_f(tri_ptr->Ng.z);
                
		  prefetch<PFHINT_L1EX>(&ray16.tfar);  
		  prefetch<PFHINT_L1EX>(&ray16.u);
		  prefetch<PFHINT_L1EX>(&ray16.v);
		  prefetch<PFHINT_L1EX>(&ray16.Ng.x); 
		  prefetch<PFHINT_L1EX>(&ray16.Ng.y); 
		  prefetch<PFHINT_L1EX>(&ray16.Ng.z); 
		  prefetch<PFHINT_L1EX>(&ray16.geomID);
		  prefetch<PFHINT_L1EX>(&ray16.primID);

		  max_dist_xyz = min_dist;
		  
		  compactustore16f_low(m_tri,&ray16.tfar[rayIndex],min_dist);
		  compactustore16f_low(m_tri,&ray16.u[rayIndex],u); 
		  compactustore16f_low(m_tri,&ray16.v[rayIndex],v); 
		  compactustore16f_low(m_tri,&ray16.Ng.x[rayIndex],gnormalx); 
		  compactustore16f_low(m_tri,&ray16.Ng.y[rayIndex],gnormaly); 
		  compactustore16f_low(m_tri,&ray16.Ng.z[rayIndex],gnormalz); 

		  ray16.geomID[rayIndex] = tri_ptr->geomID();
		  ray16.primID[rayIndex] = tri_ptr->primID();
#endif
		  /* compact the stack if size if stack >= 2 */
		  compactStack(stack_node,stack_dist,sindex,max_dist_xyz);

		}
	      // ------------------------
	    }	  
	}
    }
    
    void BVH4iIntersector16Single::occluded(mic_i* valid_i, BVH4i* bvh, Ray16& ray16)
    {
      /* near and node stack */
      __aligned(64) NodeRef stack_node[3*BVH4i::maxDepth+1];

      /* setup */
      const mic_m m_valid     = *(mic_i*)valid_i != mic_i(0);
      const mic3f rdir16      = rcp_safe(ray16.dir);
      unsigned int terminated = toInt(!m_valid);
      const mic_f inf         = mic_f(pos_inf);
      const mic_f zero        = mic_f::zero();

      const Node      * __restrict__ nodes = (Node     *)bvh->nodePtr();
      const Triangle1 * __restrict__ accel = (Triangle1*)bvh->triPtr();

      stack_node[0] = BVH4i::invalidNode;

      long rayIndex = -1;
      while((rayIndex = bitscan64(rayIndex,toInt(m_valid))) != BITSCAN_NO_BIT_SET_64)	    
        {
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
	  const mic_m m7777 = 0x7777; 
	  const mic_m m_rdir0 = lt(m7777,rdir_xyz,mic_f::zero());
	  const mic_m m_rdir1 = ge(m7777,rdir_xyz,mic_f::zero());

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

	      STAT3(shadow.trav_leaves,1,1,1);
	      STAT3(shadow.trav_prims,4,4,4);

	      /* intersect one ray against four triangles */

	      //////////////////////////////////////////////////////////////////////////////////////////////////

	      const Triangle1* tptr  = (Triangle1*) curNode.leaf(accel);
	      prefetch<PFHINT_L1>(tptr + 3);
	      prefetch<PFHINT_L1>(tptr + 2);
	      prefetch<PFHINT_L1>(tptr + 1);
	      prefetch<PFHINT_L1>(tptr + 0); 

	      const mic_i and_mask = broadcast4to16i(zlc4);
	      
	      const mic_f v0 = gather_4f_zlc(and_mask,
					     (float*)&tptr[0].v0,
					     (float*)&tptr[1].v0,
					     (float*)&tptr[2].v0,
					     (float*)&tptr[3].v0);
	      
	      const mic_f v1 = gather_4f_zlc(and_mask,
					     (float*)&tptr[0].v1,
					     (float*)&tptr[1].v1,
					     (float*)&tptr[2].v1,
					     (float*)&tptr[3].v1);
	      
	      const mic_f v2 = gather_4f_zlc(and_mask,
					     (float*)&tptr[0].v2,
					     (float*)&tptr[1].v2,
					     (float*)&tptr[2].v2,
					     (float*)&tptr[3].v2);

	      const mic_f e1 = v1 - v0;
	      const mic_f e2 = v0 - v2;	     
	      const mic_f normal = lcross_zxy(e1,e2);
	      const mic_f org = v0 - org_xyz;
	      const mic_f odzxy = msubr231(org * swizzle(dir_xyz,_MM_SWIZ_REG_DACB), dir_xyz, swizzle(org,_MM_SWIZ_REG_DACB));
	      const mic_f den = ldot3_zxy(dir_xyz,normal);	      
	      const mic_f rcp_den = rcp(den);
	      const mic_f uu = ldot3_zxy(e2,odzxy); 
	      const mic_f vv = ldot3_zxy(e1,odzxy); 
	      const mic_f u = uu * rcp_den;
	      const mic_f v = vv * rcp_den;

#if defined(__BACKFACE_CULLING__)
	      const mic_m m_init = (mic_m)0x1111 & (den > zero);
#else
	      const mic_m m_init = 0x1111;
#endif

	      const mic_m valid_u = ge((mic_m)m_init,u,zero);
	      const mic_m valid_v = ge(valid_u,v,zero);
	      const mic_m m_aperture = le(valid_v,u+v,mic_f::one()); 

	      const mic_f nom = ldot3_zxy(org,normal);
	      const mic_f t = rcp_den*nom;
	      if (unlikely(none(m_aperture))) continue;

	      mic_m m_final  = lt(lt(m_aperture,min_dist_xyz,t),t,max_dist_xyz);

#if defined(__USE_RAY_MASK__)
	      const mic_i rayMask(ray16.mask[rayIndex]);
	      const mic_i triMask = swDDDD(gather16i_4i_align(&tptr[0].v2,&tptr[1].v2,&tptr[2].v2,&tptr[3].v2));
	      const mic_m m_ray_mask = (rayMask & triMask) != mic_i::zero();
	      m_final &= m_ray_mask;	      
#endif

#if defined(__INTERSECTION_FILTER__) 
              
              /* did the ray hit one of the four triangles? */
              while (any(m_final)) 
		{
		  const mic_f temp_t  = select(m_final,t,max_dist_xyz);
		  const mic_f min_dist = vreduce_min(temp_t);
		  const mic_m m_dist = eq(min_dist,temp_t);
		  const size_t vecIndex = bitscan(toInt(m_dist));
		  const size_t triIndex = vecIndex >> 2;
		  const Triangle1  *__restrict__ tri_ptr = tptr + triIndex;
		  const mic_m m_tri = m_dist^(m_dist & (mic_m)((unsigned int)m_dist - 1));
		  const mic_f gnormalx = mic_f(tri_ptr->Ng.x);
		  const mic_f gnormaly = mic_f(tri_ptr->Ng.y);
		  const mic_f gnormalz = mic_f(tri_ptr->Ng.z);
		  const int geomID = tri_ptr->geomID();
		  const int primID = tri_ptr->primID();                
		  Geometry* geom = ((Scene*)bvh->geometry)->get(geomID);
		  if (likely(!geom->hasOcclusionFilter16())) break;
                
		  if (runOcclusionFilter16(geom,ray16,rayIndex,u,v,min_dist,gnormalx,gnormaly,gnormalz,m_tri,geomID,primID)) 
		    break;

		  m_final ^= m_tri; /* clear bit */
		}
#endif
	      if (unlikely(any(m_final)))
		{
		  STAT3(shadow.trav_prim_hits,1,1,1);
		  terminated |= mic_m::shift1[rayIndex];
		  break;
		}
	      //////////////////////////////////////////////////////////////////////////////////////////////////

	    }


	  if (unlikely(all(toMask(terminated)))) break;
	}


      store16i(m_valid & toMask(terminated),&ray16.geomID,0);
    }
    
    DEFINE_INTERSECTOR16    (BVH4iTriangle1Intersector16SingleMoeller, BVH4iIntersector16Single);

  }
}
