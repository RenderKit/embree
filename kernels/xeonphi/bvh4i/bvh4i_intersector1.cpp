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

#include "bvh4i_intersector1.h"

#include "geometry/triangle1_intersector16_moeller.h"
#include "geometry/virtual_accel_intersector1.h"

namespace embree
{
  namespace isa
  {
    static unsigned int BVH4I_LEAF_MASK = BVH4i::leaf_mask; // needed due to compiler efficiency bug

    static __aligned(64) int zlc4[4] = {0xffffffff,0xffffffff,0xffffffff,0};
    
    void BVH4iIntersector1::intersect(BVH4i* bvh, Ray& ray)
    {
      /* near and node stack */
      __aligned(64) float   stack_dist[3*BVH4i::maxDepth+1];
      __aligned(64) NodeRef stack_node[3*BVH4i::maxDepth+1];

      /* setup */
      //const mic_m m_valid    = *(mic_i*)valid_i != mic_i(0);
      const mic3f rdir16     = rcp_safe(mic3f(mic_f(ray.dir.x),mic_f(ray.dir.y),mic_f(ray.dir.z)));
      const mic_f inf        = mic_f(pos_inf);
      const mic_f zero       = mic_f::zero();

      store16f(stack_dist,inf);

      const Node      * __restrict__ nodes = (Node    *)bvh->nodePtr();
      const Triangle1 * __restrict__ accel = (Triangle1*)bvh->triPtr();

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
	  const mic_i rayMask(ray.mask);
	  const mic_i triMask = swDDDD(gather16i_4i_align(&tptr[0].v2,&tptr[1].v2,&tptr[2].v2,&tptr[3].v2));
	  const mic_m m_ray_mask = (rayMask & triMask) != mic_i::zero();
	  m_final &= m_ray_mask;	      
#endif


              //////////////////////////////////////////////////////////////////////////////////////////////////

	  /* did the ray hot one of the four triangles? */
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

		  // if ( (tri_ptr->mask() & ray.mask) == 0 ) {
		  //   m_final ^= m_tri;
		  //   continue;
		  // }
                
		  Geometry* geom = ((Scene*)bvh->geometry)->get(geomID);
		  if (likely(!geom->hasIntersectionFilter1())) 
		    {
		      compactustore16f_low(m_tri,&ray.tfar,min_dist);
		      compactustore16f_low(m_tri,&ray.u,u); 
		      compactustore16f_low(m_tri,&ray.v,v); 
		      compactustore16f_low(m_tri,&ray.Ng.x,gnormalx); 
		      compactustore16f_low(m_tri,&ray.Ng.y,gnormaly); 
		      compactustore16f_low(m_tri,&ray.Ng.z,gnormalz); 
		      ray.geomID = geomID;
		      ray.primID = primID;
		      max_dist_xyz = min_dist;
		      break;
		    }
                
		  if (runIntersectionFilter1(geom,ray,u,v,min_dist,gnormalx,gnormaly,gnormalz,m_tri,geomID,primID)) {
		    max_dist_xyz = min_dist;
		    break;
		  }
		  m_final ^= m_tri;
		}
	      max_dist_xyz = ray.tfar;
#else
              max_dist_xyz  = select(m_final,t,max_dist_xyz);
	      const mic_f min_dist = vreduce_min(max_dist_xyz);
	      const mic_m m_dist = eq(min_dist,max_dist_xyz);

	      prefetch<PFHINT_L1EX>((mic_f*)&ray + 0);
	      prefetch<PFHINT_L1EX>((mic_f*)&ray + 1);

	      const size_t vecIndex = bitscan(toInt(m_dist));
	      const size_t triIndex = vecIndex >> 2;

	      const Triangle1  *__restrict__ tri_ptr = tptr + triIndex;

	      const mic_m m_tri = m_dist^(m_dist & (mic_m)((unsigned int)m_dist - 1));

	      const mic_f gnormalx = mic_f(tri_ptr->Ng.x);
	      const mic_f gnormaly = mic_f(tri_ptr->Ng.y);
	      const mic_f gnormalz = mic_f(tri_ptr->Ng.z);

	      max_dist_xyz = min_dist;

	      compactustore16f_low(m_tri,&ray.tfar,min_dist);
	      compactustore16f_low(m_tri,&ray.u,u); 
	      compactustore16f_low(m_tri,&ray.v,v); 
	      compactustore16f_low(m_tri,&ray.Ng.x,gnormalx); 
	      compactustore16f_low(m_tri,&ray.Ng.y,gnormaly); 
	      compactustore16f_low(m_tri,&ray.Ng.z,gnormalz); 

	      ray.geomID = tri_ptr->geomID();
	      ray.primID = tri_ptr->primID();
#endif
	      /* compact the stack if size of stack >= 2 */
	      compactStack(stack_node,stack_dist,sindex,max_dist_xyz);

	    }
	}	  
    }


    void BVH4iIntersector1::occluded(BVH4i* bvh, Ray& ray)
    {
      /* near and node stack */
      __aligned(64) NodeRef stack_node[3*BVH4i::maxDepth+1];

      /* setup */
      const mic3f rdir16      = rcp_safe(mic3f(ray.dir.x,ray.dir.y,ray.dir.z));
      const mic_f inf         = mic_f(pos_inf);
      const mic_f zero        = mic_f::zero();

      const Node      * __restrict__ nodes = (Node     *)bvh->nodePtr();
      const Triangle1 * __restrict__ accel = (Triangle1*)bvh->triPtr();

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
	  const mic_f t = rcp_den*nom;

	  if (unlikely(none(m_aperture))) continue;

	  mic_m m_final  = lt(lt(m_aperture,min_dist_xyz,t),t,max_dist_xyz);

#if defined(__USE_RAY_MASK__)
	  const mic_i rayMask(ray.mask);
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

	      if (likely(!geom->hasOcclusionFilter1())) break;
                
	      if (runOcclusionFilter1(geom,ray,u,v,min_dist,gnormalx,gnormaly,gnormalz,m_tri,geomID,primID)) 
		break;

	      m_final ^= m_tri; /* clear bit */
	    }
#endif

	  if (unlikely(any(m_final)))
	    {
	      ray.geomID = 0;
	      return;
	    }
	  //////////////////////////////////////////////////////////////////////////////////////////////////

	}
    }


    DEFINE_INTERSECTOR1    (BVH4iTriangle1Intersector1, BVH4iIntersector1);
    DEFINE_INTERSECTOR1    (BVH4iVirtualIntersector1, BVH4iIntersector1);

  }
}
