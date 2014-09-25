// ======================================================================== //
// Copyright 2009-2014 Intel Corporation                                    //
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

#pragma once

#include "subdivpatch1.h"
#include "common/ray16.h"
#include "geometry/filter.h"

namespace embree
{
  static __aligned(64) int _zlc4[4] = {0xffffffff,0xffffffff,0xffffffff,0};


  static __forceinline bool intersect1_quad(const size_t rayIndex, 
					    const mic_f &dir_xyz,
					    const mic_f &org_xyz,
					    Ray16& ray16,
					    const FinalQuad &quad)
  {
    const mic_i and_mask = broadcast4to16i(_zlc4);

    const mic_f v0 = gather_4f_zlc(and_mask,
				   &quad.vtx[0],
				   &quad.vtx[0],
				   &quad.vtx[2],
				   &quad.vtx[2]);
	      
    const mic_f v1 = gather_4f_zlc(and_mask,
				   &quad.vtx[1],
				   &quad.vtx[1],
				   &quad.vtx[3],
				   &quad.vtx[3]);
	      
    const mic_f v2 = gather_4f_zlc(and_mask,
				   &quad.vtx[2],
				   &quad.vtx[2],
				   &quad.vtx[0],
				   &quad.vtx[0]);


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

    if (unlikely(none(m_aperture))) return false;
    const mic_f t = rcp_den*nom;
    mic_f max_dist_xyz = mic_f(ray16.tfar[rayIndex]);
    mic_m m_final  = lt(lt(m_aperture,mic_f(ray16.tnear[rayIndex]),t),t,max_dist_xyz);

    //////////////////////////////////////////////////////////////////////////////////////////////////

    /* did the ray hit one of the four triangles? */
    if (unlikely(any(m_final)))
      {
	STAT3(normal.trav_prim_hits,1,1,1);
	max_dist_xyz  = select(m_final,t,max_dist_xyz);
	const mic_f min_dist = vreduce_min(max_dist_xyz);
	const mic_m m_dist = eq(min_dist,max_dist_xyz);
	const size_t vecIndex = bitscan(toInt(m_dist));
	const size_t triIndex = vecIndex >> 2;

	const mic_m m_tri = m_dist^(m_dist & (mic_m)((unsigned int)m_dist - 1));

                
	prefetch<PFHINT_L1EX>(&ray16.tfar);  
	prefetch<PFHINT_L1EX>(&ray16.u);
	prefetch<PFHINT_L1EX>(&ray16.v);
	prefetch<PFHINT_L1EX>(&ray16.Ng.x); 
	prefetch<PFHINT_L1EX>(&ray16.Ng.y); 
	prefetch<PFHINT_L1EX>(&ray16.Ng.z); 
	prefetch<PFHINT_L1EX>(&ray16.geomID);
	prefetch<PFHINT_L1EX>(&ray16.primID);

	max_dist_xyz = min_dist;

	const mic_f gnormalx(normal[triIndex*4+1]);
	const mic_f gnormaly(normal[triIndex*4+2]);
	const mic_f gnormalz(normal[triIndex*4+0]);
		  
	compactustore16f_low(m_tri,&ray16.tfar[rayIndex],min_dist);
	compactustore16f_low(m_tri,&ray16.u[rayIndex],u); 
	compactustore16f_low(m_tri,&ray16.v[rayIndex],v); 
	compactustore16f_low(m_tri,&ray16.Ng.x[rayIndex],gnormalx); 
	compactustore16f_low(m_tri,&ray16.Ng.y[rayIndex],gnormaly); 
	compactustore16f_low(m_tri,&ray16.Ng.z[rayIndex],gnormalz); 

	ray16.geomID[rayIndex] = quad.geomID;
	ray16.primID[rayIndex] = quad.primID;
	return true;
      
      }
    return false;      
  };

  extern size_t g_subdivision_level;


  void subdivide_intersect1(const size_t rayIndex, 
			    const mic_f &dir_xyz,
			    const mic_f &org_xyz,
			    Ray16& ray16,
			    const IrregularCatmullClarkPatch &patch,
			    const unsigned int subdiv_level = 0);

  void subdivide_intersect1(const size_t rayIndex, 
			    const mic_f &dir_xyz,
			    const mic_f &org_xyz,
			    Ray16& ray16,
			    const RegularCatmullClarkPatch &patch,
			    const unsigned int subdiv_level = 0);


  template< bool ENABLE_INTERSECTION_FILTER>
    struct SubdivPatchIntersector16
    {
      typedef SubdivPatch1 Primitive;


      static __forceinline bool intersect1(const size_t rayIndex, 
					   const mic_f &dir_xyz,
					   const mic_f &org_xyz,
					   Ray16& ray16,
					   const SubdivPatch1& subdiv_patch)
      {
	STAT3(normal.trav_prims,1,1,1);


	bool hit = false;
      
#if 1
	if (subdiv_patch.isRegular())
	  {
	    RegularCatmullClarkPatch regular_patch;
	    subdiv_patch.init( regular_patch );
	    subdivide_intersect1(rayIndex,dir_xyz,org_xyz,ray16,regular_patch,g_subdivision_level);
	  }
	else
#endif
	{
	  IrregularCatmullClarkPatch irregular_patch;
	  subdiv_patch.init( irregular_patch );
	  subdivide_intersect1(rayIndex,dir_xyz,org_xyz,ray16,irregular_patch,g_subdivision_level);
	}

	return hit;
      };

      static __forceinline bool occluded1_quad(const size_t rayIndex, 
					       const mic_f &dir_xyz,
					       const mic_f &org_xyz,
					       const Ray16& ray16,
					       mic_m &m_terminated,
					       const FinalQuad &quad)
      {
	const mic_i and_mask = broadcast4to16i(_zlc4);

	const mic_f v0 = gather_4f_zlc(and_mask,
				       &quad.vtx[0],
				       &quad.vtx[0],
				       &quad.vtx[2],
				       &quad.vtx[2]);
	      
	const mic_f v1 = gather_4f_zlc(and_mask,
				       &quad.vtx[1],
				       &quad.vtx[1],
				       &quad.vtx[3],
				       &quad.vtx[3]);
	      
	const mic_f v2 = gather_4f_zlc(and_mask,
				       &quad.vtx[2],
				       &quad.vtx[2],
				       &quad.vtx[0],
				       &quad.vtx[0]);

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
	if (unlikely(none(m_aperture))) return false;

	mic_m m_final  = lt(lt(m_aperture,mic_f(ray16.tnear[rayIndex]),t),t,mic_f(ray16.tfar[rayIndex]));

	if (unlikely(any(m_final)))
	  {
	    STAT3(shadow.trav_prim_hits,1,1,1);
	    m_terminated |= mic_m::shift1[rayIndex];
	    return true;
	  }
	return false;
      }

      static __forceinline bool occluded1(const size_t rayIndex, 
					  const mic_f &dir_xyz,
					  const mic_f &org_xyz,
					  const Ray16& ray16, 
					  mic_m &m_terminated,
					  const SubdivPatch1& subdiv_patch) 
      {
	STAT3(shadow.trav_prims,1,1,1);

	__aligned(64) FinalQuad finalQuad;

	IrregularCatmullClarkPatch irregular_patch;
	subdiv_patch.init( irregular_patch );
      
	irregular_patch.init( finalQuad );
	return occluded1_quad(rayIndex,dir_xyz,org_xyz,ray16,m_terminated,finalQuad);
      };
    };

  template< bool ENABLE_INTERSECTION_FILTER>
    struct SubdivPatchIntersector1
    {
      typedef SubdivPatch1 Primitive;

      // ==================================================================
      // ==================================================================
      // ==================================================================


      static __forceinline bool intersect1(const mic_f &dir_xyz,
					   const mic_f &org_xyz,
					   Ray& ray, 
					   const SubdivPatch1& patch)
      {
	STAT3(normal.trav_prims,1,1,1);

	return true;
      }

      static __forceinline bool occluded1(const mic_f &dir_xyz,
					  const mic_f &org_xyz,
					  Ray& ray, 
					  const SubdivPatch1& patch) 
      {
	STAT3(shadow.trav_prims,1,1,1);
	return true;
      }

    };
}

