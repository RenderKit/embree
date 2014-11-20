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

  static __forceinline void intersect1_tri16(const size_t rayIndex, 
					     const mic_f &dir_xyz,
					     const mic_f &org_xyz,
					     Ray16& ray16,
					     const mic3f &v0,
					     const mic3f &v1,
					     const mic3f &v2,
					     const mic_f &u_grid,
					     const mic_f &v_grid,
					     const unsigned int offset_v0,
					     const unsigned int offset_v1,
					     const unsigned int offset_v2,
					     const mic_m &m_active,
					     const unsigned int subdiv_patch_index)
  {
    const mic3f ray_org(swAAAA(org_xyz),swBBBB(org_xyz),swCCCC(org_xyz));
    const mic3f ray_dir(swAAAA(dir_xyz),swBBBB(dir_xyz),swCCCC(dir_xyz));

    const mic3f e1      = v1 - v0;
    const mic3f e2      = v0 - v2;	     
    const mic3f normal  = cross(e1,e2);
    const mic3f org     = v0 - ray_org;
    const mic3f od      = cross(org,ray_dir); 
    const mic_f den     = dot(normal,ray_dir);	      
    const mic_f rcp_den = rcp(den);
    const mic_f uu      = dot(e2,od); 
    const mic_f vv      = dot(e1,od); 
    const mic_f u       = uu * rcp_den;
    const mic_f v       = vv * rcp_den;

#if defined(__BACKFACE_CULLING__)
    const mic_m m_init = m_active & (den > zero);
#else
    const mic_m m_init = m_active;
#endif

    const mic_m valid_u    = ge(m_init,u,zero);
    const mic_m valid_v    = ge(valid_u,v,zero);
    const mic_m m_aperture = le(valid_v,u+v,mic_f::one()); 

    const mic_f nom = dot(org,normal);

    if (unlikely(none(m_aperture))) return;

    const mic_f t      = rcp_den*nom;
    mic_f max_dist_xyz = mic_f(ray16.tfar[rayIndex]);
    mic_m m_final      = lt(lt(m_aperture,mic_f(ray16.tnear[rayIndex]),t),t,max_dist_xyz);

    //////////////////////////////////////////////////////////////////////////////////////////////////

    /* did the ray hit one of the four triangles? */
    if (unlikely(any(m_final)))
      {
	STAT3(normal.trav_prim_hits,1,1,1);
	max_dist_xyz  = select(m_final,t,max_dist_xyz);
	const mic_f min_dist = vreduce_min(max_dist_xyz);
	const mic_m m_dist = eq(min_dist,max_dist_xyz);
	const size_t index = bitscan(toInt(m_dist));

	const mic_f u0 = uload16f_low(&u_grid[offset_v0]);
	const mic_f u1 = uload16f_low(&u_grid[offset_v1]);
	const mic_f u2 = uload16f_low(&u_grid[offset_v2]);
	const mic_f u_final = u * u1 + v * u2 + (1.0f-u-v) * u0;

	const mic_f v0 = uload16f_low(&v_grid[offset_v0]);
	const mic_f v1 = uload16f_low(&v_grid[offset_v1]);
	const mic_f v2 = uload16f_low(&v_grid[offset_v2]);
	const mic_f v_final = u * v1 + v * v2 + (1.0f-u-v) * v0;


	const mic_m m_tri = m_dist^(m_dist & (mic_m)((unsigned int)m_dist - 1));
                
	prefetch<PFHINT_L1EX>(&ray16.tfar);  
	prefetch<PFHINT_L1EX>(&ray16.u);
	prefetch<PFHINT_L1EX>(&ray16.v);
	prefetch<PFHINT_L1EX>(&ray16.Ng.x); 
	prefetch<PFHINT_L1EX>(&ray16.Ng.y); 
	prefetch<PFHINT_L1EX>(&ray16.Ng.z); 
	prefetch<PFHINT_L1EX>(&ray16.geomID);
	prefetch<PFHINT_L1EX>(&ray16.primID);

	const mic_f gnormalx(normal.x[index]);
	const mic_f gnormaly(normal.y[index]);
	const mic_f gnormalz(normal.z[index]);
		  
	compactustore16f_low(m_tri,&ray16.tfar[rayIndex],min_dist);
	compactustore16f_low(m_tri,&ray16.u[rayIndex],u_final); 
	compactustore16f_low(m_tri,&ray16.v[rayIndex],v_final); 
	compactustore16f_low(m_tri,&ray16.Ng.x[rayIndex],gnormalx); 
	compactustore16f_low(m_tri,&ray16.Ng.y[rayIndex],gnormaly); 
	compactustore16f_low(m_tri,&ray16.Ng.z[rayIndex],gnormalz); 

	ray16.geomID[rayIndex] = -1;
	ray16.primID[rayIndex] = subdiv_patch_index;      
      }
  };

  static __forceinline void intersect1_quad16(const size_t rayIndex, 
					      const mic_f &dir_xyz,
					      const mic_f &org_xyz,
					      Ray16& ray16,
					      const mic3f &vtx,
					      const mic_f &u,
					      const mic_f &v,
					      const unsigned int grid_res,
					      const mic_m &m_active,
					      const unsigned int subdiv_patch_index)
  {
    const unsigned int offset_v0 = 0;
    const unsigned int offset_v1 = 1;
    const unsigned int offset_v2 = grid_res+1;
    const unsigned int offset_v3 = grid_res+0;

    const mic3f &v0 = vtx;
    const mic3f  v1( uload16f_low(&vtx.x[offset_v1]), uload16f_low(&vtx.y[offset_v1]), uload16f_low(&vtx.z[offset_v1]));
    const mic3f  v2( uload16f_low(&vtx.x[offset_v2]), uload16f_low(&vtx.y[offset_v2]), uload16f_low(&vtx.z[offset_v2]));
    const mic3f  v3( uload16f_low(&vtx.x[offset_v3]), uload16f_low(&vtx.y[offset_v3]), uload16f_low(&vtx.z[offset_v3]));

    intersect1_tri16(rayIndex,dir_xyz,org_xyz,ray16,v0,v1,v3,u,v,offset_v0,offset_v1,offset_v3,m_active,subdiv_patch_index);
    intersect1_tri16(rayIndex,dir_xyz,org_xyz,ray16,v3,v1,v2,u,v,offset_v3,offset_v1,offset_v2,m_active,subdiv_patch_index);
  }

  static __forceinline bool intersect1_quad(const size_t rayIndex, 
					    const mic_f &dir_xyz,
					    const mic_f &org_xyz,
					    Ray16& ray16,
					    const Vec3fa &vtx0,
					    const Vec3fa &vtx1,
					    const Vec3fa &vtx2,
					    const Vec3fa &vtx3,
					    const unsigned int subdiv_patch_index)
  {
    const mic_f v0 = broadcast4to16f(&vtx0);
    const mic_f v1 = select(0x00ff,broadcast4to16f(&vtx1),broadcast4to16f(&vtx2));
    const mic_f v2 = select(0x00ff,broadcast4to16f(&vtx2),broadcast4to16f(&vtx3));
	      
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

	ray16.geomID[rayIndex] = -1;
	ray16.primID[rayIndex] = subdiv_patch_index;
	return true;
      
      }
    return false;      
  };


  static __forceinline bool occluded1_quad(const size_t rayIndex, 
					   const mic_f &dir_xyz,
					   const mic_f &org_xyz,
					   const Ray16& ray16,
					   mic_m &m_terminated,
					   const Vec3fa &vtx0,
					   const Vec3fa &vtx1,
					   const Vec3fa &vtx2,
					   const Vec3fa &vtx3)
  {
    const mic_f v0 = broadcast4to16f(&vtx0);
    const mic_f v1 = select(0x00ff,broadcast4to16f(&vtx1),broadcast4to16f(&vtx2));
    const mic_f v2 = select(0x00ff,broadcast4to16f(&vtx2),broadcast4to16f(&vtx3));

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
    return false;
  };

}

