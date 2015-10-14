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

#pragma once

#include "subdivpatch1.h"
#include "../../common/ray.h"
#include "filter.h"

namespace embree
{

  static __forceinline void intersect1_tri16_precise(const vfloat16 &dir_xyz,
						     const vfloat16 &org_xyz,
						     Ray& ray,

						     const Vec3vf16 &v0_org,
						     const Vec3vf16 &v1_org,
						     const Vec3vf16 &v2_org,
						     const vfloat16 &u_grid,
						     const vfloat16 &v_grid,
						     const unsigned int offset_v0,
						     const unsigned int offset_v1,
						     const unsigned int offset_v2,
						     const vbool16 &m_active,
						     const unsigned int subdiv_patch_index)
  {
    const Vec3vf16 ray_dir(swAAAA(dir_xyz),swBBBB(dir_xyz),swCCCC(dir_xyz));
    
    const Vec3vf16 v0 = v0_org; // - ray_org;
    const Vec3vf16 v1 = v1_org; // - ray_org;
    const Vec3vf16 v2 = v2_org; // - ray_org;
   
    const Vec3vf16 e0 = v2 - v0;
    const Vec3vf16 e1 = v0 - v1;	     
    const Vec3vf16 e2 = v1 - v2;	     

    const Vec3vf16 Ng1     = cross(e1,e0);
    const Vec3vf16 Ng      = Ng1+Ng1;
    const vfloat16 den     = dot(Ng,ray_dir);	      
    const vfloat16 rcp_den = rcp(den);

#if defined(RTCORE_BACKFACE_CULLING)
    vbool16 m_valid = m_active & (den > zero);
#else
    vbool16 m_valid = m_active;
#endif

    const vfloat16 u = dot(cross(v2+v0,e0),ray_dir) * rcp_den; 
    m_valid       = ge( m_valid, u, zero);

    const vfloat16 v       = dot(cross(v0+v1,e1),ray_dir) * rcp_den; 
    m_valid       = ge( m_valid, v, zero);

    const vfloat16 w       = dot(cross(v1+v2,e2),ray_dir) * rcp_den;  
    m_valid       = ge( m_valid, w, zero);

    if (unlikely(none(m_valid))) return;
    
    const vbool16 m_den = ne(m_valid,den,zero);
    const vfloat16 t = dot(v0,Ng) * rcp_den;
    vfloat16 max_dist_xyz = vfloat16(ray.tfar);
    vbool16 m_final      = lt(lt(m_den,vfloat16(ray.tnear),t),t,max_dist_xyz);

    //////////////////////////////////////////////////////////////////////////////////////////////////

    if (unlikely(any(m_final)))
      {
	STAT3(normal.trav_prim_hits,1,1,1);
	max_dist_xyz  = select(m_final,t,max_dist_xyz);
	const vfloat16 min_dist = vreduce_min(max_dist_xyz);
	const vbool16 m_dist = eq(min_dist,max_dist_xyz);
	const size_t index = bitscan(toInt(m_dist));

	const vfloat16 u0 = uload16f_low(&u_grid[offset_v0]);
	const vfloat16 u1 = uload16f_low(&u_grid[offset_v1]);
	const vfloat16 u2 = uload16f_low(&u_grid[offset_v2]);
	const vfloat16 u_final = u * u1 + v * u2 + (1.0f-u-v) * u0;

	const vfloat16 v0 = uload16f_low(&v_grid[offset_v0]);
	const vfloat16 v1 = uload16f_low(&v_grid[offset_v1]);
	const vfloat16 v2 = uload16f_low(&v_grid[offset_v2]);
	const vfloat16 v_final = u * v1 + v * v2 + (1.0f-u-v) * v0;

	//const vfloat16 u_final = u;
	//const vfloat16 v_final = v;

	const vbool16 m_tri = m_dist^(m_dist & (vbool16)((unsigned int)m_dist - 1));
                
	assert( countbits(m_tri) == 1);

	const vfloat16 gnormalx(Ng.x[index]);
	const vfloat16 gnormaly(Ng.y[index]);
	const vfloat16 gnormalz(Ng.z[index]);
		  
	compactustore16f_low(m_tri,&ray.tfar,min_dist);
	compactustore16f_low(m_tri,&ray.u,u_final); 
	compactustore16f_low(m_tri,&ray.v,v_final); 
	compactustore16f_low(m_tri,&ray.Ng.x,gnormalx); 
	compactustore16f_low(m_tri,&ray.Ng.y,gnormaly); 
	compactustore16f_low(m_tri,&ray.Ng.z,gnormalz); 

	ray.geomID = -1;
	ray.primID = subdiv_patch_index;      
      }
  };


  static __forceinline bool occluded1_tri16_precise( const vfloat16 &dir_xyz,
						     const vfloat16 &org_xyz,
						     Ray& ray,
						     const Vec3vf16 &v0_org,
						     const Vec3vf16 &v1_org,
						     const Vec3vf16 &v2_org,
						     const vfloat16 &u_grid,
						     const vfloat16 &v_grid,
						     const unsigned int offset_v0,
						     const unsigned int offset_v1,
						     const unsigned int offset_v2,
						     const vbool16 &m_active,
						     const unsigned int subdiv_patch_index)
  {
    const Vec3vf16 ray_dir(swAAAA(dir_xyz),swBBBB(dir_xyz),swCCCC(dir_xyz));
    
    const Vec3vf16 v0 = v0_org; // - ray_org;
    const Vec3vf16 v1 = v1_org; // - ray_org;
    const Vec3vf16 v2 = v2_org; // - ray_org;
   
    const Vec3vf16 e0 = v2 - v0;
    const Vec3vf16 e1 = v0 - v1;	     
    const Vec3vf16 e2 = v1 - v2;	     

    const Vec3vf16 Ng1     = cross(e1,e0);
    const Vec3vf16 Ng      = Ng1+Ng1;
    const vfloat16 den     = dot(Ng,ray_dir);	      
    const vfloat16 rcp_den = rcp(den);

#if defined(RTCORE_BACKFACE_CULLING)
    vbool16 m_valid = m_active & (den > zero);
#else
    vbool16 m_valid = m_active;
#endif

    const vfloat16 u = dot(cross(v2+v0,e0),ray_dir) * rcp_den; 
    m_valid       = ge( m_valid, u, zero);

    const vfloat16 v       = dot(cross(v0+v1,e1),ray_dir) * rcp_den; 
    m_valid       = ge( m_valid, v, zero);

    const vfloat16 w       = dot(cross(v1+v2,e2),ray_dir) * rcp_den;  
    m_valid       = ge( m_valid, w, zero);

    if (unlikely(none(m_valid))) return false;
    
    const vbool16 m_den = ne(m_valid,den,zero);
    const vfloat16 t = dot(v0,Ng) * rcp_den;
    vfloat16 max_dist_xyz = vfloat16(ray.tfar);
    vbool16 m_final      = lt(lt(m_den,vfloat16(ray.tnear),t),t,max_dist_xyz);

    //////////////////////////////////////////////////////////////////////////////////////////////////

    if (unlikely(any(m_final))) return true;
    return false;
  };

  
  static __forceinline void intersect1_quad16(const vfloat16 &dir_xyz,
					      const vfloat16 &org_xyz,
					      Ray& ray,
					      const Vec3vf16 &vtx,
					      const vfloat16 &u,
					      const vfloat16 &v,
					      const unsigned int grid_res,
					      const vbool16 &m_active,
					      const unsigned int subdiv_patch_index)
  {
    const unsigned int offset_v0 = 0;
    const unsigned int offset_v1 = 1;
    const unsigned int offset_v2 = grid_res+1;
    const unsigned int offset_v3 = grid_res+0;


    const Vec3vf16 ray_org(swAAAA(org_xyz),swBBBB(org_xyz),swCCCC(org_xyz));
    
    Vec3vf16 vtx_org = vtx - ray_org;

    const Vec3vf16 &v0 = vtx_org;
    const Vec3vf16  v1( uload16f_low(&vtx_org.x[offset_v1]), uload16f_low(&vtx_org.y[offset_v1]), uload16f_low(&vtx_org.z[offset_v1]));
    const Vec3vf16  v2( uload16f_low(&vtx_org.x[offset_v2]), uload16f_low(&vtx_org.y[offset_v2]), uload16f_low(&vtx_org.z[offset_v2]));
    const Vec3vf16  v3( uload16f_low(&vtx_org.x[offset_v3]), uload16f_low(&vtx_org.y[offset_v3]), uload16f_low(&vtx_org.z[offset_v3]));

    intersect1_tri16_precise(dir_xyz,org_xyz,ray,v0,v1,v3,u,v,offset_v0,offset_v1,offset_v3,m_active,subdiv_patch_index);
    intersect1_tri16_precise(dir_xyz,org_xyz,ray,v3,v1,v2,u,v,offset_v3,offset_v1,offset_v2,m_active,subdiv_patch_index);

  }

  static __forceinline bool occluded1_quad16(const vfloat16 &dir_xyz,
					     const vfloat16 &org_xyz,
					     Ray& ray,
					     const Vec3vf16 &vtx,
					     const vfloat16 &u,
					     const vfloat16 &v,
					     const unsigned int grid_res,
					     const vbool16 &m_active,
					     const unsigned int subdiv_patch_index)
  {
    const unsigned int offset_v0 = 0;
    const unsigned int offset_v1 = 1;
    const unsigned int offset_v2 = grid_res+1;
    const unsigned int offset_v3 = grid_res+0;

    const Vec3vf16 ray_org(swAAAA(org_xyz),swBBBB(org_xyz),swCCCC(org_xyz));
    
    Vec3vf16 vtx_org = vtx - ray_org;

    const Vec3vf16 &v0 = vtx_org;
    const Vec3vf16  v1( uload16f_low(&vtx_org.x[offset_v1]), uload16f_low(&vtx_org.y[offset_v1]), uload16f_low(&vtx_org.z[offset_v1]));
    const Vec3vf16  v2( uload16f_low(&vtx_org.x[offset_v2]), uload16f_low(&vtx_org.y[offset_v2]), uload16f_low(&vtx_org.z[offset_v2]));
    const Vec3vf16  v3( uload16f_low(&vtx_org.x[offset_v3]), uload16f_low(&vtx_org.y[offset_v3]), uload16f_low(&vtx_org.z[offset_v3]));

    if (occluded1_tri16_precise(dir_xyz,org_xyz,ray,v0,v1,v3,u,v,offset_v0,offset_v1,offset_v3,m_active,subdiv_patch_index)) return true;
    if (occluded1_tri16_precise(dir_xyz,org_xyz,ray,v3,v1,v2,u,v,offset_v3,offset_v1,offset_v2,m_active,subdiv_patch_index)) return true;
    return false;
  }


};
