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
#include "../../common/ray16.h"
#include "filter.h"

namespace embree
{

#define FORCE_TRIANGLE_UV 0

  __forceinline float16 load3x5f_unalign(const void *__restrict__ const ptr0,
				       const void *__restrict__ const ptr1,
				       const void *__restrict__ const ptr2) 
  {
    float16 v = uload16f((float*)ptr0);
    v = uload16f(v,0b1111100000,(float*)ptr1);
    v = uload16f(v,0b111110000000000,(float*)ptr2);
    return v;
  }


  struct __aligned(64) Quad3x5 {

    const static int tri_permute_v0[16];
    const static int tri_permute_v1[16];
    const static int tri_permute_v2[16];

    Vec3f16 vtx;
    float16 uv;

    __forceinline void prefetchData() const
    {
      prefetch<PFHINT_NT>(&vtx.x);
      prefetch<PFHINT_NT>(&vtx.y);
      prefetch<PFHINT_NT>(&vtx.z);
      prefetch<PFHINT_NT>(&uv);
    }
    
    __forceinline float16 getU() const
    {
      const int16 u = cast(uv) & 0xffff;
      return float16(u) * 2.0f/65535.0f;
    }

    __forceinline float16 getV() const
    {
      const int16 v = cast(uv) >> 16;
      return float16(v) * 2.0f/65535.0f;
    }

    
    __forceinline void init(size_t offset_bytes, 
			    const SubdivPatch1Base &patch,
			    float *lazyCachePtr)
    {
      const size_t dim_offset    = patch.grid_size_simd_blocks * 16;
      const size_t line_offset   = patch.grid_u_res;
      const float *const grid_x  = (float*)(offset_bytes + (size_t)lazyCachePtr);
      const float *const grid_y  = grid_x + 1 * dim_offset;
      const float *const grid_z  = grid_x + 2 * dim_offset;
      const float *const grid_uv = grid_x + 3 * dim_offset;
      const size_t offset0 = 0*line_offset;
      const size_t offset1 = 1*line_offset;
      const size_t offset2 = 2*line_offset;
      vtx.x = load3x5f_unalign(&grid_x[offset0] ,&grid_x[offset1], &grid_x[offset2]);
      vtx.y = load3x5f_unalign(&grid_y[offset0] ,&grid_y[offset1], &grid_y[offset2]);
      vtx.z = load3x5f_unalign(&grid_z[offset0] ,&grid_z[offset1], &grid_z[offset2]);
      uv    = load3x5f_unalign(&grid_uv[offset0],&grid_uv[offset1],&grid_uv[offset2]);	
    }

    __forceinline void intersect1_tri16_precise(const size_t rayIndex, 
						const float16 &dir_xyz,
						const float16 &org_xyz,
						Ray16& ray16,
						const Precalculations &pre,
						const unsigned int subdiv_patch_index,
                                                const bool16 m_input = 0xffff) const
    {
      const int16 perm_v0 = load16i(tri_permute_v0);
      const int16 perm_v1 = load16i(tri_permute_v1);
      const int16 perm_v2 = load16i(tri_permute_v2);

      const Vec3f16 ray_dir(swAAAA(dir_xyz),swBBBB(dir_xyz),swCCCC(dir_xyz));
      const Vec3f16 ray_org(swAAAA(org_xyz),swBBBB(org_xyz),swCCCC(org_xyz));
    
      const Vec3f16 vtx_org = vtx - ray_org;
      
      const Vec3f16 v0( permute16f(perm_v0,vtx_org.x), permute16f(perm_v0,vtx_org.y), permute16f(perm_v0,vtx_org.z) );
      const Vec3f16 v1( permute16f(perm_v1,vtx_org.x), permute16f(perm_v1,vtx_org.y), permute16f(perm_v1,vtx_org.z) );
      const Vec3f16 v2( permute16f(perm_v2,vtx_org.x), permute16f(perm_v2,vtx_org.y), permute16f(perm_v2,vtx_org.z) );
  
      const Vec3f16 e0 = v2 - v0;
      const Vec3f16 e1 = v0 - v1;	     
      const Vec3f16 e2 = v1 - v2;	     

      const Vec3f16 Ng1     = cross(e1,e0);
      const Vec3f16 Ng      = Ng1+Ng1;
      const float16 den     = dot(Ng,ray_dir);	      
      const float16 rcp_den = rcp(den);

#if defined(RTCORE_BACKFACE_CULLING)
      bool16 m_valid = m_input & (den > zero);
#else
      bool16 m_valid = m_input;
#endif

      const float16 u = dot(cross(v2+v0,e0),ray_dir) * rcp_den; 
      m_valid       = ge( m_valid, u, zero);

      const float16 v       = dot(cross(v0+v1,e1),ray_dir) * rcp_den; 
      m_valid       = ge( m_valid, v, zero);

      const float16 w       = dot(cross(v1+v2,e2),ray_dir) * rcp_den;  
      m_valid       = ge( m_valid, w, zero);

      if (unlikely(none(m_valid))) return;
    
      const bool16 m_den = ne(m_valid,den,zero);
      const float16 t = dot(v0,Ng) * rcp_den;
      float16 max_dist_xyz = float16(ray16.tfar[rayIndex]);
      bool16 m_final      = lt(lt(m_den,float16(ray16.tnear[rayIndex]),t),t,max_dist_xyz);

      //////////////////////////////////////////////////////////////////////////////////////////////////

      if (unlikely(any(m_final)))
	{
	  STAT3(normal.trav_prim_hits,1,1,1);
	  max_dist_xyz  = select(m_final,t,max_dist_xyz);
	  const float16 min_dist = vreduce_min(max_dist_xyz);
	  const bool16 m_dist = eq(min_dist,max_dist_xyz);
	  const size_t index = bitscan(toInt(m_dist));

#if FORCE_TRIANGLE_UV == 0
	  const float16 uu  = getU();
	  const float16 u0 = permute16f(perm_v0,uu);
	  const float16 u1 = permute16f(perm_v1,uu);
	  const float16 u2 = permute16f(perm_v2,uu);
	  const float16 u_final = u * u1 + v * u2 + (1.0f-u-v) * u0;

	  const float16 vv = getV();
	  const float16 v0 = permute16f(perm_v0,vv); 
	  const float16 v1 = permute16f(perm_v1,vv);
	  const float16 v2 = permute16f(perm_v2,vv);
	  const float16 v_final = u * v1 + v * v2 + (1.0f-u-v) * v0;
#else
	  const float16 u_final = u;
	  const float16 v_final = v;
#endif

	  const bool16 m_tri = m_dist^(m_dist & (bool16)((unsigned int)m_dist - 1));
                
	  prefetch<PFHINT_L1EX>(&ray16.tfar);  
	  prefetch<PFHINT_L1EX>(&ray16.u);
	  prefetch<PFHINT_L1EX>(&ray16.v);
	  prefetch<PFHINT_L1EX>(&ray16.Ng.x); 
	  prefetch<PFHINT_L1EX>(&ray16.Ng.y); 
	  prefetch<PFHINT_L1EX>(&ray16.Ng.z); 
	  prefetch<PFHINT_L1EX>(&ray16.geomID);
	  prefetch<PFHINT_L1EX>(&ray16.primID);

	  const bool16 m_neg = 0b0101010101010101;
	  const float16 gnormalx(select(m_neg,float16(-Ng.x),float16(Ng.x)));
	  const float16 gnormaly(select(m_neg,float16(-Ng.y),float16(Ng.y)));
	  const float16 gnormalz(select(m_neg,float16(-Ng.z),float16(Ng.z)));
		  
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

    __forceinline bool occluded1_tri16_precise(const size_t rayIndex, 
					       const float16 &dir_xyz,
					       const float16 &org_xyz,
					       Ray16& ray16,
					       const Precalculations &pre,
                                               const bool16 m_input = 0xffff)
    {
      const int16 perm_v0 = load16i(tri_permute_v0);
      const int16 perm_v1 = load16i(tri_permute_v1);
      const int16 perm_v2 = load16i(tri_permute_v2);

      const Vec3f16 ray_dir(swAAAA(dir_xyz),swBBBB(dir_xyz),swCCCC(dir_xyz));
      const Vec3f16 ray_org(swAAAA(org_xyz),swBBBB(org_xyz),swCCCC(org_xyz));
    
      const Vec3f16 vtx_org = vtx - ray_org;
      
      const Vec3f16 v0( permute16f(perm_v0,vtx_org.x), permute16f(perm_v0,vtx_org.y), permute16f(perm_v0,vtx_org.z) );
      const Vec3f16 v1( permute16f(perm_v1,vtx_org.x), permute16f(perm_v1,vtx_org.y), permute16f(perm_v1,vtx_org.z) );
      const Vec3f16 v2( permute16f(perm_v2,vtx_org.x), permute16f(perm_v2,vtx_org.y), permute16f(perm_v2,vtx_org.z) );
   
      const Vec3f16 e0 = v2 - v0;
      const Vec3f16 e1 = v0 - v1;	     
      const Vec3f16 e2 = v1 - v2;	     

      const Vec3f16 Ng1     = cross(e1,e0);
      const Vec3f16 Ng      = Ng1+Ng1;
      const float16 den     = dot(Ng,ray_dir);	      
      const float16 rcp_den = rcp(den);

#if defined(RTCORE_BACKFACE_CULLING)
      bool16 m_valid = m_input & (den > zero);
#else
      bool16 m_valid = m_input;
#endif

      const float16 u = dot(cross(v2+v0,e0),ray_dir) * rcp_den; 
      m_valid       = ge( m_valid, u, zero);

      const float16 v       = dot(cross(v0+v1,e1),ray_dir) * rcp_den; 
      m_valid       = ge( m_valid, v, zero);

      const float16 w       = dot(cross(v1+v2,e2),ray_dir) * rcp_den;  
      m_valid       = ge( m_valid, w, zero);

      if (unlikely(none(m_valid))) return false;
    
      const bool16 m_den = ne(m_valid,den,zero);
      const float16 t = dot(v0,Ng) * rcp_den;
      float16 max_dist_xyz = float16(ray16.tfar[rayIndex]);
      bool16 m_final      = lt(lt(m_den,float16(ray16.tnear[rayIndex]),t),t,max_dist_xyz);

      //////////////////////////////////////////////////////////////////////////////////////////////////

      if (unlikely(any(m_final))) return true;
      return false;
    };


    __forceinline void intersect1_tri16_precise(const float16 &dir_xyz,
						const float16 &org_xyz,
						Ray& ray,
						const Precalculations &pre,
						const unsigned int subdiv_patch_index,
                                                const bool16 m_input = 0xffff) const
    {
      const int16 perm_v0 = load16i(tri_permute_v0);
      const int16 perm_v1 = load16i(tri_permute_v1);
      const int16 perm_v2 = load16i(tri_permute_v2);

      const Vec3f16 ray_dir(swAAAA(dir_xyz),swBBBB(dir_xyz),swCCCC(dir_xyz));
      const Vec3f16 ray_org(swAAAA(org_xyz),swBBBB(org_xyz),swCCCC(org_xyz));
    
      const Vec3f16 vtx_org = vtx - ray_org;
      
      const Vec3f16 v0( permute16f(perm_v0,vtx_org.x), permute16f(perm_v0,vtx_org.y), permute16f(perm_v0,vtx_org.z) );
      const Vec3f16 v1( permute16f(perm_v1,vtx_org.x), permute16f(perm_v1,vtx_org.y), permute16f(perm_v1,vtx_org.z) );
      const Vec3f16 v2( permute16f(perm_v2,vtx_org.x), permute16f(perm_v2,vtx_org.y), permute16f(perm_v2,vtx_org.z) );
  
      const Vec3f16 e0 = v2 - v0;
      const Vec3f16 e1 = v0 - v1;	     
      const Vec3f16 e2 = v1 - v2;	     
      const Vec3f16 Ng1     = cross(e1,e0);
      const Vec3f16 Ng      = Ng1+Ng1;
      const float16 den     = dot(Ng,ray_dir);	      
      const float16 rcp_den = rcp(den);

#if defined(RTCORE_BACKFACE_CULLING)
      bool16 m_valid = m_input & (den > zero);
#else
      bool16 m_valid = m_input;
#endif

      const float16 u = dot(cross(v2+v0,e0),ray_dir) * rcp_den; 
      m_valid       = ge( m_valid, u, zero);

      const float16 v       = dot(cross(v0+v1,e1),ray_dir) * rcp_den; 
      m_valid       = ge( m_valid, v, zero);

      const float16 w       = dot(cross(v1+v2,e2),ray_dir) * rcp_den;  
      m_valid       = ge( m_valid, w, zero);

      if (unlikely(none(m_valid))) return;
    
      const bool16 m_den = ne(m_valid,den,zero);
      const float16 t = dot(v0,Ng) * rcp_den;
      float16 max_dist_xyz = float16(ray.tfar);
      bool16 m_final      = lt(lt(m_den,float16(ray.tnear),t),t,max_dist_xyz);

      //////////////////////////////////////////////////////////////////////////////////////////////////

      if (unlikely(any(m_final)))
	{
	  STAT3(normal.trav_prim_hits,1,1,1);
	  max_dist_xyz  = select(m_final,t,max_dist_xyz);
	  const float16 min_dist = vreduce_min(max_dist_xyz);
	  const bool16 m_dist = eq(min_dist,max_dist_xyz);
	  const size_t index = bitscan(toInt(m_dist));

#if FORCE_TRIANGLE_UV == 0
	  const float16 uu  = getU();
	  const float16 u0 = permute16f(perm_v0,uu);
	  const float16 u1 = permute16f(perm_v1,uu);
	  const float16 u2 = permute16f(perm_v2,uu);
	  const float16 u_final = u * u1 + v * u2 + (1.0f-u-v) * u0;

	  const float16 vv = getV();
	  const float16 v0 = permute16f(perm_v0,vv); 
	  const float16 v1 = permute16f(perm_v1,vv);
	  const float16 v2 = permute16f(perm_v2,vv);
	  const float16 v_final = u * v1 + v * v2 + (1.0f-u-v) * v0;
#else
	  const float16 u_final = u;
	  const float16 v_final = v;
#endif

	  const bool16 m_tri = m_dist^(m_dist & (bool16)((unsigned int)m_dist - 1));
                
	  assert( countbits(m_tri) == 1);

	  const bool16 m_neg = 0b0101010101010101;
	  const float16 gnormalx(select(m_neg,float16(-Ng.x),float16(Ng.x)));
	  const float16 gnormaly(select(m_neg,float16(-Ng.y),float16(Ng.y)));
	  const float16 gnormalz(select(m_neg,float16(-Ng.z),float16(Ng.z)));
		  
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

    __forceinline bool occluded1_tri16_precise(const float16 &dir_xyz,
					       const float16 &org_xyz,
					       Ray& ray,
					       const Precalculations &pre,
                                               const bool16 m_input = 0xffff)
    {
      const int16 perm_v0 = load16i(tri_permute_v0);
      const int16 perm_v1 = load16i(tri_permute_v1);
      const int16 perm_v2 = load16i(tri_permute_v2);

      const Vec3f16 ray_dir(swAAAA(dir_xyz),swBBBB(dir_xyz),swCCCC(dir_xyz));
      const Vec3f16 ray_org(swAAAA(org_xyz),swBBBB(org_xyz),swCCCC(org_xyz));
    
      const Vec3f16 vtx_org = vtx - ray_org;
      
      const Vec3f16 v0( permute16f(perm_v0,vtx_org.x), permute16f(perm_v0,vtx_org.y), permute16f(perm_v0,vtx_org.z) );
      const Vec3f16 v1( permute16f(perm_v1,vtx_org.x), permute16f(perm_v1,vtx_org.y), permute16f(perm_v1,vtx_org.z) );
      const Vec3f16 v2( permute16f(perm_v2,vtx_org.x), permute16f(perm_v2,vtx_org.y), permute16f(perm_v2,vtx_org.z) );
   
      const Vec3f16 e0 = v2 - v0;
      const Vec3f16 e1 = v0 - v1;	     
      const Vec3f16 e2 = v1 - v2;	     
      const Vec3f16 Ng1     = cross(e1,e0);
      const Vec3f16 Ng      = Ng1+Ng1;
      const float16 den     = dot(Ng,ray_dir);	      
      const float16 rcp_den = rcp(den);

#if defined(RTCORE_BACKFACE_CULLING)
      bool16 m_valid = m_input & (den > zero);
#else
      bool16 m_valid = m_input;
#endif

      const float16 u = dot(cross(v2+v0,e0),ray_dir) * rcp_den; 
      m_valid       = ge( m_valid, u, zero);

      const float16 v       = dot(cross(v0+v1,e1),ray_dir) * rcp_den; 
      m_valid       = ge( m_valid, v, zero);

      const float16 w       = dot(cross(v1+v2,e2),ray_dir) * rcp_den;  
      m_valid       = ge( m_valid, w, zero);

      if (unlikely(none(m_valid))) return false;
    
      const bool16 m_den = ne(m_valid,den,zero);
      const float16 t = dot(v0,Ng) * rcp_den;
      float16 max_dist_xyz = float16(ray.tfar);
      bool16 m_final      = lt(lt(m_den,float16(ray.tnear),t),t,max_dist_xyz);

      //////////////////////////////////////////////////////////////////////////////////////////////////

      if (unlikely(any(m_final))) return true;
      return false;

    }


  };

  static __forceinline void intersect1_tri16(const size_t rayIndex, 
					     const float16 &dir_xyz,
					     const float16 &org_xyz,
					     Ray16& ray16,
					     const Vec3f16 &v0,
					     const Vec3f16 &v1,
					     const Vec3f16 &v2,
					     const float16 &u_grid,
					     const float16 &v_grid,
					     const unsigned int offset_v0,
					     const unsigned int offset_v1,
					     const unsigned int offset_v2,
					     const bool16 &m_active,
					     const unsigned int subdiv_patch_index)
  {
    const Vec3f16 ray_org(swAAAA(org_xyz),swBBBB(org_xyz),swCCCC(org_xyz));
    const Vec3f16 ray_dir(swAAAA(dir_xyz),swBBBB(dir_xyz),swCCCC(dir_xyz));

    const Vec3f16 e1      = v1 - v0;
    const Vec3f16 e2      = v0 - v2;	     
    const Vec3f16 normal  = cross(e1,e2);
    const Vec3f16 org     = v0 - ray_org;
    const Vec3f16 od      = cross(org,ray_dir); 
    const float16 den     = dot(normal,ray_dir);	      
    const float16 rcp_den = rcp(den);
    const float16 uu      = dot(e2,od); 
    const float16 vv      = dot(e1,od); 
    const float16 u       = uu * rcp_den;
    const float16 v       = vv * rcp_den;

#if defined(RTCORE_BACKFACE_CULLING)
    const bool16 m_init = m_active & (den > zero);
#else
    const bool16 m_init = m_active;
#endif

    const bool16 valid_u    = ge(m_init,u,zero);
    const bool16 valid_v    = ge(valid_u,v,zero);
    const bool16 m_aperture = le(valid_v,u+v,float16::one()); 

    const float16 nom = dot(org,normal);

    if (unlikely(none(m_aperture))) return;

    const float16 t      = rcp_den*nom;
    float16 max_dist_xyz = float16(ray16.tfar[rayIndex]);
    bool16 m_final      = lt(lt(m_aperture,float16(ray16.tnear[rayIndex]),t),t,max_dist_xyz);

    //////////////////////////////////////////////////////////////////////////////////////////////////

    if (unlikely(any(m_final)))
      {
	STAT3(normal.trav_prim_hits,1,1,1);
	max_dist_xyz  = select(m_final,t,max_dist_xyz);
	const float16 min_dist = vreduce_min(max_dist_xyz);
	const bool16 m_dist = eq(min_dist,max_dist_xyz);
	const size_t index = bitscan(toInt(m_dist));

#if FORCE_TRIANGLE_UV == 0
	const float16 u0 = uload16f_low(&u_grid[offset_v0]);
	const float16 u1 = uload16f_low(&u_grid[offset_v1]);
	const float16 u2 = uload16f_low(&u_grid[offset_v2]);
	const float16 u_final = u * u1 + v * u2 + (1.0f-u-v) * u0;

	const float16 v0 = uload16f_low(&v_grid[offset_v0]);
	const float16 v1 = uload16f_low(&v_grid[offset_v1]);
	const float16 v2 = uload16f_low(&v_grid[offset_v2]);
	const float16 v_final = u * v1 + v * v2 + (1.0f-u-v) * v0;
#else
	const float16 u_final = u;
	const float16 v_final = v;
#endif

	const bool16 m_tri = m_dist^(m_dist & (bool16)((unsigned int)m_dist - 1));
                
	prefetch<PFHINT_L1EX>(&ray16.tfar);  
	prefetch<PFHINT_L1EX>(&ray16.u);
	prefetch<PFHINT_L1EX>(&ray16.v);
	prefetch<PFHINT_L1EX>(&ray16.Ng.x); 
	prefetch<PFHINT_L1EX>(&ray16.Ng.y); 
	prefetch<PFHINT_L1EX>(&ray16.Ng.z); 
	prefetch<PFHINT_L1EX>(&ray16.geomID);
	prefetch<PFHINT_L1EX>(&ray16.primID);

	const bool16 m_neg = 0b0101010101010101;
	const float16 gnormalx(select(m_neg,float16(-normal.x),float16(normal.x)));
	const float16 gnormaly(select(m_neg,float16(-normal.y),float16(normal.y)));
	const float16 gnormalz(select(m_neg,float16(-normal.z),float16(normal.z)));

		  
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


  static __forceinline void intersect1_tri16_precise(const size_t rayIndex, 
						     const float16 &dir_xyz,
						     const float16 &org_xyz,
						     Ray16& ray16,
						     const Vec3f16 &v0_org,
						     const Vec3f16 &v1_org,
						     const Vec3f16 &v2_org,
						     const float16 &u_grid,
						     const float16 &v_grid,
						     const unsigned int offset_v0,
						     const unsigned int offset_v1,
						     const unsigned int offset_v2,
						     const bool16 &m_active,
						     const unsigned int subdiv_patch_index)
  {
    const Vec3f16 ray_dir(swAAAA(dir_xyz),swBBBB(dir_xyz),swCCCC(dir_xyz));
    
    const Vec3f16 v0 = v0_org; // - ray_org;
    const Vec3f16 v1 = v1_org; // - ray_org;
    const Vec3f16 v2 = v2_org; // - ray_org;
   
    const Vec3f16 e0 = v2 - v0;
    const Vec3f16 e1 = v0 - v1;	     
    const Vec3f16 e2 = v1 - v2;	     

    const Vec3f16 Ng1     = cross(e1,e0);
    const Vec3f16 Ng      = Ng1+Ng1;
    const float16 den     = dot(Ng,ray_dir);	      
    const float16 rcp_den = rcp(den);

#if defined(RTCORE_BACKFACE_CULLING)
    bool16 m_valid = m_active & (den > zero);
#else
    bool16 m_valid = m_active;
#endif

    const float16 u = dot(cross(v2+v0,e0),ray_dir) * rcp_den; 
    m_valid       = ge( m_valid, u, zero);

    const float16 v       = dot(cross(v0+v1,e1),ray_dir) * rcp_den; 
    m_valid       = ge( m_valid, v, zero);

    const float16 w       = dot(cross(v1+v2,e2),ray_dir) * rcp_den;  
    m_valid       = ge( m_valid, w, zero);

    if (unlikely(none(m_valid))) return;
    
    const bool16 m_den = ne(m_valid,den,zero);
    const float16 t = dot(v0,Ng) * rcp_den;
    float16 max_dist_xyz = float16(ray16.tfar[rayIndex]);
    bool16 m_final      = lt(lt(m_den,float16(ray16.tnear[rayIndex]),t),t,max_dist_xyz);

    //////////////////////////////////////////////////////////////////////////////////////////////////

    if (unlikely(any(m_final)))
      {
	STAT3(normal.trav_prim_hits,1,1,1);
	max_dist_xyz  = select(m_final,t,max_dist_xyz);
	const float16 min_dist = vreduce_min(max_dist_xyz);
	const bool16 m_dist = eq(min_dist,max_dist_xyz);
	const size_t index = bitscan(toInt(m_dist));

#if FORCE_TRIANGLE_UV == 0
	const float16 u0 = uload16f_low(&u_grid[offset_v0]);
	const float16 u1 = uload16f_low(&u_grid[offset_v1]);
	const float16 u2 = uload16f_low(&u_grid[offset_v2]);
	const float16 u_final = u * u1 + v * u2 + (1.0f-u-v) * u0;

	const float16 v0 = uload16f_low(&v_grid[offset_v0]);
	const float16 v1 = uload16f_low(&v_grid[offset_v1]);
	const float16 v2 = uload16f_low(&v_grid[offset_v2]);
	const float16 v_final = u * v1 + v * v2 + (1.0f-u-v) * v0;
#else
	const float16 u_final = u;
	const float16 v_final = v;
#endif

	const bool16 m_tri = m_dist^(m_dist & (bool16)((unsigned int)m_dist - 1));
                
	prefetch<PFHINT_L1EX>(&ray16.tfar);  
	prefetch<PFHINT_L1EX>(&ray16.u);
	prefetch<PFHINT_L1EX>(&ray16.v);
	prefetch<PFHINT_L1EX>(&ray16.Ng.x); 
	prefetch<PFHINT_L1EX>(&ray16.Ng.y); 
	prefetch<PFHINT_L1EX>(&ray16.Ng.z); 
	prefetch<PFHINT_L1EX>(&ray16.geomID);
	prefetch<PFHINT_L1EX>(&ray16.primID);

	const bool16 m_neg = 0b0101010101010101;
	const float16 gnormalx(select(m_neg,float16(-Ng.x),float16(Ng.x)));
	const float16 gnormaly(select(m_neg,float16(-Ng.y),float16(Ng.y)));
	const float16 gnormalz(select(m_neg,float16(-Ng.z),float16(Ng.z)));
		  
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


  static __forceinline bool occluded1_tri16_precise(const size_t rayIndex, 
						    const float16 &dir_xyz,
						    const float16 &org_xyz,
						    Ray16& ray16,
						    const Vec3f16 &v0_org,
						    const Vec3f16 &v1_org,
						    const Vec3f16 &v2_org,
						    const float16 &u_grid,
						    const float16 &v_grid,
						    const unsigned int offset_v0,
						    const unsigned int offset_v1,
						    const unsigned int offset_v2,
						    const bool16 &m_active,
						    const unsigned int subdiv_patch_index)
  {
    const Vec3f16 ray_dir(swAAAA(dir_xyz),swBBBB(dir_xyz),swCCCC(dir_xyz));
    
    const Vec3f16 v0 = v0_org; // - ray_org;
    const Vec3f16 v1 = v1_org; // - ray_org;
    const Vec3f16 v2 = v2_org; // - ray_org;
   
    const Vec3f16 e0 = v2 - v0;
    const Vec3f16 e1 = v0 - v1;	     
    const Vec3f16 e2 = v1 - v2;	     

    const Vec3f16 Ng1     = cross(e1,e0);
    const Vec3f16 Ng      = Ng1+Ng1;
    const float16 den     = dot(Ng,ray_dir);	      
    const float16 rcp_den = rcp(den);

#if defined(RTCORE_BACKFACE_CULLING)
    bool16 m_valid = m_active & (den > zero);
#else
    bool16 m_valid = m_active;
#endif

    const float16 u = dot(cross(v2+v0,e0),ray_dir) * rcp_den; 
    m_valid       = ge( m_valid, u, zero);

    const float16 v       = dot(cross(v0+v1,e1),ray_dir) * rcp_den; 
    m_valid       = ge( m_valid, v, zero);

    const float16 w       = dot(cross(v1+v2,e2),ray_dir) * rcp_den;  
    m_valid       = ge( m_valid, w, zero);

    if (unlikely(none(m_valid))) return false;
    
    const bool16 m_den = ne(m_valid,den,zero);
    const float16 t = dot(v0,Ng) * rcp_den;
    float16 max_dist_xyz = float16(ray16.tfar[rayIndex]);
    bool16 m_final      = lt(lt(m_den,float16(ray16.tnear[rayIndex]),t),t,max_dist_xyz);

    //////////////////////////////////////////////////////////////////////////////////////////////////

    if (unlikely(any(m_final))) return true;
    return false;
  };



};

