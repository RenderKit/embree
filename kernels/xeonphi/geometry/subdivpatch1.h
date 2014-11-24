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

#include "primitive.h"
#include "common/scene_subdiv_mesh.h"
#include "common/scene_subdivision.h"
#include "bicubic_bezier_patch.h"

// right now everything is shared between xeon and xeon phi, so moved all stuff to common/scene_subdivision.h

#define FORCE_TESSELLATION_BOUNDS 1

namespace embree
{

  __forceinline BBox3fa getBBox3fa(const mic3f &v, const mic_m m_valid = 0xffff)
  {
    const mic_f x_min = select(m_valid,v.x,mic_f::inf());
    const mic_f y_min = select(m_valid,v.y,mic_f::inf());
    const mic_f z_min = select(m_valid,v.z,mic_f::inf());

    const mic_f x_max = select(m_valid,v.x,mic_f::minus_inf());
    const mic_f y_max = select(m_valid,v.y,mic_f::minus_inf());
    const mic_f z_max = select(m_valid,v.z,mic_f::minus_inf());

    const Vec3fa b_min( reduce_min(x_min), reduce_min(y_min), reduce_min(z_min) );
    const Vec3fa b_max( reduce_max(x_max), reduce_max(y_max), reduce_max(z_max) );
    return BBox3fa( b_min, b_max );
  }

  struct __aligned(64) SubdivPatch1
  {
  public:
    enum {
      REGULAR_PATCH = 1 // = 0 => Gregory Patch 
    };

    /*! Default constructor. */
    SubdivPatch1 (const SubdivMesh::HalfEdge * first_half_edge,
		  const Vec3fa *vertices,
		  unsigned int geomID,
		  unsigned int primID) 
      : geomID(geomID),
      primID(primID),
      under_construction(0),
      bvh4i_subtree_root(-1),
      flags(0)
    {
      assert(sizeof(SubdivPatch1) == 5 * 64);

      u_range = Vec2f(0.0f,1.0f);
      v_range = Vec2f(0.0f,1.0f);

      /* init irregular patch */

      IrregularCatmullClarkPatch ipatch ( first_half_edge, vertices ); 

      /* init discrete edge tessellation levels and grid resolution */

      assert( ipatch.level[0] >= 0.0f );
      assert( ipatch.level[1] >= 0.0f );
      assert( ipatch.level[2] >= 0.0f );
      assert( ipatch.level[3] >= 0.0f );

      level[0] = max(ceilf(ipatch.level[0]),1.0f);
      level[1] = max(ceilf(ipatch.level[1]),1.0f);
      level[2] = max(ceilf(ipatch.level[2]),1.0f);
      level[3] = max(ceilf(ipatch.level[3]),1.0f);

      grid_u_res = max(level[0],level[2])+1; // n segments -> n+1 points
      grid_v_res = max(level[1],level[3])+1;
      //grid_size = (grid_u_res*grid_v_res+15)&(-16);

      grid_size_64b_blocks = 1;

      /* tessellate into 4x4 grid blocks */
      if (grid_u_res*grid_v_res > 16)
	grid_size_64b_blocks = getSubTreeSize64bBlocks();

      assert(grid_size_64b_blocks >= 1);

      /* compute 16-bit quad mask for quad-tessellation */

      if (grid_u_res*grid_v_res <= 16)
	{
	  mic_m m_active = 0xffff;
	  for (unsigned int i=grid_u_res-1;i<16;i+=grid_u_res)
	    m_active ^= (unsigned int)1 << i;
	  m_active &= ((unsigned int)1 << (grid_u_res * (grid_v_res-1)))-1;
	  grid_mask = m_active;
	}
      else
	{
	  grid_mask = 0;
	}

      /* determine whether patch is regular or not */

      flags = 0;
      if (ipatch.dicable()) 
	{
	  flags |= REGULAR_PATCH;
	  patch.init( ipatch );
	}
      else
	{
	  GregoryPatch gpatch; 
	  gpatch.init( ipatch ); 
	  gpatch.exportDenseConrolPoints( patch.v );
	}
    }

    __forceinline mic3f eval16(const mic_f &uu,
			       const mic_f &vv) const
    {
      patch.prefetchData();

      if (likely(isRegular()))
	{
	  return patch.eval16(uu,vv);
	}
      else 
	{
	  return GregoryPatch::eval16( patch.v, uu, vv );
	}
      
    }

    __forceinline mic_f eval4(const mic_f &uu,
			      const mic_f &vv) const
    {
      patch.prefetchData();

      if (likely(isRegular()))
	{
	  return patch.eval4(uu,vv);
	}
      else 
	{	  
	  return GregoryPatch::eval4( patch.v, uu, vv );
	}
      
    }

    __forceinline Vec3fa normal(const float &uu,
				const float &vv) const
    {
      if (likely(isRegular()))
	{
	  const mic_f n = patch.normal4(uu,vv);
	  return Vec3fa(n[0],n[1],n[2]);
	  //return patch.normal(uu,vv);
	}
      else 
	{
	  // FIXME: fast "lane" code for gregory patch normal
	  return GregoryPatch::normal( patch.v, uu, vv );
	}
      
    }


    __forceinline bool isRegular() const
    {
      return (flags & REGULAR_PATCH) == REGULAR_PATCH;
    }

    __forceinline bool isGregoryPatch() const
    {
      return !isRegular();
    }

    BBox3fa bounds() const
    {
      //FIXME: process in 16-wide blocks
#if FORCE_TESSELLATION_BOUNDS == 1

      __aligned(64) float u_array[(grid_size_64b_blocks+1)*16];
      __aligned(64) float v_array[(grid_size_64b_blocks+1)*16];

      const unsigned int real_grid_size = grid_u_res*grid_v_res;
      gridUVTessellator(level,grid_u_res,grid_v_res,u_array,v_array);

      for (size_t i=real_grid_size;i<grid_size_64b_blocks*16;i++)
	{
	  u_array[i] = 0.0f;
	  v_array[i] = 0.0f;
	}


#if 0
      BBox3fa b ( empty );

      if (isRegular())
	for (size_t i=0;i<real_grid_size;i++)
	  {
	    const Vec3fa vtx = patch.eval( u_array[i], v_array[i] );	    
	    b.extend( vtx );
	  }
      else
	{
	  Vec3fa f_m[2][2];
	  f_m[0][0] = GregoryPatch::extract_f_m_Vec3fa(patch.v,0);
	  f_m[0][1] = GregoryPatch::extract_f_m_Vec3fa(patch.v,1);
	  f_m[1][1] = GregoryPatch::extract_f_m_Vec3fa(patch.v,2);
	  f_m[1][0] = GregoryPatch::extract_f_m_Vec3fa(patch.v,3);

	  GregoryPatch gpatch( patch.v, f_m);
	  for (size_t i=0;i<real_grid_size;i++)
	    {
	      const Vec3fa vtx = gpatch.eval( u_array[i], v_array[i] );
	      b.extend( vtx );
	    }
	}

      b.lower.a = 0.0f;
      b.upper.a = 0.0f;

#else
      BBox3fa b ( empty );
      
      assert( grid_size_64b_blocks >= 1 );
      for (size_t i=0;i<grid_size_64b_blocks;i++)
	{
	  const mic3f vtx = eval16( load16f(&u_array[i*16]), load16f(&v_array[i*16]) );
	  b.extend( getBBox3fa(vtx) );
	}

      b.lower.a = 0.0f;
      b.upper.a = 0.0f;
#endif
     
#if DEBUG
      isfinite(b.lower.x);
      isfinite(b.lower.y);
      isfinite(b.lower.z);

      isfinite(b.upper.x);
      isfinite(b.upper.y);
      isfinite(b.upper.z);
#endif
      
#else
      BBox3fa b = patch.bounds();
      if (unlikely(isGregoryPatch()))
	{
	  b.extend( extract_f_m_Vec3fa(patch.v,0) );
	  b.extend( extract_f_m_Vec3fa(patch.v,1) );
	  b.extend( extract_f_m_Vec3fa(patch.v,2) );
	  b.extend( extract_f_m_Vec3fa(patch.v,3) );
	}
#endif

      return b;
    }

    __forceinline void store(void *mem)
    {
      const mic_f *const src = (mic_f*)this;
      assert(sizeof(SubdivPatch1) % 64 == 0);
      mic_f *const dst = (mic_f*)mem;
#pragma unroll
      for (size_t i=0;i<sizeof(SubdivPatch1) / 64;i++)
	store16f_ngo(&dst[i],src[i]);
    }

  private:
    size_t get64BytesBlocksForGridSubTree(const unsigned int u_start,
					  const unsigned int u_end,
					  const unsigned int v_start,
					  const unsigned int v_end)
    {
      const unsigned int u_size = u_end-u_start+1;
      const unsigned int v_size = v_end-v_start+1;
      if (u_size <= 4 && v_size <= 4)
	return 2;/* 128 bytes for 16x 'u' and 'v' */

      const unsigned int u_mid = (u_start+u_end)/2;
      const unsigned int v_mid = (v_start+v_end)/2;

      const unsigned int subtree_u_start[4] = { u_start ,u_mid ,u_mid ,u_start };
      const unsigned int subtree_u_end  [4] = { u_mid   ,u_end ,u_end ,u_mid };
      const unsigned int subtree_v_start[4] = { v_start ,v_start ,v_mid ,v_mid};
      const unsigned int subtree_v_end  [4] = { v_mid   ,v_mid   ,v_end ,v_end };
      
      size_t blocks = 2; /* 128 bytes bvh4i node layout */

      for (unsigned int i=0;i<4;i++)
	blocks += get64BytesBlocksForGridSubTree(subtree_u_start[i], 
						 subtree_u_end[i],
						 subtree_v_start[i],
						 subtree_v_end[i]);
      return blocks;    
    }

    __forceinline unsigned int getSubTreeSize64bBlocks()
    {
      return get64BytesBlocksForGridSubTree(0,grid_u_res-1,0,grid_v_res-1);
    }

  public:
   
    Vec2f u_range;
    Vec2f v_range;
    float level[4];

    unsigned int flags;
    unsigned int geomID;                          //!< geometry ID of the subdivision mesh this patch belongs to
    unsigned int primID;                          //!< primitive ID of this subdivision patch
    volatile unsigned int bvh4i_subtree_root;

    unsigned short grid_u_res;
    unsigned short grid_v_res;
    unsigned int   grid_size_64b_blocks;
    unsigned int   grid_mask;
    volatile unsigned int under_construction; // 0 = not build yet, 1 = under construction, 2 = built

    __aligned(64) RegularCatmullClarkPatch patch;
  };

  __forceinline std::ostream &operator<<(std::ostream &o, const SubdivPatch1 &p)
    {
      o << " flags " << p.flags << " geomID " << p.geomID << " primID " << p.primID << " u_range << " << p.u_range << " v_range " << p.v_range << " levels: " << p.level[0] << "," << p.level[1] << "," << p.level[2] << "," << p.level[3] << std::endl;
      o << " patch " << p.patch;

      return o;
    } 

};

