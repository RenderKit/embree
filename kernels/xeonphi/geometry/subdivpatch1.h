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
#include "common/subdiv/bspline_patch.h"
#include "common/subdiv/gregory_patch.h"
#include "common/subdiv/tessellation.h"
#include "bicubic_bezier_patch.h"

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
      REGULAR_PATCH     = 1,  // 0 => Gregory Patch 
      TRANSITION_PATCH  = 2,  // needs stiching?
      HAS_DISPLACEMENTS = 4   // 0 => no displacments
    };

    /*! Default constructor. */
    SubdivPatch1 (const SubdivMesh::HalfEdge * first_half_edge,
		  const Vec3fa *vertices,
		  unsigned int geomID,
		  unsigned int primID,
		  const SubdivMesh *const mesh) 
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

      CatmullClarkPatch ipatch ( first_half_edge, vertices ); 

      /* init discrete edge tessellation levels and grid resolution */

      assert( ipatch.ring[0].edge_level >= 0.0f );
      assert( ipatch.ring[1].edge_level >= 0.0f );
      assert( ipatch.ring[2].edge_level >= 0.0f );
      assert( ipatch.ring[3].edge_level >= 0.0f );

      level[0] = max(ceilf(ipatch.ring[0].edge_level),1.0f);
      level[1] = max(ceilf(ipatch.ring[1].edge_level),1.0f);
      level[2] = max(ceilf(ipatch.ring[2].edge_level),1.0f);
      level[3] = max(ceilf(ipatch.ring[3].edge_level),1.0f);

      grid_u_res = max(level[0],level[2])+1; // n segments -> n+1 points
      grid_v_res = max(level[1],level[3])+1;

      grid_size_16wide_blocks = ((grid_u_res*grid_v_res+15)&(-16)) / 16;
      assert( grid_size_16wide_blocks == 1);
      grid_mask = 0;
      grid_subtree_size_64b_blocks = 5; // single leaf with u,v,x,y,z

      /* need stiching? */

      const unsigned int int_edge_points0 = (unsigned int)level[0] + 1;
      const unsigned int int_edge_points1 = (unsigned int)level[1] + 1;
      const unsigned int int_edge_points2 = (unsigned int)level[2] + 1;
      const unsigned int int_edge_points3 = (unsigned int)level[3] + 1;

      if (int_edge_points0 < (unsigned int)grid_u_res ||
	  int_edge_points2 < (unsigned int)grid_u_res ||
	  int_edge_points1 < (unsigned int)grid_v_res ||
	  int_edge_points3 < (unsigned int)grid_v_res)
	flags |= TRANSITION_PATCH;
      

      /* has displacements? */
      if (mesh->displFunc != NULL)
	flags |= HAS_DISPLACEMENTS;


      /* tessellate into 4x4 grid blocks for larger grid resolutions, generate bvh4i subtree over 4x4 grid blocks*/
      if (grid_size_16wide_blocks > 1)
	grid_subtree_size_64b_blocks = getSubTreeSize64bBlocks( 5 ); // u,v,x,y,z 

      /* compute 16-bit quad mask for direct evaluation */

      if (grid_size_16wide_blocks == 1)
	{
	  mic_m m_active = 0xffff;
	  for (unsigned int i=grid_u_res-1;i<16;i+=grid_u_res)
	    m_active ^= (unsigned int)1 << i;
	  m_active &= ((unsigned int)1 << (grid_u_res * (grid_v_res-1)))-1;
	  grid_mask = m_active;
	}

      /* determine whether patch is regular or not */

      if (ipatch.isRegular()) 
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
#if 0
      DBG_PRINT( grid_u_res );
      DBG_PRINT( grid_v_res );
      DBG_PRINT( grid_size_16wide_blocks );
      DBG_PRINT( grid_mask );
      DBG_PRINT( grid_subtree_size_64b_blocks );
#endif
    }

    __forceinline bool needsStiching() const
    {
      return (flags & TRANSITION_PATCH) == TRANSITION_PATCH;      
    }

    __forceinline void prefetchData() const
    {
      patch.prefetchData();
    }

    __forceinline mic3f eval16(const mic_f &uu,
			       const mic_f &vv) const
    {
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

    __forceinline mic3f normal16(const mic_f &uu,
				 const mic_f &vv) const
    {
      mic3f n;
      if (likely(isRegular()))
	{
	  for (size_t i=0;i<16;i++)
	    {
	      const mic_f n_i = patch.normal4(uu[i],vv[i]);
	      n.x[i] = n_i[0];
	      n.y[i] = n_i[1];
	      n.z[i] = n_i[2];
	    }
	}
      else 
	{
	  // FIXME: fast "lane" code for gregory patch normal
	  for (size_t i=0;i<16;i++)
	    {
	      const Vec3fa n_i = GregoryPatch::normal( patch.v, uu[i], vv[i] );
	      n.x[i] = n_i.x;
	      n.y[i] = n_i.y;
	      n.z[i] = n_i.z;
	    }
	}
      return n;
    }



    __forceinline bool isRegular() const
    {
      return (flags & REGULAR_PATCH) == REGULAR_PATCH;
    }

    __forceinline bool isGregoryPatch() const
    {
      return !isRegular();
    }

    BBox3fa bounds(const SubdivMesh* const geom) const
    {
#if FORCE_TESSELLATION_BOUNDS == 1

      __aligned(64) float u_array[(grid_size_16wide_blocks+1)*16];
      __aligned(64) float v_array[(grid_size_16wide_blocks+1)*16];

      if (grid_size_16wide_blocks == 1)
      	{
      	  gridUVTessellator16f(level,grid_u_res,grid_v_res,u_array,v_array);

      	  /* if necessary stich different tessellation levels in u/v grid */
      	  if (unlikely(needsStiching()))
      	    stichUVGrid(level,grid_u_res,grid_v_res,u_array,v_array);
      	}
      else
	{
	  const unsigned int real_grid_size = grid_u_res*grid_v_res;
	  gridUVTessellatorMIC(level,grid_u_res,grid_v_res,u_array,v_array);

	  // FIXME: remove
	  for (size_t i=real_grid_size;i<grid_size_16wide_blocks*16;i++)
	    {
	      u_array[i] = 1.0f;
	      v_array[i] = 1.0f;
	    }
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
      
      assert( grid_size_16wide_blocks >= 1 );
      for (size_t i=0;i<grid_size_16wide_blocks;i++)
	{
	  const mic_f u = load16f(&u_array[i*16]);
	  const mic_f v = load16f(&v_array[i*16]);

	  mic3f vtx = eval16( u, v );

	  /* eval displacement function */
	  if (unlikely(geom->displFunc != NULL))
	    {
	      PING;
	      mic3f normal      = normal16(u,v);

	      geom->displFunc(geom->userPtr,
			      geomID,
			      primID,
			      (const float*)&u,
			      (const float*)&v,
			      (const float*)&normal,
			      (const float*)&normal,
			      (const float*)&normal,
			      (float*)&vtx.x,
			      (float*)&vtx.y,
			      (float*)&vtx.z,
			      16);

	    }

	  /* extend bounding box */
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
	  b.extend( GregoryPatch::extract_f_m_Vec3fa(patch.v,0) );
	  b.extend( GregoryPatch::extract_f_m_Vec3fa(patch.v,1) );
	  b.extend( GregoryPatch::extract_f_m_Vec3fa(patch.v,2) );
	  b.extend( GregoryPatch::extract_f_m_Vec3fa(patch.v,3) );
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
					  const unsigned int v_end,
					  const unsigned int leafBlocks)
    {
      const unsigned int u_size = u_end-u_start+1;
      const unsigned int v_size = v_end-v_start+1;
      if (u_size <= 4 && v_size <= 4)
	return leafBlocks;/* 128 bytes for 16x 'u' and 'v' plus 16x x,y,z for vtx with displacment*/

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
						 subtree_v_end[i],
						 leafBlocks);
      return blocks;    
    }

    __forceinline unsigned int getSubTreeSize64bBlocks(const unsigned int leafBlocks = 2)
    {
      return get64BytesBlocksForGridSubTree(0,grid_u_res-1,0,grid_v_res-1,leafBlocks);
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
    unsigned short grid_size_16wide_blocks;
    unsigned short grid_mask;
    unsigned int   grid_subtree_size_64b_blocks;
    volatile unsigned int under_construction; // 0 = not build yet, 1 = under construction, 2 = built

    __aligned(64) BSplinePatch patch;
  };

  __forceinline std::ostream &operator<<(std::ostream &o, const SubdivPatch1 &p)
    {
      o << " flags " << p.flags << " geomID " << p.geomID << " primID " << p.primID << " u_range << " << p.u_range << " v_range " << p.v_range << " levels: " << p.level[0] << "," << p.level[1] << "," << p.level[2] << "," << p.level[3] << std::endl;
      o << " patch " << p.patch;

      return o;
    } 

};

