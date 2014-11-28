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
#include "common/subdiv/bspline_patch.h"
#include "common/subdiv/gregory_patch.h"
#include "common/subdiv/tessellation.h"

#define FORCE_TESSELLATION_BOUNDS 1
#define USE_DISPLACEMENT_FOR_TESSELLATION_BOUNDS 0

using namespace std;

namespace embree
{

#if defined(__AVX__)
  __forceinline BBox3fa getBBox3fa(const avx3f &v)
  {
    const Vec3fa b_min( reduce_min(v.x), reduce_min(v.y), reduce_min(v.z) );
    const Vec3fa b_max( reduce_max(v.x), reduce_max(v.z), reduce_max(v.z) );
    return BBox3fa( b_min, b_max );
  }
#endif


  struct __aligned(64) SubdivPatch1Cached
  {
    struct Type : public PrimitiveType 
    {
      Type ();
      size_t blocks(size_t x) const; 
      size_t size(const char* This) const;
    };

    static Type type;

  public:

    enum {
      REGULAR_PATCH     = 1,  // 0 => Gregory Patch 
      TRANSITION_PATCH  = 2,  // needs stiching?
      HAS_DISPLACEMENT  = 4   // 0 => no displacments
    };

    /*! Default constructor. */
    __forceinline SubdivPatch1Cached () {}

    /*! Construction from vertices and IDs. */
    __forceinline SubdivPatch1Cached (const SubdivMesh::HalfEdge * first_half_edge,
                                      const Vec3fa *vertices,
                                      const unsigned int gID,
                                      const unsigned int pID,
                                      const SubdivMesh *const mesh) 
      : geom(gID),
      prim(pID),
      flags(0)
    {
      assert(sizeof(SubdivPatch1Cached) == 5 * 64);

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

      grid_size_8wide_blocks       = ((grid_u_res*grid_v_res+7)&(-8)) / 8;
      grid_subtree_size_64b_blocks = 5; // single leaf with 16-wide u,v,x,y,z

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
	flags |= HAS_DISPLACEMENT;


      /* tessellate into 4x4 grid blocks for larger grid resolutions, generate bvh4i subtree over 4x4 grid blocks*/
      if (grid_size_8wide_blocks > 2)
	grid_subtree_size_64b_blocks = getSubTreeSize64bBlocks( 5 ); // u,v,x,y,z 

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

#if defined(__AVX__)
    __forceinline avx3f eval8(const avxf &uu,
        const avxf &vv) const
    {
      if (likely(isRegular()))
	{
	  return patch.eval8(uu,vv);
	}
      else 
	{
	  return GregoryPatch::eval8( patch.v, uu, vv );
	}
      
    }
#endif

    __forceinline Vec3fa normal(const float &uu,
				const float &vv) const
    {
      if (likely(isRegular()))
	{
      return patch.normal(uu,vv);
	}
      else 
	{
	  // FIXME: fast "lane" code for gregory patch normal
	  return GregoryPatch::normal( patch.v, uu, vv );
	}      
    }

#if defined(__AVX__)
    __forceinline avx3f normal8(const avxf &uu,
                                const avxf &vv) const
    {
      if (likely(isRegular()))
	return patch.normal8(uu,vv);
      else
        return GregoryPatch::normal8( patch.v, uu, vv );
    }
#endif


    __forceinline bool isRegular() const
    {
      return (flags & REGULAR_PATCH) == REGULAR_PATCH;
    }

    __forceinline bool isGregoryPatch() const
    {
      return !isRegular();
    }

    __forceinline bool hasDisplacement() const
    {
      return (flags & HAS_DISPLACEMENT) == HAS_DISPLACEMENT;
    }

    BBox3fa bounds(const SubdivMesh* const mesh) const
    {
#if FORCE_TESSELLATION_BOUNDS == 1

      __aligned(64) float u_array[(grid_size_8wide_blocks+1)*8]; // +8 for unaligned access
      __aligned(64) float v_array[(grid_size_8wide_blocks+1)*8]; // +8 for unaligned access

      const unsigned int real_grid_size = grid_u_res*grid_v_res;
      gridUVTessellator(level,grid_u_res,grid_v_res,u_array,v_array);

      // FIXME: remove
      for (size_t i=real_grid_size;i<grid_size_8wide_blocks*8;i++)
        {
          u_array[i] = 1.0f;
          v_array[i] = 1.0f;
        }


#if !defined(__AVX__)
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
      
      assert( grid_size_8wide_blocks >= 1 );
      for (size_t i=0;i<grid_size_8wide_blocks;i++)
	{
	  const avxf u = load8f(&u_array[i*8]);
	  const avxf v = load8f(&v_array[i*8]);

	  avx3f vtx = eval8( u, v );

#if USE_DISPLACEMENT_FOR_TESSELLATION_BOUNDS == 1
	  /* eval displacement function */
	  if (unlikely(mesh->displFunc != NULL))
	    {
	      avx3f normal = normal8(u,v);
	      normal = normalize(normal);

	      mesh->displFunc(mesh->userPtr,
			      geom,
			      prim,
			      (const float*)&u,
			      (const float*)&v,
			      (const float*)&normal,
			      (const float*)&normal,
			      (const float*)&normal,
			      (float*)&vtx.x,
			      (float*)&vtx.y,
			      (float*)&vtx.z,
			      8);

	    }
#endif
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


    /*! returns required number of primitive blocks for N primitives */
    static __forceinline size_t blocks(size_t N) { return N; }

    /*! return geometry ID */
    template<bool list>
    __forceinline unsigned int geomID() const { 
      return geom; 
    }

    /*! return primitive ID */
    template<bool list>
    __forceinline unsigned int primID() const { 
      if (list) return prim & 0x7FFFFFFF; 
      else      return prim; 
    }

    /*! checks if this is the last primitive in list leaf mode */
    __forceinline int last() const { 
      return prim & 0x80000000; 
    }

    /*! builder interface to fill primitive */
    __forceinline void fill(atomic_set<PrimRefBlock>::block_iterator_unsafe& prims, Scene* scene, const bool list)
    {
      const PrimRef& prim = *prims;
      prims++;

      //const unsigned int last   = list && !prims;
      const unsigned int geomID = prim.geomID();
      const unsigned int primID = prim.primID();
      const SubdivMesh* const subdiv_mesh = scene->getSubdivMesh(geomID);
      new (this) SubdivPatch1Cached(subdiv_mesh->getHalfEdge(primID),
                                    subdiv_mesh->getVertexPositionPtr(),
                                    geomID,
                                    primID,
                                    subdiv_mesh); 
    }

    /*! builder interface to fill primitive */
    __forceinline void fill(const PrimRef* prims, size_t& i, size_t end, Scene* scene, const bool list)
    {
      const PrimRef& prim = prims[i];
      i++;

      //const unsigned int last = list && i >= end;
      const unsigned int geomID = prim.geomID();
      const unsigned int primID = prim.primID();
      const SubdivMesh* const subdiv_mesh = scene->getSubdivMesh(geomID);
      new (this) SubdivPatch1Cached(subdiv_mesh->getHalfEdge(primID),
                                    subdiv_mesh->getVertexPositionPtr(),
                                    geomID,
                                    primID,
                                    subdiv_mesh); 
    }

    
  public:
    Vec2f u_range;
    Vec2f v_range;
    float level[4];

    unsigned int flags;
    unsigned int geom;                          //!< geometry ID of the subdivision mesh this patch belongs to
    unsigned int prim;                          //!< primitive ID of this subdivision patch
    unsigned int dummy;

    unsigned int grid_u_res;
    unsigned int grid_v_res;
    unsigned int grid_size_8wide_blocks;
    unsigned int grid_subtree_size_64b_blocks;

    __aligned(64) BSplinePatch patch;
  };

  __forceinline std::ostream &operator<<(std::ostream &o, const SubdivPatch1Cached &p)
    {
      o << " flags " << p.flags << " geomID " << p.geom << " primID " << p.prim << " u_range << " << p.u_range << " v_range " << p.v_range << " levels: " << p.level[0] << "," << p.level[1] << "," << p.level[2] << "," << p.level[3] << std::endl;
      o << " patch " << p.patch;

      return o;
    } 

}
