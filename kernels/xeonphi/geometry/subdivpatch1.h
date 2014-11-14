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

#define FORCE_TESSELLATION_BOUNDS 0

namespace embree
{

  struct SubdivPatch1
  {
  public:
    enum {
      REGULAR_PATCH = 1 // = 0 => Gregory Patch 
    };

    /*! Default constructor. */
    __forceinline SubdivPatch1 (const SubdivMesh::HalfEdge * first_half_edge,
				const Vec3fa *vertices,
				unsigned int geomID,
				unsigned int primID) 
      : geomID(geomID),
      primID(primID),
      under_construction(0),
      bvh4i_subtree_root((unsigned int)-1),
      flags(0)
    {
      assert(sizeof(SubdivPatch1) == 6 * 64);

      u_range = Vec2f(0.0f,1.0f);
      v_range = Vec2f(0.0f,1.0f);

      f_m[0][0] = 0.0f;
      f_m[0][1] = 0.0f;
      f_m[1][1] = 0.0f;
      f_m[1][0] = 0.0f;

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
      grid_size = (grid_u_res*grid_v_res+15)&(-16);

      assert(grid_size > 0);
      assert(grid_size % 16 == 0);

      /* compute 16-bit quad mask for quad-tessellation */

      mic_m m_active = 0xffff;
      for (unsigned int i=grid_u_res-1;i<16;i+=grid_u_res)
	m_active ^= (unsigned int)1 << i;
      m_active &= ((unsigned int)1 << (grid_u_res * (grid_v_res-1)))-1;
      grid_mask = m_active;

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
	  gpatch.exportConrolPoints( patch.v, f_m );

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
	  prefetch<PFHINT_L1>(f_m);
	  return GregoryPatch::eval16( patch.v, f_m, uu, vv );
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
	  prefetch<PFHINT_L1>(f_m);
	  return GregoryPatch::eval4( patch.v, f_m, uu, vv );
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

    __forceinline BBox3fa bounds() const
    {
      //FIXME: always enable?
#if FORCE_TESSELLATION_BOUNDS == 1
      BBox3fa b;
      if (unlikely(isGregoryPatch()))
	{
	  GregoryPatch gpatch( patch.v, f_m);
	  b = gpatch.getDiscreteTessellationBounds(level);
	}
      else
	b = patch.getDiscreteTessellationBounds(level);    

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
	  b.extend( f_m[0][0] );
	  b.extend( f_m[0][1] );
	  b.extend( f_m[1][0] );
	  b.extend( f_m[1][1] );
	}
#endif
      return b;
    }
   
    Vec2f u_range;
    Vec2f v_range;
    float level[4];

    unsigned int flags;
    unsigned int geomID;                          //!< geometry ID of the subdivision mesh this patch belongs to
    unsigned int primID;                          //!< primitive ID of this subdivision patch
    unsigned int bvh4i_subtree_root;

    unsigned short grid_u_res;
    unsigned short grid_v_res;
    unsigned int   grid_size;
    unsigned int   grid_mask;
    volatile unsigned int under_construction; // 0 = not build yet, 1 = under construction, 2 = built

    __aligned(64) RegularCatmullClarkPatch patch;
    Vec3fa f_m[2][2];    
  };

  __forceinline std::ostream &operator<<(std::ostream &o, const SubdivPatch1 &p)
    {
      o << " flags " << p.flags << " geomID " << p.geomID << " primID " << p.primID << " u_range << " << p.u_range << " v_range " << p.v_range << " levels: " << p.level[0] << "," << p.level[1] << "," << p.level[2] << "," << p.level[3];

      return o;
    } 

};

