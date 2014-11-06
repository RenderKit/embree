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

namespace embree
{

  struct SubdivPatch1
  {
  public:
    enum {
      REGULAR_PATCH = 1,
      HAS_BORDERS   = 2,
      HAS_CREASES   = 4,
      GREGORY_PATCH = 8
    };

    /*! Default constructor. */
    __forceinline SubdivPatch1 (const SubdivMesh::HalfEdge * first_half_edge,
				const Vec3fa *vertices,
				unsigned int geomID,
				unsigned int primID,
				unsigned int subdivision_level = 0) 
      : geomID(geomID),
      primID(primID),
      subdivision_level(subdivision_level),
      bvh4i_parent_ref(0),
      bvh4i_parent_local_index(0),
      under_construction(0),
      bvh4i_subtree_root((unsigned int)-1)
    {
      u_range = Vec2f(0.0f,1.0f);
      v_range = Vec2f(0.0f,1.0f);

      f_m[0][0] = 0.0f;
      f_m[0][1] = 0.0f;
      f_m[1][1] = 0.0f;
      f_m[1][0] = 0.0f;


      IrregularCatmullClarkPatch ipatch ( first_half_edge, vertices ); 

#if 0
      DBG_PRINT( ipatch );

      DBG_PRINT( ipatch.getLimitVertex( 0 ) );
      DBG_PRINT( ipatch.getLimitVertex( 1 ) );
      DBG_PRINT( ipatch.getLimitVertex( 2 ) );
      DBG_PRINT( ipatch.getLimitVertex( 3 ) );

      DBG_PRINT( ipatch.getLimitTangent( 0 ) );
      DBG_PRINT( ipatch.getSecondLimitTangent( 0 ) );

      DBG_PRINT( ipatch.getLimitTangent( 1 ) );
      DBG_PRINT( ipatch.getSecondLimitTangent( 1 ) );

      DBG_PRINT( ipatch.getLimitTangent( 2 ) );
      DBG_PRINT( ipatch.getSecondLimitTangent( 2 ) );

      DBG_PRINT( ipatch.getLimitTangent( 3 ) );
      DBG_PRINT( ipatch.getSecondLimitTangent( 3 ) );
      //exit(0);

#endif

      flags = 0;
      if (ipatch.dicable()) 
	{
	  flags |= REGULAR_PATCH;
	  patch.init( ipatch );
	}
      else
	{
	  flags |= GREGORY_PATCH;

	  GregoryPatch gpatch; 
	  gpatch.init( ipatch ); 
	  gpatch.exportConrolPoints( patch.v, f_m );

	}
    }

    __forceinline bool isRegular() const
    {
      return (flags & REGULAR_PATCH) == REGULAR_PATCH;
    }

    __forceinline bool isGregoryPatch() const
    {
      return (flags & GREGORY_PATCH) == GREGORY_PATCH;
    }

    __forceinline BBox3fa bounds() const
    {
      BBox3fa b = patch.bounds();
      if (unlikely(isGregoryPatch()))
	{
	  b.extend( f_m[0][0] );
	  b.extend( f_m[0][1] );
	  b.extend( f_m[1][0] );
	  b.extend( f_m[1][1] );
	}
      return b;
    }

    __forceinline BBox3fa evalQuadBounds(const float s0 = 0.0f,
					 const float s1 = 1.0f,
					 const float t0 = 0.0f,
					 const float t1 = 1.0f) const
    {
      Vec3fa vtx[4];
      if (likely(isRegular()))
	{
	  vtx[0] = patch.eval(s0,t0);
	  vtx[1] = patch.eval(s1,t0);
	  vtx[2] = patch.eval(s1,t1);
	  vtx[3] = patch.eval(s0,t1);

	}
      else if (likely(isGregoryPatch()))
	{
	  __aligned(64) GregoryPatch gpatch(patch.v, f_m );
	  vtx[0] = gpatch.eval(s0,t0);
	  vtx[1] = gpatch.eval(s1,t0);
	  vtx[2] = gpatch.eval(s1,t1);
	  vtx[3] = gpatch.eval(s0,t1);
	}
      else
	FATAL("not implemented");

      BBox3fa b( empty );
      b.extend( vtx[0] );
      b.extend( vtx[1] );
      b.extend( vtx[2] );
      b.extend( vtx[3] );
      return b;
    }
   
    //const SubdivMesh::HalfEdge * first_half_edge; //!< pointer to first half edge of corresponding quad in the subdivision mesh
    //const Vec3fa *vertices;                       //!< pointer to the vertex positions in the subdivison mesh
    Vec2f u_range;
    Vec2f v_range;
    unsigned int flags;
    unsigned int subdivision_level;
    unsigned int geomID;                          //!< geometry ID of the subdivision mesh this patch belongs to
    unsigned int primID;                          //!< primitive ID of this subdivision patch

    unsigned int bvh4i_subtree_root;
    unsigned int bvh4i_parent_ref;
    unsigned int bvh4i_parent_local_index;
    volatile unsigned int under_construction; // 0 = not build yet, 1 = under construction, 2 = built
    __aligned(64) RegularCatmullClarkPatch patch;
    Vec3fa f_m[2][2];    
  };

  __forceinline std::ostream &operator<<(std::ostream &o, const SubdivPatch1 &p)
    {
      o << " flags " << p.flags << " geomID " << p.geomID << " primID " << p.primID << " u_range << " << p.u_range << " v_range " << p.v_range;

      return o;
    } 

};

