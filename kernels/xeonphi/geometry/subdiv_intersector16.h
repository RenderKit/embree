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

  extern size_t g_subdivision_level;


  void subdivide_intersect1(const size_t rayIndex, 
			    const mic_f &dir_xyz,
			    const mic_f &org_xyz,
			    Ray16& ray16,
			    const IrregularCatmullClarkPatch &patch,
			    const unsigned int geomID,
			    const unsigned int primID,			    
			    const unsigned int subdiv_level = 0);

  void subdivide_intersect1(const size_t rayIndex, 
			    const mic_f &dir_xyz,
			    const mic_f &org_xyz,
			    Ray16& ray16,
			    const RegularCatmullClarkPatch &patch,
			    const unsigned int geomID,
			    const unsigned int primID,
			    const unsigned int subdiv_level = 0);

  void subdivide_intersect1_eval(const size_t rayIndex, 
				 const mic_f &dir_xyz,
				 const mic_f &org_xyz,
				 Ray16& ray16,
				 const RegularCatmullClarkPatch &patch,
				 const unsigned int geomID,
				 const unsigned int primID,
				 const Vec2f &s,
				 const Vec2f &t,
				 const unsigned int subdiv_level);

  void subdivide_intersect1_eval(const size_t rayIndex, 
				 const mic_f &dir_xyz,
				 const mic_f &org_xyz,
				 Ray16& ray16,
				 const GregoryPatch &patch,
				 const unsigned int geomID,
				 const unsigned int primID,
				 const Vec2f &s,
				 const Vec2f &t,
				 const unsigned int subdiv_level);
  

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


#if 1
	if (likely(subdiv_patch.isRegular()))
	  {
	    //RegularCatmullClarkPatch regular_patch;
	    //subdiv_patch.init( regular_patch );
	    const RegularCatmullClarkPatch &regular_patch = subdiv_patch.patch;
	    regular_patch.prefetchData();
	    //subdivide_intersect1(rayIndex,dir_xyz,org_xyz,ray16,regular_patch,subdiv_patch.geomID,subdiv_patch.primID,g_subdivision_level);
	    Vec2f s_val(0.0f,1.0f);
	    Vec2f t_val(0.0f,1.0f);
	    subdivide_intersect1_eval(rayIndex,dir_xyz,org_xyz,ray16,regular_patch,subdiv_patch.geomID,subdiv_patch.primID,s_val,t_val,g_subdivision_level);
	  }
#endif
#if 1
	else if (likely(subdiv_patch.isGregoryPatch()))
	  {
	    const RegularCatmullClarkPatch &regular_patch = subdiv_patch.patch;
	    regular_patch.prefetchData();
	    __aligned(64) GregoryPatch gpatch( regular_patch.v, subdiv_patch.f_m );

	    Vec2f s_val(0.0f,1.0f);
	    Vec2f t_val(0.0f,1.0f);
	    subdivide_intersect1_eval(rayIndex,dir_xyz,org_xyz,ray16,gpatch,
				      subdiv_patch.geomID,
				      subdiv_patch.primID,
				      s_val,
				      t_val,
				      g_subdivision_level);

	  }
	  
#endif
	else
	{
	  IrregularCatmullClarkPatch irregular_patch;
	  subdiv_patch.init( irregular_patch );
	  subdivide_intersect1(rayIndex,dir_xyz,org_xyz,ray16,irregular_patch,subdiv_patch.geomID,subdiv_patch.primID,g_subdivision_level);
	}

	return true;
      };


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
	FATAL("not yet implemented");
	return false;
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

