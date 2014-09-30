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
#include "common/ray.h"
#include "geometry/filter.h"

namespace embree
{

  static __forceinline void intersectTri(const Vec3fa& tri_v0,
                                         const Vec3fa& tri_v1,
                                         const Vec3fa& tri_v2,
                                         Ray& ray, 
                                         const unsigned int geomID,
                                         const unsigned int primID,
                                         const void* geom)
    {
      /* load triangle */
      STAT3(normal.trav_prims,1,1,1);
 
      /* calculate vertices relative to ray origin */
      const Vec3fa O = ray.org;
      const Vec3fa D = ray.dir;
      const Vec3fa v0 = tri_v0-O;
      const Vec3fa v1 = tri_v1-O;
      const Vec3fa v2 = tri_v2-O;

      /* calculate triangle edges */
      const Vec3fa e0 = v2-v0;
      const Vec3fa e1 = v0-v1;
      const Vec3fa e2 = v1-v2;

      /* calculate geometry normal and denominator */
      const Vec3fa Ng1 = cross(e1,e0);
      const Vec3fa Ng = Ng1+Ng1;
      const float den = dot(Ng,D);
      const float absDen = abs(den);
      const float sgnDen = signmsk(den);

      /* perform edge tests */
      const float U = xorf(dot(cross(v2+v0,e0),D),sgnDen);
      if (unlikely(U < 0.0f)) return;
      const float V = xorf(dot(cross(v0+v1,e1),D),sgnDen);
      if (unlikely(V < 0.0f)) return;
      const float W = xorf(dot(cross(v1+v2,e2),D),sgnDen);
      if (unlikely(W < 0.0f)) return;
      
      /* perform depth test */
      const float T = xorf(dot(v0,Ng),sgnDen);
      if (unlikely(absDen*float(ray.tfar) < T)) return;
      if (unlikely(T < absDen*float(ray.tnear))) return;

      /* perform backface culling */
#if defined(__BACKFACE_CULLING__)
      if (unlikely(den <= 0.0f)) return;
#else
      if (unlikely(den == 0.0f)) return;
#endif

      /* ray masking test */
#if 0 && defined(__USE_RAY_MASK__) // FIXME: enable
      if (unlikely((tri.mask() & ray.mask) == 0)) return;
#endif

      /* calculate hit information */
      const float rcpAbsDen = rcp(absDen);
      const float u = U * rcpAbsDen;
      const float v = V * rcpAbsDen;
      const float t = T * rcpAbsDen;
      
      /* intersection filter test */
#if 0 && defined(__INTERSECTION_FILTER__) // FIXME: enable
      Geometry* geometry = ((Scene*)geom)->get(geomID);
      if (unlikely(geometry->hasIntersectionFilter1())) {
        runIntersectionFilter1(geometry,ray,u,v,t,Ng,geomID,primID);
        return;
      }
#endif

      /* update hit information */
      ray.u = u;
      ray.v = v;
      ray.tfar = t;
      ray.Ng  = Ng;
      ray.geomID = geomID;
      ray.primID = primID;
    }

  static __forceinline bool occludedTri(const Vec3fa& tri_v0,
					const Vec3fa& tri_v1,
					const Vec3fa& tri_v2,
					Ray& ray, 
					const unsigned int geomID,
					const unsigned int primID,
					const void* geom)
    {
      /* load triangle */
      STAT3(normal.trav_prims,1,1,1);
 
      /* calculate vertices relative to ray origin */
      const Vec3fa O = ray.org;
      const Vec3fa D = ray.dir;
      const Vec3fa v0 = tri_v0-O;
      const Vec3fa v1 = tri_v1-O;
      const Vec3fa v2 = tri_v2-O;

      /* calculate triangle edges */
      const Vec3fa e0 = v2-v0;
      const Vec3fa e1 = v0-v1;
      const Vec3fa e2 = v1-v2;

      /* calculate geometry normal and denominator */
      const Vec3fa Ng1 = cross(e1,e0);
      const Vec3fa Ng = Ng1+Ng1;
      const float den = dot(Ng,D);
      const float absDen = abs(den);
      const float sgnDen = signmsk(den);

      /* perform edge tests */
      const float U = xorf(dot(cross(v2+v0,e0),D),sgnDen);
      if (unlikely(U < 0.0f)) return false;
      const float V = xorf(dot(cross(v0+v1,e1),D),sgnDen);
      if (unlikely(V < 0.0f)) return false;
      const float W = xorf(dot(cross(v1+v2,e2),D),sgnDen);
      if (unlikely(W < 0.0f)) return false;
      
      /* perform depth test */
      const float T = xorf(dot(v0,Ng),sgnDen);
      if (unlikely(absDen*float(ray.tfar) < T)) return false;
      if (unlikely(T < absDen*float(ray.tnear))) return false;

      /* perform backface culling */
#if defined(__BACKFACE_CULLING__)
      if (unlikely(den <= 0.0f)) return false;
#else
      if (unlikely(den == 0.0f)) return false;
#endif

      /* ray masking test */
#if 0 && defined(__USE_RAY_MASK__) // FIXME: enable
      if (unlikely((tri.mask() & ray.mask) == 0)) return false;
#endif

      /* intersection filter test */
#if 0 && defined(__INTERSECTION_FILTER__) // FIXME: enable
      const int geomID = tri.geomID<list>();
      Geometry* geometry = ((Scene*)geom)->get(geomID);
      if (unlikely(geometry->hasOcclusionFilter1()))
      {
        /* calculate hit information */
        const float rcpAbsDen = rcp(absDen);
        const float u = U*rcpAbsDen;
        const float v = V*rcpAbsDen;
        const float t = T*rcpAbsDen;
        const int primID = tri.primID<list>();
        return runOcclusionFilter1(geometry,ray,u,v,t,Ng,geomID,primID);
      }
#endif
      return true;
    }

  void subdivide_intersect1(Ray& ray,
			    const IrregularCatmullClarkPatch &patch,
			    const unsigned int subdiv_level = 0);

  bool subdivide_occluded1(Ray& ray,
			   const IrregularCatmullClarkPatch &patch,
			   const unsigned int subdiv_level = 0);

  void subdivide_intersect1(Ray& ray,
			    const RegularCatmullClarkPatch &patch,
			    const unsigned int subdiv_level = 0);

  bool subdivide_occluded1(Ray& ray,
			   const RegularCatmullClarkPatch &patch,
			   const unsigned int subdiv_level = 0);

  extern size_t g_subdivision_level;

  template<bool list>
  struct SubdivPatch1Intersector1
  {
    typedef SubdivPatch1 Primitive;

    struct Precalculations {
      __forceinline Precalculations (const Ray& ray) {}
    };

    /*! Intersect a ray with the primitive. */
    static __forceinline void intersect(const Precalculations& pre, Ray& ray, const Primitive& subdiv_patch, const void* geom)
    {
      STAT3(normal.trav_prims,1,1,1);

#if 1 // not working properly, disabled for now
      if (subdiv_patch.isRegular())
	{
	   /* IrregularCatmullClarkPatch irregular_patch;  */
	   /* subdiv_patch.init( irregular_patch );  */

	   /* std::cout << irregular_patch.ring[0] << std::endl;  */

	  RegularCatmullClarkPatch regular_patch;
	  subdiv_patch.init( regular_patch );
	  subdivide_intersect1(ray,regular_patch,g_subdivision_level);

	  /* std::cout << regular_patch << std::endl;  */

	  /*  FATAL("COMPARE 1-RINGS");  */
	}
      else
#endif
	{
	  IrregularCatmullClarkPatch irregular_patch;
	  subdiv_patch.init( irregular_patch );
	  subdivide_intersect1(ray,irregular_patch,g_subdivision_level);
	}
    }

    /*! Test if the ray is occluded by the primitive */
    static __forceinline bool occluded(const Precalculations& pre, Ray& ray, const Primitive& subdiv_patch, const void* geom)
    {
      STAT3(shadow.trav_prims,1,1,1);

      IrregularCatmullClarkPatch irregular_patch;
      subdiv_patch.init( irregular_patch );

      return subdivide_occluded1(ray,irregular_patch,g_subdivision_level);
    }
  };
}
