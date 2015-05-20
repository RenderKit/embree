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

#include "subdivpatch1_intersector1.h"

namespace embree
{
  namespace isa
  {  
    static __forceinline void intersectTri(const Vec3fa& tri_v0,
                                           const Vec3fa& tri_v1,
                                           const Vec3fa& tri_v2,
                                           Ray& ray, 
                                           const unsigned int geomID,
                                           const unsigned int primID,
                                           Scene* scene)
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
#if defined(RTCORE_BACKFACE_CULLING)
      if (unlikely(den <= 0.0f)) return;
#else
      if (unlikely(den == 0.0f)) return;
#endif
      
      /* ray masking test */
#if 0 && defined(RTCORE_RAY_MASK) // FIXME: enable
      if (unlikely((tri.mask() & ray.mask) == 0)) return;
#endif
      
      /* calculate hit information */
      const float rcpAbsDen = rcp(absDen);
      const float u = U * rcpAbsDen;
      const float v = V * rcpAbsDen;
      const float t = T * rcpAbsDen;
      
      /* intersection filter test */
#if 0 && defined(RTCORE_INTERSECTION_FILTER) // FIXME: enable
      Geometry* geometry = scene->get(geomID);
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
                                          Scene* scene)
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
#if defined(RTCORE_BACKFACE_CULLING)
      if (unlikely(den <= 0.0f)) return false;
#else
      if (unlikely(den == 0.0f)) return false;
#endif
      
      /* ray masking test */
#if 0 && defined(RTCORE_RAY_MASK) // FIXME: enable
      if (unlikely((tri.mask() & ray.mask) == 0)) return false;
#endif
      
      /* intersection filter test */
#if 0 && defined(RTCORE_INTERSECTION_FILTER) // FIXME: enable
      const int geomID = tri.geomID<list>();
      Geometry* geometry = scene->get(geomID);
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

    void SubdivPatch1Intersector1::subdivide_intersect1(const Precalculations& pre,
                                                        Ray& ray,
                                                        const GeneralCatmullClarkPatch3fa &patch,
                                                        const unsigned int geomID,
                                                        const unsigned int primID,
                                                        const unsigned int subdiv_level)
    {
      size_t N;
      array_t<CatmullClarkPatch3fa,GeneralCatmullClarkPatch3fa::SIZE> patches; 
      patch.subdivide(patches,N);

      for (size_t i=0; i<N; i++)
        if (intersectBounds(pre,ray,patches[i].bounds()))
          subdivide_intersect1(pre,ray,patches[i],geomID,primID,subdiv_level - 1);	    
    }

    void SubdivPatch1Intersector1::subdivide_intersect1(const Precalculations& pre,
                                                        Ray& ray,
                                                        const CatmullClarkPatch3fa& patch,
                                                        const unsigned int geomID,
                                                        const unsigned int primID,
                                                        const unsigned int subdiv_level)
    {
      if (subdiv_level == 0)
      {
        __aligned(64) FinalQuad finalQuad;
        patch.init( finalQuad );
        intersectTri(finalQuad.vtx[0],finalQuad.vtx[1],finalQuad.vtx[2],ray,geomID,primID,nullptr); 
        intersectTri(finalQuad.vtx[2],finalQuad.vtx[3],finalQuad.vtx[0],ray,geomID,primID,nullptr); 
      }
      else
      {
        array_t<CatmullClarkPatch3fa,4> subpatches;
        patch.subdivide(subpatches);
        for (size_t i=0;i<4;i++)
          if (intersectBounds(pre,ray,subpatches[i].bounds()))
            subdivide_intersect1(pre,ray,subpatches[i],geomID,primID,subdiv_level - 1);
      }   
    }

    bool SubdivPatch1Intersector1::subdivide_occluded1(const Precalculations& pre,
                                                       Ray& ray,
                                                       const GeneralCatmullClarkPatch3fa& patch,
                                                       const unsigned int geomID,
                                                       const unsigned int primID,
                                                       const unsigned int subdiv_level)
    {
      size_t N;
      array_t<CatmullClarkPatch3fa,GeneralCatmullClarkPatch3fa::SIZE> patches; 
      patch.subdivide(patches,N);

      for (size_t i=0; i<N; i++)
        if (intersectBounds(pre,ray,patches[i].bounds()))
          if (subdivide_occluded1(pre,ray,patches[i],geomID,primID,subdiv_level - 1))
            return true;

      return false;
    }

    bool SubdivPatch1Intersector1::subdivide_occluded1(const Precalculations& pre,
                                                       Ray& ray,
                                                       const CatmullClarkPatch3fa &patch,
                                                       const unsigned int geomID,
                                                       const unsigned int primID,						     
                                                       const unsigned int subdiv_level)
    {
      if (subdiv_level == 0)
      {
        __aligned(64) FinalQuad finalQuad;
        patch.init( finalQuad );
        if (occludedTri(finalQuad.vtx[0],finalQuad.vtx[1],finalQuad.vtx[2],ray,geomID,primID,nullptr)) return true; 
        if (occludedTri(finalQuad.vtx[2],finalQuad.vtx[3],finalQuad.vtx[0],ray,geomID,primID,nullptr)) return false;
      }
      else
      {
        array_t<CatmullClarkPatch3fa,4> subpatches;
        patch.subdivide(subpatches);
        for (size_t i=0;i<4;i++)
          if (intersectBounds(pre,ray,subpatches[i].bounds()))
            if (subdivide_occluded1(pre,ray,subpatches[i],geomID,primID,subdiv_level - 1)) 
              return true;
      }   
      return false;
    }
  }
}
