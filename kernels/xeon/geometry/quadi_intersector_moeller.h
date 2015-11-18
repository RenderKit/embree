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

#include "quadi.h"
#include "quad_intersector_moeller.h"

/*! This intersector implements a modified version of the Moeller
 *  Trumbore intersector from the paper "Fast, Minimum Storage
 *  Ray-Triangle Intersection". In contrast to the paper we
 *  precalculate some factors and factor the calculations differently
 *  to allow precalculating the cross product e1 x e2. The resulting
 *  algorithm is similar to the fastest one of the paper "Optimizing
 *  Ray-Triangle Intersection via Automated Search". */

namespace embree
{
  namespace isa
  {
    template<int M, int Mx, bool filter>
      struct QuadMiIntersector1MoellerTrumbore;

    /*! Intersects 4 quads with 1 ray using SSE */
    template<bool filter>
      struct QuadMiIntersector1MoellerTrumbore<4,4,filter>
    {
      typedef QuadMi<4> Primitive;
      typedef MoellerTrumboreIntersectorQuad1<4> Precalculations;
        
      /*! Intersect a ray with the M quads and updates the hit. */
      static __forceinline void intersect(const Precalculations& pre, Ray& ray, const Primitive& quad, Scene* scene, const unsigned* geomID_to_instID)
      {
        STAT3(normal.trav_prims,1,1,1);
        Vec3vf4 v0, v1, v2, v3; 
        quad.gather(v0,v1,v2,v3);
        {
          Vec3vf4 vtx0(v0.x,v0.y,v0.z);
          Vec3vf4 vtx1(v1.x,v1.y,v1.z);
          Vec3vf4 vtx2(v3.x,v3.y,v3.z);
          vint4   geomIDs(quad.geomIDs); 
          vint4   primIDs(quad.primIDs);
          const vbool4 flags( false );
          pre.intersect(ray,vtx0,vtx1,vtx2,flags,Intersect1Epilog<4,4,filter>(ray,geomIDs,primIDs,scene,geomID_to_instID)); 
        }
        {
          Vec3vf4 vtx0(v2.x,v2.y,v2.z);
          Vec3vf4 vtx1(v3.x,v3.y,v3.z);
          Vec3vf4 vtx2(v1.x,v1.y,v1.z);
          vint4   geomIDs(quad.geomIDs); 
          vint4   primIDs(quad.primIDs);
          const vbool4 flags( true );
          pre.intersect(ray,vtx0,vtx1,vtx2,flags,Intersect1Epilog<4,4,filter>(ray,geomIDs,primIDs,scene,geomID_to_instID)); 
        }
      }
        
      /*! Test if the ray is occluded by one of M quads. */
      static __forceinline bool occluded(const Precalculations& pre, Ray& ray, const Primitive& quad, Scene* scene, const unsigned* geomID_to_instID)
      {
        STAT3(shadow.trav_prims,1,1,1);
        Vec3vf4 v0, v1, v2, v3; 
        quad.gather(v0,v1,v2,v3);
        {
          Vec3vf4 vtx0(v0.x,v0.y,v0.z);
          Vec3vf4 vtx1(v1.x,v1.y,v1.z);
          Vec3vf4 vtx2(v3.x,v3.y,v3.z);
          vint4   geomIDs(quad.geomIDs); 
          vint4   primIDs(quad.primIDs);
          const vbool4 flags( false );
          if (pre.intersect(ray,vtx0,vtx1,vtx2,flags,Occluded1Epilog<4,4,filter>(ray,geomIDs,primIDs,scene,geomID_to_instID)))
            return true;
        }
        {
          Vec3vf4 vtx0(v2.x,v2.y,v2.z);
          Vec3vf4 vtx1(v3.x,v3.y,v3.z);
          Vec3vf4 vtx2(v1.x,v1.y,v1.z);
          vint4   geomIDs(quad.geomIDs); 
          vint4   primIDs(quad.primIDs);
          const vbool4 flags( true );
          if (pre.intersect(ray,vtx0,vtx1,vtx2,flags,Occluded1Epilog<4,4,filter>(ray,geomIDs,primIDs,scene,geomID_to_instID)))
            return true;
        }          
        return false;
      }
    };

#if defined(__AVX__)

    /*! Intersects 4 quads with 1 ray using AVX */
    template<bool filter>
      struct QuadMiIntersector1MoellerTrumbore<4,8,filter>
    {
      typedef QuadMi<4> Primitive;
      typedef MoellerTrumboreIntersectorQuad1<8> Precalculations;
        
      /*! Intersect a ray with the M quads and updates the hit. */
      static __forceinline void intersect(const Precalculations& pre, Ray& ray, const Primitive& quad, Scene* scene, const unsigned* geomID_to_instID)
      {
        STAT3(normal.trav_prims,1,1,1);
        Vec3vf4 v0, v1, v2, v3; 
        quad.gather(v0,v1,v2,v3);

        Vec3vf8 vtx0(vfloat8(v0.x,v2.x),vfloat8(v0.y,v2.y),vfloat8(v0.z,v2.z));
        Vec3vf8 vtx1(vfloat8(v1.x),vfloat8(v1.y),vfloat8(v1.z));
        Vec3vf8 vtx2(vfloat8(v3.x),vfloat8(v3.y),vfloat8(v3.z));
        vint8   geomIDs(quad.geomIDs); 
        vint8   primIDs(quad.primIDs);
        const vbool8 flags(0,0,0,0,1,1,1,1);
        pre.intersect(ray,vtx0,vtx1,vtx2,flags,Intersect1Epilog<8,8,filter>(ray,geomIDs,primIDs,scene,geomID_to_instID)); 
      }
        
      /*! Test if the ray is occluded by one of M quads. */
      static __forceinline bool occluded(const Precalculations& pre, Ray& ray, const Primitive& quad, Scene* scene, const unsigned* geomID_to_instID)
      {
        STAT3(shadow.trav_prims,1,1,1);
        Vec3vf4 v0, v1, v2, v3; 
        quad.gather(v0,v1,v2,v3);

        Vec3vf8 vtx0(vfloat8(v0.x,v2.x),vfloat8(v0.y,v2.y),vfloat8(v0.z,v2.z));
        Vec3vf8 vtx1(vfloat8(v1.x),vfloat8(v1.y),vfloat8(v1.z));
        Vec3vf8 vtx2(vfloat8(v3.x),vfloat8(v3.y),vfloat8(v3.z));
        vint8   geomIDs(quad.geomIDs); 
        vint8   primIDs(quad.primIDs);
        const vbool8 flags(0,0,0,0,1,1,1,1);
        return pre.intersect(ray,vtx0,vtx1,vtx2,flags,Occluded1Epilog<8,8,filter>(ray,geomIDs,primIDs,scene,geomID_to_instID)); 
      }
    };

#endif


  }
}

