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
        quad.gather(v0,v1,v2,v3,scene);
        const vint4 geomIDs(quad.geomIDs); 
        const vint4 primIDs(quad.primIDs);
        pre.intersect(ray,v0,v1,v3,vbool4(false),Intersect1Epilog<4,4,filter>(ray,geomIDs,primIDs,scene,geomID_to_instID)); 
        pre.intersect(ray,v2,v3,v1,vbool4( true),Intersect1Epilog<4,4,filter>(ray,geomIDs,primIDs,scene,geomID_to_instID)); 
      }
        
      /*! Test if the ray is occluded by one of M quads. */
      static __forceinline bool occluded(const Precalculations& pre, Ray& ray, const Primitive& quad, Scene* scene, const unsigned* geomID_to_instID)
      {
        STAT3(shadow.trav_prims,1,1,1);
        Vec3vf4 v0, v1, v2, v3; 
        quad.gather(v0,v1,v2,v3,scene);
        const vint4 geomIDs(quad.geomIDs); 
        const vint4 primIDs(quad.primIDs);
        if (pre.intersect(ray,v0,v1,v3,vbool4(false),Occluded1Epilog<4,4,filter>(ray,geomIDs,primIDs,scene,geomID_to_instID))) return true;
        if (pre.intersect(ray,v2,v3,v1,vbool4(true ),Occluded1Epilog<4,4,filter>(ray,geomIDs,primIDs,scene,geomID_to_instID))) return true;
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
        quad.gather(v0,v1,v2,v3,scene);
        const Vec3vf8 vtx0(vfloat8(v0.x,v2.x),vfloat8(v0.y,v2.y),vfloat8(v0.z,v2.z));
        const Vec3vf8 vtx1(vfloat8(v1.x),vfloat8(v1.y),vfloat8(v1.z));
        const Vec3vf8 vtx2(vfloat8(v3.x),vfloat8(v3.y),vfloat8(v3.z));
        const vint8   geomIDs(quad.geomIDs); 
        const vint8   primIDs(quad.primIDs);
        const vbool8 flags(0,0,0,0,1,1,1,1);
        pre.intersect(ray,vtx0,vtx1,vtx2,flags,Intersect1Epilog<8,8,filter>(ray,geomIDs,primIDs,scene,geomID_to_instID)); 
      }
        
      /*! Test if the ray is occluded by one of M quads. */
      static __forceinline bool occluded(const Precalculations& pre, Ray& ray, const Primitive& quad, Scene* scene, const unsigned* geomID_to_instID)
      {
        STAT3(shadow.trav_prims,1,1,1);
        Vec3vf4 v0, v1, v2, v3; 
        quad.gather(v0,v1,v2,v3,scene);
        const Vec3vf8 vtx0(vfloat8(v0.x,v2.x),vfloat8(v0.y,v2.y),vfloat8(v0.z,v2.z));
        const Vec3vf8 vtx1(vfloat8(v1.x),vfloat8(v1.y),vfloat8(v1.z));
        const Vec3vf8 vtx2(vfloat8(v3.x),vfloat8(v3.y),vfloat8(v3.z));
        const vint8   geomIDs(quad.geomIDs); 
        const vint8   primIDs(quad.primIDs);
        const vbool8 flags(0,0,0,0,1,1,1,1);
        return pre.intersect(ray,vtx0,vtx1,vtx2,flags,Occluded1Epilog<8,8,filter>(ray,geomIDs,primIDs,scene,geomID_to_instID)); 
      }
    };

#endif

#if defined(__AVX512F__)

    /*! Intersects 4 quads with 1 ray using AVX */
    template<bool filter>
      struct QuadMiIntersector1MoellerTrumbore<4,16,filter>
    {
      typedef QuadMi<4> Primitive;
      typedef MoellerTrumboreIntersectorQuad1<16> Precalculations;
        
      /*! Intersect a ray with the M quads and updates the hit. */
      static __forceinline void intersect(const Precalculations& pre, Ray& ray, const Primitive& quad, Scene* scene, const unsigned* geomID_to_instID)
      {
        STAT3(normal.trav_prims,1,1,1);
        Vec3vf4 v0, v1, v2, v3; 
        quad.gather(v0,v1,v2,v3,scene);
        Vec3vf16 vtx0(select(0x0f0f,vfloat16(quad.v0.x),vfloat16(quad.v2.x)),
                      select(0x0f0f,vfloat16(quad.v0.y),vfloat16(quad.v2.y)),
                      select(0x0f0f,vfloat16(quad.v0.z),vfloat16(quad.v2.z)));
        Vec3vf16 vtx1(vfloat16(quad.v1.x),vfloat16(quad.v1.y),vfloat16(quad.v1.z));
        Vec3vf16 vtx2(vfloat16(quad.v3.x),vfloat16(quad.v3.y),vfloat16(quad.v3.z));
        vint8   geomIDs(quad.geomIDs); 
        vint8   primIDs(quad.primIDs);        
        const vbool16 flags(0xf0f0);
        pre.intersect(ray,vtx0,vtx1,vtx2,flags,Intersect1Epilog<8,16,filter>(ray,geomIDs,primIDs,scene,geomID_to_instID)); 
      }
        
      /*! Test if the ray is occluded by one of M quads. */
      static __forceinline bool occluded(const Precalculations& pre, Ray& ray, const Primitive& quad, Scene* scene, const unsigned* geomID_to_instID)
      {
        STAT3(shadow.trav_prims,1,1,1);
        Vec3vf4 v0, v1, v2, v3; 
        quad.gather(v0,v1,v2,v3,scene);
        Vec3vf16 vtx0(select(0x0f0f,vfloat16(quad.v0.x),vfloat16(quad.v2.x)),
                      select(0x0f0f,vfloat16(quad.v0.y),vfloat16(quad.v2.y)),
                      select(0x0f0f,vfloat16(quad.v0.z),vfloat16(quad.v2.z)));
        Vec3vf16 vtx1(vfloat16(quad.v1.x),vfloat16(quad.v1.y),vfloat16(quad.v1.z));
        Vec3vf16 vtx2(vfloat16(quad.v3.x),vfloat16(quad.v3.y),vfloat16(quad.v3.z));
        vint8   geomIDs(quad.geomIDs); 
        vint8   primIDs(quad.primIDs);        
        const vbool16 flags(0xf0f0);
        return pre.intersect(ray,vtx0,vtx1,vtx2,flags,Occluded1Epilog<8,16,filter>(ray,geomIDs,primIDs,scene,geomID_to_instID)); 
      }
    };

#endif


    /*! Intersects M triangles with K rays. */
    template<int M, int K, bool filter>
      struct QuadMiIntersectorKMoellerTrumbore
      {
        typedef QuadMi<M> Primitive;
        typedef MoellerTrumboreIntersectorQuadMvK<2*M,K> Precalculations;
        
        /*! Intersects K rays with M triangles. */
        static __forceinline void intersect(const vbool<K>& valid_i, Precalculations& pre, RayK<K>& ray, const QuadMi<M>& quad, Scene* scene)
        {
          for (size_t i=0; i<QuadMi<M>::max_size(); i++)
          {
            if (!quad.valid(i)) break;
            STAT3(normal.trav_prims,1,popcnt(valid_i),K);
            const Vec3<vfloat<K>> p0 = quad.getVertexK<K>(quad.v0,i,scene);
            const Vec3<vfloat<K>> p1 = quad.getVertexK<K>(quad.v1,i,scene);
            const Vec3<vfloat<K>> p2 = quad.getVertexK<K>(quad.v2,i,scene);
            const Vec3<vfloat<K>> p3 = quad.getVertexK<K>(quad.v3,i,scene);
            pre.intersectK(valid_i,ray,p0,p1,p3,vbool<K>(false),IntersectKEpilog<M,K,filter>(ray,quad.geomIDs,quad.primIDs,i,scene));
            pre.intersectK(valid_i,ray,p2,p3,p1,vbool<K>(true ),IntersectKEpilog<M,K,filter>(ray,quad.geomIDs,quad.primIDs,i,scene));
          }
        }
        
        /*! Test for K rays if they are occluded by any of the M triangles. */
        static __forceinline vbool<K> occluded(const vbool<K>& valid_i, Precalculations& pre, RayK<K>& ray, const QuadMi<M>& quad, Scene* scene)
        {
          vbool<K> valid0 = valid_i;
          for (size_t i=0; i<QuadMi<M>::max_size(); i++)
          {
            if (!quad.valid(i)) break;
            STAT3(shadow.trav_prims,1,popcnt(valid0),K);
            const Vec3<vfloat<K>> p0 = quad.getVertexK<K>(quad.v0,i,scene);
            const Vec3<vfloat<K>> p1 = quad.getVertexK<K>(quad.v1,i,scene);
            const Vec3<vfloat<K>> p2 = quad.getVertexK<K>(quad.v2,i,scene);
            const Vec3<vfloat<K>> p3 = quad.getVertexK<K>(quad.v3,i,scene);
            pre.intersectK(valid0,ray,p0,p1,p3,vbool<K>(false),OccludedKEpilog<M,K,filter>(valid0,ray,quad.geomIDs,quad.primIDs,i,scene));
            if (none(valid0)) break;
            pre.intersectK(valid0,ray,p2,p3,p1,vbool<K>(true ),OccludedKEpilog<M,K,filter>(valid0,ray,quad.geomIDs,quad.primIDs,i,scene));
            if (none(valid0)) break;
          }
          return !valid0;
        }
        
        /*! Intersect a ray with M triangles and updates the hit. */
        static __forceinline void intersect(Precalculations& pre, RayK<K>& ray, size_t k, const QuadMi<M>& quad, Scene* scene)
        {
          STAT3(normal.trav_prims,1,1,1);
          Vec3vf4 v0, v1, v2, v3; 
          quad.gather(v0,v1,v2,v3,scene);

          Vec3<vfloat<2*M>> vtx0(vfloat<2*M>(v0.x,v2.x),
                                 vfloat<2*M>(v0.y,v2.y),
                                 vfloat<2*M>(v0.z,v2.z));
          Vec3<vfloat<2*M>> vtx1(vfloat<2*M>(v1.x),
                                 vfloat<2*M>(v1.y),
                                 vfloat<2*M>(v1.z));
          Vec3<vfloat<2*M>> vtx2(vfloat<2*M>(v3.x),
                                 vfloat<2*M>(v3.y),
                                 vfloat<2*M>(v3.z));
          vint<2*M> geomIDs(quad.geomIDs); 
          vint<2*M> primIDs(quad.primIDs);
          vbool<2*M> flags(0,1);
          pre.intersect1(ray,k,vtx0,vtx1,vtx2,flags,Intersect1KEpilog<2*M,2*M,K,filter>(ray,k,geomIDs,primIDs,scene)); 
        }
        
        /*! Test if the ray is occluded by one of the M triangles. */
        static __forceinline bool occluded(Precalculations& pre, RayK<K>& ray, size_t k, const QuadMi<M>& quad, Scene* scene)
        {
          STAT3(shadow.trav_prims,1,1,1);
          Vec3vf4 v0, v1, v2, v3; 
          quad.gather(v0,v1,v2,v3,scene);

          Vec3<vfloat<2*M>> vtx0(vfloat<2*M>(v0.x,v2.x),
                                 vfloat<2*M>(v0.y,v2.y),
                                 vfloat<2*M>(v0.z,v2.z));
          Vec3<vfloat<2*M>> vtx1(vfloat<2*M>(v1.x),
                                 vfloat<2*M>(v1.y),
                                 vfloat<2*M>(v1.z));
          Vec3<vfloat<2*M>> vtx2(vfloat<2*M>(v3.x),
                                 vfloat<2*M>(v3.y),
                                 vfloat<2*M>(v3.z));
          vint<2*M> geomIDs(quad.geomIDs); 
          vint<2*M> primIDs(quad.primIDs);
          vbool<2*M> flags(0,1);
          return pre.intersect1(ray,k,vtx0,vtx1,vtx2,flags,Occluded1KEpilog<2*M,2*M,K,filter>(ray,k,geomIDs,primIDs,scene)); 
        }
      };

  }
}

