// ======================================================================== //
// Copyright 2009-2018 Intel Corporation                                    //
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

#include "subgrid_intersector.h"

namespace embree
{
  namespace isa
  {

    /*! Intersects M quads with 1 ray */
    template<int N, bool filter>
    struct SubGridMBIntersector1Moeller
    {
      //typedef SubGrid Primitive;
      typedef SubGridMBQBVHN<N> Primitive;
      typedef SubGridQuadMIntersector1MoellerTrumbore<4,filter> Precalculations;

      /*! Intersect a ray with the M quads and updates the hit. */
      static __forceinline void intersect(const Precalculations& pre, RayHit& ray, IntersectContext* context, const SubGrid& subgrid)
      {
        STAT3(normal.trav_prims,1,1,1);
        const GridMesh* mesh    = context->scene->get<GridMesh>(subgrid.geomID());
        const GridMesh::Grid &g = mesh->grid(subgrid.primID());

        float ftime;
        const int itime = getTimeSegment(ray.time(), float(mesh->fnumTimeSegments), ftime);
        Vec3vf4 v0,v1,v2,v3; subgrid.gatherMB(v0,v1,v2,v3,context->scene,itime,ftime);
        pre.intersect(ray,context,v0,v1,v2,v3,g,subgrid);
      }

      /*! Test if the ray is occluded by one of M subgrids. */
      static __forceinline bool occluded(const Precalculations& pre, Ray& ray, IntersectContext* context, const SubGrid& subgrid)
      {
        STAT3(shadow.trav_prims,1,1,1);
        const GridMesh* mesh    = context->scene->get<GridMesh>(subgrid.geomID());
        const GridMesh::Grid &g = mesh->grid(subgrid.primID());

        float ftime;
        const int itime = getTimeSegment(ray.time(), float(mesh->fnumTimeSegments), ftime);

        Vec3vf4 v0,v1,v2,v3; subgrid.gatherMB(v0,v1,v2,v3,context->scene,itime,ftime);
        return pre.occluded(ray,context,v0,v1,v2,v3,g,subgrid);
      }

      template<int Nx, bool robust>
        static __forceinline void intersect(const Accel::Intersectors* This, Precalculations& pre, RayHit& ray, IntersectContext* context, const Primitive* prim, size_t num, const TravRay<N,Nx,robust> &tray, size_t& lazy_node)
      {
        for (size_t i=0;i<num;i++)
        {
          vfloat<Nx> dist;
          /* QBVH intersection test */
          size_t mask = intersectNode(&prim[i].qnode,tray,ray.time(),dist); 
#if defined(__AVX__)
          STAT3(normal.trav_hit_boxes[popcnt(mask)],1,1,1);
#endif
          while(mask != 0)
          {
            const size_t ID = bscf(mask); 
            if (unlikely(dist[ID] > ray.tfar)) continue;
            intersect(pre,ray,context,prim[i].subgrid(ID));
          }
        }
      }

      template<int Nx, bool robust>        
        static __forceinline bool occluded(const Accel::Intersectors* This, Precalculations& pre, Ray& ray, IntersectContext* context, const Primitive* prim, size_t num, const TravRay<N,Nx,robust> &tray, size_t& lazy_node)
      {
        for (size_t i=0;i<num;i++)
        {
          vfloat<Nx> dist;
          /* QBVH intersection test */
          //size_t mask = ((size_t)1 << prim[i].size())-1;
          size_t mask = intersectNode(&prim[i].qnode,tray,ray.time(),dist); 
          while(mask != 0)
          {
            const size_t ID = bscf(mask); 
            if (occluded(pre,ray,context,prim[i].subgrid(ID)))
              return true;
          }
        }
        return false;
      }
    };


    /*! Intersects M triangles with K rays. */
    template<int N, int K, bool filter>
    struct SubGridMBIntersectorKMoeller
    {
      typedef SubGridMBQBVHN<N> Primitive;
      typedef SubGridQuadMIntersectorKMoellerTrumbore<4,K,filter> Precalculations;

      /*! Intersects K rays with M triangles. */
      static __forceinline void intersect(const vbool<K>& valid_i, Precalculations& pre, RayHitK<K>& ray, IntersectContext* context, const SubGrid& subgrid)
      {
        Vec3fa vtx[16];
        const GridMesh* mesh    = context->scene->get<GridMesh>(subgrid.geomID());
        const GridMesh::Grid &g = mesh->grid(subgrid.primID());

        vfloat<K> ftime;
        const vint<K> itime = getTimeSegment(ray.time(), vfloat<K>(mesh->fnumTimeSegments), ftime);
        size_t m_valid = movemask(valid_i);

        while(m_valid)
        {
          size_t firstValidID = bsf(m_valid);
          const vbool<K> m_same_time = (itime[firstValidID] == itime) & valid_i;
          subgrid.gatherMB((vfloat4*)vtx,context->scene,itime[firstValidID],ftime[firstValidID]);
          m_valid &= ~(size_t)movemask(m_same_time);
          for (unsigned int i=0; i<4; i++)
          {
            const Vec3vf<K> p0 = vtx[i*4+0];
            const Vec3vf<K> p1 = vtx[i*4+1];
            const Vec3vf<K> p2 = vtx[i*4+2];
            const Vec3vf<K> p3 = vtx[i*4+3];
            STAT3(normal.trav_prims,1,popcnt(valid_i),K);
            pre.intersectK(m_same_time,ray,p0,p1,p2,p3,g,subgrid,i,IntersectKEpilogM<4,K,filter>(ray,context,subgrid.geomID(),subgrid.primID(),i));
          }
        }
      }

      /*! Test for K rays if they are occluded by any of the M triangles. */
      static __forceinline vbool<K> occluded(const vbool<K>& valid_i, Precalculations& pre, RayK<K>& ray, IntersectContext* context, const SubGrid& subgrid)
      {
        vbool<K> valid0 = valid_i;
        Vec3fa vtx[16];
        const GridMesh* mesh    = context->scene->get<GridMesh>(subgrid.geomID());
        const GridMesh::Grid &g = mesh->grid(subgrid.primID());

        vfloat<K> ftime;
        const vint<K> itime = getTimeSegment(ray.time(), vfloat<K>(mesh->fnumTimeSegments), ftime);
        size_t m_valid = movemask(valid_i);

        while(m_valid)
        {
          size_t firstValidID = bsf(m_valid);
          const vbool<K> m_same_time = (itime[firstValidID] == itime) & valid_i;
          subgrid.gatherMB((vfloat4*)vtx,context->scene,itime[firstValidID],ftime[firstValidID]);
          m_valid &= ~(size_t)movemask(m_same_time);
          for (unsigned int i=0; i<4; i++)
          {
            const Vec3vf<K> p0 = vtx[i*4+0];
            const Vec3vf<K> p1 = vtx[i*4+1];
            const Vec3vf<K> p2 = vtx[i*4+2];
            const Vec3vf<K> p3 = vtx[i*4+3];
            STAT3(shadow.trav_prims,1,popcnt(valid0),K);
            if (pre.intersectK(m_same_time,ray,p0,p1,p2,p3,g,subgrid,i,OccludedKEpilogM<4,K,filter>(valid0,ray,context,subgrid.geomID(),subgrid.primID(),i)))
              break;
          }
        }
        return !valid0;
      }

      /*! Intersect a ray with M triangles and updates the hit. */
      static __forceinline void intersect(Precalculations& pre, RayHitK<K>& ray, size_t k, IntersectContext* context, const SubGrid& subgrid)
      {
        STAT3(normal.trav_prims,1,1,1);
        const GridMesh* mesh    = context->scene->get<GridMesh>(subgrid.geomID());
        const GridMesh::Grid &g = mesh->grid(subgrid.primID());

        vfloat<K> ftime;
        const vint<K> itime = getTimeSegment(ray.time(), vfloat<K>(mesh->fnumTimeSegments), ftime);
        Vec3vf4 v0,v1,v2,v3; subgrid.gatherMB(v0,v1,v2,v3,context->scene,itime[k],ftime[k]);
        pre.intersect1(ray,k,context,v0,v1,v2,v3,g,subgrid);
      }

      /*! Test if the ray is occluded by one of the M triangles. */
      static __forceinline bool occluded(Precalculations& pre, RayK<K>& ray, size_t k, IntersectContext* context, const SubGrid& subgrid)
      {
        STAT3(shadow.trav_prims,1,1,1);
        const GridMesh* mesh    = context->scene->get<GridMesh>(subgrid.geomID());
        const GridMesh::Grid &g = mesh->grid(subgrid.primID());

        vfloat<K> ftime;
        const vint<K> itime = getTimeSegment(ray.time(), vfloat<K>(mesh->fnumTimeSegments), ftime);
        Vec3vf4 v0,v1,v2,v3; subgrid.gatherMB(v0,v1,v2,v3,context->scene,itime[k],ftime[k]);
        return pre.occluded1(ray,k,context,v0,v1,v2,v3,g,subgrid);
      }

        template<bool robust>
          static __forceinline void intersect(const vbool<K>& valid, const Accel::Intersectors* This, Precalculations& pre, RayHitK<K>& ray, IntersectContext* context, const Primitive* prim, size_t num, const TravRayK<K, robust> &tray, size_t& lazy_node)
        {
          for (size_t j=0;j<num;j++)
          {
            size_t m_valid = movemask(prim[j].qnode.validMask());
            vfloat<K> dist;
            while(m_valid)
            {
              const size_t i = bscf(m_valid);
              //if (none(valid & intersectNodeK<N>(&prim[j].qnode,i,tray,dist))) continue;
              FATAL("not yet implemented");
              intersect(valid,pre,ray,context,prim[j].subgrid(i));
            }
          }
        }

        template<bool robust>        
        static __forceinline vbool<K> occluded(const vbool<K>& valid, const Accel::Intersectors* This, Precalculations& pre, RayK<K>& ray, IntersectContext* context, const Primitive* prim, size_t num, const TravRayK<K, robust> &tray, size_t& lazy_node)
        {
          vbool<K> valid0 = valid;
          for (size_t j=0;j<num;j++)
          {
            size_t m_valid = movemask(prim[j].qnode.validMask());
            vfloat<K> dist;
            while(m_valid)
            {
              const size_t i = bscf(m_valid);
              FATAL("not yet implemented");
              //if (none(valid0 & intersectNodeK<N>(&prim[j].qnode,i,tray,dist))) continue;
              valid0 &= !occluded(valid0,pre,ray,context,prim[j].subgrid(i));
              if (none(valid0)) break;
            }
          }
          return !valid0;
        }

        template<int Nx, bool robust>        
          static __forceinline void intersect(const Accel::Intersectors* This, Precalculations& pre, RayHitK<K>& ray, size_t k, IntersectContext* context, const Primitive* prim, size_t num, const TravRay<N,Nx,robust> &tray, size_t& lazy_node)
        {
          for (size_t i=0;i<num;i++)
          {
            vfloat<N> dist;
            /* QBVH intersection test */
            size_t mask = intersectNode(&prim[i].qnode,tray,ray.time()[k],dist); //FIXME: maybe do node ordering here
            while(mask != 0)
            {
              const size_t ID = bscf(mask); 
              if (unlikely(dist[ID] > ray.tfar[k])) continue;
              intersect(pre,ray,k,context,prim[i].subgrid(ID));
            }
          }
        }
        
        template<int Nx, bool robust>
        static __forceinline bool occluded(const Accel::Intersectors* This, Precalculations& pre, RayK<K>& ray, size_t k, IntersectContext* context, const Primitive* prim, size_t num, const TravRay<N,Nx,robust> &tray, size_t& lazy_node)
        {
          for (size_t i=0;i<num;i++)
          {
            vfloat<N> dist;
            /* QBVH intersection test */
            size_t mask = intersectNode(&prim[i].qnode,tray,ray.time()[k],dist); 
            while(mask != 0)
            {
              const size_t ID = bscf(mask); 
              if (occluded(pre,ray,k,context,prim[i].subgrid(ID)))
                return true;
            }
          }
          return false;
        }


    };



  }
}
