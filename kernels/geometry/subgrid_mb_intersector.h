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
          size_t mask = ((size_t)1 << prim[i].size())-1; //intersectNode(&prim[i].qnode0,tray,dist); 
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
          //size_t mask = intersectNode(&prim[i].qnode0,tray,dist); 
          size_t mask = ((size_t)1 << prim[i].size())-1;
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
  }
}
