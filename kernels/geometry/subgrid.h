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

#include "../common/ray.h"
#include "../common/scene_grid_mesh.h"
#include "../bvh/bvh.h"

namespace embree
{
    /* Stores M quads from an indexed face set */
      struct SubGrid
      {
        /* Virtual interface to query information about the quad type */
        struct Type : public PrimitiveType
        {
          Type();
          size_t size(const char* This) const;
        };
        static Type type;

      public:

        /* primitive supports multiple time segments */
        static const bool singleTimeSegment = false;

        /* Returns maximum number of stored quads */
        static __forceinline size_t max_size() { return 1; }

        /* Returns required number of primitive blocks for N primitives */
        static __forceinline size_t blocks(size_t N) { return (N+max_size()-1)/max_size(); }

      public:

        /* Default constructor */
        __forceinline SubGrid() {  }

        /* Construction from vertices and IDs */
        __forceinline SubGrid(const unsigned int x,
                              const unsigned int y,
                              const unsigned int geomID,
                              const unsigned int primID)
          : x(x), y(y), geomID(geomID), primID(primID) {}

        __forceinline Vec3fa getVertex(const size_t x, const size_t y, const Scene *const scene) const
        {
          const GridMesh* mesh = scene->get<GridMesh>(geomID);
          const GridMesh::Grid &g= mesh->grid(primID);
          const size_t vtxID = g.startVtxID + x + y * g.lineVtxOffset;
          return mesh->vertex(vtxID);
        }

        __forceinline Vec3fa getVertex(const size_t x, const size_t y, const Scene *const scene, const size_t itime) const
        {
          const GridMesh* mesh = scene->get<GridMesh>(geomID);
          const GridMesh::Grid &g= mesh->grid(primID);
          const size_t vtxID = g.startVtxID + x + y * g.lineVtxOffset;
          return Vec3fa::loadu(mesh->vertexPtr(vtxID,itime));
        }

        template<typename T>
        __forceinline Vec3<T> getVertex(const size_t x, const size_t y, const Scene *const scene, const size_t itime, const T& ftime) const
        {
          const GridMesh* mesh = scene->get<GridMesh>(geomID);
          const GridMesh::Grid &g = mesh->grid(primID);
          const size_t vtxID = g.startVtxID + x + y * g.lineVtxOffset;
          const Vec3fa* vertices0 = (const Vec3fa*) mesh->vertexPtr(0,itime+0);
          const Vec3fa* vertices1 = (const Vec3fa*) mesh->vertexPtr(0,itime+1);
          const Vec3fa v0 = Vec3fa::loadu(&vertices0[vtxID]);
          const Vec3fa v1 = Vec3fa::loadu(&vertices1[vtxID]);
          const Vec3<T> p0(v0.x,v0.y,v0.z);
          const Vec3<T> p1(v1.x,v1.y,v1.z);
          return lerp(p0,p1,ftime);
        }

        /* Gather the quads */
        __forceinline void gather(Vec3vf4& p0,
                                  Vec3vf4& p1,
                                  Vec3vf4& p2,
                                  Vec3vf4& p3,
                                  const Scene *const scene) const
        {
          const GridMesh* mesh    = scene->get<GridMesh>(geomID);
          const GridMesh::Grid &g = mesh->grid(primID);

          /* row0 */
          const size_t vtxID00 = g.startVtxID + x + y * g.lineVtxOffset;
          const size_t vtxID01 = vtxID00 + 1;
          const size_t vtxID02 = vtxID00 + 2;       
          const vfloat4 vtx00  = vfloat4::loadu(mesh->vertexPtr(vtxID00));
          const vfloat4 vtx01  = vfloat4::loadu(mesh->vertexPtr(vtxID01));
          const vfloat4 vtx02  = vfloat4::loadu(mesh->vertexPtr(vtxID02));

          /* row1 */
          const size_t vtxID10 = vtxID00 + g.lineVtxOffset;
          const size_t vtxID11 = vtxID01 + g.lineVtxOffset;
          const size_t vtxID12 = vtxID02 + g.lineVtxOffset;       
          const vfloat4 vtx10  = vfloat4::loadu(mesh->vertexPtr(vtxID10));
          const vfloat4 vtx11  = vfloat4::loadu(mesh->vertexPtr(vtxID11));
          const vfloat4 vtx12  = vfloat4::loadu(mesh->vertexPtr(vtxID12));

          /* row2 */
          const size_t vtxID20 = vtxID10 + g.lineVtxOffset;
          const size_t vtxID21 = vtxID11 + g.lineVtxOffset;
          const size_t vtxID22 = vtxID12 + g.lineVtxOffset;       
          const vfloat4 vtx20  = vfloat4::loadu(mesh->vertexPtr(vtxID20));
          const vfloat4 vtx21  = vfloat4::loadu(mesh->vertexPtr(vtxID21));
          const vfloat4 vtx22  = vfloat4::loadu(mesh->vertexPtr(vtxID22));

#if 0
          PRINT(vtx00);
          PRINT(vtx01);
          PRINT(vtx02);

          PRINT(vtx10);
          PRINT(vtx11);
          PRINT(vtx12);

          PRINT(vtx20);
          PRINT(vtx21);
          PRINT(vtx22);
          PRINT(p0);
          PRINT(p1);
          PRINT(p2);
          PRINT(p3);
          exit(0);
#endif

          transpose(vtx00,vtx01,vtx11,vtx10,p0.x,p0.y,p0.z);
          transpose(vtx01,vtx02,vtx12,vtx11,p1.x,p1.y,p1.z);
          transpose(vtx11,vtx12,vtx22,vtx21,p2.x,p2.y,p2.z);
          transpose(vtx10,vtx11,vtx21,vtx20,p3.x,p3.y,p3.z);          
          
        }

        /* Calculate the bounds of the subgrid */
        __forceinline const BBox3fa bounds(const Scene *const scene, const size_t itime=0) const
        {
          BBox3fa bounds = empty;
          FATAL("not implemented yet");
          return bounds;
        }

        /* Calculate the linear bounds of the primitive */
        __forceinline LBBox3fa linearBounds(const Scene* const scene, const size_t itime)
        {
          return LBBox3fa(bounds(scene,itime+0),bounds(scene,itime+1));
        }

        __forceinline LBBox3fa linearBounds(const Scene *const scene, size_t itime, size_t numTimeSteps)
        {
          LBBox3fa allBounds = empty;
          //const GridMesh* mesh = scene->get<GridMesh>(geomID);
          //allBounds.extend(mesh->linearBounds(primID, itime, numTimeSteps));
          FATAL("not implemented yet");
          return allBounds;
        }

        __forceinline LBBox3fa linearBounds(const Scene *const scene, const BBox1f time_range)
        {
          LBBox3fa allBounds = empty;
          FATAL("not implemented yet");
          return allBounds;
        }


        /* Fill triangle from triangle list */
        __forceinline void fill(const PrimRef* prims, size_t& begin, size_t end, Scene* scene)
        {
          assert(begin+1 == end);
          PING;
        }



        friend std::ostream& operator<<(std::ostream& cout, const SubGrid& sg) {
          return cout << "SubGrid " << " ( x " << sg.x << ", y = " << sg.y << ", geomID = " << sg.geomID << ", primID = " << sg.primID << " )";
        }


      public:
        unsigned short x;
        unsigned short y;
        unsigned int geomID;    // geometry ID of mesh
        unsigned int primID;    // primitive ID of primitive inside mesh
      };

}
