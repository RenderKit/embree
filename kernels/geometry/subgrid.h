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
          : x(x), y(y), _geomID(geomID), _primID(primID)
        {
        }

        __forceinline bool invalid3x3X() const { return _geomID & (1<<31); }
        __forceinline bool invalid3x3Y() const { return _geomID & (1<<30); }


        __forceinline Vec3fa getVertex(const size_t x, const size_t y, const Scene *const scene) const
        {
          const GridMesh* mesh = scene->get<GridMesh>(geomID());
          const GridMesh::Grid &g= mesh->grid(primID());
          const size_t vtxID = g.startVtxID + x + y * g.lineVtxOffset;
          return mesh->vertex(vtxID);
        }

        __forceinline Vec3fa getVertex(const size_t x, const size_t y, const Scene *const scene, const size_t itime) const
        {
          const GridMesh* mesh = scene->get<GridMesh>(geomID());
          const GridMesh::Grid &g= mesh->grid(primID());
          const size_t vtxID = g.startVtxID + x + y * g.lineVtxOffset;
          return Vec3fa::loadu(mesh->vertexPtr(vtxID,itime));
        }

        template<typename T>
        __forceinline Vec3<T> getVertex(const size_t x, const size_t y, const Scene *const scene, const size_t itime, const T& ftime) const
        {
          const GridMesh* mesh = scene->get<GridMesh>(geomID());
          const GridMesh::Grid &g = mesh->grid(primID());
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
          const GridMesh* mesh     = scene->get<GridMesh>(geomID());
          const GridMesh::Grid &g  = mesh->grid(primID());

          /* first quad always valid */
          const size_t vtxID00 = g.startVtxID + x + y * g.lineVtxOffset;
          const size_t vtxID01 = vtxID00 + 1;
          const vfloat4 vtx00  = vfloat4::loadu(mesh->vertexPtr(vtxID00));
          const vfloat4 vtx01  = vfloat4::loadu(mesh->vertexPtr(vtxID01));
          const size_t vtxID10 = vtxID00 + g.lineVtxOffset;
          const size_t vtxID11 = vtxID01 + g.lineVtxOffset;
          const vfloat4 vtx10  = vfloat4::loadu(mesh->vertexPtr(vtxID10));
          const vfloat4 vtx11  = vfloat4::loadu(mesh->vertexPtr(vtxID11));

          /* deltaX => vtx02, vtx12 */
          const size_t deltaX  = invalid3x3X() ? 0 : 1;
          const size_t vtxID02 = vtxID01 + deltaX;       
          const vfloat4 vtx02  = vfloat4::loadu(mesh->vertexPtr(vtxID02));
          const size_t vtxID12 = vtxID11 + deltaX;       
          const vfloat4 vtx12  = vfloat4::loadu(mesh->vertexPtr(vtxID12));

          /* deltaY => vtx20, vtx21 */
          const size_t deltaY  = invalid3x3Y() ? 0 : g.lineVtxOffset;
          const size_t vtxID20 = vtxID10 + deltaY;
          const size_t vtxID21 = vtxID11 + deltaY;
          const vfloat4 vtx20  = vfloat4::loadu(mesh->vertexPtr(vtxID20));
          const vfloat4 vtx21  = vfloat4::loadu(mesh->vertexPtr(vtxID21));

          /* deltaX/deltaY => vtx22 */
          const size_t vtxID22 = vtxID11 + deltaX + deltaY;       
          const vfloat4 vtx22  = vfloat4::loadu(mesh->vertexPtr(vtxID22));

          transpose(vtx00,vtx01,vtx11,vtx10,p0.x,p0.y,p0.z);
          transpose(vtx01,vtx02,vtx12,vtx11,p1.x,p1.y,p1.z);
          transpose(vtx11,vtx12,vtx22,vtx21,p2.x,p2.y,p2.z);
          transpose(vtx10,vtx11,vtx21,vtx20,p3.x,p3.y,p3.z);          
          
        }


        /* Gather the quads */
        __forceinline void gather(Vec3fa vtx[16], const Scene *const scene) const
        {
          const GridMesh* mesh     = scene->get<GridMesh>(geomID());
          const GridMesh::Grid &g  = mesh->grid(primID());

          /* first quad always valid */
          const size_t vtxID00 = g.startVtxID + x + y * g.lineVtxOffset;
          const size_t vtxID01 = vtxID00 + 1;
          const Vec3fa vtx00  = Vec3fa::loadu(mesh->vertexPtr(vtxID00));
          const Vec3fa vtx01  = Vec3fa::loadu(mesh->vertexPtr(vtxID01));
          const size_t vtxID10 = vtxID00 + g.lineVtxOffset;
          const size_t vtxID11 = vtxID01 + g.lineVtxOffset;
          const Vec3fa vtx10  = Vec3fa::loadu(mesh->vertexPtr(vtxID10));
          const Vec3fa vtx11  = Vec3fa::loadu(mesh->vertexPtr(vtxID11));

          /* deltaX => vtx02, vtx12 */
          const size_t deltaX  = invalid3x3X() ? 0 : 1;
          const size_t vtxID02 = vtxID01 + deltaX;       
          const Vec3fa vtx02  = Vec3fa::loadu(mesh->vertexPtr(vtxID02));
          const size_t vtxID12 = vtxID11 + deltaX;       
          const Vec3fa vtx12  = Vec3fa::loadu(mesh->vertexPtr(vtxID12));

          /* deltaY => vtx20, vtx21 */
          const size_t deltaY  = invalid3x3Y() ? 0 : g.lineVtxOffset;
          const size_t vtxID20 = vtxID10 + deltaY;
          const size_t vtxID21 = vtxID11 + deltaY;
          const Vec3fa vtx20  = Vec3fa::loadu(mesh->vertexPtr(vtxID20));
          const Vec3fa vtx21  = Vec3fa::loadu(mesh->vertexPtr(vtxID21));

          /* deltaX/deltaY => vtx22 */
          const size_t vtxID22 = vtxID11 + deltaX + deltaY;       
          const Vec3fa vtx22  = Vec3fa::loadu(mesh->vertexPtr(vtxID22));

          vtx[ 0] = vtx00; vtx[ 1] = vtx01; vtx[ 2] = vtx11; vtx[ 3] = vtx10;
          vtx[ 4] = vtx01; vtx[ 5] = vtx02; vtx[ 6] = vtx12; vtx[ 7] = vtx11;
          vtx[ 8] = vtx11; vtx[ 9] = vtx12; vtx[10] = vtx22; vtx[11] = vtx21;
          vtx[12] = vtx10; vtx[13] = vtx11; vtx[14] = vtx21; vtx[15] = vtx20;
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


        friend std::ostream& operator<<(std::ostream& cout, const SubGrid& sg) {
          return cout << "SubGrid " << " ( x " << sg.x << ", y = " << sg.y << ", geomID = " << sg.geomID() << ", primID = " << sg.primID() << " )";
        }

        __forceinline unsigned int geomID() const { return _geomID & 0x3fffffff; }
        __forceinline unsigned int primID() const { return _primID; }


      public:
        unsigned short x;
        unsigned short y;
      private:
        unsigned int _geomID;    // geometry ID of mesh
        unsigned int _primID;    // primitive ID of primitive inside mesh
      };

}
