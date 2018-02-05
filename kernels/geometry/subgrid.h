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
        __forceinline SubGrid(const unsigned int startVtxID,
                              const unsigned int lineVtxOffset,
                              const unsigned int geomID,
                              const unsigned int primID)
          : startVtxID(startVtxID), lineVtxOffset(lineVtxOffset), geomID(geomID), primID(primID) {}

        __forceinline Vec3fa getVertex(const size_t x, const size_t y, const Scene *const scene) const
        {
          const GridMesh* mesh = scene->get<GridMesh>(geomID);
          //const GridMesh::Grid = mesh->grid(primID());
          const size_t vtxID = startVtxID + x + y * lineVtxOffset;
          return mesh->vertex(vtxID);
        }

        __forceinline Vec3fa getVertex(const size_t x, const size_t y, const Scene *const scene, const size_t itime) const
        {
          const GridMesh* mesh = scene->get<GridMesh>(geomID);
          //const GridMesh::Grid = mesh->grid(primID());
          const size_t vtxID = startVtxID + x + y * lineVtxOffset;
          return Vec3fa::loadu(mesh->vertexPtr(vtxID,itime));
        }

        template<typename T>
        __forceinline Vec3<T> getVertex(const size_t x, const size_t y, const Scene *const scene, const size_t itime, const T& ftime) const
        {
          const GridMesh* mesh = scene->get<GridMesh>(geomID);
          //const GridMesh::Grid = mesh->grid(primID());
          const size_t vtxID = startVtxID + x + y * lineVtxOffset;
          const Vec3fa* vertices0 = (const Vec3fa*) mesh->vertexPtr(0,itime+0);
          const Vec3fa* vertices1 = (const Vec3fa*) mesh->vertexPtr(0,itime+1);
          const Vec3fa v0 = Vec3fa::loadu(&vertices0[vtxID]);
          const Vec3fa v1 = Vec3fa::loadu(&vertices1[vtxID]);
          const Vec3<T> p0(v0.x,v0.y,v0.z);
          const Vec3<T> p1(v1.x,v1.y,v1.z);
          return lerp(p0,p1,ftime);
        }

        /* Calculate the bounds of the subgrid */
        __forceinline const BBox3fa bounds(const Scene *const scene, const size_t itime=0) const
        {
          BBox3fa bounds = empty;
          for (size_t y=0;y<3;y++) // does not consider corner cases
            for (size_t x=0;x<3;x++)
              bounds.extend(getVertex(x,y,scene,itime));
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
          FATAL("HERE");
          return allBounds;
        }

        __forceinline LBBox3fa linearBounds(const Scene *const scene, const BBox1f time_range)
        {
          LBBox3fa allBounds = empty;
          FATAL("HERE");
          return allBounds;
        }


        /* Fill triangle from triangle list */
        __forceinline void fill(const PrimRef* prims, size_t& begin, size_t end, Scene* scene)
        {
          assert(begin+1 == end);
          PING;
        }



        friend std::ostream& operator<<(std::ostream& cout, const SubGrid& sg) {
          return cout << "SubGrid " << " ( startVtxID " << sg.startVtxID << ", lineVtxOffset = " << sg.lineVtxOffset << ", geomID = " << sg.geomID << ", primID = " << sg.primID << " )";
        }


      public:
        unsigned int startVtxID;
        unsigned int lineVtxOffset;
      private:
        unsigned int geomID;    // geometry ID of mesh
        unsigned int primID;    // primitive ID of primitive inside mesh
      };

}
