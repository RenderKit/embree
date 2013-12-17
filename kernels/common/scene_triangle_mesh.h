// ======================================================================== //
// Copyright 2009-2013 Intel Corporation                                    //
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

#ifndef __EMBREE_TRIANGLE_MESH_SCENE_H__
#define __EMBREE_TRIANGLE_MESH_SCENE_H__

#include "common/default.h"
#include "common/geometry.h"
#include "xeon/builders/build_source.h"

namespace embree
{
  namespace TriangleMeshScene
  {

    /*! Triangle Mesh */
    struct TriangleMesh : public Geometry, public BuildSource
    {
      struct Triangle {
        unsigned int v[3];
      };

    public:
      TriangleMesh (Scene* parent, RTCGeometryFlags flags, size_t numTriangles, size_t numVertices, size_t numTimeSteps); 
      ~TriangleMesh ();
      
    public:
      void setMask (unsigned mask);
      void enable ();
      void update ();
      void disable ();
      void erase ();
      void immutable ();
      bool verify ();
      void* map(RTCBufferType type);
      void unmap(RTCBufferType type);

      void enabling();
      void disabling();

    public:

      bool isEmpty () const { 
        return numTriangles == 0;
      }
      
      size_t groups () const { 
        return 1;
      }
      
      size_t prims (size_t group, size_t* pnumVertices) const {
        if (pnumVertices) *pnumVertices = numVertices*numTimeSteps;
        return numTriangles;
      }

      const BBox3f bounds(size_t group, size_t prim) const {
        return bounds(prim);
      }

      void bounds(size_t group, size_t begin, size_t end, BBox3f* bounds_o) const 
      {
        BBox3f b = empty;
        for (size_t i=begin; i<end; i++) b.extend(bounds(i));
        *bounds_o = b;
      }

      void split (const PrimRef& prim, int dim, float pos, PrimRef& left_o, PrimRef& right_o) const;

    public:

      __forceinline const Triangle& triangle(size_t i) const {
        assert(i < numTriangles);
        return triangles[i];
      }

      __forceinline const Vec3fa& vertex(size_t i, size_t j = 0) const {
        assert(i < numVertices);
        assert(j < 2);
        return vertices_[j][i];
      }

      __forceinline BBox3f bounds(size_t index) const 
      {
        const Triangle& tri = triangle(index);
        const Vec3fa& v0 = vertex(tri.v[0]);
        const Vec3fa& v1 = vertex(tri.v[1]);
        const Vec3fa& v2 = vertex(tri.v[2]);
	return BBox3f( min(min(v0,v1),v2), max(max(v0,v1),v2) );
      }

      __forceinline const Vec3fa& getTriangleVertex(size_t index, size_t vtxID)
      {
        const Triangle& tri = triangle(index);
        return vertex(tri.v[vtxID]);
      }

      __forceinline bool anyMappedBuffers() const {
        return mappedTriangles || mappedVertices[0] || mappedVertices[1];
      }

    public:
      unsigned mask;              //!< for masking out geometry
      bool built;                //!< geometry got built
      unsigned char numTimeSteps;

      Triangle* triangles;        //!< array of triangles
      size_t numTriangles;        //!< number of triangles in array
      bool mappedTriangles;       //!< is triangles buffer mapped?
      bool needTriangles;         //!< true if triangle array required by acceleration structure

      Vec3fa* vertices_[2];           //!< array of vertices, has to be aligned to 16 bytes
      size_t numVertices;         //!< number of vertices in array
      bool mappedVertices[2];        //!< is vertex buffer mapped?
      bool needVertices;          //!< true if vertex array required by acceleration structure
    };
  }
}

#endif
