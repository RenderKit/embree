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

#include "geometry.h"
#include "buffer.h"

namespace embree
{
  /*! Quad Mesh */
  struct QuadMesh : public Geometry
  {
    /*! type of this geometry */
    static const Geometry::Type geom_type = Geometry::QUAD_MESH;
    
    /*! triangle indices */
    struct Quad
    {
      uint32_t v[4];

      /*! outputs triangle indices */
      __forceinline friend std::ostream &operator<<(std::ostream& cout, const Quad& q) {
        return cout << "{ quad " << q.v[0] << ", " << q.v[1] << ", " << q.v[2] << ", " << q.v[3] << " }";
      }
    };

  public:

    /*! quad mesh construction */
    QuadMesh (Scene* parent, RTCGeometryFlags flags, size_t numQuads, size_t numVertices, size_t numTimeSteps); 
  
    /*! writes the quad mesh geometry to disk */
    void write(std::ofstream& file);

    /* geometry interface */
  public:
    void enabling();
    void disabling();
    void setMask (unsigned mask);
    void setBuffer(RTCBufferType type, void* ptr, size_t offset, size_t stride);
    void* map(RTCBufferType type);
    void unmap(RTCBufferType type);
    void immutable ();
    bool verify ();
    void interpolate(unsigned primID, float u, float v, RTCBufferType buffer, float* P, float* dPdu, float* dPdv, size_t numFloats);
    // FIXME: implement interpolateN

  public:

    /*! returns number of quads */
    __forceinline size_t size() const {
      return quads.size();
    }

    /*! returns number of vertices */
    __forceinline size_t numVertices() const {
      return vertices[0].size();
    }
    
    /*! returns i'th quad*/
    __forceinline const Quad& quad(size_t i) const {
      return quads[i];
    }

    /*! returns i'th vertex of j'th timestep */
    __forceinline const Vec3fa vertex(size_t i, size_t j = 0) const {
      return vertices[j][i];
    }

    /*! returns i'th vertex of j'th timestep */
    __forceinline const char* vertexPtr(size_t i, size_t j = 0) const {
      return vertices[j].getPtr(i);
    }

    /*! calculates the bounds of the i'th quad */
    __forceinline BBox3fa bounds(size_t i) const 
    {
      const Quad& q = quad(i);
      const Vec3fa v0  = vertex(q.v[0]);
      const Vec3fa v1  = vertex(q.v[1]);
      const Vec3fa v2  = vertex(q.v[2]);
      const Vec3fa v3  = vertex(q.v[3]);
      return BBox3fa(min(v0,v1,v2,v3),max(v0,v1,v2,v3));
    }

    /*! check if the i'th primitive is valid */
    __forceinline bool valid(size_t i, BBox3fa* bbox = nullptr) const 
    {
      const Quad& q = quad(i);
      if (q.v[0] >= numVertices()) return false;
      if (q.v[1] >= numVertices()) return false;
      if (q.v[2] >= numVertices()) return false;
      if (q.v[3] >= numVertices()) return false;

      for (size_t j=0; j<numTimeSteps; j++) 
      {
        const Vec3fa v0 = vertex(q.v[0],j);
        const Vec3fa v1 = vertex(q.v[1],j);
        const Vec3fa v2 = vertex(q.v[2],j);
        const Vec3fa v3 = vertex(q.v[3],j);

        if (!isvalid(v0) || !isvalid(v1) || !isvalid(v2) || !isvalid(v3))
          return false;
      }

      if (bbox) 
        *bbox = bounds(i);

      return true;
    }
    
  public:
    BufferT<Quad> quads;                            //!< array of quads
    array_t<BufferT<Vec3fa>,2> vertices;            //!< vertex array
    array_t<std::unique_ptr<Buffer>,2> userbuffers; //!< user buffers
  };
}
