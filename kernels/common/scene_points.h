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

#include "default.h"
#include "geometry.h"
#include "primref.h"
#include "buffer.h"

namespace embree
{
  /*! represents an array of points */
  struct Points : public Geometry
  {
    /*! type of this geometry */
    static const Geometry::Type geom_type = Geometry::POINTS;

  public:

    /*! line segments construction */
    Points (Scene* parent, RTCGeometryFlags flags, size_t numPrimitives, size_t numTimeSteps);

    /*! writes the point geometry to disk */
    void write(std::ofstream& file);

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

    /*! returns number of points */
    __forceinline size_t size() const {
      return numVertices();
    }

    /*! returns the number of vertices */
    __forceinline size_t numVertices() const {
      return vertices[0].size();
    }

    /*! returns the i'th point */
    __forceinline size_t point(size_t i) const {
      return i;
    }

    /*! returns i'th vertex of j'th timestep */
    __forceinline Vec3fa vertex(size_t i, size_t j = 0) const {
      return vertices[j][i];
    }

    /*! returns i'th vertex of j'th timestep */
    __forceinline const char* vertexPtr(size_t i, size_t j = 0) const {
      return vertices[j].getPtr(i);
    }

    /*! returns i'th radius of j'th timestep */
    __forceinline float radius(size_t i, size_t j = 0) const {
      return vertices[j][i].w;
    }

    /*! check if the i'th primitive is valid */
    __forceinline bool valid(size_t i, BBox3fa* bbox = nullptr) const
    {
      const int index = point(i);
      if (index < 0 || index >= numVertices()) return false;

      for (size_t j=0; j<numTimeSteps; j++)
      {
        const float r0 = radius(index+0,j);
        if (!isvalid(r0))
          return false;
        if (r0 < 0.0f)
          return false;

        const Vec3fa v0 = vertex(index+0,j);
        if (!isvalid(v0))
          return false;
      }

      if (bbox) *bbox = bounds(i);
      return true;
    }

    /*! calculates bounding box of i'th point */
    __forceinline BBox3fa bounds(size_t i, size_t j = 0) const
    {
      const int index = i;
      const float r0 = radius(index+0,j);
      const Vec3fa v0 = vertex(index+0,j);
      const BBox3fa b = BBox3fa(v0);
      return enlarge(b,Vec3fa(r0));
    }

    /*! calculates bounding box of i'th point */
    __forceinline BBox3fa bounds(const AffineSpace3fa& space, size_t i, size_t j = 0) const
    {
      const int index = i;
      const float r0 = radius(index+0,j);
      const Vec3fa v0 = xfmPoint(space,vertex(index+0,j));
      const BBox3fa b = BBox3fa(v0);
      return enlarge(b,Vec3fa(r0));
    }

  public:
    array_t<BufferT<Vec3fa>,2> vertices;            //!< vertex array
    array_t<std::unique_ptr<Buffer>,2> userbuffers; //!< user buffers
  };
}
