// ======================================================================== //
// Copyright 2009-2016 Intel Corporation                                    //
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
  /*! represents an array of line segments */
  struct LineSegments : public Geometry
  {
    /*! type of this geometry */
    static const Geometry::Type geom_type = Geometry::LINE_SEGMENTS;

  public:

    /*! line segments construction */
    LineSegments (Scene* parent, RTCGeometryFlags flags, size_t numPrimitives, size_t numVertices, size_t numTimeSteps);

    /*! writes the bezier segment geometry to disk */
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
    void interpolate(unsigned primID, float u, float v, RTCBufferType buffer, float* P, float* dPdu, float* dPdv, float* ddPdudu, float* ddPdvdv, float* ddPdudv, size_t numFloats);
    // FIXME: implement interpolateN

  public:

    /*! returns number of line segments */
    __forceinline size_t size() const {
      return segments.size();
    }

    /*! returns the number of vertices */
    __forceinline size_t numVertices() const {
      return vertices[0].size();
    }

    /*! returns the i'th segment */
    __forceinline const unsigned int& segment(size_t i) const {
      return segments[i];
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

    /*! calculates bounding box of i'th line segment */
    __forceinline BBox3fa bounds(size_t i) const
    {
      const unsigned int index = segment(i);
      const float r0 = radius(index+0);
      const float r1 = radius(index+1);
      const Vec3fa v0 = vertex(index+0);
      const Vec3fa v1 = vertex(index+1);
      const BBox3fa b = merge(BBox3fa(v0),BBox3fa(v1));
      return enlarge(b,Vec3fa(max(r0,r1)));
    }

    /*! calculates bounding box of i'th line segment for the j'th time step */
    __forceinline BBox3fa bounds(size_t i, size_t j) const
    {
      const unsigned int index = segment(i);
      const float r0 = radius(index+0,j);
      const float r1 = radius(index+1,j);
      const Vec3fa v0 = vertex(index+0,j);
      const Vec3fa v1 = vertex(index+1,j);
      const BBox3fa b = merge(BBox3fa(v0),BBox3fa(v1));
      return enlarge(b,Vec3fa(max(r0,r1)));
    }

    /*! check if the i'th primitive is valid at the j'th time step */
    __forceinline bool valid1(size_t i, size_t j) const
    {
      const unsigned int index = segment(i);
      if (index+1 >= numVertices()) return false;

      const float r0 = radius(index+0,j); if (!isvalid(r0)) return false;
      const float r1 = radius(index+1,j); if (!isvalid(r1)) return false;
      if (min(r0,r1) < 0.0f) return false;
      
      const Vec3fa v0 = vertex(index+0,j); if (!isvalid(v0)) return false;
      const Vec3fa v1 = vertex(index+1,j); if (!isvalid(v1)) return false;
      
      return true;
    }

    /*! check if the i'th primitive is valid */
    __forceinline bool valid(size_t i, BBox3fa* bbox) const {
      *bbox = bounds(i); return valid1(i,0);
    }

    /*! check if the i'th primitive is valid at the j'th time segment */
    __forceinline bool valid2(size_t i, size_t j, BBox3fa& bbox) const 
    {
      bbox = bounds(i,j);  // use bounds of first time step in builder
      return valid1(i,j+0) && valid1(i,j+1);
    }

  public:
    BufferT<unsigned int> segments;                 //!< array of line segment indices
    vector<BufferT<Vec3fa>> vertices;               //!< vertex array for each timestep
    array_t<std::unique_ptr<Buffer>,2> userbuffers; //!< user buffers // FIXME: no std::unique_ptr here
  };
}
