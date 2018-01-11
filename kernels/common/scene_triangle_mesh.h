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

#include "geometry.h"
#include "buffer.h"

namespace embree
{
  /*! Triangle Mesh */
  struct TriangleMesh : public Geometry
  {
    /*! type of this geometry */
    static const Geometry::Type geom_type = Geometry::TRIANGLE_MESH;

    /*! triangle indices */
    struct Triangle 
    {
      uint32_t v[3];

      /*! outputs triangle indices */
      __forceinline friend std::ostream &operator<<(std::ostream& cout, const Triangle& t) {
        return cout << "Triangle { " << t.v[0] << ", " << t.v[1] << ", " << t.v[2] << " }";
      }
    };

  public:

    /*! triangle mesh construction */
    TriangleMesh (Scene* scene, RTCGeometryFlags flags, size_t numTriangles, size_t numVertices, size_t numTimeSteps); 

    /* geometry interface */
  public:
    void enabling();
    void disabling();
    void setMask (unsigned mask);
    void setBuffer(RTCBufferType type, void* ptr, size_t offset, size_t stride, size_t size);
    void* map(RTCBufferType type);
    void unmap(RTCBufferType type);
    void preCommit();
    void postCommit();
    void immutable ();
    bool verify ();
    void interpolate(unsigned primID, float u, float v, RTCBufferType buffer, float* P, float* dPdu, float* dPdv, float* ddPdudu, float* ddPdvdv, float* ddPdudv, size_t numFloats);
    // FIXME: implement interpolateN

  public:

    /*! returns number of triangles */
    __forceinline size_t size() const {
      return triangles.size();
    }

    /*! returns number of vertices */
    __forceinline size_t numVertices() const {
      return vertices[0].size();
    }
    
    /*! returns i'th triangle*/
    __forceinline const Triangle& triangle(size_t i) const {
      return triangles[i];
    }

    /*! returns i'th vertex of the first time step  */
    __forceinline const Vec3fa vertex(size_t i) const {
      return vertices0[i];
    }

    /*! returns i'th vertex of the first time step */
    __forceinline const char* vertexPtr(size_t i) const {
      return vertices0.getPtr(i);
    }

    /*! returns i'th vertex of itime'th timestep */
    __forceinline const Vec3fa vertex(size_t i, size_t itime) const {
      return vertices[itime][i];
    }

    /*! returns i'th vertex of itime'th timestep */
    __forceinline const char* vertexPtr(size_t i, size_t itime) const {
      return vertices[itime].getPtr(i);
    }

    /*! calculates the bounds of the i'th triangle */
    __forceinline BBox3fa bounds(size_t i) const 
    {
      const Triangle& tri = triangle(i);
      const Vec3fa v0 = vertex(tri.v[0]);
      const Vec3fa v1 = vertex(tri.v[1]);
      const Vec3fa v2 = vertex(tri.v[2]);
      return BBox3fa(min(v0,v1,v2),max(v0,v1,v2));
    }

    /*! calculates the bounds of the i'th triangle at the itime'th timestep */
    __forceinline BBox3fa bounds(size_t i, size_t itime) const
    {
      const Triangle& tri = triangle(i);
      const Vec3fa v0 = vertex(tri.v[0],itime);
      const Vec3fa v1 = vertex(tri.v[1],itime);
      const Vec3fa v2 = vertex(tri.v[2],itime);
      return BBox3fa(min(v0,v1,v2),max(v0,v1,v2));
    }

    /*! calculates the interpolated bounds of the i'th triangle at the specified time */
    __forceinline BBox3fa bounds(size_t i, float time) const
    {
      float ftime; size_t itime = getTimeSegment(time, fnumTimeSegments, ftime);
      const BBox3fa b0 = bounds(i, itime+0);
      const BBox3fa b1 = bounds(i, itime+1);
      return lerp(b0, b1, ftime);
    }

    /*! check if the i'th primitive is valid at the itime'th timestep */
    __forceinline bool valid(size_t i, size_t itime) const {
      return valid(i, make_range(itime, itime));
    }

    /*! check if the i'th primitive is valid between the specified time range */
    __forceinline bool valid(size_t i, const range<size_t>& itime_range) const
    {
      const Triangle& tri = triangle(i);
      if (unlikely(tri.v[0] >= numVertices())) return false;
      if (unlikely(tri.v[1] >= numVertices())) return false;
      if (unlikely(tri.v[2] >= numVertices())) return false;

      for (size_t itime = itime_range.begin(); itime <= itime_range.end(); itime++)
      {
        if (!isvalid(vertex(tri.v[0],itime))) return false;
        if (!isvalid(vertex(tri.v[1],itime))) return false;
        if (!isvalid(vertex(tri.v[2],itime))) return false;
      }

      return true;
    }

    /*! calculates the linear bounds of the i'th primitive at the itimeGlobal'th time segment */
    __forceinline LBBox3fa linearBounds(size_t i, size_t itime) const {
      return LBBox3fa(bounds(i,itime+0),bounds(i,itime+1));
    }

    /*! calculates the build bounds of the i'th primitive, if it's valid */
    __forceinline bool buildBounds(size_t i, BBox3fa* bbox = nullptr) const
    {
      const Triangle& tri = triangle(i);
      if (unlikely(tri.v[0] >= numVertices())) return false;
      if (unlikely(tri.v[1] >= numVertices())) return false;
      if (unlikely(tri.v[2] >= numVertices())) return false;

      for (size_t t=0; t<numTimeSteps; t++)
      {
        const Vec3fa v0 = vertex(tri.v[0],t);
        const Vec3fa v1 = vertex(tri.v[1],t);
        const Vec3fa v2 = vertex(tri.v[2],t);
        if (unlikely(!isvalid(v0) || !isvalid(v1) || !isvalid(v2)))
          return false;
      }

      if (likely(bbox)) 
        *bbox = bounds(i);

      return true;
    }

    /*! calculates the build bounds of the i'th primitive at the itime'th time segment, if it's valid */
    __forceinline bool buildBounds(size_t i, size_t itime, BBox3fa& bbox) const
    {
      const Triangle& tri = triangle(i);
      if (unlikely(tri.v[0] >= numVertices())) return false;
      if (unlikely(tri.v[1] >= numVertices())) return false;
      if (unlikely(tri.v[2] >= numVertices())) return false;

      assert(itime+1 < numTimeSteps);
      const Vec3fa a0 = vertex(tri.v[0],itime+0); if (unlikely(!isvalid(a0))) return false;
      const Vec3fa a1 = vertex(tri.v[1],itime+0); if (unlikely(!isvalid(a1))) return false;
      const Vec3fa a2 = vertex(tri.v[2],itime+0); if (unlikely(!isvalid(a2))) return false;
      const Vec3fa b0 = vertex(tri.v[0],itime+1); if (unlikely(!isvalid(b0))) return false;
      const Vec3fa b1 = vertex(tri.v[1],itime+1); if (unlikely(!isvalid(b1))) return false;
      const Vec3fa b2 = vertex(tri.v[2],itime+1); if (unlikely(!isvalid(b2))) return false;
      
      /* use bounds of first time step in builder */
      bbox = BBox3fa(min(a0,a1,a2),max(a0,a1,a2));
      return true;
    }

    /*! calculates the linear bounds of the i'th primitive for the specified time range */
    __forceinline LBBox3fa linearBounds(size_t primID, const BBox1f& time_range) const {
      return LBBox3fa([&] (size_t itime) { return bounds(primID, itime); }, time_range, fnumTimeSegments);
    }

    /*! calculates the linear bounds of the i'th primitive for the specified time range */
    __forceinline bool linearBounds(size_t i, const BBox1f& time_range, LBBox3fa& bbox) const  {
      if (!valid(i, getTimeSegmentRange(time_range, fnumTimeSegments))) return false;
      bbox = linearBounds(i, time_range);
      return true;
    }

  public:
    APIBuffer<Triangle> triangles;                    //!< array of triangles
    BufferRefT<Vec3fa> vertices0;                     //!< fast access to first vertex buffer
    vector<APIBuffer<Vec3fa>> vertices;               //!< vertex array for each timestep
    vector<APIBuffer<char>> userbuffers;         //!< user buffers
  };

  namespace isa
  {
    struct TriangleMeshISA : public TriangleMesh
    {
      TriangleMeshISA (Scene* scene, RTCGeometryFlags flags, size_t numTriangles, size_t numVertices, size_t numTimeSteps)
        : TriangleMesh(scene,flags,numTriangles,numVertices,numTimeSteps) {}
    };
  }

  DECLARE_ISA_FUNCTION(TriangleMesh*, createTriangleMesh, Scene* COMMA RTCGeometryFlags COMMA size_t COMMA size_t COMMA size_t);
}
