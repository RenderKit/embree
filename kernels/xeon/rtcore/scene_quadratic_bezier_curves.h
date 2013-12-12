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

#ifndef __EMBREE_QUADRATIC_BEZIER_CURVES_SCENE_H__
#define __EMBREE_QUADRATIC_BEZIER_CURVES_SCENE_H__

#include "common/default.h"
#include "rtcore/geometry.h"

namespace embree
{
  namespace QuadraticBezierCurvesScene
  {

    struct QuadraticBezierCurves : public Geometry
    {
      struct Curve {
        int v0,v1,v2,v3;
      };

      struct Vertex {
        float x,y,z,r;
      };

    public:
      QuadraticBezierCurves (Scene* parent, RTCGeometryFlags flags, size_t numCurves, size_t numVertices); 
      ~QuadraticBezierCurves ();
      
    public:
      void setMask (unsigned mask);
      void enable ();
      void update ();
      void disable ();
      void erase ();
      void* map(RTCBufferType type);
      void unmap(RTCBufferType type);
      void postBuild(bool needVertices);

      void enabling() {}
      void disabling() {}

    public:

      __forceinline const Curve& curve(size_t i) const {
        assert(i < numCurves);
        return curves[i];
      }

      __forceinline const Vec3fa& vertex(size_t i) const {
        assert(i < numVertices);
        return (Vec3fa&)vertices[i];
      }

      __forceinline float radius(size_t i) const {
        assert(i < numVertices);
        return vertices[i].r;
      }

      __forceinline BBox3f bounds(size_t i) const 
      {
        const Curve& index = curve(i);
        const float r0 = radius(index.v0);
        const float r1 = radius(index.v1);
        const float r2 = radius(index.v2);
        const float r3 = radius(index.v3);
        const Vec3fa& v0 = vertex(index.v0);
        const Vec3fa& v1 = vertex(index.v1);
        const Vec3fa& v2 = vertex(index.v2);
        const Vec3fa& v3 = vertex(index.v3);
        const BBox3f b = merge(BBox3f(v0),BBox3f(v1),BBox3f(v2),BBox3f(v3));
        return enlarge(b,Vec3fa(max(r0,r1,r2,r3)));
      }

      __forceinline bool anyMappedBuffers() const {
        return mappedCurves || mappedVertices;
      }

    public:
      unsigned mask;              //!< for masking out geometry
      bool built;                //!< geometry got built

      Curve* curves;              //!< array of curves
      size_t numCurves;           //!< number of curves in array
      bool mappedCurves;          //!< is curves buffer mapped?

      Vertex* vertices;           //!< array of vertices and radii
      size_t numVertices;         //!< number of vertices in array
      bool mappedVertices;        //!< is vertices buffer mapped?
    };

  }
}

#endif
