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

#pragma once

#include "common/default.h"
#include "common/geometry.h"
#include "common/buildsource.h"
#include "common/buffer.h"

namespace embree
{
    struct BezierCurves : public Geometry
    {
      struct Vertex {
        float x,y,z,r;
      };

    public:
      BezierCurves (Scene* parent, RTCGeometryFlags flags, size_t numCurves, size_t numVertices, size_t numTimeSteps); 
      
    public:
      void setMask (unsigned mask);
      void enable ();
      void update ();
      void disable ();
      void erase ();
      void immutable ();
      bool verify ();
      void setBuffer(RTCBufferType type, void* ptr, size_t offset, size_t stride);
      void* map(RTCBufferType type);
      void unmap(RTCBufferType type);
      void setUserData (void* ptr, bool ispc);

      void enabling();
      void disabling();

    public:

      /* bool isEmpty () const {  */
      /*   return numCurves == 0; */
      /* } */
      
      /* virtual size_t groups () const {  */
      /*   return 1; */
      /* } */
      
      /* virtual size_t prims (size_t group, size_t* pnumVertices) const { */
      /*   if (pnumVertices) *pnumVertices = numVertices*numTimeSteps; */
      /*   return numCurves; */
      /* } */

      /* const BBox3fa bounds(size_t group, size_t prim) const { */
      /*   return bounds(prim); */
      /* } */

    public:

      __forceinline const int& curve(size_t i) const {
        assert(i < numCurves);
        return curves[i];
      }

      __forceinline const Vec3fa& vertex(size_t i, size_t j = 0) const {
        assert(i < numVertices);
        assert(j < 2);
        return (Vec3fa&)vertices[j][i];
      }

      __forceinline float radius(size_t i, size_t j = 0) const {
        assert(i < numVertices);
        assert(j < 2);
        return vertices[j][i].r;
      }

      __forceinline unsigned int maxSubdivisionSteps(const float eps, const Vec3fa& p0, const Vec3fa& p1, const Vec3fa& p2, const Vec3fa& p3) const {
	const Vec3fa d0 = abs(p0 - 2.0f * p1 + p2);
	const Vec3fa d1 = abs(p1 - 2.0f * p2 + p3);
	const float d0_max = max(d0.x,d0.y,d0.z);
	const float d1_max = max(d1.x,d1.y,d1.z);
	const float L0 = max(d0_max,d1_max);
	const float r0 = (sqrtf(2.0f) * 4 * (4-1) * L0) / (8.0f * eps);      
	return (unsigned int)logf(r0);
      }


      __forceinline BBox3fa bounds(size_t i) const 
      {
        const int index = curve(i);
        const float r0 = radius(index+0);
        const float r1 = radius(index+1);
        const float r2 = radius(index+2);
        const float r3 = radius(index+3);
        const Vec3fa& v0 = vertex(index+0);
        const Vec3fa& v1 = vertex(index+1);
        const Vec3fa& v2 = vertex(index+2);
        const Vec3fa& v3 = vertex(index+3);

#if 0
	BBox3fa b;
	b = empty;
	for (unsigned int step=0;step<=8;step++)
	  {
	    float t1 = (float)step / 8.0f;
	    float t0 = 1.0f - t1;
	    const float coeff0 = t0 * t0 * t0;
	    const float coeff1 = 3.0f * t1* t0 * t0;
	    const float coeff2 = 3.0f * t1* t1 * t0;
	    const float coeff3 = t1 * t1 * t1;
	    const Vec3fa p = coeff0 * v0 + coeff1 * v1 + coeff2 * v2 + coeff3 * v3; 
	    b.extend(p);
	  }
#else	
        const BBox3fa b = merge(BBox3fa(v0),BBox3fa(v1),BBox3fa(v2),BBox3fa(v3));
#endif

        return enlarge(b,Vec3fa(max(r0,r1,r2,r3)));
      }

      __forceinline bool anyMappedBuffers() const {
        return curves.isMapped() || vertices[0].isMapped() || vertices[1].isMapped();
      }

    public:
      unsigned mask;                    //!< for masking out geometry
      bool built;                       //!< geometry got built
      unsigned char numTimeSteps;       //!< number of time steps (1 or 2)

      BufferT<int> curves;              //!< array of curve indices
      bool needCurves;                  //!< set if curve indices required by acceleration structure
      size_t numCurves;                 //!< number of triangles

      BufferT<Vertex> vertices[2];      //!< vertex array
      bool needVertices;                //!< set if vertex array required by acceleration structure
      size_t numVertices;               //!< number of vertices
    };
}
