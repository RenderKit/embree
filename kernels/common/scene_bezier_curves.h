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

        const BBox3fa b = merge(BBox3fa(v0),BBox3fa(v1),BBox3fa(v2),BBox3fa(v3));

        return enlarge(b,Vec3fa(max(r0,r1,r2,r3)));
      }

      __forceinline const Vec3fa *fristVertexPtr(size_t i) const 
      {
        const int index = curve(i);
        return &vertex(index+0);
      }

#if defined(__MIC__)

      __forceinline mic2f bounds_mic2f(size_t i) const 
      {
        const int index = curve(i);
        const Vec3fa& cp0 = vertex(index+0);
        const Vec3fa& cp1 = vertex(index+1);
        const Vec3fa& cp2 = vertex(index+2);
        const Vec3fa& cp3 = vertex(index+3);
	
	const mic_f v0 = broadcast4to16f((float*)&cp0);
	const mic_f v1 = broadcast4to16f((float*)&cp1);
	const mic_f v2 = broadcast4to16f((float*)&cp2);
	const mic_f v3 = broadcast4to16f((float*)&cp3);

	const mic_f b_min = min(min(v0,v1),min(v2,v3));
	const mic_f b_max = max(max(v0,v1),max(v2,v3));

	const mic_f b_min_r = b_min - swDDDD(b_max);
	const mic_f b_max_r = b_max + swDDDD(b_max);

        return mic2f(b_min_r,b_max_r);
      }
      
#endif

      __forceinline BBox3fa subBounds(size_t curveID, size_t segmentID) const 
      {
	assert(curveID < numCurves);
	assert(segmentID < 8);
        const int index = curve(curveID);
        const float r0 = radius(index+0);
        const float r1 = radius(index+1);
        const float r2 = radius(index+2);
        const float r3 = radius(index+3);
        const Vec3fa& v0 = vertex(index+0);
        const Vec3fa& v1 = vertex(index+1);
        const Vec3fa& v2 = vertex(index+2);
        const Vec3fa& v3 = vertex(index+3);

	BBox3fa b(empty);
	{
	  unsigned int step = segmentID;
	  float t1 = (float)step / 8.0f;
	  float t0 = 1.0f - t1;
	  const float coeff0 = t0 * t0 * t0;
	  const float coeff1 = 3.0f * t1* t0 * t0;
	  const float coeff2 = 3.0f * t1* t1 * t0;
	  const float coeff3 = t1 * t1 * t1;
	  const Vec3fa p = coeff0 * v0 + coeff1 * v1 + coeff2 * v2 + coeff3 * v3; 
	  b.extend(p);
	}

	{
	  unsigned int step = segmentID+1;
	  float t1 = (float)step / 8.0f;
	  float t0 = 1.0f - t1;
	  const float coeff0 = t0 * t0 * t0;
	  const float coeff1 = 3.0f * t1* t0 * t0;
	  const float coeff2 = 3.0f * t1* t1 * t0;
	  const float coeff3 = t1 * t1 * t1;
	  const Vec3fa p = coeff0 * v0 + coeff1 * v1 + coeff2 * v2 + coeff3 * v3; 
	  b.extend(p);
	}
        return enlarge(b,Vec3fa(max(r0,r1,r2,r3)));
      }

      __forceinline bool anyMappedBuffers() const {
        return curves.isMapped() || vertices[0].isMapped() || vertices[1].isMapped();
      }

    public:
      unsigned int mask;                //!< for masking out geometry
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
