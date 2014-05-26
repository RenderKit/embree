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
    /*! Triangle Mesh */
    struct TriangleMesh : public Geometry, public BuildSource
    {
      struct Triangle {
        unsigned int v[3];
      };

    public:
      TriangleMesh (Scene* parent, RTCGeometryFlags flags, size_t numTriangles, size_t numVertices, size_t numTimeSteps); 
      
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

      __forceinline size_t size1() const {
	return numTriangles;
      }

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

      const BBox3fa bounds(size_t group, size_t prim) const {
        return bounds(prim);
      }

      void bounds(size_t group, size_t begin, size_t end, BBox3fa* bounds_o) const 
      {
        BBox3fa b = empty;
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
        return vertices[j][i];
      }

      __forceinline size_t getTriangleBufferStride() const {
	return triangles.getBufferStride();
      }

      __forceinline size_t getVertexBufferStride() const {
	return vertices[0].getBufferStride();
      }

      __forceinline BBox3fa bounds(size_t index) const 
      {
        const Triangle& tri = triangle(index);
        const Vec3fa& v0 = vertex(tri.v[0]);
        const Vec3fa& v1 = vertex(tri.v[1]);
        const Vec3fa& v2 = vertex(tri.v[2]);
	return BBox3fa( min(min(v0,v1),v2), max(max(v0,v1),v2) );
      }

      __forceinline bool anyMappedBuffers() const {
        return triangles.isMapped() || vertices[0].isMapped() || vertices[1].isMapped();
      }

#if defined(__MIC__)

      template<unsigned int HINT=0>
	__forceinline mic3f getTriangleVertices(const Triangle &tri,const size_t dim=0) const 
      {
#if !defined(__BUFFER_STRIDE__)

	const float *__restrict__ const vptr0 = (float*)&vertex(tri.v[0],dim);
	const float *__restrict__ const vptr1 = (float*)&vertex(tri.v[1],dim);
	const float *__restrict__ const vptr2 = (float*)&vertex(tri.v[2],dim);
#else
	const mic_i tri_v  = uload16i(0x7,(int*)&tri);
	const mic_i stride = vertices[dim].getStride();
	const mic_i offset = tri_v * stride;
	const unsigned int *__restrict__ const offset_ptr = (unsigned int*)&offset;
	const char  *__restrict__ const base  = vertices[dim].getPtr();
	const float *__restrict__ const vptr0 = (float*)(base + offset_ptr[0]);
	const float *__restrict__ const vptr1 = (float*)(base + offset_ptr[1]);
	const float *__restrict__ const vptr2 = (float*)(base + offset_ptr[2]);

	
#endif	
	if (HINT)
	  {
	    prefetch<HINT>(vptr1);
	    prefetch<HINT>(vptr2);
	  }

	const mic_f v0 = broadcast4to16f(vptr0);
	const mic_f v1 = broadcast4to16f(vptr1);
	const mic_f v2 = broadcast4to16f(vptr2);
	return mic3f(v0,v1,v2);
      }

#endif

    public:
      unsigned int mask;                //!< for masking out geometry
      bool built;                       //!< geometry got built
      unsigned char numTimeSteps;       //!< number of time steps (1 or 2)

      BufferT<Triangle> triangles;      //!< array of triangles
      bool needTriangles;               //!< set if triangle array required by acceleration structure
      size_t numTriangles;              //!< number of triangles

      BufferT<Vec3fa> vertices[2];      //!< vertex array
      bool needVertices;                //!< set if vertex array required by acceleration structure
      size_t numVertices;               //!< number of vertices
    };
}
