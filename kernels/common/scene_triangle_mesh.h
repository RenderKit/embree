// ======================================================================== //
// Copyright 2009-2014 Intel Corporation                                    //
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

#include "common/geometry.h"
#include "common/buffer.h"

namespace embree
{
  /*! Triangle Mesh */
  struct TriangleMesh : public Geometry
  {
    struct Triangle {
      unsigned int v[3];
    };
    
  public:
    TriangleMesh (Scene* parent, RTCGeometryFlags flags, size_t numTriangles, size_t numVertices, size_t numTimeSteps); 
    
    /* geometry interface */
  public:
    void enabling();
    void disabling();
    void setMask (unsigned mask);
    void setBuffer(RTCBufferType type, void* ptr, size_t offset, size_t stride);
    void* map(RTCBufferType type);
    void unmap(RTCBufferType type);
    void setUserData (void* ptr, bool ispc);
    void immutable ();
    bool verify ();
    
  public:

    /*! returns number of triangles */
    __forceinline size_t size() const {
      return numTriangles;
    }
    
    /*! returns i'th triangle*/
    __forceinline const Triangle& triangle(size_t i) const {
      assert(i < numTriangles);
      return triangles[i];
    }
    
    /*! returns i'th vertex of j'th timestep */
    __forceinline const Vec3fa& vertex(size_t i, size_t j = 0) const 
    {
      assert(i < numVertices);
      assert(j < numTimeSteps);
      return vertices[j][i];
    }
    
    /*! returns the stride in bytes of the triangle buffer */
    __forceinline size_t getTriangleBufferStride() const {
      return triangles.getBufferStride();
    }

    /*! returns the stride in butes of the vertex buffer */
    __forceinline size_t getVertexBufferStride() const {
      return vertices[0].getBufferStride();
    }
    
    /*! calculates the bounds of the i'th triangle */
    __forceinline BBox3fa bounds(size_t i) const 
    {
      const Triangle& tri = triangle(i);
      const Vec3fa& v0 = vertex(tri.v[0]);
      const Vec3fa& v1 = vertex(tri.v[1]);
      const Vec3fa& v2 = vertex(tri.v[2]);
      return BBox3fa(min(v0,v1,v2),max(v0,v1,v2));
    }
    
#if defined(__MIC__)
    
    template<unsigned int HINT=0>
      __forceinline mic3f getTriangleVertices(const Triangle &tri,const size_t dim=0) const 
      {

	assert( tri.v[0] < numVertices );
	assert( tri.v[1] < numVertices );
	assert( tri.v[2] < numVertices );

#if !defined(__BUFFER_STRIDE__)
	
	const float *__restrict__ const vptr0 = (float*)&vertex(tri.v[0],dim);
	const float *__restrict__ const vptr1 = (float*)&vertex(tri.v[1],dim);
	const float *__restrict__ const vptr2 = (float*)&vertex(tri.v[2],dim);
#else
#if 0
	const mic_i tri_v  = uload16i(0x7,(int*)&tri);
	const mic_i stride = vertices[dim].getStride();

	const mic_i offset = tri_v * stride;
	__aligned(64) unsigned int offset_ptr[16];
	store16i(offset_ptr,offset);

	const char  *__restrict__ const base  = vertices[dim].getPtr();
	const float *__restrict__ const vptr0 = (float*)(base + offset_ptr[0]);
	const float *__restrict__ const vptr1 = (float*)(base + offset_ptr[1]);
	const float *__restrict__ const vptr2 = (float*)(base + offset_ptr[2]);

#else
	const mic_i stride = vertices[dim].getStride();

	mic_i offset0 = stride * mic_i(tri.v[0]);
	mic_i offset1 = stride * mic_i(tri.v[1]);
	mic_i offset2 = stride * mic_i(tri.v[2]);

	const char  *__restrict__ const base  = vertices[dim].getPtr();
	const float *__restrict__ const vptr0 = (float*)(base + offset0[0]);
	const float *__restrict__ const vptr1 = (float*)(base + offset1[0]);
	const float *__restrict__ const vptr2 = (float*)(base + offset2[0]);

#endif
	
	
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
    unsigned char numTimeSteps;       //!< number of time steps (1 or 2)
    
    BufferT<Triangle> triangles;      //!< array of triangles
    size_t numTriangles;              //!< number of triangles
    
    BufferT<Vec3fa> vertices[2];      //!< vertex array
    size_t numVertices;               //!< number of vertices
  };
}
