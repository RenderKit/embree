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
        return cout << "{ tri " << t.v[0] << ", " << t.v[1] << ", " << t.v[2] << " }";
      }
    };

    /*! triangle edge based on two indices */
    struct Edge 
    {
      uint64_t e;

      __forceinline Edge() {}

      __forceinline Edge(const uint32_t &v0,
                         const uint32_t &v1)
      {
        e = v0 < v1 ? (((uint64_t)v1 << 32) | (uint64_t)v0) : (((uint64_t)v0 << 32) | (uint64_t)v1);
      }

      __forceinline friend bool operator==( const Edge& a, const Edge& b ) 
      { return a.e == b.e; };

    };




    /* last edge of triangle 0 is shared */
    static __forceinline unsigned int pair_order(const unsigned int tri0_vtx_index0,
                                                 const unsigned int tri0_vtx_index1,
                                                 const unsigned int tri0_vtx_index2,
                                                 const unsigned int tri1_vtx_index)
    {
      return \
        (tri0_vtx_index0  <<  0) |
        (tri0_vtx_index1  <<  8) |
        (tri0_vtx_index2  << 16) |
        (tri1_vtx_index   << 24);
    }

    /*! tests if a shared exists between two triangles, returns -1 if no shared edge exists and the opposite vertex index of the second triangle if a shared edge exists */
    static __forceinline int sharedEdge(const Triangle &tri0,
                                        const Triangle &tri1)
    {
      const Edge tri0_edge0(tri0.v[0],tri0.v[1]);
      const Edge tri0_edge1(tri0.v[1],tri0.v[2]);
      const Edge tri0_edge2(tri0.v[2],tri0.v[0]);

      const Edge tri1_edge0(tri1.v[0],tri1.v[1]);
      const Edge tri1_edge1(tri1.v[1],tri1.v[2]);
      const Edge tri1_edge2(tri1.v[2],tri1.v[0]);

      int opp_vtx = -1;

      /* rotate triangle 0 to force shared edge between the first and last vertex */

      if (unlikely(tri0_edge0 == tri1_edge0)) return pair_order(1,2,0, 2); 
      if (unlikely(tri0_edge1 == tri1_edge0)) return pair_order(2,0,1, 2); 
      if (unlikely(tri0_edge2 == tri1_edge0)) return pair_order(0,1,2, 2);

      if (unlikely(tri0_edge0 == tri1_edge1)) return pair_order(1,2,0, 0); 
      if (unlikely(tri0_edge1 == tri1_edge1)) return pair_order(2,0,1, 0); 
      if (unlikely(tri0_edge2 == tri1_edge1)) return pair_order(0,1,2, 0); 

      if (unlikely(tri0_edge0 == tri1_edge2)) return pair_order(1,2,0, 1); 
      if (unlikely(tri0_edge1 == tri1_edge2)) return pair_order(2,0,1, 1); 
      if (unlikely(tri0_edge2 == tri1_edge2)) return pair_order(0,1,2, 1); 

      return -1;
    }

    
  public:

    /*! triangle mesh construction */
    TriangleMesh (Scene* parent, RTCGeometryFlags flags, size_t numTriangles, size_t numVertices, size_t numTimeSteps); 
  
    /*! writes the triangle mesh geometry to disk */
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

    /*! returns i'th vertex of j'th timestep */
    __forceinline const Vec3fa vertex(size_t i, size_t j = 0) const {
      return vertices[j][i];
    }

    /*! returns i'th vertex of j'th timestep */
    __forceinline const char* vertexPtr(size_t i, size_t j = 0) const {
      return vertices[j].getPtr(i);
    }

#if defined(__MIC__)    
    /*! returns the stride in bytes of the triangle buffer */
    __forceinline size_t getTriangleBufferStride() const {
      return triangles.getBufferStride();
    }

    /*! returns the stride in butes of the vertex buffer */
    __forceinline size_t getVertexBufferStride() const {
      return vertices[0].getBufferStride();
    }
#endif

    /*! calculates the bounds of the i'th triangle */
    __forceinline BBox3fa bounds(size_t i) const 
    {
      const Triangle& tri = triangle(i);
      const Vec3fa v0 = vertex(tri.v[0]);
      const Vec3fa v1 = vertex(tri.v[1]);
      const Vec3fa v2 = vertex(tri.v[2]);
      return BBox3fa(min(v0,v1,v2),max(v0,v1,v2));
    }

    /*! check if the i'th primitive is valid */
    __forceinline bool valid(size_t i, BBox3fa* bbox = nullptr) const 
    {
      const Triangle& tri = triangle(i);
      if (tri.v[0] >= numVertices()) return false;
      if (tri.v[1] >= numVertices()) return false;
      if (tri.v[2] >= numVertices()) return false;

      for (size_t j=0; j<numTimeSteps; j++) 
      {
        const Vec3fa v0 = vertex(tri.v[0],j);
        const Vec3fa v1 = vertex(tri.v[1],j);
        const Vec3fa v2 = vertex(tri.v[2],j);
        if (!isvalid(v0) || !isvalid(v1) || !isvalid(v2))
          return false;
      }

      if (bbox) 
        *bbox = bounds(i);

      return true;
    }

#if defined(__MIC__)


    template<unsigned int HINT=0>
      __forceinline Vec3vf16 getTriangleVertices(const Triangle &tri,const size_t dim=0) const 
      {
	assert( tri.v[0] < numVertices() );
	assert( tri.v[1] < numVertices() );
	assert( tri.v[2] < numVertices() );

#if !defined(RTCORE_BUFFER_STRIDE)
	
	const float *__restrict__ const vptr0 = (float*) vertices[dim].getPtr(tri.v[0]);
	const float *__restrict__ const vptr1 = (float*) vertices[dim].getPtr(tri.v[1]);
	const float *__restrict__ const vptr2 = (float*) vertices[dim].getPtr(tri.v[2]);

	const vfloat16 v0 = broadcast4to16f(vptr0); 
	const vfloat16 v1 = broadcast4to16f(vptr1); 
 	const vfloat16 v2 = broadcast4to16f(vptr2); 
	return Vec3vf16(v0,v1,v2);
#else
	const vint16 stride = vertices[dim].getBufferStride();

	const vint16 offset0_64 = mul_uint64_t(stride,vint16(tri.v[0]));
	const vint16 offset1_64 = mul_uint64_t(stride,vint16(tri.v[1]));
	const vint16 offset2_64 = mul_uint64_t(stride,vint16(tri.v[2]));

	const char  *__restrict__ const base  = vertices[dim].getPtr();
	const size_t off0 = offset0_64.uint64_t(0);
	const size_t off1 = offset1_64.uint64_t(0);
	const size_t off2 = offset2_64.uint64_t(0);

	const float *__restrict__ const vptr0_64 = (float*)(base + off0);
	const float *__restrict__ const vptr1_64 = (float*)(base + off1);
	const float *__restrict__ const vptr2_64 = (float*)(base + off2);

	if (HINT)
	{
	  prefetch<HINT>(vptr1_64);
	  prefetch<HINT>(vptr2_64);
	}

	assert( vptr0_64 == (float*)vertexPtr(tri.v[0],dim) );
	assert( vptr1_64 == (float*)vertexPtr(tri.v[1],dim) );
	assert( vptr2_64 == (float*)vertexPtr(tri.v[2],dim) );
	
	const vbool16 m_3f = 0x7;
	const vfloat16 v0 = shuffle4<0,0,0,0>(vfloat16::loadu(m_3f,vptr0_64));
	const vfloat16 v1 = shuffle4<0,0,0,0>(vfloat16::loadu(m_3f,vptr1_64));
	const vfloat16 v2 = shuffle4<0,0,0,0>(vfloat16::loadu(m_3f,vptr2_64));

	return Vec3vf16(select(0x7777,v0,vfloat16::zero()),select(0x7777,v1,vfloat16::zero()),select(0x7777,v2,vfloat16::zero()));
#endif	
      }
    
#endif
    
  public:
    BufferT<Triangle> triangles;                    //!< array of triangles
    array_t<BufferT<Vec3fa>,2> vertices;            //!< vertex array
    array_t<std::unique_ptr<Buffer>,2> userbuffers; //!< user buffers

  };
}
