// ======================================================================== //
// Copyright 2009-2017 Intel Corporation                                    //
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

#include "scene_triangle_mesh.h"
#include "scene.h"

namespace embree
{
#if defined(EMBREE_LOWEST_ISA)

  TriangleMesh::TriangleMesh (Device* device)
    : Geometry(device,TRIANGLE_MESH,0,1)
  {
    vertices.resize(numTimeSteps);
  }

  void TriangleMesh::enabling() 
  { 
    if (numTimeSteps == 1) scene->world.numTriangles += numPrimitives;
    else                   scene->worldMB.numTriangles += numPrimitives;
  }
  
  void TriangleMesh::disabling() 
  { 
    if (numTimeSteps == 1) scene->world.numTriangles -= numPrimitives;
    else                   scene->worldMB.numTriangles -= numPrimitives;
  }

  void TriangleMesh::setMask (unsigned mask) 
  {
    this->mask = mask; 
    Geometry::update();
  }

  void TriangleMesh::setGeometryIntersector(RTCGeometryIntersector type_in)
  {
    if (type_in != RTC_GEOMETRY_INTERSECTOR_SURFACE)
      throw_RTCError(RTC_INVALID_OPERATION,"invalid geometry intersector");
    
    Geometry::update();
  }
  
  void* TriangleMesh::newBuffer(RTCBufferType type, size_t stride, unsigned int size) 
  { 
    /* verify that all accesses are 4 bytes aligned */
    if (stride & 0x3) 
      throw_RTCError(RTC_INVALID_OPERATION,"data must be 4 bytes aligned");

    unsigned bid = type & 0xFFFF;
    if (type >= RTC_VERTEX_BUFFER0 && type < RTC_VERTEX_BUFFER_(RTC_MAX_TIME_STEPS)) 
    {
       /* if buffer is larger than 16GB the premultiplied index optimization does not work */
      if (stride*size > 16ll*1024ll*1024ll*1024ll)
       throw_RTCError(RTC_INVALID_OPERATION,"vertex buffer can be at most 16GB large");

      if (bid >= vertices.size()) vertices.resize(bid+1);
      vertices[bid].newBuffer(device,size,stride);
      vertices0 = vertices[0];
      setNumTimeSteps((unsigned int)vertices.size());
      return vertices[bid].get();
    } 
    else if (type >= RTC_USER_VERTEX_BUFFER0 && type < RTC_USER_VERTEX_BUFFER0+RTC_MAX_USER_VERTEX_BUFFERS)
    {
      if (bid >= userbuffers.size()) userbuffers.resize(bid+1);
      userbuffers[bid] = APIBuffer<char>(device,size,stride,true);
      return userbuffers[bid].get();
    }
    else if (type == RTC_INDEX_BUFFER) 
    {
      triangles.newBuffer(device,size,stride); 
      setNumPrimitives(size);
      return triangles.get();
    }
    else 
      throw_RTCError(RTC_INVALID_ARGUMENT,"unknown buffer type");

    return nullptr;
  }

  void TriangleMesh::setBuffer(RTCBufferType type, void* ptr, size_t offset, size_t stride, unsigned int size) 
  {
    /* verify that all accesses are 4 bytes aligned */
    if (((size_t(ptr) + offset) & 0x3) || (stride & 0x3)) 
      throw_RTCError(RTC_INVALID_OPERATION,"data must be 4 bytes aligned");

    unsigned bid = type & 0xFFFF;
    if (type >= RTC_VERTEX_BUFFER0 && type < RTC_VERTEX_BUFFER_(RTC_MAX_TIME_STEPS)) 
    {
      /* if buffer is larger than 16GB the premultiplied index optimization does not work */
      if (stride*size > 16ll*1024ll*1024ll*1024ll) {
        throw_RTCError(RTC_INVALID_OPERATION,"vertex buffer can be at most 16GB large");
      }

      if (bid >= vertices.size()) vertices.resize(bid+1);
      vertices[bid].set(device,ptr,offset,stride,size);
      vertices[bid].checkPadding16();
      vertices0 = vertices[0];
      //while (vertices.size() > 1 && vertices.back().getPtr() == nullptr)
      //  vertices.pop_back();
      setNumTimeSteps((unsigned int)vertices.size());
    }
    else if (type >= RTC_USER_VERTEX_BUFFER0 && type < RTC_USER_VERTEX_BUFFER0+RTC_MAX_USER_VERTEX_BUFFERS)
    {
      if (bid >= userbuffers.size()) userbuffers.resize(bid+1);
      userbuffers[bid] = APIBuffer<char>(device,size,stride);
      userbuffers[bid].set(device,ptr,offset,stride,size);
      userbuffers[bid].checkPadding16();
    }
    else if (type == RTC_INDEX_BUFFER) 
    {
      triangles.set(device,ptr,offset,stride,size); 
      setNumPrimitives(size);
    }
    else 
      throw_RTCError(RTC_INVALID_ARGUMENT,"unknown buffer type");
  }

  void* TriangleMesh::getBuffer(RTCBufferType type) 
  {
    if (type == RTC_INDEX_BUFFER) {
      return triangles.get();
    }
    else if (type >= RTC_VERTEX_BUFFER0 && type < RTC_VERTEX_BUFFER_(numTimeSteps)) {
      return vertices[type - RTC_VERTEX_BUFFER0].get();
    }
    else {
      throw_RTCError(RTC_INVALID_ARGUMENT,"unknown buffer type"); 
      return nullptr;
    }
  }

  void TriangleMesh::preCommit () 
  {
    /* verify that stride of all time steps are identical */
    for (unsigned int t=0; t<numTimeSteps; t++)
      if (vertices[t].getStride() != vertices[0].getStride())
        throw_RTCError(RTC_INVALID_OPERATION,"stride of vertex buffers have to be identical for each time step");

    Geometry::preCommit();
  }

  void TriangleMesh::postCommit () 
  {
    scene->vertices[geomID] = (int*) vertices0.getPtr();
    Geometry::postCommit();
  }

  bool TriangleMesh::verify () 
  {
    /*! verify size of vertex arrays */
    if (vertices.size() == 0) return false;
    for (const auto& buffer : vertices)
      if (buffer.size() != numVertices())
        return false;

    /*! verify size of user vertex arrays */
    for (const auto& buffer : userbuffers)
      if (buffer.size() != numVertices())
        return false;

    /*! verify triangle indices */
    for (size_t i=0; i<size(); i++) {     
      if (triangles[i].v[0] >= numVertices()) return false; 
      if (triangles[i].v[1] >= numVertices()) return false; 
      if (triangles[i].v[2] >= numVertices()) return false; 
    }

    /*! verify vertices */
    for (const auto& buffer : vertices)
      for (size_t i=0; i<buffer.size(); i++)
	if (!isvalid(buffer[i])) 
	  return false;

    return true;
  }
  
  void TriangleMesh::interpolate(unsigned primID, float u, float v, RTCBufferType buffer, float* P, float* dPdu, float* dPdv, float* ddPdudu, float* ddPdvdv, float* ddPdudv, unsigned int numFloats) 
  {
    /* calculate base pointer and stride */
    assert((buffer >= RTC_VERTEX_BUFFER0 && buffer < RTC_VERTEX_BUFFER_(numTimeSteps)) ||
           (buffer >= RTC_USER_VERTEX_BUFFER0 && buffer <= RTC_USER_VERTEX_BUFFER1));
    const char* src = nullptr; 
    size_t stride = 0;
    if (buffer >= RTC_USER_VERTEX_BUFFER0) {
      src    = userbuffers[buffer&0xFFFF].getPtr();
      stride = userbuffers[buffer&0xFFFF].getStride();
    } else {
      src    = vertices[buffer&0xFFFF].getPtr();
      stride = vertices[buffer&0xFFFF].getStride();
    }
    
    for (unsigned int i=0; i<numFloats; i+=VSIZEX)
    {
      size_t ofs = i*sizeof(float);
      const float w = 1.0f-u-v;
      const Triangle& tri = triangle(primID);
      const vboolx valid = vintx((int)i)+vintx(step) < vintx(int(numFloats));
      const vfloatx p0 = vfloatx::loadu(valid,(float*)&src[tri.v[0]*stride+ofs]);
      const vfloatx p1 = vfloatx::loadu(valid,(float*)&src[tri.v[1]*stride+ofs]);
      const vfloatx p2 = vfloatx::loadu(valid,(float*)&src[tri.v[2]*stride+ofs]);
      
      if (P) {
        vfloatx::storeu(valid,P+i,madd(w,p0,madd(u,p1,v*p2)));
      }
      if (dPdu) {
        assert(dPdu); vfloatx::storeu(valid,dPdu+i,p1-p0);
        assert(dPdv); vfloatx::storeu(valid,dPdv+i,p2-p0);
      }
      if (ddPdudu) {
        assert(ddPdudu); vfloatx::storeu(valid,ddPdudu+i,vfloatx(zero));
        assert(ddPdvdv); vfloatx::storeu(valid,ddPdvdv+i,vfloatx(zero));
        assert(ddPdudv); vfloatx::storeu(valid,ddPdudv+i,vfloatx(zero));
      }
    }
  }
  
#endif
  
  namespace isa
  {
    TriangleMesh* createTriangleMesh(Device* device) {
      return new TriangleMeshISA(device);
    }
  }
}
