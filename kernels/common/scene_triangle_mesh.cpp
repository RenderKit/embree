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

#include "scene_triangle_mesh.h"
#include "scene.h"

namespace embree
{

  TriangleMesh::TriangleMesh (Scene* parent, RTCGeometryFlags flags, size_t numTriangles, size_t numVertices, size_t numTimeSteps)
    : Geometry(parent,TRIANGLE_MESH,numTriangles,numTimeSteps,flags)
  {
    triangles.init(parent->device,numTriangles,sizeof(Triangle));
    for (size_t i=0; i<numTimeSteps; i++) {
      vertices[i].init(parent->device,numVertices,sizeof(Vec3fa));
    }
    enabling();
  }
  
  void TriangleMesh::enabling() 
  { 
    if (numTimeSteps == 1) parent->world1.numTriangles += triangles.size();
    else                   parent->world2.numTriangles += triangles.size();
  }
  
  void TriangleMesh::disabling() 
  { 
    if (numTimeSteps == 1) parent->world1.numTriangles -= triangles.size();
    else                   parent->world2.numTriangles -= triangles.size();
  }

  void TriangleMesh::setMask (unsigned mask) 
  {
    if (parent->isStatic() && parent->isBuild())
      throw_RTCError(RTC_INVALID_OPERATION,"static scenes cannot get modified");

    this->mask = mask; 
    Geometry::update();
  }

  void TriangleMesh::setBuffer(RTCBufferType type, void* ptr, size_t offset, size_t stride) 
  { 
    if (parent->isStatic() && parent->isBuild()) 
      throw_RTCError(RTC_INVALID_OPERATION,"static scenes cannot get modified");

    /* verify that all accesses are 4 bytes aligned */
    if (((size_t(ptr) + offset) & 0x3) || (stride & 0x3)) 
      throw_RTCError(RTC_INVALID_OPERATION,"data must be 4 bytes aligned");

    switch (type) {
    case RTC_INDEX_BUFFER  : 
      triangles.set(ptr,offset,stride); 
      break;
    case RTC_VERTEX_BUFFER0: 
      vertices[0].set(ptr,offset,stride); 
      vertices[0].checkPadding16();
      break;
    case RTC_VERTEX_BUFFER1: 
      vertices[1].set(ptr,offset,stride); 
      vertices[1].checkPadding16();
      break;
    case RTC_USER_VERTEX_BUFFER0: 
      if (userbuffers[0] == nullptr) userbuffers[0].reset(new Buffer(parent->device,numVertices(),stride)); 
      userbuffers[0]->set(ptr,offset,stride);  
      userbuffers[0]->checkPadding16();
      break;
    case RTC_USER_VERTEX_BUFFER1: 
      if (userbuffers[1] == nullptr) userbuffers[1].reset(new Buffer(parent->device,numVertices(),stride)); 
      userbuffers[1]->set(ptr,offset,stride);  
      userbuffers[1]->checkPadding16();
      break;

    default: 
      throw_RTCError(RTC_INVALID_ARGUMENT,"unknown buffer type");
    }
  }

  void* TriangleMesh::map(RTCBufferType type) 
  {
    if (parent->isStatic() && parent->isBuild())
      throw_RTCError(RTC_INVALID_OPERATION,"static scenes cannot get modified");

    switch (type) {
    case RTC_INDEX_BUFFER  : return triangles.map(parent->numMappedBuffers);
    case RTC_VERTEX_BUFFER0: return vertices[0].map(parent->numMappedBuffers);
    case RTC_VERTEX_BUFFER1: return vertices[1].map(parent->numMappedBuffers);
    default                : throw_RTCError(RTC_INVALID_ARGUMENT,"unknown buffer type"); return nullptr;
    }
  }

  void TriangleMesh::unmap(RTCBufferType type) 
  {
    if (parent->isStatic() && parent->isBuild())
      throw_RTCError(RTC_INVALID_OPERATION,"static scenes cannot get modified");

    switch (type) {
    case RTC_INDEX_BUFFER  : triangles  .unmap(parent->numMappedBuffers); break;
    case RTC_VERTEX_BUFFER0: vertices[0].unmap(parent->numMappedBuffers); break;
    case RTC_VERTEX_BUFFER1: vertices[1].unmap(parent->numMappedBuffers); break;
    default                : throw_RTCError(RTC_INVALID_ARGUMENT,"unknown buffer type"); break;
    }
  }

  void TriangleMesh::immutable () 
  {
    const bool freeTriangles = !parent->needTriangleIndices;
    const bool freeVertices  = !parent->needTriangleVertices;
    if (freeTriangles) triangles.free(); 
    if (freeVertices ) vertices[0].free();
    if (freeVertices ) vertices[1].free();
  }

  bool TriangleMesh::verify () 
  {
    /*! verify consistent size of vertex arrays */
    if (numTimeSteps == 2 && vertices[0].size() != vertices[1].size())
        return false;

    /*! verify proper triangle indices */
    for (size_t i=0; i<triangles.size(); i++) {     
      if (triangles[i].v[0] >= numVertices()) return false; 
      if (triangles[i].v[1] >= numVertices()) return false; 
      if (triangles[i].v[2] >= numVertices()) return false; 
    }

    /*! verify proper triangle vertices */
    for (size_t j=0; j<numTimeSteps; j++) 
    {
      BufferT<Vec3fa>& verts = vertices[j];
      for (size_t i=0; i<verts.size(); i++) {
	if (!isvalid(verts[i])) 
	  return false;
      }
    }
    return true;
  }

  void TriangleMesh::interpolate(unsigned primID, float u, float v, RTCBufferType buffer, float* P, float* dPdu, float* dPdv, float* ddPdudu, float* ddPdvdv, float* ddPdudv, size_t numFloats) 
  {
    /* test if interpolation is enabled */
#if defined(DEBUG)
    if ((parent->aflags & RTC_INTERPOLATE) == 0) 
      throw_RTCError(RTC_INVALID_OPERATION,"rtcInterpolate can only get called when RTC_INTERPOLATE is enabled for the scene");
#endif

    /* calculate base pointer and stride */
    assert((buffer >= RTC_VERTEX_BUFFER0 && buffer <= RTC_VERTEX_BUFFER1) ||
           (buffer >= RTC_USER_VERTEX_BUFFER0 && buffer <= RTC_USER_VERTEX_BUFFER1));
    const char* src = nullptr; 
    size_t stride = 0;
    if (buffer >= RTC_USER_VERTEX_BUFFER0) {
      src    = userbuffers[buffer&0xFFFF]->getPtr();
      stride = userbuffers[buffer&0xFFFF]->getStride();
    } else {
      src    = vertices[buffer&0xFFFF].getPtr();
      stride = vertices[buffer&0xFFFF].getStride();
    }

    for (size_t i=0; i<numFloats; i+=VSIZEX)
    {
      size_t ofs = i*sizeof(float);
      const float w = 1.0f-u-v;
      const Triangle& tri = triangle(primID);
      const vboolx valid = vintx(i)+vintx(step) < vintx(numFloats);
      const vfloatx p0 = vfloatx::loadu(valid,(float*)&src[tri.v[0]*stride+ofs]);
      const vfloatx p1 = vfloatx::loadu(valid,(float*)&src[tri.v[1]*stride+ofs]);
      const vfloatx p2 = vfloatx::loadu(valid,(float*)&src[tri.v[2]*stride+ofs]);
      
      if (P) {
        vfloatx::storeu(valid,P+i,w*p0 + u*p1 + v*p2);
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

  void TriangleMesh::write(std::ofstream& file)
  {
    int type = TRIANGLE_MESH;
    file.write((char*)&type,sizeof(int));
    file.write((char*)&numTimeSteps,sizeof(int));
    size_t numVerts = numVertices();
    file.write((char*)&numVerts,sizeof(int));
    size_t numTriangles = triangles.size();
    file.write((char*)&numTriangles,sizeof(int));

    for (size_t j=0; j<numTimeSteps; j++) {
      while ((file.tellp() % 16) != 0) { char c = 0; file.write(&c,1); }
      for (size_t i=0; i<numVerts; i++) file.write((char*)vertexPtr(i,j),sizeof(Vec3fa));  
    }

    while ((file.tellp() % 16) != 0) { char c = 0; file.write(&c,1); }
    for (size_t i=0; i<numTriangles; i++) file.write((char*)&triangle(i),sizeof(Triangle));  

  }
}
