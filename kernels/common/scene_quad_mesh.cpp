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

#include "scene_quad_mesh.h"
#include "scene.h"

namespace embree
{

  QuadMesh::QuadMesh (Scene* parent, RTCGeometryFlags flags, size_t numQuads, size_t numVertices, size_t numTimeSteps)
    : Geometry(parent,QUAD_MESH,numQuads,numTimeSteps,flags)
  {
    quads.init(parent->device,numQuads,sizeof(Quad));
    for (size_t i=0; i<numTimeSteps; i++) {
      vertices[i].init(parent->device,numVertices,sizeof(Vec3fa));
    }
    enabling();
  }
  
  void QuadMesh::enabling() 
  { 
    if (numTimeSteps == 1) atomic_add(&parent->world1.numQuads,quads.size());
    else                   atomic_add(&parent->world2.numQuads,quads.size());
  }
  
  void QuadMesh::disabling() 
  { 
    if (numTimeSteps == 1) atomic_add(&parent->world1.numQuads,-(ssize_t)quads.size());
    else                   atomic_add(&parent->world2.numQuads,-(ssize_t)quads.size());
  }

  void QuadMesh::setMask (unsigned mask) 
  {
    if (parent->isStatic() && parent->isBuild())
      throw_RTCError(RTC_INVALID_OPERATION,"static scenes cannot get modified");

    this->mask = mask; 
    Geometry::update();
  }

  void QuadMesh::setBuffer(RTCBufferType type, void* ptr, size_t offset, size_t stride) 
  { 
    if (parent->isStatic() && parent->isBuild()) 
      throw_RTCError(RTC_INVALID_OPERATION,"static scenes cannot get modified");

    /* verify that all accesses are 4 bytes aligned */
    if (((size_t(ptr) + offset) & 0x3) || (stride & 0x3)) 
      throw_RTCError(RTC_INVALID_OPERATION,"data must be 4 bytes aligned");

    switch (type) {
    case RTC_INDEX_BUFFER  : 
      quads.set(ptr,offset,stride); 
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

  void* QuadMesh::map(RTCBufferType type) 
  {
    if (parent->isStatic() && parent->isBuild())
      throw_RTCError(RTC_INVALID_OPERATION,"static scenes cannot get modified");

    switch (type) {
    case RTC_INDEX_BUFFER  : return quads.map(parent->numMappedBuffers);
    case RTC_VERTEX_BUFFER0: return vertices[0].map(parent->numMappedBuffers);
    case RTC_VERTEX_BUFFER1: return vertices[1].map(parent->numMappedBuffers);
    default                : throw_RTCError(RTC_INVALID_ARGUMENT,"unknown buffer type"); return nullptr;
    }
  }

  void QuadMesh::unmap(RTCBufferType type) 
  {
    if (parent->isStatic() && parent->isBuild())
      throw_RTCError(RTC_INVALID_OPERATION,"static scenes cannot get modified");

    switch (type) {
    case RTC_INDEX_BUFFER  : quads  .unmap(parent->numMappedBuffers); break;
    case RTC_VERTEX_BUFFER0: vertices[0].unmap(parent->numMappedBuffers); break;
    case RTC_VERTEX_BUFFER1: vertices[1].unmap(parent->numMappedBuffers); break;
    default                : throw_RTCError(RTC_INVALID_ARGUMENT,"unknown buffer type"); break;
    }
  }

  void QuadMesh::immutable () 
  {
    const bool freeQuads = !parent->needQuadIndices;
    const bool freeVertices  = !parent->needQuadVertices;
    if (freeQuads) quads.free(); 
    if (freeVertices ) vertices[0].free();
    if (freeVertices ) vertices[1].free();
  }

  bool QuadMesh::verify () 
  {
    /*! verify consistent size of vertex arrays */
    if (numTimeSteps == 2 && vertices[0].size() != vertices[1].size())
        return false;

    /*! verify proper quad indices */
    for (size_t i=0; i<quads.size(); i++) {     
      if (quads[i].v[0] >= numVertices()) return false; 
      if (quads[i].v[1] >= numVertices()) return false; 
      if (quads[i].v[2] >= numVertices()) return false; 
      if (quads[i].v[3] >= numVertices()) return false; 
    }

    /*! verify proper quad vertices */
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

  void QuadMesh::interpolate(unsigned primID, float u, float v, RTCBufferType buffer, float* P, float* dPdu, float* dPdv, size_t numFloats)  // FIXME: do not interpolate bilinearly here!
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

#if !defined(__MIC__)

    for (size_t i=0; i<numFloats; i+=4)
    {
      size_t ofs = i*sizeof(float);
      const Quad& tri = quad(primID);
      const vfloat4 p0 = vfloat4::loadu((float*)&src[tri.v[0]*stride+ofs]);
      const vfloat4 p1 = vfloat4::loadu((float*)&src[tri.v[1]*stride+ofs]);
      const vfloat4 p2 = vfloat4::loadu((float*)&src[tri.v[2]*stride+ofs]);
      const vfloat4 p3 = vfloat4::loadu((float*)&src[tri.v[3]*stride+ofs]);
      const vbool4 valid = vint4(i)+vint4(step) < vint4(numFloats);
      const vfloat4 p  = ((1.0f-u)*p0 + u*p1)*(1.0f-v) + v*((1.0f-u)*p3 + u*p2);
      const vfloat4 du = (1.0f-v) * (p1-p0) + v * (p2-p3);
      const vfloat4 dv = (1.0f-u) * (p3-p0) + u * (p2-p1);
      if (P   ) vfloat4::storeu(valid,P+i,p);
      if (dPdu) vfloat4::storeu(valid,dPdu+i,du);
      if (dPdv) vfloat4::storeu(valid,dPdv+i,dv);
    }

#else

    for (size_t i=0; i<numFloats; i+=16) 
    {
      size_t ofs = i*sizeof(float);
      vbool16 mask = (i+16 > numFloats) ? (vbool16)(((unsigned int)1 << (numFloats-i))-1) : vbool16( true );
      const float w = 1.0f-u-v;
      const Quad& tri = quad(primID);
      const vfloat16 p0 = vfloat16::loadu(mask,(float*)&src[tri.v[0]*stride+ofs]);
      const vfloat16 p1 = vfloat16::loadu(mask,(float*)&src[tri.v[1]*stride+ofs]);
      const vfloat16 p2 = vfloat16::loadu(mask,(float*)&src[tri.v[2]*stride+ofs]);
      const vfloat16 p3 = vfloat16::loadu(mask,(float*)&src[tri.v[3]*stride+ofs]);
      const vfloat16 p  = (1.0f-v)*((1.0f-u)*p0 + u*p1) + v*((1.0f-u)*p3 + u*p2);
      const vfloat16 du = (1.0f-v) * (p1-p0) + v * (p2-p3);
      const vfloat16 dv = (1.0f-u) * (p3-p0) + u * (p2-p1);
      if (P   ) vfloat16::storeu_compact(mask,P+i,p);
      if (dPdu) vfloat16::storeu_compact(mask,dPdu+i,du);
      if (dPdv) vfloat16::storeu_compact(mask,dPdv+i,dv);
    }
#endif
  }

  void QuadMesh::write(std::ofstream& file)
  {
    int type = QUAD_MESH;
    file.write((char*)&type,sizeof(int));
    file.write((char*)&numTimeSteps,sizeof(int));
    int numVerts = numVertices();
    file.write((char*)&numVerts,sizeof(int));
    int numQuads = quads.size();
    file.write((char*)&numQuads,sizeof(int));

    for (size_t j=0; j<numTimeSteps; j++) {
      while ((file.tellp() % 16) != 0) { char c = 0; file.write(&c,1); }
      for (size_t i=0; i<numVerts; i++) file.write((char*)vertexPtr(i,j),sizeof(Vec3fa));  
    }

    while ((file.tellp() % 16) != 0) { char c = 0; file.write(&c,1); }
    for (size_t i=0; i<numQuads; i++) file.write((char*)&quad(i),sizeof(Quad));  

  }
}
