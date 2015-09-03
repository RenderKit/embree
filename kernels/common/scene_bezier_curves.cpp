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

#include "scene_bezier_curves.h"
#include "scene.h"
#include "subdiv/bezier_curve.h"

namespace embree
{
  BezierCurves::BezierCurves (Scene* parent, RTCGeometryFlags flags, size_t numPrimitives, size_t numVertices, size_t numTimeSteps) 
    : Geometry(parent,BEZIER_CURVES,numPrimitives,numTimeSteps,flags)
  {
    curves.init(parent->device,numPrimitives,sizeof(int));
    for (size_t i=0; i<numTimeSteps; i++) {
      vertices[i].init(parent->device,numVertices,sizeof(Vec3fa));
    }
    enabling();
  }

  void BezierCurves::enabling() 
  { 
    if (numTimeSteps == 1) atomic_add(&parent->numBezierCurves ,numPrimitives); 
    else                   atomic_add(&parent->numBezierCurves2,numPrimitives); 
  }
  
  void BezierCurves::disabling() 
  { 
    if (numTimeSteps == 1) atomic_add(&parent->numBezierCurves ,-(ssize_t)numPrimitives); 
    else                   atomic_add(&parent->numBezierCurves2,-(ssize_t)numPrimitives);
  }
  
  void BezierCurves::setMask (unsigned mask) 
  {
    if (parent->isStatic() && parent->isBuild())
      throw_RTCError(RTC_INVALID_OPERATION,"static geometries cannot get modified");

    this->mask = mask; 
    Geometry::update();
  }

  void BezierCurves::setBuffer(RTCBufferType type, void* ptr, size_t offset, size_t stride) 
  { 
    if (parent->isStatic() && parent->isBuild())
      throw_RTCError(RTC_INVALID_OPERATION,"static geometries cannot get modified");

    /* verify that all accesses are 4 bytes aligned */
    if (((size_t(ptr) + offset) & 0x3) || (stride & 0x3)) 
      throw_RTCError(RTC_INVALID_OPERATION,"data must be 4 bytes aligned");

    /* verify that all vertex accesses are 16 bytes aligned */
#if defined(__MIC__) && 0
    if (type == RTC_VERTEX_BUFFER0 || type == RTC_VERTEX_BUFFER1) {
      if (((size_t(ptr) + offset) & 0xF) || (stride & 0xF))
        throw_RTCError(RTC_INVALID_OPERATION,"data must be 16 bytes aligned");
    }
#endif

    switch (type) {
    case RTC_INDEX_BUFFER  : 
      curves.set(ptr,offset,stride); 
      break;
    case RTC_VERTEX_BUFFER0: 
      vertices[0].set(ptr,offset,stride); 
      vertices[0].checkPadding16();
      break;
    case RTC_VERTEX_BUFFER1: 
      vertices[1].set(ptr,offset,stride); 
      vertices[1].checkPadding16();
      break;
    case RTC_USER_VERTEX_BUFFER0  : 
      if (userbuffers[0] == nullptr) userbuffers[0].reset(new Buffer(parent->device,numVertices(),stride)); 
      userbuffers[0]->set(ptr,offset,stride);  
      userbuffers[0]->checkPadding16();
      break;
    case RTC_USER_VERTEX_BUFFER1  : 
      if (userbuffers[1] == nullptr) userbuffers[1].reset(new Buffer(parent->device,numVertices(),stride)); 
      userbuffers[1]->set(ptr,offset,stride);  
      userbuffers[1]->checkPadding16();
      break;
    default: 
      throw_RTCError(RTC_INVALID_ARGUMENT,"unknown buffer type"); 
      break;
    }
  }

  void* BezierCurves::map(RTCBufferType type) 
  {
    if (parent->isStatic() && parent->isBuild()) {
      throw_RTCError(RTC_INVALID_OPERATION,"static geometries cannot get modified");
      return nullptr;
    }

    switch (type) {
    case RTC_INDEX_BUFFER  : return curves.map(parent->numMappedBuffers);
    case RTC_VERTEX_BUFFER0: return vertices[0].map(parent->numMappedBuffers);
    case RTC_VERTEX_BUFFER1: return vertices[1].map(parent->numMappedBuffers);
    default: throw_RTCError(RTC_INVALID_ARGUMENT,"unknown buffer type"); return nullptr;
    }
  }

  void BezierCurves::unmap(RTCBufferType type) 
  {
    if (parent->isStatic() && parent->isBuild()) 
      throw_RTCError(RTC_INVALID_OPERATION,"static geometries cannot get modified");

    switch (type) {
    case RTC_INDEX_BUFFER  : curves.unmap(parent->numMappedBuffers); break;
    case RTC_VERTEX_BUFFER0: vertices[0].unmap(parent->numMappedBuffers); break;
    case RTC_VERTEX_BUFFER1: vertices[1].unmap(parent->numMappedBuffers); break;
    default: throw_RTCError(RTC_INVALID_ARGUMENT,"unknown buffer type"); break;
    }
  }

  void BezierCurves::immutable () 
  {
    const bool freeIndices = !parent->needBezierIndices;
    const bool freeVertices  = !parent->needBezierVertices;
    if (freeIndices) curves.free();
    if (freeVertices ) vertices[0].free();
    if (freeVertices ) vertices[1].free();
  }

  bool BezierCurves::verify () 
  {
    if (numTimeSteps == 2 && vertices[0].size() != vertices[1].size())
        return false;

    for (size_t i=0; i<numPrimitives; i++) {
      if (curves[i]+3 >= numVertices()) return false;
    }
    for (size_t j=0; j<numTimeSteps; j++) {
      BufferT<Vec3fa>& verts = vertices[j];
      for (size_t i=0; i<verts.size(); i++) {
        if (!isvalid(verts[i].x)) return false;
	if (!isvalid(verts[i].y)) return false;
	if (!isvalid(verts[i].z)) return false;
	if (!isvalid(verts[i].w)) return false;
      }
    }
    return true;
  }

  void BezierCurves::interpolate(unsigned primID, float u, float v, RTCBufferType buffer, float* P, float* dPdu, float* dPdv, size_t numFloats) 
  {
#if defined(DEBUG) // FIXME: use function pointers and also throw error in release mode
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
      const size_t curve = curves[primID];
      const float4 p0 = float4::loadu((float*)&src[(curve+0)*stride+ofs]);
      const float4 p1 = float4::loadu((float*)&src[(curve+1)*stride+ofs]);
      const float4 p2 = float4::loadu((float*)&src[(curve+2)*stride+ofs]);
      const float4 p3 = float4::loadu((float*)&src[(curve+3)*stride+ofs]);
      const bool4 valid = int4(i)+int4(step) < int4(numFloats);
      const BezierCurveT<float4> bezier(p0,p1,p2,p3,0.0f,1.0f,0);
      float4 Q, dQdu; bezier.eval(u,Q,dQdu);
      if (P   ) float4::storeu(valid,P+i,Q);
      if (dPdu) float4::storeu(valid,dPdu+i,dQdu);
    }

#else

    for (size_t i=0; i<numFloats; i+=16) 
    {
      size_t ofs = i*sizeof(float);
      bool16 mask = (i+16 > numFloats) ? (bool16)(((unsigned int)1 << (numFloats-i))-1) : bool16( true );
      const size_t curve = curves[primID];
      const float16 p0 = uload16f(mask,(float*)&src[(curve+0)*stride+ofs]);
      const float16 p1 = uload16f(mask,(float*)&src[(curve+1)*stride+ofs]);
      const float16 p2 = uload16f(mask,(float*)&src[(curve+2)*stride+ofs]);
      const float16 p3 = uload16f(mask,(float*)&src[(curve+3)*stride+ofs]);
      const BezierCurveT<float16> bezier(p0,p1,p2,p3,0.0f,1.0f,0);
      float16 Q, dQdu; bezier.eval(u,Q,dQdu);
      if (P   ) compactustore16f(mask,P+i,Q);
      if (dPdu) compactustore16f(mask,dPdu+i,dQdu);
    }


#endif
  }

  void BezierCurves::write(std::ofstream& file)
  {
    int type = BEZIER_CURVES;
    file.write((char*)&type,sizeof(int));
    file.write((char*)&numTimeSteps,sizeof(int));
    int numVerts = numVertices();
    file.write((char*)&numVerts,sizeof(int));
    file.write((char*)&numPrimitives,sizeof(int));

    for (size_t j=0; j<numTimeSteps; j++) {
      while ((file.tellp() % 16) != 0) { char c = 0; file.write(&c,1); }
      for (size_t i=0; i<vertices[j].size(); i++) {
        Vec3fa v = vertex(i,j);
        file.write((char*)&v,sizeof(Vec3fa)); 
      }
    }

    while ((file.tellp() % 16) != 0) { char c = 0; file.write(&c,1); }
    for (size_t i=0; i<numPrimitives; i++) file.write((char*)&curve(i),sizeof(int));  
  }
}
