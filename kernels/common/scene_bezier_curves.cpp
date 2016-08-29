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

#include "scene_bezier_curves.h"
#include "scene.h"
#include "../subdiv/bezier_curve.h"

namespace embree
{
  BezierCurves::BezierCurves (Scene* parent, SubType subtype, RTCGeometryFlags flags, size_t numPrimitives, size_t numVertices, size_t numTimeSteps) 
    : Geometry(parent,BEZIER_CURVES,numPrimitives,numTimeSteps,flags), subtype(subtype), tessellationRate(4)
  {
    curves.init(parent->device,numPrimitives,sizeof(int));
    for (size_t i=0; i<numTimeSteps; i++) {
      vertices[i].init(parent->device,numVertices,sizeof(Vec3fa));
    }
    enabling();
  }

  void BezierCurves::enabling() 
  { 
    if (numTimeSteps == 1) parent->world1.numBezierCurves += numPrimitives; 
    else                   parent->world2.numBezierCurves += numPrimitives; 
  }
  
  void BezierCurves::disabling() 
  { 
    if (numTimeSteps == 1) parent->world1.numBezierCurves -= numPrimitives; 
    else                   parent->world2.numBezierCurves -= numPrimitives;
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
  
  void BezierCurves::setTessellationRate(float N)
  {
    if (parent->isStatic() && parent->isBuild()) 
      throw_RTCError(RTC_INVALID_OPERATION,"static geometries cannot get modified");

    tessellationRate = clamp((int)N,1,16);
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

  void BezierCurves::interpolate(unsigned primID, float u, float v, RTCBufferType buffer, float* P, float* dPdu, float* dPdv, float* ddPdudu, float* ddPdvdv, float* ddPdudv, size_t numFloats) 
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
      const size_t curve = curves[primID];
      const vboolx valid = vintx(i)+vintx(step) < vintx(numFloats);
      const vfloatx p0 = vfloatx::loadu(valid,(float*)&src[(curve+0)*stride+ofs]);
      const vfloatx p1 = vfloatx::loadu(valid,(float*)&src[(curve+1)*stride+ofs]);
      const vfloatx p2 = vfloatx::loadu(valid,(float*)&src[(curve+2)*stride+ofs]);
      const vfloatx p3 = vfloatx::loadu(valid,(float*)&src[(curve+3)*stride+ofs]);
      
      const BezierCurveT<vfloatx> bezier(p0,p1,p2,p3,0.0f,1.0f,0);
      if (P      ) vfloatx::storeu(valid,P+i,      bezier.eval(u));
      if (dPdu   ) vfloatx::storeu(valid,dPdu+i,   bezier.eval_du(u));
      if (ddPdudu) vfloatx::storeu(valid,ddPdudu+i,bezier.eval_dudu(u));
    }
  }

  void BezierCurves::write(std::ofstream& file)
  {
    int type = BEZIER_CURVES;
    file.write((char*)&type,sizeof(int));
    file.write((char*)&numTimeSteps,sizeof(int));
    size_t numVerts = numVertices();
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
