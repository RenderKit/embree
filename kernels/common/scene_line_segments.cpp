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

#include "scene_line_segments.h"
#include "scene.h"

namespace embree
{
#if defined(EMBREE_LOWEST_ISA)

  LineSegments::LineSegments (Device* device)
    : Geometry(device,LINE_SEGMENTS,0,1)
  {
    vertices.resize(numTimeSteps);
  }

  void LineSegments::enabling()
  {
    if (numTimeSteps == 1) scene->world.numLineSegments += numPrimitives;
    else                   scene->worldMB.numLineSegments += numPrimitives;
  }

  void LineSegments::disabling()
  {
    if (numTimeSteps == 1) scene->world.numLineSegments -= numPrimitives;
    else                   scene->worldMB.numLineSegments -= numPrimitives;
  }

  void LineSegments::setMask (unsigned mask)
  {
    this->mask = mask;
    Geometry::update();
  }

  void LineSegments::setSubtype(RTCGeometrySubtype type)
  {
    if (type != RTC_GEOMETRY_SUBTYPE_RIBBON)
      throw_RTCError(RTC_ERROR_INVALID_ARGUMENT,"invalid geometry subtype");
    
    Geometry::update();
  }

  void LineSegments::setBuffer(RTCBufferType type, unsigned int slot, RTCFormat format, const Ref<Buffer>& buffer, size_t offset, unsigned int num)
  {
#if 0
    /* verify that all accesses are 4 bytes aligned */
    if (((size_t(ptr) + offset) & 0x3) || (stride & 0x3))
      throw_RTCError(RTC_ERROR_INVALID_OPERATION,"data must be 4 bytes aligned");

    unsigned bid = type & 0xFFFF;
    if (type >= RTC_VERTEX_BUFFER0 && type < RTC_VERTEX_BUFFER_(RTC_MAX_TIME_STEPS)) 
    {
      if (bid >= vertices.size()) vertices.resize(bid+1);
      vertices[bid].set(device,ptr,offset,stride,size);
      vertices[bid].checkPadding16();
      vertices0 = vertices[0];
      //while (vertices.size() > 1 && vertices.back().getPtr() == nullptr)
      // vertices.pop_back();
      setNumTimeSteps((unsigned int)vertices.size());
    } 
    else if (type >= RTC_USER_VERTEX_BUFFER0 && type < RTC_USER_VERTEX_BUFFER0+RTC_MAX_USER_VERTEX_BUFFERS)
    {
      if (bid >= userbuffers.size()) userbuffers.resize(bid+1);
      userbuffers[bid] = Buffer<char>(device,size,stride);
      userbuffers[bid].set(device,ptr,offset,stride,size);
      userbuffers[bid].checkPadding16();
    }
    else if (type == RTC_BUFFER_TYPE_INDEX)
    {
      segments.set(device,ptr,offset,stride,size); 
      setNumPrimitives(size);
    }
    else
      throw_RTCError(RTC_ERROR_INVALID_ARGUMENT,"unknown buffer type");
#endif
  }

  bool LineSegments::verify ()
  { 
    /*! verify consistent size of vertex arrays */
    if (vertices.size() == 0) return false;
    for (const auto& buffer : vertices)
      if (buffer.size() != numVertices())
        return false;

    /*! verify segment indices */
    for (unsigned int i=0; i<size(); i++) {
      if (segments[i]+1 >= numVertices()) return false;
    }

    /*! verify vertices */
    for (const auto& buffer : vertices) {
      for (size_t i=0; i<buffer.size(); i++) {
	if (!isvalid(buffer[i].x)) return false;
        if (!isvalid(buffer[i].y)) return false;
        if (!isvalid(buffer[i].z)) return false;
        if (!isvalid(buffer[i].w)) return false;
      }
    }
    return true;
  }

  void LineSegments::interpolate(const RTCInterpolateArguments* const args)
  {
    unsigned int primID = args->primID;
    float u = args->u;
    RTCBufferType bufferType = args->bufferType;
    unsigned int bufferSlot = args->bufferSlot;
    float* P = args->P;
    float* dPdu = args->dPdu;
    float* ddPdudu = args->ddPdudu;
    unsigned int numFloats = args->numFloats;
      
    /* calculate base pointer and stride */
    assert((bufferType == RTC_BUFFER_TYPE_VERTEX && bufferSlot < numTimeSteps) ||
           (bufferType == RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE && bufferSlot <= 1));
    const char* src = nullptr;
    size_t stride = 0;
    if (bufferType == RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE) {
      src    = userbuffers[bufferSlot].getPtr();
      stride = userbuffers[bufferSlot].getStride();
    } else {
      src    = vertices[bufferSlot].getPtr();
      stride = vertices[bufferSlot].getStride();
    }
    
    for (unsigned int i=0; i<numFloats; i+=VSIZEX)
    {
      const size_t ofs = i*sizeof(float);
      const size_t segment = segments[primID];
      const vboolx valid = vintx((int)i)+vintx(step) < vintx(int(numFloats));
      const vfloatx p0 = vfloatx::loadu(valid,(float*)&src[(segment+0)*stride+ofs]);
      const vfloatx p1 = vfloatx::loadu(valid,(float*)&src[(segment+1)*stride+ofs]);
      if (P      ) vfloatx::storeu(valid,P+i,lerp(p0,p1,u));
      if (dPdu   ) vfloatx::storeu(valid,dPdu+i,p1-p0);
      if (ddPdudu) vfloatx::storeu(valid,dPdu+i,vfloatx(zero));
    }
  }
#endif

  namespace isa
  {
    LineSegments* createLineSegments(Device* device) {
      return new LineSegmentsISA(device);
    }
  }
}
