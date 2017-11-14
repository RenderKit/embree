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

  void LineSegments::setCurveType(RTCCurveType type)
  {
    if (type != RTC_CURVE_RIBBON)
      throw_RTCError(RTC_INVALID_ARGUMENT,"invalid curve type");
    
    Geometry::update();
  }

  void* LineSegments::newBuffer(RTCBufferType type, size_t stride, unsigned int size)
  {
    /* verify that all accesses are 4 bytes aligned */
    if (stride & 0x3) 
      throw_RTCError(RTC_INVALID_OPERATION,"data must be 4 bytes aligned");

    unsigned bid = type & 0xFFFF;
    if (type >= RTC_VERTEX_BUFFER0 && type < RTC_VERTEX_BUFFER_(RTC_MAX_TIME_STEPS)) 
    {
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
      segments.newBuffer(device,size,stride); 
      setNumPrimitives(size);
      return segments.get();
      // if (isEnabled() && size != (size_t)-1) disabling();
      // segments.set(ptr,offset,stride,size); 
      // setNumPrimitives(size);
      // if (isEnabled() && size != (size_t)-1) enabling();
    }
    else
      throw_RTCError(RTC_INVALID_ARGUMENT,"unknown buffer type");

    return nullptr;
  }

  void LineSegments::setBuffer(RTCBufferType type, void* ptr, size_t offset, size_t stride, unsigned int size)
  {
    /* verify that all accesses are 4 bytes aligned */
    if (((size_t(ptr) + offset) & 0x3) || (stride & 0x3))
      throw_RTCError(RTC_INVALID_OPERATION,"data must be 4 bytes aligned");

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
      userbuffers[bid] = APIBuffer<char>(device,size,stride);
      userbuffers[bid].set(device,ptr,offset,stride,size);
      userbuffers[bid].checkPadding16();
    }
    else if (type == RTC_INDEX_BUFFER) 
    {
      segments.set(device,ptr,offset,stride,size); 
      setNumPrimitives(size);
    }
    else
      throw_RTCError(RTC_INVALID_ARGUMENT,"unknown buffer type");
  }

  void* LineSegments::getBuffer(RTCBufferType type)
  {
    if (type == RTC_INDEX_BUFFER) {
      return segments.get();
    }
    else if (type >= RTC_VERTEX_BUFFER0 && type < RTC_VERTEX_BUFFER_(numTimeSteps)) {
      return vertices[type - RTC_VERTEX_BUFFER0].get();
    }
    else {
      throw_RTCError(RTC_INVALID_ARGUMENT,"unknown buffer type"); 
      return nullptr;
    }
  }

  bool LineSegments::verify ()
  { 
    /*! verify consistent size of vertex arrays */
    if (vertices.size() == 0) return false;
    for (const auto& buffer : vertices)
      if (buffer.size() != numVertices())
        return false;

    /*! verify segment indices */
    for (unsigned int i=0; i<numPrimitives; i++) {
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

  void LineSegments::interpolate(unsigned primID, float u, float v, RTCBufferType buffer, float* P, float* dPdu, float* dPdv, float* ddPdudu, float* ddPdvdv, float* ddPdudv, unsigned int numFloats)
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
