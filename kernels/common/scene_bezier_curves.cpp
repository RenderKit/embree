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

#include "scene_bezier_curves.h"
#include "scene.h"

namespace embree
{
#if defined(EMBREE_LOWEST_ISA)

  NativeCurves::NativeCurves (Device* device, CurveType type, CurveSubtype subtype)
    : Geometry(device,BEZIER_CURVES,0,1), type(type), subtype(subtype), tessellationRate(4)
  {
    vertices.resize(numTimeSteps);
  }

  void NativeCurves::enabling() 
  {
    if (numTimeSteps == 1) scene->world.numBezierCurves += numPrimitives; 
    else                   scene->worldMB.numBezierCurves += numPrimitives; 
  }
  
  void NativeCurves::disabling() 
  {
    if (numTimeSteps == 1) scene->world.numBezierCurves -= numPrimitives; 
    else                   scene->worldMB.numBezierCurves -= numPrimitives;
  }
  
  void NativeCurves::setMask (unsigned mask) 
  {
    this->mask = mask; 
    Geometry::update();
  }

  void NativeCurves::setNumTimeSteps (unsigned int numTimeSteps)
  {
    vertices.resize(numTimeSteps);
    Geometry::setNumTimeSteps(numTimeSteps);
  }
  
  void NativeCurves::setVertexAttributeCount (unsigned int N)
  {
    vertexAttribs.resize(N);
    Geometry::update();
  }

  void NativeCurves::setBuffer(RTCBufferType type, unsigned int slot, RTCFormat format, const Ref<Buffer>& buffer, size_t offset, size_t stride, unsigned int num)
  { 
    /* verify that all accesses are 4 bytes aligned */
    if ((type != RTC_BUFFER_TYPE_FLAGS) && (((size_t(buffer->getPtr()) + offset) & 0x3) || (stride & 0x3)))
      throw_RTCError(RTC_ERROR_INVALID_OPERATION, "data must be 4 bytes aligned");

    if (type == RTC_BUFFER_TYPE_VERTEX)
    {
      if (format != RTC_FORMAT_FLOAT4)
        throw_RTCError(RTC_ERROR_INVALID_OPERATION, "invalid vertex buffer format");

      if (slot >= vertices.size())
        throw_RTCError(RTC_ERROR_INVALID_OPERATION, "invalid vertex buffer slot");
      
      vertices[slot].set(buffer, offset, stride, num, format);
      vertices[slot].checkPadding16();
    }
    else if (type == RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE)
    {
      if (format < RTC_FORMAT_FLOAT || format > RTC_FORMAT_FLOAT16)
        throw_RTCError(RTC_ERROR_INVALID_OPERATION, "invalid vertex attribute buffer format");

      if (slot >= vertexAttribs.size())
        throw_RTCError(RTC_ERROR_INVALID_OPERATION, "invalid vertex attribute buffer slot");
      
      vertexAttribs[slot].set(buffer, offset, stride, num, format);
      vertexAttribs[slot].checkPadding16();
    }
    else if (type == RTC_BUFFER_TYPE_INDEX)
    {
      if (slot != 0)
        throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "invalid buffer slot");
      if (format != RTC_FORMAT_UINT)
        throw_RTCError(RTC_ERROR_INVALID_OPERATION, "invalid index buffer format");

      curves.set(buffer, offset, stride, num, format);
      setNumPrimitives(num);
    }
    else if (type == RTC_BUFFER_TYPE_FLAGS)
    {
      if (slot != 0)
        throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "invalid buffer slot");
      if (format != RTC_FORMAT_UCHAR)
        throw_RTCError(RTC_ERROR_INVALID_OPERATION, "invalid flag buffer format");

      flags.set(buffer, offset, stride, num, format);
    }
    else 
      throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "unknown buffer type");
  }

  void* NativeCurves::getBuffer(RTCBufferType type, unsigned int slot)
  {
    if (type == RTC_BUFFER_TYPE_INDEX)
    {
      if (slot != 0)
        throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "invalid buffer slot");
      return curves.getPtr();
    }
    else if (type == RTC_BUFFER_TYPE_VERTEX)
    {
      if (slot >= vertices.size())
        throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "invalid buffer slot");
      return vertices[slot].getPtr();
    }
    else if (type == RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE)
    {
      if (slot >= vertexAttribs.size())
        throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "invalid buffer slot");
      return vertexAttribs[slot].getPtr();
    }
    else if (type == RTC_BUFFER_TYPE_FLAGS)
    {
      if (slot != 0)
        throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "invalid buffer slot");
      return flags.getPtr();
    }
    else
    {
      throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "unknown buffer type");
      return nullptr;
    }
  }

  void NativeCurves::updateBuffer(RTCBufferType type, unsigned int slot)
  {
    if (type == RTC_BUFFER_TYPE_INDEX)
    {
      if (slot != 0)
        throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "invalid buffer slot");
      curves.setModified(true);
    }
    else if (type == RTC_BUFFER_TYPE_VERTEX)
    {
      if (slot >= vertices.size())
        throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "invalid buffer slot");
      vertices[slot].setModified(true);
    }
    else if (type == RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE)
    {
      if (slot >= vertexAttribs.size())
        throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "invalid buffer slot");
      vertexAttribs[slot].setModified(true);
    }
    else if (type == RTC_BUFFER_TYPE_FLAGS) 
    {
      if (slot != 0)
        throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "invalid buffer slot");
      flags.setModified(true);
    }
    else
    {
      throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "unknown buffer type");
    }

    Geometry::update();
  }

  void NativeCurves::setTessellationRate(float N)
  {
    tessellationRate = clamp((int)N,1,16);
  }

  bool NativeCurves::verify () 
  {
    /*! verify consistent size of vertex arrays */
    if (vertices.size() == 0) return false;
    for (const auto& buffer : vertices)
      if (vertices[0].size() != buffer.size())
        return false;

    /*! verify indices */
    for (unsigned int i=0; i<numPrimitives; i++) {
      if (curves[i]+3 >= numVertices()) return false;
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

  void NativeCurves::preCommit()
  {
    /* verify that stride of all time steps are identical */
    for (unsigned int t=0; t<numTimeSteps; t++)
      if (vertices[t].getStride() != vertices[0].getStride())
        throw_RTCError(RTC_ERROR_INVALID_OPERATION,"stride of vertex buffers have to be identical for each time step");

    native_curves = (BufferView<unsigned>) curves;
    if (native_vertices.size() != vertices.size())
      native_vertices.resize(vertices.size());

    native_vertices0 = vertices[0];
    for (size_t i=0; i<vertices.size(); i++)
      native_vertices[i] = (BufferView<Vec3fa>) vertices[i];
  }

  void NativeCurves::postCommit() 
  {
    curves.setModified(false);
    for (auto& buf : vertices)
      buf.setModified(false);
    for (auto& attrib : vertexAttribs)
      attrib.setModified(false);
    flags.setModified(false);

    Geometry::postCommit();
  }

#endif

  namespace isa
  {
    template<typename Curve>
    __forceinline void NativeCurvesISA::interpolate_helper(const RTCInterpolateArguments* const args)
    {
      unsigned int primID = args->primID;
      float u = args->u;
      RTCBufferType bufferType = args->bufferType;
      unsigned int bufferSlot = args->bufferSlot;
      float* P = args->P;
      float* dPdu = args->dPdu;
      float* ddPdudu = args->ddPdudu;
      unsigned int valueCount = args->valueCount;
    
      /* calculate base pointer and stride */
      assert((bufferType == RTC_BUFFER_TYPE_VERTEX && bufferSlot < numTimeSteps) ||
             (bufferType == RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE && bufferSlot <= vertexAttribs.size()));
      const char* src = nullptr; 
      size_t stride = 0;
      if (bufferType == RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE) {
        src    = vertexAttribs[bufferSlot].getPtr();
        stride = vertexAttribs[bufferSlot].getStride();
      } else {
        src    = vertices[bufferSlot].getPtr();
        stride = vertices[bufferSlot].getStride();
      }
      
      for (unsigned int i=0; i<valueCount; i+=4)
      {
        size_t ofs = i*sizeof(float);
        const size_t curve = curves[primID];
        const vbool4 valid = vint4((int)i)+vint4(step) < vint4((int)valueCount);
        const vfloat4 p0 = vfloat4::loadu(valid,(float*)&src[(curve+0)*stride+ofs]);
        const vfloat4 p1 = vfloat4::loadu(valid,(float*)&src[(curve+1)*stride+ofs]);
        const vfloat4 p2 = vfloat4::loadu(valid,(float*)&src[(curve+2)*stride+ofs]);
        const vfloat4 p3 = vfloat4::loadu(valid,(float*)&src[(curve+3)*stride+ofs]);
        
        const Curve bezier(p0,p1,p2,p3);
        if (P      ) vfloat4::storeu(valid,P+i,      bezier.eval(u));
        if (dPdu   ) vfloat4::storeu(valid,dPdu+i,   bezier.eval_du(u));
        if (ddPdudu) vfloat4::storeu(valid,ddPdudu+i,bezier.eval_dudu(u));
      }
    }
    
    template<typename InputCurve3fa, typename OutputCurve3fa>
    void NativeCurvesISA::commit_helper()
    {
      if (native_curves.size() != size()) 
      {
        native_curves.set(new Buffer(device, size()*sizeof(unsigned int)), 0, sizeof(unsigned int), size(), RTC_FORMAT_UINT);
        parallel_for(size_t(0), size(), size_t(1024), [&] ( const range<size_t> r) {
            for (size_t i=r.begin(); i<r.end(); i++) {
              if (curves[i]+3 >= numVertices()) native_curves[i] = 0xFFFFFFF0; // invalid curves stay invalid this way
              else                              native_curves[i] = unsigned(4*i);
            }
          });
      }
      
      if (native_vertices.size() != vertices.size())
        native_vertices.resize(vertices.size());
      
      parallel_for(vertices.size(), [&] (const size_t i) {
          
          if (native_vertices[i].size() != 4*size())
            native_vertices[i].set(new Buffer(device, 4*size()*sizeof(Vec3fa)), 0, sizeof(Vec3fa), 4*size(), RTC_FORMAT_FLOAT4);
          
          parallel_for(size_t(0), size(), size_t(1024), [&] ( const range<size_t> rj ) {
              
              for (size_t j=rj.begin(); j<rj.end(); j++)
              {
                const unsigned id = curves[j];
                if (id+3 >= numVertices()) continue; // ignore invalid curves
                const Vec3fa v0 = vertices[i][id+0];
                const Vec3fa v1 = vertices[i][id+1];
                const Vec3fa v2 = vertices[i][id+2];
                const Vec3fa v3 = vertices[i][id+3];
                const InputCurve3fa icurve(v0,v1,v2,v3);
                OutputCurve3fa ocurve; convert<Vec3fa>(icurve,ocurve);
                native_vertices[i].store(4*j+0,ocurve.v0);
                native_vertices[i].store(4*j+1,ocurve.v1);
                native_vertices[i].store(4*j+2,ocurve.v2);
                native_vertices[i].store(4*j+3,ocurve.v3);
              }
            });
        });
      native_vertices0 = native_vertices[0];
    }
    
    NativeCurves* createCurvesBezier(Device* device, CurveSubtype subtype) {
      return new CurvesBezier(device,BEZIER_CURVE,subtype);
    }
    
    void CurvesBezier::preCommit() {
#if defined(EMBREE_NATIVE_CURVE_BSPLINE)
      if (isEnabled()) commit_helper<BezierCurve3fa,BSplineCurve3fa>();
#else
      NativeCurves::preCommit();
#endif
      Geometry::preCommit();
    }
    
    void CurvesBezier::interpolate(const RTCInterpolateArguments* const args) {
      interpolate_helper<BezierCurveT<vfloat4>>(args);
    }
    
    NativeCurves* createCurvesBSpline(Device* device, CurveSubtype subtype) {
      return new CurvesBSpline(device,BSPLINE_CURVE,subtype);
    }
    
    void CurvesBSpline::preCommit() {
#if defined(EMBREE_NATIVE_CURVE_BSPLINE)
      NativeCurves::preCommit();
#else
      if (isEnabled()) commit_helper<BSplineCurve3fa,BezierCurve3fa>();
#endif
      Geometry::preCommit();
    }
    
    void CurvesBSpline::interpolate(const RTCInterpolateArguments* const args) {
      interpolate_helper<BSplineCurveT<vfloat4>>(args);
    }
  }
}
