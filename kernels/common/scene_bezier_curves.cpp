// ======================================================================== //
// Copyright 2009-2018 Intel Corporation                                    //
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

  NativeCurves::NativeCurves (Device* device, GType gtype)
    : Geometry(device,gtype,0,1), tessellationRate(4)
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
    if ((getType() & GTY_SUBTYPE_MASK) == GTY_SUBTYPE_ORIENTED_CURVE)
      normals.resize(numTimeSteps);
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
    else if (type == RTC_BUFFER_TYPE_NORMAL)
    {
      if ((getType() & GTY_SUBTYPE_MASK) != GTY_SUBTYPE_ORIENTED_CURVE)
        throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "unknown buffer type");
        
      if (format != RTC_FORMAT_FLOAT3)
        throw_RTCError(RTC_ERROR_INVALID_OPERATION, "invalid normal buffer format");

      if (slot >= normals.size())
        throw_RTCError(RTC_ERROR_INVALID_OPERATION, "invalid normal buffer slot");
      
      normals[slot].set(buffer, offset, stride, num, format);
      normals[slot].checkPadding16();
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
    else if (type == RTC_BUFFER_TYPE_NORMAL)
    {
      if (slot >= normals.size())
        throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "invalid buffer slot");
      normals[slot].setModified(true);
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
    if (vertices.size() == 0)
      return false;
    
    for (const auto& buffer : vertices)
      if (vertices[0].size() != buffer.size())
        return false;

    for (const auto& buffer : normals)
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

    vertices0 = vertices[0];
    if ((getType() & GTY_SUBTYPE_MASK) == GTY_SUBTYPE_ORIENTED_CURVE)
      normals0 = normals[0];
  }

  void NativeCurves::postCommit() 
  {
    curves.setModified(false);
    for (auto& buf : vertices) buf.setModified(false);
    for (auto& buf : normals)  buf.setModified(false);
    for (auto& attrib : vertexAttribs) attrib.setModified(false);
    flags.setModified(false);

    Geometry::postCommit();
  }

#endif

  namespace isa
  {
    template<typename Curve3fa, typename Curve4f>
      struct NativeCurvesISA : public NativeCurves
    {
      NativeCurvesISA (Device* device, Geometry::GType gtype)
        : NativeCurves(device,gtype) {}

      /*! returns the i'th curve */
      __forceinline const Curve3fa getCurve(size_t i, size_t itime = 0) const 
      {
        const unsigned int index = curve(i);
        const Vec3fa v0 = vertex(index+0,itime);
        const Vec3fa v1 = vertex(index+1,itime);
        const Vec3fa v2 = vertex(index+2,itime);
        const Vec3fa v3 = vertex(index+3,itime);
        return Curve3fa (v0,v1,v2,v3);
      }

      /*! calculates bounding box of i'th bezier curve */
      __forceinline BBox3fa bounds(size_t i, size_t itime = 0) const
      {
        const Curve3fa curve = getCurve(i,itime);
        if (likely(!(getType() & GTY_ROUND_CURVE)))
          return curve.tessellatedBounds(tessellationRate);
        else
          return curve.accurateBounds();
      }
      
      /*! calculates bounding box of i'th bezier curve */
      __forceinline BBox3fa bounds(const AffineSpace3fa& space, size_t i, size_t itime = 0) const
      {
        const unsigned int index = curve(i);
        const Vec3fa v0 = vertex(index+0,itime);
        const Vec3fa v1 = vertex(index+1,itime);
        const Vec3fa v2 = vertex(index+2,itime);
        const Vec3fa v3 = vertex(index+3,itime);
        Vec3fa w0 = xfmPoint(space,v0); w0.w = v0.w;
        Vec3fa w1 = xfmPoint(space,v1); w1.w = v1.w;
        Vec3fa w2 = xfmPoint(space,v2); w2.w = v2.w;
        Vec3fa w3 = xfmPoint(space,v3); w3.w = v3.w;
        const Curve3fa curve(w0,w1,w2,w3);
        if (likely(!(getType() & GTY_ROUND_CURVE))) return curve.tessellatedBounds(tessellationRate);
        else                                        return curve.accurateBounds();
      }
      
      /*! calculates bounding box of i'th bezier curve */
      __forceinline BBox3fa bounds(const Vec3fa& ofs, const float scale, const float r_scale0, const LinearSpace3fa& space, size_t i, size_t itime = 0) const
      {
        const float r_scale = r_scale0*scale;
        const unsigned int index = curve(i);
        const Vec3fa v0 = vertex(index+0,itime);
        const Vec3fa v1 = vertex(index+1,itime);
        const Vec3fa v2 = vertex(index+2,itime);
        const Vec3fa v3 = vertex(index+3,itime);
        Vec3fa w0 = xfmPoint(space,(v0-ofs)*Vec3fa(scale)); w0.w = v0.w*r_scale;
        Vec3fa w1 = xfmPoint(space,(v1-ofs)*Vec3fa(scale)); w1.w = v1.w*r_scale;
        Vec3fa w2 = xfmPoint(space,(v2-ofs)*Vec3fa(scale)); w2.w = v2.w*r_scale;
        Vec3fa w3 = xfmPoint(space,(v3-ofs)*Vec3fa(scale)); w3.w = v3.w*r_scale;
        const Curve3fa curve(w0,w1,w2,w3);
        if (likely(!(getType() & GTY_ROUND_CURVE))) return curve.tessellatedBounds(tessellationRate);
        else                                        return curve.accurateBounds();
      }

      /*! calculates the linear bounds of the i'th primitive at the itimeGlobal'th time segment */
      __forceinline LBBox3fa linearBounds(size_t i, size_t itime) const {
        return LBBox3fa(bounds(i,itime+0),bounds(i,itime+1));
      }
      
      /*! calculates the linear bounds of the i'th primitive at the itimeGlobal'th time segment */
      __forceinline LBBox3fa linearBounds(const AffineSpace3fa& space, size_t i, size_t itime) const {
        return LBBox3fa(bounds(space,i,itime+0),bounds(space,i,itime+1));
      }
      
      /*! calculates the linear bounds of the i'th primitive for the specified time range */
      __forceinline LBBox3fa linearBounds(size_t primID, const BBox1f& time_range) const {
        return LBBox3fa([&] (size_t itime) { return bounds(primID, itime); }, time_range, fnumTimeSegments);
      }
      
      /*! calculates the linear bounds of the i'th primitive for the specified time range */
      __forceinline LBBox3fa linearBounds(const AffineSpace3fa& space, size_t primID, const BBox1f& time_range) const {
        return LBBox3fa([&] (size_t itime) { return bounds(space, primID, itime); }, time_range, fnumTimeSegments);
      }
      
      /*! calculates the linear bounds of the i'th primitive for the specified time range */
      __forceinline LBBox3fa linearBounds(const Vec3fa& ofs, const float scale, const float r_scale0, const LinearSpace3fa& space, size_t primID, const BBox1f& time_range) const {
        return LBBox3fa([&] (size_t itime) { return bounds(ofs, scale, r_scale0, space, primID, itime); }, time_range, fnumTimeSegments);
      }
      
      /*! calculates the build bounds of the i'th primitive, if it's valid */
      __forceinline bool buildBounds(size_t i, BBox3fa* bbox = nullptr) const
      {
        const unsigned int index = curve(i);
        if (index+3 >= numVertices()) return false;
        
        for (size_t t=0; t<numTimeSteps; t++)
        {
          const float r0 = radius(index+0,t);
          const float r1 = radius(index+1,t);
          const float r2 = radius(index+2,t);
          const float r3 = radius(index+3,t);
          if (!isvalid(r0) || !isvalid(r1) || !isvalid(r2) || !isvalid(r3))
            return false;
          //if (min(r0,r1,r2,r3) < 0.0f)
          //  return false;
          
          const Vec3fa v0 = vertex(index+0,t);
          const Vec3fa v1 = vertex(index+1,t);
          const Vec3fa v2 = vertex(index+2,t);
          const Vec3fa v3 = vertex(index+3,t);
          if (!isvalid(v0) || !isvalid(v1) || !isvalid(v2) || !isvalid(v3))
            return false;
        }
        
        if (bbox) *bbox = bounds(i);
        return true;
      }
      
      /*! calculates the i'th build primitive at the itime'th time segment, if it's valid */
      __forceinline bool buildPrim(size_t i, size_t itime, Vec3fa& c0, Vec3fa& c1, Vec3fa& c2, Vec3fa& c3) const
      {
        const unsigned int index = curve(i);
        if (index+3 >= numVertices()) return false;
        const Vec3fa a0 = vertex(index+0,itime+0); if (unlikely(!isvalid((vfloat4)a0))) return false;
        const Vec3fa a1 = vertex(index+1,itime+0); if (unlikely(!isvalid((vfloat4)a1))) return false;
        const Vec3fa a2 = vertex(index+2,itime+0); if (unlikely(!isvalid((vfloat4)a2))) return false;
        const Vec3fa a3 = vertex(index+3,itime+0); if (unlikely(!isvalid((vfloat4)a3))) return false;
        const Vec3fa b0 = vertex(index+0,itime+1); if (unlikely(!isvalid((vfloat4)b0))) return false;
        const Vec3fa b1 = vertex(index+1,itime+1); if (unlikely(!isvalid((vfloat4)b1))) return false;
        const Vec3fa b2 = vertex(index+2,itime+1); if (unlikely(!isvalid((vfloat4)b2))) return false;
        const Vec3fa b3 = vertex(index+3,itime+1); if (unlikely(!isvalid((vfloat4)b3))) return false;
        if (unlikely(min(a0.w,a1.w,a2.w,a3.w) < 0.0f)) return false;
        if (unlikely(min(b0.w,b1.w,b2.w,b3.w) < 0.0f)) return false;
        c0 = 0.5f*(a0+b0);
        c1 = 0.5f*(a1+b1);
        c2 = 0.5f*(a2+b2);
        c3 = 0.5f*(a3+b3);
        return true;
      }

      /*! check if the i'th primitive is valid at the itime'th timestep */
      __forceinline bool valid(size_t i, size_t itime) const {
        return valid(i, make_range(itime, itime));
      }
      
      /*! check if the i'th primitive is valid at the itime'th time step */
      __forceinline bool valid(size_t i, const range<size_t>& itime_range) const
      {
        const unsigned int index = curve(i);
        if (index+3 >= numVertices()) return false;
        
        for (size_t itime = itime_range.begin(); itime <= itime_range.end(); itime++)
        {
          const float r0 = radius(index+0,itime);
          const float r1 = radius(index+1,itime);
          const float r2 = radius(index+2,itime);
          const float r3 = radius(index+3,itime);
          if (!isvalid(r0) || !isvalid(r1) || !isvalid(r2) || !isvalid(r3))
            return false;
          if (min(r0,r1,r2,r3) < 0.0f)
            return false;
          
          const Vec3fa v0 = vertex(index+0,itime);
          const Vec3fa v1 = vertex(index+1,itime);
          const Vec3fa v2 = vertex(index+2,itime);
          const Vec3fa v3 = vertex(index+3,itime);
          if (!isvalid(v0) || !isvalid(v1) || !isvalid(v2) || !isvalid(v3))
            return false;
        }
        
        return true;
      }
      
      /*! calculates the linear bounds of the i'th primitive for the specified time range */
      __forceinline bool linearBounds(size_t i, const BBox1f& time_range, LBBox3fa& bbox) const
      {
        if (!valid(i, getTimeSegmentRange(time_range, fnumTimeSegments))) return false;
        bbox = linearBounds(i, time_range);
        return true;
      }
      
      PrimInfo createPrimRefArray(mvector<PrimRef>& prims, const range<size_t>& r, size_t k) const
      {
        PrimInfo pinfo(empty);
        for (size_t j=r.begin(); j<r.end(); j++)
        {
          BBox3fa bounds = empty;
          if (!buildBounds(j,&bounds)) continue;
          const PrimRef prim(bounds,geomID,unsigned(j));
          pinfo.add_center2(prim);
          prims[k++] = prim;
        }
        return pinfo;
      }

      PrimInfoMB createPrimRefMBArray(mvector<PrimRefMB>& prims, const BBox1f& t0t1, const range<size_t>& r, size_t k) const
      {
        PrimInfoMB pinfo(empty);
        for (size_t j=r.begin(); j<r.end(); j++)
        {
          LBBox3fa bounds = empty;
          if (!this->linearBounds(j,t0t1,bounds)) continue;
          const PrimRefMB prim(bounds,this->numTimeSegments(),this->numTimeSegments(),this->geomID,unsigned(j));
          pinfo.add_primref(prim);
          prims[k++] = prim;
        }
        return pinfo;
      }

      LinearSpace3fa computeAlignedSpace(const size_t primID) const
      {
        Vec3fa axisz(0,0,1);
        Vec3fa axisy(0,1,0);
        
        const unsigned vtxID = this->curve(primID);
        const Vec3fa v0 = this->vertex(vtxID+0);
        const Vec3fa v1 = this->vertex(vtxID+1);
        const Vec3fa v2 = this->vertex(vtxID+2);
        const Vec3fa v3 = this->vertex(vtxID+3);
        const Curve3fa curve(v0,v1,v2,v3);
        const Vec3fa p0 = curve.begin();
        const Vec3fa p3 = curve.end();
        const Vec3fa d0 = curve.eval_du(0.0f);
        //const Vec3fa d1 = curve.eval_du(1.0f);
        const Vec3fa axisz_ = normalize(p3 - p0);
        const Vec3fa axisy_ = cross(axisz_,d0);
        if (sqr_length(p3-p0) > 1E-18f) {
          axisz = axisz_;
          axisy = axisy_;
        }
        
        if (sqr_length(axisy) > 1E-18) {
          axisy = normalize(axisy);
          Vec3fa axisx = normalize(cross(axisy,axisz));
          return LinearSpace3fa(axisx,axisy,axisz);
        }
        return frame(axisz);
      }

      LinearSpace3fa computeAlignedSpaceMB(const size_t primID, const BBox1f time_range) const
      {
        Vec3fa axisz(0,0,1);
        Vec3fa axisy(0,1,0);

        const unsigned num_time_segments = this->numTimeSegments();
        const range<int> tbounds = getTimeSegmentRange(time_range, (float)num_time_segments);
        if (tbounds.size() == 0) return frame(axisz);
        
        const size_t t = (tbounds.begin()+tbounds.end())/2;
        const unsigned int vertexID = this->curve(primID);
        const Vec3fa a0 = this->vertex(vertexID+0,t);
        const Vec3fa a1 = this->vertex(vertexID+1,t);
        const Vec3fa a2 = this->vertex(vertexID+2,t);
        const Vec3fa a3 = this->vertex(vertexID+3,t);
        const Curve3fa curve(a0,a1,a2,a3);
        const Vec3fa p0 = curve.begin();
        const Vec3fa p3 = curve.end();
        const Vec3fa d0 = curve.eval_du(0.0f);
        //const Vec3fa d1 = curve.eval_du(1.0f);
        const Vec3fa axisz_ = normalize(p3 - p0);
        const Vec3fa axisy_ = cross(axisz_,d0);
        if (sqr_length(p3-p0) > 1E-18f) {
          axisz = axisz_;
          axisy = axisy_;
        }
        
        if (sqr_length(axisy) > 1E-18) {
          axisy = normalize(axisy);
          Vec3fa axisx = normalize(cross(axisy,axisz));
          return LinearSpace3fa(axisx,axisy,axisz);
        }
        return frame(axisz);
      }
      
      Vec3fa computeDirection(unsigned int primID) const
      {
        const unsigned vtxID = curve(primID);
        const Vec3fa v0 = vertex(vtxID+0);
        const Vec3fa v1 = vertex(vtxID+1);
        const Vec3fa v2 = vertex(vtxID+2);
        const Vec3fa v3 = vertex(vtxID+3);
        const Curve3fa c(v0,v1,v2,v3);
        const Vec3fa p0 = c.begin();
        const Vec3fa p3 = c.end();
        const Vec3fa axis1 = p3 - p0;
        return axis1;
      }

      Vec3fa computeDirection(unsigned int primID, size_t time) const
      {
        const unsigned vtxID = curve(primID);
        const Vec3fa v0 = vertex(vtxID+0,time);
        const Vec3fa v1 = vertex(vtxID+1,time);
        const Vec3fa v2 = vertex(vtxID+2,time);
        const Vec3fa v3 = vertex(vtxID+3,time);
        const Curve3fa c(v0,v1,v2,v3);
        const Vec3fa p0 = c.begin();
        const Vec3fa p3 = c.end();
        const Vec3fa axis1 = p3 - p0;
        return axis1;
      }

      BBox3fa vbounds(size_t i) const {
        return bounds(i);
      }
      
      BBox3fa vbounds(const AffineSpace3fa& space, size_t i) const {
        return bounds(space,i);
      }

      BBox3fa vbounds(const Vec3fa& ofs, const float scale, const float r_scale0, const LinearSpace3fa& space, size_t i, size_t itime = 0) const {
        return bounds(ofs,scale,r_scale0,space,i,itime);
      }

      LBBox3fa vlinearBounds(size_t primID, const BBox1f& time_range) const {
        return linearBounds(primID,time_range);
      }
      
      LBBox3fa vlinearBounds(const AffineSpace3fa& space, size_t primID, const BBox1f& time_range) const {
        return linearBounds(space,primID,time_range);
      }

      LBBox3fa vlinearBounds(const Vec3fa& ofs, const float scale, const float r_scale0, const LinearSpace3fa& space, size_t primID, const BBox1f& time_range) const {
        return linearBounds(ofs,scale,r_scale0,space,primID,time_range);
      }
      
      void interpolate(const RTCInterpolateArguments* const args)
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
          
          const Curve4f bezier(p0,p1,p2,p3);
          if (P      ) vfloat4::storeu(valid,P+i,      bezier.eval(u));
          if (dPdu   ) vfloat4::storeu(valid,dPdu+i,   bezier.eval_du(u));
          if (ddPdudu) vfloat4::storeu(valid,ddPdudu+i,bezier.eval_dudu(u));
        }
      }
    };
    
    NativeCurves* createCurves(Device* device, Geometry::GType gtype)
    {
      switch (gtype) {
      case Geometry::GTY_ROUND_BEZIER_CURVE: return new NativeCurvesISA<BezierCurve3fa,BezierCurveT<vfloat4>>(device,gtype);
      case Geometry::GTY_FLAT_BEZIER_CURVE : return new NativeCurvesISA<BezierCurve3fa,BezierCurveT<vfloat4>>(device,gtype);
      case Geometry::GTY_ORIENTED_BEZIER_CURVE : return new NativeCurvesISA<BezierCurve3fa,BezierCurveT<vfloat4>>(device,gtype);
      case Geometry::GTY_ROUND_BSPLINE_CURVE: return new NativeCurvesISA<BSplineCurve3fa,BSplineCurveT<vfloat4>>(device,gtype);
      case Geometry::GTY_FLAT_BSPLINE_CURVE : return new NativeCurvesISA<BSplineCurve3fa,BSplineCurveT<vfloat4>>(device,gtype);
      case Geometry::GTY_ORIENTED_BSPLINE_CURVE : return new NativeCurvesISA<BSplineCurve3fa,BSplineCurveT<vfloat4>>(device,gtype);
      default: throw_RTCError(RTC_ERROR_INVALID_OPERATION,"invalid geometry type");
      }
    }
  }
}
