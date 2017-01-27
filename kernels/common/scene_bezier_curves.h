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

#pragma once

#include "default.h"
#include "geometry.h"
#include "primref.h"
#include "buffer.h"

namespace embree
{
  /*! represents an array of bicubic bezier curves */
  struct BezierCurves : public Geometry
  {
    /*! type of this geometry */
    static const Geometry::Type geom_type = Geometry::BEZIER_CURVES;

    /*! this geometry represents approximate hair geometry and real bezier surface geometry */
    enum SubType { HAIR = 1, SURFACE = 0 };

  public:
    
    /*! bezier curve construction */
    BezierCurves (Scene* parent, SubType subtype, RTCGeometryFlags flags, size_t numPrimitives, size_t numVertices, size_t numTimeSteps); 
    
  public:
    void enabling();
    void disabling();
    void setMask (unsigned mask);
    void setBuffer(RTCBufferType type, void* ptr, size_t offset, size_t stride, size_t size);
    void* map(RTCBufferType type);
    void unmap(RTCBufferType type);
    void immutable ();
    bool verify ();
    void interpolate(unsigned primID, float u, float v, RTCBufferType buffer, float* P, float* dPdu, float* dPdv, float* ddPdudu, float* ddPdvdv, float* ddPdudv, size_t numFloats);
    void setTessellationRate(float N);
    // FIXME: implement interpolateN

  public:
    
    /*! returns number of bezier curves */
    __forceinline size_t size() const {
      return curves.size();
    }
    
    /*! returns the number of vertices */
    __forceinline size_t numVertices() const {
      return vertices[0].size();
    }
    
    /*! returns the i'th curve */
    __forceinline const unsigned int& curve(size_t i) const {
      return curves[i];
    }
    
    /*! returns i'th vertex of the first time step */
    __forceinline Vec3fa vertex(size_t i) const {
      return vertices0[i];
    }
    
    /*! returns i'th radius of the first time step */
    __forceinline float radius(size_t i) const {
      return vertices0[i].w;
    }

    /*! returns i'th vertex of itime'th timestep */
    __forceinline Vec3fa vertex(size_t i, size_t itime) const {
      return vertices[itime][i];
    }
    
    /*! returns i'th radius of itime'th timestep */
    __forceinline float radius(size_t i, size_t itime) const {
      return vertices[itime][i].w;
    }

    /*! gathers the curve starting with i'th vertex of itime'th timestep */
    __forceinline void gather(Vec3fa& p0,
                              Vec3fa& p1,
                              Vec3fa& p2,
                              Vec3fa& p3,
                              size_t i,
                              size_t itime = 0) const
    {
      p0 = vertex(i+0,itime);
      p1 = vertex(i+1,itime);
      p2 = vertex(i+2,itime);
      p3 = vertex(i+3,itime);
    }

    __forceinline void gather(Vec3fa& p0,
                              Vec3fa& p1,
                              Vec3fa& p2,
                              Vec3fa& p3,
                              size_t i,
                              float time) const
    {
      float ftime;
      const size_t itime = getTimeSegment(time, fnumTimeSegments, ftime);

      const float t0 = 1.0f - ftime;
      const float t1 = ftime;
      Vec3fa a0,a1,a2,a3;
      gather(a0,a1,a2,a3,i,itime);
      Vec3fa b0,b1,b2,b3;
      gather(b0,b1,b2,b3,i,itime+1);
      p0 = madd(Vec3fa(t0),a0,t1*b0);
      p1 = madd(Vec3fa(t0),a1,t1*b1);
      p2 = madd(Vec3fa(t0),a2,t1*b2);
      p3 = madd(Vec3fa(t0),a3,t1*b3);
    }
    
    /*! calculates bounding box of i'th bezier curve */
    __forceinline BBox3fa bounds(size_t i, size_t itime = 0) const
    {
      const unsigned int index = curve(i);
      const float r0 = radius(index+0,itime);
      const float r1 = radius(index+1,itime);
      const float r2 = radius(index+2,itime);
      const float r3 = radius(index+3,itime);
      const Vec3fa v0 = vertex(index+0,itime);
      const Vec3fa v1 = vertex(index+1,itime);
      const Vec3fa v2 = vertex(index+2,itime);
      const Vec3fa v3 = vertex(index+3,itime);
      const BBox3fa b = merge(BBox3fa(v0),BBox3fa(v1),BBox3fa(v2),BBox3fa(v3));
      return enlarge(b,Vec3fa(max(r0,r1,r2,r3)));
    }
    
    /*! calculates bounding box of i'th bezier curve */
    __forceinline BBox3fa bounds(const AffineSpace3fa& space, size_t i, size_t itime = 0) const
    {
      const unsigned int index = curve(i);
      const float r0 = radius(index+0,itime);
      const float r1 = radius(index+1,itime);
      const float r2 = radius(index+2,itime);
      const float r3 = radius(index+3,itime);
      const Vec3fa v0 = xfmPoint(space,vertex(index+0,itime));
      const Vec3fa v1 = xfmPoint(space,vertex(index+1,itime));
      const Vec3fa v2 = xfmPoint(space,vertex(index+2,itime));
      const Vec3fa v3 = xfmPoint(space,vertex(index+3,itime));
      const BBox3fa b = merge(BBox3fa(v0),BBox3fa(v1),BBox3fa(v2),BBox3fa(v3));
      return enlarge(b,Vec3fa(max(r0,r1,r2,r3)));
    }

    /*! check if the i'th primitive is valid at the itime'th time step */
    __forceinline bool valid(size_t i, size_t itime) const
    {
      const unsigned int index = curve(i);
      if (index+3 >= numVertices()) return false;

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

      return true;
    }

    /*! calculates the linear bounds of the i'th primitive at the itimeGlobal'th time segment */
    __forceinline LBBox3fa linearBounds(size_t i, size_t itimeGlobal, size_t numTimeStepsGlobal) const
    {
      return Geometry::linearBounds([&] (size_t itime) { return bounds(i, itime); },
                                    itimeGlobal, numTimeStepsGlobal, numTimeSteps);
    }

    /*! calculates the linear bounds of the i'th primitive at the itimeGlobal'th time segment */
    __forceinline LBBox3fa linearBounds(const AffineSpace3fa& space, size_t i, size_t itimeGlobal, size_t numTimeStepsGlobal) const
    {
      return Geometry::linearBounds([&] (size_t itime) { return bounds(space, i, itime); },
                                    itimeGlobal, numTimeStepsGlobal, numTimeSteps);
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
        if (min(r0,r1,r2,r3) < 0.0f)
          return false;

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

    /*! calculates the i'th build primitive at the itimeGlobal'th time segment, if it's valid */
    __forceinline bool buildPrim(size_t i, size_t itimeGlobal, size_t numTimeStepsGlobal, Vec3fa& c0, Vec3fa& c1, Vec3fa& c2, Vec3fa& c3) const
    {
      if (!Geometry::validLinearBounds([&] (size_t itime) { return valid(i, itime); },
                                       itimeGlobal, numTimeStepsGlobal, numTimeSteps))
        return false;

      const unsigned int index = curve(i);
      float time = (float(int(itimeGlobal)) + 0.5f) / float(int(numTimeStepsGlobal-1));
      gather(c0,c1,c2,c3,index,time);
      return true;
    }

  public:
    APIBuffer<unsigned int> curves;                   //!< array of curve indices
    BufferRefT<Vec3fa> vertices0;                     //!< fast access to first vertex buffer
    vector<APIBuffer<Vec3fa>> vertices;               //!< vertex array for each timestep
    vector<APIBuffer<char>> userbuffers;            //!< user buffers
    SubType subtype;                                //!< hair or surface geometry
    int tessellationRate;                           //!< tessellation rate for bezier curve
  };
}
