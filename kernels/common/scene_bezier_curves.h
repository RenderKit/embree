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
    
    /*! writes the bezier curve geometry to disk */
    void write(std::ofstream& file);
    
  public:
    void enabling();
    void disabling();
    void setMask (unsigned mask);
    void setBuffer(RTCBufferType type, void* ptr, size_t offset, size_t stride);
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
    
    /*! returns i'th vertex of j'th timestep */
    __forceinline Vec3fa vertex(size_t i, size_t j = 0) const {
      return vertices[j][i];
    }
    
    /*! returns i'th radius of j'th timestep */
    __forceinline float radius(size_t i, size_t j = 0) const {
      return vertices[j][i].w;
    }
    
    /*! check if the i'th primitive is valid */
    __forceinline bool valid(size_t i, BBox3fa* bbox = nullptr) const 
    {
      const unsigned int index = curve(i);
      if (index+3 >= numVertices()) return false;
      
      for (size_t j=0; j<numTimeSteps; j++) 
      {
        const float r0 = radius(index+0,j);
        const float r1 = radius(index+1,j);
        const float r2 = radius(index+2,j);
        const float r3 = radius(index+3,j);
        if (!isvalid(r0) || !isvalid(r1) || !isvalid(r2) || !isvalid(r3))
          return false;
        if (min(r0,r1,r2,r3) < 0.0f)
          return false;
        
        const Vec3fa v0 = vertex(index+0,j);
        const Vec3fa v1 = vertex(index+1,j);
        const Vec3fa v2 = vertex(index+2,j);
        const Vec3fa v3 = vertex(index+3,j);
        if (!isvalid(v0) || !isvalid(v1) || !isvalid(v2) || !isvalid(v3))
          return false;
      }
      
      if (bbox) *bbox = bounds(i);
      return true;
    }
    
    /*! calculates bounding box of i'th bezier curve */
    __forceinline BBox3fa bounds(size_t i, size_t j = 0) const 
    {
      const unsigned int index = curve(i);
      const float r0 = radius(index+0,j);
      const float r1 = radius(index+1,j);
      const float r2 = radius(index+2,j);
      const float r3 = radius(index+3,j);
      const Vec3fa v0 = vertex(index+0,j);
      const Vec3fa v1 = vertex(index+1,j);
      const Vec3fa v2 = vertex(index+2,j);
      const Vec3fa v3 = vertex(index+3,j);
      const BBox3fa b = merge(BBox3fa(v0),BBox3fa(v1),BBox3fa(v2),BBox3fa(v3));
      return enlarge(b,Vec3fa(max(r0,r1,r2,r3)));
    }
    
    /*! calculates bounding box of i'th bezier curve */
    __forceinline BBox3fa bounds(const AffineSpace3fa& space, size_t i, size_t j = 0) const 
    {
      const unsigned int index = curve(i);
      const float r0 = radius(index+0,j);
      const float r1 = radius(index+1,j);
      const float r2 = radius(index+2,j);
      const float r3 = radius(index+3,j);
      const Vec3fa v0 = xfmPoint(space,vertex(index+0,j));
      const Vec3fa v1 = xfmPoint(space,vertex(index+1,j));
      const Vec3fa v2 = xfmPoint(space,vertex(index+2,j));
      const Vec3fa v3 = xfmPoint(space,vertex(index+3,j));
      const BBox3fa b = merge(BBox3fa(v0),BBox3fa(v1),BBox3fa(v2),BBox3fa(v3));
      return enlarge(b,Vec3fa(max(r0,r1,r2,r3)));
    }

  public:
    BufferT<unsigned int> curves;                   //!< array of curve indices
    array_t<BufferT<Vec3fa>,2> vertices;            //!< vertex array
    array_t<std::unique_ptr<Buffer>,2> userbuffers; //!< user buffers
    SubType subtype;                                //!< hair or surface geometry
    int tessellationRate;                           //!< tessellation rate for bezier curve
  };
}
