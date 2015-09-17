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

#pragma once

#include "default.h"

namespace embree
{
  class Scene;

  /*! Base class all geometries are derived from */
  class Geometry
  {
    friend class Scene;
  public:

    /*! type of geometry */
    enum Type { TRIANGLE_MESH = 1, USER_GEOMETRY = 2, BEZIER_CURVES = 4, SUBDIV_MESH = 8 };

  public:
    
    /*! Geometry constructor */
    Geometry (Scene* scene, Type type, size_t numPrimitives, size_t numTimeSteps, RTCGeometryFlags flags);

    /*! Geometry destructor */
    virtual ~Geometry();

    /*! writes geometry to disk */
    virtual void write(std::ofstream& file);

    /*! updates intersection filter function counts in scene */
    void updateIntersectionFilters(bool enable);

  public:

    /*! tests if geometry is enabled */
    __forceinline bool isEnabled() const { return numPrimitives && enabled; }

    /*! tests if geometry is disabled */
    __forceinline bool isDisabled() const { return !isEnabled(); }

    /*! tests if geometry is modified */
    __forceinline bool isModified() const { return numPrimitives && modified; }

    /*! clears modified flag */
    __forceinline void clearModified() { modified = false; }

    /*! tests if geometry is tagged for deletion */
    __forceinline bool isErasing() const { return erasing; }

    /*! test if this is a static geometry */
    __forceinline bool isStatic() const { return flags == RTC_GEOMETRY_STATIC; }

    /*! test if this is a deformable geometry */
    __forceinline bool isDeformable() const { return flags == RTC_GEOMETRY_DEFORMABLE; }

    /*! test if this is a dynamic geometry */
    __forceinline bool isDynamic() const { return flags == RTC_GEOMETRY_DYNAMIC; }

    /*! returns geometry type */
    __forceinline Type getType() const { return type; }

    /*! returns number of primitives */
    __forceinline size_t size() const { return numPrimitives; }

    /*! for all geometries */
  public:

    /*! Enable geometry. */
    virtual void enable ();

    /*! Update geometry. */
    virtual void update ();

    /*! Update geometry buffer. */
    virtual void updateBuffer (RTCBufferType type) {
      update(); // update everything for geometries not supporting this call
    }
    
    /*! Disable geometry. */
    virtual void disable ();

    /*! Deletes the geometry again. */
    virtual void erase ();

    /*! Free buffers that are unused */
    virtual void immutable () {}

    /*! Verify the geometry */
    virtual bool verify () { return true; }

    /*! called if geometry is switching from disabled to enabled state */
    virtual void enabling() = 0;

    /*! called if geometry is switching from enabled to disabled state */
    virtual void disabling() = 0;

    /*! Set user data pointer. */
    virtual void setUserData (void* ptr);
      
    /*! Get user data pointer. */
    __forceinline void* getUserData() const {
      return userPtr;
    }

    /*! interpolates user data to the specified u/v location */
    virtual void interpolate(unsigned primID, float u, float v, RTCBufferType buffer, float* P, float* dPdu, float* dPdv, size_t numFloats) {
      throw_RTCError(RTC_INVALID_OPERATION,"operation not supported for this geometry"); 
    }

    /*! interpolates user data to the specified u/v locations */
    virtual void interpolateN(const void* valid_i, const unsigned* primIDs, const float* u, const float* v, size_t numUVs, 
                              RTCBufferType buffer, float* P, float* dPdu, float* dPdv, size_t numFloats);

    /*! for subdivision surfaces only */
  public:
    virtual void setBoundaryMode (RTCBoundaryMode mode) {
      throw_RTCError(RTC_INVALID_OPERATION,"operation not supported for this geometry"); 
    }

    /*! for triangle meshes and bezier curves only */
  public:

    /*! Sets ray mask. */
    virtual void setMask (unsigned mask) { 
      throw_RTCError(RTC_INVALID_OPERATION,"operation not supported for this geometry"); 
    }

    /*! Maps specified buffer. */
    virtual void* map(RTCBufferType type) { 
      throw_RTCError(RTC_INVALID_OPERATION,"operation not supported for this geometry"); 
      return nullptr; 
    }

    /*! Unmap specified buffer. */
    virtual void unmap(RTCBufferType type) { 
      throw_RTCError(RTC_INVALID_OPERATION,"operation not supported for this geometry"); 
    }

    /*! Sets specified buffer. */
    virtual void setBuffer(RTCBufferType type, void* ptr, size_t offset, size_t stride) { 
      throw_RTCError(RTC_INVALID_OPERATION,"operation not supported for this geometry"); 
    }

    /*! Set displacement function. */
    virtual void setDisplacementFunction (RTCDisplacementFunc filter, RTCBounds* bounds) {
      throw_RTCError(RTC_INVALID_OPERATION,"operation not supported for this geometry"); 
    }

    /*! Set intersection filter function for single rays. */
    virtual void setIntersectionFilterFunction (RTCFilterFunc filter, bool ispc = false);
    
    /*! Set intersection filter function for ray packets of size 4. */
    virtual void setIntersectionFilterFunction4 (RTCFilterFunc4 filter4, bool ispc = false);
    
    /*! Set intersection filter function for ray packets of size 8. */
    virtual void setIntersectionFilterFunction8 (RTCFilterFunc8 filter8, bool ispc = false);
    
    /*! Set intersection filter function for ray packets of size 16. */
    virtual void setIntersectionFilterFunction16 (RTCFilterFunc16 filter16, bool ispc = false);

    /*! Set occlusion filter function for single rays. */
    virtual void setOcclusionFilterFunction (RTCFilterFunc filter, bool ispc = false);
    
    /*! Set occlusion filter function for ray packets of size 4. */
    virtual void setOcclusionFilterFunction4 (RTCFilterFunc4 filter4, bool ispc = false);
    
    /*! Set occlusion filter function for ray packets of size 8. */
    virtual void setOcclusionFilterFunction8 (RTCFilterFunc8 filter8, bool ispc = false);
    
    /*! Set occlusion filter function for ray packets of size 16. */
    virtual void setOcclusionFilterFunction16 (RTCFilterFunc16 filter16, bool ispc = false);

    /*! for instances only */
  public:
    
    /*! Sets transformation of the instance */
    virtual void setTransform(const AffineSpace3fa& transform) {
      throw_RTCError(RTC_INVALID_OPERATION,"operation not supported for this geometry"); 
    }

    /*! for user geometries only */
  public:

    /*! Set bounds function. */
    virtual void setBoundsFunction (RTCBoundsFunc bounds) { 
      throw_RTCError(RTC_INVALID_OPERATION,"operation not supported for this geometry"); 
    }
    
    /*! Set intersect function for single rays. */
    virtual void setIntersectFunction (RTCIntersectFunc intersect, bool ispc = false) { 
      throw_RTCError(RTC_INVALID_OPERATION,"operation not supported for this geometry"); 
    }
    
    /*! Set intersect function for ray packets of size 4. */
    virtual void setIntersectFunction4 (RTCIntersectFunc4 intersect4, bool ispc = false) { 
      throw_RTCError(RTC_INVALID_OPERATION,"operation not supported for this geometry"); 
    }
    
    /*! Set intersect function for ray packets of size 8. */
    virtual void setIntersectFunction8 (RTCIntersectFunc8 intersect8, bool ispc = false) { 
      throw_RTCError(RTC_INVALID_OPERATION,"operation not supported for this geometry"); 
    }
    
    /*! Set intersect function for ray packets of size 16. */
    virtual void setIntersectFunction16 (RTCIntersectFunc16 intersect16, bool ispc = false) { 
      throw_RTCError(RTC_INVALID_OPERATION,"operation not supported for this geometry"); 
    }

    /*! Set occlusion function for single rays. */
    virtual void setOccludedFunction (RTCOccludedFunc occluded, bool ispc = false) { 
      throw_RTCError(RTC_INVALID_OPERATION,"operation not supported for this geometry"); 
    }
    
    /*! Set occlusion function for ray packets of size 4. */
    virtual void setOccludedFunction4 (RTCOccludedFunc4 occluded4, bool ispc = false) { 
      throw_RTCError(RTC_INVALID_OPERATION,"operation not supported for this geometry"); 
    }
    
    /*! Set occlusion function for ray packets of size 8. */
    virtual void setOccludedFunction8 (RTCOccludedFunc8 occluded8, bool ispc = false) { 
      throw_RTCError(RTC_INVALID_OPERATION,"operation not supported for this geometry"); 
    }
    
    /*! Set occlusion function for ray packets of size 16. */
    virtual void setOccludedFunction16 (RTCOccludedFunc16 occluded16, bool ispc = false) { 
      throw_RTCError(RTC_INVALID_OPERATION,"operation not supported for this geometry"); 
    }

  public:
    __forceinline bool hasIntersectionFilter1() const { return intersectionFilter1 != nullptr; }
    __forceinline bool hasOcclusionFilter1() const { return occlusionFilter1 != nullptr; }
    template<typename simd> __forceinline bool hasIntersectionFilter() const { return false; } // FIXME: should be deleted!?
    template<typename simd> __forceinline bool hasOcclusionFilter() const { return false; } // FIXME: should be deleted!?

  public:
    Scene* parent;             //!< pointer to scene this mesh belongs to
    unsigned id;               //!< internal geometry ID
    Type type;                 //!< geometry type 
    ssize_t numPrimitives;     //!< number of primitives of this geometry
    unsigned numTimeSteps;     //!< number of time steps (1 or 2)
    RTCGeometryFlags flags;    //!< flags of geometry
    bool enabled;              //!< true if geometry is enabled
    bool modified;             //!< true if geometry is modified
    bool erasing;              //!< true if geometry is tagged for deletion
    void* userPtr;             //!< user pointer
    unsigned mask;             //!< for masking out geometry
    
  public:
    RTCFilterFunc intersectionFilter1;
    RTCFilterFunc occlusionFilter1;

    RTCFilterFunc4 intersectionFilter4;
    RTCFilterFunc4 occlusionFilter4;

    RTCFilterFunc8 intersectionFilter8;
    RTCFilterFunc8 occlusionFilter8;

    RTCFilterFunc16 intersectionFilter16;
    RTCFilterFunc16 occlusionFilter16;

    bool ispcIntersectionFilter4;	
    bool ispcOcclusionFilter4;

    bool ispcIntersectionFilter8;
    bool ispcOcclusionFilter8;

    bool ispcIntersectionFilter16;
    bool ispcOcclusionFilter16;
  };

#if defined(__SSE__)
  template<> __forceinline bool Geometry::hasIntersectionFilter<float4>() const { return intersectionFilter4 != nullptr; }
  template<> __forceinline bool Geometry::hasOcclusionFilter   <float4>() const { return occlusionFilter4    != nullptr; }
#endif

#if defined(__AVX__)
  template<> __forceinline bool Geometry::hasIntersectionFilter<float8>() const { return intersectionFilter8 != nullptr; }
  template<> __forceinline bool Geometry::hasOcclusionFilter   <float8>() const { return occlusionFilter8    != nullptr; }
#endif

#if defined(__MIC__) || defined(__AVX512F__)
  template<> __forceinline bool Geometry::hasIntersectionFilter<float16>() const { return intersectionFilter16 != nullptr; }
  template<> __forceinline bool Geometry::hasOcclusionFilter   <float16>() const { return occlusionFilter16    != nullptr; }
#endif
}
