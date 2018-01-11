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

#pragma once

#include "default.h"

namespace embree
{
  class Scene;

  /* calculate time segment itime and fractional time ftime */
  __forceinline int getTimeSegment(float time, float numTimeSegments, float& ftime)
  {
    const float timeScaled = time * numTimeSegments;
    const float itimef = clamp(floor(timeScaled), 0.0f, numTimeSegments-1.0f);
    ftime = timeScaled - itimef;
    return int(itimef);
  }

  template<int N>
  __forceinline vint<N> getTimeSegment(const vfloat<N>& time, const vfloat<N>& numTimeSegments, vfloat<N>& ftime)
  {
    const vfloat<N> timeScaled = time * numTimeSegments;
    const vfloat<N> itimef = clamp(floor(timeScaled), vfloat<N>(zero), numTimeSegments-1.0f);
    ftime = timeScaled - itimef;
    return vint<N>(itimef);
  }

  /* calculate overlapping time segment range */
  __forceinline range<int> getTimeSegmentRange(const BBox1f& time_range, float numTimeSegments)
  {
    const int itime_lower = (int)floor(time_range.lower*numTimeSegments);
    const int itime_upper = (int)ceil (time_range.upper*numTimeSegments);
    return make_range(itime_lower, itime_upper);
  }

  /*! Base class all geometries are derived from */
  class Geometry
  {
    friend class Scene;
  public:

    /*! type of geometry */
    enum Type { TRIANGLE_MESH = 1, QUAD_MESH = 2, BEZIER_CURVES = 4, LINE_SEGMENTS = 8, SUBDIV_MESH = 16, USER_GEOMETRY = 32, INSTANCE = 64, GROUP = 128 };
    static const int NUM_TYPES = 8;

  public:
    
    /*! Geometry constructor */
    Geometry (Scene* scene, Type type, size_t numPrimitives, size_t numTimeSteps, RTCGeometryFlags flags);

    /*! Geometry destructor */
    virtual ~Geometry();

    /*! updates intersection filter function counts in scene */
    void updateIntersectionFilters(bool enable);

  public:

    /*! tests if geometry is enabled */
    __forceinline bool isEnabled() const { return numPrimitives && enabled; }

    /*! tests if geometry is disabled */
    __forceinline bool isDisabled() const { return !isEnabled(); }

    /*! tests if geomery is used by any instance (including world space instance) */
    __forceinline bool isUsed() const { return used; }

     /*! tests if geometry is used by any non-world space instance */
    __forceinline bool isInstanced() const { return used-enabled; }

    /*! tests if geometry is modified */
    __forceinline bool isModified() const { return numPrimitives && modified; }

    /*! clears modified flag */
    __forceinline void clearModified() { modified = false; }

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

    /*! sets the number of primitives */
    __forceinline void setNumPrimitives(size_t numPrimitives_in)
    { 
      if ((ssize_t)numPrimitives_in == -1) return;
      if (numPrimitives_in == numPrimitives) return;
      numPrimitives = numPrimitives_in;
      numPrimitivesChanged = true;
    }

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

    /*! Free buffers that are unused */
    virtual void immutable () {}

    /*! Verify the geometry */
    virtual bool verify () { return true; }

    /*! called if geometry is switching from disabled to enabled state */
    virtual void enabling() = 0;

    /*! called if geometry is switching from enabled to disabled state */
    virtual void disabling() = 0;

    /*! called before every build */
    virtual void preCommit();

    /*! called after every build */
    virtual void postCommit();

    /*! sets constant tessellation rate for the geometry */
    virtual void setTessellationRate(float N) {
      throw_RTCError(RTC_INVALID_OPERATION,"operation not supported for this geometry"); 
    }

    /*! Set user data pointer. */
    virtual void setUserData (void* ptr);
      
    /*! Get user data pointer. */
    __forceinline void* getUserData() const {
      return userPtr;
    }

    /*! interpolates user data to the specified u/v location */
    virtual void interpolate(unsigned primID, float u, float v, RTCBufferType buffer, float* P, float* dPdu, float* dPdv, float* ddPdudu, float* ddPdvdv, float* ddPdudv, size_t numFloats) {
      throw_RTCError(RTC_INVALID_OPERATION,"operation not supported for this geometry"); 
    }

    /*! interpolates user data to the specified u/v locations */
    virtual void interpolateN(const void* valid_i, const unsigned* primIDs, const float* u, const float* v, size_t numUVs, 
                              RTCBufferType buffer, float* P, float* dPdu, float* dPdv, float* ddPdudu, float* ddPdvdv, float* ddPdudv, size_t numFloats);

    /*! for subdivision surfaces only */
  public:
    virtual void setSubdivisionMode (unsigned topologyID, RTCSubdivisionMode mode) {
      throw_RTCError(RTC_INVALID_OPERATION,"operation not supported for this geometry"); 
    }

    virtual void setIndexBuffer(RTCBufferType vertexBuffer, RTCBufferType indexBuffer) {
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
    virtual void setBuffer(RTCBufferType type, void* ptr, size_t offset, size_t stride, size_t size) { 
      throw_RTCError(RTC_INVALID_OPERATION,"operation not supported for this geometry"); 
    }

    /*! Set displacement function. */
    virtual void setDisplacementFunction (RTCDisplacementFunc filter, RTCBounds* bounds) {
      throw_RTCError(RTC_INVALID_OPERATION,"operation not supported for this geometry"); 
    }

    /*! Set displacement function. */
    virtual void setDisplacementFunction2 (RTCDisplacementFunc2 filter, RTCBounds* bounds) {
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

    /*! Set intersection filter function for ray packets of size N. */
    virtual void setIntersectionFilterFunctionN (RTCFilterFuncN filterN);

    /*! Set occlusion filter function for single rays. */
    virtual void setOcclusionFilterFunction (RTCFilterFunc filter, bool ispc = false);
    
    /*! Set occlusion filter function for ray packets of size 4. */
    virtual void setOcclusionFilterFunction4 (RTCFilterFunc4 filter4, bool ispc = false);
    
    /*! Set occlusion filter function for ray packets of size 8. */
    virtual void setOcclusionFilterFunction8 (RTCFilterFunc8 filter8, bool ispc = false);
    
    /*! Set occlusion filter function for ray packets of size 16. */
    virtual void setOcclusionFilterFunction16 (RTCFilterFunc16 filter16, bool ispc = false);

    /*! Set occlusion filter function for ray packets of size N. */
    virtual void setOcclusionFilterFunctionN (RTCFilterFuncN filterN);

    /*! for instances only */
  public:
    
    /*! Sets transformation of the instance */
    virtual void setTransform(const AffineSpace3fa& transform, size_t timeStep) {
      throw_RTCError(RTC_INVALID_OPERATION,"operation not supported for this geometry"); 
    }

    /*! for user geometries only */
  public:

    /*! Set bounds function. */
    virtual void setBoundsFunction (RTCBoundsFunc bounds) { 
      throw_RTCError(RTC_INVALID_OPERATION,"operation not supported for this geometry"); 
    }

    /*! Set bounds function. */
    virtual void setBoundsFunction2 (RTCBoundsFunc2 bounds, void* userPtr) { 
      throw_RTCError(RTC_INVALID_OPERATION,"operation not supported for this geometry"); 
    }

    /*! Set bounds function. */
    virtual void setBoundsFunction3 (RTCBoundsFunc3 bounds, void* userPtr) { 
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

    /*! Set intersect function for streams of single rays. */
    virtual void setIntersectFunction1Mp (RTCIntersectFunc1Mp intersect) { 
      throw_RTCError(RTC_INVALID_OPERATION,"operation not supported for this geometry"); 
    }

    /*! Set intersect function for ray packets of size N. */
    virtual void setIntersectFunctionN (RTCIntersectFuncN intersect) { 
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

    /*! Set occlusion function for streams of single rays. */
    virtual void setOccludedFunction1Mp (RTCOccludedFunc1Mp occluded) { 
      throw_RTCError(RTC_INVALID_OPERATION,"operation not supported for this geometry"); 
    }

    /*! Set occlusion function for ray packets of size N. */
    virtual void setOccludedFunctionN (RTCOccludedFuncN occluded) { 
      throw_RTCError(RTC_INVALID_OPERATION,"operation not supported for this geometry"); 
    }

    /*! returns number of time segments */
    __forceinline unsigned numTimeSegments () const {
      return numTimeSteps-1;
    }

  public:
    __forceinline bool hasIntersectionFilter1() const { return (hasIntersectionFilterMask & (HAS_FILTER1 | HAS_FILTERN)) != 0;  }
    __forceinline bool hasOcclusionFilter1   () const { return (hasOcclusionFilterMask    & (HAS_FILTER1 | HAS_FILTERN)) != 0; }
    template<typename simd> __forceinline bool hasIntersectionFilter() const;
    template<typename simd> __forceinline bool hasOcclusionFilter() const;

    template<typename simd> __forceinline bool hasISPCIntersectionFilter() const;
    template<typename simd> __forceinline bool hasISPCOcclusionFilter() const;

  public:
    Scene* scene;              //!< pointer to scene this mesh belongs to
    unsigned geomID;           //!< internal geometry ID
    Type type;                 //!< geometry type 
    size_t numPrimitives;      //!< number of primitives of this geometry
    bool numPrimitivesChanged; //!< true if number of primitives changed
    unsigned numTimeSteps;     //!< number of time steps
    float fnumTimeSegments;    //!< number of time segments (precalculation)
    RTCGeometryFlags flags;    //!< flags of geometry
    bool enabled;              //!< true if geometry is enabled
    bool modified;             //!< true if geometry is modified
    void* userPtr;             //!< user pointer
    unsigned mask;             //!< for masking out geometry
    std::atomic<size_t> used;  //!< counts by how many enabled instances this geometry is used
    
  public:
    RTCFilterFunc intersectionFilter1;
    RTCFilterFunc occlusionFilter1;

    RTCFilterFunc4 intersectionFilter4;
    RTCFilterFunc4 occlusionFilter4;

    RTCFilterFunc8 intersectionFilter8;
    RTCFilterFunc8 occlusionFilter8;

    RTCFilterFunc16 intersectionFilter16;
    RTCFilterFunc16 occlusionFilter16;

    RTCFilterFuncN intersectionFilterN;
    RTCFilterFuncN occlusionFilterN;

  public: 
    enum { HAS_FILTER1 = 1, HAS_FILTER4 = 2, HAS_FILTER8 = 4, HAS_FILTER16 = 8, HAS_FILTERN = 16 };  
    int hasIntersectionFilterMask;
    int hasOcclusionFilterMask;
    int ispcIntersectionFilterMask;
    int ispcOcclusionFilterMask;
  };

#if defined(__SSE__)
  template<> __forceinline bool Geometry::hasIntersectionFilter<vfloat4>() const { return (hasIntersectionFilterMask & (HAS_FILTER4 | HAS_FILTERN)) != 0; }
  template<> __forceinline bool Geometry::hasOcclusionFilter   <vfloat4>() const { return (hasOcclusionFilterMask    & (HAS_FILTER4 | HAS_FILTERN)) != 0; }
  template<> __forceinline bool Geometry::hasISPCIntersectionFilter<vfloat4>() const { return (ispcIntersectionFilterMask & HAS_FILTER4) != 0; }
  template<> __forceinline bool Geometry::hasISPCOcclusionFilter   <vfloat4>() const { return (ispcOcclusionFilterMask    & HAS_FILTER4) != 0; }
#endif

#if defined(__AVX__)
  template<> __forceinline bool Geometry::hasIntersectionFilter<vfloat8>() const { return (hasIntersectionFilterMask & (HAS_FILTER8 | HAS_FILTERN)) != 0; }
  template<> __forceinline bool Geometry::hasOcclusionFilter   <vfloat8>() const { return (hasOcclusionFilterMask    & (HAS_FILTER8 | HAS_FILTERN)) != 0; }
  template<> __forceinline bool Geometry::hasISPCIntersectionFilter<vfloat8>() const { return (ispcIntersectionFilterMask & HAS_FILTER8) != 0; }
  template<> __forceinline bool Geometry::hasISPCOcclusionFilter   <vfloat8>() const { return (ispcOcclusionFilterMask    & HAS_FILTER8) != 0; }
#endif

#if defined(__AVX512F__)
  template<> __forceinline bool Geometry::hasIntersectionFilter<vfloat16>() const { return (hasIntersectionFilterMask & (HAS_FILTER16 | HAS_FILTERN)) != 0; }
  template<> __forceinline bool Geometry::hasOcclusionFilter   <vfloat16>() const { return (hasOcclusionFilterMask    & (HAS_FILTER16 | HAS_FILTERN)) != 0; }
  template<> __forceinline bool Geometry::hasISPCIntersectionFilter<vfloat16>() const { return (ispcIntersectionFilterMask & HAS_FILTER16) != 0; }
  template<> __forceinline bool Geometry::hasISPCOcclusionFilter   <vfloat16>() const { return (ispcOcclusionFilterMask    & HAS_FILTER16) != 0; }
#endif
}
