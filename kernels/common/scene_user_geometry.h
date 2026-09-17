// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "accelset.h"

namespace embree
{
  /*! User geometry with user defined intersection functions */
  struct UserGeometry : public AccelSet
  {
    /*! type of this geometry */
    static const Geometry::GTypeMask geom_type = Geometry::MTY_USER_GEOMETRY;

  public:
    UserGeometry (Device* device, unsigned int items = 0, unsigned int numTimeSteps = 1, Geometry::GType gtype = Geometry::GTY_USER_GEOMETRY);
    virtual void setMask (unsigned mask) override;
    virtual void setBoundsFunction (RTCBoundsFunction bounds, void* userPtr) override;
    virtual void setOrientedBoundsFunction (RTCOrientedBoundsFunction bounds, void* userPtr) override;
    virtual void setIntersectFunctionN (RTCIntersectFunctionN intersect) override;
    virtual void setOccludedFunctionN (RTCOccludedFunctionN occluded) override;
    virtual void build() override {}
    virtual void addElementsToCount (GeometryCounts & counts) const override;
    virtual size_t getGeometryDataDeviceByteSize() const override;
    virtual void convertToDeviceRepresentation(size_t offset, char* data_host, char* data_device) const override;

    virtual BBox3fa vbounds(size_t primID) const override {
      return bounds(primID);
    }

    virtual BBox3fa vbounds(const LinearSpace3fa& space, size_t primID) const override {
      return xfmBounds(space, bounds(primID));
    }

    virtual LBBox3fa vlinearBounds(size_t primID, const BBox1f& time_range) const override {
      return linearBounds(primID, time_range);
    }

    virtual LBBox3fa vlinearBounds(const LinearSpace3fa& space, size_t primID, const BBox1f& time_range) const override {
      const LBBox3fa lb = linearBounds(primID, time_range);
      return LBBox3fa(xfmBounds(space, lb.bounds0), xfmBounds(space, lb.bounds1));
    }

    __forceinline float projectedPrimitiveArea(const size_t i) const { return 0.0f; }
  };

  struct OrientedUserGeometry : public UserGeometry
  {
    static const Geometry::GTypeMask geom_type = Geometry::MTY_USER_GEOMETRY_ORIENTED;

  public:
    OrientedUserGeometry(Device* device, unsigned int items = 0, unsigned int numTimeSteps = 1)
      : UserGeometry(device, items, numTimeSteps, Geometry::GTY_USER_GEOMETRY_ORIENTED) {}

    virtual void setBoundsFunction (RTCBoundsFunction bounds, void* userPtr) override {
      (void)bounds;
      (void)userPtr;
      throw_RTCError(RTC_ERROR_INVALID_OPERATION,"use rtcSetGeometryOrientedBoundsFunction for oriented user geometry");
    }

    virtual void setOrientedBoundsFunction (RTCOrientedBoundsFunction bounds, void* userPtr) override {
      this->orientedBoundsFunc = bounds;
      this->boundsUserPtr = userPtr;
      Geometry::update();
    }

    __forceinline RTCOrientedBounds orientedBounds(size_t primID, size_t timeStep = 0) const {
      RTCOrientedBounds bounds = {};
      RTCOrientedBoundsFunctionArguments args;
      args.geometryUserPtr = userPtr;
      args.primID = unsigned(primID);
      args.timeStep = unsigned(timeStep);
      args.bounds_o = &bounds;
      args.boundsUserPtr = boundsUserPtr;
      orientedBoundsFunc(&args);
      return bounds;
    }

    static __forceinline BBox3fa bounds(const RTCOrientedBounds& bounds, const LinearSpace3fa& space = one) {
      const Vec3fa localCenter(bounds.center_x, bounds.center_y, bounds.center_z);
      const Vec3fa localAxis0(bounds.axis0_x, bounds.axis0_y, bounds.axis0_z);
      const Vec3fa localAxis1(bounds.axis1_x, bounds.axis1_y, bounds.axis1_z);
      const Vec3fa localAxis2(bounds.axis2_x, bounds.axis2_y, bounds.axis2_z);
      if (!is_finite(localCenter) || !is_finite(localAxis0) ||
          !is_finite(localAxis1) || !is_finite(localAxis2))
        return BBox3fa(empty);

      const Vec3fa center = space * localCenter;
      const Vec3fa axis0 = space * localAxis0;
      const Vec3fa axis1 = space * localAxis1;
      const Vec3fa axis2 = space * localAxis2;
      const Vec3fa extent = abs(axis0) + abs(axis1) + abs(axis2);
      const BBox3fa box(center - extent, center + extent);
      return isvalid_non_empty(box) ? box : BBox3fa(empty);
    }

    __forceinline BBox3fa bounds(size_t primID, size_t timeStep = 0) const {
      return bounds(orientedBounds(primID, timeStep));
    }

    __forceinline bool valid(size_t primID, const range<size_t>& timeSteps) const {
      for (size_t timeStep = timeSteps.begin(); timeStep <= timeSteps.end(); ++timeStep)
        if (!isvalid_non_empty(bounds(primID, timeStep)))
          return false;
      return true;
    }

    __forceinline LBBox3fa linearBounds(size_t primID, size_t timeStep) const {
      return LBBox3fa(bounds(primID, timeStep), bounds(primID, timeStep + 1));
    }

    __forceinline LBBox3fa linearBounds(size_t primID, const BBox1f& timeRange) const {
      return LBBox3fa([&] (size_t timeStep) { return bounds(primID, timeStep); },
                      timeRange, time_range, fnumTimeSegments);
    }

    __forceinline bool linearBounds(size_t primID, const BBox1f& timeRange, LBBox3fa& bounds_o) const {
      if (!valid(primID, timeSegmentRange(timeRange)))
        return false;
      bounds_o = linearBounds(primID, timeRange);
      return true;
    }

    __forceinline bool buildBounds(size_t primID, BBox3fa* bounds_o = nullptr) const {
      const BBox3fa box = bounds(primID);
      if (bounds_o)
        *bounds_o = box;
      return isvalid_non_empty(box);
    }

    __forceinline bool buildBounds(size_t primID, size_t timeStep, BBox3fa& bounds_o) const {
      const LBBox3fa linear = linearBounds(primID, timeStep);
      bounds_o = linear.bounds0;
      return isvalid_non_empty(linear);
    }

    static __forceinline bool normalizeAxis(const Vec3fa& axis, Vec3fa& normalized) {
      if (!is_finite(axis))
        return false;
      const float scale = max(abs(axis.x), max(abs(axis.y), abs(axis.z)));
      if (!(scale > 0.0f) || !std::isfinite(scale))
        return false;
      const Vec3fa scaled = axis / scale;
      const float lengthSquared = sqr_length(scaled);
      if (!(lengthSquared > 1.0e-20f) || !std::isfinite(lengthSquared))
        return false;
      normalized = scaled * rsqrt(lengthSquared);
      return is_finite(normalized);
    }

    static __forceinline Vec3fa direction(const RTCOrientedBounds& bounds) {
      const Vec3fa axes[] = {
        Vec3fa(bounds.axis0_x, bounds.axis0_y, bounds.axis0_z),
        Vec3fa(bounds.axis1_x, bounds.axis1_y, bounds.axis1_z),
        Vec3fa(bounds.axis2_x, bounds.axis2_y, bounds.axis2_z)
      };

      Vec3fa direction(zero);
      float longest = 0.0f;
      for (size_t i = 0; i < 3; ++i) {
        if (!is_finite(axes[i]))
          continue;
        const float extent = max(abs(axes[i].x), max(abs(axes[i].y), abs(axes[i].z)));
        if (extent > longest && normalizeAxis(axes[i], direction))
          longest = extent;
      }
      return direction;
    }

    static __forceinline LinearSpace3fa alignedSpace(const RTCOrientedBounds& bounds) {
      const Vec3fa inputAxis0(bounds.axis0_x, bounds.axis0_y, bounds.axis0_z);
      const Vec3fa inputAxis1(bounds.axis1_x, bounds.axis1_y, bounds.axis1_z);
      const Vec3fa inputAxis2(bounds.axis2_x, bounds.axis2_y, bounds.axis2_z);

      Vec3fa axis0;
      if (!normalizeAxis(inputAxis0, axis0) &&
          !normalizeAxis(inputAxis1, axis0) &&
          !normalizeAxis(inputAxis2, axis0))
        return LinearSpace3fa(one);

      Vec3fa axis1;
      Vec3fa orthogonal = inputAxis1 - dot(inputAxis1, axis0) * axis0;
      if (!normalizeAxis(orthogonal, axis1)) {
        orthogonal = inputAxis2 - dot(inputAxis2, axis0) * axis0;
        if (!normalizeAxis(orthogonal, axis1))
          return frame(axis0).transposed();
      }

      Vec3fa axis2 = cross(axis0, axis1);
      if (dot(axis2, inputAxis2) < 0.0f)
        axis2 = -axis2;
      return LinearSpace3fa(axis0, axis1, axis2).transposed();
    }

    virtual LinearSpace3fa computeAlignedSpace(const size_t primID) const override {
      return alignedSpace(orientedBounds(primID));
    }

    virtual LinearSpace3fa computeAlignedSpaceMB(const size_t primID, const BBox1f timeRange) const override {
      const range<int> timeSteps = timeSegmentRange(timeRange);
      return alignedSpace(orientedBounds(primID, (timeSteps.begin() + timeSteps.end()) / 2));
    }

    virtual Vec3fa computeDirection(unsigned int primID) const override {
      return direction(orientedBounds(primID));
    }

    virtual Vec3fa computeDirection(unsigned int primID, size_t time) const override {
      return direction(orientedBounds(primID, time));
    }

    virtual BBox3fa vbounds(size_t primID) const override {
      return bounds(primID);
    }

    virtual BBox3fa vbounds(const LinearSpace3fa& space, size_t primID) const override {
      return bounds(orientedBounds(primID), space);
    }

    virtual LBBox3fa vlinearBounds(size_t primID, const BBox1f& time_range) const override {
      return linearBounds(primID, time_range);
    }

    virtual LBBox3fa vlinearBounds(const LinearSpace3fa& space, size_t primID, const BBox1f& time_range) const override {
      const LBBox3fa lb = LBBox3fa(
        [&] (size_t timeStep) { return bounds(orientedBounds(primID, timeStep), space); },
        time_range, this->time_range, fnumTimeSegments);
      return lb;
    }

  public:
    RTCOrientedBoundsFunction orientedBoundsFunc = nullptr;
  };

  namespace isa
  {
    struct UserGeometryISA : public UserGeometry
    {
      UserGeometryISA (Device* device)
        : UserGeometry(device) {}

      PrimInfo createPrimRefArray(PrimRef* prims, const range<size_t>& r, size_t k, unsigned int geomID) const
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

      PrimInfo createPrimRefArrayMB(mvector<PrimRef>& prims, size_t itime, const range<size_t>& r, size_t k, unsigned int geomID) const
      {
        PrimInfo pinfo(empty);
        for (size_t j=r.begin(); j<r.end(); j++)
        {
          BBox3fa bounds = empty;
          if (!buildBounds(j,itime,bounds)) continue;
          const PrimRef prim(bounds,geomID,unsigned(j));
          pinfo.add_center2(prim);
          prims[k++] = prim;
        }
        return pinfo;
      }

      PrimInfo createPrimRefArrayMB(PrimRef* prims, const BBox1f& time_range, const range<size_t>& r, size_t k, unsigned int geomID) const
      {
        PrimInfo pinfo(empty);
        const BBox1f t0t1 = BBox1f::intersect(getTimeRange(), time_range);
        if (t0t1.empty()) return pinfo;
        
        for (size_t j = r.begin(); j < r.end(); j++) {
          LBBox3fa lbounds = empty;
          if (!linearBounds(j, t0t1, lbounds))
            continue;
          const PrimRef prim(lbounds.bounds(), geomID, unsigned(j));
          pinfo.add_center2(prim);
          prims[k++] = prim;
        }
        return pinfo;
      }

      PrimInfoMB createPrimRefMBArray(mvector<PrimRefMB>& prims, const BBox1f& t0t1, const range<size_t>& r, size_t k, unsigned int geomID) const
      {
        PrimInfoMB pinfo(empty);
        for (size_t j=r.begin(); j<r.end(); j++)
        {
          if (!valid(j, timeSegmentRange(t0t1))) continue;
          const PrimRefMB prim(linearBounds(j,t0t1),this->numTimeSegments(),this->time_range,this->numTimeSegments(),geomID,unsigned(j));
          pinfo.add_primref(prim);
          prims[k++] = prim;
        }
        return pinfo;
      }
    };

    struct OrientedUserGeometryISA : public OrientedUserGeometry
    {
      OrientedUserGeometryISA (Device* device)
        : OrientedUserGeometry(device) {}

      PrimInfo createPrimRefArray(PrimRef* prims, const range<size_t>& r, size_t k, unsigned int geomID) const
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

      PrimInfo createPrimRefArrayMB(mvector<PrimRef>& prims, size_t itime, const range<size_t>& r, size_t k, unsigned int geomID) const
      {
        PrimInfo pinfo(empty);
        for (size_t j=r.begin(); j<r.end(); j++)
        {
          BBox3fa bounds = empty;
          if (!buildBounds(j,itime,bounds)) continue;
          const PrimRef prim(bounds,geomID,unsigned(j));
          pinfo.add_center2(prim);
          prims[k++] = prim;
        }
        return pinfo;
      }

      PrimInfo createPrimRefArrayMB(PrimRef* prims, const BBox1f& time_range, const range<size_t>& r, size_t k, unsigned int geomID) const
      {
        PrimInfo pinfo(empty);
        const BBox1f t0t1 = BBox1f::intersect(getTimeRange(), time_range);
        if (t0t1.empty()) return pinfo;
        
        for (size_t j = r.begin(); j < r.end(); j++) {
          LBBox3fa lbounds = empty;
          if (!linearBounds(j, t0t1, lbounds))
            continue;
          const PrimRef prim(lbounds.bounds(), geomID, unsigned(j));
          pinfo.add_center2(prim);
          prims[k++] = prim;
        }
        return pinfo;
      }

      PrimInfoMB createPrimRefMBArray(mvector<PrimRefMB>& prims, const BBox1f& t0t1, const range<size_t>& r, size_t k, unsigned int geomID) const
      {
        PrimInfoMB pinfo(empty);
        for (size_t j=r.begin(); j<r.end(); j++)
        {
          if (!valid(j, timeSegmentRange(t0t1))) continue;
          const PrimRefMB prim(linearBounds(j,t0t1),this->numTimeSegments(),this->time_range,this->numTimeSegments(),geomID,unsigned(j));
          pinfo.add_primref(prim);
          prims[k++] = prim;
        }
        return pinfo;
      }
    };
  }
  
  DECLARE_ISA_FUNCTION(UserGeometry*, createUserGeometry, Device*);
  DECLARE_ISA_FUNCTION(UserGeometry*, createOrientedUserGeometry, Device*);
}
