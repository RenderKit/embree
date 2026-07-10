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
    virtual void setOrientedBoundsFunction (RTCBoundsFunction bounds, void* userPtr) override;
    virtual void setIntersectFunctionN (RTCIntersectFunctionN intersect) override;
    virtual void setOccludedFunctionN (RTCOccludedFunctionN occluded) override;
    virtual void build() override {}
    virtual void addElementsToCount (GeometryCounts & counts) const override;
    virtual size_t getGeometryDataDeviceByteSize() const override;
    virtual void convertToDeviceRepresentation(size_t offset, char* data_host, char* data_device) const override;

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

    virtual void setOrientedBoundsFunction (RTCBoundsFunction bounds, void* userPtr) override {
      this->boundsFunc = bounds;
      Geometry::update();
    }

    virtual Vec3fa computeDirection(unsigned int primID) const override {
      return Vec3fa(1.0f, 0.0f, 0.0f);
    }

    virtual Vec3fa computeDirection(unsigned int primID, size_t time) const override {
      return Vec3fa(1.0f, 0.0f, 0.0f);
    }

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
