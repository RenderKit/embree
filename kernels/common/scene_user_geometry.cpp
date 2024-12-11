// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "scene_user_geometry.h"
#include "scene.h"

namespace embree
{
#if defined(EMBREE_LOWEST_ISA)

  UserGeometry::UserGeometry (Device* device, unsigned int items, unsigned int numTimeSteps) 
    : AccelSet(device,Geometry::GTY_USER_GEOMETRY,items,numTimeSteps) {}

  void UserGeometry::addElementsToCount (GeometryCounts & counts) const
  {
    if (numTimeSteps == 1) counts.numUserGeometries += numPrimitives;
    else                   counts.numMBUserGeometries += numPrimitives;
  }
  
  void UserGeometry::setMask (unsigned mask) 
  {
    this->mask = mask; 
    Geometry::update();
  }

  void UserGeometry::setBoundsFunction (RTCBoundsFunction bounds, void* userPtr) {
    this->boundsFunc = bounds;
  }

  void UserGeometry::setIntersectFunctionN (RTCIntersectFunctionN intersect) {
    intersectorN.intersect = intersect;
  }

  void UserGeometry::setOccludedFunctionN (RTCOccludedFunctionN occluded) {
    intersectorN.occluded = occluded;
  }

  size_t UserGeometry::getGeometryDataDeviceByteSize() const {
    return 16 * ((sizeof(UserGeometry) + 15) / 16);
  }

  void UserGeometry::convertToDeviceRepresentation(size_t offset, char* data_host, char* data_device) const {
    std::memcpy(data_host + offset, (void*)this, sizeof(UserGeometry));
    offset += sizeof(Instance);
  }

#endif

  namespace isa
  {
    UserGeometry* createUserGeometry(Device* device) {
      return new UserGeometryISA(device);
    }
  }
}
