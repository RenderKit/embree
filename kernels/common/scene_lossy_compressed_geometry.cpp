// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "scene_lossy_compressed_geometry.h"
#include "scene.h"

namespace embree
{
#if defined(EMBREE_LOWEST_ISA)

  LossyCompressedGeometry::LossyCompressedGeometry (Device* device, unsigned int items, unsigned int numTimeSteps) 
    : AccelSet(device,Geometry::GTY_LOSSY_COMPRESSED_GEOMETRY,items,numTimeSteps) {}

  void LossyCompressedGeometry::addElementsToCount (GeometryCounts & counts) const
  {
    if (numTimeSteps == 1) counts.numUserGeometries += numPrimitives;
    else                   counts.numMBUserGeometries += numPrimitives;
  }
  
  void LossyCompressedGeometry::setMask (unsigned mask) 
  {
    this->mask = mask; 
    Geometry::update();
  }

  void LossyCompressedGeometry::setIntersectFunctionN (RTCIntersectFunctionN intersect) {
    intersectorN.intersect = intersect;
  }

  void LossyCompressedGeometry::setOccludedFunctionN (RTCOccludedFunctionN occluded) {
    intersectorN.occluded = occluded;
  }
  
#endif

  namespace isa
  {
    LossyCompressedGeometry* createLossyCompressedGeometry(Device* device) {
      return new LossyCompressedGeometryISA(device);
    }
  }
}
