// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../common/tutorial/tutorial_device.h"

namespace embree {

struct Point;

struct TutorialData
{
  RTCScene scene;
  Point* points;
  Point* points_tmp;
  Vec3fa* colors;

  Vec3fa query_point;
  int num_points;
  int num_knn;
  bool show_voronoi;
  bool point_repulsion;
  float tmax;
};

inline void TutorialData_Constructor(TutorialData* This)
{
  This->scene = nullptr;
  This->points = nullptr;
  This->points_tmp = nullptr;
  This->colors = nullptr;
}

inline void TutorialData_Destructor(TutorialData* This)
{
  rtcReleaseScene (This->scene); This->scene = nullptr;
  alignedFree(This->points); This->points = nullptr;
  alignedFree(This->points_tmp); This->points_tmp = nullptr;
  alignedFree(This->colors); This->colors = nullptr;
}

} // namespace embree
