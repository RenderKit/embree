// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include <catch2/catch_test_macros.hpp>

#include <embree4/rtcore.h>

#include <limits>
#include <cassert>
#include <iostream>


struct Hit
{
  bool found_hit = false;
  unsigned int geomID = RTC_INVALID_GEOMETRY_ID;
  unsigned int primID = RTC_INVALID_GEOMETRY_ID;
  float tfar = std::numeric_limits<float>::infinity();
};

inline Hit castRay(RTCScene scene, 
                    float ox, float oy, float oz,
                    float dx, float dy, float dz)
{
  Hit result;
  struct RTCRayHit rayhit;
  rayhit.ray.org_x = ox;
  rayhit.ray.org_y = oy;
  rayhit.ray.org_z = oz;
  rayhit.ray.dir_x = dx;
  rayhit.ray.dir_y = dy;
  rayhit.ray.dir_z = dz;
  rayhit.ray.tnear = 0;
  rayhit.ray.tfar = std::numeric_limits<float>::infinity();
  rayhit.ray.mask = -1;
  rayhit.ray.flags = 0;
  rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
  rayhit.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
  rayhit.hit.instPrimID[0] = RTC_INVALID_GEOMETRY_ID;

  rtcIntersect1(scene, &rayhit);

  printf("%f, %f, %f: ", ox, oy, oz);
  if (rayhit.hit.geomID != RTC_INVALID_GEOMETRY_ID)
  {
    result.found_hit = true;
    result.geomID = rayhit.hit.geomID;
    result.primID = rayhit.hit.primID;
    result.tfar = rayhit.ray.tfar;
    printf("Found intersection on geometry %d, primitive %d at tfar=%f\n", rayhit.hit.geomID, rayhit.hit.primID, rayhit.ray.tfar);
  }
  else
    printf("Did not find any intersection\n");
  
  return result;
}


TEST_CASE("Minimal test", "[minimal]")
{
  RTCDevice device = rtcNewDevice(NULL);
  RTCScene scene = rtcNewScene(device);

  RTCGeometry geom = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_TRIANGLE);
  float* vertices = (float*) rtcSetNewGeometryBuffer(geom,
                                                     RTC_BUFFER_TYPE_VERTEX,
                                                     0,
                                                     RTC_FORMAT_FLOAT3,
                                                     3*sizeof(float),
                                                     3);

  unsigned* indices = (unsigned*) rtcSetNewGeometryBuffer(geom,
                                                          RTC_BUFFER_TYPE_INDEX,
                                                          0,
                                                          RTC_FORMAT_UINT3,
                                                          3*sizeof(unsigned),
                                                          1);

  vertices[0] = 0.f; vertices[1] = 0.f; vertices[2] = 0.f;
  vertices[3] = 1.f; vertices[4] = 0.f; vertices[5] = 0.f;
  vertices[6] = 0.f; vertices[7] = 1.f; vertices[8] = 0.f;
  indices[0] = 0; indices[1] = 1; indices[2] = 2;
  rtcCommitGeometry(geom);
  rtcAttachGeometry(scene, geom);
  rtcReleaseGeometry(geom);
  rtcCommitScene(scene);

  SECTION("ray hitting") {
    Hit hit = castRay(scene, 0.33f, 0.33f, -1, 0, 0, 1);
    REQUIRE(hit.found_hit == true);
    REQUIRE(hit.geomID == 0);
    REQUIRE(hit.primID == 0);
    REQUIRE(std::abs(hit.tfar - 1.0) < 1e-6f);
  }
  SECTION("ray missing") {
    Hit hit = castRay(scene, 1.00f, 1.00f, -1, 0, 0, 1);
    REQUIRE(hit.found_hit == false);
    REQUIRE(hit.geomID == RTC_INVALID_GEOMETRY_ID);
    REQUIRE(hit.primID == RTC_INVALID_GEOMETRY_ID);
    REQUIRE(hit.tfar == std::numeric_limits<float>::infinity());
  }

  rtcReleaseScene(scene);
  rtcReleaseDevice(device);

  REQUIRE(true);
}

