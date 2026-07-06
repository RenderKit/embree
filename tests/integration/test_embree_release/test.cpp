// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include <catch2/catch_test_macros.hpp>

#include <embree4/rtcore.h>
#include <embree4/rtcore_builder.h>

#include <limits>
#include <cassert>
#include <cstring>
#include <iostream>
#include <vector>


struct Hit
{
  bool found_hit = false;
  unsigned int geomID = RTC_INVALID_GEOMETRY_ID;
  unsigned int primID = RTC_INVALID_GEOMETRY_ID;
  float tfar = std::numeric_limits<float>::infinity();
};

struct BuildTestNodeHeader
{
  unsigned int childCount;
};

static bool buildProgress(void* /*userPtr*/, double /*f*/)
{
  return true;
}

static void* createNode(RTCThreadLocalAllocator alloc, unsigned int childCount, void* /*userPtr*/)
{
  const size_t bytes =
      sizeof(BuildTestNodeHeader) +
      sizeof(void*) * childCount +
      sizeof(RTCBounds) * childCount;

  char* p = (char*) rtcThreadLocalAlloc(alloc, bytes, 16);
  std::memset(p, 0, bytes);
  ((BuildTestNodeHeader*)p)->childCount = childCount;
  return p;
}

static void setNodeChildren(void* nodePtr, void** children, unsigned int childCount, void* /*userPtr*/)
{
  BuildTestNodeHeader* h = (BuildTestNodeHeader*) nodePtr;
  void** out = (void**) (h + 1);
  for (unsigned int i = 0; i < childCount; ++i) out[i] = children[i];
}

static void setNodeBounds(void* nodePtr, const RTCBounds** bounds, unsigned int childCount, void* /*userPtr*/)
{
  BuildTestNodeHeader* h = (BuildTestNodeHeader*) nodePtr;
  void** childBase = (void**) (h + 1);
  RTCBounds* out = (RTCBounds*) (childBase + h->childCount);
  for (unsigned int i = 0; i < childCount; ++i) out[i] = *bounds[i];
}

static void* createLeaf(RTCThreadLocalAllocator alloc,
                        const RTCBuildPrimitive* prims,
                        size_t primCount,
                        void* /*userPtr*/)
{
  const size_t bytes = sizeof(size_t) + primCount * sizeof(RTCBuildPrimitive);
  char* p = (char*) rtcThreadLocalAlloc(alloc, bytes, 16);
  *((size_t*)p) = primCount;
  std::memcpy(p + sizeof(size_t), prims, primCount * sizeof(RTCBuildPrimitive));
  return p;
}

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

TEST_CASE("Morton builder clamps oversized branching factor", "[bvh-builder]")
{
  RTCDevice device = rtcNewDevice(nullptr);
  RTCBVH bvh = rtcNewBVH(device);

  const size_t primitiveCount = 1024;
  std::vector<RTCBuildPrimitive> prims(primitiveCount);
  for (size_t i = 0; i < primitiveCount; ++i)
  {
    const float x = float(i % 32);
    const float y = float((i / 32) % 32);

    RTCBuildPrimitive p{};
    p.lower_x = x * 2.0f;
    p.lower_y = y * 2.0f;
    p.lower_z = 0.0f;
    p.upper_x = p.lower_x + 0.5f;
    p.upper_y = p.lower_y + 0.5f;
    p.upper_z = 0.5f;
    p.geomID = 0;
    p.primID = (unsigned int)i;
    prims[i] = p;
  }

  RTCBuildArguments args = rtcDefaultBuildArguments();
  args.byteSize = sizeof(args);
  args.buildQuality = RTC_BUILD_QUALITY_LOW;
  args.maxBranchingFactor = 64;
  args.maxDepth = 1024;
  args.minLeafSize = 1;
  args.maxLeafSize = 1;
  args.bvh = bvh;
  args.primitives = prims.data();
  args.primitiveCount = prims.size();
  args.primitiveArrayCapacity = prims.size();
  args.createNode = createNode;
  args.setNodeChildren = setNodeChildren;
  args.setNodeBounds = setNodeBounds;
  args.createLeaf = createLeaf;
  args.buildProgress = buildProgress;

  void* root = rtcBuildBVH(&args);
  REQUIRE(root != nullptr);

  rtcReleaseBVH(bvh);
  rtcReleaseDevice(device);
}
