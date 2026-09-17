// Copyright 2009-2026 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include <embree4/rtcore.h>

#include <atomic>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

struct Primitive
{
  RTCOrientedBounds bounds;
};

static void* expectedBoundsUserPtr = nullptr;
static std::atomic<bool> boundsUserPtrMismatch(false);

static void boundsFunction(const RTCOrientedBoundsFunctionArguments* args)
{
  if (args->boundsUserPtr != expectedBoundsUserPtr)
    boundsUserPtrMismatch.store(true);
  const Primitive* primitives = static_cast<const Primitive*>(args->geometryUserPtr);
  *args->bounds_o = primitives[args->primID].bounds;
}

static bool intersectPrimitive(const Primitive& primitive, RTCRayN* rays, unsigned int N, unsigned int lane, float& t)
{
  const RTCOrientedBounds& bounds = primitive.bounds;
  const float dirZ = RTCRayN_dir_z(rays, N, lane);
  if (dirZ == 0.0f)
    return false;

  t = (bounds.center_z - RTCRayN_org_z(rays, N, lane)) / dirZ;
  if (t < RTCRayN_tnear(rays, N, lane) || t > RTCRayN_tfar(rays, N, lane))
    return false;

  const float x = RTCRayN_org_x(rays, N, lane) + t * RTCRayN_dir_x(rays, N, lane);
  const float y = RTCRayN_org_y(rays, N, lane) + t * RTCRayN_dir_y(rays, N, lane);
  const float extentX = std::abs(bounds.axis0_x) + std::abs(bounds.axis1_x) + std::abs(bounds.axis2_x);
  const float extentY = std::abs(bounds.axis0_y) + std::abs(bounds.axis1_y) + std::abs(bounds.axis2_y);
  return x >= bounds.center_x - extentX && x <= bounds.center_x + extentX &&
         y >= bounds.center_y - extentY && y <= bounds.center_y + extentY;
}

static void intersectFunction(const RTCIntersectFunctionNArguments* args)
{
  const Primitive* primitives = static_cast<const Primitive*>(args->geometryUserPtr);
  RTCRayHitN* rayHits = reinterpret_cast<RTCRayHitN*>(args->rayhit);
  RTCRayN* rays = RTCRayHitN_RayN(rayHits, args->N);
  RTCHitN* hits = RTCRayHitN_HitN(rayHits, args->N);

  for (unsigned int lane = 0; lane < args->N; ++lane) {
    if (!args->valid[lane])
      continue;

    float t;
    if (!intersectPrimitive(primitives[args->primID], rays, args->N, lane, t))
      continue;

    RTCRayN_tfar(rays, args->N, lane) = t;
    RTCHitN_geomID(hits, args->N, lane) = args->geomID;
    RTCHitN_primID(hits, args->N, lane) = args->primID;
  }
}

static void occludedFunction(const RTCOccludedFunctionNArguments* args)
{
  const Primitive* primitives = static_cast<const Primitive*>(args->geometryUserPtr);
  RTCRayN* rays = reinterpret_cast<RTCRayN*>(args->ray);

  for (unsigned int lane = 0; lane < args->N; ++lane) {
    if (!args->valid[lane])
      continue;

    float t;
    if (intersectPrimitive(primitives[args->primID], rays, args->N, lane, t))
      RTCRayN_tfar(rays, args->N, lane) = -std::numeric_limits<float>::infinity();
  }
}

static std::vector<Primitive> makePrimitives()
{
  std::vector<Primitive> primitives(259);
  const float invSqrtTwo = 1.0f / std::sqrt(2.0f);

  for (size_t i = 0; i < 256; ++i) {
    const float u = (float(int(i % 16) - 8)) * 1.5f;
    const float v = (float(int(i / 16) - 8)) * 0.35f;
    const float x = (u - v) * invSqrtTwo;
    const float y = (u + v) * invSqrtTwo;
    const float z = 10.0f;
    const float radius = 0.1f;

    primitives[i].bounds.center_x = x;
    primitives[i].bounds.center_y = y;
    primitives[i].bounds.center_z = z;
    primitives[i].bounds.axis0_x = invSqrtTwo;
    primitives[i].bounds.axis0_y = invSqrtTwo;
    primitives[i].bounds.axis0_z = 0.0f;
    primitives[i].bounds.axis1_x = -radius * invSqrtTwo;
    primitives[i].bounds.axis1_y = radius * invSqrtTwo;
    primitives[i].bounds.axis1_z = 0.0f;
    primitives[i].bounds.axis2_x = 0.0f;
    primitives[i].bounds.axis2_y = 0.0f;
    primitives[i].bounds.axis2_z = radius;
  }

  primitives[256].bounds = primitives[0].bounds;
  primitives[256].bounds.center_x = std::numeric_limits<float>::quiet_NaN();
  primitives[257].bounds = primitives[0].bounds;
  primitives[257].bounds.center_x = 1000.0f;
  primitives[257].bounds.axis0_x = 0.0f;
  primitives[257].bounds.axis0_y = 0.0f;
  primitives[257].bounds.axis0_z = 0.0f;
  primitives[257].bounds.axis1_x = 0.0f;
  primitives[257].bounds.axis1_y = 0.0f;
  primitives[257].bounds.axis1_z = 0.0f;
  primitives[257].bounds.axis2_x = 0.0f;
  primitives[257].bounds.axis2_y = 0.0f;
  primitives[257].bounds.axis2_z = 0.0f;
  primitives[258].bounds = primitives[0].bounds;
  primitives[258].bounds.center_x = 2000.0f;
  primitives[258].bounds.axis0_x = std::numeric_limits<float>::infinity();
  return primitives;
}

static void initializeRay(RTCRay& ray, float x, float y)
{
  ray.org_x = x;
  ray.org_y = y;
  ray.org_z = 0.0f;
  ray.dir_x = 0.0f;
  ray.dir_y = 0.0f;
  ray.dir_z = 1.0f;
  ray.tnear = 0.0f;
  ray.tfar = std::numeric_limits<float>::infinity();
  ray.time = 0.0f;
  ray.mask = -1;
  ray.id = 0;
  ray.flags = 0;
}

static void initializeRayPacket(RTCRay4& rays, RTCHit4* hits, const float x[4], const float y[4])
{
  for (unsigned int lane = 0; lane < 4; ++lane) {
    rays.org_x[lane] = x[lane];
    rays.org_y[lane] = y[lane];
    rays.org_z[lane] = 0.0f;
    rays.dir_x[lane] = 0.0f;
    rays.dir_y[lane] = 0.0f;
    rays.dir_z[lane] = 1.0f;
    rays.tnear[lane] = 0.0f;
    rays.tfar[lane] = std::numeric_limits<float>::infinity();
    rays.time[lane] = 0.0f;
    rays.mask[lane] = -1;
    rays.id[lane] = lane;
    rays.flags[lane] = 0;
    if (hits) {
      hits->geomID[lane] = RTC_INVALID_GEOMETRY_ID;
      hits->primID[lane] = RTC_INVALID_GEOMETRY_ID;
    }
  }
}

int main(int argc, char** argv)
{
  RTCDevice device = rtcNewDevice(argc > 1 ? argv[1] : nullptr);
  if (!device) {
    std::cerr << "Failed to create Embree device\n";
    return 1;
  }
  RTCScene scene = rtcNewScene(device);
  std::vector<Primitive> primitives = makePrimitives();
  int boundsPayload = 0;
  expectedBoundsUserPtr = &boundsPayload;

  RTCGeometry geometry = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_USER_ORIENTED);
  rtcSetGeometryUserPrimitiveCount(geometry, primitives.size());
  rtcSetGeometryUserData(geometry, primitives.data());
  rtcSetGeometryOrientedBoundsFunction(geometry, boundsFunction, &boundsPayload);
  rtcSetGeometryIntersectFunction(geometry, intersectFunction);
  rtcSetGeometryOccludedFunction(geometry, occludedFunction);
  rtcCommitGeometry(geometry);
  const unsigned int geomID = rtcAttachGeometry(scene, geometry);
  rtcReleaseGeometry(geometry);

  std::vector<Primitive> motionPrimitives = primitives;
  for (Primitive& primitive : motionPrimitives) {
    primitive.bounds.center_x += 100.0f;
  }
  RTCGeometry motionGeometry = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_USER_ORIENTED);
  rtcSetGeometryTimeStepCount(motionGeometry, 2);
  rtcSetGeometryUserPrimitiveCount(motionGeometry, motionPrimitives.size());
  rtcSetGeometryUserData(motionGeometry, motionPrimitives.data());
  rtcSetGeometryOrientedBoundsFunction(motionGeometry, boundsFunction, &boundsPayload);
  rtcSetGeometryIntersectFunction(motionGeometry, intersectFunction);
  rtcSetGeometryOccludedFunction(motionGeometry, occludedFunction);
  rtcCommitGeometry(motionGeometry);
  const unsigned int motionGeomID = rtcAttachGeometry(scene, motionGeometry);
  rtcReleaseGeometry(motionGeometry);
  rtcCommitScene(scene);

  const Primitive& target = primitives[8 * 16 + 8];
  const float x = target.bounds.center_x;
  const float y = target.bounds.center_y;

  RTCRayHit rayHit{};
  initializeRay(rayHit.ray, x, y);
  rayHit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
  rtcIntersect1(scene, &rayHit);
  const bool intersected = rayHit.hit.geomID == geomID;

  RTCRay shadow{};
  initializeRay(shadow, x, y);
  rtcOccluded1(scene, &shadow);
  const bool occluded = shadow.tfar < 0.0f;

  const Primitive& motionTarget = motionPrimitives[8 * 16 + 8];
  const float motionX = motionTarget.bounds.center_x;
  const float motionY = motionTarget.bounds.center_y;

  RTCRayHit motionRayHit{};
  initializeRay(motionRayHit.ray, motionX, motionY);
  motionRayHit.ray.time = 0.5f;
  motionRayHit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
  rtcIntersect1(scene, &motionRayHit);
  const bool motionIntersected = motionRayHit.hit.geomID == motionGeomID;

  RTCRay motionShadow{};
  initializeRay(motionShadow, motionX, motionY);
  motionShadow.time = 0.5f;
  rtcOccluded1(scene, &motionShadow);
  const bool motionOccluded = motionShadow.tfar < 0.0f;

  const Primitive& secondTarget = primitives[0];
  const float packetX[4] = {
    x,
    motionX,
    secondTarget.bounds.center_x,
    1000.0f
  };
  const float packetY[4] = {
    y,
    motionY,
    secondTarget.bounds.center_y,
    1000.0f
  };
  const int valid[4] = {-1, -1, -1, -1};

  RTCRayHit4 rayHit4{};
  initializeRayPacket(rayHit4.ray, &rayHit4.hit, packetX, packetY);
  rayHit4.ray.time[1] = 0.5f;
  rtcIntersect4(valid, scene, &rayHit4);
  const bool packetIntersected =
    rayHit4.hit.geomID[0] == geomID &&
    rayHit4.hit.geomID[1] == motionGeomID &&
    rayHit4.hit.geomID[2] == geomID &&
    rayHit4.hit.geomID[3] == RTC_INVALID_GEOMETRY_ID;

  RTCRay4 shadow4{};
  initializeRayPacket(shadow4, nullptr, packetX, packetY);
  shadow4.time[1] = 0.5f;
  rtcOccluded4(valid, scene, &shadow4);
  const bool packetOccluded =
    shadow4.tfar[0] < 0.0f &&
    shadow4.tfar[1] < 0.0f &&
    shadow4.tfar[2] < 0.0f &&
    shadow4.tfar[3] >= 0.0f;

  RTCRayHit malformedRayHit{};
  initializeRay(malformedRayHit.ray, 1000.0f, primitives[257].bounds.center_y);
  malformedRayHit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
  rtcIntersect1(scene, &malformedRayHit);

  RTCRayHit nonFiniteRayHit{};
  initializeRay(nonFiniteRayHit.ray, 2000.0f, primitives[258].bounds.center_y);
  nonFiniteRayHit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
  rtcIntersect1(scene, &nonFiniteRayHit);
  const bool nonFiniteMissed = nonFiniteRayHit.hit.geomID == RTC_INVALID_GEOMETRY_ID;

  rtcReleaseScene(scene);
  rtcReleaseDevice(device);

  if (!intersected)
    std::cerr << "Oriented user geometry intersection failed\n";
  if (!occluded)
    std::cerr << "Oriented user geometry occlusion failed\n";
  if (!motionIntersected)
    std::cerr << "Oriented motion-blur user geometry intersection failed\n";
  if (!motionOccluded)
    std::cerr << "Oriented motion-blur user geometry occlusion failed\n";
  if (!packetIntersected)
    std::cerr << "Oriented user geometry packet intersection failed\n";
  if (!packetOccluded)
    std::cerr << "Oriented user geometry packet occlusion failed\n";
  if (boundsUserPtrMismatch.load())
    std::cerr << "Oriented bounds callback payload mismatch\n";
  if (!nonFiniteMissed)
    std::cerr << "Non-finite oriented bound was not ignored\n";
  return intersected && occluded && motionIntersected && motionOccluded &&
         packetIntersected && packetOccluded && !boundsUserPtrMismatch.load() &&
         nonFiniteMissed ? 0 : 1;
}
