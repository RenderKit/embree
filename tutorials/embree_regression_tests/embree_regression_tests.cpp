// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include <embree4/rtcore.h>
#include <embree4/rtcore_builder.h>

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#if defined(RTC_NAMESPACE_USE)
RTC_NAMESPACE_USE
#endif

namespace
{
  struct CaseResult
  {
    bool pass;
    bool skip;
    std::string message;
  };

  static RTCError consumeDeviceError(RTCDevice device)
  {
    return rtcGetDeviceError(device);
  }

  static bool isFiniteBounds(const RTCBounds& b)
  {
    const float v[6] = { b.lower_x, b.lower_y, b.lower_z, b.upper_x, b.upper_y, b.upper_z };
    for (size_t i = 0; i < 6; ++i) {
      if (!std::isfinite(v[i]))
        return false;
    }
    return true;
  }

  static bool errorIsAccepted(RTCError err)
  {
    return err == RTC_ERROR_NONE || err == RTC_ERROR_INVALID_ARGUMENT || err == RTC_ERROR_INVALID_OPERATION;
  }

  static float* setTransformBuffer1(RTCGeometry geom, float* xfm)
  {
    rtcSetSharedGeometryBuffer(
      geom,
      RTC_BUFFER_TYPE_TRANSFORM,
      0,
      RTC_FORMAT_FLOAT4X4_COLUMN_MAJOR,
      xfm,
      0,
      16 * sizeof(float),
      1);
    return xfm;
  }

  static void setIdentityXfm(float* m)
  {
    for (int i = 0; i < 16; ++i)
      m[i] = 0.0f;
    m[0] = 1.0f;
    m[5] = 1.0f;
    m[10] = 1.0f;
    m[15] = 1.0f;
  }

  static RTCScene createTriangleScene(RTCDevice device)
  {
    RTCScene scene = rtcNewScene(device);
    RTCGeometry geom = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_TRIANGLE);

    float* vertices = (float*)rtcSetNewGeometryBuffer(
      geom, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, 3 * sizeof(float), 3);
    unsigned* indices = (unsigned*)rtcSetNewGeometryBuffer(
      geom, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, 3 * sizeof(unsigned), 1);

    vertices[0] = 0.0f; vertices[1] = 0.0f; vertices[2] = 0.0f;
    vertices[3] = 1.0f; vertices[4] = 0.0f; vertices[5] = 0.0f;
    vertices[6] = 0.0f; vertices[7] = 1.0f; vertices[8] = 0.0f;

    indices[0] = 0;
    indices[1] = 1;
    indices[2] = 2;

    rtcCommitGeometry(geom);
    rtcAttachGeometry(scene, geom);
    rtcReleaseGeometry(geom);
    rtcCommitScene(scene);
    return scene;
  }

  static CaseResult passResult(const char* msg)
  {
    CaseResult r;
    r.pass = true;
    r.skip = false;
    r.message = msg;
    return r;
  }

  static CaseResult failResult(const char* msg)
  {
    CaseResult r;
    r.pass = false;
    r.skip = false;
    r.message = msg;
    return r;
  }

  static CaseResult skipResult(const char* msg)
  {
    CaseResult r;
    r.pass = true;
    r.skip = true;
    r.message = msg;
    return r;
  }

  static CaseResult issue01_time_segment_range_clamp(RTCDevice device)
  {
    RTCScene child = createTriangleScene(device);
    if (consumeDeviceError(device) != RTC_ERROR_NONE) {
      rtcReleaseScene(child);
      return failResult("failed to build child scene");
    }

    RTCScene top = rtcNewScene(device);
    RTCGeometry inst = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_INSTANCE);
    if (!inst) {
      rtcReleaseScene(top);
      rtcReleaseScene(child);
      return skipResult("instance geometry unsupported");
    }

    rtcSetGeometryInstancedScene(inst, child);
    rtcSetGeometryTimeStepCount(inst, 2);
    rtcSetGeometryTimeRange(inst, 1.0f, 0.0f);
    float xfm0[16], xfm1[16];
    setIdentityXfm(xfm0);
    setIdentityXfm(xfm1);
    rtcSetGeometryTransform(inst, 0, RTC_FORMAT_FLOAT4X4_COLUMN_MAJOR, xfm0);
    rtcSetGeometryTransform(inst, 1, RTC_FORMAT_FLOAT4X4_COLUMN_MAJOR, xfm1);

    rtcCommitGeometry(inst);
    rtcAttachGeometry(top, inst);
    rtcReleaseGeometry(inst);
    rtcCommitScene(top);

    RTCError err = consumeDeviceError(device);
    if (!errorIsAccepted(err)) {
      rtcReleaseScene(top);
      rtcReleaseScene(child);
      return failResult("unexpected API error");
    }

    if (err == RTC_ERROR_NONE) {
      RTCBounds b;
      rtcGetSceneBounds(top, &b);
      if (!isFiniteBounds(b)) {
        rtcReleaseScene(top);
        rtcReleaseScene(child);
        return failResult("non-finite bounds");
      }
    }

    rtcReleaseScene(top);
    rtcReleaseScene(child);
    return passResult("safe handling for inverted time range");
  }

  static CaseResult issue02_lbbox_nan_range(RTCDevice device)
  {
    RTCScene child = createTriangleScene(device);
    if (consumeDeviceError(device) != RTC_ERROR_NONE) {
      rtcReleaseScene(child);
      return failResult("failed to build child scene");
    }

    RTCScene top = rtcNewScene(device);
    RTCGeometry inst = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_INSTANCE);
    if (!inst) {
      rtcReleaseScene(top);
      rtcReleaseScene(child);
      return skipResult("instance geometry unsupported");
    }

    rtcSetGeometryInstancedScene(inst, child);
    rtcSetGeometryTimeStepCount(inst, 2);
    rtcSetGeometryTimeRange(inst, std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN());
    float xfm0[16], xfm1[16];
    setIdentityXfm(xfm0);
    setIdentityXfm(xfm1);
    rtcSetGeometryTransform(inst, 0, RTC_FORMAT_FLOAT4X4_COLUMN_MAJOR, xfm0);
    rtcSetGeometryTransform(inst, 1, RTC_FORMAT_FLOAT4X4_COLUMN_MAJOR, xfm1);

    rtcCommitGeometry(inst);
    rtcAttachGeometry(top, inst);
    rtcReleaseGeometry(inst);
    rtcCommitScene(top);

    RTCError err = consumeDeviceError(device);
    if (!errorIsAccepted(err)) {
      rtcReleaseScene(top);
      rtcReleaseScene(child);
      return failResult("unexpected API error");
    }

    if (err == RTC_ERROR_NONE) {
      RTCBounds b;
      rtcGetSceneBounds(top, &b);
      if (!isFiniteBounds(b)) {
        rtcReleaseScene(top);
        rtcReleaseScene(child);
        return failResult("NaN bounds");
      }
    }

    rtcReleaseScene(top);
    rtcReleaseScene(child);
    return passResult("safe handling for NaN range");
  }

  static CaseResult issue03_lbbox_extreme_range(RTCDevice device)
  {
    RTCScene child = createTriangleScene(device);
    RTCScene top = rtcNewScene(device);
    RTCGeometry inst = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_INSTANCE);
    if (!inst) {
      rtcReleaseScene(top);
      rtcReleaseScene(child);
      return skipResult("instance geometry unsupported");
    }

    rtcSetGeometryInstancedScene(inst, child);
    rtcSetGeometryTimeStepCount(inst, 2);
    rtcSetGeometryTimeRange(inst, -1.0e30f, 1.0e30f);
    float xfm0[16], xfm1[16];
    setIdentityXfm(xfm0);
    setIdentityXfm(xfm1);
    rtcSetGeometryTransform(inst, 0, RTC_FORMAT_FLOAT4X4_COLUMN_MAJOR, xfm0);
    rtcSetGeometryTransform(inst, 1, RTC_FORMAT_FLOAT4X4_COLUMN_MAJOR, xfm1);

    rtcCommitGeometry(inst);
    rtcAttachGeometry(top, inst);
    rtcReleaseGeometry(inst);
    rtcCommitScene(top);

    RTCError err = consumeDeviceError(device);
    if (!errorIsAccepted(err)) {
      rtcReleaseScene(top);
      rtcReleaseScene(child);
      return failResult("unexpected API error");
    }

    if (err == RTC_ERROR_NONE) {
      RTCBounds b;
      rtcGetSceneBounds(top, &b);
      if (!isFiniteBounds(b)) {
        rtcReleaseScene(top);
        rtcReleaseScene(child);
        return failResult("non-finite bounds under extreme range");
      }
    }

    rtcReleaseScene(top);
    rtcReleaseScene(child);
    return passResult("extreme range handled safely");
  }

  static CaseResult issue04_instance_bound_segment_guard(RTCDevice device)
  {
    RTCScene child = createTriangleScene(device);
    RTCScene top = rtcNewScene(device);
    RTCGeometry inst = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_INSTANCE);
    if (!inst) {
      rtcReleaseScene(top);
      rtcReleaseScene(child);
      return skipResult("instance geometry unsupported");
    }

    rtcSetGeometryInstancedScene(inst, child);
    rtcSetGeometryTimeStepCount(inst, 2);
    float xfm0[16], xfm1[16];
    setIdentityXfm(xfm0);
    setIdentityXfm(xfm1);
    xfm1[12] = 0.1f;
    rtcSetGeometryTransform(inst, 0, RTC_FORMAT_FLOAT4X4_COLUMN_MAJOR, xfm0);
    rtcSetGeometryTransform(inst, 1, RTC_FORMAT_FLOAT4X4_COLUMN_MAJOR, xfm1);
    rtcCommitGeometry(inst);
    rtcAttachGeometry(top, inst);
    rtcReleaseGeometry(inst);
    rtcCommitScene(top);

    RTCRayHit rayhit;
    std::memset(&rayhit, 0, sizeof(rayhit));
    rayhit.ray.org_x = 0.2f;
    rayhit.ray.org_y = 0.2f;
    rayhit.ray.org_z = -1.0f;
    rayhit.ray.dir_x = 0.0f;
    rayhit.ray.dir_y = 0.0f;
    rayhit.ray.dir_z = 1.0f;
    rayhit.ray.tnear = 0.0f;
    rayhit.ray.tfar = std::numeric_limits<float>::infinity();
    rayhit.ray.time = 2.0f;
    rayhit.ray.mask = 0xFFFFFFFFu;
    rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
    rayhit.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;

    RTCIntersectArguments args;
    rtcInitIntersectArguments(&args);
    rtcIntersect1(top, &rayhit, &args);

    RTCError err = consumeDeviceError(device);
    rtcReleaseScene(top);
    rtcReleaseScene(child);
    if (err != RTC_ERROR_NONE)
      return failResult("intersection returned API error");
    return passResult("out-of-range ray time handled safely");
  }

  static CaseResult issue05_instance_nonlinear_bounds(RTCDevice device)
  {
    RTCScene child = createTriangleScene(device);
    RTCScene top = rtcNewScene(device);
    RTCGeometry inst = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_INSTANCE);
    if (!inst) {
      rtcReleaseScene(top);
      rtcReleaseScene(child);
      return skipResult("instance geometry unsupported");
    }

    rtcSetGeometryInstancedScene(inst, child);
    rtcSetGeometryTimeStepCount(inst, 2);
    rtcSetGeometryTimeRange(inst, 0.9f, 0.1f);

    RTCQuaternionDecomposition q0;
    RTCQuaternionDecomposition q1;
    rtcInitQuaternionDecomposition(&q0);
    rtcInitQuaternionDecomposition(&q1);
    rtcQuaternionDecompositionSetTranslation(&q0, 0.0f, 0.0f, 0.0f);
    rtcQuaternionDecompositionSetTranslation(&q1, 1.0f, 0.0f, 0.0f);
    rtcQuaternionDecompositionSetQuaternion(&q1, 0.9238795f, 0.0f, 0.3826834f, 0.0f);
    rtcSetGeometryTransformQuaternion(inst, 0, &q0);
    rtcSetGeometryTransformQuaternion(inst, 1, &q1);

    rtcCommitGeometry(inst);
    rtcAttachGeometry(top, inst);
    rtcReleaseGeometry(inst);
    rtcCommitScene(top);

    RTCError err = consumeDeviceError(device);
    if (!errorIsAccepted(err)) {
      rtcReleaseScene(top);
      rtcReleaseScene(child);
      return failResult("unexpected API error");
    }

    if (err == RTC_ERROR_NONE) {
      RTCBounds b;
      rtcGetSceneBounds(top, &b);
      if (!isFiniteBounds(b)) {
        rtcReleaseScene(top);
        rtcReleaseScene(child);
        return failResult("non-finite bounds");
      }
    }

    rtcReleaseScene(top);
    rtcReleaseScene(child);
    return passResult("nonlinear bounds path handled safely");
  }

  static CaseResult issue06_mb_builder_range_check(RTCDevice device)
  {
    RTCScene scene = rtcNewScene(device);
    RTCGeometry geom = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_TRIANGLE);
    if (!geom) {
      rtcReleaseScene(scene);
      return skipResult("triangle geometry unsupported");
    }

    const unsigned int numTri = 2048;
    const unsigned int numVertices = numTri * 3;
    rtcSetGeometryTimeStepCount(geom, 2);

    float* v0 = (float*)rtcSetNewGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, 3 * sizeof(float), numVertices);
    float* v1 = (float*)rtcSetNewGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX, 1, RTC_FORMAT_FLOAT3, 3 * sizeof(float), numVertices);
    unsigned* idx = (unsigned*)rtcSetNewGeometryBuffer(geom, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, 3 * sizeof(unsigned), numTri);

    for (unsigned int i = 0; i < numTri; ++i) {
      const float x = float(i % 64) * 0.05f;
      const float y = float(i / 64) * 0.05f;
      const unsigned int base = 3 * i;
      v0[3 * base + 0] = x;        v0[3 * base + 1] = y;        v0[3 * base + 2] = 0.0f;
      v0[3 * base + 3] = x + 0.01f;v0[3 * base + 4] = y;        v0[3 * base + 5] = 0.0f;
      v0[3 * base + 6] = x;        v0[3 * base + 7] = y + 0.01f;v0[3 * base + 8] = 0.0f;

      v1[3 * base + 0] = x;         v1[3 * base + 1] = y;         v1[3 * base + 2] = 0.02f;
      v1[3 * base + 3] = x + 0.01f; v1[3 * base + 4] = y;         v1[3 * base + 5] = 0.02f;
      v1[3 * base + 6] = x;         v1[3 * base + 7] = y + 0.01f; v1[3 * base + 8] = 0.02f;

      idx[3 * i + 0] = base + 0;
      idx[3 * i + 1] = base + 1;
      idx[3 * i + 2] = base + 2;
    }

    rtcCommitGeometry(geom);
    rtcAttachGeometry(scene, geom);
    rtcReleaseGeometry(geom);
    rtcCommitScene(scene);

    RTCError err = consumeDeviceError(device);
    rtcReleaseScene(scene);
    if (err != RTC_ERROR_NONE)
      return failResult("motion-blur scene build failed");
    return passResult("motion-blur builder completed safely");
  }

  static void simpleBoundsFunc(const RTCBoundsFunctionArguments* args)
  {
    args->bounds_o->lower_x = -1.0f;
    args->bounds_o->lower_y = -1.0f;
    args->bounds_o->lower_z = -1.0f;
    args->bounds_o->upper_x = +1.0f;
    args->bounds_o->upper_y = +1.0f;
    args->bounds_o->upper_z = +1.0f;
  }

  static CaseResult issue07_user_bounds_time_range_validation(RTCDevice device)
  {
    RTCScene scene = rtcNewScene(device);
    RTCGeometry user = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_USER);
    if (!user) {
      rtcReleaseScene(scene);
      return skipResult("user geometry unsupported");
    }

    rtcSetGeometryUserPrimitiveCount(user, 1);
    rtcSetGeometryBoundsFunction(user, simpleBoundsFunc, nullptr);
    rtcSetGeometryTimeStepCount(user, 2);
    rtcSetGeometryTimeRange(user, 1.0f, 0.0f);
    rtcCommitGeometry(user);
    rtcAttachGeometry(scene, user);
    rtcReleaseGeometry(user);
    rtcCommitScene(scene);

    RTCError err = consumeDeviceError(device);
    rtcReleaseScene(scene);
    if (!errorIsAccepted(err))
      return failResult("unexpected API error");
    return passResult("invalid user-geometry range handled safely");
  }

  static CaseResult issue08_instance_array_object_id_validation(RTCDevice device)
  {
    RTCScene child = createTriangleScene(device);
    RTCScene top = rtcNewScene(device);
    RTCGeometry iarr = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_INSTANCE_ARRAY);
    if (!iarr) {
      rtcReleaseScene(top);
      rtcReleaseScene(child);
      return skipResult("instance array unsupported");
    }

    float xfm[16];
    setIdentityXfm(xfm);
    unsigned int objectIDs[1] = { 1u };
    RTCScene scenes[1] = { child };

    setTransformBuffer1(iarr, xfm);
    rtcSetSharedGeometryBuffer(iarr, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT, objectIDs, 0, sizeof(unsigned int), 1);
    rtcSetGeometryInstancedScenes(iarr, scenes, 1);
    rtcCommitGeometry(iarr);

    RTCError err = consumeDeviceError(device);
    rtcReleaseGeometry(iarr);
    rtcReleaseScene(top);
    rtcReleaseScene(child);
    if (err != RTC_ERROR_INVALID_ARGUMENT)
      return failResult("expected invalid object id failure");
    return passResult("invalid object id rejected");
  }

  static CaseResult issue09_instance_array_transform_oob(RTCDevice device)
  {
    RTCScene child = createTriangleScene(device);
    RTCScene top = rtcNewScene(device);
    RTCGeometry iarr = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_INSTANCE_ARRAY);
    if (!iarr) {
      rtcReleaseScene(top);
      rtcReleaseScene(child);
      return skipResult("instance array unsupported");
    }

    float xfm[16];
    setIdentityXfm(xfm);
    unsigned int objectIDs[1] = { 0u };
    RTCScene scenes[1] = { child };

    setTransformBuffer1(iarr, xfm);
    rtcSetSharedGeometryBuffer(iarr, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT, objectIDs, 0, sizeof(unsigned int), 1);
    rtcSetGeometryInstancedScenes(iarr, scenes, 1);
    rtcCommitGeometry(iarr);
    if (consumeDeviceError(device) != RTC_ERROR_NONE) {
      rtcReleaseGeometry(iarr);
      rtcReleaseScene(top);
      rtcReleaseScene(child);
      return failResult("failed to commit valid instance array");
    }

    float outXfm[12] = {};
    rtcGetGeometryTransformEx(iarr, 3, 0.5f, RTC_FORMAT_FLOAT3X4_COLUMN_MAJOR, outXfm);
    RTCError err = consumeDeviceError(device);

    rtcReleaseGeometry(iarr);
    rtcReleaseScene(top);
    rtcReleaseScene(child);
    if (err != RTC_ERROR_INVALID_ARGUMENT)
      return failResult("expected out-of-range instPrimID failure");
    return passResult("out-of-range instPrimID rejected");
  }

  static CaseResult issue10_line_segments_second_derivative_output(RTCDevice device)
  {
    RTCGeometry curve = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_FLAT_LINEAR_CURVE);
    if (!curve)
      return skipResult("flat linear curve unsupported");

    float* vertices = (float*)rtcSetNewGeometryBuffer(curve, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT4, 4 * sizeof(float), 2);
    unsigned int* indices = (unsigned int*)rtcSetNewGeometryBuffer(curve, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT, sizeof(unsigned int), 1);

    vertices[0] = 0.0f; vertices[1] = 0.0f; vertices[2] = 0.0f; vertices[3] = 0.1f;
    vertices[4] = 2.0f; vertices[5] = 0.0f; vertices[6] = 0.0f; vertices[7] = 0.1f;
    indices[0] = 0;

    rtcCommitGeometry(curve);
    RTCError err = consumeDeviceError(device);
    if (err != RTC_ERROR_NONE) {
      rtcReleaseGeometry(curve);
      return failResult("failed to commit test curve");
    }

    float P[4] = {};
    float dPdu[4] = { -777.0f, -777.0f, -777.0f, -777.0f };
    float dPdv[4] = { -777.0f, -777.0f, -777.0f, -777.0f };
    float ddPdudu[4] = { 999.0f, 999.0f, 999.0f, 999.0f };
    float ddPdvdv[4] = { 999.0f, 999.0f, 999.0f, 999.0f };
    float ddPdudv[4] = { 999.0f, 999.0f, 999.0f, 999.0f };

    rtcInterpolate2(curve, 0, 0.5f, 0.0f, RTC_BUFFER_TYPE_VERTEX, 0,
      P, dPdu, dPdv, ddPdudu, ddPdvdv, ddPdudv, 3);
    err = consumeDeviceError(device);
    rtcReleaseGeometry(curve);

    if (err != RTC_ERROR_NONE)
      return failResult("interpolation call failed");

    if (!(std::fabs(dPdu[0] - 2.0f) < 1.0e-4f && std::fabs(dPdu[1]) < 1.0e-4f && std::fabs(dPdu[2]) < 1.0e-4f))
      return failResult("dPdu was overwritten unexpectedly");

    if (!(std::fabs(ddPdudu[0]) < 1.0e-4f && std::fabs(ddPdudu[1]) < 1.0e-4f && std::fabs(ddPdudu[2]) < 1.0e-4f))
      return failResult("ddPdudu not populated as expected");

    return passResult("second derivative output uses correct pointer");
  }

  static CaseResult issue11_subdiv_verify_before_halfedge(RTCDevice device)
  {
    RTCGeometry subdiv = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_SUBDIVISION);
    if (!subdiv)
      return skipResult("subdivision geometry unsupported");

    float vertices[12] = {
      0.0f, 0.0f, 0.0f,
      1.0f, 0.0f, 0.0f,
      1.0f, 1.0f, 0.0f,
      0.0f, 1.0f, 0.0f
    };
    unsigned int indices[3] = { 0, 1, 2 };
    unsigned int faces[1] = { 4 };

    rtcSetSharedGeometryBuffer(subdiv, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, vertices, 0, 3 * sizeof(float), 4);
    rtcSetSharedGeometryBuffer(subdiv, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT, indices, 0, sizeof(unsigned int), 3);
    rtcSetSharedGeometryBuffer(subdiv, RTC_BUFFER_TYPE_FACE, 0, RTC_FORMAT_UINT, faces, 0, sizeof(unsigned int), 1);

    rtcCommitGeometry(subdiv);
    RTCError err = consumeDeviceError(device);
    rtcReleaseGeometry(subdiv);

    if (err == RTC_ERROR_NONE)
      return failResult("invalid subdiv topology unexpectedly committed");
    return passResult("invalid subdiv topology rejected without crash");
  }

  static CaseResult issue12_curve_index_overflow_validation(RTCDevice device)
  {
    RTCGeometry curve = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_ROUND_BSPLINE_CURVE);
    if (!curve)
      return skipResult("round bspline curve unsupported");

    float vertices[32] = {};
    for (int i = 0; i < 8; ++i) {
      vertices[4 * i + 0] = float(i);
      vertices[4 * i + 1] = 0.0f;
      vertices[4 * i + 2] = 0.0f;
      vertices[4 * i + 3] = 0.1f;
    }
    unsigned int indices[1] = { 0xFFFFFFFEu };

    rtcSetSharedGeometryBuffer(curve, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT4, vertices, 0, 4 * sizeof(float), 8);
    rtcSetSharedGeometryBuffer(curve, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT, indices, 0, sizeof(unsigned int), 1);
    rtcCommitGeometry(curve);

    RTCError err = consumeDeviceError(device);
    rtcReleaseGeometry(curve);

    if (err == RTC_ERROR_NONE)
      return failResult("overflowing curve index unexpectedly accepted");
    return passResult("overflowing curve index rejected");
  }

  static CaseResult issue13_motion_derivative_root_bound(RTCDevice device)
  {
    RTCScene child = createTriangleScene(device);
    RTCScene top = rtcNewScene(device);
    RTCGeometry inst = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_INSTANCE);
    if (!inst) {
      rtcReleaseScene(top);
      rtcReleaseScene(child);
      return skipResult("instance geometry unsupported");
    }

    rtcSetGeometryInstancedScene(inst, child);
    rtcSetGeometryTimeStepCount(inst, 2);

    RTCQuaternionDecomposition q0;
    RTCQuaternionDecomposition q1;
    rtcInitQuaternionDecomposition(&q0);
    rtcInitQuaternionDecomposition(&q1);
    rtcQuaternionDecompositionSetQuaternion(&q0, 0.7071067f, 0.0f, 0.7071067f, 0.0f);
    rtcQuaternionDecompositionSetQuaternion(&q1, 0.7071067f, 0.7071067f, 0.0f, 0.0f);
    rtcQuaternionDecompositionSetScale(&q0, 10.0f, 0.1f, 5.0f);
    rtcQuaternionDecompositionSetScale(&q1, 0.1f, 10.0f, 5.0f);
    rtcQuaternionDecompositionSetTranslation(&q0, -1000.0f, 1000.0f, 0.0f);
    rtcQuaternionDecompositionSetTranslation(&q1, 1000.0f, -1000.0f, 0.0f);

    rtcSetGeometryTransformQuaternion(inst, 0, &q0);
    rtcSetGeometryTransformQuaternion(inst, 1, &q1);

    rtcCommitGeometry(inst);
    rtcAttachGeometry(top, inst);
    rtcReleaseGeometry(inst);
    rtcCommitScene(top);

    RTCError err = consumeDeviceError(device);
    rtcReleaseScene(top);
    rtcReleaseScene(child);
    if (err != RTC_ERROR_NONE)
      return failResult("nonlinear bounds scene commit failed");
    return passResult("nonlinear derivative path completed safely");
  }

  static CaseResult issue14_grid_leaf_decode_guards(RTCDevice device)
  {
    RTCGeometry gridGeom = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_GRID);
    if (!gridGeom)
      return skipResult("grid geometry unsupported");

    RTCGrid grid;
    grid.startVertexID = 0;
    grid.stride = 4;
    grid.width = 4;
    grid.height = 4;

    float vertices[4 * 4 * 3];
    for (int y = 0; y < 4; ++y) {
      for (int x = 0; x < 4; ++x) {
        const int i = 3 * (y * 4 + x);
        vertices[i + 0] = float(x) * 0.25f;
        vertices[i + 1] = float(y) * 0.25f;
        vertices[i + 2] = 0.0f;
      }
    }

    rtcSetSharedGeometryBuffer(gridGeom, RTC_BUFFER_TYPE_GRID, 0, RTC_FORMAT_GRID, &grid, 0, sizeof(RTCGrid), 1);
    rtcSetSharedGeometryBuffer(gridGeom, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, vertices, 0, 3 * sizeof(float), 16);

    rtcCommitGeometry(gridGeom);
    if (consumeDeviceError(device) != RTC_ERROR_NONE) {
      rtcReleaseGeometry(gridGeom);
      return failResult("grid geometry commit failed");
    }

    RTCScene scene = rtcNewScene(device);
    rtcAttachGeometry(scene, gridGeom);
    rtcReleaseGeometry(gridGeom);
    rtcCommitScene(scene);

    RTCRayHit rayhit;
    std::memset(&rayhit, 0, sizeof(rayhit));
    rayhit.ray.org_x = 0.5f;
    rayhit.ray.org_y = 0.5f;
    rayhit.ray.org_z = -1.0f;
    rayhit.ray.dir_x = 0.0f;
    rayhit.ray.dir_y = 0.0f;
    rayhit.ray.dir_z = 1.0f;
    rayhit.ray.tnear = 0.0f;
    rayhit.ray.tfar = std::numeric_limits<float>::infinity();
    rayhit.ray.mask = 0xFFFFFFFFu;
    rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;

    rtcIntersect1(scene, &rayhit, nullptr);
    RTCError err = consumeDeviceError(device);
    rtcReleaseScene(scene);

    if (err != RTC_ERROR_NONE)
      return failResult("grid traversal failed");
    return passResult("grid traversal completed safely");
  }

  static CaseResult issue15_instance_array_time_clamp(RTCDevice device)
  {
    RTCScene child = createTriangleScene(device);
    RTCScene top = rtcNewScene(device);
    RTCGeometry iarr = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_INSTANCE_ARRAY);
    if (!iarr) {
      rtcReleaseScene(top);
      rtcReleaseScene(child);
      return skipResult("instance array unsupported");
    }

    rtcSetGeometryTimeStepCount(iarr, 2);
    rtcSetGeometryTimeRange(iarr, 0.75f, 0.25f);

    float xfm0[16];
    float xfm1[16];
    setIdentityXfm(xfm0);
    setIdentityXfm(xfm1);
    xfm1[12] = 0.5f;

    unsigned int objectIDs[1] = { 0u };
    RTCScene scenes[1] = { child };

    rtcSetSharedGeometryBuffer(iarr, RTC_BUFFER_TYPE_TRANSFORM, 0, RTC_FORMAT_FLOAT4X4_COLUMN_MAJOR, xfm0, 0, 16 * sizeof(float), 1);
    rtcSetSharedGeometryBuffer(iarr, RTC_BUFFER_TYPE_TRANSFORM, 1, RTC_FORMAT_FLOAT4X4_COLUMN_MAJOR, xfm1, 0, 16 * sizeof(float), 1);
    rtcSetSharedGeometryBuffer(iarr, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT, objectIDs, 0, sizeof(unsigned int), 1);
    rtcSetGeometryInstancedScenes(iarr, scenes, 1);

    rtcCommitGeometry(iarr);
    rtcAttachGeometry(top, iarr);
    rtcReleaseGeometry(iarr);
    rtcCommitScene(top);

    RTCError err = consumeDeviceError(device);
    if (!errorIsAccepted(err)) {
      rtcReleaseScene(top);
      rtcReleaseScene(child);
      return failResult("unexpected API error");
    }

    if (err == RTC_ERROR_NONE) {
      RTCRayHit rayhit;
      std::memset(&rayhit, 0, sizeof(rayhit));
      rayhit.ray.org_x = 0.2f;
      rayhit.ray.org_y = 0.2f;
      rayhit.ray.org_z = -1.0f;
      rayhit.ray.dir_x = 0.0f;
      rayhit.ray.dir_y = 0.0f;
      rayhit.ray.dir_z = 1.0f;
      rayhit.ray.tnear = 0.0f;
      rayhit.ray.tfar = std::numeric_limits<float>::infinity();
      rayhit.ray.time = 2.0f;
      rayhit.ray.mask = 0xFFFFFFFFu;
      rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;

      rtcIntersect1(top, &rayhit, nullptr);
      err = consumeDeviceError(device);
      if (err != RTC_ERROR_NONE) {
        rtcReleaseScene(top);
        rtcReleaseScene(child);
        return failResult("instance array traversal failed");
      }
    }

    rtcReleaseScene(top);
    rtcReleaseScene(child);
    return passResult("instance array time range handled safely");
  }

  constexpr unsigned int max_branching_factor = 8;

  struct MortonNode
  {
    MortonNode()
    {
      for (unsigned int i = 0; i < max_branching_factor; ++i)
        children[i] = nullptr;
    }

    MortonNode* children[max_branching_factor];
  };

  static bool mortonBuildProgress(void* /*userPtr*/, double /*f*/)
  {
    return true;
  }

  static void* createMortonNode(RTCThreadLocalAllocator alloc, unsigned int childCount, void* /*userPtr*/)
  {
    assert(childCount <= max_branching_factor);
    if (childCount > max_branching_factor)
      return nullptr;

    MortonNode* node = (MortonNode*)rtcThreadLocalAlloc(alloc, sizeof(MortonNode), 16);
    new (node) MortonNode();
    return node;
  }

  static void setMortonNodeChildren(void* nodePtr, void** children, unsigned int childCount, void* /*userPtr*/)
  {
    assert(childCount <= max_branching_factor);
    if (childCount > max_branching_factor)
      return;

    MortonNode* node = (MortonNode*)nodePtr;
    for (unsigned int i = 0; i < childCount; ++i)
      node->children[i] = (MortonNode*)children[i];
  }

  static void setMortonNodeBounds(void* /*nodePtr*/, const RTCBounds** /*bounds*/, unsigned int childCount, void* /*userPtr*/)
  {
    assert(childCount <= max_branching_factor);
  }

  static void* createMortonLeaf(RTCThreadLocalAllocator alloc,
                                const RTCBuildPrimitive* /*prims*/,
                                size_t /*primCount*/,
                                void* /*userPtr*/)
  {
    MortonNode* node = (MortonNode*)rtcThreadLocalAlloc(alloc, sizeof(MortonNode), 16);
    new (node) MortonNode();
    return node;
  }

  static std::vector<RTCBuildPrimitive> makeMortonGridPrimitives(size_t primitiveCount)
  {
    std::vector<RTCBuildPrimitive> prims(primitiveCount);
    for (size_t i = 0; i < primitiveCount; ++i)
    {
      const float x = float(i % 32);
      const float y = float((i / 32) % 32);
      RTCBuildPrimitive& p = prims[i];
      p = {};
      p.lower_x = x * 2.0f;
      p.lower_y = y * 2.0f;
      p.upper_x = p.lower_x + 0.5f;
      p.upper_y = p.lower_y + 0.5f;
      p.upper_z = 0.5f;
      p.geomID = 0;
      p.primID = (unsigned int)i;
    }
    return prims;
  }

  static bool mortonBuilderRejectsOversizedBranchingFactor(RTCDevice device, unsigned int maxBranchingFactor)
  {
    RTCBVH bvh = rtcNewBVH(device);
    if (!bvh)
      return false;

    std::vector<RTCBuildPrimitive> prims = makeMortonGridPrimitives(1024);
    RTCBuildArguments args = rtcDefaultBuildArguments();
    args.byteSize = sizeof(args);
    args.buildQuality = RTC_BUILD_QUALITY_LOW;
    args.maxBranchingFactor = maxBranchingFactor;
    args.maxDepth = 1024;
    args.minLeafSize = 1;
    args.maxLeafSize = 1;
    args.bvh = bvh;
    args.primitives = prims.data();
    args.primitiveCount = prims.size();
    args.primitiveArrayCapacity = prims.size();
    args.createNode = createMortonNode;
    args.setNodeChildren = setMortonNodeChildren;
    args.setNodeBounds = setMortonNodeBounds;
    args.createLeaf = createMortonLeaf;
    args.buildProgress = mortonBuildProgress;

    rtcGetDeviceError(device);
    void* root = rtcBuildBVH(&args);
    const RTCError error = rtcGetDeviceError(device);
    rtcReleaseBVH(bvh);
    return root == nullptr && error == RTC_ERROR_INVALID_ARGUMENT;
  }

  static CaseResult morton_builder_clamp(RTCDevice device)
  {
    if (!mortonBuilderRejectsOversizedBranchingFactor(device, 64))
      return failResult("maxBranchingFactor=64 was not rejected");
    if (!mortonBuilderRejectsOversizedBranchingFactor(device, std::numeric_limits<unsigned int>::max()))
      return failResult("maxBranchingFactor=UINT_MAX was not rejected");
    return passResult("oversized maxBranchingFactor values are rejected");
  }

  struct TestCase
  {
    const char* name;
    CaseResult (*fn)(RTCDevice);
  };
}

int main()
{
  RTCDevice device = rtcNewDevice(nullptr);
  if (!device) {
    std::printf("FAIL create_device\n");
    return 1;
  }

  const TestCase tests[] = {
    { "Morton-builder-clamp", morton_builder_clamp },
    { "Issue-01", issue01_time_segment_range_clamp },
    { "Issue-02", issue02_lbbox_nan_range },
    { "Issue-03", issue03_lbbox_extreme_range },
    { "Issue-04", issue04_instance_bound_segment_guard },
    { "Issue-05", issue05_instance_nonlinear_bounds },
    { "Issue-06", issue06_mb_builder_range_check },
    { "Issue-07", issue07_user_bounds_time_range_validation },
    { "Issue-08", issue08_instance_array_object_id_validation },
    { "Issue-09", issue09_instance_array_transform_oob },
    { "Issue-10", issue10_line_segments_second_derivative_output },
    { "Issue-11", issue11_subdiv_verify_before_halfedge },
    { "Issue-12", issue12_curve_index_overflow_validation },
    { "Issue-13", issue13_motion_derivative_root_bound },
    { "Issue-14", issue14_grid_leaf_decode_guards },
    { "Issue-15", issue15_instance_array_time_clamp }
  };

  int failed = 0;
  int skipped = 0;

  for (const TestCase& tc : tests) {
    CaseResult r = tc.fn(device);
    if (r.skip) {
      ++skipped;
      std::printf("SKIP %s: %s\n", tc.name, r.message.c_str());
      continue;
    }

    if (r.pass) {
      std::printf("PASS %s: %s\n", tc.name, r.message.c_str());
    } else {
      ++failed;
      std::printf("FAIL %s: %s\n", tc.name, r.message.c_str());
    }
  }

  rtcReleaseDevice(device);

  std::printf("SUMMARY total=%u passed=%u failed=%d skipped=%d\n",
    (unsigned)(sizeof(tests) / sizeof(tests[0])),
    (unsigned)(sizeof(tests) / sizeof(tests[0])) - (unsigned)failed - (unsigned)skipped,
    failed,
    skipped);

  return failed == 0 ? 0 : 1;
}
