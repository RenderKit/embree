// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "host_device_memory_device.h"

#include <stdlib.h>

namespace embree {

/* all features required by this tutorial */
#define FEATURE_MASK \
  RTC_FEATURE_FLAG_TRIANGLE | \
  RTC_FEATURE_FLAG_MOTION_BLUR

RTCScene g_scene = nullptr;
TutorialData data;

#define GEOMETRY_MOTION_BLUR

/* adds a cube to the scene with explicitly managed host/device memory */
unsigned int addCubeHostDevice (RTCScene scene, Vec3fa d)
{
  /* create a triangulated cube with 12 triangles and 8 vertices */
  RTCGeometry mesh = rtcNewGeometry(g_device, RTC_GEOMETRY_TYPE_TRIANGLE);

  /* set vertices and vertex colors */
  Vertex* hVertices0;
  Vertex* dVertices0;
  rtcSetNewGeometryBufferHostDevice(mesh, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, sizeof(Vertex), 8, (void **)&hVertices0, (void **)&dVertices0);
  hVertices0[0].x = -1 + d.x; hVertices0[0].y = -1 + d.y; hVertices0[0].z = -1 + d.z;
  hVertices0[1].x = -1 + d.x; hVertices0[1].y = -1 + d.y; hVertices0[1].z = +1 + d.z;
  hVertices0[2].x = -1 + d.x; hVertices0[2].y = +1 + d.y; hVertices0[2].z = -1 + d.z;
  hVertices0[3].x = -1 + d.x; hVertices0[3].y = +1 + d.y; hVertices0[3].z = +1 + d.z;
  hVertices0[4].x = +1 + d.x; hVertices0[4].y = -1 + d.y; hVertices0[4].z = -1 + d.z;
  hVertices0[5].x = +1 + d.x; hVertices0[5].y = -1 + d.y; hVertices0[5].z = +1 + d.z;
  hVertices0[6].x = +1 + d.x; hVertices0[6].y = +1 + d.y; hVertices0[6].z = -1 + d.z;
  hVertices0[7].x = +1 + d.x; hVertices0[7].y = +1 + d.y; hVertices0[7].z = +1 + d.z;

#if defined(GEOMETRY_MOTION_BLUR)
  rtcSetGeometryTimeStepCount(mesh, 2);
  Vertex* hVertices1;
  Vertex* dVertices1;
  rtcSetNewGeometryBufferHostDevice(mesh, RTC_BUFFER_TYPE_VERTEX, 1, RTC_FORMAT_FLOAT3, sizeof(Vertex), 8, (void **)&hVertices1, (void **)&dVertices1);
  hVertices1[0].x = -1 + d.x; hVertices1[0].y = -1 + 1.f + d.y; hVertices1[0].z = -1 + d.z;
  hVertices1[1].x = -1 + d.x; hVertices1[1].y = -1 + 1.f + d.y; hVertices1[1].z = +1 + d.z;
  hVertices1[2].x = -1 + d.x; hVertices1[2].y = +1 + 1.f + d.y; hVertices1[2].z = -1 + d.z;
  hVertices1[3].x = -1 + d.x; hVertices1[3].y = +1 + 1.f + d.y; hVertices1[3].z = +1 + d.z;
  hVertices1[4].x = +1 + d.x; hVertices1[4].y = -1 + 1.f + d.y; hVertices1[4].z = -1 + d.z;
  hVertices1[5].x = +1 + d.x; hVertices1[5].y = -1 + 1.f + d.y; hVertices1[5].z = +1 + d.z;
  hVertices1[6].x = +1 + d.x; hVertices1[6].y = +1 + 1.f + d.y; hVertices1[6].z = -1 + d.z;
  hVertices1[7].x = +1 + d.x; hVertices1[7].y = +1 + 1.f + d.y; hVertices1[7].z = +1 + d.z;
#endif

  /* set triangles and face colors */
  Triangle* fillTriangles;
  Triangle* hTriangles = fillTriangles = data.hTrianglesHostDevice = (Triangle*) alignedMalloc(12*sizeof(Triangle), 16);
  Triangle* dTriangles = nullptr;
#if defined(EMBREE_SYCL_TUTORIAL)
  dTriangles = fillTriangles = data.dTrianglesHostDevice = sycl::aligned_alloc_device<Triangle>(16, 12, *global_gpu_device, *global_gpu_context);
  global_gpu_queue->single_task([=](){
#endif
    fillTriangles[ 0].v0 = 0; fillTriangles[ 0].v1 = 1; fillTriangles[ 0].v2 = 2;
    fillTriangles[ 1].v0 = 1; fillTriangles[ 1].v1 = 3; fillTriangles[ 1].v2 = 2;
    fillTriangles[ 2].v0 = 4; fillTriangles[ 2].v1 = 6; fillTriangles[ 2].v2 = 5;
    fillTriangles[ 3].v0 = 5; fillTriangles[ 3].v1 = 6; fillTriangles[ 3].v2 = 7;
    fillTriangles[ 4].v0 = 0; fillTriangles[ 4].v1 = 4; fillTriangles[ 4].v2 = 1;
    fillTriangles[ 5].v0 = 1; fillTriangles[ 5].v1 = 4; fillTriangles[ 5].v2 = 5;
    fillTriangles[ 6].v0 = 2; fillTriangles[ 6].v1 = 3; fillTriangles[ 6].v2 = 6;
    fillTriangles[ 7].v0 = 3; fillTriangles[ 7].v1 = 7; fillTriangles[ 7].v2 = 6;
    fillTriangles[ 8].v0 = 0; fillTriangles[ 8].v1 = 2; fillTriangles[ 8].v2 = 4;
    fillTriangles[ 9].v0 = 2; fillTriangles[ 9].v1 = 6; fillTriangles[ 9].v2 = 4;
    fillTriangles[10].v0 = 1; fillTriangles[10].v1 = 5; fillTriangles[10].v2 = 3;
    fillTriangles[11].v0 = 3; fillTriangles[11].v1 = 5; fillTriangles[11].v2 = 7;
#if defined(EMBREE_SYCL_TUTORIAL)
  });
  // needs to be copied back into host memory, because 
  // - rtcCommitGeometry will copy from host to device again
  // - bvh builde uses host meory internally
  global_gpu_queue->memcpy(hTriangles, dTriangles, 12 * sizeof(Triangle)).wait();
#endif

  rtcSetSharedGeometryBufferHostDevice(mesh, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, hTriangles, dTriangles, 0, sizeof(Triangle), 12);
  rtcCommitGeometry(mesh);

  unsigned int geomID = rtcAttachGeometry(scene,mesh);
  rtcReleaseGeometry(mesh);
  return geomID;
}


/* adds a cube to the scene with USM shared memory */
unsigned int addCubeShared (RTCScene scene, Vec3fa d)
{
  /* create a triangulated cube with 12 triangles and 8 vertices */
  RTCGeometry mesh = rtcNewGeometry(g_device, RTC_GEOMETRY_TYPE_TRIANGLE);

  /* set vertices and vertex colors */
  Vertex* vertices0 = (Vertex*)rtcSetNewGeometryBuffer(mesh, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, sizeof(Vertex), 8);
  vertices0[0].x = -1 + d.x; vertices0[0].y = -1 + d.y; vertices0[0].z = -1 + d.z;
  vertices0[1].x = -1 + d.x; vertices0[1].y = -1 + d.y; vertices0[1].z = +1 + d.z;
  vertices0[2].x = -1 + d.x; vertices0[2].y = +1 + d.y; vertices0[2].z = -1 + d.z;
  vertices0[3].x = -1 + d.x; vertices0[3].y = +1 + d.y; vertices0[3].z = +1 + d.z;
  vertices0[4].x = +1 + d.x; vertices0[4].y = -1 + d.y; vertices0[4].z = -1 + d.z;
  vertices0[5].x = +1 + d.x; vertices0[5].y = -1 + d.y; vertices0[5].z = +1 + d.z;
  vertices0[6].x = +1 + d.x; vertices0[6].y = +1 + d.y; vertices0[6].z = -1 + d.z;
  vertices0[7].x = +1 + d.x; vertices0[7].y = +1 + d.y; vertices0[7].z = +1 + d.z;

#if defined(GEOMETRY_MOTION_BLUR)
  rtcSetGeometryTimeStepCount(mesh, 2);
  Vertex* vertices1 = (Vertex*)rtcSetNewGeometryBuffer(mesh, RTC_BUFFER_TYPE_VERTEX, 1, RTC_FORMAT_FLOAT3, sizeof(Vertex), 8);
  vertices1[0].x = -1 + d.x; vertices1[0].y = -1 + 1 + d.y; vertices1[0].z = -1 + d.z;
  vertices1[1].x = -1 + d.x; vertices1[1].y = -1 + 1 + d.y; vertices1[1].z = +1 + d.z;
  vertices1[2].x = -1 + d.x; vertices1[2].y = +1 + 1 + d.y; vertices1[2].z = -1 + d.z;
  vertices1[3].x = -1 + d.x; vertices1[3].y = +1 + 1 + d.y; vertices1[3].z = +1 + d.z;
  vertices1[4].x = +1 + d.x; vertices1[4].y = -1 + 1 + d.y; vertices1[4].z = -1 + d.z;
  vertices1[5].x = +1 + d.x; vertices1[5].y = -1 + 1 + d.y; vertices1[5].z = +1 + d.z;
  vertices1[6].x = +1 + d.x; vertices1[6].y = +1 + 1 + d.y; vertices1[6].z = -1 + d.z;
  vertices1[7].x = +1 + d.x; vertices1[7].y = +1 + 1 + d.y; vertices1[7].z = +1 + d.z;
#endif

  /* set triangles and face colors */
  Triangle* triangles = data.trianglesShared = (Triangle*) alignedUSMMalloc(12*sizeof(Triangle), 16);
#if defined(EMBREE_SYCL_TUTORIAL)
  auto sycl_event = global_gpu_queue->single_task([=](){
#endif
    triangles[ 0].v0 = 0; triangles[ 0].v1 = 1; triangles[ 0].v2 = 2;
    triangles[ 1].v0 = 1; triangles[ 1].v1 = 3; triangles[ 1].v2 = 2;
    triangles[ 2].v0 = 4; triangles[ 2].v1 = 6; triangles[ 2].v2 = 5;
    triangles[ 3].v0 = 5; triangles[ 3].v1 = 6; triangles[ 3].v2 = 7;
    triangles[ 4].v0 = 0; triangles[ 4].v1 = 4; triangles[ 4].v2 = 1;
    triangles[ 5].v0 = 1; triangles[ 5].v1 = 4; triangles[ 5].v2 = 5;
    triangles[ 6].v0 = 2; triangles[ 6].v1 = 3; triangles[ 6].v2 = 6;
    triangles[ 7].v0 = 3; triangles[ 7].v1 = 7; triangles[ 7].v2 = 6;
    triangles[ 8].v0 = 0; triangles[ 8].v1 = 2; triangles[ 8].v2 = 4;
    triangles[ 9].v0 = 2; triangles[ 9].v1 = 6; triangles[ 9].v2 = 4;
    triangles[10].v0 = 1; triangles[10].v1 = 5; triangles[10].v2 = 3;
    triangles[11].v0 = 3; triangles[11].v1 = 5; triangles[11].v2 = 7;
#if defined(EMBREE_SYCL_TUTORIAL)
  });
  // rtcCommit uses different queue internally, so we have to wait here
  sycl_event.wait();
#endif

  rtcSetSharedGeometryBuffer(mesh, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, triangles, 0, sizeof(Triangle), 12);

  rtcCommitGeometry(mesh);

  unsigned int geomID = rtcAttachGeometry(scene,mesh);
  rtcReleaseGeometry(mesh);
  return geomID;
}


/* adds a cube to the scene with explicitly managed host/device memory through the RTCBuffer interface */
unsigned int addCubeBufferHostDevice (RTCScene scene, Vec3fa d)
{
  /* create a triangulated cube with 12 triangles and 8 vertices */
  RTCGeometry mesh = rtcNewGeometry(g_device, RTC_GEOMETRY_TYPE_TRIANGLE);

  /* set vertices and vertex colors */
  RTCBuffer vertexBuffer0 = rtcNewBufferHostDevice(g_device, 8 * sizeof(Vertex));
  rtcSetGeometryBuffer(mesh, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, vertexBuffer0, 0, sizeof(Vertex), 8);
  Vertex* hVertices0 = (Vertex*)rtcGetBufferData(vertexBuffer0);
  hVertices0[0].x = -1 + d.x; hVertices0[0].y = -1 + d.y; hVertices0[0].z = -1 + d.z;
  hVertices0[1].x = -1 + d.x; hVertices0[1].y = -1 + d.y; hVertices0[1].z = +1 + d.z;
  hVertices0[2].x = -1 + d.x; hVertices0[2].y = +1 + d.y; hVertices0[2].z = -1 + d.z;
  hVertices0[3].x = -1 + d.x; hVertices0[3].y = +1 + d.y; hVertices0[3].z = +1 + d.z;
  hVertices0[4].x = +1 + d.x; hVertices0[4].y = -1 + d.y; hVertices0[4].z = -1 + d.z;
  hVertices0[5].x = +1 + d.x; hVertices0[5].y = -1 + d.y; hVertices0[5].z = +1 + d.z;
  hVertices0[6].x = +1 + d.x; hVertices0[6].y = +1 + d.y; hVertices0[6].z = -1 + d.z;
  hVertices0[7].x = +1 + d.x; hVertices0[7].y = +1 + d.y; hVertices0[7].z = +1 + d.z;
  rtcCommitBuffer(vertexBuffer0);
  rtcReleaseBuffer(vertexBuffer0);

#if defined(GEOMETRY_MOTION_BLUR)
  rtcSetGeometryTimeStepCount(mesh, 2);
  RTCBuffer vertexBuffer1 = rtcNewBufferHostDevice(g_device, 8 * sizeof(Vertex));
  rtcSetGeometryBuffer(mesh, RTC_BUFFER_TYPE_VERTEX, 1, RTC_FORMAT_FLOAT3, vertexBuffer1, 0, sizeof(Vertex), 8);
  Vertex* hVertices1 = (Vertex*)rtcGetBufferData(vertexBuffer1);
  hVertices1[0].x = -1 + d.x; hVertices1[0].y = -1 + 1.f + d.y; hVertices1[0].z = -1 + d.z;
  hVertices1[1].x = -1 + d.x; hVertices1[1].y = -1 + 1.f + d.y; hVertices1[1].z = +1 + d.z;
  hVertices1[2].x = -1 + d.x; hVertices1[2].y = +1 + 1.f + d.y; hVertices1[2].z = -1 + d.z;
  hVertices1[3].x = -1 + d.x; hVertices1[3].y = +1 + 1.f + d.y; hVertices1[3].z = +1 + d.z;
  hVertices1[4].x = +1 + d.x; hVertices1[4].y = -1 + 1.f + d.y; hVertices1[4].z = -1 + d.z;
  hVertices1[5].x = +1 + d.x; hVertices1[5].y = -1 + 1.f + d.y; hVertices1[5].z = +1 + d.z;
  hVertices1[6].x = +1 + d.x; hVertices1[6].y = +1 + 1.f + d.y; hVertices1[6].z = -1 + d.z;
  hVertices1[7].x = +1 + d.x; hVertices1[7].y = +1 + 1.f + d.y; hVertices1[7].z = +1 + d.z;
  rtcCommitBuffer(vertexBuffer1);
  rtcReleaseBuffer(vertexBuffer1);
#endif

  /* set triangles and face colors */

  Triangle* hTriangles = data.hTrianglesBufferHostDevice = (Triangle*) alignedMalloc(12*sizeof(Triangle), 16);
  RTCBuffer triangleBuffer = rtcNewSharedBufferHostDevice(g_device, hTriangles, 12*sizeof(Triangle));
  rtcSetGeometryBuffer(mesh, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, triangleBuffer, 0, sizeof(Triangle), 12);
  hTriangles[ 0].v0 = 0; hTriangles[ 0].v1 = 1; hTriangles[ 0].v2 = 2;
  hTriangles[ 1].v0 = 1; hTriangles[ 1].v1 = 3; hTriangles[ 1].v2 = 2;
  hTriangles[ 2].v0 = 4; hTriangles[ 2].v1 = 6; hTriangles[ 2].v2 = 5;
  hTriangles[ 3].v0 = 5; hTriangles[ 3].v1 = 6; hTriangles[ 3].v2 = 7;
  hTriangles[ 4].v0 = 0; hTriangles[ 4].v1 = 4; hTriangles[ 4].v2 = 1;
  hTriangles[ 5].v0 = 1; hTriangles[ 5].v1 = 4; hTriangles[ 5].v2 = 5;
  hTriangles[ 6].v0 = 2; hTriangles[ 6].v1 = 3; hTriangles[ 6].v2 = 6;
  hTriangles[ 7].v0 = 3; hTriangles[ 7].v1 = 7; hTriangles[ 7].v2 = 6;
  hTriangles[ 8].v0 = 0; hTriangles[ 8].v1 = 2; hTriangles[ 8].v2 = 4;
  hTriangles[ 9].v0 = 2; hTriangles[ 9].v1 = 6; hTriangles[ 9].v2 = 4;
  hTriangles[10].v0 = 1; hTriangles[10].v1 = 5; hTriangles[10].v2 = 3;
  hTriangles[11].v0 = 3; hTriangles[11].v1 = 5; hTriangles[11].v2 = 7;
  rtcCommitBuffer(triangleBuffer);
  rtcReleaseBuffer(triangleBuffer);

  rtcCommitGeometry(mesh);

  unsigned int geomID = rtcAttachGeometry(scene,mesh);
  rtcReleaseGeometry(mesh);
  return geomID;
}


/* adds a cube to the scene with explicitly managed host/device memory through the RTCBuffer interface */
unsigned int addCubeBufferShared (RTCScene scene, Vec3fa d)
{
  /* create a triangulated cube with 12 triangles and 8 vertices */
  RTCGeometry mesh = rtcNewGeometry(g_device, RTC_GEOMETRY_TYPE_TRIANGLE);

  /* set vertices and vertex colors */
  RTCBuffer vertexBuffer0 = rtcNewBuffer(g_device, 8 * sizeof(Vertex));
  rtcSetGeometryBuffer(mesh, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, vertexBuffer0, 0, sizeof(Vertex), 8);
  Vertex* hVertices0 = (Vertex*)rtcGetBufferData(vertexBuffer0);
  hVertices0[0].x = -1 + d.x; hVertices0[0].y = -1 + d.y; hVertices0[0].z = -1 + d.z;
  hVertices0[1].x = -1 + d.x; hVertices0[1].y = -1 + d.y; hVertices0[1].z = +1 + d.z;
  hVertices0[2].x = -1 + d.x; hVertices0[2].y = +1 + d.y; hVertices0[2].z = -1 + d.z;
  hVertices0[3].x = -1 + d.x; hVertices0[3].y = +1 + d.y; hVertices0[3].z = +1 + d.z;
  hVertices0[4].x = +1 + d.x; hVertices0[4].y = -1 + d.y; hVertices0[4].z = -1 + d.z;
  hVertices0[5].x = +1 + d.x; hVertices0[5].y = -1 + d.y; hVertices0[5].z = +1 + d.z;
  hVertices0[6].x = +1 + d.x; hVertices0[6].y = +1 + d.y; hVertices0[6].z = -1 + d.z;
  hVertices0[7].x = +1 + d.x; hVertices0[7].y = +1 + d.y; hVertices0[7].z = +1 + d.z;
  rtcReleaseBuffer(vertexBuffer0);

#if defined(GEOMETRY_MOTION_BLUR)
  rtcSetGeometryTimeStepCount(mesh, 2);
  RTCBuffer vertexBuffer1 = rtcNewBuffer(g_device, 8 * sizeof(Vertex));
  rtcSetGeometryBuffer(mesh, RTC_BUFFER_TYPE_VERTEX, 1, RTC_FORMAT_FLOAT3, vertexBuffer1, 0, sizeof(Vertex), 8);
  Vertex* hVertices1 = (Vertex*)rtcGetBufferData(vertexBuffer1);
  hVertices1[0].x = -1 + d.x; hVertices1[0].y = -1 + 1.f + d.y; hVertices1[0].z = -1 + d.z;
  hVertices1[1].x = -1 + d.x; hVertices1[1].y = -1 + 1.f + d.y; hVertices1[1].z = +1 + d.z;
  hVertices1[2].x = -1 + d.x; hVertices1[2].y = +1 + 1.f + d.y; hVertices1[2].z = -1 + d.z;
  hVertices1[3].x = -1 + d.x; hVertices1[3].y = +1 + 1.f + d.y; hVertices1[3].z = +1 + d.z;
  hVertices1[4].x = +1 + d.x; hVertices1[4].y = -1 + 1.f + d.y; hVertices1[4].z = -1 + d.z;
  hVertices1[5].x = +1 + d.x; hVertices1[5].y = -1 + 1.f + d.y; hVertices1[5].z = +1 + d.z;
  hVertices1[6].x = +1 + d.x; hVertices1[6].y = +1 + 1.f + d.y; hVertices1[6].z = -1 + d.z;
  hVertices1[7].x = +1 + d.x; hVertices1[7].y = +1 + 1.f + d.y; hVertices1[7].z = +1 + d.z;
  rtcReleaseBuffer(vertexBuffer1);
#endif

  /* set triangles */
  Triangle* triangles = data.trianglesBufferShared = (Triangle*) alignedUSMMalloc(12*sizeof(Triangle), 16);
#if defined(EMBREE_SYCL_TUTORIAL)
  global_gpu_queue->single_task([=](){
#endif
    triangles[ 0].v0 = 0; triangles[ 0].v1 = 1; triangles[ 0].v2 = 2;
    triangles[ 1].v0 = 1; triangles[ 1].v1 = 3; triangles[ 1].v2 = 2;
    triangles[ 2].v0 = 4; triangles[ 2].v1 = 6; triangles[ 2].v2 = 5;
    triangles[ 3].v0 = 5; triangles[ 3].v1 = 6; triangles[ 3].v2 = 7;
    triangles[ 4].v0 = 0; triangles[ 4].v1 = 4; triangles[ 4].v2 = 1;
    triangles[ 5].v0 = 1; triangles[ 5].v1 = 4; triangles[ 5].v2 = 5;
    triangles[ 6].v0 = 2; triangles[ 6].v1 = 3; triangles[ 6].v2 = 6;
    triangles[ 7].v0 = 3; triangles[ 7].v1 = 7; triangles[ 7].v2 = 6;
    triangles[ 8].v0 = 0; triangles[ 8].v1 = 2; triangles[ 8].v2 = 4;
    triangles[ 9].v0 = 2; triangles[ 9].v1 = 6; triangles[ 9].v2 = 4;
    triangles[10].v0 = 1; triangles[10].v1 = 5; triangles[10].v2 = 3;
    triangles[11].v0 = 3; triangles[11].v1 = 5; triangles[11].v2 = 7;
#if defined(EMBREE_SYCL_TUTORIAL)
  }).wait();
#endif

  RTCBuffer triangleBuffer = rtcNewSharedBuffer(g_device, triangles, 12 * sizeof(Triangle));
  rtcSetGeometryBuffer(mesh, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, triangleBuffer, 0, sizeof(Triangle), 12);
  rtcCommitBuffer(triangleBuffer); // optional
  rtcReleaseBuffer(triangleBuffer);

  rtcCommitGeometry(mesh);

  unsigned int geomID = rtcAttachGeometry(scene,mesh);
  rtcReleaseGeometry(mesh);
  return geomID;
}

/* adds a ground plane to the scene */
unsigned int addGroundPlane (RTCScene scene_i)
{
  /* create a triangulated plane with 2 triangles and 4 vertices */
  RTCGeometry mesh = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_TRIANGLE);

  /* set vertices */
  Vertex* vertices = (Vertex*) rtcSetNewGeometryBuffer(mesh,RTC_BUFFER_TYPE_VERTEX,0,RTC_FORMAT_FLOAT3,sizeof(Vertex),4);
  vertices[0].x = -10; vertices[0].y = -2; vertices[0].z = -10;
  vertices[1].x = -10; vertices[1].y = -2; vertices[1].z = +10;
  vertices[2].x = +10; vertices[2].y = -2; vertices[2].z = -10;
  vertices[3].x = +10; vertices[3].y = -2; vertices[3].z = +10;

  /* set triangles */
  Triangle* triangles = (Triangle*) rtcSetNewGeometryBuffer(mesh,RTC_BUFFER_TYPE_INDEX,0,RTC_FORMAT_UINT3,sizeof(Triangle),2);
  triangles[0].v0 = 0; triangles[0].v1 = 1; triangles[0].v2 = 2;
  triangles[1].v0 = 1; triangles[1].v1 = 3; triangles[1].v2 = 2;

  rtcCommitGeometry(mesh);
  unsigned int geomID = rtcAttachGeometry(scene_i,mesh);
  rtcReleaseGeometry(mesh);
  return geomID;
}

/* called by the C++ code for initialization */
extern "C" void device_init (char* cfg)
{ 
  /* create scene */
  TutorialData_Constructor(&data);

  g_scene = data.g_scene = rtcNewScene(g_device);

  /* create face and vertex color arrays */
  data.face_colors = (Vec3fa*) alignedUSMMalloc((12)*sizeof(Vec3fa),16);
  data.vertex_colors = (Vec3fa*) alignedUSMMalloc((8)*sizeof(Vec3fa),16);

  data.vertex_colors[0] = Vec3fa(0, 0, 0);
  data.vertex_colors[1] = Vec3fa(0, 0, 1);
  data.vertex_colors[2] = Vec3fa(0, 1, 0);
  data.vertex_colors[3] = Vec3fa(0, 1, 1);
  data.vertex_colors[4] = Vec3fa(1, 0, 0);
  data.vertex_colors[5] = Vec3fa(1, 0, 1);
  data.vertex_colors[6] = Vec3fa(1, 1, 0);
  data.vertex_colors[7] = Vec3fa(1, 1, 1);
  data.face_colors[ 0] = Vec3fa(1, 0, 0);
  data.face_colors[ 1] = Vec3fa(1, 0, 0);
  data.face_colors[ 2] = Vec3fa(0, 1, 0);
  data.face_colors[ 3] = Vec3fa(0, 1, 0);
  data.face_colors[ 4] = Vec3fa(0.5f);
  data.face_colors[ 5] = Vec3fa(0.5f);
  data.face_colors[ 6] = Vec3fa(1.0f);
  data.face_colors[ 7] = Vec3fa(1.0f);
  data.face_colors[ 8] = Vec3fa(0, 0, 1);
  data.face_colors[ 9] = Vec3fa(0, 0, 1);
  data.face_colors[10] = Vec3fa(1, 1, 0);
  data.face_colors[11] = Vec3fa(1, 1, 0);

  /* add cubes */
  addCubeHostDevice       (g_scene, Vec3fa(-3.f, 0.f, -3.f));
  addCubeShared           (g_scene, Vec3fa(-3.f, 0.f,  3.f));
  addCubeBufferHostDevice (g_scene, Vec3fa( 3.f, 0.f, -3.f));
  addCubeBufferShared     (g_scene, Vec3fa( 3.f, 0.f,  3.f));

  /* add ground plane */
  addGroundPlane(data.g_scene);

#if defined(EMBREE_SYCL_SUPPORT) && defined(EMBREE_SYCL_TUTORIAL)
  // we fill data on GPU and copy to host. we have to ensure
  // that the data is on host before building the scene
  // with rtcCommitScene
  global_gpu_queue->wait_and_throw();

  rtcCommitSceneWithQueue (data.g_scene, *global_gpu_queue);
#else
  rtcCommitScene (data.g_scene);
#endif
  data.g_traversable = rtcGetSceneTraversable(data.g_scene);
}

static inline uint32_t doodle(uint32_t x)
{
  x ^= x << 13;
  x ^= x >> 17;
  x ^= x << 5;
  return x;
}

static inline float doodlef(uint32_t x)
{
  return ((float)doodle(x)) / (float)(uint32_t(-1));
}

/* task that renders a single screen tile */
void renderPixelStandard(const TutorialData& data,
                         int x, int y, 
                         int* pixels,
                         const unsigned int width,
                         const unsigned int height,
                         const float time,
                         const ISPCCamera& camera, RayStats& stats)
{
  /* initialize ray */
  uint32_t state = doodle(x + y * width);
  state = doodle(state);
  float t = doodlef(state);
  Ray ray(Vec3fa(camera.xfm.p), Vec3fa(normalize(x*camera.xfm.l.vx + y*camera.xfm.l.vy + camera.xfm.l.vz)), 0.0f, inf, t);

  /* intersect ray with scene */
  RTCIntersectArguments iargs;
  rtcInitIntersectArguments(&iargs);
  iargs.feature_mask = (RTCFeatureFlags) (FEATURE_MASK);
  rtcTraversableIntersect1(data.g_traversable,RTCRayHit_(ray),&iargs);
  RayStats_addRay(stats);

  /* shade pixels */
  Vec3fa color = Vec3fa(0.0f);
  if (ray.geomID != RTC_INVALID_GEOMETRY_ID) // || ray.instID[0] != RTC_INVALID_GEOMETRY_ID)
  {
    Vec3fa diffuse = data.face_colors[ray.primID];
    color = color + diffuse*0.5f;
    Vec3fa lightDir = normalize(Vec3fa(-1,-1,-1));

    /* initialize shadow ray */
    Ray shadow(ray.org + ray.tfar*ray.dir, neg(lightDir), 0.001f, inf, 1.f - t);

    /* trace shadow ray */
    RTCOccludedArguments sargs;
    rtcInitOccludedArguments(&sargs);
    sargs.feature_mask = (RTCFeatureFlags) (FEATURE_MASK);
    rtcTraversableOccluded1(data.g_traversable,RTCRay_(shadow),&sargs);
    RayStats_addShadowRay(stats);

    /* add light contribution */
    if (shadow.tfar >= 0.0f)
      color = color + diffuse*clamp(-dot(lightDir,normalize(ray.Ng)),0.0f,1.0f);
  }

  /* write color to framebuffer */
  unsigned int r = (unsigned int) (255.0f * clamp(color.x,0.0f,1.0f));
  unsigned int g = (unsigned int) (255.0f * clamp(color.y,0.0f,1.0f));
  unsigned int b = (unsigned int) (255.0f * clamp(color.z,0.0f,1.0f));
  pixels[y*width+x] = (b << 16) + (g << 8) + r;
}

/* task that renders a single screen tile */
void renderTileTask (int taskIndex, int threadIndex, int* pixels,
                         const unsigned int width,
                         const unsigned int height,
                         const float time,
                         const ISPCCamera& camera,
                         const int numTilesX,
                         const int numTilesY)
{
  const unsigned int tileY = taskIndex / numTilesX;
  const unsigned int tileX = taskIndex - tileY * numTilesX;
  const unsigned int x0 = tileX * TILE_SIZE_X;
  const unsigned int x1 = min(x0+TILE_SIZE_X,width);
  const unsigned int y0 = tileY * TILE_SIZE_Y;
  const unsigned int y1 = min(y0+TILE_SIZE_Y,height);

  for (unsigned int y=y0; y<y1; y++) for (unsigned int x=x0; x<x1; x++)
  {
    renderPixelStandard(data,x,y,pixels,width,height,time,camera,g_stats[threadIndex]);
  }
}

/* called by the C++ code to render */
extern "C" void renderFrameStandard (int* pixels,
                          const unsigned int width,
                          const unsigned int height,
                          const float time,
                          const ISPCCamera& camera)
{
#if defined(EMBREE_SYCL_TUTORIAL) && !defined(EMBREE_SYCL_RT_SIMULATION) && !defined(EMBREE_SYCL_RT_SIMULATION)
  TutorialData ldata = data;
  sycl::event event = global_gpu_queue->submit([=](sycl::handler& cgh){
    const sycl::nd_range<2> nd_range = make_nd_range(height,width);
    cgh.parallel_for(nd_range,[=](sycl::nd_item<2> item) {
      const unsigned int x = item.get_global_id(1); if (x >= width ) return;
      const unsigned int y = item.get_global_id(0); if (y >= height) return;
      RayStats stats;
      renderPixelStandard(ldata,x,y,pixels,width,height,time,camera,stats);
    });
  });
  global_gpu_queue->wait_and_throw();

  const auto t0 = event.template get_profiling_info<sycl::info::event_profiling::command_start>();
  const auto t1 = event.template get_profiling_info<sycl::info::event_profiling::command_end>();
  const double dt = (t1-t0)*1E-9;
  ((ISPCCamera*)&camera)->render_time = dt;
#else
  const int numTilesX = (width +TILE_SIZE_X-1)/TILE_SIZE_X;
  const int numTilesY = (height+TILE_SIZE_Y-1)/TILE_SIZE_Y;
  parallel_for(size_t(0),size_t(numTilesX*numTilesY),[&](const range<size_t>& range) {
    const int threadIndex = (int)TaskScheduler::threadIndex();
    for (size_t i=range.begin(); i<range.end(); i++)
      renderTileTask((int)i,threadIndex,pixels,width,height,time,camera,numTilesX,numTilesY);
  }); 
#endif
}

/* called by the C++ code to render */
extern "C" void device_render (int* pixels,
                           const unsigned int width,
                           const unsigned int height,
                           const float time,
                           const ISPCCamera& camera)
{
}

/* called by the C++ code for cleanup */
extern "C" void device_cleanup ()
{
  TutorialData_Destructor(&data);
}

} // namespace embree
