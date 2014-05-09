// ======================================================================== //
// Copyright 2009-2014 Intel Corporation                                    //
//                                                                          //
// Licensed under the Apache License, Version 2.0 (the "License");          //
// you may not use this file except in compliance with the License.         //
// You may obtain a copy of the License at                                  //
//                                                                          //
//     http://www.apache.org/licenses/LICENSE-2.0                           //
//                                                                          //
// Unless required by applicable law or agreed to in writing, software      //
// distributed under the License is distributed on an "AS IS" BASIS,        //
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. //
// See the License for the specific language governing permissions and      //
// limitations under the License.                                           //
// ======================================================================== //

#include "../common/tutorial/tutorial_device.h"
#include "extensions/api.h"

/* Function used to render a pixel. */
renderPixelFunc renderPixel;

/* Embree state identifier for the scene. */
RTCScene g_scene = NULL;

/* Requested subdivision level set in tutorial08.cpp. */
int subdivisionLevel = 0;

RTCRay constructRay(const Vec3fa &origin, const Vec3fa &direction, float near, float far, int originGeomID, int originPrimID) {

    RTCRay ray;
    ray.org = origin;
    ray.dir = direction;
    ray.tnear = near;
    ray.tfar = far;
    ray.geomID = originGeomID;
    ray.primID = originPrimID;
    ray.mask = -1;
    ray.time = 0;
    return(ray);

}

unsigned int packPixel(const Vec3f &color) {

    unsigned int r = (unsigned int) (255.0f * clamp(color.x, 0.0f, 1.0f));
    unsigned int g = (unsigned int) (255.0f * clamp(color.y, 0.0f, 1.0f));
    unsigned int b = (unsigned int) (255.0f * clamp(color.z, 0.0f, 1.0f));
    return((b << 16) + (g << 8) + r);

}

void constructCubeMesh() {

    /* Cube vertices. */
    static const float cubeVertices[24] = {-1, -1, -1, 1, -1, -1, 1, -1, 1, -1, -1, 1, -1, 1, -1, 1, 1, -1, 1, 1, 1, -1, 1, 1};

    /* Cube faces. */
    static const int32_t cubeIndices[24] = {0, 1, 5, 4, 1, 2, 6, 5, 2, 3, 7, 6, 0, 4, 7, 3, 4, 5, 6, 7, 0, 3, 2, 1};

    /* Offsets into the index array per face. */
    static const int32_t cubeOffsets[7] = {0, 4, 8, 12, 16, 20, 24};

    /* Construct a subdivision mesh object. */
    unsigned int meshID = rtcxNewSubdivisionMesh(g_scene, RTC_GEOMETRY_STATIC, 6, 12, 8);

    /* Map the subdivision mesh vertex buffer from Embree space into user space. */
    float *vertices = (float *) rtcxMapBuffer(g_scene, meshID, RTCX_VERTEX_BUFFER);

    /* Copy vertex data into the subdivision mesh buffer. */
    memcpy(vertices, cubeVertices, 24 * sizeof(float));

    /* Unmap the subdivision mesh buffer. */
    rtcxUnmapBuffer(g_scene, meshID, RTCX_VERTEX_BUFFER);

    /* Map the subdivision mesh index buffer from Embree space into user space. */
    int32_t *indices = (int32_t *) rtcxMapBuffer(g_scene, meshID, RTCX_INDEX_BUFFER);

    /* Copy vertex indices into the subdivision mesh buffer. */
    memcpy(indices, cubeIndices, 24 * sizeof(int32_t));

    /* Unmap the subdivision mesh buffer. */
    rtcxUnmapBuffer(g_scene, meshID, RTCX_INDEX_BUFFER);

    /* Map the subdivision mesh offset buffer from Embree space into user space. */
    int32_t *offsets = (int32_t *) rtcxMapBuffer(g_scene, meshID, RTCX_OFFSET_BUFFER);

    /* Copy face offsets into the subdivision mesh buffer. */
    memcpy(offsets, cubeOffsets, 7 * sizeof(int32_t));

    /* Unmap the subdivision mesh buffer. */
    rtcxUnmapBuffer(g_scene, meshID, RTCX_OFFSET_BUFFER);

}

void constructGroundPlane() {

    /* Plane vertices. */
    static Vertex planeVertices[4] = {{-100, -2, -100, 0}, {-100, -2, 100, 0}, {100, -2, -100, 0}, {100, -2, 100, 0}};

    /* Construct a triangle mesh object. */
    unsigned int meshID = rtcNewTriangleMesh(g_scene, RTC_GEOMETRY_STATIC, 2, 4);

    /* Map the triangle mesh vertex buffer from Embree space into user space. */
    Vertex *vertices = (Vertex *) rtcMapBuffer(g_scene, meshID, RTC_VERTEX_BUFFER);

    /* Copy vertex data into the triangle mesh buffer. */
    memcpy(vertices, planeVertices, 4 * sizeof(Vertex));

    /* Unmap the triangle mesh buffer. */
    rtcUnmapBuffer(g_scene, meshID, RTC_VERTEX_BUFFER);

    /* Map the triangle mesh index buffer from Embree space into user space. */
    Triangle *triangles = (Triangle *) rtcMapBuffer(g_scene, meshID, RTC_INDEX_BUFFER);

    /* Copy vertex indices into the triangle mesh buffer. */
    triangles[0].v0 = 0;  triangles[0].v1 = 2;  triangles[0].v2 = 1;  triangles[1].v0 = 1;  triangles[1].v1 = 2;  triangles[1].v2 = 3;

    /* Unmap the triangle mesh buffer. */
    rtcUnmapBuffer(g_scene, meshID, RTC_INDEX_BUFFER);

}

void constructScene() {

    /* Create an Embree object to hold scene state. */
    g_scene = rtcNewScene(RTC_SCENE_STATIC, RTC_INTERSECT1);

    /* Construct a cube shaped subdivision mesh. */
    constructCubeMesh();

    /* Construct a triangle mesh object for the ground plane. */
    constructGroundPlane();

    /* Commit the changes to the scene state. */
    rtcxCommit(g_scene);

}

Vec3fa renderPixelStandard(float x, float y, const Vec3fa &vx, const Vec3fa &vy, const Vec3fa &vz, const Vec3fa &p) {

    /* Colors of the subdivision mesh and ground plane. */
    Vec3f colors[2];  colors[0] = Vec3f(1.0f, 0.0f, 0.0f);  colors[1] = Vec3f(0.5f, 0.5f, 0.5f);

    /* Initialize a ray and intersect with the scene. */
    RTCRay ray = constructRay(p, normalize(x * vx + y * vy + vz), 0.0f, inf, RTC_INVALID_GEOMETRY_ID, RTC_INVALID_GEOMETRY_ID);  rtcIntersect(g_scene, ray);
  
    /* The ray may not have hit anything. */
    if (ray.geomID == RTC_INVALID_GEOMETRY_ID) return(Vec3f(0.0f));

    /* Compute a vector parallel to a directional light. */
    Vec3f lightVector = normalize(Vec3f(-1.0f, -1.0f, -1.0f));

    /* Initialize a shadow ray and intersect with the scene. */
    RTCRay shadow = constructRay(ray.org + ray.tfar * ray.dir, neg(lightVector), 0.001f, inf, 1, 0);  rtcOccluded(g_scene, shadow);

    /* Compute a color at the ray hit point. */
    Vec3f color = Vec3f(0.0f), diffuse = colors[ray.geomID % 2];  color = color + diffuse * 0.5f;

    /* Add contribution from the light. */
    if (shadow.geomID) color = color + diffuse * clamp(-dot(lightVector, normalize(ray.Ng)), 0.0f, 1.0f);  return(color);

}

void renderTile(int taskIndex, int *pixels, int width, int height, float time, const Vec3fa &vx, const Vec3fa &vy, const Vec3fa &vz, const Vec3fa &p, int tileCountX, int tileCountY) {

    /* 2D indices of the tile in the window. */
    const Vec2i tileIndex(taskIndex % tileCountX, taskIndex / tileCountX);

    /* 2D indices of the pixel in the lower left of the tile corner. */
    const int x0 = tileIndex.x * TILE_SIZE_X, y0 = tileIndex.y * TILE_SIZE_Y;

    /* 2D indices of the pixel in the upper right of the tile corner. */
    const int x1 = min(x0 + TILE_SIZE_X, width), y1 = min(y0 + TILE_SIZE_Y, height);

    /* Compute the color of each pixel in the tile. */
    for (int y=y0 ; y < y1 ; y++) for (int x=x0 ; x < x1 ; x++) pixels[y * width + x] = packPixel(renderPixel(x, y, vx, vy, vz, p));

}

extern "C" void device_cleanup() {

    rtcDeleteScene(g_scene);
    rtcExit();

}

extern "C" void device_init(int8 *configuration) {

    /* Initialize Embree ray tracing state. */
    rtcInit(configuration);

    /* Set the render mode to use on entry into the run loop. */
    renderPixel = renderPixelStandard;

    /* Construct a scene with a cube shaped subdivision mesh and a ground plane. */
    constructScene();

}

extern "C" void device_render(int *pixels, int width, int height, float time, const Vec3fa &vx, const Vec3fa &vy, const Vec3fa &vz, const Vec3fa &p) {

    /* Refine the subdivision mesh as needed. */
    static int currentLevel = 0;  if (currentLevel != subdivisionLevel) rtcDeleteScene(g_scene), constructScene(), currentLevel = subdivisionLevel;

    /* Number of tiles spanning the window in width and height. */
    const Vec2i tileCount((width + TILE_SIZE_X - 1) / TILE_SIZE_X, (height + TILE_SIZE_Y - 1) / TILE_SIZE_Y);

    /* Render a tile at a time. */
    launch_renderTile(tileCount.x * tileCount.y, pixels, width, height, time, vx, vy, vz, p, tileCount.x, tileCount.y); 

    /* Debugging information. */
    rtcDebug();

}

