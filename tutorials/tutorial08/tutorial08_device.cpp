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
#include "catmullclark.h"
#include "subdivisionmesh.h"

/* Current and requested (tutorial08.cpp) subdivision refinement level. */
int currentSubdivisionLevel = 0, subdivisionLevel = 0;

/* Embree state identifiers for the scene and triangle mesh corresponding to the subdivision mesh. */
RTCScene g_scene = NULL;  unsigned int g_mesh;

/* Coarse subdivision mesh. */
SubdivisionMesh originalMesh;

/* Function used to render a pixel. */
renderPixelFunc renderPixel;

/* Construct a new Embree triangle mesh object and triangulate the subdivision mesh. */
void triangulateMesh(SubdivisionMesh &mesh);

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
    static const int32_t cubeFaceIndices[24] = {0, 1, 5, 4, 1, 2, 6, 5, 2, 3, 7, 6, 0, 4, 7, 3, 4, 5, 6, 7, 0, 3, 2, 1};

    /* Offsets into the index array per face. */
    static const int32_t cubeFaceOffsets[7] = {0, 4, 8, 12, 16, 20, 24};

    /* Construct the subdivision mesh [vertex count, half edge count, face count]. */
    originalMesh = SubdivisionMesh(cubeVertices, cubeFaceIndices, cubeFaceOffsets, 8, 24, 6);

}

void constructGroundPlane() {

    /* Plane vertices. */
    static Vertex planeVertices[4] = {{-100, -2, -100, 0}, {-100, -2, 100, 0}, {100, -2, -100, 0}, {100, -2, 100, 0}};

    /* Construct a triangulated plane with 2 triangles and 4 vertices. */
    unsigned int plane = rtcNewTriangleMesh(g_scene, RTC_GEOMETRY_STATIC, 2, 4);

    /* Map the triangle mesh vertex buffer from Embree space into user space. */
    Vertex *vertices = (Vertex *) rtcMapBuffer(g_scene, plane, RTC_VERTEX_BUFFER);

    /* Copy plane vertex data into the triangle mesh vertex buffer. */
    memcpy(vertices, planeVertices, 4 * sizeof(Vertex));

    /* Unmap the triangle mesh vertex buffer. */
    rtcUnmapBuffer(g_scene, plane, RTC_VERTEX_BUFFER);

    /* Map the triangle buffer from Embree space into user space. */
    Triangle *triangles = (Triangle *) rtcMapBuffer(g_scene, plane, RTC_INDEX_BUFFER);

    /* Store the indices of the vertices composing each triangle in the mesh. */
    triangles[0].v0 = 0;  triangles[0].v1 = 2;  triangles[0].v2 = 1;  triangles[1].v0 = 1;  triangles[1].v1 = 2;  triangles[1].v2 = 3;

    /* Unmap the triangle buffer. */
    rtcUnmapBuffer(g_scene, plane, RTC_INDEX_BUFFER);

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

void renderTile(int taskIndex, int *pixels, int width, int height, float time, const Vec3f &vx, const Vec3f &vy, const Vec3f &vz, const Vec3f &p, int tileCountX, int tileCountY) {

    /* 2D indices of the tile in the window. */
    const Vec2i tileIndex(taskIndex % tileCountX, taskIndex / tileCountX);

    /* 2D indices of the pixel in the lower left of the tile corner. */
    const int x0 = tileIndex.x * TILE_SIZE_X, y0 = tileIndex.y * TILE_SIZE_Y;

    /* 2D indices of the pixel in the upper right of the tile corner. */
    const int x1 = min(x0 + TILE_SIZE_X, width), y1 = min(y0 + TILE_SIZE_Y, height);

    /* Compute the color of each pixel in the tile. */
    for (int y=y0 ; y < y1 ; y++) for (int x=x0 ; x < x1 ; x++) pixels[y * width + x] = packPixel(renderPixel(x, y, vx, vy, vz, p));

}

void subdivideMesh(int level) {

    /* Remove the old scene from the Embree state. */
    rtcDeleteScene(g_scene);

    /* Create an Embree object to hold scene state. */
    g_scene = rtcNewScene(RTC_SCENE_STATIC, RTC_INTERSECT1);

    /* Temporary meshes used during subdivision. */
    SubdivisionMesh meshes[2];  meshes[0] = originalMesh;

    /* Subdivide the mesh using Catmull-Clark. */
    for (size_t i=0 ; i < level ; i++) meshes[i % 2].commit(), catmullClarkSubdivideMesh(meshes[i % 2], meshes[(i + 1) % 2]);

    /* Construct a new Embree triangle mesh object and triangulate the subdivision mesh. */
    triangulateMesh(meshes[level % 2]);

    /* Construct a new Embree triangle mesh object for the ground plane. */
    constructGroundPlane();

    /* Commit the changes to the scene state. */
    rtcCommit(g_scene);

}

void triangulateFace(const SubdivisionMesh::Face &face, Triangle *triangles) {

    for (size_t i=0, j=1, k=2 ; j < face.vertexCount() - 1 ; i++, j++, k++) {

        triangles[i].v0 = face.getVertex(0).getIndex();
        triangles[i].v1 = face.getVertex(j).getIndex();
        triangles[i].v2 = face.getVertex(k).getIndex();

    }

}

void triangulateMesh(SubdivisionMesh &mesh) {

    /* Number of triangles to be constructed from the subdivision mesh. */
    size_t triangleCount = mesh.faceCount() + mesh.edgeCount() - 3 * mesh.faceCount();

    /* Construct a new Embree triangle mesh object. */
    g_mesh = rtcNewTriangleMesh(g_scene, RTC_GEOMETRY_STATIC, triangleCount, mesh.vertexCount());

    /* Map the triangle mesh vertex buffer from Embree space into user space. */
    Vertex *vertices = (Vertex *) rtcMapBuffer(g_scene, g_mesh, RTC_VERTEX_BUFFER);

    /* Copy vertex data from the subdivision mesh into the triangle mesh vertex buffer. */
    for (size_t i=0 ; i < mesh.vertexCount() ; i++) { Vec3f p = mesh.getCoordinates(i);  vertices[i].x = p.x;  vertices[i].y = p.y;  vertices[i].z = p.z; }

    /* Unmap the triangle mesh vertex buffer. */
    rtcUnmapBuffer(g_scene, g_mesh, RTC_VERTEX_BUFFER);

    /* Map the triangle buffer from Embree space into user space. */
    Triangle *triangles = (Triangle *) rtcMapBuffer(g_scene, g_mesh, RTC_INDEX_BUFFER);

    /* Store the indices of the vertices composing each triangle in the mesh. */
    for (size_t i=0, j=0 ; i < mesh.faceCount() ; j += mesh.getFace(i).vertexCount() - 2, i++) triangulateFace(mesh.getFace(i), &triangles[j]);

    /* Unmap the triangle buffer. */
    rtcUnmapBuffer(g_scene, g_mesh, RTC_INDEX_BUFFER);

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

    /* Create an Embree object to hold scene state. */
    g_scene = rtcNewScene(RTC_SCENE_STATIC, RTC_INTERSECT1);

    /* Construct a cube shaped subdivision mesh. */
    constructCubeMesh();

    /* Construct a new Embree triangle mesh object and triangulate the subdivision mesh. */
    triangulateMesh(originalMesh);

    /* Construct a new Embree triangle mesh object for the ground plane. */
    constructGroundPlane();

    /* Commit the changes to the scene state. */
    rtcCommit(g_scene);

}

extern "C" void device_render(int *pixels, int width, int height, float time, const Vec3f &vx, const Vec3f &vy, const Vec3f &vz, const Vec3f &p) {

    /* Refine the subdivision mesh as needed. */
    if (currentSubdivisionLevel != subdivisionLevel) subdivideMesh(currentSubdivisionLevel = subdivisionLevel);

    /* Number of tiles spanning the window in width and height. */
    const Vec2i tileCount((width + TILE_SIZE_X - 1) / TILE_SIZE_X, (height + TILE_SIZE_Y - 1) / TILE_SIZE_Y);

    /* Render a tile at a time. */
    launch_renderTile(tileCount.x * tileCount.y, pixels, width, height, time, vx, vy, vz, p, tileCount.x, tileCount.y); 

    /* Debugging information. */
    rtcDebug();

}

