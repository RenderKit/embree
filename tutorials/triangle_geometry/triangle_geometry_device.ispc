// ======================================================================== //
// Copyright 2009-2018 Intel Corporation                                    //
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

#include "../common/tutorial/tutorial_device.isph"

/* scene data */
RTCScene g_scene = NULL;
uniform Vec3f* uniform face_colors = NULL;
uniform Vec3fa* uniform vertex_colors = NULL;

/* adds a cube to the scene */
uniform unsigned int addCube (RTCScene scene_i)
{
  /* create a triangulated cube with 12 triangles and 8 vertices */
  RTCGeometry mesh = rtcNewGeometry(g_device, RTC_GEOMETRY_TYPE_TRIANGLE);

  /* create face and vertex color arrays */
  face_colors = uniform new uniform Vec3f[12];
  vertex_colors = uniform new uniform Vec3fa[8];

  /* set vertices and vertex colors */
  uniform Vertex* uniform vertices = (uniform Vertex* uniform) rtcSetNewGeometryBuffer(mesh,RTC_BUFFER_TYPE_VERTEX,0,RTC_FORMAT_FLOAT3,sizeof(uniform Vertex),8);
  vertex_colors[0] = make_Vec3fa(0,0,0); vertices[0].x = -1; vertices[0].y = -1; vertices[0].z = -1;
  vertex_colors[1] = make_Vec3fa(0,0,1); vertices[1].x = -1; vertices[1].y = -1; vertices[1].z = +1;
  vertex_colors[2] = make_Vec3fa(0,1,0); vertices[2].x = -1; vertices[2].y = +1; vertices[2].z = -1;
  vertex_colors[3] = make_Vec3fa(0,1,1); vertices[3].x = -1; vertices[3].y = +1; vertices[3].z = +1;
  vertex_colors[4] = make_Vec3fa(1,0,0); vertices[4].x = +1; vertices[4].y = -1; vertices[4].z = -1;
  vertex_colors[5] = make_Vec3fa(1,0,1); vertices[5].x = +1; vertices[5].y = -1; vertices[5].z = +1;
  vertex_colors[6] = make_Vec3fa(1,1,0); vertices[6].x = +1; vertices[6].y = +1; vertices[6].z = -1;
  vertex_colors[7] = make_Vec3fa(1,1,1); vertices[7].x = +1; vertices[7].y = +1; vertices[7].z = +1;

  /* set triangles and face colors */
  uniform int tri = 0;
  uniform Triangle* uniform triangles = (uniform Triangle* uniform) rtcSetNewGeometryBuffer(mesh,RTC_BUFFER_TYPE_INDEX,0,RTC_FORMAT_UINT3,sizeof(uniform Triangle),12);

  // left side
  face_colors[tri] = make_Vec3f(1,0,0); triangles[tri].v0 = 0; triangles[tri].v1 = 1; triangles[tri].v2 = 2; tri++;
  face_colors[tri] = make_Vec3f(1,0,0); triangles[tri].v0 = 1; triangles[tri].v1 = 3; triangles[tri].v2 = 2; tri++;

  // right side
  face_colors[tri] = make_Vec3f(0,1,0); triangles[tri].v0 = 4; triangles[tri].v1 = 6; triangles[tri].v2 = 5; tri++;
  face_colors[tri] = make_Vec3f(0,1,0); triangles[tri].v0 = 5; triangles[tri].v1 = 6; triangles[tri].v2 = 7; tri++;

  // bottom side
  face_colors[tri] = make_Vec3f(0.5f);  triangles[tri].v0 = 0; triangles[tri].v1 = 4; triangles[tri].v2 = 1; tri++;
  face_colors[tri] = make_Vec3f(0.5f);  triangles[tri].v0 = 1; triangles[tri].v1 = 4; triangles[tri].v2 = 5; tri++;

  // top side
  face_colors[tri] = make_Vec3f(1.0f);  triangles[tri].v0 = 2; triangles[tri].v1 = 3; triangles[tri].v2 = 6; tri++;
  face_colors[tri] = make_Vec3f(1.0f);  triangles[tri].v0 = 3; triangles[tri].v1 = 7; triangles[tri].v2 = 6; tri++;

  // front side
  face_colors[tri] = make_Vec3f(0,0,1); triangles[tri].v0 = 0; triangles[tri].v1 = 2; triangles[tri].v2 = 4; tri++;
  face_colors[tri] = make_Vec3f(0,0,1); triangles[tri].v0 = 2; triangles[tri].v1 = 6; triangles[tri].v2 = 4; tri++;

  // back side
  face_colors[tri] = make_Vec3f(1,1,0); triangles[tri].v0 = 1; triangles[tri].v1 = 5; triangles[tri].v2 = 3; tri++;
  face_colors[tri] = make_Vec3f(1,1,0); triangles[tri].v0 = 3; triangles[tri].v1 = 5; triangles[tri].v2 = 7; tri++;

  rtcSetGeometryVertexAttributeCount(mesh,1);
  rtcSetSharedGeometryBuffer(mesh,RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE,0,RTC_FORMAT_FLOAT3,vertex_colors,0,sizeof(uniform Vec3fa),8);
  
  rtcCommitGeometry(mesh);
  uniform unsigned int geomID = rtcAttachGeometry(scene_i,mesh);
  rtcReleaseGeometry(mesh);
  return geomID;
}

/* adds a ground plane to the scene */
uniform unsigned int addGroundPlane (RTCScene scene_i)
{
  /* create a triangulated plane with 2 triangles and 4 vertices */
  RTCGeometry mesh = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_TRIANGLE);

  /* set vertices */
  uniform Vertex* uniform vertices = (uniform Vertex* uniform) rtcSetNewGeometryBuffer(mesh,RTC_BUFFER_TYPE_VERTEX,0,RTC_FORMAT_FLOAT3,sizeof(uniform Vertex),4);
  vertices[0].x = -10; vertices[0].y = -2; vertices[0].z = -10;
  vertices[1].x = -10; vertices[1].y = -2; vertices[1].z = +10;
  vertices[2].x = +10; vertices[2].y = -2; vertices[2].z = -10;
  vertices[3].x = +10; vertices[3].y = -2; vertices[3].z = +10;

  /* set triangles */
  uniform Triangle* uniform triangles = (uniform Triangle* uniform) rtcSetNewGeometryBuffer(mesh,RTC_BUFFER_TYPE_INDEX,0,RTC_FORMAT_UINT3,sizeof(uniform Triangle),2);
  triangles[0].v0 = 0; triangles[0].v1 = 1; triangles[0].v2 = 2;
  triangles[1].v0 = 1; triangles[1].v1 = 3; triangles[1].v2 = 2;
  
  rtcCommitGeometry(mesh);
  uniform unsigned int geomID = rtcAttachGeometry(scene_i,mesh);
  rtcReleaseGeometry(mesh);
  return geomID;
}

/* called by the C++ code for initialization */
export void device_init (uniform int8* uniform cfg)
{
  /* create scene */
  g_scene = rtcNewScene(g_device);

  /* add cube */
  addCube(g_scene);

  /* add ground plane */
  addGroundPlane(g_scene);

  /* commit changes to scene */
  rtcCommitScene (g_scene);

  /* set start render mode */
  renderTile = renderTileStandard;
  key_pressed_handler = device_key_pressed_default;
}

/* task that renders a single screen tile */
Vec3f renderPixelStandard(float x, float y, const uniform ISPCCamera& camera, uniform RayStats& stats)
{
  uniform RTCIntersectContext context;
  rtcInitIntersectContext(&context);
  
  /* initialize ray */
  Ray ray = make_Ray(make_Vec3f(camera.xfm.p), make_Vec3f(normalize(x*camera.xfm.l.vx + y*camera.xfm.l.vy + camera.xfm.l.vz)), 0.0f, inf);

  /* intersect ray with scene */
  rtcIntersectV(g_scene,&context,RTCRayHit_(ray));
  RayStats_addRay(stats);

  /* shade pixels */
  Vec3f color = make_Vec3f(0.0f);
  if (ray.geomID != RTC_INVALID_GEOMETRY_ID)
  {
    Vec3f diffuse = face_colors[ray.primID];
    color = color + diffuse*0.5f;
    Vec3f lightDir = normalize(make_Vec3f(-1,-1,-1));

    /* initialize shadow ray */
    Ray shadow = make_Ray(ray.org + ray.tfar*ray.dir, neg(lightDir), 0.001f, inf, 0.0f);

    /* trace shadow ray */
    rtcOccludedV(g_scene,&context,RTCRay_(shadow));
    RayStats_addShadowRay(stats);

    /* add light contribution */
    if (shadow.tfar >= 0.0f)
      color = color + diffuse*clamp(-dot(lightDir,normalize(ray.Ng)),0.0f,1.0f);
  }
  return color;
}

/* renders a single screen tile */
void renderTileStandard(uniform int taskIndex,
                        uniform int threadIndex,
                        uniform int* uniform pixels,
                        const uniform unsigned int width,
                        const uniform unsigned int height,
                        const uniform float time,
                        const uniform ISPCCamera& camera,
                        const uniform int numTilesX,
                        const uniform int numTilesY)
{
  const uniform unsigned int tileY = taskIndex / numTilesX;
  const uniform unsigned int tileX = taskIndex - tileY * numTilesX;
  const uniform unsigned int x0 = tileX * TILE_SIZE_X;
  const uniform unsigned int x1 = min(x0+TILE_SIZE_X,width);
  const uniform unsigned int y0 = tileY * TILE_SIZE_Y;
  const uniform unsigned int y1 = min(y0+TILE_SIZE_Y,height);

  foreach_tiled (y = y0 ... y1, x = x0 ... x1)
  {
    /* calculate pixel color */
    Vec3f color = renderPixelStandard((float)x,(float)y,camera,g_stats[threadIndex]);

    /* write color to framebuffer */
    unsigned int r = (unsigned int) (255.0f * clamp(color.x,0.0f,1.0f));
    unsigned int g = (unsigned int) (255.0f * clamp(color.y,0.0f,1.0f));
    unsigned int b = (unsigned int) (255.0f * clamp(color.z,0.0f,1.0f));
    pixels[y*width+x] = (b << 16) + (g << 8) + r;
  }
}

/* task that renders a single screen tile */
task void renderTileTask(uniform int* uniform pixels,
                         const uniform unsigned int width,
                         const uniform unsigned int height,
                         const uniform float time,
                         const uniform ISPCCamera& camera,
                         const uniform int numTilesX,
                         const uniform int numTilesY)
{
  renderTile(taskIndex,threadIndex,pixels,width,height,time,camera,numTilesX,numTilesY);
}

/* called by the C++ code to render */
export void device_render (uniform int* uniform pixels,
                           const uniform unsigned int width,
                           const uniform unsigned int height,
                           const uniform float time,
                           const uniform ISPCCamera& camera)
{
  const uniform int numTilesX = (width +TILE_SIZE_X-1)/TILE_SIZE_X;
  const uniform int numTilesY = (height+TILE_SIZE_Y-1)/TILE_SIZE_Y;
  launch[numTilesX*numTilesY] renderTileTask(pixels,width,height,time,camera,numTilesX,numTilesY); sync;
}

/* called by the C++ code for cleanup */
export void device_cleanup ()
{
  rtcReleaseScene (g_scene); g_scene = NULL;
  delete[] face_colors; face_colors = NULL;
  delete[] vertex_colors; vertex_colors = NULL;
}
