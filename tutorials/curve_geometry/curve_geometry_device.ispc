// ======================================================================== //
// Copyright 2009-2016 Intel Corporation                                    //
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
#include "../common/tutorial/optics.isph"

/* scene data */
RTCDevice g_device = NULL;
RTCScene  g_scene  = NULL;

#define NUM_VERTICES 9
#define NUM_CURVES 6

#define W 2.0f

uniform float hair_vertices[9][4] =
{
  { -1.0f, 0.0f, -   W, 0.2f },

  { +0.0f,-1.0f, +0.0f, 0.2f },
  { +1.0f, 0.0f, +   W, 0.2f },
  { -1.0f, 0.0f, +   W, 0.2f },
  { +0.0f,+1.0f, +0.0f, 0.6f },
  { +1.0f, 0.0f, -   W, 0.2f },
  { -1.0f, 0.0f, -   W, 0.2f },

  { +0.0f,-1.0f, +0.0f, 0.2f },
  { +1.0f, 0.0f, +   W, 0.2f },
};

uniform float hair_vertex_colors[9][4] =
{
  {  1.0f,  1.0f,  0.0f, 0.0f },

  {  1.0f,  0.0f,  0.0f, 0.0f },
  {  1.0f,  1.0f,  0.0f, 0.0f },
  {  0.0f,  0.0f,  1.0f, 0.0f },
  {  1.0f,  1.0f,  1.0f, 0.0f },
  {  1.0f,  0.0f,  0.0f, 0.0f },
  {  1.0f,  1.0f,  0.0f, 0.0f },

  {  1.0f,  0.0f,  0.0f, 0.0f },
  {  1.0f,  1.0f,  0.0f, 0.0f },
};

uniform unsigned int hair_indices[6] = {
  0, 1, 2, 3, 4, 5
};

/* add hair geometry */
uniform unsigned int addCurve (RTCScene scene, const uniform Vec3f& pos)
{
  uniform unsigned int geomID = rtcNewCurveGeometry (scene, RTC_GEOMETRY_STATIC, NUM_CURVES, 4*NUM_CURVES);

  /* converts b-spline to bezier basis */
  uniform Vec3fa* uniform vtx = (uniform Vec3fa* uniform) rtcMapBuffer(scene, geomID, RTC_VERTEX_BUFFER);
  for (uniform size_t i=0; i<NUM_CURVES; i++)
  {
    uniform Vec3fa P = make_Vec3fa(pos.x,pos.y,pos.z,0.0f);
    const uniform Vec3fa v0 = make_Vec3fa(hair_vertices[i+0][0],hair_vertices[i+0][1],hair_vertices[i+0][2],hair_vertices[i+0][3]);
    const uniform Vec3fa v1 = make_Vec3fa(hair_vertices[i+1][0],hair_vertices[i+1][1],hair_vertices[i+1][2],hair_vertices[i+1][3]);
    const uniform Vec3fa v2 = make_Vec3fa(hair_vertices[i+2][0],hair_vertices[i+2][1],hair_vertices[i+2][2],hair_vertices[i+2][3]);
    const uniform Vec3fa v3 = make_Vec3fa(hair_vertices[i+3][0],hair_vertices[i+3][1],hair_vertices[i+3][2],hair_vertices[i+3][3]);
    vtx[4*i+0] = P + (1.0f/6.0f)*v0 + (2.0f/3.0f)*v1 + (1.0f/6.0f)*v2;
    vtx[4*i+1] = P + (2.0f/3.0f)*v1 + (1.0f/3.0f)*v2;
    vtx[4*i+2] = P + (1.0f/3.0f)*v1 + (2.0f/3.0f)*v2;
    vtx[4*i+3] = P + (1.0f/6.0f)*v1 + (2.0f/3.0f)*v2 + (1.0f/6.0f)*v3;
  }
  rtcUnmapBuffer(scene, geomID, RTC_VERTEX_BUFFER);

  uniform Vec3fa* uniform colors = uniform new uniform Vec3fa[4*NUM_CURVES];
  for (uniform size_t i=0; i<NUM_CURVES; i++)
  {
    const uniform Vec3fa v0 = make_Vec3fa(hair_vertex_colors[i+0][0],hair_vertex_colors[i+0][1],hair_vertex_colors[i+0][2],hair_vertex_colors[i+0][3]);
    const uniform Vec3fa v1 = make_Vec3fa(hair_vertex_colors[i+1][0],hair_vertex_colors[i+1][1],hair_vertex_colors[i+1][2],hair_vertex_colors[i+1][3]);
    const uniform Vec3fa v2 = make_Vec3fa(hair_vertex_colors[i+2][0],hair_vertex_colors[i+2][1],hair_vertex_colors[i+2][2],hair_vertex_colors[i+2][3]);
    const uniform Vec3fa v3 = make_Vec3fa(hair_vertex_colors[i+3][0],hair_vertex_colors[i+3][1],hair_vertex_colors[i+3][2],hair_vertex_colors[i+3][3]);
    colors[4*i+0] = (1.0f/6.0f)*v0 + (2.0f/3.0f)*v1 + (1.0f/6.0f)*v2;
    colors[4*i+1] = (2.0f/3.0f)*v1 + (1.0f/3.0f)*v2;
    colors[4*i+2] = (1.0f/3.0f)*v1 + (2.0f/3.0f)*v2;
    colors[4*i+3] = (1.0f/6.0f)*v1 + (2.0f/3.0f)*v2 + (1.0f/6.0f)*v3;
  }

  uniform int* uniform index = (uniform int* uniform) rtcMapBuffer(scene, geomID, RTC_INDEX_BUFFER);
  for (uniform size_t i=0; i<NUM_CURVES; i++) {
    index[i] = 4*i;
  }
  rtcUnmapBuffer(scene,geomID,RTC_INDEX_BUFFER);

  rtcSetBuffer(scene, geomID, RTC_USER_VERTEX_BUFFER0, colors, 0, sizeof(uniform Vec3fa));
  return geomID;
}

/* adds a ground plane to the scene */
uniform unsigned int addGroundPlane (RTCScene scene_i)
{
  /* create a triangulated plane with 2 triangles and 4 vertices */
  uniform unsigned int mesh = rtcNewTriangleMesh (scene_i, RTC_GEOMETRY_STATIC, 2, 4);

  /* set vertices */
  uniform Vertex* uniform vertices = (uniform Vertex* uniform) rtcMapBuffer(scene_i,mesh,RTC_VERTEX_BUFFER);
  vertices[0].x = -10; vertices[0].y = -2; vertices[0].z = -10;
  vertices[1].x = -10; vertices[1].y = -2; vertices[1].z = +10;
  vertices[2].x = +10; vertices[2].y = -2; vertices[2].z = -10;
  vertices[3].x = +10; vertices[3].y = -2; vertices[3].z = +10;
  rtcUnmapBuffer(scene_i,mesh,RTC_VERTEX_BUFFER);

  /* set triangles */
  uniform Triangle* uniform triangles = (uniform Triangle* uniform) rtcMapBuffer(scene_i,mesh,RTC_INDEX_BUFFER);
  triangles[0].v0 = 0; triangles[0].v1 = 2; triangles[0].v2 = 1;
  triangles[1].v0 = 1; triangles[1].v1 = 2; triangles[1].v2 = 3;
  rtcUnmapBuffer(scene_i,mesh,RTC_INDEX_BUFFER);

  return mesh;
}

/* called by the C++ code for initialization */
export void device_init (uniform int8* uniform cfg)
{
  /* create new Embree device */
  g_device = rtcNewDevice(cfg);
  error_handler(rtcDeviceGetError(g_device));

  /* set error handler */
  rtcDeviceSetErrorFunction(g_device,error_handler);

  /* create scene */
  g_scene = rtcDeviceNewScene(g_device, RTC_SCENE_STATIC,RTC_INTERSECT_UNIFORM | RTC_INTERSECT_VARYING | RTC_INTERPOLATE);

  /* add ground plane */
  addGroundPlane(g_scene);

  /* add curve */
  addCurve(g_scene,make_Vec3f(0.0f,0.0f,0.0f));

  /* commit changes to scene */
  rtcCommit (g_scene);

  /* set start render mode */
  renderTile = renderTileStandard;
  key_pressed_handler = device_key_pressed_default;
}

/* task that renders a single screen tile */
Vec3f renderPixelStandard(float x, float y, const uniform ISPCCamera& camera)
{
  /* initialize ray */
  RTCRay ray;
  ray.org = make_Vec3f(camera.xfm.p);
  ray.dir = make_Vec3f(normalize(x*camera.xfm.l.vx + y*camera.xfm.l.vy + camera.xfm.l.vz));
  ray.tnear = 0.0f;
  ray.tfar = inf;
  ray.geomID = RTC_INVALID_GEOMETRY_ID;
  ray.primID = RTC_INVALID_GEOMETRY_ID;
  ray.mask = -1;
  ray.time = 0;

  /* intersect ray with scene */
  rtcIntersect(g_scene,ray);

  /* shade pixels */
  Vec3f color = make_Vec3f(0.0f);
  if (ray.geomID != RTC_INVALID_GEOMETRY_ID)
  {
    /* interpolate diffuse color */
    Vec3f diffuse = make_Vec3f(1.0f,0.0f,0.0f);
    if (ray.geomID > 0)
    {
      foreach_unique (geomID in ray.geomID) {
        rtcInterpolate(g_scene,geomID,ray.primID,ray.u,ray.v,RTC_USER_VERTEX_BUFFER0,&diffuse.x,NULL,NULL,3);
      }
      diffuse = 0.5f*diffuse;
    }

    /* calculate smooth shading normal */
    Vec3f Ng = normalize(ray.Ng);
    color = color + diffuse*0.5f;
    Vec3f lightDir = normalize(make_Vec3f(-1,-1,-1));

    /* initialize shadow ray */
    RTCRay shadow;
    shadow.org = ray.org + ray.tfar*ray.dir;
    shadow.dir = neg(lightDir);
    shadow.tnear = 0.001f;
    shadow.tfar = inf;
    shadow.geomID = 1;
    shadow.primID = 0;
    shadow.mask = -1;
    shadow.time = 0;

    /* trace shadow ray */
    rtcOccluded(g_scene,shadow);

    /* add light contribution */
    if (shadow.geomID) {
      Vec3f r = normalize(reflect(ray.dir,Ng));
      float s = pow(clamp(dot(r,lightDir),0.0f,1.0f),10.0f);
      float d = clamp(-dot(lightDir,Ng),0.0f,1.0f);
      color = color + diffuse*d + 0.5f*make_Vec3f(s);
    }
  }
  return color;
}

/* renders a single screen tile */
void renderTileStandard(uniform int taskIndex,
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
    Vec3f color = renderPixelStandard((float)x,(float)y,camera);

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
  renderTile(taskIndex,pixels,width,height,time,camera,numTilesX,numTilesY);
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
  rtcDeleteScene (g_scene); g_scene = NULL;
  rtcDeleteDevice(g_device); g_device = NULL;
}
