// ======================================================================== //
// Copyright 2009-2015 Intel Corporation                                    //
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

/* render function to use */
renderPixelFunc renderPixel;

/* error reporting function */
void error_handler(const uniform RTCError code, const uniform int8* uniform str)
{
  print("Embree: ");
  switch (code) {
  case RTC_UNKNOWN_ERROR    : print("RTC_UNKNOWN_ERROR"); break;
  case RTC_INVALID_ARGUMENT : print("RTC_INVALID_ARGUMENT"); break;
  case RTC_INVALID_OPERATION: print("RTC_INVALID_OPERATION"); break;
  case RTC_OUT_OF_MEMORY    : print("RTC_OUT_OF_MEMORY"); break;
  case RTC_UNSUPPORTED_CPU  : print("RTC_UNSUPPORTED_CPU"); break;
  case RTC_CANCELLED        : print("RTC_CANCELLED"); break;
  default                   : print("invalid error code"); break;
  }
  if (str) { 
    print(" ("); 
    while (*str) putchar(*str++); 
    print(")\n"); 
  }
  abort();
}

__aligned(16) uniform float cube_vertices[8][4] = 
{
  { -1.0f, -1.0f, -1.0f, 0.0f },
  { -1.0f, -1.0f, +1.0f, 0.0f },
  { -1.0f, +1.0f, -1.0f, 0.0f },
  { -1.0f, +1.0f, +1.0f, 0.0f },
  { +1.0f, -1.0f, -1.0f, 0.0f },
  { +1.0f, -1.0f, +1.0f, 0.0f },
  { +1.0f, +1.0f, -1.0f, 0.0f },
  { +1.0f, +1.0f, +1.0f, 0.0f }
};

uniform unsigned int cube_indices[36] = { 
  0, 2, 1,  1, 2, 3,
  4, 5, 6,  5, 7, 6,
  0, 1, 4,  1, 6, 4,
  2, 6, 3,  3, 6, 7, 
  0, 4, 2,  2, 4, 6, 
  1, 3, 5,  3, 7, 5
};

uniform unsigned int cube_faces[12] = { 
  3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3
};

/* adds a cube to the scene */
uniform unsigned int addCube (RTCScene scene_i)
{
  /* create a triangulated cube with 12 triangles and 8 vertices */
  uniform unsigned int geomID = rtcNewTriangleMesh (scene_i, RTC_GEOMETRY_STATIC, 12, 8, 2);
  rtcSetBuffer(scene_i, geomID, RTC_INDEX_BUFFER,  cube_indices , 0, 3*sizeof(uniform unsigned int));
  rtcSetBuffer(scene_i, geomID, RTC_VERTEX_BUFFER0, cube_vertices, 0, 4*sizeof(uniform float));

  uniform Vec3fa* uniform vertex1 = (uniform Vec3fa* uniform) rtcMapBuffer(scene_i,geomID,RTC_VERTEX_BUFFER1);
  for (uniform i=0; i<8; i++) vertex1[i] = make_Vec3fa(cube_vertices[i][0]+1.0f,cube_vertices[i][1]+1.0f,cube_vertices[i][2]+1.0f);
  rtcUnmapBuffer(scene_i,geomID,RTC_VERTEX_BUFFER1);
  
  /* create face color array */
  face_colors = uniform new uniform Vec3f[12];
  face_colors[0] = make_Vec3f(1,0,0);
  face_colors[1] = make_Vec3f(1,0,0);
  face_colors[2] = make_Vec3f(0,1,0);
  face_colors[3] = make_Vec3f(0,1,0);
  face_colors[4] = make_Vec3f(0.5f);
  face_colors[5] = make_Vec3f(0.5f);
  face_colors[6] = make_Vec3f(1.0f);
  face_colors[7] = make_Vec3f(1.0f);
  face_colors[8] = make_Vec3f(0,0,1);
  face_colors[9] = make_Vec3f(0,0,1);
  face_colors[10] = make_Vec3f(1,1,0);
  face_colors[11] = make_Vec3f(1,1,0);

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
  /* initialize ray tracing core */
  rtcInit(cfg);

  /* set error handler */
  rtcSetErrorFunction(error_handler);
 
  /* create scene */
  g_scene = rtcNewScene(RTC_SCENE_STATIC,RTC_INTERSECT_UNIFORM | RTC_INTERSECT_VARYING);

  /* add cube */
  addCube(g_scene);

  /* add ground plane */
  addGroundPlane(g_scene);

  /* commit changes to scene */
  rtcCommit (g_scene);

  /* set start render mode */
  renderPixel = renderPixelStandard;
}

uniform int frameID = 0;

/* task that renders a single screen tile */
Vec3f renderPixelStandard(float x, float y, const uniform Vec3f& vx, const uniform Vec3f& vy, const uniform Vec3f& vz, const uniform Vec3f& p)
{
  uniform float time = 0.5f+0.5f*sinf(0.05f*frameID);

  /* initialize ray */
  RTCRay ray;
  ray.org = p;
  ray.dir = normalize(x*vx + y*vy + vz);
  ray.tnear = 0.0f;
  ray.tfar = inf;
  ray.geomID = RTC_INVALID_GEOMETRY_ID;
  ray.primID = RTC_INVALID_GEOMETRY_ID;
  ray.mask = -1;
  ray.time = time;
  
  /* intersect ray with scene */
  rtcIntersect(g_scene,ray);
  
  /* shade pixels */
  Vec3f color = make_Vec3f(0.0f);
  if (ray.geomID != RTC_INVALID_GEOMETRY_ID) 
  {
    Vec3f diffuse = face_colors[ray.primID];
    color = color + diffuse*0.5f; // FIXME: +=
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
    shadow.time = time;
    
    /* trace shadow ray */
    rtcOccluded(g_scene,shadow);
    
    /* add light contribution */
    if (shadow.geomID)
      color = color + diffuse*clamp(-dot(lightDir,normalize(ray.Ng)),0.0f,1.0f); // FIXME: +=
  }
  return color;
}

/* task that renders a single screen tile */
task void renderTile(uniform int* uniform pixels,
                     const uniform int width,
                     const uniform int height, 
                     const uniform float time,
                     const uniform Vec3f& vx, 
                     const uniform Vec3f& vy, 
                     const uniform Vec3f& vz, 
                     const uniform Vec3f& p,
                     const uniform int numTilesX, 
                     const uniform int numTilesY)
{
  const uniform int tileY = taskIndex / numTilesX;
  const uniform int tileX = taskIndex - tileY * numTilesX;
  const uniform int x0 = tileX * TILE_SIZE_X;
  const uniform int x1 = min(x0+TILE_SIZE_X,width);
  const uniform int y0 = tileY * TILE_SIZE_Y;
  const uniform int y1 = min(y0+TILE_SIZE_Y,height);

  foreach_tiled (y = y0 ... y1, x = x0 ... x1)
  {
    /* calculate pixel color */
    Vec3f color = renderPixel(x,y,vx,vy,vz,p);

    /* write color to framebuffer */
    unsigned int r = (unsigned int) (255.0f * clamp(color.x,0.0f,1.0f));
    unsigned int g = (unsigned int) (255.0f * clamp(color.y,0.0f,1.0f));
    unsigned int b = (unsigned int) (255.0f * clamp(color.z,0.0f,1.0f));
    pixels[y*width+x] = (b << 16) + (g << 8) + r;  
  }
}

/* called by the C++ code to render */
export void device_render (uniform int* uniform pixels,
                    const uniform int width,
                    const uniform int height,
                    const uniform float time,
                    const uniform Vec3f& vx, 
                    const uniform Vec3f& vy, 
                    const uniform Vec3f& vz, 
                    const uniform Vec3f& p)
{
  frameID++;
  const uniform int numTilesX = (width +TILE_SIZE_X-1)/TILE_SIZE_X;
  const uniform int numTilesY = (height+TILE_SIZE_Y-1)/TILE_SIZE_Y;
  launch[numTilesX*numTilesY] renderTile(pixels,width,height,time,vx,vy,vz,p,numTilesX,numTilesY); sync;
}

/* called by the C++ code for cleanup */
export void device_cleanup ()
{
  rtcDeleteScene (g_scene);
  delete[] face_colors;
  rtcExit();
}

