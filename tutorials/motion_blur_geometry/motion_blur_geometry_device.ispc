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

/* scene data */
RTCDevice g_device = NULL;
RTCScene g_scene = NULL;
uniform Vec3f* uniform face_colors = NULL;

/* accumulation buffer */
uniform Vec3fa* uniform g_accu = NULL;
uniform unsigned int g_accu_width = 0;
uniform unsigned int g_accu_height = 0;
uniform unsigned int g_accu_count = 0;
uniform Vec3fa g_accu_vx;
uniform Vec3fa g_accu_vy;
uniform Vec3fa g_accu_vz;
uniform Vec3fa g_accu_p;
extern uniform bool g_changed;

uniform unsigned int numTimeSteps = 8;

__aligned(16) uniform float cube_vertices[8][4] =
{
  { -1.0f, -1.0f, -1.0f, 0.0f },
  {  1.0f, -1.0f, -1.0f, 0.0f },
  {  1.0f, -1.0f,  1.0f, 0.0f },
  { -1.0f, -1.0f,  1.0f, 0.0f },
  { -1.0f,  1.0f, -1.0f, 0.0f },
  {  1.0f,  1.0f, -1.0f, 0.0f },
  {  1.0f,  1.0f,  1.0f, 0.0f },
  { -1.0f,  1.0f,  1.0f, 0.0f }
};

uniform unsigned int cube_triangle_indices[36] = {
  1, 5, 4,  0, 1, 4,
  2, 6, 5,  1, 2, 5,
  3, 7, 6,  2, 3, 6,
  4, 7, 3,  0, 4, 3,
  5, 6, 7,  4, 5, 7,
  3, 2, 1,  0, 3, 1
};

uniform unsigned int cube_quad_indices[24] = {
  0, 1, 5, 4,
  1, 2, 6, 5,
  2, 3, 7, 6,
  0, 4, 7, 3,
  4, 5, 6, 7,
  0, 3, 2, 1,
};

__aligned(16) uniform float cube_vertex_crease_weights[8] = {
  inf, inf,inf, inf, inf, inf, inf, inf
};

__aligned(16) uniform unsigned int cube_vertex_crease_indices[8] = {
  0,1,2,3,4,5,6,7
};

__aligned(16) uniform float cube_edge_crease_weights[12] = {
  inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf
};

__aligned(16) uniform unsigned int cube_edge_crease_indices[24] =
{
  0,1, 1,2, 2,3, 3,0,
  4,5, 5,6, 6,7, 7,4,
  0,4, 1,5, 2,6, 3,7,
};

#define NUM_INDICES 24
#define NUM_FACES 6
#define FACE_SIZE 4

uniform unsigned int cube_quad_faces[6] = {
  4, 4, 4, 4, 4, 4
};

/* adds a cube to the scene */
uniform unsigned int addTriangleCube (RTCScene scene, const uniform Vec3f& pos)
{
  /* create a triangulated cube with 12 triangles and 8 vertices */
  uniform unsigned int geomID = rtcNewTriangleMesh (scene, RTC_GEOMETRY_STATIC, 12, 8, numTimeSteps);
  rtcSetBuffer(scene, geomID, RTC_INDEX_BUFFER,  cube_triangle_indices , 0, 3*sizeof(uniform unsigned int));

  for (uniform size_t t=0; t<numTimeSteps; t++) 
  {
    uniform RTCBufferType bufID = (uniform RTCBufferType)(RTC_VERTEX_BUFFER0+t);
    uniform Vec3fa* uniform vertices = (uniform Vec3fa* uniform) rtcMapBuffer(scene,geomID,bufID);

    uniform AffineSpace3f rotation = make_AffineSpace3f_rotate(make_Vec3f(0,0,0),make_Vec3f(0,1,0),(float)t*pi/(float)numTimeSteps);
    uniform AffineSpace3f scale = make_AffineSpace3f_scale(make_Vec3f(2.0f,1.0f,1.0f));
    
    for (uniform int i=0; i<8; i++) {
      uniform Vec3f v = make_Vec3f(cube_vertices[i][0],cube_vertices[i][1],cube_vertices[i][2]);
      vertices[i] = make_Vec3fa(xfmPoint(rotation*scale,v)+pos);
    }
    rtcUnmapBuffer(scene,geomID,bufID);
  }

  /* create face color array */
  /*face_colors = uniform new uniform Vec3f[12];
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
  face_colors[11] = make_Vec3f(1,1,0);*/
  return geomID;
}

/* adds a cube to the scene */
uniform unsigned int addQuadCube (RTCScene scene, const uniform Vec3f& pos)
{
  /* create a quad cube with 6 quads and 8 vertices */
  uniform unsigned int geomID = rtcNewQuadMesh (scene, RTC_GEOMETRY_STATIC, 6, 8, numTimeSteps);
  rtcSetBuffer(scene, geomID, RTC_INDEX_BUFFER,  cube_quad_indices , 0, 4*sizeof(uniform unsigned int));
  
  for (uniform size_t t=0; t<numTimeSteps; t++) 
  {
    uniform RTCBufferType bufID = (uniform RTCBufferType)(RTC_VERTEX_BUFFER0+t);
    uniform Vec3fa* uniform vertices = (uniform Vec3fa* uniform) rtcMapBuffer(scene,geomID,bufID);

    uniform AffineSpace3f rotation = make_AffineSpace3f_rotate(make_Vec3f(0,0,0),make_Vec3f(0,1,0),(float)t*pi/(float)numTimeSteps);
    uniform AffineSpace3f scale = make_AffineSpace3f_scale(make_Vec3f(2.0f,1.0f,1.0f));
    
    for (uniform int i=0; i<8; i++) {
      uniform Vec3f v = make_Vec3f(cube_vertices[i][0],cube_vertices[i][1],cube_vertices[i][2]);
      vertices[i] = make_Vec3fa(xfmPoint(rotation*scale,v)+pos);
    }
    rtcUnmapBuffer(scene,geomID,bufID);
  }

  /* create face color array */
  /*face_colors = uniform new uniform Vec3f[6];
  face_colors[0] = make_Vec3f(1,0,0);
  face_colors[1] = make_Vec3f(0,1,0);
  face_colors[2] = make_Vec3f(0.5f);
  face_colors[3] = make_Vec3f(1.0f);
  face_colors[4] = make_Vec3f(0,0,1);
  face_colors[5] = make_Vec3f(1,1,0);*/
  return geomID;
}

/* adds a subdivision cube to the scene */
uniform unsigned int addSubdivCube (RTCScene scene, const uniform Vec3f& pos)
{
  /* create a triangulated cube with 6 quads and 8 vertices */
  uniform unsigned int geomID = rtcNewSubdivisionMesh(scene, RTC_GEOMETRY_STATIC, NUM_FACES, NUM_INDICES, 8, 0, 0, 0, numTimeSteps);

  //rtcSetBuffer(scene, geomID, RTC_VERTEX_BUFFER, cube_vertices,  0, sizeof(uniform Vec3fa  ));
  rtcSetBuffer(scene, geomID, RTC_INDEX_BUFFER,  cube_quad_indices, 0, sizeof(uniform unsigned int));
  rtcSetBuffer(scene, geomID, RTC_FACE_BUFFER,   cube_quad_faces,0, sizeof(uniform unsigned int));

  rtcSetBuffer(scene, geomID, RTC_EDGE_CREASE_INDEX_BUFFER,   cube_edge_crease_indices,  0, 2*sizeof(uniform unsigned int));
  rtcSetBuffer(scene, geomID, RTC_EDGE_CREASE_WEIGHT_BUFFER,  cube_edge_crease_weights,  0, sizeof(uniform float));

  rtcSetBuffer(scene, geomID, RTC_VERTEX_CREASE_INDEX_BUFFER, cube_vertex_crease_indices,0, sizeof(uniform unsigned int));
  rtcSetBuffer(scene, geomID, RTC_VERTEX_CREASE_WEIGHT_BUFFER,cube_vertex_crease_weights,0, sizeof(uniform float));

  uniform float* uniform level = (uniform float* uniform) rtcMapBuffer(scene, geomID, RTC_LEVEL_BUFFER);
  for (uniform size_t i=0; i<NUM_INDICES; i++) level[i] = 16.0f;
  rtcUnmapBuffer(scene, geomID, RTC_LEVEL_BUFFER);

  for (uniform size_t t=0; t<numTimeSteps; t++) 
  {
    uniform RTCBufferType bufID = (uniform RTCBufferType)(RTC_VERTEX_BUFFER0+t);
    uniform Vec3fa* uniform vertices = (uniform Vec3fa* uniform) rtcMapBuffer(scene,geomID,bufID);

    uniform AffineSpace3f rotation = make_AffineSpace3f_rotate(make_Vec3f(0,0,0),make_Vec3f(0,1,0),(float)t*pi/(float)numTimeSteps);
    uniform AffineSpace3f scale = make_AffineSpace3f_scale(make_Vec3f(2.0f,1.0f,1.0f));
    
    for (uniform int i=0; i<8; i++) {
      uniform Vec3f v = make_Vec3f(cube_vertices[i][0],cube_vertices[i][1],cube_vertices[i][2]);
      vertices[i] = make_Vec3fa(xfmPoint(rotation*scale,v)+pos);
    }
    rtcUnmapBuffer(scene,geomID,bufID);
  }

  return geomID;
}

/* add hair geometry */
uniform unsigned int addCurveOrHair (RTCScene scene, const uniform Vec3f& pos, bool curve)
{
  uniform unsigned int geomID = 0;
  if (curve) 
    geomID = rtcNewCurveGeometry (scene, RTC_GEOMETRY_STATIC, 13, 4*13, numTimeSteps);
  else 
  {
    geomID = rtcNewHairGeometry (scene, RTC_GEOMETRY_STATIC, 13, 4*13, numTimeSteps);
    rtcSetTessellationRate (scene,geomID,16.0f);
  }

  uniform Vec3f* uniform bspline = uniform new uniform Vec3f[16];
  for (uniform int i=0; i<16; i++) {
    uniform float f = (uniform float)(i)/16.0f;
    bspline[i] = make_Vec3f(4.0f*(f-0.5f),sin(12.0f*f),cos(12.0f*f));
  }
  uniform Vec3f* uniform bezier = uniform new uniform Vec3f[4*13];
  for (uniform int i=0; i<13; i++) {
    bezier[4*i+0] = (1.0f/6.0f)*bspline[i+0] + (2.0f/3.0f)*bspline[i+1] + (1.0f/6.0f)*bspline[i+2];
    bezier[4*i+1] = (2.0f/3.0f)*bspline[i+1] + (1.0f/3.0f)*bspline[i+2];
    bezier[4*i+2] = (1.0f/3.0f)*bspline[i+1] + (2.0f/3.0f)*bspline[i+2];
    bezier[4*i+3] = (1.0f/6.0f)*bspline[i+1] + (2.0f/3.0f)*bspline[i+2] + (1.0f/6.0f)*bspline[i+3];
  }

  for (uniform size_t t=0; t<numTimeSteps; t++) 
  {
    uniform RTCBufferType bufID = (uniform RTCBufferType)(RTC_VERTEX_BUFFER0+t);
    uniform Vec3fa* uniform vertices = (uniform Vec3fa* uniform) rtcMapBuffer(scene,geomID,bufID);

    uniform AffineSpace3f rotation = make_AffineSpace3f_rotate(make_Vec3f(0,0,0),make_Vec3f(0,1,0),(float)t*pi/(float)numTimeSteps);
    uniform AffineSpace3f scale = make_AffineSpace3f_scale(make_Vec3f(2.0f,1.0f,1.0f));
    
    for (uniform int i=0; i<4*13; i++)
      vertices[i] = make_Vec3fa(xfmPoint(rotation*scale,bezier[i])+pos,0.02f);

    rtcUnmapBuffer(scene,geomID,bufID);
  }

  uniform int* uniform indices = (uniform int* uniform) rtcMapBuffer(scene,geomID,RTC_INDEX_BUFFER);
  for (uniform int i=0; i<13; i++) {
    indices[i] = 4*i;
  }
  rtcUnmapBuffer(scene,geomID,RTC_INDEX_BUFFER);

  delete[] bspline;
  delete[] bezier;
  return geomID;
}

/* add line geometry */
uniform unsigned int addLines (RTCScene scene, const uniform Vec3f& pos)
{
  uniform unsigned int geomID = rtcNewLineSegments (scene, RTC_GEOMETRY_STATIC, 15, 16, numTimeSteps);

  uniform Vec3f* uniform bspline = uniform new uniform Vec3f[16];
  for (uniform int i=0; i<16; i++) {
    uniform float f = (uniform float)(i)/16.0f;
    bspline[i] = make_Vec3f(4.0f*(f-0.5f),sin(12.0f*f),cos(12.0f*f));
  }

  for (uniform size_t t=0; t<numTimeSteps; t++) 
  {
    uniform RTCBufferType bufID = (uniform RTCBufferType)(RTC_VERTEX_BUFFER0+t);
    uniform Vec3fa* uniform vertices = (uniform Vec3fa* uniform) rtcMapBuffer(scene,geomID,bufID);

    uniform AffineSpace3f rotation = make_AffineSpace3f_rotate(make_Vec3f(0,0,0),make_Vec3f(0,1,0),(float)t*pi/(float)numTimeSteps);
    uniform AffineSpace3f scale = make_AffineSpace3f_scale(make_Vec3f(2.0f,1.0f,1.0f));
    
    for (uniform int i=0; i<16; i++)
      vertices[i] = make_Vec3fa(xfmPoint(rotation*scale,bspline[i])+pos,0.02f);

    rtcUnmapBuffer(scene,geomID,bufID);
  }

  uniform int* uniform indices = (uniform int* uniform) rtcMapBuffer(scene,geomID,RTC_INDEX_BUFFER);
  for (uniform int i=0; i<15; i++) {
    indices[i] = i;
  }
  rtcUnmapBuffer(scene,geomID,RTC_INDEX_BUFFER);

  delete[] bspline;
  return geomID;
}

/* adds an instanced triangle cube to the scene, rotate instance */
uniform unsigned int addInstancedTriangleCube (RTCScene global_scene, const uniform Vec3f& pos)
{
  RTCScene scene = rtcDeviceNewScene(g_device, RTC_SCENE_STATIC,RTC_INTERSECT_UNIFORM | RTC_INTERSECT_VARYING);
  uniform unsigned int meshID = rtcNewTriangleMesh (scene, RTC_GEOMETRY_STATIC, 12, 8, 1);
  rtcSetBuffer(scene, meshID, RTC_INDEX_BUFFER,  cube_triangle_indices , 0, 3*sizeof(uniform unsigned int));
  rtcSetBuffer(scene, meshID, RTC_VERTEX_BUFFER, cube_vertices, 0, 4*sizeof(uniform float));
  rtcCommit(scene);
  
  uniform unsigned int instID = rtcNewInstance2(global_scene,scene,numTimeSteps);

  for (uniform size_t t=0; t<numTimeSteps; t++)
  {
    uniform AffineSpace3f rotation = make_AffineSpace3f_rotate(make_Vec3f(0,0,0),make_Vec3f(0,1,0),(float)t*pi/(float)numTimeSteps);
    uniform AffineSpace3f scale = make_AffineSpace3f_scale(make_Vec3f(2.0f,1.0f,1.0f));
    uniform AffineSpace3f translation = make_AffineSpace3f_translate(pos);
    uniform AffineSpace3f xfm = translation*rotation*scale;
    rtcSetTransform2(global_scene,instID,RTC_MATRIX_COLUMN_MAJOR,(uniform float* uniform)&xfm,t);
  }
  return instID;
}

/* adds an instanced quad cube to the scene, rotate instance and geometry */
uniform unsigned int addInstancedQuadCube (RTCScene global_scene, const uniform Vec3f& pos)
{
  RTCScene scene = rtcDeviceNewScene(g_device, RTC_SCENE_STATIC,RTC_INTERSECT_UNIFORM | RTC_INTERSECT_VARYING);
  uniform unsigned int geomID = rtcNewQuadMesh (scene, RTC_GEOMETRY_STATIC, 6, 8, numTimeSteps);
  rtcSetBuffer(scene, geomID, RTC_INDEX_BUFFER,  cube_quad_indices , 0, 4*sizeof(uniform unsigned int));
  
  for (uniform size_t t=0; t<numTimeSteps; t++) 
  {
    uniform RTCBufferType bufID = (uniform RTCBufferType)(RTC_VERTEX_BUFFER0+t);
    uniform Vec3fa* uniform vertices = (uniform Vec3fa* uniform) rtcMapBuffer(scene,geomID,bufID);

    uniform AffineSpace3f rotation = make_AffineSpace3f_rotate(make_Vec3f(0,0,0),make_Vec3f(0,1,0),0.5f*(float)t*pi/(float)numTimeSteps);
    uniform AffineSpace3f scale = make_AffineSpace3f_scale(make_Vec3f(2.0f,1.0f,1.0f));
    
    for (uniform int i=0; i<8; i++) {
      uniform Vec3f v = make_Vec3f(cube_vertices[i][0],cube_vertices[i][1],cube_vertices[i][2]);
      vertices[i] = make_Vec3fa(xfmPoint(rotation*scale,v));
    }
    rtcUnmapBuffer(scene,geomID,bufID);
  }
  rtcCommit(scene);
  
  uniform unsigned int instID = rtcNewInstance2(global_scene,scene,numTimeSteps);

  for (uniform size_t t=0; t<numTimeSteps; t++)
  {
    uniform AffineSpace3f rotation = make_AffineSpace3f_rotate(make_Vec3f(0,0,0),make_Vec3f(0,1,0),0.5f*(float)t*pi/(float)numTimeSteps);
    uniform AffineSpace3f translation = make_AffineSpace3f_translate(pos);
    uniform AffineSpace3f xfm = translation*rotation;
    rtcSetTransform2(global_scene,instID,RTC_MATRIX_COLUMN_MAJOR,(uniform float* uniform)&xfm,t);
  }
  return instID;
}

/* adds a ground plane to the scene */
uniform unsigned int addGroundPlane (RTCScene scene)
{
  /* create a triangulated plane with 2 triangles and 4 vertices */
  uniform unsigned int mesh = rtcNewTriangleMesh (scene, RTC_GEOMETRY_STATIC, 2, 4);

  /* set vertices */
  uniform Vertex* uniform vertices = (uniform Vertex* uniform) rtcMapBuffer(scene,mesh,RTC_VERTEX_BUFFER);
  vertices[0].x = -10; vertices[0].y = -2; vertices[0].z = -10;
  vertices[1].x = -10; vertices[1].y = -2; vertices[1].z = +10;
  vertices[2].x = +10; vertices[2].y = -2; vertices[2].z = -10;
  vertices[3].x = +10; vertices[3].y = -2; vertices[3].z = +10;
  rtcUnmapBuffer(scene,mesh,RTC_VERTEX_BUFFER);

  /* set triangles */
  uniform Triangle* uniform triangles = (uniform Triangle* uniform) rtcMapBuffer(scene,mesh,RTC_INDEX_BUFFER);
  triangles[0].v0 = 0; triangles[0].v1 = 2; triangles[0].v2 = 1;
  triangles[1].v0 = 1; triangles[1].v1 = 2; triangles[1].v2 = 3;
  rtcUnmapBuffer(scene,mesh,RTC_INDEX_BUFFER);

  return mesh;
}

/* called by the C++ code for initialization */
export void device_init (uniform int8* uniform cfg)
{
  /* initialize last seen camera */
  g_accu_vx = make_Vec3fa(0.0f);
  g_accu_vy = make_Vec3fa(0.0f);
  g_accu_vz = make_Vec3fa(0.0f);
  g_accu_p  = make_Vec3fa(0.0f);

  /* create new Embree device */
  g_device = rtcNewDevice(cfg);
  error_handler(rtcDeviceGetError(g_device));

  /* set error handler */
  rtcDeviceSetErrorFunction(g_device,error_handler);

  /* create scene */
  g_scene = rtcDeviceNewScene(g_device, RTC_SCENE_STATIC,RTC_INTERSECT_UNIFORM | RTC_INTERSECT_VARYING);

  /* add geometry to the scene */
  addTriangleCube(g_scene,make_Vec3f(-5,3,-5));
  addQuadCube    (g_scene,make_Vec3f( 0,3,-5));
  addSubdivCube  (g_scene,make_Vec3f(+5,3,-5));
  addLines       (g_scene,make_Vec3f(-5,3, 0));
  addCurveOrHair (g_scene,make_Vec3f( 0,3, 0),false);
  addCurveOrHair (g_scene,make_Vec3f(+5,3, 0),true);
  addInstancedTriangleCube(g_scene,make_Vec3f(-5,3,+5));
  addInstancedQuadCube    (g_scene,make_Vec3f( 0,3,+5));
  addGroundPlane(g_scene);

  /* commit changes to scene */
  rtcCommit (g_scene);

  /* set start render mode */
  renderTile = renderTileStandard;
  key_pressed_handler = device_key_pressed_default;
}

uniform int frameID = 50;

/* task that renders a single screen tile */
Vec3f renderPixelStandard(float x, float y, const uniform ISPCCamera& camera)
{
  uniform float time = abs((uniform int)(0.01f*frameID) - 0.01f*frameID);

  /* initialize ray */
  RTCRay ray;
  ray.org = make_Vec3f(camera.xfm.p);
  ray.dir = make_Vec3f(normalize(x*camera.xfm.l.vx + y*camera.xfm.l.vy + camera.xfm.l.vz));
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
    Vec3f diffuse = make_Vec3f(0.5f,0.5f,0.5f);
    //if (ray.geomID == 0) diffuse = face_colors[ray.primID];
    //else if (ray.geomID == 1) diffuse = make_Vec3f(0.0f,1.0f,0.0f);
    //else diffuse = make_Vec3f(0.5f,0.5f,0.5f);
    color = color + diffuse*0.5f;
    Vec3f lightDir = normalize(make_Vec3f(-1,-4,-1));

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
      color = color + diffuse*clamp(-dot(lightDir,normalize(ray.Ng)),0.0f,1.0f);
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
    Vec3fa accu_color = g_accu[y*width+x] + make_Vec3fa(color.x,color.y,color.z,1.0f); g_accu[y*width+x] = accu_color;
    float f = rcp(max(0.001f,accu_color.w));
    unsigned int r = (unsigned int) (255.0f * clamp(accu_color.x*f,0.0f,1.0f));
    unsigned int g = (unsigned int) (255.0f * clamp(accu_color.y*f,0.0f,1.0f));
    unsigned int b = (unsigned int) (255.0f * clamp(accu_color.z*f,0.0f,1.0f));
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
  /* create accumulator */
  if (g_accu_width != width || g_accu_height != height) {
    delete[] g_accu;
    g_accu = uniform new uniform Vec3fa[width*height];
    g_accu_width = width;
    g_accu_height = height;
    memset(g_accu,0,width*height*sizeof(uniform Vec3fa));
  }

  /* reset accumulator */
  uniform bool camera_changed = g_changed; g_changed = false;
  camera_changed |= ne(g_accu_vx,camera.xfm.l.vx); g_accu_vx = camera.xfm.l.vx;
  camera_changed |= ne(g_accu_vy,camera.xfm.l.vy); g_accu_vy = camera.xfm.l.vy;
  camera_changed |= ne(g_accu_vz,camera.xfm.l.vz); g_accu_vz = camera.xfm.l.vz;
  camera_changed |= ne(g_accu_p, camera.xfm.p);    g_accu_p  = camera.xfm.p;
  //camera_changed = true;
  if (camera_changed) {
    g_accu_count=0;
    memset(g_accu,0,width*height*sizeof(uniform Vec3fa));
  }

  /* render next frame */
  frameID++;
  const uniform int numTilesX = (width +TILE_SIZE_X-1)/TILE_SIZE_X;
  const uniform int numTilesY = (height+TILE_SIZE_Y-1)/TILE_SIZE_Y;
  launch[numTilesX*numTilesY] renderTileTask(pixels,width,height,time,camera,numTilesX,numTilesY); sync;
}

/* called by the C++ code for cleanup */
export void device_cleanup ()
{
  rtcDeleteScene (g_scene); g_scene = NULL;
  rtcDeleteDevice(g_device); g_device = NULL;
  delete[] face_colors; face_colors = NULL;
  delete[] g_accu; g_accu = NULL;
  g_accu_width = 0;
  g_accu_height = 0;
  g_accu_count = 0;
}
