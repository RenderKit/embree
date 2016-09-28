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

#include "../common/tutorial/tutorial_device.h"

namespace embree {

/* scene data */
RTCDevice g_device = nullptr;
RTCScene g_scene = nullptr;
Vec3fa* face_colors = nullptr;

/* accumulation buffer */
Vec3fa* g_accu = nullptr;
unsigned int g_accu_width = 0;
unsigned int g_accu_height = 0;
unsigned int g_accu_count = 0;
Vec3fa g_accu_vx;
Vec3fa g_accu_vy;
Vec3fa g_accu_vz;
Vec3fa g_accu_p;
extern "C" bool g_changed;

unsigned int numTimeSteps = 8;

__aligned(16) float cube_vertices[8][4] =
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

unsigned int cube_indices[36] = {
  1, 5, 4,  0, 1, 4,
  2, 6, 5,  1, 2, 5,
  3, 7, 6,  2, 3, 6,
  4, 7, 3,  0, 4, 3,
  5, 6, 7,  4, 5, 7,
  3, 2, 1,  0, 3, 1
};

unsigned int cube_quad_indices[24] = {
  0, 1, 5, 4,
  1, 2, 6, 5,
  2, 3, 7, 6,
  0, 4, 7, 3,
  4, 5, 6, 7,
  0, 3, 2, 1,
};

/* adds a cube to the scene */
unsigned int addTriangleCube (RTCScene scene, const Vec3fa& pos)
{
  /* create a triangulated cube with 12 triangles and 8 vertices */
  unsigned int geomID = rtcNewTriangleMesh (scene, RTC_GEOMETRY_STATIC, 12, 8, numTimeSteps);
  rtcSetBuffer(scene, geomID, RTC_INDEX_BUFFER,  cube_indices , 0, 3*sizeof(unsigned int));

  for (size_t t=0; t<numTimeSteps; t++) 
  {
    RTCBufferType bufID = (RTCBufferType)(RTC_VERTEX_BUFFER0+t);
    Vec3fa* vertices = (Vec3fa*) rtcMapBuffer(scene,geomID,bufID);

    AffineSpace3fa rotation = AffineSpace3fa::rotate(Vec3fa(0,0,0),Vec3fa(0,1,0),(float)t*float(pi)/(float)numTimeSteps);
    AffineSpace3fa scale = AffineSpace3fa::scale(Vec3fa(2.0f,1.0f,1.0f));
    
    for (int i=0; i<8; i++) {
      Vec3fa v = Vec3fa(cube_vertices[i][0],cube_vertices[i][1],cube_vertices[i][2]);
      vertices[i] = Vec3fa(xfmPoint(rotation*scale,v)+pos);
    }
    rtcUnmapBuffer(scene,geomID,bufID);
  }

  /* create face color array */
  /*face_colors = (Vec3fa*) alignedMalloc(12*sizeof(Vec3fa));
  face_colors[0] = Vec3fa(1,0,0);
  face_colors[1] = Vec3fa(1,0,0);
  face_colors[2] = Vec3fa(0,1,0);
  face_colors[3] = Vec3fa(0,1,0);
  face_colors[4] = Vec3fa(0.5f);
  face_colors[5] = Vec3fa(0.5f);
  face_colors[6] = Vec3fa(1.0f);
  face_colors[7] = Vec3fa(1.0f);
  face_colors[8] = Vec3fa(0,0,1);
  face_colors[9] = Vec3fa(0,0,1);
  face_colors[10] = Vec3fa(1,1,0);
  face_colors[11] = Vec3fa(1,1,0);*/
  return geomID;
}

/* adds a cube to the scene */
unsigned int addQuadCube (RTCScene scene, const Vec3fa& pos)
{
  /* create a quad cube with 6 quads and 8 vertices */
  unsigned int geomID = rtcNewQuadMesh (scene, RTC_GEOMETRY_STATIC, 6, 8, numTimeSteps);
  rtcSetBuffer(scene, geomID, RTC_INDEX_BUFFER,  cube_quad_indices , 0, 4*sizeof(unsigned int));
  
  for (size_t t=0; t<numTimeSteps; t++) 
  {
    RTCBufferType bufID = (RTCBufferType)(RTC_VERTEX_BUFFER0+t);
    Vec3fa* vertices = (Vec3fa*) rtcMapBuffer(scene,geomID,bufID);

    AffineSpace3fa rotation = AffineSpace3fa::rotate(Vec3fa(0,0,0),Vec3fa(0,1,0),(float)t*float(pi)/(float)numTimeSteps);
    AffineSpace3fa scale = AffineSpace3fa::scale(Vec3fa(2.0f,1.0f,1.0f));
    
    for (int i=0; i<8; i++) {
      Vec3fa v = Vec3fa(cube_vertices[i][0],cube_vertices[i][1],cube_vertices[i][2]);
      vertices[i] = Vec3fa(xfmPoint(rotation*scale,v)+pos);
    }
    rtcUnmapBuffer(scene,geomID,bufID);
  }

  /* create face color array */
  /*face_colors = (Vec3fa*) alignedMalloc(6*sizeof(Vec3fa));
  face_colors[0] = Vec3fa(1,0,0);
  face_colors[1] = Vec3fa(0,1,0);
  face_colors[2] = Vec3fa(0.5f);
  face_colors[3] = Vec3fa(1.0f);
  face_colors[4] = Vec3fa(0,0,1);
  face_colors[5] = Vec3fa(1,1,0);*/
  return geomID;
}

/* add hair geometry */
unsigned int addCurveOrHair (RTCScene scene, const Vec3fa& pos, bool curve)
{
  unsigned int geomID = 0;
  if (curve) 
    geomID = rtcNewCurveGeometry (scene, RTC_GEOMETRY_STATIC, 13, 4*13, numTimeSteps);
  else 
  {
    geomID = rtcNewHairGeometry (scene, RTC_GEOMETRY_STATIC, 13, 4*13, numTimeSteps);
    rtcSetTessellationRate (scene,geomID,16.0f);
  }

  Vec3fa* bspline = (Vec3fa*) alignedMalloc(16*sizeof(Vec3fa));
  for (int i=0; i<16; i++) {
    float f = (float)(i)/16.0f;
    bspline[i] = Vec3fa(4.0f*(f-0.5f),sin(12.0f*f),cos(12.0f*f));
  }
  Vec3fa* bezier = (Vec3fa*) alignedMalloc(4*13*sizeof(Vec3fa));
  for (int i=0; i<13; i++) {
    bezier[4*i+0] = (1.0f/6.0f)*bspline[i+0] + (2.0f/3.0f)*bspline[i+1] + (1.0f/6.0f)*bspline[i+2];
    bezier[4*i+1] = (2.0f/3.0f)*bspline[i+1] + (1.0f/3.0f)*bspline[i+2];
    bezier[4*i+2] = (1.0f/3.0f)*bspline[i+1] + (2.0f/3.0f)*bspline[i+2];
    bezier[4*i+3] = (1.0f/6.0f)*bspline[i+1] + (2.0f/3.0f)*bspline[i+2] + (1.0f/6.0f)*bspline[i+3];
  }

  for (size_t t=0; t<numTimeSteps; t++) 
  {
    RTCBufferType bufID = (RTCBufferType)(RTC_VERTEX_BUFFER0+t);
    Vec3fa* vertices = (Vec3fa*) rtcMapBuffer(scene,geomID,bufID);

    AffineSpace3fa rotation = AffineSpace3fa::rotate(Vec3fa(0,0,0),Vec3fa(0,1,0),(float)t*float(pi)/(float)numTimeSteps);
    AffineSpace3fa scale = AffineSpace3fa::scale(Vec3fa(2.0f,1.0f,1.0f));
    
    for (int i=0; i<4*13; i++)
      vertices[i] = Vec3fa(xfmPoint(rotation*scale,bezier[i])+pos,0.02f);

    rtcUnmapBuffer(scene,geomID,bufID);
  }

  int* indices = (int*) rtcMapBuffer(scene,geomID,RTC_INDEX_BUFFER);
  for (int i=0; i<13; i++) {
    indices[i] = 4*i;
  }
  rtcUnmapBuffer(scene,geomID,RTC_INDEX_BUFFER);

  alignedFree(bspline);
  alignedFree(bezier);
  return geomID;
}

/* adds a ground plane to the scene */
unsigned int addGroundPlane (RTCScene scene)
{
  /* create a triangulated plane with 2 triangles and 4 vertices */
  unsigned int mesh = rtcNewTriangleMesh (scene, RTC_GEOMETRY_STATIC, 2, 4);

  /* set vertices */
  Vertex* vertices = (Vertex*) rtcMapBuffer(scene,mesh,RTC_VERTEX_BUFFER);
  vertices[0].x = -10; vertices[0].y = -2; vertices[0].z = -10;
  vertices[1].x = -10; vertices[1].y = -2; vertices[1].z = +10;
  vertices[2].x = +10; vertices[2].y = -2; vertices[2].z = -10;
  vertices[3].x = +10; vertices[3].y = -2; vertices[3].z = +10;
  rtcUnmapBuffer(scene,mesh,RTC_VERTEX_BUFFER);

  /* set triangles */
  Triangle* triangles = (Triangle*) rtcMapBuffer(scene,mesh,RTC_INDEX_BUFFER);
  triangles[0].v0 = 0; triangles[0].v1 = 2; triangles[0].v2 = 1;
  triangles[1].v0 = 1; triangles[1].v1 = 2; triangles[1].v2 = 3;
  rtcUnmapBuffer(scene,mesh,RTC_INDEX_BUFFER);

  return mesh;
}

/* called by the C++ code for initialization */
extern "C" void device_init (char* cfg)
{
  /* initialize last seen camera */
  g_accu_vx = Vec3fa(0.0f);
  g_accu_vy = Vec3fa(0.0f);
  g_accu_vz = Vec3fa(0.0f);
  g_accu_p  = Vec3fa(0.0f);

  /* create new Embree device */
  g_device = rtcNewDevice(cfg);
  error_handler(rtcDeviceGetError(g_device));

  /* set error handler */
  rtcDeviceSetErrorFunction(g_device,error_handler);

  /* create scene */
  g_scene = rtcDeviceNewScene(g_device, RTC_SCENE_STATIC,RTC_INTERSECT1);

  /* add geometry to the scene */
  addTriangleCube(g_scene,Vec3fa(-5,3,-5));
  addQuadCube    (g_scene,Vec3fa( 0,3,-5));
  addCurveOrHair (g_scene,Vec3fa(-5,3, 0),false);
  addCurveOrHair (g_scene,Vec3fa( 0,3, 0),true);
  addGroundPlane(g_scene);

  /* commit changes to scene */
  rtcCommit (g_scene);

  /* set start render mode */
  renderTile = renderTileStandard;
  key_pressed_handler = device_key_pressed_default;
}

int frameID = 50;

/* task that renders a single screen tile */
Vec3fa renderPixelStandard(float x, float y, const ISPCCamera& camera)
{
  float time = abs((int)(0.01f*frameID) - 0.01f*frameID);

  /* initialize ray */
  RTCRay ray;
  ray.org = Vec3fa(camera.xfm.p);
  ray.dir = Vec3fa(normalize(x*camera.xfm.l.vx + y*camera.xfm.l.vy + camera.xfm.l.vz));
  ray.tnear = 0.0f;
  ray.tfar = inf;
  ray.geomID = RTC_INVALID_GEOMETRY_ID;
  ray.primID = RTC_INVALID_GEOMETRY_ID;
  ray.mask = -1;
  ray.time = time;

  /* intersect ray with scene */
  rtcIntersect(g_scene,ray);
  
  /* shade pixels */
  Vec3fa color = Vec3fa(0.0f);
  if (ray.geomID != RTC_INVALID_GEOMETRY_ID)
  {
    Vec3fa diffuse = Vec3fa(0.5f,0.5f,0.5f);
    //if (ray.geomID == 0) diffuse = face_colors[ray.primID];
    //else if (ray.geomID == 1) diffuse = Vec3fa(0.0f,1.0f,0.0f);
    //else diffuse = Vec3fa(0.5f,0.5f,0.5f);
    color = color + diffuse*0.5f;
    Vec3fa lightDir = normalize(Vec3fa(-1,-4,-1));

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
void renderTileStandard(int taskIndex,
                        int* pixels,
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
    /* calculate pixel color */
    Vec3fa color = renderPixelStandard((float)x,(float)y,camera);

    /* write color to framebuffer */
    Vec3fa accu_color = g_accu[y*width+x] + Vec3fa(color.x,color.y,color.z,1.0f); g_accu[y*width+x] = accu_color;
    float f = rcp(max(0.001f,accu_color.w));
    unsigned int r = (unsigned int) (255.0f * clamp(accu_color.x*f,0.0f,1.0f));
    unsigned int g = (unsigned int) (255.0f * clamp(accu_color.y*f,0.0f,1.0f));
    unsigned int b = (unsigned int) (255.0f * clamp(accu_color.z*f,0.0f,1.0f));
    pixels[y*width+x] = (b << 16) + (g << 8) + r;
  }
}

/* task that renders a single screen tile */
void renderTileTask (int taskIndex, int* pixels,
                         const unsigned int width,
                         const unsigned int height,
                         const float time,
                         const ISPCCamera& camera,
                         const int numTilesX,
                         const int numTilesY)
{
  renderTile(taskIndex,pixels,width,height,time,camera,numTilesX,numTilesY);
}

/* called by the C++ code to render */
extern "C" void device_render (int* pixels,
                           const unsigned int width,
                           const unsigned int height,
                           const float time,
                           const ISPCCamera& camera)
{
  /* create accumulator */
  if (g_accu_width != width || g_accu_height != height) {
    alignedFree(g_accu);
    g_accu = (Vec3fa*) alignedMalloc(width*height*sizeof(Vec3fa));
    g_accu_width = width;
    g_accu_height = height;
    memset(g_accu,0,width*height*sizeof(Vec3fa));
  }

  /* reset accumulator */
  bool camera_changed = g_changed; g_changed = false;
  camera_changed |= ne(g_accu_vx,camera.xfm.l.vx); g_accu_vx = camera.xfm.l.vx;
  camera_changed |= ne(g_accu_vy,camera.xfm.l.vy); g_accu_vy = camera.xfm.l.vy;
  camera_changed |= ne(g_accu_vz,camera.xfm.l.vz); g_accu_vz = camera.xfm.l.vz;
  camera_changed |= ne(g_accu_p, camera.xfm.p);    g_accu_p  = camera.xfm.p;
  //camera_changed = true;
  if (camera_changed) {
    g_accu_count=0;
    memset(g_accu,0,width*height*sizeof(Vec3fa));
  }

  /* render next frame */
  frameID++;
  const int numTilesX = (width +TILE_SIZE_X-1)/TILE_SIZE_X;
  const int numTilesY = (height+TILE_SIZE_Y-1)/TILE_SIZE_Y;
  parallel_for(size_t(0),size_t(numTilesX*numTilesY),[&](const range<size_t>& range) {
    for (size_t i=range.begin(); i<range.end(); i++)
      renderTileTask((int)i,pixels,width,height,time,camera,numTilesX,numTilesY);
  }); 
}

/* called by the C++ code for cleanup */
extern "C" void device_cleanup ()
{
  rtcDeleteScene (g_scene); g_scene = nullptr;
  rtcDeleteDevice(g_device); g_device = nullptr;
  alignedFree(face_colors); face_colors = nullptr;
  alignedFree(g_accu); g_accu = nullptr;
  g_accu_width = 0;
  g_accu_height = 0;
  g_accu_count = 0;
}

} // namespace embree
