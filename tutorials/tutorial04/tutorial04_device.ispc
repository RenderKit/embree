// ======================================================================== //
// Copyright 2009-2013 Intel Corporation                                    //
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

/* render function to use */
vec3f renderPixelNormal(int x, int y, const uniform vec3f& vx, const uniform vec3f& vy, const uniform vec3f& vz, const uniform vec3f& p);
vec3f (* uniform renderPixel)(int x, int y, const uniform vec3f& vx, const uniform vec3f& vy, const uniform vec3f& vz, const uniform vec3f& p);

const uniform int numPhi = 5;
const uniform int numTheta = 2*numPhi;

uniform unsigned int createTriangulatedSphere (RTCScene scene, uniform vec3f p, uniform float r)
{
  /* create triangle mesh */
  uniform unsigned int mesh = rtcNewTriangleMesh (scene, RTC_STATIC, 2*numTheta*(numPhi-1), numTheta*(numPhi+1));
  
  /* map triangle and vertex buffers */
  uniform Vertex* uniform vertices = (uniform Vertex* uniform) rtcMapBuffer(scene,mesh,RTC_VERTEX_BUFFER); 
  uniform Triangle* uniform triangles = (uniform Triangle* uniform) rtcMapBuffer(scene,mesh,RTC_INDEX_BUFFER);
  
  /* create sphere */
  uniform int tri = 0;
  const uniform float rcpNumTheta = rcp(numTheta);
  const uniform float rcpNumPhi   = rcp(numPhi);
  for (uniform int phi=0; phi<=numPhi; phi++)
  {
    for (uniform int theta=0; theta<numTheta; theta++)
    {
      const uniform float phif   = phi*pi*rcpNumPhi;
      const uniform float thetaf = theta*2.0f*pi*rcpNumTheta;

      uniform Vertex& v = vertices[phi*numTheta+theta];
      v.x = p.x + r*sin(phif)*sin(thetaf);
      v.y = p.y + r*cos(phif);
      v.z = p.z + r*sin(phif)*cos(thetaf);
    }
    if (phi == 0) continue;

    for (uniform int theta=1; theta<=numTheta; theta++) 
    {
      uniform int p00 = (phi-1)*numTheta+theta-1;
      uniform int p01 = (phi-1)*numTheta+theta%numTheta;
      uniform int p10 = phi*numTheta+theta-1;
      uniform int p11 = phi*numTheta+theta%numTheta;

      if (phi > 1) {
        triangles[tri].v0 = p10; 
        triangles[tri].v1 = p00; 
        triangles[tri].v2 = p01; 
        tri++;
      }

      if (phi < numPhi) {
        triangles[tri].v0 = p11; 
        triangles[tri].v1 = p10;
        triangles[tri].v2 = p01;
        tri++;
      }
    }
  }
  rtcUnmapBuffer(scene,mesh,RTC_VERTEX_BUFFER); 
  rtcUnmapBuffer(scene,mesh,RTC_INDEX_BUFFER);
  return mesh;
}

/* creates a ground plane */
uniform unsigned int createGroundPlane (RTCScene scene)
{
  /* create a triangulated plane with 2 triangles and 4 vertices */
  uniform unsigned int mesh = rtcNewTriangleMesh (scene, RTC_STATIC, 2, 4);

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

/* scene data */
RTCScene g_scene  = NULL;
RTCScene g_scene1 = NULL;

uniform unsigned int g_instance0 = -1;
uniform unsigned int g_instance1 = -1;
uniform unsigned int g_instance2 = -1;
uniform unsigned int g_instance3 = -1;

uniform vec3f colors[4][4];

/* called by the C++ code for initialization */
export void device_init (uniform int8* uniform cfg)
{
  /* initialize ray tracing core */
  rtcInit(cfg);

  /* create scene */
  g_scene = rtcNewScene(RTC_DYNAMIC,RTC_INTERSECT_UNIFORM | RTC_INTERSECT_VARYING);

  /* create scene with 4 triangulated spheres */
  g_scene1 = rtcNewScene(RTC_STATIC,RTC_INTERSECT_UNIFORM | RTC_INTERSECT_VARYING);
  createTriangulatedSphere(g_scene1,make_vec3f( 0, 0,+1),0.5);
  createTriangulatedSphere(g_scene1,make_vec3f(+1, 0, 0),0.5);
  createTriangulatedSphere(g_scene1,make_vec3f( 0, 0,-1),0.5);
  createTriangulatedSphere(g_scene1,make_vec3f(-1, 0, 0),0.5);
  rtcCommit(g_scene1);

  /* instantiate geometry */
  g_instance0 = rtcNewInstance(g_scene,g_scene1);
  g_instance1 = rtcNewInstance(g_scene,g_scene1);
  g_instance2 = rtcNewInstance(g_scene,g_scene1);
  g_instance3 = rtcNewInstance(g_scene,g_scene1);
  createGroundPlane(g_scene);

  /* set all colors */
  colors[0][0] = make_vec3f(0.25,0,0);
  colors[0][1] = make_vec3f(0.50,0,0);
  colors[0][2] = make_vec3f(0.75,0,0);
  colors[0][3] = make_vec3f(1.00,0,0);

  colors[1][0] = make_vec3f(0,0.25,0);
  colors[1][1] = make_vec3f(0,0.50,0);
  colors[1][2] = make_vec3f(0,0.75,0);
  colors[1][3] = make_vec3f(0,1.00,0);

  colors[2][0] = make_vec3f(0,0,0.25);
  colors[2][1] = make_vec3f(0,0,0.50);
  colors[2][2] = make_vec3f(0,0,0.75);
  colors[2][3] = make_vec3f(0,0,1.00);

  colors[3][0] = make_vec3f(0.25,0.25,0);
  colors[3][1] = make_vec3f(0.50,0.50,0);
  colors[3][2] = make_vec3f(0.75,0.75,0);
  colors[3][3] = make_vec3f(1.00,1.00,0);

  /* set start render mode */
  renderPixel = renderPixelNormal;
}

/* task that renders a single screen tile */
vec3f renderPixelNormal(int x, int y, const uniform vec3f& vx, const uniform vec3f& vy, const uniform vec3f& vz, const uniform vec3f& p)
{
  /* initialize ray */
  RTCRay ray;
  ray.org = p;
  ray.dir = normalize(add(mul(x,vx), mul(y,vy), vz));
  ray.tnear = 0.0f;
  ray.tfar = inf;
  ray.geomID = -1;
  ray.primID = -1;
  ray.instID = -1;
  ray.mask = -1;
  ray.time = 0;

  /* intersect ray with scene */
  rtcIntersect(g_scene,ray);
  
  /* shade pixels */
  vec3f color = make_vec3f(0.0f);
  if (ray.geomID != -1) 
  {
    vec3f diffuse = make_vec3f(1,1,1);
    if (ray.instID != -1)
      diffuse = colors[ray.instID][ray.geomID];
    color = add(color,mul(diffuse,0.5));
    vec3f lightDir = normalize(make_vec3f(-1,-1,-1));
    
    /* initialize shadow ray */
    RTCRay shadow;
    shadow.org = add(ray.org,mul(ray.tfar,ray.dir));
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
    if (shadow.geomID)
      color = add(color,mul(diffuse,clamp(-dot(lightDir,normalize(ray.Ng)),0,1)));
  }
  return color;
}

/* renders a single pixel with UV shading */
vec3f renderPixelUV(int x, int y, const uniform vec3f& vx, const uniform vec3f& vy, const uniform vec3f& vz, const uniform vec3f& p)
{
  /* initialize ray */
  RTCRay ray;
  ray.org = p;
  ray.dir = normalize(add(mul(x,vx), mul(y,vy), vz));
  ray.tnear = 0.0f;
  ray.tfar = inf;
  ray.geomID = -1;
  ray.primID = -1;
  ray.mask = -1;
  ray.time = 0;

  /* intersect ray with scene */
  rtcIntersect(g_scene,ray);

  /* shade pixel */
  if (ray.geomID == -1) return make_vec3f(0.0f);
  else return make_vec3f(ray.u,ray.v,1.0f-ray.u-ray.v);
}

/* vizualizes the traversal cost of a pixel */
uniform float scale = 0.0001f;

vec3f renderPixelCycles(int x, int y, const uniform vec3f& vx, const uniform vec3f& vy, const uniform vec3f& vz, const uniform vec3f& p)
{
  /* initialize ray */
  RTCRay ray;
  ray.org = p;
  ray.dir = normalize(add(mul(x,vx), mul(y,vy), vz));
  ray.tnear = 0.0f;
  ray.tfar = inf;
  ray.geomID = -1;
  ray.primID = -1;
  ray.mask = -1;
  ray.time = 0;

  /* intersect ray with scene */
  uniform int64 c0 = get_tsc();
  rtcIntersect(g_scene,ray);
  uniform int64 c1 = get_tsc();
  
  /* shade pixel */
  return make_vec3f((uniform float)(c1-c0)*scale,0.0f,0.0f);
}

/* called when a key is pressed */
export void device_key_pressed(uniform int key)
{
  if      (key == GLUT_KEY_F1) renderPixel = renderPixelNormal;
  else if (key == GLUT_KEY_F2) renderPixel = renderPixelUV;
  else if (key == GLUT_KEY_F3) {
    if (renderPixel == renderPixelCycles) scale *= 1.1f;
    renderPixel = renderPixelCycles;
  }
  else if (key == GLUT_KEY_F4) {
    if (renderPixel == renderPixelCycles) scale *= 0.9f;
    renderPixel = renderPixelCycles;
  }
}

/* returns the point seen through specified pixel */
export uniform bool device_pick(const uniform float x,
                                const uniform float y, 
                                const uniform vec3f& vx, 
                                const uniform vec3f& vy, 
                                const uniform vec3f& vz, 
                                const uniform vec3f& p,
                                uniform vec3f& hitPos)
{
  /* initialize ray */
  uniform RTCRay1 ray;
  ray.org = p;
  ray.dir = normalize(add(mul(x,vx), mul(y,vy), vz));
  ray.tnear = 0.0f;
  ray.tfar = inf;
  ray.geomID = -1;
  ray.primID = -1;
  ray.mask = -1;
  ray.time = 0;

  /* intersect ray with scene */
  rtcIntersect1(g_scene,ray);

  /* shade pixel */
  if (ray.geomID == -1) {
    hitPos = make_vec3f(0.0f,0.0f,0.0f);
    return false;
  }
  else {
    hitPos = add(ray.org,mul(ray.tfar,ray.dir));
    return true;
  }
}

/* task that renders a single screen tile */
task void renderTile(uniform int* uniform pixels,
                     const uniform int width,
                     const uniform int height, 
                     const uniform float time,
                     const uniform vec3f& vx, 
                     const uniform vec3f& vy, 
                     const uniform vec3f& vz, 
                     const uniform vec3f& p,
                     const uniform int numTilesX, 
                     const uniform int numTilesY)
{
  const uniform int tileY = taskIndex / numTilesX;
  const uniform int tileX = taskIndex - tileY * numTilesX;
  const uniform int x0 = tileX * TILE_SIZE_X;
  const uniform int x1 = min(x0+TILE_SIZE_X,width);
  const uniform int y0 = tileY * TILE_SIZE_Y;
  const uniform int y1 = min(y0+TILE_SIZE_Y,height);

  foreach (y = y0 ... y1, x = x0 ... x1)
  {
    /* calculate pixel color */
    vec3f color = renderPixel(x,y,vx,vy,vz,p);

    /* write color to framebuffer */
    unsigned int r = (unsigned int) (255.0f * clamp(color.x,0,1));
    unsigned int g = (unsigned int) (255.0f * clamp(color.y,0,1));
    unsigned int b = (unsigned int) (255.0f * clamp(color.z,0,1));
    pixels[y*width+x] = (b << 16) + (g << 8) + r;
  }
}

/* called by the C++ code to render */
export void device_render (uniform int* uniform pixels,
                           const uniform int width,
                           const uniform int height, 
                           const uniform float time,
                           const uniform vec3f& vx, 
                           const uniform vec3f& vy, 
                           const uniform vec3f& vz, 
                           const uniform vec3f& p)
{
  /* create identity matrix */
  uniform AffineSpace3f xfm;
  xfm.l.vx = make_vec3f(1,0,0);
  xfm.l.vy = make_vec3f(0,1,0);
  xfm.l.vz = make_vec3f(0,0,1);
  xfm.p    = make_vec3f(0,0,0);
  uniform float t = 0.7f*time;

  /* move instances */
  xfm.p = mul(2.0f,make_vec3f(+cos(t),0.0f,+sin(t)));
  rtcSetTransform(g_scene,g_instance0,RTC_MATRIX_COLUMN_MAJOR,(uniform float* uniform)&xfm);
  xfm.p = mul(2.0f,make_vec3f(-cos(t),0.0f,-sin(t)));
  rtcSetTransform(g_scene,g_instance1,RTC_MATRIX_COLUMN_MAJOR,(uniform float* uniform)&xfm);
  xfm.p = mul(2.0f,make_vec3f(-sin(t),0.0f,+cos(t)));
  rtcSetTransform(g_scene,g_instance2,RTC_MATRIX_COLUMN_MAJOR,(uniform float* uniform)&xfm);
  xfm.p = mul(2.0f,make_vec3f(+sin(t),0.0f,-cos(t)));
  rtcSetTransform(g_scene,g_instance3,RTC_MATRIX_COLUMN_MAJOR,(uniform float* uniform)&xfm);

  /* update scene */
  rtcUpdate(g_scene,g_instance0);
  rtcUpdate(g_scene,g_instance1);
  rtcUpdate(g_scene,g_instance2);
  rtcUpdate(g_scene,g_instance3);
  rtcCommit (g_scene);

  /* render all pixels */
  const uniform int numTilesX = (width +TILE_SIZE_X-1)/TILE_SIZE_X;
  const uniform int numTilesY = (height+TILE_SIZE_Y-1)/TILE_SIZE_Y;
  launch[numTilesX*numTilesY] renderTile(pixels,width,height,time,vx,vy,vz,p,numTilesX,numTilesY); sync;
  rtcDebug();
}

/* called by the C++ code for cleanup */
export void device_cleanup ()
{
  rtcDeleteScene (g_scene);
  rtcDeleteScene (g_scene1);
  rtcExit();
}
