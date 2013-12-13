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

#if 0
const uniform int numSpheres = 1000;
const uniform int numPhi = 5; 
#else
const uniform int numSpheres = 20;
const uniform int numPhi = 120; 
//const uniform int numPhi = 400; 
#endif
const uniform int numTheta = 2*numPhi;

/* scene data */
RTCScene scene = NULL;
uniform Vec3f position[numSpheres];
uniform float radius[numSpheres];
uniform int disabledID = -1;

/* render function to use */
Vec3f renderPixelNormal(int x, int y, const uniform Vec3f& vx, const uniform Vec3f& vy, const uniform Vec3f& vz, const uniform Vec3f& p);
Vec3f (* uniform renderPixel)(int x, int y, const uniform Vec3f& vx, const uniform Vec3f& vy, const uniform Vec3f& vz, const uniform Vec3f& p);

uniform unsigned int createSphere (uniform RTCGeometryFlags flags, const uniform Vec3f pos, const uniform float r)
{
  /* create a triangulated sphere */
  uniform unsigned int mesh = rtcNewTriangleMesh (scene, flags, 2*numTheta*(numPhi-1), numTheta*(numPhi+1));

  /* map triangle and vertex buffer */
  uniform Vertex*   uniform vertices  = (uniform Vertex*   uniform) rtcMapBuffer(scene,mesh,RTC_VERTEX_BUFFER); 
  uniform Triangle* uniform triangles = (uniform Triangle* uniform) rtcMapBuffer(scene,mesh,RTC_INDEX_BUFFER);
  
  /* create sphere geometry */
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
      v.x = pos.x + r*sin(phif)*sin(thetaf);
      v.y = pos.y + r*cos(phif);
      v.z = pos.z + r*sin(phif)*cos(thetaf);
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

/* called by the C++ code for initialization */
export void device_init (uniform int8* uniform cfg)
{
  /* initialize ray tracing core */
  rtcInit(cfg);

  /* create scene */
  scene = rtcNewScene(RTC_SCENE_DYNAMIC,RTC_INTERSECT_UNIFORM | RTC_INTERSECT_VARYING);

  /* create some triangulated spheres */
  for (uniform int i=0; i<numSpheres; i++)
  {
    const uniform float phi = i*2.0f*pi/numSpheres;
    const uniform float r = 2.0f*pi/numSpheres;
    const uniform Vec3f p = mul(2.0f,make_Vec3f(sin(phi),0.0f,-cos(phi)));
    uniform RTCGeometryFlags flags = i%2 ? RTC_GEOMETRY_DEFORMABLE : RTC_GEOMETRY_DYNAMIC;
    //uniform RTCGeometryFlags flags = RTC_GEOMETRY_DEFORMABLE;
    uniform int id = createSphere(flags,p,r);
    position[id] = p;
    radius[id] = r;
  }

  /* commit changes to scene */
  rtcCommit (scene);

  /* set start render mode */
  renderPixel = renderPixelNormal;
}

/* animates the sphere */
task void animateSphere (uniform Vertex* uniform vertices, 
                         const uniform float rcpNumTheta,
                         const uniform float rcpNumPhi,
                         const uniform Vec3f pos, 
                         const uniform float r,
                         const uniform float f)
{
  uniform int phi = taskIndex;
  foreach (theta = 0 ... numTheta)
  {
    uniform Vertex* v = &vertices[phi*numTheta+theta];
    const float phif   = phi*pi*rcpNumPhi;
    const float thetaf = theta*2.0f*pi*rcpNumTheta;
    v->x = pos.x + r*sin(f*phif)*sin(thetaf);
    v->y = pos.y + r*cos(phif);
    v->z = pos.z + r*sin(f*phif)*cos(thetaf);
  }
}

/* task that renders a single screen tile */
Vec3f renderPixelNormal(int x, int y, const uniform Vec3f& vx, const uniform Vec3f& vy, const uniform Vec3f& vz, const uniform Vec3f& p)
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
  rtcIntersect(scene,ray);
  
  /* shade pixels */
  if (ray.geomID == -1) return make_Vec3f(0.0f);
  else return make_Vec3f(abs(dot(normalize(ray.Ng),ray.dir)));
}

/* renders a single pixel with UV shading */
Vec3f renderPixelUV(int x, int y, const uniform Vec3f& vx, const uniform Vec3f& vy, const uniform Vec3f& vz, const uniform Vec3f& p)
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
  rtcIntersect(scene,ray);

  /* shade pixel */
  if (ray.geomID == -1) return make_Vec3f(0.0f);
  else return make_Vec3f(ray.u,ray.v,1.0f-ray.u-ray.v);
}

/* vizualizes the traversal cost of a pixel */
uniform float scale = 0.0001f;

Vec3f renderPixelCycles(int x, int y, const uniform Vec3f& vx, const uniform Vec3f& vy, const uniform Vec3f& vz, const uniform Vec3f& p)
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
  rtcIntersect(scene,ray);
  uniform int64 c1 = get_tsc();
  
  /* shade pixel */
  return make_Vec3f((uniform float)(c1-c0)*scale,0.0f,0.0f);
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
                                const uniform Vec3f& vx, 
                                const uniform Vec3f& vy, 
                                const uniform Vec3f& vz, 
                                const uniform Vec3f& p,
                                uniform Vec3f& hitPos)
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
  rtcIntersect1(scene,ray);

  /* shade pixel */
  if (ray.geomID == -1) {
    hitPos = make_Vec3f(0.0f,0.0f,0.0f);
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

  foreach (y = y0 ... y1, x = x0 ... x1)
  {
    /* calculate pixel color */
    Vec3f color = renderPixel(x,y,vx,vy,vz,p);

    /* write color to framebuffer */
    unsigned int r = (unsigned int) (255.0f * clamp(color.x,0,1));
    unsigned int g = (unsigned int) (255.0f * clamp(color.y,0,1));
    unsigned int b = (unsigned int) (255.0f * clamp(color.z,0,1));
    pixels[y*width+x] = (b << 16) + (g << 8) + r;
  }
}

/* animates a sphere */
void animateSphere (uniform int id, uniform float time)
{
  /* animate vertices */
  uniform Vertex* uniform vertices = (uniform Vertex* uniform) rtcMapBuffer(scene,id,RTC_VERTEX_BUFFER); 
  const uniform float rcpNumTheta = rcp(numTheta);
  const uniform float rcpNumPhi   = rcp(numPhi);
  const uniform Vec3f pos = position[id];
  const uniform float r = radius[id];
  const uniform float f = 2.0f*(1.0f+0.5f*sin(time));

  /* loop over all vertices */
#if 1
  launch[numPhi+1] animateSphere(vertices,rcpNumTheta,rcpNumPhi,pos,r,f); sync;
#else
  foreach (phi = 0 ... numPhi+1, theta = 0 ... numTheta)
  {
    uniform RTCVertex* v = &vertices[phi*numTheta+theta];
    const float phif   = phi*pi*rcpNumPhi;
    const float thetaf = theta*2.0f*pi*rcpNumTheta;
    v->x = pos.x+r*sin(f*phif)*sin(thetaf);
    v->y = pos.x+r*cos(phif);
    v->z = pos.x+r*sin(f*phif)*cos(thetaf);
  }
#endif

  rtcUnmapBuffer(scene,id,RTC_VERTEX_BUFFER); 

  /* update mesh */
  rtcUpdate (scene,id);
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
  /* animate sphere */
  for (uniform int i=0; i<numSpheres; i++)
    animateSphere(i,time+i);

  /* commit changes to scene */
  rtcCommit (scene);
 
  /* render all pixels */
  const uniform int numTilesX = (width +TILE_SIZE_X-1)/TILE_SIZE_X;
  const uniform int numTilesY = (height+TILE_SIZE_Y-1)/TILE_SIZE_Y;
  launch[numTilesX*numTilesY] renderTile(pixels,width,height,time,vx,vy,vz,p,numTilesX,numTilesY); sync;
  rtcDebug();
}

/* called by the C++ code for cleanup */
export void device_cleanup ()
{
  rtcDeleteScene (scene);
  rtcExit();
}
