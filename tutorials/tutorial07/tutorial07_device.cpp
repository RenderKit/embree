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

#include "../common/tutorial/tutorial_device.h"

#define USE_INTERSECTION_FILTER 0
#define USE_OCCLUSION_FILTER 0

Vec3fa lightDir = normalize(-Vec3fa(-20.6048, 22.2367, -2.93452));
Vec3fa lightIntensity = Vec3fa(4.0f);

struct ISPCTriangle 
{
  int v0;                /*< first triangle vertex */
  int v1;                /*< second triangle vertex */
  int v2;                /*< third triangle vertex */
  int materialID;        /*< material of triangle */
};

struct ISPCMaterial
{
  int illum;             /*< illumination model */
  
  float d;               /*< dissolve factor, 1=opaque, 0=transparent */
  float Ns;              /*< specular exponent */
  float Ni;              /*< optical density for the surface (index of refraction) */
  
  Vec3f Ka;              /*< ambient reflectivity */
  Vec3f Kd;              /*< diffuse reflectivity */
  Vec3f Ks;              /*< specular reflectivity */
  Vec3f Tf;              /*< transmission filter */
};

struct ISPCHair
{
  Vec3fa* v;     //!< hair control points (x,y,z,r)
  int* index;    //!< for each hair, index to first control point
  int numVertices;
  int numHairs;
};

struct ISPCMesh
{
  Vec3fa* positions;    //!< vertex position array
  Vec3fa* normals;       //!< vertex normal array
  Vec2f* texcoords;     //!< vertex texcoord array
  ISPCTriangle* triangles;  //!< list of triangles
  int numVertices;
  int numTriangles;
};

struct ISPCScene
{
  ISPCMesh** meshes;         //!< list of meshes
  ISPCMaterial* materials;  //!< material list
  int numMeshes;
  int numMaterials;
  ISPCHair** hairs;
  int numHairSets;
};

/* scene data */
extern "C" ISPCScene* g_ispc_scene;
RTCScene g_scene = NULL;
Vec3f* colors = NULL;

/* render function to use */
renderPixelFunc renderPixel;
float T_hair = 0.3f;

//Vertex* vertices = NULL;
//int*    indices = NULL;

__forceinline Vec3fa evalBezier(const int geomID, const int primID, const float t)
{
  const float t0 = 1.0f - t, t1 = t;
  const ISPCHair* hair = g_ispc_scene->hairs[geomID]; // FIXME: works only because hairs are added first to scene
  const Vec3fa* vertices = hair->v;
  const int* indices = hair->index;
  
  const int i = indices[primID];
  const Vec3fa p00 = *(Vec3fa*)&vertices[i+0];
  const Vec3fa p01 = *(Vec3fa*)&vertices[i+1];
  const Vec3fa p02 = *(Vec3fa*)&vertices[i+2];
  const Vec3fa p03 = *(Vec3fa*)&vertices[i+3];

  const Vec3fa p10 = p00 * t0 + p01 * t1;
  const Vec3fa p11 = p01 * t0 + p02 * t1;
  const Vec3fa p12 = p02 * t0 + p03 * t1;
  const Vec3fa p20 = p10 * t0 + p11 * t1;
  const Vec3fa p21 = p11 * t0 + p12 * t1;
  const Vec3fa p30 = p20 * t0 + p21 * t1;
  
  return p30;
  //tangent = p21-p20;
}

struct HitList;

/* extended ray structure that includes total transparency along the ray */
struct RTCRay2
{
  Vec3fa org;     //!< Ray origin
  Vec3fa dir;     //!< Ray direction
  float tnear;   //!< Start of ray segment
  float tfar;    //!< End of ray segment
  float time;    //!< Time of this ray for motion blur.
  int mask;      //!< used to mask out objects during traversal
  Vec3fa Ng;      //!< Geometric normal.
  float u;       //!< Barycentric u coordinate of hit
  float v;       //!< Barycentric v coordinate of hit
  int geomID;    //!< geometry ID
  int primID;    //!< primitive ID
  int instID;    //!< instance ID

  // ray extensions
  RTCFilterFunc filter;
  float transparency; //!< accumulated transparency value
  HitList* list;
};

struct HitList
{
  RTCRay2 data[128];
  RTCRay2* rays[128];
  size_t num;
};

bool addHit(HitList* list, RTCRay2& ray)
{
  if (list->num >= 128) 
    return false;

  int i = list->num++;
  list->data[i] = ray;
  RTCRay2* r = &list->data[i];
  while (i>0 && list->rays[i-1]->tfar > r->tfar) {
    list->rays[i] = list->rays[i-1];
    i--;
  }
  list->rays[i] = r;
  return true;
}

/* adds a cube to the scene */
unsigned int addCube (RTCScene scene_i)
{
  /* create a triangulated cube with 12 triangles and 8 vertices */
  unsigned int mesh = rtcNewTriangleMesh (scene_i, RTC_GEOMETRY_STATIC, 12, 8);

  /* set vertices */
  Vertex* vertices = (Vertex*) rtcMapBuffer(scene_i,mesh,RTC_VERTEX_BUFFER); 
  vertices[0].x = -1; vertices[0].y = -1; vertices[0].z = -1; 
  vertices[1].x = -1; vertices[1].y = -1; vertices[1].z = +1; 
  vertices[2].x = -1; vertices[2].y = +1; vertices[2].z = -1; 
  vertices[3].x = -1; vertices[3].y = +1; vertices[3].z = +1; 
  vertices[4].x = +1; vertices[4].y = -1; vertices[4].z = -1; 
  vertices[5].x = +1; vertices[5].y = -1; vertices[5].z = +1; 
  vertices[6].x = +1; vertices[6].y = +1; vertices[6].z = -1; 
  vertices[7].x = +1; vertices[7].y = +1; vertices[7].z = +1; 
  rtcUnmapBuffer(scene_i,mesh,RTC_VERTEX_BUFFER); 

  /* create triangle color array */
  colors = new Vec3f[12];

  /* set triangles and colors */
  int tri = 0;
  Triangle* triangles = (Triangle*) rtcMapBuffer(scene_i,mesh,RTC_INDEX_BUFFER);
  
  // left side
  colors[tri] = Vec3f(1,0,0); triangles[tri].v0 = 0; triangles[tri].v1 = 2; triangles[tri].v2 = 1; tri++;
  colors[tri] = Vec3f(1,0,0); triangles[tri].v0 = 1; triangles[tri].v1 = 2; triangles[tri].v2 = 3; tri++;

  // right side
  colors[tri] = Vec3f(0,1,0); triangles[tri].v0 = 4; triangles[tri].v1 = 5; triangles[tri].v2 = 6; tri++;
  colors[tri] = Vec3f(0,1,0); triangles[tri].v0 = 5; triangles[tri].v1 = 7; triangles[tri].v2 = 6; tri++;

  // bottom side
  colors[tri] = Vec3f(0.5f);  triangles[tri].v0 = 0; triangles[tri].v1 = 1; triangles[tri].v2 = 4; tri++;
  colors[tri] = Vec3f(0.5f);  triangles[tri].v0 = 1; triangles[tri].v1 = 5; triangles[tri].v2 = 4; tri++;

  // top side
  colors[tri] = Vec3f(1.0f);  triangles[tri].v0 = 2; triangles[tri].v1 = 6; triangles[tri].v2 = 3; tri++;
  colors[tri] = Vec3f(1.0f);  triangles[tri].v0 = 3; triangles[tri].v1 = 6; triangles[tri].v2 = 7; tri++;

  // front side
  colors[tri] = Vec3f(0,0,1); triangles[tri].v0 = 0; triangles[tri].v1 = 4; triangles[tri].v2 = 2; tri++;
  colors[tri] = Vec3f(0,0,1); triangles[tri].v0 = 2; triangles[tri].v1 = 4; triangles[tri].v2 = 6; tri++;

  // back side
  colors[tri] = Vec3f(1,1,0); triangles[tri].v0 = 1; triangles[tri].v1 = 3; triangles[tri].v2 = 5; tri++;
  colors[tri] = Vec3f(1,1,0); triangles[tri].v0 = 3; triangles[tri].v1 = 7; triangles[tri].v2 = 5; tri++;

  rtcUnmapBuffer(scene_i,mesh,RTC_INDEX_BUFFER);

  return mesh;
}

/*! random number generator for floating point numbers in range [0,1] */
inline float frand(int& seed) {
  /*seed = 7 * seed + 5;
  seed = 13 * seed + 17;
  seed = 3 * seed + 2;
  seed = 127 * seed + 13;
  return (seed & 0xFFFF)/(float)0xFFFF;*/
  return drand48();
}

/*! Uniform hemisphere sampling. Up direction is the z direction. */
Vec3f sampleSphere(const float& u, const float& v) 
{
  const float phi = 2.0f*(float)pi * u;
  const float cosTheta = 1.0f - 2.0f * v, sinTheta = 2.0f * sqrt(v * (1.0f - v));
  return Vec3f(cos(phi) * sinTheta, sin(phi) * sinTheta, cosTheta);
}

Vec3f noise(Vec3f p, float t) {
  //return div(p,length(p));
  return p + Vec3f(sin(2.0f*t),4.0f*t,cos(2.0f*t));
  //return p + Vec3f(4.0f*t,4.0f*t,0.0f);
}

/* filter dispatch function */
void filterDispatch(void* ptr, RTCRay2& ray) {
  if (ray.filter) ray.filter(ptr,(RTCRay&)ray);
}

RTCScene convertScene(ISPCScene* scene_in)
{
  /* create scene */
  RTCScene scene_out = rtcNewScene(RTC_SCENE_STATIC | RTC_SCENE_INCOHERENT,RTC_INTERSECT1);

  /* add all hair sets to the scene */
  for (int i=0; i<scene_in->numHairSets; i++)
  {
    /* get ith hair set */
    ISPCHair* hair = scene_in->hairs[i];
    
    /* create a hair set */
    unsigned int geomID = rtcNewQuadraticBezierCurves (scene_out, RTC_GEOMETRY_STATIC, hair->numHairs, hair->numVertices);
    rtcSetBuffer(scene_out,geomID,RTC_VERTEX_BUFFER,hair->v,0,sizeof(Vertex));
    rtcSetBuffer(scene_out,geomID,RTC_INDEX_BUFFER,hair->index,0,sizeof(int));
#if USE_OCCLUSION_FILTER
  rtcSetOcclusionFilterFunction(scene_out,geomID,(RTCFilterFunc)filterDispatch);
#endif
#if USE_INTERSECTION_FILTER
  rtcSetIntersectionFilterFunction(scene_out,geomID,(RTCFilterFunc)filterDispatch);
#endif
  }

  /* add all meshes to the scene */
  for (int i=0; i<scene_in->numMeshes; i++)
  {
    /* get ith mesh */
    ISPCMesh* mesh = scene_in->meshes[i];

    /* create a triangle mesh */
    unsigned int geomID = rtcNewTriangleMesh (scene_out, RTC_GEOMETRY_STATIC, mesh->numTriangles, mesh->numVertices);
    
    /* set vertices */
    Vertex* vertices = (Vertex*) rtcMapBuffer(scene_out,geomID,RTC_VERTEX_BUFFER); 
    for (int j=0; j<mesh->numVertices; j++) {
      vertices[j].x = mesh->positions[j].x;
      vertices[j].y = mesh->positions[j].y;
      vertices[j].z = mesh->positions[j].z;
    }

    /* set triangles */
    Triangle* triangles = (Triangle*) rtcMapBuffer(scene_out,geomID,RTC_INDEX_BUFFER);
    for (int j=0; j<mesh->numTriangles; j++) {
      triangles[j].v0 = mesh->triangles[j].v0;
      triangles[j].v1 = mesh->triangles[j].v1;
      triangles[j].v2 = mesh->triangles[j].v2;
    }
    rtcUnmapBuffer(scene_out,geomID,RTC_VERTEX_BUFFER); 
    rtcUnmapBuffer(scene_out,geomID,RTC_INDEX_BUFFER);

#if USE_INTERSECTION_FILTER
    rtcSetIntersectionFilterFunction(scene_out,geomID,(RTCFilterFunc)filterDispatch);
#endif
  }

  /* commit changes to scene */
  rtcCommit (scene_out);
  return scene_out;
}

/* intersection filter function */
void intersectionFilter(void* ptr, RTCRay2& ray)
{
#if 0
  if (ray.geomID == 0) {
    /* calculate how much the curve occludes the ray */
    float sizeRay = max(ray.org.w + ray.tfar*ray.dir.w, 0.00001f);
    float sizeCurve = evalBezier(ray.primID,ray.u).w;
    float T = 1.0f-clamp((1.0f-T_hair)*sizeCurve/sizeRay,0.0f,1.0f);
    T *= ray.transparency;
    ray.transparency = T;
  }
#endif
  bool added = addHit(ray.list,ray);
  /*if (T != 0.0f && added)*/ ray.geomID = RTC_INVALID_GEOMETRY_ID; // FIXME: enable this
}

/* occlusion filter function */
void occlusionFilter(void* ptr, RTCRay2& ray)
{
  // FIXME: handle triangles properly
  /* calculate how much the curve occludes the ray */
  float sizeRay = max(ray.org.w + ray.tfar*ray.dir.w, 0.00001f);
  float sizeCurve = evalBezier(ray.geomID,ray.primID,ray.u).w;
  float T = 1.0f-clamp((1.0f-T_hair)*sizeCurve/sizeRay,0.0f,1.0f);
  T *= ray.transparency;
  ray.transparency = T;
  if (T != 0.0f) ray.geomID = RTC_INVALID_GEOMETRY_ID;
}

#if USE_OCCLUSION_FILTER

float occluded(RTCScene scene, RTCRay2& ray)
{
  return 1.0f;
  ray.filter = (RTCFilterFunc) occlusionFilter;
  ray.transparency = 1.0f;
  rtcOccluded(scene,(RTCRay&)ray);
  return ray.transparency;
}

#else

float occluded(RTCScene scene, RTCRay2& ray)
{
  float T = 1.0f;
  while (true) 
  {
    rtcIntersect(scene,(RTCRay&)ray);
    if (ray.geomID == RTC_INVALID_GEOMETRY_ID) break;
    return 0.0f;
    if (ray.geomID >= g_ispc_scene->numHairSets) return 0.0f; // make all surfaces opaque
    
    /* calculate how much the curve occludes the ray */
    float sizeRay = max(ray.org.w + ray.tfar*ray.dir.w, 0.00001f);
    float sizeCurve = evalBezier(ray.geomID,ray.primID,ray.u).w;
    T *= 1.0f-clamp((1.0f-T_hair)*sizeCurve/sizeRay,0.0f,1.0f);

    /* continue ray ray */
    ray.geomID = RTC_INVALID_GEOMETRY_ID;
    ray.tnear = 1.001f*ray.tfar;
    ray.tfar = inf;
  }
  return T;
}

#endif

#if 0
/* adds hair to the scene */
unsigned int addHair (RTCScene scene_i)
{
  int seed = 879;
  const int numCurves = 400;
  const int numCurveSegments = 4;
  const int numCurvePoints = 3*numCurveSegments+1;
  const float R = 0.01f;

  /* create set of bezier curves */
  unsigned int geomID = rtcNewQuadraticBezierCurves (scene_i, RTC_GEOMETRY_STATIC, numCurves*numCurveSegments, numCurves*numCurvePoints);
  vertices = new Vertex[numCurves*numCurvePoints];
  indices  = new int   [numCurves*numCurveSegments];
  rtcSetBuffer(scene_i,geomID,RTC_VERTEX_BUFFER,vertices,0,sizeof(Vertex));
  rtcSetBuffer(scene_i,geomID,RTC_INDEX_BUFFER,indices,0,sizeof(int));

  for (size_t i=0; i<numCurves; i++)
  {
    float ru = frand(seed);
    float rv = frand(seed);
    Vec3f p = Vec3f(-2.0f+ru*4.0f,-2.0f,-2.0f+rv*4.0f);
    for (size_t j=0; j<=numCurveSegments; j++) 
    {
      bool last = j == numCurveSegments;
      float f0 = float(2*j+0)/float(2*numCurveSegments);
      float f1 = float(2*j+1)/float(2*numCurveSegments);
      Vec3f p0 = noise(p,f0);
      Vec3f p1 = noise(p,f1);
      
      if (j>0) {
        vertices[i*numCurvePoints+3*j-1].x = 2.0f*p0.x-p1.x;
        vertices[i*numCurvePoints+3*j-1].y = 2.0f*p0.y-p1.y;
        vertices[i*numCurvePoints+3*j-1].z = 2.0f*p0.z-p1.z;
        vertices[i*numCurvePoints+3*j-1].r = last ? 0.0f : R;
      }
      
      vertices[i*numCurvePoints+3*j+0].x = p0.x;
      vertices[i*numCurvePoints+3*j+0].y = p0.y;
      vertices[i*numCurvePoints+3*j+0].z = p0.z;
      vertices[i*numCurvePoints+3*j+0].r = last ? 0.0f : R;

      if (j<numCurveSegments) {
        vertices[i*numCurvePoints+3*j+1].x = p1.x;
        vertices[i*numCurvePoints+3*j+1].y = p1.y;
        vertices[i*numCurvePoints+3*j+1].z = p1.z;
        vertices[i*numCurvePoints+3*j+1].r = R;
      }
    }

    for (size_t j=0; j<numCurveSegments; j++) {
      indices[i*numCurveSegments+j] = i*numCurvePoints+3*j;
    }
  }

#if USE_INTERSECTION_FILTER
  rtcSetIntersectionFilterFunction(scene_i,geomID,(RTCFilterFunc)filterDispatch);
#endif
#if USE_OCCLUSION_FILTER
  rtcSetOcclusionFilterFunction(scene_i,geomID,(RTCFilterFunc)filterDispatch);
#endif
  return geomID;
}
#endif

/* adds a ground plane to the scene */
unsigned int addGroundPlane (RTCScene scene_i)
{
  /* create a triangulated plane with 2 triangles and 4 vertices */
  unsigned int geomID = rtcNewTriangleMesh (scene_i, RTC_GEOMETRY_STATIC, 2, 4);

  /* set vertices */
  Vertex* vertices = (Vertex*) rtcMapBuffer(scene_i,geomID,RTC_VERTEX_BUFFER); 
  vertices[0].x = -10; vertices[0].y = -2; vertices[0].z = -10; 
  vertices[1].x = -10; vertices[1].y = -2; vertices[1].z = +10; 
  vertices[2].x = +10; vertices[2].y = -2; vertices[2].z = -10; 
  vertices[3].x = +10; vertices[3].y = -2; vertices[3].z = +10;
  rtcUnmapBuffer(scene_i,geomID,RTC_VERTEX_BUFFER); 

  /* set triangles */
  Triangle* triangles = (Triangle*) rtcMapBuffer(scene_i,geomID,RTC_INDEX_BUFFER);
  triangles[0].v0 = 0; triangles[0].v1 = 2; triangles[0].v2 = 1;
  triangles[1].v0 = 1; triangles[1].v1 = 2; triangles[1].v2 = 3;
  rtcUnmapBuffer(scene_i,geomID,RTC_INDEX_BUFFER);

#if USE_INTERSECTION_FILTER
  rtcSetIntersectionFilterFunction(scene_i,geomID,(RTCFilterFunc)filterDispatch);
#endif

  return geomID;
}

/* called by the C++ code for initialization */
extern "C" void device_init (int8* cfg)
{
  /* initialize ray tracing core */
  rtcInit(cfg);

#if 0
  /* create scene */
  g_scene = rtcNewScene(RTC_SCENE_STATIC,RTC_INTERSECT1);

  /* add cube */
  //addCube(g_scene);

  /* add hairs */
  addHair(g_scene);

  /* add ground plane */
  addGroundPlane(g_scene);

  /* commit changes to scene */
  rtcCommit (g_scene);
#endif

  /* set start render mode */
  renderPixel = renderPixelStandard;
}

/*! Anisotropic power cosine microfacet distribution. */
class AnisotropicPowerCosineDistribution {
public:

  __forceinline AnisotropicPowerCosineDistribution() {}

  /*! Anisotropic power cosine distribution constructor. */
  __forceinline AnisotropicPowerCosineDistribution(const Vec3fa& R, const Vec3fa& dx, float nx, const Vec3fa& dy, float ny, const Vec3fa& dz) 
    : R(R), dx(dx), nx(nx), dy(dy), ny(ny), dz(dz),
      //norm1(sqrtf((nx+1)*(ny+1)) * float(one_over_two_pi)),
      //norm2(sqrtf((nx+2)*(ny+2)) * float(one_over_two_pi)) {}
      norm1(1.0f), norm2(1.0f) {}

  /*! Evaluates the power cosine distribution. \param wh is the half
   *  vector */
  __forceinline float eval(const Vec3fa& wh) const 
  {
    const float cosPhiH   = dot(wh, dx);
    const float sinPhiH   = dot(wh, dy);
    const float cosThetaH = dot(wh, dz);
    const float R = sqr(cosPhiH)+sqr(sinPhiH);
    if (R == 0.0f) return norm2;
    const float n = (nx*sqr(cosPhiH)+ny*sqr(sinPhiH))*rcp(R);
    return norm2 * pow(abs(cosThetaH), n);
  }

  __forceinline Vec3fa reflect(const Vec3fa& I, const Vec3fa& N) const {
    return I-2.0f*dot(I,N)*N;
  }

  __forceinline Vec3fa eval(int geomID, const Vec3fa& wo, const Vec3fa& wi_) const
  {
    Vec3fa wi = wi_;
    //if (dot(wi,dz) <= 0) return zero;
    const float cosThetaO = dot(wo,dz);
    const float cosThetaI = dot(wi,dz);
    //if (cosThetaI <= 0.0f || cosThetaO <= 0.0f) return zero;
    if (cosThetaI < 0.0f) {
      wi = reflect(wi,dz);
    }
    const Vec3fa wh = normalize(wi + wo);
    const float cosThetaH = dot(wh, dz);
    const float cosTheta = dot(wi, wh); // = dot(wo, wh);
    const float D = eval(wh);
    const float G = min(1.0f, 2.0f * cosThetaH * cosThetaO * rcp(cosTheta), 2.0f * cosThetaH * cosThetaI * rcp(cosTheta));
    return R * D * G * rcp(4.0f*cosThetaO);
    //const float G = dot(wi,dz);
    //return R*D*G;
  }

public:
  Vec3fa R;
  Vec3fa dx;       //!< x-direction of the distribution.
  float nx;        //!< Glossiness in x direction with range [0,infinity[ where 0 is a diffuse surface.
  Vec3fa dy;       //!< y-direction of the distribution.
  float ny;        //!< Exponent that determines the glossiness in y direction.
  Vec3fa dz;       //!< z-direction of the distribution.
  float norm1;     //!< Normalization constant for calculating the pdf for sampling.
  float norm2;     //!< Normalization constant for calculating the distribution.
};

#if USE_INTERSECTION_FILTER

/* task that renders a single screen tile */
Vec3fa renderPixelStandard(int x, int y, const Vec3fa& vx, const Vec3fa& vy, const Vec3fa& vz, const Vec3fa& p)
{
  //if (x != 256 || y != 256) return zero;

  /* initialize ray */
  RTCRay2 ray;
  ray.org = p;
  ray.org.w = 0.0f;
  ray.dir = normalize(x*vx + y*vy + vz);
  Vec3fa dir1 = normalize((x+1)*vx + (y+1)*vy + vz);
  ray.dir.w = 0.5f*0.707f*length(dir1-ray.dir);
  ray.tnear = 0.0f;
  ray.tfar = inf;
  ray.geomID = RTC_INVALID_GEOMETRY_ID;
  ray.primID = RTC_INVALID_GEOMETRY_ID;
  ray.mask = -1;
  ray.time = 0;
  ray.filter = (RTCFilterFunc) intersectionFilter;

  Vec3fa color = Vec3f(0.0f);
  float weight = 1.0f;

  HitList hits;
  hits.num = 0;
  ray.list = &hits;

  /* intersect ray with scene and gather all hits */
  rtcIntersect(g_scene,(RTCRay&)ray);

/*  for (size_t i=0; i<hits.num; i++) 
  {
    RTCRay2* ray2 = hits.rays[i];
    PRINT(ray2->tfar);
    }*/

  /* iterate through all hits */
  for (size_t i=0; i<hits.num; i++) 
  {
    RTCRay2* ray2 = hits.rays[i];
  
    /* calculate transmissivity of hair */
    AnisotropicPowerCosineDistribution brdf;

    float Th = 0.0f;
    if (ray2->geomID < g_ispc_scene->numHairSets) 
    {
      /* calculate how much the curve occludes the ray */
      float sizeRay = max(ray2->org.w + ray2->tfar*ray2->dir.w, 0.00001f);
      float sizeCurve = evalBezier(ray2->geomID,ray2->primID,ray2->u).w;
      Th = 1.0f-clamp((1.0f-T_hair)*sizeCurve/sizeRay,0.0f,1.0f);

      /* calculate tangent space */
      const Vec3fa dx = normalize(ray2->Ng);
      const Vec3fa dy = normalize(cross(ray2->dir,dx));
      const Vec3fa dz = normalize(cross(dy,dx));

      /* generate anisotropic BRDF */
      //const Vec3fa color(0.5f,0.4f,0.4f);
      const Vec3fa color(1.0f);
      new (&brdf) AnisotropicPowerCosineDistribution(color,dx,10.0f,dy,1.0f,dz);
    }
    else 
    {
      /* calculate tangent space */
      const Vec3fa dz = normalize(ray2->Ng);
      const Vec3fa dx = normalize(cross(dz,ray2->dir));
      const Vec3fa dy = normalize(cross(dz,dx));
      
      /* generate isotropic BRDF */
      const Vec3fa color(1.0f,1.0f,1.0f);
      new (&brdf) AnisotropicPowerCosineDistribution(color,dx,1.0f,dy,1.0f,dz);
    }
    
    /* calculate shadows */
    //Vec3f diffuse = Vec3f(0.5f,0.4f,0.4f); //colors[ray2->primID];
    //if (ray2->geomID == 1) diffuse = Vec3f(1.0f,1.0f,1.0f);
    //color = color + diffuse*0.5f; // FIXME: use +=
    
    /* initialize shadow ray */
    RTCRay2 shadow;
    shadow.org = ray2->org + ray2->tfar*ray2->dir;
    shadow.org.w = ray2->org.w+ray2->tfar*ray2->dir.w;
    shadow.dir = neg(lightDir);
    shadow.dir.w = 0.0f;
    shadow.tnear = 0.1f;
    shadow.tfar = inf;
    shadow.geomID = RTC_INVALID_GEOMETRY_ID;
    shadow.primID = RTC_INVALID_GEOMETRY_ID;
    shadow.mask = -1;
    shadow.time = 0;
    shadow.filter = NULL;
    
    /* trace shadow ray */
    float T = occluded(g_scene,shadow);
    
    /* add light contribution */
    Vec3fa c = brdf.eval(ray2->geomID,neg(ray2->dir),neg(lightDir));
    //if (ray2->geomID == 0) PRINT(c);
    //Vec3fa c = clamp(dot(neg(ray2->dir),brdf.dz),0.0f,1.0f);
    color = color + weight*(1.0f-Th)*c*T*lightIntensity; //clamp(-dot(lightDir,normalize(ray2->Ng)),0.0f,1.0f))); // FIXME: use +=
    weight *= Th;
  }
  return color;
}

#else

/* task that renders a single screen tile */
Vec3fa renderPixelStandard(int x, int y, const Vec3fa& vx, const Vec3fa& vy, const Vec3fa& vz, const Vec3fa& p)
{
  //if (x != 417 || y != 376) return zero;

  /* initialize ray */
  RTCRay2 ray;
  ray.org = p;
  ray.org.w = 0.0f;
  ray.dir = normalize(x*vx + y*vy + vz);
  Vec3fa dir1 = normalize((x+1)*vx + (y+1)*vy + vz);
  ray.dir.w = 0.5f*0.707f*length(dir1-ray.dir);
  ray.tnear = 0.0f;
  ray.tfar = inf;
  ray.geomID = RTC_INVALID_GEOMETRY_ID;
  ray.primID = RTC_INVALID_GEOMETRY_ID;
  ray.mask = -1;
  ray.time = 0;

  Vec3fa color = Vec3f(0.0f);
  float weight = 1.0f;

  while (true)
  {
    /* intersect ray with scene and gather all hits */
    rtcIntersect(g_scene,(RTCRay&)ray);
    //PRINT(ray.tfar);

    /* exit if we hit environment */
    if (ray.geomID == RTC_INVALID_GEOMETRY_ID) 
      return color;

    /* calculate transmissivity of hair */
    AnisotropicPowerCosineDistribution brdf;

    float Th = 0.0f;
    if (ray.geomID < g_ispc_scene->numHairSets) 
    {
      /* calculate how much the curve occludes the ray */
      float sizeRay = max(ray.org.w + ray.tfar*ray.dir.w, 0.00001f);
      float sizeCurve = evalBezier(ray.geomID,ray.primID,ray.u).w;
      Th = 1.0f-clamp((1.0f-T_hair)*sizeCurve/sizeRay,0.0f,1.0f);

      /* calculate tangent space */
      const Vec3fa dx = normalize(ray.Ng);
      const Vec3fa dy = normalize(cross(ray.dir,dx));
      const Vec3fa dz = normalize(cross(dy,dx));

      /* generate anisotropic BRDF */
      //const Vec3fa color(0.5f,0.4f,0.4f);
      const Vec3fa color(1.0f);
      new (&brdf) AnisotropicPowerCosineDistribution(color,dx,10.0f,dy,1.0f,dz);
    }
    else 
    {
      /* calculate tangent space */
      const Vec3fa dz = normalize(ray.Ng);
      const Vec3fa dx = normalize(cross(dz,ray.dir));
      const Vec3fa dy = normalize(cross(dz,dx));
      
      /* generate isotropic BRDF */
      const Vec3fa color(1.0f,1.0f,1.0f);
      new (&brdf) AnisotropicPowerCosineDistribution(color,dx,1.0f,dy,1.0f,dz);
    }

    /* calculate shadows */
    //Vec3f diffuse = Vec3f(0.5f,0.4f,0.4f); //colors[ray.primID];
    //if (ray.geomID == 1) diffuse = Vec3f(1.0f,1.0f,1.0f);
    //color = color + diffuse*0.5f;
    
    /* initialize shadow ray */
    RTCRay2 shadow;
    shadow.org = ray.org + ray.tfar*ray.dir;
    shadow.org.w = ray.org.w+ray.tfar*ray.dir.w;
    shadow.dir = neg(lightDir);
    shadow.dir.w = 0.0f;
    shadow.tnear = 0.1f;
    shadow.tfar = inf;
    shadow.geomID = RTC_INVALID_GEOMETRY_ID;
    shadow.primID = RTC_INVALID_GEOMETRY_ID;
    shadow.mask = -1;
    shadow.time = 0;
    shadow.filter = NULL;
    
    /* trace shadow ray */
    float T = occluded(g_scene,shadow);

    /* add light contribution */
    Vec3fa c = brdf.eval(ray.geomID,neg(ray.dir),neg(lightDir));
    //Vec3fa c = clamp(dot(neg(ray.dir),brdf.dz),0.0f,1.0f);
    color = color + weight*(1.0f-Th)*c*T*lightIntensity; //clamp(-dot(lightDir,normalize(ray.Ng)),0.0f,1.0f))); // FIXME: use +=
    weight *= Th;
    if (weight < 0.01) return color;
    return color;

    /* continue ray */
    ray.geomID = RTC_INVALID_GEOMETRY_ID;
    ray.tnear = 1.001f*ray.tfar; //+2.0f*shadow.org.w;
    ray.tfar = inf;
  }
  return color;
}

#endif

/* task that renders a single screen tile */
void renderTile(int taskIndex, int* pixels,
                const int width,
                const int height, 
                const float time,
                const Vec3f& vx, 
                const Vec3f& vy, 
                const Vec3f& vz, 
                const Vec3f& p,
                const int numTilesX, 
                const int numTilesY)
{
  const int tileY = taskIndex / numTilesX;
  const int tileX = taskIndex - tileY * numTilesX;
  const int x0 = tileX * TILE_SIZE_X;
  const int x1 = min(x0+TILE_SIZE_X,width);
  const int y0 = tileY * TILE_SIZE_Y;
  const int y1 = min(y0+TILE_SIZE_Y,height);

  for (int y = y0; y<y1; y++) for (int x = x0; x<x1; x++)
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
extern "C" void device_render (int* pixels,
                               const int width,
                               const int height,
                               const float time,
                               const Vec3f& vx, 
                               const Vec3f& vy, 
                               const Vec3f& vz, 
                               const Vec3f& p)
{
  /* create scene */
  if (g_scene == NULL)
    g_scene = convertScene(g_ispc_scene);

  const int numTilesX = (width +TILE_SIZE_X-1)/TILE_SIZE_X;
  const int numTilesY = (height+TILE_SIZE_Y-1)/TILE_SIZE_Y;
  launch_renderTile(numTilesX*numTilesY,pixels,width,height,time,vx,vy,vz,p,numTilesX,numTilesY); 
  rtcDebug();
}

/* called by the C++ code for cleanup */
extern "C" void device_cleanup ()
{
  rtcDeleteScene (g_scene);
  delete[] colors;
  rtcExit();
}

