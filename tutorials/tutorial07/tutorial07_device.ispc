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

/* scene data */
RTCScene g_scene = NULL;
uniform Vec3f* uniform colors = NULL;

/* render function to use */
renderPixelFunc renderPixel;

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

struct ISPCMesh
{
  uniform Vec3fa* positions;    //!< vertex position array
  uniform Vec3fa* normals;       //!< vertex normal array
  uniform Vec2f* texcoords;     //!< vertex texcoord array
  uniform ISPCTriangle* triangles;  //!< list of triangles
  int numVertices;
  int numTriangles;
  Vec3f dir;
  float offset;
};

struct ISPCHair
{
 int vertex,id;  //!< index of first control point and hair ID
};

struct ISPCHairSet
{
 uniform Vec3fa *positions;   //!< hair control points (x,y,z,r)
 uniform ISPCHair *hairs;    //!< list of hairs
 int numVertices;
 int numHairs;
};


struct ISPCScene
{
  uniform ISPCMesh* uniform* meshes;         //!< list of meshes
  uniform ISPCMaterial* materials;  //!< material list
  int numMeshes;
  int numMaterials;
  uniform ISPCHairSet** hairsets;
  int numHairSets;
  bool animate;
};

/* scene data */
extern uniform ISPCScene* uniform g_ispc_scene;

RTCScene convertScene(uniform ISPCScene* uniform scene_in)
{
  /* create scene */
  RTCScene scene_out = rtcNewScene(RTC_SCENE_STATIC,RTC_INTERSECT_UNIFORM | RTC_INTERSECT_VARYING);

  /* add all meshes to the scene */
  for (uniform int i=0; i<scene_in->numMeshes; i++)
  {
    /* get ith mesh */
    uniform ISPCMesh* uniform mesh = scene_in->meshes[i];

    /* create a triangle mesh */
    uniform unsigned int geometry = rtcNewTriangleMesh (scene_out, RTC_GEOMETRY_STATIC, mesh->numTriangles, mesh->numVertices);

#if 0
    /* share vertex buffer */
    rtcSetBuffer(scene_out, geometry, RTC_VERTEX_BUFFER, mesh->positions, 0, sizeof(uniform Vec3fa      ));
    //rtcSetBuffer(scene_out, geometry, RTC_INDEX_BUFFER,  mesh->triangles, 0, sizeof(uniform ISPCTriangle));

    /* set triangles */
    uniform Triangle* uniform triangles = (uniform Triangle* uniform) rtcMapBuffer(scene_out,geometry,RTC_INDEX_BUFFER);
    for (uniform int j=0; j<mesh->numTriangles; j++) {
      triangles[j].v0 = mesh->triangles[j].v0;
      triangles[j].v1 = mesh->triangles[j].v1;
      triangles[j].v2 = mesh->triangles[j].v2;
    }
    rtcUnmapBuffer(scene_out,geometry,RTC_INDEX_BUFFER);

#elif 0
    uniform Vec3f* uniform positions = uniform new uniform Vec3f[mesh->numVertices+1];
    for (uniform int j=0; j<mesh->numVertices; j++) {
      positions[j].x = mesh->positions[j].x;
      positions[j].y = mesh->positions[j].y;
      positions[j].z = mesh->positions[j].z;
    }
    rtcSetBuffer(scene_out, geometry, RTC_VERTEX_BUFFER, positions, 0, sizeof(uniform Vec3f));
    rtcSetBuffer(scene_out, geometry, RTC_INDEX_BUFFER,  mesh->triangles, 0, sizeof(uniform ISPCTriangle));
#else

    /* set vertices */
    uniform Vertex* uniform vertices = (uniform Vertex* uniform) rtcMapBuffer(scene_out,geometry,RTC_VERTEX_BUFFER); 
    for (uniform int j=0; j<mesh->numVertices; j++) {
      vertices[j].x = mesh->positions[j].x;
      vertices[j].y = mesh->positions[j].y;
      vertices[j].z = mesh->positions[j].z;
    }
    rtcUnmapBuffer(scene_out,geometry,RTC_VERTEX_BUFFER); 

    /* set triangles */
    uniform Triangle* uniform triangles = (uniform Triangle* uniform) rtcMapBuffer(scene_out,geometry,RTC_INDEX_BUFFER);
    for (uniform int j=0; j<mesh->numTriangles; j++) {
      triangles[j].v0 = mesh->triangles[j].v0;
      triangles[j].v1 = mesh->triangles[j].v1;
      triangles[j].v2 = mesh->triangles[j].v2;
    }
    rtcUnmapBuffer(scene_out,geometry,RTC_INDEX_BUFFER);
#endif
  }


  /* add all meshes to the scene */
  for (uniform int i=0; i<scene_in->numHairSets; i++)
  {
  
    /* get ith hair set */
    uniform ISPCHairSet* uniform hairset = scene_in->hairsets[i];
    
    /* create a hair set */
    uniform unsigned int geomID = rtcNewBezierCurves (scene_out, RTC_GEOMETRY_STATIC, hairset->numHairs, hairset->numVertices);
    uniform Vertex* uniform vertices = (uniform Vertex* uniform)rtcMapBuffer(scene_out,geomID,RTC_VERTEX_BUFFER);

    for (uniform int j=0; j<hairset->numVertices; j++) {
      vertices[j].x = hairset->positions[j].x;
      vertices[j].y = hairset->positions[j].y;
      vertices[j].z = hairset->positions[j].z;
    }

    rtcUnmapBuffer(scene_out,geomID,RTC_VERTEX_BUFFER); 

    uniform ISPCHair* uniform hairs   = (uniform ISPCHair* uniform)rtcMapBuffer(scene_out,geomID,RTC_INDEX_BUFFER);

    for (uniform int j=0; j<hairset->numVertices; j++) {
      hairs[j].vertex = hairset->hairs[j].vertex;
      hairs[j].id     = hairset->hairs[j].id;
    }
    
    rtcUnmapBuffer(scene_out,geomID,RTC_INDEX_BUFFER); 

  }
  /* commit changes to scene */
  rtcCommit (scene_out);
  return scene_out;
}

/* adds a cube to the scene */
uniform unsigned int addCube (RTCScene scene_i)
{
  /* create a triangulated cube with 12 triangles and 8 vertices */
  uniform unsigned int mesh = rtcNewTriangleMesh (scene_i, RTC_GEOMETRY_STATIC, 12, 8);

  /* set vertices */
  uniform Vertex* uniform vertices = (uniform Vertex* uniform) rtcMapBuffer(scene_i,mesh,RTC_VERTEX_BUFFER); 
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
  colors = uniform new uniform Vec3f[12];

  /* set triangles and colors */
  uniform int tri = 0;
  uniform Triangle* uniform triangles = (uniform Triangle* uniform) rtcMapBuffer(scene_i,mesh,RTC_INDEX_BUFFER);
  
  // left side
  colors[tri] = make_Vec3f(1,0,0); triangles[tri].v0 = 0; triangles[tri].v1 = 2; triangles[tri].v2 = 1; tri++;
  colors[tri] = make_Vec3f(1,0,0); triangles[tri].v0 = 1; triangles[tri].v1 = 2; triangles[tri].v2 = 3; tri++;

  // right side
  colors[tri] = make_Vec3f(0,1,0); triangles[tri].v0 = 4; triangles[tri].v1 = 5; triangles[tri].v2 = 6; tri++;
  colors[tri] = make_Vec3f(0,1,0); triangles[tri].v0 = 5; triangles[tri].v1 = 7; triangles[tri].v2 = 6; tri++;

  // bottom side
  colors[tri] = make_Vec3f(0.5f);  triangles[tri].v0 = 0; triangles[tri].v1 = 1; triangles[tri].v2 = 4; tri++;
  colors[tri] = make_Vec3f(0.5f);  triangles[tri].v0 = 1; triangles[tri].v1 = 5; triangles[tri].v2 = 4; tri++;

  // top side
  colors[tri] = make_Vec3f(1.0f);  triangles[tri].v0 = 2; triangles[tri].v1 = 6; triangles[tri].v2 = 3; tri++;
  colors[tri] = make_Vec3f(1.0f);  triangles[tri].v0 = 3; triangles[tri].v1 = 6; triangles[tri].v2 = 7; tri++;

  // front side
  colors[tri] = make_Vec3f(0,0,1); triangles[tri].v0 = 0; triangles[tri].v1 = 4; triangles[tri].v2 = 2; tri++;
  colors[tri] = make_Vec3f(0,0,1); triangles[tri].v0 = 2; triangles[tri].v1 = 4; triangles[tri].v2 = 6; tri++;

  // back side
  colors[tri] = make_Vec3f(1,1,0); triangles[tri].v0 = 1; triangles[tri].v1 = 3; triangles[tri].v2 = 5; tri++;
  colors[tri] = make_Vec3f(1,1,0); triangles[tri].v0 = 3; triangles[tri].v1 = 7; triangles[tri].v2 = 5; tri++;

  rtcUnmapBuffer(scene_i,mesh,RTC_INDEX_BUFFER);

  return mesh;
}

/*! random number generator for floating point numbers in range [0,1] */
inline uniform float frand(uniform int& seed) {
  seed = 1103515245 * seed + 12345;
  seed = 235543534 * seed + 2341233;
  seed = 43565 * seed + 2332443;
  return (seed & 0xFFFF)/(uniform float)0xFFFF;
}

/*! Uniform hemisphere sampling. Up direction is the z direction. */
uniform Vec3f sampleSphere(const uniform float& u, const uniform float& v) 
{
  const uniform float phi = 2.0f*(float)pi * u;
  const uniform float cosTheta = 1.0f - 2.0f * v, sinTheta = 2.0f * sqrt(v * (1.0f - v));
  return make_Vec3f(cos(phi) * sinTheta, sin(phi) * sinTheta, cosTheta);
}

uniform Vec3f noise(uniform Vec3f p) {
  return p / length(p);
}

/* adds hair to the scene */
uniform unsigned int addHair (RTCScene scene_i)
{
#if 1
  print("addHair\n");
  uniform unsigned int geomID = rtcNewBezierCurves (scene_i, RTC_GEOMETRY_STATIC, 1, 4);
  uniform Vertex* uniform vertices = (uniform Vertex* uniform) rtcMapBuffer(scene_i,geomID,RTC_VERTEX_BUFFER); 
  uniform int*    uniform indices  = (uniform int*    uniform) rtcMapBuffer(scene_i,geomID,RTC_INDEX_BUFFER); 
  vertices[0].x = 0; vertices[0].y = -2; vertices[0].z = 0; vertices[0].r = 0.5; 
  vertices[1].x = 0; vertices[1].y = -1; vertices[1].z = 0; vertices[1].r = 0.5; 
  vertices[2].x = 0; vertices[2].y =  0; vertices[2].z = 0; vertices[2].r = 0.5; 
  vertices[3].x = 0; vertices[3].y =  1; vertices[3].z = 0; vertices[3].r = 0.5; 
  indices[0] = 0;
  rtcUnmapBuffer(scene_i,geomID,RTC_VERTEX_BUFFER); 
  rtcUnmapBuffer(scene_i,geomID,RTC_INDEX_BUFFER); 
  return geomID;

#else
  uniform int seed = 0;
  const uniform int numCurves = 1000;
  const uniform int numCurveSegments = 4;
  const uniform int numCurvePoints = 3*numCurveSegments+1;

  /* create set of bezier curves */
  uniform unsigned int geomID = rtcNewBezierCurves (scene_i, RTC_GEOMETRY_STATIC, numCurves*numCurveSegments, numCurves*numCurvePoints);

  uniform Vertex* uniform vertices = (uniform Vertex* uniform) rtcMapBuffer(scene_i,geomID,RTC_VERTEX_BUFFER); 
  uniform int*    uniform indices  = (uniform int*    uniform) rtcMapBuffer(scene_i,geomID,RTC_INDEX_BUFFER); 

  for (uniform size_t i=0; i<numCurves; i++)
  {
    uniform float ru = frand(seed);
    uniform float rv = frand(seed);
    uniform Vec3f d = sampleSphere(ru,rv);
    uniform Vec3f p = div(d,max(abs(d.x),max(abs(d.y),abs(d.z))));
    for (uniform size_t j=0; j<=numCurveSegments; j++) 
    {
      uniform Vec3f p0 = p;
      p = add(p,noise(p));
      uniform Vec3f p1 = p;
      p = add(p,noise(p));
      
      if (j>0) {
        vertices[i*numCurvePoints+3*j-1].x = 2.0f*p0.x-p1.x;
        vertices[i*numCurvePoints+3*j-1].y = 2.0f*p0.y-p1.y;
        vertices[i*numCurvePoints+3*j-1].z = 2.0f*p0.z-p1.z;
        vertices[i*numCurvePoints+3*j-1].r = 0.01f;
      }
      
      vertices[i*numCurvePoints+3*j+0].x = p0.x;
      vertices[i*numCurvePoints+3*j+0].y = p0.y;
      vertices[i*numCurvePoints+3*j+0].z = p0.z;
      vertices[i*numCurvePoints+3*j+0].r = 0.01f;

      if (j<numCurveSegments) {
        vertices[i*numCurvePoints+3*j+1].x = p1.x;
        vertices[i*numCurvePoints+3*j+1].y = p1.y;
        vertices[i*numCurvePoints+3*j+1].z = p1.z;
        vertices[i*numCurvePoints+3*j+1].r = 0.01f;
      }
    }

    for (uniform size_t j=0; j<numCurveSegments; j++) {
      indices[i*numCurveSegments+j] = i*numCurvePoints+3*j;
    }
  }

  rtcUnmapBuffer(scene_i,geomID,RTC_VERTEX_BUFFER); 
  rtcUnmapBuffer(scene_i,geomID,RTC_INDEX_BUFFER); 

  return geomID;
#endif
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

  /* create scene */
  //g_scene = rtcNewScene(RTC_SCENE_STATIC,RTC_INTERSECT_UNIFORM | RTC_INTERSECT_VARYING);

  /* add cube */
  //addCube(g_scene);

  /* add hairs */
  //addHair(g_scene);

  /* add ground plane */
  //addGroundPlane(g_scene);

  //addMeshes(g_scene,g_ispc_scene);

  /* commit changes to scene */
  //rtcCommit (g_scene);

  /* set start render mode */
  renderPixel = renderPixelStandard;
}

/* task that renders a single screen tile */
Vec3f renderPixelStandard(float x, float y, const uniform Vec3f& vx, const uniform Vec3f& vy, const uniform Vec3f& vz, const uniform Vec3f& p)
{
  /* initialize ray */
  RTCRay ray;
  ray.org = p;
  ray.dir = normalize(x*vx + y*vy + vz);
  ray.tnear = 0.0f;
  ray.tfar = inf;
  ray.geomID = RTC_INVALID_GEOMETRY_ID;
  ray.primID = RTC_INVALID_GEOMETRY_ID;
  ray.mask = -1;
  ray.time = 0;
  
  /* intersect ray with scene */
  rtcIntersect(g_scene,ray);
  
  if (ray.geomID == RTC_INVALID_GEOMETRY_ID) return make_Vec3f(0.0f);
  else return make_Vec3f(ray.u,ray.v,1.0f-ray.u-ray.v);

#if 0
  /* shade pixels */
  Vec3f color = make_Vec3f(0.0f);
  if (ray.geomID != RTC_INVALID_GEOMETRY_ID) 
  {
    Vec3f diffuse = colors[ray.primID];
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
    shadow.time = 0;
    
    /* trace shadow ray */
    rtcOccluded(g_scene,shadow);
    
    /* add light contribution */
    if (shadow.geomID)
      color = color + diffuse*clamp(-dot(lightDir,normalize(ray.Ng)),0.0f,1.0f); // FIXME: +=
  }
  return color;
#endif
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
  if (g_scene == NULL)
    g_scene = convertScene(g_ispc_scene);

  const uniform int numTilesX = (width +TILE_SIZE_X-1)/TILE_SIZE_X;
  const uniform int numTilesY = (height+TILE_SIZE_Y-1)/TILE_SIZE_Y;
  launch[numTilesX*numTilesY] renderTile(pixels,width,height,time,vx,vy,vz,p,numTilesX,numTilesY); sync;
  rtcDebug();
}

/* called by the C++ code for cleanup */
export void device_cleanup ()
{
  rtcDeleteScene (g_scene);
  delete[] colors;
  rtcExit();
}

