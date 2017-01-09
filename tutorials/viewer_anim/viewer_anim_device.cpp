// ======================================================================== //
// Copyright 2009-2017 Intel Corporation                                    //
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

/* hack to quickly enable use 8-wide ray initialization and rtcIntersectNM */

#define VECTOR_MODE 0
#define ENABLE_ANIM 1

#if VECTOR_MODE  == 1
#define __SSE4_2__
#define __AVX__
#endif 

#include "../common/math/random_sampler.h"
#include "../common/math/sampling.h"
#include "../common/tutorial/tutorial_device.h"
#include "../common/tutorial/scene_device.h"


namespace embree {

#define RAYN_FLAGS RTC_INTERSECT_COHERENT
#define ANIM_FPS 15.0f

  extern "C" ISPCScene* g_ispc_scene;

  /* scene data */
  RTCDevice g_device = nullptr;
  RTCScene g_scene   = nullptr;

  /* animation data */
  
  size_t frameID         = 0;
  double animTime        = -1.0f; // global time counter
  unsigned int staticID  = 0;
  unsigned int dynamicID = 0;

  std::vector<double> buildTime;
  std::vector<double> vertexUpdateTime;
  std::vector<double> renderTime;
  bool printStats = false;
  bool timeInitialized = false;

  static const size_t numProfileFrames = 200;

  /* shadow distance map */

  float *shadowDistanceMap = NULL;
  float *zBuffer = NULL;


  void dumpBuildAndRenderTimes();

  void device_key_pressed_handler(int key)
  {
    if (key == 100 /*d*/) { 
      std::cout << "dumping build and render times per frame [" << buildTime.size() << " frames]..." << std::flush;
      dumpBuildAndRenderTimes(); 
      std::cout << "done" << std::endl;
    }
    else if (key == 115 /*s*/) { 
      printStats = !printStats; 
    }
    else device_key_pressed_default(key);
  }

  // ==================================================================================================
  // ==================================================================================================
  // ==================================================================================================


  unsigned int convertTriangleMesh(ISPCTriangleMesh* mesh, RTCScene scene_out, RTCGeometryFlags object_flags = RTC_GEOMETRY_STATIC)
  {
    unsigned int geomID = rtcNewTriangleMesh (scene_out, object_flags, mesh->numTriangles, mesh->numVertices, mesh->numTimeSteps);
    for (size_t t=0; t<mesh->numTimeSteps; t++) {
      //rtcSetBuffer(scene_out, geomID, (RTCBufferType)(RTC_VERTEX_BUFFER+t),mesh->positions+t*mesh->numVertices, 0, sizeof(Vec3fa      ));
      Vec3fa* vertices = (Vec3fa*) rtcMapBuffer(scene_out,geomID,(RTCBufferType)(RTC_VERTEX_BUFFER+t));
      for (size_t i=0;i<mesh->numVertices;i++) vertices[i] = *(mesh->positions+t*mesh->numVertices+i);        
      rtcUnmapBuffer(scene_out, geomID, (RTCBufferType)(RTC_VERTEX_BUFFER+t));
    }
    rtcSetBuffer(scene_out, geomID, RTC_INDEX_BUFFER,  mesh->triangles, 0, sizeof(ISPCTriangle));
    mesh->geomID = geomID;
    return geomID;
  }


  unsigned int convertQuadMesh(ISPCQuadMesh* mesh, RTCScene scene_out, RTCGeometryFlags object_flags = RTC_GEOMETRY_STATIC)
  {
    unsigned int geomID = rtcNewQuadMesh (scene_out, object_flags, mesh->numQuads, mesh->numVertices, mesh->numTimeSteps);
    for (size_t t=0; t<mesh->numTimeSteps; t++) {
      rtcSetBuffer(scene_out, geomID, (RTCBufferType)(RTC_VERTEX_BUFFER+t),mesh->positions+t*mesh->numVertices, 0, sizeof(Vec3fa      ));
    }
    rtcSetBuffer(scene_out, geomID, RTC_INDEX_BUFFER,  mesh->quads, 0, sizeof(ISPCQuad));
    mesh->geomID = geomID;
    return geomID;
  }

  unsigned int convertSubdivMesh(ISPCSubdivMesh* mesh, RTCScene scene_out, RTCGeometryFlags object_flags = RTC_GEOMETRY_STATIC)
  {
    unsigned int geomID = rtcNewSubdivisionMesh(scene_out, object_flags, mesh->numFaces, mesh->numEdges, mesh->numVertices,
                                                mesh->numEdgeCreases, mesh->numVertexCreases, mesh->numHoles, mesh->numTimeSteps);
    mesh->geomID = geomID;
    for (size_t i=0; i<mesh->numEdges; i++) mesh->subdivlevel[i] = 16.0f;
    for (size_t t=0; t<mesh->numTimeSteps; t++) {
      rtcSetBuffer(scene_out, geomID, (RTCBufferType)(RTC_VERTEX_BUFFER+t),mesh->positions+t*mesh->numVertices, 0, sizeof(Vec3fa  ));
    }
    rtcSetBuffer(scene_out, geomID, RTC_LEVEL_BUFFER,  mesh->subdivlevel, 0, sizeof(float));
    rtcSetBuffer(scene_out, geomID, RTC_INDEX_BUFFER,  mesh->position_indices  , 0, sizeof(unsigned int));
    rtcSetBuffer(scene_out, geomID, RTC_FACE_BUFFER,   mesh->verticesPerFace, 0, sizeof(unsigned int));
    rtcSetBuffer(scene_out, geomID, RTC_HOLE_BUFFER,   mesh->holes, 0, sizeof(unsigned int));
    rtcSetBuffer(scene_out, geomID, RTC_EDGE_CREASE_INDEX_BUFFER,    mesh->edge_creases,          0, 2*sizeof(unsigned int));
    rtcSetBuffer(scene_out, geomID, RTC_EDGE_CREASE_WEIGHT_BUFFER,   mesh->edge_crease_weights,   0, sizeof(float));
    rtcSetBuffer(scene_out, geomID, RTC_VERTEX_CREASE_INDEX_BUFFER,  mesh->vertex_creases,        0, sizeof(unsigned int));
    rtcSetBuffer(scene_out, geomID, RTC_VERTEX_CREASE_WEIGHT_BUFFER, mesh->vertex_crease_weights, 0, sizeof(float));
    return geomID;
  }

  unsigned int convertLineSegments(ISPCLineSegments* mesh, RTCScene scene_out, RTCGeometryFlags object_flags = RTC_GEOMETRY_STATIC)
  {
    unsigned int geomID = rtcNewLineSegments (scene_out, object_flags, mesh->numSegments, mesh->numVertices, mesh->numTimeSteps);
    for (size_t t=0; t<mesh->numTimeSteps; t++) {
      rtcSetBuffer(scene_out,geomID,(RTCBufferType)(RTC_VERTEX_BUFFER+t),mesh->positions+t*mesh->numVertices,0,sizeof(Vertex));
    }
    rtcSetBuffer(scene_out,geomID,RTC_INDEX_BUFFER,mesh->indices,0,sizeof(int));
    return geomID;
  }

  unsigned int convertHairSet(ISPCHairSet* hair, RTCScene scene_out, RTCGeometryFlags object_flags = RTC_GEOMETRY_STATIC)
  {
    unsigned int geomID = rtcNewHairGeometry (scene_out, object_flags, hair->numHairs, hair->numVertices, hair->numTimeSteps);
    for (size_t t=0; t<hair->numTimeSteps; t++) {
      rtcSetBuffer(scene_out,geomID,(RTCBufferType)(RTC_VERTEX_BUFFER+t),hair->positions+t*hair->numVertices,0,sizeof(Vertex));
    }
    rtcSetBuffer(scene_out,geomID,RTC_INDEX_BUFFER,hair->hairs,0,sizeof(ISPCHair));
    return geomID;
  }

  unsigned int convertCurveGeometry(ISPCHairSet* hair, RTCScene scene_out, RTCGeometryFlags object_flags = RTC_GEOMETRY_STATIC)
  {
    unsigned int geomID = rtcNewCurveGeometry (scene_out, object_flags, hair->numHairs, hair->numVertices, hair->numTimeSteps);
    for (size_t t=0; t<hair->numTimeSteps; t++) {
      rtcSetBuffer(scene_out,geomID,(RTCBufferType)(RTC_VERTEX_BUFFER+t),hair->positions+t*hair->numVertices,0,sizeof(Vertex));
    }
    rtcSetBuffer(scene_out,geomID,RTC_INDEX_BUFFER,hair->hairs,0,sizeof(ISPCHair));
    return geomID;
  }

  size_t getNumObjects(ISPCScene* scene_in)
  {
    return scene_in->numGeometries;
  }

  RTCScene createScene(ISPCScene* scene_in, bool dynamic = false)
  {
    int scene_flags = RTC_SCENE_INCOHERENT | (dynamic ? RTC_SCENE_DYNAMIC : RTC_SCENE_STATIC);
    int scene_aflags = RTC_INTERSECT1 | RTC_INTERSECT_STREAM | RTC_INTERPOLATE;
    return rtcDeviceNewScene(g_device, (RTCSceneFlags)scene_flags,(RTCAlgorithmFlags) scene_aflags);
  }
    

  unsigned int createObject(const size_t i, ISPCScene* scene_in, RTCScene scene_out, bool dynamic = false)
  {
    RTCGeometryFlags object_flags = dynamic ? RTC_GEOMETRY_DYNAMIC : RTC_GEOMETRY_STATIC;

    ISPCGeometry* geometry = scene_in->geometries[i];
    unsigned int geomID = 0;

    if (geometry->type == SUBDIV_MESH) {
      geomID = convertSubdivMesh((ISPCSubdivMesh*) geometry, scene_out, object_flags);
      ((ISPCSubdivMesh*)geometry)->geomID = geomID;
      assert(geomID == i);
    }
    else if (geometry->type == TRIANGLE_MESH) {
      geomID = convertTriangleMesh((ISPCTriangleMesh*) geometry, scene_out, object_flags);
      ((ISPCTriangleMesh*)geometry)->geomID = geomID;
      assert(geomID == i);
    }
    else if (geometry->type == QUAD_MESH) {
      geomID = convertQuadMesh((ISPCQuadMesh*) geometry, scene_out, object_flags);
      ((ISPCQuadMesh*)geometry)->geomID = geomID;
      assert(geomID == i);
    }
    else if (geometry->type == LINE_SEGMENTS) {
      geomID = convertLineSegments((ISPCLineSegments*) geometry, scene_out, object_flags);
      ((ISPCLineSegments*)geometry)->geomID = geomID;
      assert(geomID == i);
    }
    else if (geometry->type == HAIR_SET) {
      geomID = convertHairSet((ISPCHairSet*) geometry, scene_out, object_flags);
      ((ISPCHairSet*)geometry)->geomID = geomID;
      assert(geomID == i);
    }
    else if (geometry->type == CURVES) {
      geomID = convertCurveGeometry((ISPCHairSet*) geometry, scene_out, object_flags);
      ((ISPCHairSet*)geometry)->geomID = geomID;
      assert(geomID == i);
    }
    else
      assert(false);
    return geomID;
  }

  void updateVertexData(const unsigned int geomID, ISPCScene* scene_in, RTCScene scene_out, size_t keyFrameID, const float t)
  {
    size_t numGeometries = scene_in->numGeometries;
    if (!numGeometries) return;

    ISPCGeometry* geometry0 = scene_in->geometries[max((keyFrameID+0) % numGeometries,(size_t)1)];
    ISPCGeometry* geometry1 = scene_in->geometries[max((keyFrameID+1) % numGeometries,(size_t)1)];

    /* FIXME: only updating triangle meshes works so far */

    if (geometry0->type == SUBDIV_MESH) {
      unsigned int geomID = ((ISPCSubdivMesh*)geometry0)->geomID;
      rtcUpdate(scene_out,geomID);
    }
    else if (geometry0->type == TRIANGLE_MESH) {
      ISPCTriangleMesh* mesh0 = (ISPCTriangleMesh*)geometry0;
      ISPCTriangleMesh* mesh1 = (ISPCTriangleMesh*)geometry1;

      assert(mesh0->numTimeSteps == mesh1->numTimeSteps);
      assert(mesh0->numVertices  == mesh1->numVertices);

      for (size_t t=0; t<mesh0->numTimeSteps; t++) {
        //rtcSetBuffer(scene_out, geomID, (RTCBufferType)(RTC_VERTEX_BUFFER+t),mesh->positions+t*mesh->numVertices, 0, sizeof(Vec3fa      ));
        Vec3fa* __restrict__ vertices = (Vec3fa*) rtcMapBuffer(scene_out,geomID,(RTCBufferType)(RTC_VERTEX_BUFFER+t));
        const Vec3fa* __restrict__ const input0 = mesh0->positions+t*mesh0->numVertices;
        const Vec3fa* __restrict__ const input1 = mesh1->positions+t*mesh1->numVertices;

        parallel_for(size_t(0),size_t(mesh0->numVertices),[&](const range<size_t>& range) {
            for (size_t i=range.begin(); i<range.end(); i++)
              vertices[i] = lerp(input0[i],input1[i],t);
          }); 

        rtcUnmapBuffer(scene_out, geomID, (RTCBufferType)(RTC_VERTEX_BUFFER+t));

      }
      rtcUpdate(scene_out,geomID);
    }
    else if (geometry0->type == QUAD_MESH) {
      unsigned int geomID = ((ISPCQuadMesh*)geometry0)->geomID;
      rtcUpdate(scene_out,geomID);
    }
    else if (geometry0->type == LINE_SEGMENTS) {
      unsigned int geomID = ((ISPCLineSegments*)geometry0)->geomID;
      rtcUpdate(scene_out,geomID);
    }
    else if (geometry0->type == HAIR_SET) {
      unsigned int geomID = ((ISPCHairSet*)geometry0)->geomID;
      rtcUpdate(scene_out,geomID);
    }
    else if (geometry0->type == CURVES) {
      unsigned int geomID = ((ISPCHairSet*)geometry0)->geomID;
      rtcUpdate(scene_out,geomID);
    }
    else
      assert(false);
  }

#if VECTOR_MODE  == 1
  
  template<bool fillDistanceMap>
  void renderTile8x8(int taskIndex,
                     int* pixels,
                     const unsigned int width,
                     const unsigned int height,
                     const float time,
                     const ISPCCamera& camera,
                     const int numTilesX,
                     const int numTilesY)
  {
    assert(TILE_SIZE_X == 8);
    assert(TILE_SIZE_Y == 8);

    const unsigned int tileY = taskIndex / numTilesX;
    const unsigned int tileX = taskIndex - tileY * numTilesX;
    const unsigned int x0 = tileX * TILE_SIZE_X;
    const unsigned int x1 = min(x0+TILE_SIZE_X,width);
    const unsigned int y0 = tileY * TILE_SIZE_Y;
    const unsigned int y1 = min(y0+TILE_SIZE_Y,height);

    RTCRay8 rays[TILE_SIZE_Y];
    const Vec3fa ray_org(camera.xfm.p);
    for (unsigned int i=0; i<TILE_SIZE_Y; i++) 
      {
        const vfloat8 x(vfloat8(step) + x0);
        const vfloat8 y((float)(y0 + i));
        const vboolf8 mask((x < vfloat8(x1)) & (y < vfloat8(y1))); 
        vfloat8::store(rays[i].orgx,ray_org.x);
        vfloat8::store(rays[i].orgy,ray_org.y);
        vfloat8::store(rays[i].orgz,ray_org.z);
        vfloat8::store(rays[i].dirx,x * camera.xfm.l.vx.x + y * camera.xfm.l.vy.x + camera.xfm.l.vz.x);
        vfloat8::store(rays[i].diry,x * camera.xfm.l.vx.y + y * camera.xfm.l.vy.y + camera.xfm.l.vz.y);
        vfloat8::store(rays[i].dirz,x * camera.xfm.l.vx.z + y * camera.xfm.l.vy.z + camera.xfm.l.vz.z);
        vfloat8::store(rays[i].tnear,0.0f);
        vfloat8::store(rays[i].tfar,select(mask,vfloat8(pos_inf),vfloat8(-1.0f)));
        vfloat8::store(rays[i].time,0.0f);
        vint8::store(rays[i].mask,-1);
        vint8::store(rays[i].geomID,RTC_INVALID_GEOMETRY_ID);
        vint8::store(rays[i].primID,RTC_INVALID_GEOMETRY_ID);
      }

    RTCIntersectContext context;
    context.flags = RAYN_FLAGS;

    /* trace stream of rays */
    rtcIntersectNM(g_scene,&context,(RTCRayN *)rays,8,8,sizeof(RTCRay8));

    /* shade stream of rays */
    for (unsigned int i=0; i<TILE_SIZE_Y; i++) 
      {
        const Vec3<vfloat8> ray_dir(vfloat8::load(rays[i].dirx),vfloat8::load(rays[i].diry),vfloat8::load(rays[i].dirz));
        const Vec3<vfloat8> ray_Ng(vfloat8::load(rays[i].Ngx),vfloat8::load(rays[i].Ngy),vfloat8::load(rays[i].Ngz));        
        const vfloat8 color = abs(dot(normalize(ray_dir),normalize(ray_Ng)));

        const vint8 r = (vint8) (255.0f * clamp(color,vfloat8(0.0f),vfloat8(1.0f)));
        const vint8 g = (vint8) (255.0f * clamp(color,vfloat8(0.0f),vfloat8(1.0f)));
        const vint8 b = (vint8) (255.0f * clamp(color,vfloat8(0.0f),vfloat8(1.0f)));   
        const vint8 rgb = select(vint8::load(rays[i].geomID) != vint8(RTC_INVALID_GEOMETRY_ID),(b << 16) + (g << 8) + r,vint8(zero));

        vint8::storeu(&pixels[(y0+i)*width+x0],rgb);
      }

    if (fillDistanceMap)
    {
      // -vp -1.513921618 13.41725254 -8.156237602 -vi -0.3423615694 3.748182058 -3.199102163 -vu 0 1 0 -fov 90
      const Vec3<vfloat8> shadow_org(vfloat8(-1.513921618),vfloat8(13.41725254),vfloat8(-8.156237602));

      for (unsigned int i=0; i<TILE_SIZE_Y; i++) 
      {
        const Vec3<vfloat8> ray_dir(vfloat8::load(rays[i].dirx),vfloat8::load(rays[i].diry),vfloat8::load(rays[i].dirz));
        const Vec3<vfloat8> ray_org(vfloat8::load(rays[i].orgx),vfloat8::load(rays[i].orgy),vfloat8::load(rays[i].orgz));
        const vfloat8 ray_dist(vfloat8::load(rays[i].tfar));
        
        vfloat8::storeu(&zBuffer[(y0+i)*width+x0],ray_dist);

        const Vec3<vfloat8> pos = ray_org + ray_dir * ray_dist;
        const Vec3<vfloat8> shadow_dir = pos - shadow_org;

        const vfloat8 shadow_dist(0.99f);

        const vbool8 mask = vint8::load(rays[i].geomID) != vint8(RTC_INVALID_GEOMETRY_ID);

        vfloat8::store(rays[i].orgx,shadow_org.x);
        vfloat8::store(rays[i].orgy,shadow_org.y);
        vfloat8::store(rays[i].orgz,shadow_org.z);
        vfloat8::store(rays[i].dirx,shadow_dir.x);
        vfloat8::store(rays[i].diry,shadow_dir.y);
        vfloat8::store(rays[i].dirz,shadow_dir.z);
        vfloat8::store(rays[i].tnear,0.0f);
        vfloat8::store(rays[i].tfar,select(mask,shadow_dist,vfloat8(-1.0f)));
        vfloat8::store(rays[i].time,0.0f);
        vint8::store(rays[i].mask,-1);
        vint8::store(rays[i].geomID,RTC_INVALID_GEOMETRY_ID);
        vint8::store(rays[i].primID,RTC_INVALID_GEOMETRY_ID);
      }

      /* trace stream of rays */
      rtcIntersectNM(g_scene,&context,(RTCRayN *)rays,8,8,sizeof(RTCRay8));

      /* store distance map */
      for (unsigned int i=0; i<TILE_SIZE_Y; i++) 
      {
        const vbool8 mask = vint8::load(rays[i].geomID) == vint8(RTC_INVALID_GEOMETRY_ID);
        vfloat8::storeu(&shadowDistanceMap[(y0+i)*width+x0],select(mask,1.0f,vfloat8::load(rays[i].tfar)));
      }
    }
  }


  void filterShadowDistanceMap8x8(int taskIndex,
                                  int* pixels,
                                  const unsigned int width,
                                  const unsigned int height,
                                  const float time,
                                  const ISPCCamera& camera,
                                  const int numTilesX,
                                  const int numTilesY)
  {
    assert(TILE_SIZE_X == 8);
    assert(TILE_SIZE_Y == 8);


    const unsigned int tileY = taskIndex / numTilesX;
    const unsigned int tileX = taskIndex - tileY * numTilesX;
    const unsigned int x0 = tileX * TILE_SIZE_X;
    const unsigned int x1 = min(x0+TILE_SIZE_X,width);
    const unsigned int y0 = tileY * TILE_SIZE_Y;
    const unsigned int y1 = min(y0+TILE_SIZE_Y,height);

    vfloat8 factor[TILE_SIZE_Y];
    for (size_t i=0;i<TILE_SIZE_Y;i++) 
    {
      factor[i] = vfloat8::load(&shadowDistanceMap[(y0+i)*width+x0]);
      //factor[i] = select(factor[i] < 1.0f, vfloat8(0.0f), factor[i]);
      factor[i] = select(factor[i] < 1.0f, vfloat8(0.0f), vfloat8(1.0f));
    }

    for (unsigned int x=x0;x<x1;x++)
      for (unsigned int y=y0;y<y1;y++)
      {

        const int x_p1 = min((int)x+1,(int)width);
        const int x_m1 = max((int)x-1,0);

        const int y_p1 = min((int)y+1,(int)height);
        const int y_m1 = max((int)y-1,0);

        const float z_xy = zBuffer[y*width+x];
        float z_dx = min(abs(z_xy-zBuffer[y*width+x_m1]),abs(z_xy-zBuffer[y*width+x_p1]));
        float z_dy = max(abs(z_xy-zBuffer[y_m1*width+x]),abs(z_xy-zBuffer[y_p1*width+x]));
        if (z_dx < z_dy) z_dy = z_dx;
        if (z_dy < z_dx) z_dx = z_dy;

#if 1
        if (shadowDistanceMap[y*width+x] < 1.0f)
        {
          int FILTER_WIDTH = max((int)( (1.0f - shadowDistanceMap[y*width+x]) * 0.5f / z_xy),1);
          FILTER_WIDTH = min(FILTER_WIDTH,8);


          const int min_x = max((int)x-FILTER_WIDTH,0);
          const int max_x = min((int)x+FILTER_WIDTH,(int)width);

          const int min_y = max((int)y-FILTER_WIDTH,0);
          const int max_y = min((int)y+FILTER_WIDTH,(int)height);



          int min_sqrdist = 2*FILTER_WIDTH*FILTER_WIDTH;
          size_t validTests = 0;
#if 1
          for (int mx=min_x;mx<max_x;mx++)
            for (int my=min_y;my<max_y;my++)
              if (mx != x && my != y && shadowDistanceMap[my*width+mx] == 1.0f)
              {
                const int stepsx = std::abs(mx-(int)x);
                const int stepsy = std::abs(my-(int)y);
                const float deltaz = abs(zBuffer[my*width+mx] - z_xy);
                if (deltaz >= stepsx * z_dx + stepsy * z_dy + 1E-3f) continue; 
                validTests++;
                const int sqrdist = (mx-x)*(mx-x)+(my-y)*(my-y);
                if ( sqrdist < min_sqrdist) min_sqrdist = sqrdist;
              }
#else
          for (int mx=min_x;mx<max_x;mx++)
            if (mx != x && shadowDistanceMap[y*width+mx] == 1.0f)
              {
                const int stepsx = std::abs(mx-(int)x);
                const float deltaz = abs(zBuffer[y*width+mx] - z_xy);
                if (deltaz >= stepsx * z_dx + 1E-3f) continue; 
                validTests++;

                const int sqrdist = (mx-x)*(mx-x);
                if ( sqrdist < min_sqrdist) min_sqrdist = sqrdist;
              }
          for (int my=min_y;my<max_y;my++)
            if (my != y && shadowDistanceMap[my*width+x] == 1.0f)
              {
                const int stepsy = std::abs(my-(int)y);
                const float deltaz = abs(zBuffer[my*width+x] - z_xy);
                if (deltaz >= stepsy * z_dy + 1E-3f) continue; 
                validTests++;

                const int sqrdist = (my-y)*(my-y);
                if ( sqrdist < min_sqrdist) min_sqrdist = sqrdist;
              }

#endif
          if (min_sqrdist == 2*FILTER_WIDTH*FILTER_WIDTH || validTests <= 3)
            factor[y-y0][x-x0] = 0.0f;
          else
          {
            factor[y-y0][x-x0] = 1.0f - (float)sqrtf(min_sqrdist) / sqrtf(2*FILTER_WIDTH*FILTER_WIDTH);
          }
        }
#endif            
      }

    for (unsigned int i=0; i<TILE_SIZE_Y; i++) 
    {
      const vint8 rgb = vint8::load(&pixels[(y0+i)*width+x0]);
      const vint8 r = vint8(vfloat8((rgb >>  0) & 0xff) * factor[i]);
      const vint8 g = vint8(vfloat8((rgb >>  8) & 0xff) * factor[i]);
      const vint8 b = vint8(vfloat8((rgb >> 16) & 0xff) * factor[i]);
      vint8::storeu(&pixels[(y0+i)*width+x0],(b << 16) + (g << 8) + r);
    }      
    
  }
#endif

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

    RTCRay rays[TILE_SIZE_X*TILE_SIZE_Y];

    /* generate stream of primary rays */
    int N = 0;
    for (unsigned int y=y0; y<y1; y++) 
      for (unsigned int x=x0; x<x1; x++)
      {
        /* initialize ray */
        RTCRay& ray = rays[N++];

        ray.org = Vec3fa(camera.xfm.p);
        ray.dir = Vec3fa(normalize((float)x*camera.xfm.l.vx + (float)y*camera.xfm.l.vy + camera.xfm.l.vz));
        bool mask = 1; { // invalidates inactive rays
          ray.tnear = mask ? 0.0f         : (float)(pos_inf);
          ray.tfar  = mask ? (float)(inf) : (float)(neg_inf);
        }
        ray.geomID = RTC_INVALID_GEOMETRY_ID;
        ray.primID = RTC_INVALID_GEOMETRY_ID;
        ray.mask = -1;
        ray.time = 0.0f;
      }

    RTCIntersectContext context;
    context.flags = RAYN_FLAGS;

    /* trace stream of rays */
    rtcIntersect1M(g_scene,&context,rays,N,sizeof(RTCRay));
    //rtcOccluded1M(g_scene,&context,rays,N,sizeof(RTCRay));

    /* shade stream of rays */
    N = 0;
    for (unsigned int y=y0; y<y1; y++) 
      for (unsigned int x=x0; x<x1; x++)
      {
        /* ISPC workaround for mask == 0 */
    
        RTCRay& ray = rays[N++];

        /* eyelight shading */
        Vec3fa color = Vec3fa(0.0f);
        if (ray.geomID != RTC_INVALID_GEOMETRY_ID)
          color = Vec3fa(abs(dot(ray.dir,normalize(ray.Ng))));

        /* write color to framebuffer */
        unsigned int r = (unsigned int) (255.0f * clamp(color.x,0.0f,1.0f));
        unsigned int g = (unsigned int) (255.0f * clamp(color.y,0.0f,1.0f));
        unsigned int b = (unsigned int) (255.0f * clamp(color.z,0.0f,1.0f));
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

/* called by the C++ code for initialization */
  extern "C" void device_init (char* cfg)
  {
    /* create new Embree device */
    g_device = rtcNewDevice(cfg);
    error_handler(rtcDeviceGetError(g_device));

    /* set error handler */
    rtcDeviceSetErrorFunction(g_device,error_handler);


    /* create scene */
    g_scene = createScene(g_ispc_scene,true);

    /* instantiate the first two objects (static,dynamic) */
#if 0
    /* num objects */
    size_t numObjects = getNumObjects(g_ispc_scene);
    PRINT(numObjects);

    for (size_t i=0;i<numObjects;i++)
      createObject(i,g_ispc_scene,g_scene,true);
#else
    staticID  = createObject(0,g_ispc_scene,g_scene,false);
    dynamicID = createObject(1,g_ispc_scene,g_scene,true);
    PRINT(staticID);
    PRINT(dynamicID);

#endif

    rtcCommit (g_scene);

    if (!timeInitialized)
    {
      timeInitialized = true;

      buildTime.resize(numProfileFrames);
      renderTime.resize(numProfileFrames);
      vertexUpdateTime.resize(numProfileFrames);

      for (size_t i=0;i<numProfileFrames;i++)
      {
        buildTime[i] = 0.0;
        renderTime[i] = 0.0;
        vertexUpdateTime[i] = 0.0;
      }
      
    }

    /* set render tile function to use */
    renderTile = renderTileStandard;
    key_pressed_handler = device_key_pressed_handler;
  }


  __forceinline void updateTimeLog(std::vector<double> &times, double time)
  {
    if (times[frameID] > 0.0f) 
      times[frameID] = (times[frameID] + time) * 0.5f;
    else
      times[frameID] = time;    
  }

/* called by the C++ code to render */
  extern "C" void device_render (int* pixels,
                                 const unsigned int width,
                                 const unsigned int height,
                                 const float time,
                                 const ISPCCamera& camera)
  {
    assert(frameID < renderTime.size());
    assert(frameID < vertexUpdateTime.size());
    assert(frameID < buildTime.size());

    /* ======================================= */
    /* create shadow distance map and z-buffer */
    /* ======================================= */

    if (!shadowDistanceMap)
      shadowDistanceMap = (float*) alignedMalloc(width*height*sizeof(float),64);

    if (!zBuffer)
      zBuffer = (float*) alignedMalloc(width*height*sizeof(float),64);


    /* ============ */
    /* render image */
    /* ============ */

    const double renderTime0 = getSeconds();
    const int numTilesX = (width +TILE_SIZE_X-1)/TILE_SIZE_X;
    const int numTilesY = (height+TILE_SIZE_Y-1)/TILE_SIZE_Y;

#if VECTOR_MODE == 0

    parallel_for(size_t(0),size_t(numTilesX*numTilesY),[&](const range<size_t>& range) {
        for (size_t i=range.begin(); i<range.end(); i++)
#if VECTOR_MODE  == 1
          renderTile8x8<false>((int)i,pixels,width,height,time,camera,numTilesX,numTilesY);
#else
        renderTileTask((int)i,pixels,width,height,time,camera,numTilesX,numTilesY);
#endif
      }); 

#else
    /* trace primary visibility and fill shadow distance buffer */

    parallel_for(size_t(0),size_t(numTilesX*numTilesY),[&](const range<size_t>& range) {
        for (size_t i=range.begin(); i<range.end(); i++)
          renderTile8x8<true>((int)i,pixels,width,height,time,camera,numTilesX,numTilesY);
      }); 
    
    /* filter shadow distance buffer and update framebuffer */

    parallel_for(size_t(0),size_t(numTilesX*numTilesY),[&](const range<size_t>& range) {
        for (size_t i=range.begin(); i<range.end(); i++)
          filterShadowDistanceMap8x8((int)i,pixels,width,height,time,camera,numTilesX,numTilesY);
      }); 


#endif

    const double renderTime1 = getSeconds();
    const double renderTimeDelta = renderTime1-renderTime0;
    
    updateTimeLog(renderTime,renderTimeDelta);

    if (unlikely(printStats)) std::cout << "rendering frame in : " << renderTimeDelta << " ms" << std::endl;


    /* =============== */
    /* update geometry */
    /* =============== */

    double vertexUpdateTime0 = getSeconds();    

    if (animTime < 0.0f) animTime = getSeconds();
#if ENABLE_ANIM == 0
    const double atime = 0.0;
#else
    const double atime = (getSeconds() - animTime) * ANIM_FPS;
#endif
    double intpart, fracpart;
    fracpart = modf(atime,&intpart);
    const size_t keyFrameID = ((size_t)intpart) % numProfileFrames;    
    updateVertexData(dynamicID, g_ispc_scene, g_scene, keyFrameID, (float)fracpart);

    double vertexUpdateTime1 = getSeconds();    
    const double vertexUpdateTimeDelta = vertexUpdateTime1-vertexUpdateTime0;

    updateTimeLog(vertexUpdateTime,vertexUpdateTimeDelta);

    if (unlikely(printStats)) std::cout << "vertex update in :   " << vertexUpdateTimeDelta << " ms" << std::endl;

    /* =========== */
    /* rebuild bvh */
    /* =========== */
    
    double buildTime0 = getSeconds();
    rtcCommit(g_scene);      
    double buildTime1 = getSeconds();
    double buildTimeDelta = buildTime1-buildTime0;

    updateTimeLog(buildTime,buildTimeDelta);

    if (unlikely(printStats)) std::cout << "bvh rebuild in :     " << buildTimeDelta << " ms" << std::endl;
    
    frameID = (frameID + 1) % numProfileFrames;
  }

/* plot build and render times */

  void dumpBuildAndRenderTimes()
  {
    FileName name("buildRenderTimes");
    std::fstream plot;
    plot.open(name.addExt(".plot"), std::fstream::out | std::fstream::trunc);

    plot << "set terminal png size 1920,1080 enhanced" << std::endl;
    plot << "set output \"" << name.addExt(".png") << "\"" << std::endl;
    plot << "set key inside right top vertical Right noreverse enhanced autotitles box linetype -1 linewidth 1.000" << std::endl;
    plot << "set ylabel \"" << "ms" << "\"" << std::endl;
    plot << "set yrange [0:50]" << std::endl;
    plot << "set ytics 1" << std::endl;
    plot << "factor=1000" << std::endl;
    plot << "plot \"-\" using ($1):(factor*($2)) title \"build time\" with linespoints lw 4,\"-\" using ($1):(factor*($2)) title \"vertex update time\" with linespoints lw 4,\"-\" using ($1):(factor*($2)) title \"render time\" with linespoints lw 4,\"-\" using ($1):(factor*($2)) title \"total time\" with linespoints lw 4" << std::endl;
    for (size_t i=0;i<buildTime.size();i++)
      plot << i << " " << buildTime[i] << std::endl;
    plot << "e" << std::endl;
    for (size_t i=0;i<vertexUpdateTime.size();i++)
      plot << i << " " << vertexUpdateTime[i] << std::endl;
    plot << "e" << std::endl;
    for (size_t i=0;i<renderTime.size();i++)
      plot << i << " " << renderTime[i] << std::endl;
    plot << "e" << std::endl;
    for (size_t i=0;i<renderTime.size();i++)
      plot << i << " " << buildTime[i] + renderTime[i] + vertexUpdateTime[i] << std::endl;
    plot << std::endl;
    plot.close();
  }

/* called by the C++ code for cleanup */
  extern "C" void device_cleanup ()
  {
    if (shadowDistanceMap) alignedFree(shadowDistanceMap);
    if (zBuffer) alignedFree(zBuffer);

    rtcDeleteScene (g_scene); g_scene = nullptr;
    rtcDeleteDevice(g_device); g_device = nullptr;
    /* dump data at the end of profiling */
    std::cout << "dumping build and render times per frame [" << numProfileFrames << " frames]..." << std::flush;
    dumpBuildAndRenderTimes(); 
    std::cout << "done" << std::endl;
  }

} // namespace embree
