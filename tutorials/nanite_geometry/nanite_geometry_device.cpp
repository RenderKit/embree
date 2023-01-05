// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "nanite_geometry_device.h"

namespace embree {
#define FEATURE_MASK                            \
  RTC_FEATURE_FLAG_TRIANGLE |                   \
  RTC_FEATURE_FLAG_INSTANCE

  RTCScene g_scene  = nullptr;
  TutorialData data;

  RTCLossyCompressedGrid *compressed_geometries = nullptr;  
  void **compressed_geometries_ptrs = nullptr;
  uint num_compressed_geometries_ptrs = 0;
  
  struct LCGQuadNode
  {
    RTCLossyCompressedGrid grid;
    uint level;
    uint childID[4];  
  };
  
  LCGQuadNode *compressed_quad_trees = nullptr;  
  
  RTCGeometry global_lcg_geom = nullptr;
  
#define NUM_SUBGRIDS_X 1
#define NUM_SUBGRIDS_Y 1
  
#define SUBGRID_RESOLUTION_X RTC_LOSSY_COMPRESSED_GRID_VERTEX_RES
#define SUBGRID_RESOLUTION_Y RTC_LOSSY_COMPRESSED_GRID_VERTEX_RES
#define GRID_VERTEX_RESOLUTION_X (NUM_SUBGRIDS_X*(SUBGRID_RESOLUTION_X-1)+1)
#define GRID_VERTEX_RESOLUTION_Y (NUM_SUBGRIDS_Y*(SUBGRID_RESOLUTION_Y-1)+1)
  
  
  unsigned int createLossyCompressedGeometry (RTCScene scene)
  {
    const uint numSubGrids = NUM_SUBGRIDS_X * NUM_SUBGRIDS_Y;
    PRINT(numSubGrids);
  
    compressed_geometries = (RTCLossyCompressedGrid*)alignedUSMMalloc(sizeof(RTCLossyCompressedGrid)*numSubGrids,64);
    compressed_geometries_ptrs = (void**)alignedUSMMalloc(sizeof(void*)*numSubGrids,64);

    uint index = 0;
    for (int start_y=0;start_y+SUBGRID_RESOLUTION_Y-1<GRID_VERTEX_RESOLUTION_Y;start_y+=SUBGRID_RESOLUTION_Y-1)
      for (int start_x=0;start_x+SUBGRID_RESOLUTION_X-1<GRID_VERTEX_RESOLUTION_X;start_x+=SUBGRID_RESOLUTION_X-1)
      {
        //PRINT3(start_y,start_x,index);
        for (int y=0;y<SUBGRID_RESOLUTION_Y;y++)
          for (int x=0;x<SUBGRID_RESOLUTION_X;x++)
          {
            //PRINT3(start_y+y,start_x+x,0);
#if 1            
            compressed_geometries[index].vertex[y][x][0] = start_x + x - GRID_VERTEX_RESOLUTION_X/2;
            compressed_geometries[index].vertex[y][x][1] = start_y + y - GRID_VERTEX_RESOLUTION_Y/2;
#else
            compressed_geometries[index].vertex[y][x][0] = start_x + x;
            compressed_geometries[index].vertex[y][x][1] = start_y + y;
#endif            
            compressed_geometries[index].vertex[y][x][2] = 0;          
          }
        compressed_geometries[index].ID = index;
        compressed_geometries[index].materialID = 0;
        compressed_geometries_ptrs[index] = &compressed_geometries[index];
        index++;
      }
                                                        
    RTCGeometry geom = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_LOSSY_COMPRESSED_GEOMETRY);
    rtcSetGeometryUserData(geom,compressed_geometries_ptrs);
    rtcSetLossyCompressedGeometryPrimitiveCount(geom,numSubGrids);
    rtcCommitGeometry(geom);
    unsigned int geomID = rtcAttachGeometry(scene,geom);
    //rtcReleaseGeometry(geom);
    PRINT(index);
    if (index != numSubGrids)
      FATAL("numSubGrids");
    global_lcg_geom = geom;
    return geomID;  
  }


  static const uint LOD_LEVELS = 3;
  
#if 1

  inline Vec3fa getVertex(const uint x, const uint y, const Vec3fa *const vtx, const uint grid_resX, const uint grid_resY)
  {
    const uint px = min(x,grid_resX-1);
    const uint py = min(y,grid_resY-1);    
    return vtx[py*grid_resX + px];
  }

  void createQuadNode(LCGQuadNode &current, LCGQuadNode *nodes, uint &index, const uint start_x, const uint start_y, const uint step, const Vec3fa *const vtx, const uint grid_resX, const uint grid_resY)
  {
    if (step == 0) return;
    //PING;
    PRINT4(index,start_x,start_y,step);
      
    for (int y=0;y<RTC_LOSSY_COMPRESSED_GRID_VERTEX_RES;y++)
      for (int x=0;x<RTC_LOSSY_COMPRESSED_GRID_VERTEX_RES;x++)
      {
        const uint px = start_x+x*step;
        const uint py = start_y+y*step;
        const Vec3fa v = getVertex(px,py,vtx,grid_resX,grid_resY);
        PRINT2(px,py);
        current.grid.vertex[y][x][0] = v.x;
        current.grid.vertex[y][x][1] = v.y;
        current.grid.vertex[y][x][2] = v.z;          
      }
    current.grid.ID = index;
    current.grid.materialID = 0;    

    const uint new_step = step>>1;
    const uint new_res = RTC_LOSSY_COMPRESSED_GRID_QUAD_RES*new_step;

    if (new_step)
    {    
      const uint new_index = index;
      index += 4;

      //PRINT3(new_index,new_step,new_res);

      current.childID[0] = new_index + 0;    
      current.childID[1] = new_index + 1;    
      current.childID[2] = new_index + 2;    
      current.childID[3] = new_index + 3;    

          
      createQuadNode(nodes[new_index+0],nodes,index,start_x + 0*new_res,start_y + 0*new_res,new_step,vtx,grid_resX,grid_resY);
      createQuadNode(nodes[new_index+1],nodes,index,start_x + 1*new_res,start_y + 0*new_res,new_step,vtx,grid_resX,grid_resY);
      createQuadNode(nodes[new_index+2],nodes,index,start_x + 0*new_res,start_y + 1*new_res,new_step,vtx,grid_resX,grid_resY);
      createQuadNode(nodes[new_index+3],nodes,index,start_x + 1*new_res,start_y + 1*new_res,new_step,vtx,grid_resX,grid_resY);
    }
  }
    
  RTCGeometry convertISPCGridMesh(ISPCGridMesh* grid, RTCScene scene)
  {
    Vec3fa *vtx = grid->positions[0];
    PRINT(grid->numVertices);
    PRINT(grid->numGrids);
    PRINT(sizeof(LCGQuadNode));
    
    const int numQuadNodesPerSubGrid = (1-(1<<(2*LOD_LEVELS)))/(1-4);
    PRINT( numQuadNodesPerSubGrid );

    const uint InitialSubGridRes = (1 << (LOD_LEVELS-1)) * (RTC_LOSSY_COMPRESSED_GRID_VERTEX_RES-1);
    PRINT( InitialSubGridRes );
    
    uint numSubGrids = 0;
    uint numQuadNodes = 0;
    for (uint i=0;i<grid->numGrids;i++)
    {
      PRINT3(i,grid->grids[i].resX,grid->grids[i].resY);      
      const uint grid_resX = grid->grids[i].resX;
      const uint grid_resY = grid->grids[i].resY;
      const uint numInitialSubGrids = ((grid_resX-1) / InitialSubGridRes) * ((grid_resY-1) / InitialSubGridRes);
      PRINT(numInitialSubGrids);
      numSubGrids  += numInitialSubGrids * numQuadNodesPerSubGrid;
      numQuadNodes += numInitialSubGrids;
    }
  
    PRINT(numSubGrids);
  
    compressed_quad_trees = (LCGQuadNode*)alignedUSMMalloc(sizeof(LCGQuadNode)*numQuadNodesPerSubGrid,64);
    compressed_geometries_ptrs = (void**)alignedUSMMalloc(sizeof(void*)*numQuadNodesPerSubGrid,64); // FIXME: is < numQuadNodesPerSubGrid

    for (uint i=0;i<numQuadNodesPerSubGrid;i++)
      compressed_geometries_ptrs[i] = nullptr;
    
    uint index = 0;
    for (uint i=0;i<grid->numGrids;i++)
    {
      const uint grid_resX = grid->grids[i].resX;
      const uint grid_resY = grid->grids[i].resY;
    
      for (int start_y=0;start_y+InitialSubGridRes<grid_resY;start_y+=InitialSubGridRes)
        for (int start_x=0;start_x+InitialSubGridRes<grid_resX;start_x+=InitialSubGridRes)
        {
          LCGQuadNode *current = &compressed_quad_trees[index*numQuadNodesPerSubGrid];
          uint local_index = 1;
          createQuadNode(current[0],current,local_index,start_x,start_y,(1<<(LOD_LEVELS-1)),vtx,grid_resX,grid_resY);          
          if (local_index != numQuadNodesPerSubGrid)
          {
            PRINT2(local_index,numQuadNodesPerSubGrid);
            FATAL("numQuadNodesPerSubGrid");
          }
          //compressed_geometries_ptrs[index] = &compressed_quad_trees[index].grid;
          index++;
        }
    }
    PRINT(index);
    if (index > numSubGrids)
      FATAL("numSubGrids");

    num_compressed_geometries_ptrs = index;
    PRINT( num_compressed_geometries_ptrs );
    for (uint i=0;i<index;i++)
      compressed_geometries_ptrs[i] = &compressed_quad_trees[i*numQuadNodesPerSubGrid].grid;
    
  
    RTCGeometry geom = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_LOSSY_COMPRESSED_GEOMETRY);
    rtcSetGeometryUserData(geom,compressed_geometries_ptrs);
    rtcSetLossyCompressedGeometryPrimitiveCount(geom,num_compressed_geometries_ptrs);
    rtcCommitGeometry(geom);
    unsigned int geomID = rtcAttachGeometry(scene,geom);
    //rtcReleaseGeometry(geom);

    PRINT("DONE");
    //exit(0);
    
    return geom;
  }

#else
  
  RTCGeometry convertISPCGridMesh(ISPCGridMesh* grid, RTCScene scene)
  {
    Vec3fa *vtx = grid->positions[0];
    PRINT(grid->numVertices);
    PRINT(grid->numGrids);

    uint numSubGrids = 0;
    for (uint i=0;i<grid->numGrids;i++)
    {
      PRINT3(i,grid->grids[i].resX,grid->grids[i].resY);      
      const uint grid_resX = grid->grids[i].resX;
      const uint grid_resY = grid->grids[i].resY;
      numSubGrids += ((grid_resX-1) / (SUBGRID_RESOLUTION_X-1)) * ((grid_resY-1) / (SUBGRID_RESOLUTION_Y-1));            
    }
  
    PRINT(numSubGrids);
  
    compressed_geometries = (RTCLossyCompressedGrid*)alignedUSMMalloc(sizeof(RTCLossyCompressedGrid)*numSubGrids,64);
    compressed_geometries_ptrs = (void**)alignedUSMMalloc(sizeof(void*)*numSubGrids,64);

    uint index = 0;
    for (uint i=0;i<grid->numGrids;i++)
    {
      const uint grid_resX = grid->grids[i].resX;
      const uint grid_resY = grid->grids[i].resY;
    
      for (int start_y=0;start_y+SUBGRID_RESOLUTION_Y-1<grid_resY;start_y+=SUBGRID_RESOLUTION_Y-1)
        for (int start_x=0;start_x+SUBGRID_RESOLUTION_X-1<grid_resX;start_x+=SUBGRID_RESOLUTION_X-1)
        {
          //PRINT3(start_y,start_x,index);
          for (int y=0;y<SUBGRID_RESOLUTION_Y;y++)
            for (int x=0;x<SUBGRID_RESOLUTION_X;x++)
            {
              //PRINT3(start_y+y,start_x+x,0);
              const Vec3fa v = vtx[(start_y+y)*grid_resX + start_x + x];
              compressed_geometries[index].vertex[y][x][0] = v.x;
              compressed_geometries[index].vertex[y][x][1] = v.y;
              compressed_geometries[index].vertex[y][x][2] = v.z;          
            }
          compressed_geometries[index].ID = index;
          compressed_geometries[index].materialID = 0;
          compressed_geometries_ptrs[index] = &compressed_geometries[index];
          index++;
        }
    }
    PRINT(index);
    if (index > numSubGrids)
      FATAL("numSubGrids");
  
    RTCGeometry geom = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_LOSSY_COMPRESSED_GEOMETRY);
    rtcSetGeometryUserData(geom,compressed_geometries_ptrs);
    rtcSetLossyCompressedGeometryPrimitiveCount(geom,index);
    rtcCommitGeometry(geom);
    unsigned int geomID = rtcAttachGeometry(scene,geom);
    //rtcReleaseGeometry(geom);
    return geom;
  }

#endif
  
  extern "C" ISPCScene* g_ispc_scene;

/* called by the C++ code for initialization */
  extern "C" void device_init (char* cfg)
  {
    TutorialData_Constructor(&data);

  
    //PRINT(g_ispc_scene->ge
    /* create scene */
    data.g_scene = g_scene = rtcNewScene(g_device);
    rtcSetSceneBuildQuality(data.g_scene,RTC_BUILD_QUALITY_LOW);
    rtcSetSceneFlags(data.g_scene,RTC_SCENE_FLAG_DYNAMIC);

#if 0
    createLossyCompressedGeometry(data.g_scene);
#else
    PRINT(g_ispc_scene->numGeometries);
    for (unsigned int geomID=0; geomID<g_ispc_scene->numGeometries; geomID++)
    {
      ISPCGeometry* geometry = g_ispc_scene->geometries[geomID];
      if (geometry->type == GRID_MESH)
            global_lcg_geom = convertISPCGridMesh((ISPCGridMesh*)geometry,data.g_scene);
    }  
#endif
  
    /* update scene */
    rtcCommitScene (data.g_scene);  
  }


  Vec3fa randomColor(const int ID)
  {
    int r = ((ID+13)*17*23) & 255;
    int g = ((ID+15)*11*13) & 255;
    int b = ((ID+17)* 7*19) & 255;
    const float oneOver255f = 1.f/255.f;
    return Vec3fa(r*oneOver255f,g*oneOver255f,b*oneOver255f);
  }

/* task that renders a single screen tile */
  Vec3fa renderPixel(const TutorialData& data, float x, float y, const ISPCCamera& camera, RayStats& stats)
  {
    RTCIntersectArguments args;
    rtcInitIntersectArguments(&args);
    args.feature_mask = (RTCFeatureFlags) (FEATURE_MASK);
  
    /* initialize ray */
    Ray ray(Vec3fa(camera.xfm.p), Vec3fa(normalize(x*camera.xfm.l.vx + y*camera.xfm.l.vy + camera.xfm.l.vz)), 0.0f, inf);

    /* intersect ray with scene */
    rtcIntersect1(data.g_scene,RTCRayHit_(ray),&args);

    /* shade pixels */  
    if (ray.geomID == RTC_INVALID_GEOMETRY_ID)
      return Vec3fa(0.0f);
    else
      //return Vec3fa(abs(dot(ray.dir,normalize(ray.Ng))));
      return Vec3fa(abs(dot(ray.dir,normalize(ray.Ng)))) * randomColor(ray.primID);
      //return randomColor(ray.primID);  
  }

  void renderPixelStandard(const TutorialData& data,
                           int x, int y, 
                           int* pixels,
                           const unsigned int width,
                           const unsigned int height,
                           const float time,
                           const ISPCCamera& camera, RayStats& stats)
  {
    /* calculate pixel color */
    Vec3fa color = renderPixel(data, (float)x,(float)y,camera, stats);
  
    /* write color to framebuffer */
    unsigned int r = (unsigned int) (255.0f * clamp(color.x,0.0f,1.0f));
    unsigned int g = (unsigned int) (255.0f * clamp(color.y,0.0f,1.0f));
    unsigned int b = (unsigned int) (255.0f * clamp(color.z,0.0f,1.0f));
    pixels[y*width+x] = (b << 16) + (g << 8) + r;
  }

/* renders a single screen tile */
  void renderTileStandard(int taskIndex,
                          int threadIndex,
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

    for (unsigned int y=y0; y<y1; y++)
      for (unsigned int x=x0; x<x1; x++)
        renderPixelStandard(data,x,y,pixels,width,height,time,camera,g_stats[threadIndex]);
  }

/* task that renders a single screen tile */
  void renderTileTask (int taskIndex, int threadIndex, int* pixels,
                       const unsigned int width,
                       const unsigned int height,
                       const float time,
                       const ISPCCamera& camera,
                       const int numTilesX,
                       const int numTilesY)
  {
  }

  extern "C" void renderFrameStandard (int* pixels,
                                       const unsigned int width,
                                       const unsigned int height,
                                       const float time,
                                       const ISPCCamera& camera)
  {
    /* render all pixels */
#if defined(EMBREE_SYCL_TUTORIAL)
    TutorialData ldata = data;
    sycl::event event = global_gpu_queue->submit([=](sycl::handler& cgh){
                                                   const sycl::nd_range<2> nd_range = make_nd_range(height,width);
                                                   cgh.parallel_for(nd_range,[=](sycl::nd_item<2> item) {
                                                                               const unsigned int x = item.get_global_id(1); if (x >= width ) return;
                                                                               const unsigned int y = item.get_global_id(0); if (y >= height) return;
                                                                               RayStats stats;
                                                                               renderPixelStandard(ldata,x,y,pixels,width,height,time,camera,stats);
                                                                             });
                                                 });
    global_gpu_queue->wait_and_throw();

    const auto t0 = event.template get_profiling_info<sycl::info::event_profiling::command_start>();
    const auto t1 = event.template get_profiling_info<sycl::info::event_profiling::command_end>();
    const double dt = (t1-t0)*1E-9;
    ((ISPCCamera*)&camera)->render_time = dt;
  
#else
    const int numTilesX = (width +TILE_SIZE_X-1)/TILE_SIZE_X;
    const int numTilesY = (height+TILE_SIZE_Y-1)/TILE_SIZE_Y;
    parallel_for(size_t(0),size_t(numTilesX*numTilesY),[&](const range<size_t>& range) {
                                                         const int threadIndex = (int)TaskScheduler::threadIndex();
                                                         for (size_t i=range.begin(); i<range.end(); i++)
                                                           renderTileTask((int)i,threadIndex,pixels,width,height,time,camera,numTilesX,numTilesY);
                                                       }); 
#endif
  }

/* called by the C++ code to render */
  extern "C" void device_render (int* pixels,
                                 const unsigned int width,
                                 const unsigned int height,
                                 const float time,
                                 const ISPCCamera& camera)
  {
    //rtcSetLossyCompressedGeometryPrimitiveCount(global_lcg_geom,num_compressed_geometries_ptrs);
    //rtcCommitGeometry(global_lcg_geom);
    
    /* commit changes to scene */
    rtcCommitScene (data.g_scene);
  }

/* called by the C++ code for cleanup */
  extern "C" void device_cleanup ()
  {
    TutorialData_Destructor(&data);
  }

} // namespace embree
