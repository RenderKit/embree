// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "nanite_geometry_device.h"

namespace embree {
#define FEATURE_MASK                            \
  RTC_FEATURE_FLAG_TRIANGLE |                   \
  RTC_FEATURE_FLAG_INSTANCE

  RTCScene g_scene  = nullptr;
  TutorialData data;

  enum {
    NO_BORDER     = 0,
    TOP_BORDER    = 1 << 0,
    BOTTOM_BORDER = 1 << 1,
    LEFT_BORDER   = 1 << 2,
    RIGHT_BORDER  = 1 << 3,
    FULL_BORDER   = TOP_BORDER|BOTTOM_BORDER|LEFT_BORDER|RIGHT_BORDER
  };
  
  struct LCGQuadNode
  {
    RTCLossyCompressedGrid grid;
    uint flags;

    __forceinline bool hasBorder() const { return flags & FULL_BORDER; }
    __forceinline Vec3f getVertex(const uint x, const uint y) { return Vec3f(grid.vertex[y][x][0],grid.vertex[y][x][1],grid.vertex[y][x][2]); }
    
  }; 

  __forceinline BBox3f getGridBounds(const RTCLossyCompressedGrid &grid)
  {
    BBox3f bounds(empty);
    for (uint y=0;y<RTC_LOSSY_COMPRESSED_GRID_VERTEX_RES;y++)
      for (uint x=0;x<RTC_LOSSY_COMPRESSED_GRID_VERTEX_RES;x++)
      {
        Vec3f v(grid.vertex[y][x][0],grid.vertex[y][x][1],grid.vertex[y][x][2]);
        bounds.extend(v);
      }
    return bounds;
  }

  
  static const uint LOD_LEVELS = 3;
  static const uint NUM_TOTAL_QUAD_NODES_PER_RTC_LCG = (1-(1<<(2*LOD_LEVELS)))/(1-4);

  struct LODEdgeLevel {
    uchar top,right,bottom,left; // top,right,bottom,left
    __forceinline LODEdgeLevel() {}
    __forceinline LODEdgeLevel(const uchar top, const uchar right, const uchar bottom, const uchar left) : top(top),right(right),bottom(bottom),left(left) {}
    __forceinline uint level() const { return max(max(top,right),max(bottom,left)); }

    __forceinline bool needsCrackFixing() const {
      const uint l = level();
      if ( l != top || l != right || l != bottom || l != left) return true;
      return false;
    }
  };

  __forceinline Vec2f projectVertexToPlane(const Vec3f &p, const Vec3f &vx, const Vec3f &vy, const Vec3f &vz, const uint width, const uint height)
  {
    const Vec3f vn = cross(vx,vy);    
    const float distance = (float)dot(vn,vz) / (float)dot(vn,p);
    const Vec3f pip = p * distance;
    const float a = dot((pip-vz),vx);
    const float b = dot((pip-vz),vy);
    return Vec2f(a,b);
  }
  
  __forceinline LODEdgeLevel getLODEdgeLevels(LCGQuadNode &current,const ISPCCamera& camera, const uint width, const uint height)
  {
    const Vec3f v0 = current.getVertex(0,0);
    const Vec3f v1 = current.getVertex(RTC_LOSSY_COMPRESSED_GRID_VERTEX_RES-1,0);
    const Vec3f v2 = current.getVertex(RTC_LOSSY_COMPRESSED_GRID_VERTEX_RES-1,RTC_LOSSY_COMPRESSED_GRID_VERTEX_RES-1);
    const Vec3f v3 = current.getVertex(0,RTC_LOSSY_COMPRESSED_GRID_VERTEX_RES-1);

    const Vec3f vx = camera.xfm.l.vx;
    const Vec3f vy = camera.xfm.l.vy;
    const Vec3f vz = camera.xfm.l.vz;
    const Vec3f org = camera.xfm.p;

    const Vec2f p0 = projectVertexToPlane(v0-org,vx,vy,vz,width,height);
    const Vec2f p1 = projectVertexToPlane(v1-org,vx,vy,vz,width,height);
    const Vec2f p2 = projectVertexToPlane(v2-org,vx,vy,vz,width,height);
    const Vec2f p3 = projectVertexToPlane(v3-org,vx,vy,vz,width,height);

    const float f = 1.0/8.0f;
    const float d0 = length(p1-p0) * f;
    const float d1 = length(p2-p1) * f;
    const float d2 = length(p3-p2) * f;
    const float d3 = length(p0-p3) * f;
    
    int i0 = (int)floorf(d0 / RTC_LOSSY_COMPRESSED_GRID_QUAD_RES);
    int i1 = (int)floorf(d1 / RTC_LOSSY_COMPRESSED_GRID_QUAD_RES);
    int i2 = (int)floorf(d2 / RTC_LOSSY_COMPRESSED_GRID_QUAD_RES);
    int i3 = (int)floorf(d3 / RTC_LOSSY_COMPRESSED_GRID_QUAD_RES);

    // PRINT4(p0,p1,p2,p3);        
    // PRINT4(d0,d1,d2,d3);        
    // PRINT4(i0,i1,i2,i3);    
    
    i0 = min(max(0,i0),(int)LOD_LEVELS-1);
    i1 = min(max(0,i1),(int)LOD_LEVELS-1);
    i2 = min(max(0,i2),(int)LOD_LEVELS-1);
    i3 = min(max(0,i3),(int)LOD_LEVELS-1);
        
    LODEdgeLevel lod_levels(i0,i1,i2,i3);
    return lod_levels;
  }

    __forceinline uint getCrackFixingBorderMask(LCGQuadNode &current, const LODEdgeLevel &lodEdgeLevel, const uint gridLODLevel)
    {
      uint mask = 0;
      if ((current.flags & TOP_BORDER   ) && lodEdgeLevel.top    != gridLODLevel) mask |= TOP_BORDER;
      if ((current.flags & BOTTOM_BORDER) && lodEdgeLevel.bottom != gridLODLevel) mask |= BOTTOM_BORDER;
      if ((current.flags & RIGHT_BORDER ) && lodEdgeLevel.right  != gridLODLevel) mask |= RIGHT_BORDER;
      if ((current.flags & LEFT_BORDER  ) && lodEdgeLevel.left   != gridLODLevel) mask |= LEFT_BORDER;
      return mask;
    }
  
  __forceinline void fixCracks(LCGQuadNode &current, const uint borderMask, const LODEdgeLevel &lodEdgeLevel, const uint gridLODLevel)
  {
    if (borderMask & TOP_BORDER)
    {
      const uint diff = gridLODLevel - lodEdgeLevel.top;
      for (uint i=1;i<RTC_LOSSY_COMPRESSED_GRID_VERTEX_RES-1;i++)
      {
        const uint index = (i>>diff)<<diff;
        current.grid.vertex[0][i][0] = current.grid.vertex[0][index][0];
        current.grid.vertex[0][i][1] = current.grid.vertex[0][index][1];
        current.grid.vertex[0][i][2] = current.grid.vertex[0][index][2];        
      }
    }

    if (borderMask & BOTTOM_BORDER)
    {
      const uint diff = gridLODLevel - lodEdgeLevel.bottom;
      for (uint i=1;i<RTC_LOSSY_COMPRESSED_GRID_VERTEX_RES-1;i++)
      {
        const uint index = (i>>diff)<<diff;
        current.grid.vertex[8][i][0] = current.grid.vertex[8][index][0];
        current.grid.vertex[8][i][1] = current.grid.vertex[8][index][1];
        current.grid.vertex[8][i][2] = current.grid.vertex[8][index][2];        
      }
    }
    
    if (borderMask & RIGHT_BORDER)
    {
      const uint diff = gridLODLevel - lodEdgeLevel.right;
      for (uint i=1;i<RTC_LOSSY_COMPRESSED_GRID_VERTEX_RES-1;i++)
      {
        const uint index = (i>>diff)<<diff;
        current.grid.vertex[i][8][0] = current.grid.vertex[index][8][0];
        current.grid.vertex[i][8][1] = current.grid.vertex[index][8][1];
        current.grid.vertex[i][8][2] = current.grid.vertex[index][8][2];        
      }
    }

    if (borderMask & LEFT_BORDER)
    {
      const uint diff = gridLODLevel - lodEdgeLevel.left;
      for (uint i=1;i<RTC_LOSSY_COMPRESSED_GRID_VERTEX_RES-1;i++)
      {
        const uint index = (i>>diff)<<diff;
        current.grid.vertex[i][0][0] = current.grid.vertex[index][0][0];
        current.grid.vertex[i][0][1] = current.grid.vertex[index][0][1];
        current.grid.vertex[i][0][2] = current.grid.vertex[index][0][2];        
      }
    }
    
    
  }
  
  
  struct LCG_LODQuadTree_Grid {    
    uint numQuadTrees;
    uint numTotalQuadNodes;
    uint numMaxResQuadNodes;

    LCGQuadNode *quadTrees;

    uint num_lcg_ptrs;
    void **lcg_ptrs;

    uint numCrackFixQuadNodes;
    LCGQuadNode *crackFixQuadNodes;
    
    RTCGeometry geometry;
    uint geomID;
  };

  LCG_LODQuadTree_Grid global_grid;
    
  extern "C" uint user_lod_level = 1;
  
  inline Vec3fa getVertex(const uint x, const uint y, const Vec3fa *const vtx, const uint grid_resX, const uint grid_resY)
  {
    const uint px = min(x,grid_resX-1);
    const uint py = min(y,grid_resY-1);    
    return vtx[py*grid_resX + px];
  }

  void createQuadNode(LCGQuadNode &current, LCGQuadNode *nodes, uint &index, const uint start_x, const uint start_y, const uint step, const Vec3fa *const vtx, const uint grid_resX, const uint grid_resY, const uint border_flags)
  {
    if (step == 0) return;
      
    for (int y=0;y<RTC_LOSSY_COMPRESSED_GRID_VERTEX_RES;y++)
      for (int x=0;x<RTC_LOSSY_COMPRESSED_GRID_VERTEX_RES;x++)
      {
        const uint px = start_x+x*step;
        const uint py = start_y+y*step;
        const Vec3fa v = getVertex(px,py,vtx,grid_resX,grid_resY);
        current.grid.vertex[y][x][0] = v.x;
        current.grid.vertex[y][x][1] = v.y;
        current.grid.vertex[y][x][2] = v.z;          
      }

    // PRINT4(index,step,start_x,start_y);
    // if (border_flags &   TOP_BORDER) PRINT("TOP");
    // if (border_flags &  LEFT_BORDER) PRINT("LEFT");
    // if (border_flags & RIGHT_BORDER) PRINT("RIGHT");
    // if (border_flags & BOTTOM_BORDER) PRINT("BOTTOM");
    
    current.flags = border_flags;    
      
    const uint new_step = step>>1;
    const uint new_res = RTC_LOSSY_COMPRESSED_GRID_QUAD_RES*new_step;

    if (new_step)
    {    
      const uint new_index = index;
      index += 4;
      
      createQuadNode(nodes[new_index+0],nodes,index,start_x + 0*new_res,start_y + 0*new_res,new_step,vtx,grid_resX,grid_resY,border_flags & (LEFT_BORDER|TOP_BORDER));
      createQuadNode(nodes[new_index+1],nodes,index,start_x + 1*new_res,start_y + 0*new_res,new_step,vtx,grid_resX,grid_resY,border_flags & (RIGHT_BORDER|TOP_BORDER));
      createQuadNode(nodes[new_index+2],nodes,index,start_x + 0*new_res,start_y + 1*new_res,new_step,vtx,grid_resX,grid_resY,border_flags & (LEFT_BORDER|BOTTOM_BORDER));
      createQuadNode(nodes[new_index+3],nodes,index,start_x + 1*new_res,start_y + 1*new_res,new_step,vtx,grid_resX,grid_resY,border_flags & (RIGHT_BORDER|BOTTOM_BORDER));
    }
  }
    
  void convertISPCGridMesh(ISPCGridMesh* grid, RTCScene scene)
  {
    Vec3fa *vtx = grid->positions[0];
    
    PRINT(grid->numVertices);
    PRINT(grid->numGrids);
    PRINT(sizeof(LCGQuadNode));    
    PRINT( NUM_TOTAL_QUAD_NODES_PER_RTC_LCG );

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
      numSubGrids  += numInitialSubGrids;
      numQuadNodes += numInitialSubGrids * NUM_TOTAL_QUAD_NODES_PER_RTC_LCG;
    }
  
    PRINT(numSubGrids);

    global_grid.numQuadTrees = numSubGrids;
    global_grid.numTotalQuadNodes = numQuadNodes;
    global_grid.numMaxResQuadNodes = (1<<(2*(LOD_LEVELS-1))) * numSubGrids;
    PRINT2( (1<<(2*(LOD_LEVELS-1))), global_grid.numMaxResQuadNodes );
    global_grid.quadTrees = (LCGQuadNode*)alignedUSMMalloc(sizeof(LCGQuadNode)*global_grid.numTotalQuadNodes,64);
    global_grid.lcg_ptrs   = (void**)alignedUSMMalloc(sizeof(void*)*global_grid.numMaxResQuadNodes,64);
    global_grid.crackFixQuadNodes = (LCGQuadNode*)alignedUSMMalloc(sizeof(LCGQuadNode)*global_grid.numMaxResQuadNodes,64); // FIXME: only borders at highest resolution
      
    for (uint i=0;i<global_grid.numMaxResQuadNodes;i++)
      global_grid.lcg_ptrs[i] = nullptr;
    
    uint index = 0;
    for (uint i=0;i<grid->numGrids;i++)
    {
      const uint grid_resX = grid->grids[i].resX;
      const uint grid_resY = grid->grids[i].resY;
    
      for (int start_y=0;start_y+InitialSubGridRes<grid_resY;start_y+=InitialSubGridRes)
        for (int start_x=0;start_x+InitialSubGridRes<grid_resX;start_x+=InitialSubGridRes)
        {
          LCGQuadNode *current = &global_grid.quadTrees[index*NUM_TOTAL_QUAD_NODES_PER_RTC_LCG];
          uint local_index = 1;
          createQuadNode(current[0],current,local_index,start_x,start_y,(1<<(LOD_LEVELS-1)),vtx,grid_resX,grid_resY,FULL_BORDER);          
          if (local_index != NUM_TOTAL_QUAD_NODES_PER_RTC_LCG)
          {
            PRINT2(local_index,NUM_TOTAL_QUAD_NODES_PER_RTC_LCG);
            FATAL("NUM_TOTAL_QUAD_NODES_PER_RTC_LCG");
          }
          index++;
        }
    }
    if (index > numSubGrids)
      FATAL("numSubGrids");    
  
    global_grid.geometry = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_LOSSY_COMPRESSED_GEOMETRY);
    rtcCommitGeometry(global_grid.geometry);
    global_grid.geomID = rtcAttachGeometry(scene,global_grid.geometry);
    //rtcReleaseGeometry(geom);
  }

  
  extern "C" ISPCScene* g_ispc_scene;

/* called by the C++ code for initialization */
  extern "C" void device_init (char* cfg)
  {
    TutorialData_Constructor(&data);
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
        convertISPCGridMesh((ISPCGridMesh*)geometry,data.g_scene);
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

#define LINE_THRESHOLD 0.1f
    Vec3f color(1.0f,1.0f,1.0f);
#if 1    
    if (ray.u <= LINE_THRESHOLD ||
        ray.v <= LINE_THRESHOLD ||
        ray.u + ray.v <= LINE_THRESHOLD)
      color = Vec3fa(1.0f,0.0f,0.0f);
#else
    color = randomColor(ray.primID);
#endif
    
    /* shade pixels */  
    if (ray.geomID == RTC_INVALID_GEOMETRY_ID)
      return Vec3fa(0.0f);
    else
      //return Vec3fa(color * abs(dot(ray.dir,normalize(ray.Ng))));
      return Vec3fa(abs(dot(ray.dir,normalize(ray.Ng)))) * color;
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
    uint numActiveQuads = 0;
    
    global_grid.numCrackFixQuadNodes = 0;
    
    for (uint i=0;i<global_grid.numQuadTrees;i++)
    {
      LCGQuadNode *root = &global_grid.quadTrees[i*NUM_TOTAL_QUAD_NODES_PER_RTC_LCG];
      LODEdgeLevel lod_edge_levels = getLODEdgeLevels(global_grid.quadTrees[i*NUM_TOTAL_QUAD_NODES_PER_RTC_LCG],camera,width,height);
      
      const int lod_level = lod_edge_levels.level();
      const uint crackFixingBorderMask = getCrackFixingBorderMask(*root,lod_edge_levels,lod_level);
      //PRINT3(i,lod_level,crackFixingBorderMask);

      const int numSubGrids = 1<<(2*lod_level);
      const uint offset = (1-(1<<(2*lod_level)))/(1-4);
      
      if (crackFixingBorderMask)
      {
        for (uint j=0;j<numSubGrids;j++)
        {
          LCGQuadNode *subgrid_root = &root[offset+j];
          const uint subgrid_crackFixingBorderMask = subgrid_root->flags & crackFixingBorderMask;
          if (subgrid_crackFixingBorderMask)
          {            
            subgrid_root = &global_grid.crackFixQuadNodes[global_grid.numCrackFixQuadNodes++];
            *subgrid_root = root[offset+j];
            fixCracks(*subgrid_root, subgrid_crackFixingBorderMask, lod_edge_levels, lod_level);
          }
          global_grid.lcg_ptrs[numActiveQuads+j] = &subgrid_root->grid;
        }          
      }
      else        
        for (uint j=0;j<numSubGrids;j++)
          global_grid.lcg_ptrs[numActiveQuads+j] = &root[offset+j].grid;
      numActiveQuads += numSubGrids;
    }
    PRINT2(numActiveQuads,global_grid.numCrackFixQuadNodes);
    
    rtcSetGeometryUserData(global_grid.geometry,global_grid.lcg_ptrs);
    rtcSetLossyCompressedGeometryPrimitiveCount(global_grid.geometry,numActiveQuads);
    rtcCommitGeometry(global_grid.geometry);
    
    /* commit changes to scene */
    rtcCommitScene (data.g_scene);
  }

/* called by the C++ code for cleanup */
  extern "C" void device_cleanup ()
  {
    TutorialData_Destructor(&data);
  }

} // namespace embree
