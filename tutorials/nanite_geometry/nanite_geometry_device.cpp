// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "nanite_geometry_device.h"

#if defined(USE_GLFW)

/* include GLFW for window management */
#include <GLFW/glfw3.h>

/* include ImGUI */
#include "../common/imgui/imgui.h"
#include "../common/imgui/imgui_impl_glfw_gl2.h"

#endif


namespace embree {

  template<typename Ty>
  struct Averaged
  {
    Averaged (size_t N, double dt)
      : N(N), dt(dt) {}

    void add(double v)
    {
      values.push_front(std::make_pair(getSeconds(),v));
      if (values.size() > N) values.resize(N);
    }

    Ty get() const
    {
      if (values.size() == 0) return zero;
      double t_begin = values[0].first-dt;

      Ty sum(zero);
      size_t num(0);
      for (size_t i=0; i<values.size(); i++) {
        if (values[i].first >= t_begin) {
          sum += values[i].second;
          num++;
        }
      }
      if (num == 0) return 0;
      else return sum/Ty(num);
    }

    std::deque<std::pair<double,Ty>> values;
    size_t N;
    double dt;
  };


  
  
  RTCScene g_scene  = nullptr;
  TutorialData data;

  Denoiser *denoiser = nullptr;
    
  extern "C" RenderMode user_rendering_mode = RENDER_PRIMARY;
  extern "C" unsigned int user_spp = 1;
  extern "C" unsigned int g_max_path_length = 2;
  extern "C" unsigned int g_lod_threshold = 30;
  extern "C" char* camera_file = nullptr;
  extern "C" unsigned int camera_mode = 0;
  
  Averaged<double> avg_bvh_build_time(64,1.0);
  Averaged<double> avg_lod_selection_time(64,1.0);
  Averaged<double> avg_denoising_time(64,1.0);

  extern "C" unsigned int frameIndex = 0;
  std::vector<ISPCCamera> camera_path;

#if defined(EMBREE_SYCL_TUTORIAL) && defined(USE_SPECIALIZATION_CONSTANTS)
  const static sycl::specialization_id<RTCFeatureFlags> rtc_feature_mask(RTC_FEATURE_FLAG_ALL);
#endif
  
  RTCFeatureFlags g_used_features = RTC_FEATURE_FLAG_NONE;
  

  __forceinline Vec3fa getTexel3f(const Texture* texture, float s, float t)
  {
    int iu = (int)floorf(s * (float)(texture->width-1));
    int iv = (int)floorf(t * (float)(texture->height-1));    
    const int offset = (iv * texture->width + iu) * 4;
    unsigned char * txt = (unsigned char*)texture->data;
    const unsigned char  r = txt[offset+0];
    const unsigned char  g = txt[offset+1];
    const unsigned char  b = txt[offset+2];
    return Vec3fa(  (float)r * 1.0f/255.0f, (float)g * 1.0f/255.0f, (float)b * 1.0f/255.0f );
  }
  

  // =========================================================================================================================================================
  // =========================================================================================================================================================
  // =========================================================================================================================================================

  
  inline Vec3fa getVertex(const unsigned int x, const unsigned int y, const Vec3fa *const vtx, const unsigned int grid_resX, const unsigned int grid_resY)
  {
    const unsigned int px = min(x,grid_resX-1);
    const unsigned int py = min(y,grid_resY-1);    
    return vtx[py*grid_resX + px];
  }
  
  // ==============================================================================================
  // ==============================================================================================
  // ==============================================================================================
  
  
  LCG_Scene::LCG_Scene(const unsigned int maxNumLCGBP)
  {
    bounds = BBox3f(empty);
    minLODDistance = 1.0f;
    /* --- lossy compressed bilinear patches --- */
    numLCGBP = 0;
    numCurrentLCGBPStates = 0;    
    numAllocatedLCGBP = maxNumLCGBP; 
    numAllocatedLCGBPStates = (1<<(2*(LCG_Scene::LOD_LEVELS-1))) * maxNumLCGBP;
    lcgbp = nullptr;
    lcgbp_state = nullptr;

    /* --- lossy compressed meshes --- */
    numLCQuadsTotal = 0;
    numLCBlocksTotal = 0;    
    numLCMeshClustersMaxRes = 0;
    numLCMeshClusters = 0;
    numLCMeshClusterRoots = 0;
    numLCMeshClusterRootsPerFrame = 0;
    numLCMeshClusterQuadsPerFrame = 0;
    numLCMeshClusterBlocksPerFrame = 0;
    
    lcm_cluster = nullptr;
    lcm_cluster_roots_IDs = nullptr;
    lcm_cluster_roots_IDs_per_frame = nullptr;
    lcm_cluster_active_state_per_frame = nullptr;
    
    pick_geomID = -1;
    pick_primID = -1;
    pick_pos = Vec3fa(0,0,0);
    
    if (maxNumLCGBP)
    {
      lcgbp       = (LCGBP*)alignedUSMMalloc(sizeof(LCGBP)*numAllocatedLCGBP,64,EMBREE_USM_SHARED /*EmbreeUSMMode::EMBREE_DEVICE_READ_WRITE*/);
      lcgbp_state = (LCGBP_State*)alignedUSMMalloc(sizeof(LCGBP_State)*numAllocatedLCGBPStates,64,EMBREE_USM_SHARED/*EmbreeUSMMode::EMBREE_DEVICE_READ_WRITE*/);
      PRINT2(numAllocatedLCGBP,numAllocatedLCGBP*sizeof(LCGBP));
      PRINT2(numAllocatedLCGBPStates,numAllocatedLCGBPStates*sizeof(LCGBP_State));
    }
  }

  void LCG_Scene::addGrid(const unsigned int gridResX, const unsigned int gridResY, const Vec3fa *const vtx)
  {
    double avg_error = 0.0;
    double max_error = 0.0;
    unsigned int num_error = 0;

    PRINT(gridResX);
    PRINT(gridResY);

    const unsigned int lcg_resX = ((gridResX-1) / LCGBP::GRID_RES_QUAD);
    const unsigned int lcg_resY = ((gridResY-1) / LCGBP::GRID_RES_QUAD);

    BBox3f gridBounds(empty);
    
    for (int start_y=0;start_y+LCGBP::GRID_RES_QUAD<gridResY;start_y+=LCGBP::GRID_RES_QUAD)
      for (int start_x=0;start_x+LCGBP::GRID_RES_QUAD<gridResX;start_x+=LCGBP::GRID_RES_QUAD)
      {
        LCGBP &current = lcgbp[numLCGBP];

        const Vec3f v0 = getVertex(start_x,start_y,vtx,gridResX,gridResY);
        const Vec3f v1 = getVertex(start_x+LCGBP::GRID_RES_QUAD,start_y,vtx,gridResX,gridResY);
        const Vec3f v2 = getVertex(start_x+LCGBP::GRID_RES_QUAD,start_y+LCGBP::GRID_RES_QUAD,vtx,gridResX,gridResY);
        const Vec3f v3 = getVertex(start_x,start_y+LCGBP::GRID_RES_QUAD,vtx,gridResX,gridResY);

        const Vec2f u_range((float)start_x/(gridResX-1),(float)(start_x+LCGBP::GRID_RES_QUAD)/(gridResX-1));
        const Vec2f v_range((float)start_y/(gridResY-1),(float)(start_y+LCGBP::GRID_RES_QUAD)/(gridResY-1));

        const unsigned int current_x = start_x / LCGBP::GRID_RES_QUAD;
        const unsigned int current_y = start_y / LCGBP::GRID_RES_QUAD;        
        
        const int neighbor_top    = current_y>0          ? numLCGBP-lcg_resX : -1;
        const int neighbor_right  = current_x<lcg_resX-1 ? numLCGBP+1        : -1;
        const int neighbor_bottom = current_y<lcg_resY-1 ? numLCGBP+lcg_resX : -1;
        const int neighbor_left   = current_x>0          ? numLCGBP-1        : -1;                

        //PRINT4(neighbor_top,neighbor_right,neighbor_bottom,neighbor_left);
        
        new (&current) LCGBP(v0,v1,v2,v3,numLCGBP++,u_range,v_range,neighbor_top,neighbor_right,neighbor_bottom,neighbor_left);
        
        current.encode(start_x,start_y,vtx,gridResX,gridResY);
        
        for (int y=0;y<LCGBP::GRID_RES_VERTEX;y++)
        {
          for (int x=0;x<LCGBP::GRID_RES_VERTEX;x++)
          {
            const Vec3f org_v  = getVertex(start_x+x,start_y+y,vtx,gridResX,gridResY);
            const Vec3f new_v  = current.decode(x,y);
            gridBounds.extend(new_v);
            
            const float error = length(new_v-org_v);
            if (error > 0.1)
            {
              PRINT5(x,y,LCGBP::as_uint(new_v.x),LCGBP::as_uint(new_v.y),LCGBP::as_uint(new_v.z));              
              //exit(0);
            }
            avg_error += (double)error;
            max_error = max(max_error,(double)error);
            num_error++;
          }
        }
      }
    PRINT2((float)(avg_error / num_error),max_error);
    bounds.extend(gridBounds);
    minLODDistance = length(bounds.size()) / RELATIVE_MIN_LOD_DISTANCE_FACTOR;
    PRINT( minLODDistance );
  }

  LCG_Scene *global_lcgbp_scene = nullptr;

  // ==============================================================================================
  // ==============================================================================================
  // ==============================================================================================


  Vec2i convertISPCQuadMesh(ISPCQuadMesh* mesh, RTCScene scene, ISPCOBJMaterial *material,const unsigned int geomID,std::vector<LossyCompressedMesh*> &lcm_ptrs,std::vector<LossyCompressedMeshCluster> &lcm_clusters, std::vector<unsigned int> &lcm_clusterRootIDs, size_t &totalCompressedSize, size_t &numDecompressedBlocks, sycl::queue &queue);
  
  void convertISPCGridMesh(ISPCGridMesh* grid, RTCScene scene, ISPCOBJMaterial *material)
  {
    unsigned int numLCGBP = 0;
    
    /* --- count lcgbp --- */
    for (unsigned int i=0;i<grid->numGrids;i++)
    {
      PRINT3(i,grid->grids[i].resX,grid->grids[i].resY);      
      const unsigned int grid_resX = grid->grids[i].resX;
      const unsigned int grid_resY = grid->grids[i].resY;
      const unsigned int numInitialSubGrids = ((grid_resX-1) / LCGBP::GRID_RES_QUAD) * ((grid_resY-1) / LCGBP::GRID_RES_QUAD);
      //PRINT(numInitialSubGrids);
      numLCGBP  += numInitialSubGrids;
    }
    PRINT(numLCGBP);

    /* --- allocate global LCGBP --- */
    global_lcgbp_scene = (LCG_Scene*)alignedUSMMalloc(sizeof(LCG_Scene),64,EMBREE_USM_SHARED);
    new (global_lcgbp_scene) LCG_Scene(numLCGBP);
    
    /* --- fill array of LCGBP --- */
    for (unsigned int i=0;i<grid->numGrids;i++)
      global_lcgbp_scene->addGrid(grid->grids[i].resX,grid->grids[i].resY,grid->positions[0]);    

    global_lcgbp_scene->geometry = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_LOSSY_COMPRESSED_GEOMETRY);
    rtcCommitGeometry(global_lcgbp_scene->geometry);
    global_lcgbp_scene->geomID = rtcAttachGeometry(scene,global_lcgbp_scene->geometry);
    //rtcReleaseGeometry(geom);
    global_lcgbp_scene->map_Kd = (Texture*)material->map_Kd;        
  }

  inline Vec3fa generateVertex(const int x, const int y, const int gridResX, const int gridResY,const Texture* texture)
  {
    const float scale = 1000.0f;
    const int px = min(x,gridResX-1);
    const int py = min(y,gridResY-1);
    const float u = min((float)px / (gridResX-1),0.99f);
    const float v = min((float)py / (gridResY-1),0.99f);
    Vec3f vtx = Vec3fa(px-gridResX/2,py-gridResY/2,0);
    const Vec3f d = getTexel3f(texture,u,v);
    vtx.z += d.z*scale;
    return vtx;
    //return vtx + d*scale;
  }


  

  void generateGrid(RTCScene scene, const unsigned int gridResX, const unsigned int gridResY)
  {
    const unsigned int numLCGBP = ((gridResX-1) / LCGBP::GRID_RES_QUAD) * ((gridResY-1) / LCGBP::GRID_RES_QUAD);

    /* --- allocate global LCGBP --- */
    global_lcgbp_scene = (LCG_Scene*)alignedUSMMalloc(sizeof(LCG_Scene),64,EMBREE_USM_SHARED);
    new (global_lcgbp_scene) LCG_Scene(numLCGBP);

    const unsigned int vertices = gridResX*gridResY;
    Vec3fa *vtx = (Vec3fa*)malloc(sizeof(Vec3fa)*vertices);

    const FileName fileNameDisplacement("Rock_Mossy_02_height.png");
    Texture *displacement = new Texture(loadImage(fileNameDisplacement),fileNameDisplacement);
    PRINT2(displacement->width,displacement->height);
    
    for (unsigned int y=0;y<gridResY;y++)
      for (unsigned int x=0;x<gridResX;x++)
        vtx[y*gridResX+x] = generateVertex(x,y,gridResX,gridResY,displacement);
    
    global_lcgbp_scene->addGrid(gridResX,gridResY,vtx);

    free(vtx);
    
    global_lcgbp_scene->geometry = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_LOSSY_COMPRESSED_GEOMETRY);
    rtcCommitGeometry(global_lcgbp_scene->geometry);
    global_lcgbp_scene->geomID = rtcAttachGeometry(scene,global_lcgbp_scene->geometry);
    //rtcReleaseGeometry(geom);

    const FileName fileNameDiffuse("Rock_Mossy_02_diffuseOriginal.png");
    Texture *diffuse = new Texture(loadImage(fileNameDiffuse),fileNameDiffuse);
    PRINT2(diffuse->width,diffuse->height);
    
    global_lcgbp_scene->map_Kd = diffuse;                
  }


  
  extern "C" ISPCScene* g_ispc_scene;

/* called by the C++ code for initialization */
  extern "C" void device_init (char* cfg)
  {
    if (camera_mode == 2 && camera_file)
    {
      std::ifstream input(camera_file,std::ios::in|std::ios::binary);
      if (!input) FATAL("cannot open camera file");

      input.seekg (0, std::ios::end);
      size_t length = input.tellg() / sizeof(ISPCCamera);
      PRINT(length);
      input.seekg (0, std::ios::beg);
      camera_path.resize(length);
      for (uint i=0;i<camera_path.size();i++)
        input.read((char*)&camera_path[i],sizeof(ISPCCamera));
      input.close();
      PRINT(camera_path.size());      
    }
    
    TutorialData_Constructor(&data);
    /* create scene */
    data.g_scene = g_scene = rtcNewScene(g_device);
    rtcSetSceneBuildQuality(data.g_scene,RTC_BUILD_QUALITY_LOW);
    rtcSetSceneFlags(data.g_scene,RTC_SCENE_FLAG_DYNAMIC);

    
#if 1
    PRINT(g_ispc_scene->numGeometries);
    PRINT(g_ispc_scene->numMaterials);

    unsigned int numGridMeshes = 0;
    unsigned int numQuadMeshes = 0;
    unsigned int numQuads      = 0;
    BBox3f scene_bounds(empty);
    for (unsigned int geomID=0; geomID<g_ispc_scene->numGeometries; geomID++)
    {
      ISPCGeometry* geometry = g_ispc_scene->geometries[geomID];
      if (geometry->type == GRID_MESH) numGridMeshes++;
      else if (geometry->type == QUAD_MESH) {
        ISPCQuadMesh *mesh = (ISPCQuadMesh*)geometry;
        numQuadMeshes++;
        numQuads+= mesh->numQuads;
        for (unsigned int i=0;i<mesh->numVertices;i++)
          scene_bounds.extend(mesh->positions[0][i]);
      }
    }
    
    PRINT3(scene_bounds,numQuadMeshes,numGridMeshes);
    
    global_lcgbp_scene = (LCG_Scene*)alignedUSMMalloc(sizeof(LCG_Scene),64,EMBREE_USM_SHARED);
    new (global_lcgbp_scene) LCG_Scene(0);
    
    global_lcgbp_scene->bounds = scene_bounds;
    
    std::vector<LossyCompressedMesh*> lcm_ptrs;
    std::vector<LossyCompressedMeshCluster> lcm_clusters;
    std::vector<unsigned int> lcm_clusterRootIDs;
    
    size_t totalCompressedSize = 0;
    size_t numDecompressedBlocks = 0;

    double total0 = getSeconds();    
    for (unsigned int geomID=0; geomID<g_ispc_scene->numGeometries; geomID++)
    {
      ISPCGeometry* geometry = g_ispc_scene->geometries[geomID];
      if (geometry->type == GRID_MESH)
        convertISPCGridMesh((ISPCGridMesh*)geometry,data.g_scene, (ISPCOBJMaterial*)g_ispc_scene->materials[geomID]);
      else if (geometry->type == QUAD_MESH)
      {
        std::cout << "Processing mesh " << geomID << " of " << g_ispc_scene->numGeometries << " meshes in " << std::flush;
        double t0 = getSeconds();
        Vec2i stats = convertISPCQuadMesh((ISPCQuadMesh*)geometry,data.g_scene, (ISPCOBJMaterial*)g_ispc_scene->materials[geomID],geomID,lcm_ptrs,lcm_clusters,lcm_clusterRootIDs,totalCompressedSize,numDecompressedBlocks,*global_gpu_queue);
        const unsigned int numNumClustersMaxRes = stats.x;
        const unsigned int numNumClustersMaxBlocks = stats.y;
                
        double t1= getSeconds();
        std::cout << (t1-t0) << " seconds" << std::endl << std::flush;
        global_lcgbp_scene->numLCQuadsTotal  += ((ISPCQuadMesh*)geometry)->numQuads;
        global_lcgbp_scene->numLCBlocksTotal += numNumClustersMaxBlocks;
        global_lcgbp_scene->numLCMeshClustersMaxRes += numNumClustersMaxRes;
        // free ISPC geometry memory
        delete (ISPCQuadMesh*)geometry;
        g_ispc_scene->geometries[geomID] = nullptr;
      }
    }
    double total1 = getSeconds();
    std::cout << "Total clusters generation time " << total1-total0 << " seconds" << std::endl;
        
    // === finalize quad meshes ===
    if (numQuadMeshes)
    {
      // ========== make bounds of DAG2 neighbor nodes similar to cause same decision in lod selection ==============

      for (uint i=0;i<lcm_clusters.size();i++)
        if (lcm_clusters[i].hasNeighbor())
          lcm_clusters[i].bounds.extend( lcm_clusters[lcm_clusters[i].neighborID].bounds );

      // ==========
      
      global_lcgbp_scene->minLODDistance = length(scene_bounds.size()) / RELATIVE_MIN_LOD_DISTANCE_FACTOR;
      
      global_lcgbp_scene->numLCMeshClusters = lcm_clusters.size();
      global_lcgbp_scene->numLCMeshClusterRoots = lcm_clusterRootIDs.size();

      
#if ALLOC_DEVICE_MEMORY == 1
      EmbreeUSMMode mode = EmbreeUSMMode::EMBREE_DEVICE_READ_WRITE;
#else        
      EmbreeUSMMode mode = EmbreeUSMMode::EMBREE_USM_SHARED;
#endif      
      
      global_lcgbp_scene->lcm_cluster = (LossyCompressedMeshCluster*)alignedUSMMalloc(sizeof(LossyCompressedMeshCluster)*global_lcgbp_scene->numLCMeshClusters,64,mode);
      global_lcgbp_scene->lcm_cluster_roots_IDs = (unsigned int*)alignedUSMMalloc(sizeof(unsigned int)*global_lcgbp_scene->numLCMeshClusterRoots,64,mode);
      global_lcgbp_scene->lcm_cluster_roots_IDs_per_frame = (unsigned int*)alignedUSMMalloc(sizeof(unsigned int)*global_lcgbp_scene->numLCMeshClusters,64,mode);
      global_lcgbp_scene->lcm_cluster_active_state_per_frame = (uchar*)alignedUSMMalloc(sizeof(uchar)*global_lcgbp_scene->numLCMeshClusters,64,mode);

      global_gpu_queue->memcpy(global_lcgbp_scene->lcm_cluster,&*lcm_clusters.begin(),sizeof(LossyCompressedMeshCluster)*global_lcgbp_scene->numLCMeshClusters);
      global_gpu_queue->memcpy(global_lcgbp_scene->lcm_cluster_roots_IDs,&*lcm_clusterRootIDs.begin(),sizeof(unsigned int)*global_lcgbp_scene->numLCMeshClusterRoots);

      gpu::waitOnQueueAndCatchException(*global_gpu_queue);        
      
      //for (unsigned int i=0;i<global_lcgbp_scene->numLCMeshClusters;i++)
      //  global_lcgbp_scene->lcm_cluster[i] = lcm_clusters[i];

      unsigned int numLODQuads = 0;
      for (unsigned int i=0;i<global_lcgbp_scene->numLCMeshClusterRoots;i++)
        numLODQuads += lcm_clusters[ lcm_clusterRootIDs[i] ].numQuads;

      unsigned int numTotalLODQuads = 0;
      for (unsigned int i=0;i<global_lcgbp_scene->numLCMeshClusters;i++)
        numTotalLODQuads += lcm_clusters[ i ].numQuads;
      
      
      global_lcgbp_scene->geometry = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_LOSSY_COMPRESSED_GEOMETRY);
      rtcCommitGeometry(global_lcgbp_scene->geometry);
      global_lcgbp_scene->geomID = rtcAttachGeometry(data.g_scene,global_lcgbp_scene->geometry);
      //rtcReleaseGeometry(geom);
      global_lcgbp_scene->map_Kd = nullptr;

      PRINT( global_lcgbp_scene->numLCMeshClusterRoots );      
      PRINT(global_lcgbp_scene->numLCMeshClusters);
      PRINT4(numQuadMeshes,numQuads,numLODQuads,numTotalLODQuads);
      PRINT3(totalCompressedSize,(float)totalCompressedSize/numTotalLODQuads,(float)totalCompressedSize/numTotalLODQuads*0.5f);
      PRINT3(numDecompressedBlocks,numDecompressedBlocks*64,(float)numDecompressedBlocks*64/totalCompressedSize);
    }
    
#else
    const unsigned int gridResX = 16*1024;
    const unsigned int gridResY = 16*1024;    
    generateGrid(data.g_scene,gridResX,gridResY);
#endif    
    /* update scene */
    //rtcCommitScene (data.g_scene);  
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
  Vec3fa renderPixelPrimary(const TutorialData& data, float x, float y, const ISPCCamera& camera, const unsigned int width, const unsigned int height, LCG_Scene *grid)
  {
    RTCIntersectArguments args;
    rtcInitIntersectArguments(&args);
    args.feature_mask = (RTCFeatureFlags) (FEATURE_MASK);
  
    /* initialize ray */
    Ray ray(Vec3fa(camera.xfm.p), Vec3fa(normalize(x*camera.xfm.l.vx + y*camera.xfm.l.vy + camera.xfm.l.vz)), 0.0f, inf);

    /* intersect ray with scene */
    rtcIntersect1(data.g_scene,RTCRayHit_(ray),&args);

    Vec3f color(1.0f,1.0f,1.0f);    
    if (ray.geomID != RTC_INVALID_GEOMETRY_ID)
      color = Vec3fa( abs(dot(ray.dir,normalize(ray.Ng))) );
    return color;
  }

  Vec3fa renderPixelDebug(const TutorialData& data, float x, float y, const ISPCCamera& camera, const unsigned int width, const unsigned int height, LCG_Scene *lcgbp_scene, const RenderMode mode)
  {
    RTCIntersectArguments args;
    rtcInitIntersectArguments(&args);
    args.feature_mask = (RTCFeatureFlags) (FEATURE_MASK);
  
    /* initialize ray */
    Ray ray(Vec3fa(camera.xfm.p), Vec3fa(normalize(x*camera.xfm.l.vx + y*camera.xfm.l.vy + camera.xfm.l.vz)), 0.0f, inf);

    /* intersect ray with scene */
    rtcIntersect1(data.g_scene,RTCRayHit_(ray),&args);

    if (ray.geomID == RTC_INVALID_GEOMETRY_ID) return Vec3fa(1.0f,1.0f,1.0f);

    const unsigned int localID = ray.primID & (((unsigned int)1<<RTC_LOSSY_COMPRESSED_GRID_LOCAL_ID_SHIFT)-1);
    const unsigned int primID = ray.primID >> RTC_LOSSY_COMPRESSED_GRID_LOCAL_ID_SHIFT;
    
    Vec3f color(1.0f,1.0f,1.0f);
    
    if (mode == RENDER_DEBUG_QUADS)
    {
      const float LINE_THRESHOLD = 0.1f;
      if (ray.u <= LINE_THRESHOLD ||
          ray.v <= LINE_THRESHOLD ||
          (1-ray.u) <= LINE_THRESHOLD ||
          (1-ray.v) <= LINE_THRESHOLD)
        color = Vec3fa(1.0f,0.0f,0.0f);      
    }
    else if (mode == RENDER_DEBUG_SUBGRIDS)
    {
      const unsigned int gridID = lcgbp_scene->lcgbp_state[primID].lcgbp->ID;
      const unsigned int subgridID = lcgbp_scene->lcgbp_state[primID].localID;    
      color = randomColor(gridID*(16+4+1)+subgridID);   
    }    
    else if (mode == RENDER_DEBUG_GRIDS)
    {
      const unsigned int gridID = lcgbp_scene->lcgbp_state[primID].lcgbp->ID;      
      color = randomColor(gridID);   
    }
    else if (mode == RENDER_DEBUG_LOD)
    {
      const unsigned int step = lcgbp_scene->lcgbp_state[primID].step; 
      if (step == 4)
        color = Vec3fa(0,0,1);
      else if (step == 2)
        color = Vec3fa(0,1,0);
      else if (step == 1)
        color = Vec3fa(1,0,0);            
    }
    else if (mode == RENDER_DEBUG_CRACK_FIXING)
    {
      const unsigned int cracks_fixed = lcgbp_scene->lcgbp_state[primID].cracksFixed();
      if (cracks_fixed)
        color = Vec3fa(1,0,1);      
    }
    else if (mode == RENDER_DEBUG_CLOD)
    {
      const unsigned int step = lcgbp_scene->lcgbp_state[primID].step; 
      if (step == 4)
        color = Vec3fa(0,0,1);
      else if (step == 2)
        color = Vec3fa(0,1,0);
      else if (step == 1)
        color = Vec3fa(1,0,0);                  
      const unsigned int blend = (unsigned int)lcgbp_scene->lcgbp_state[primID].blend;
      if (blend)
        color = Vec3fa(1,1,0);      
    }    
    else if (mode == RENDER_DEBUG_TEXTURE)
    {
      const unsigned int flip_uv = localID & 1;
      const unsigned int localQuadID = localID>>1;
      const unsigned int local_y = localQuadID /  RTC_LOSSY_COMPRESSED_GRID_QUAD_RES;
      const unsigned int local_x = localQuadID %  RTC_LOSSY_COMPRESSED_GRID_QUAD_RES;

      const LCGBP_State &state = lcgbp_scene->lcgbp_state[primID];
      const LCGBP &current = *state.lcgbp;
      const unsigned int start_x = state.start_x;
      const unsigned int start_y = state.start_y;
      const unsigned int end_x = state.start_x + state.step*8;
      const unsigned int end_y = state.start_y + state.step*8;

      const float blend_start_u = (float)start_x / LCGBP::GRID_RES_QUAD;
      const float blend_end_u   = (float)  end_x / LCGBP::GRID_RES_QUAD;
      const float blend_start_v = (float)start_y / LCGBP::GRID_RES_QUAD;
      const float blend_end_v   = (float)  end_y / LCGBP::GRID_RES_QUAD;

      const Vec2f u_range(lerp(current.u_range.x,current.u_range.y,blend_start_u),lerp(current.u_range.x,current.u_range.y,blend_end_u));
      const Vec2f v_range(lerp(current.v_range.x,current.v_range.y,blend_start_v),lerp(current.v_range.x,current.v_range.y,blend_end_v));
      
      const float u = flip_uv ? 1-ray.u : ray.u;
      const float v = flip_uv ? 1-ray.v : ray.v;
      const float u_size = (u_range.y - u_range.x) * (1.0f / RTC_LOSSY_COMPRESSED_GRID_QUAD_RES);
      const float v_size = (v_range.y - v_range.x) * (1.0f / RTC_LOSSY_COMPRESSED_GRID_QUAD_RES);
      const float u_start = u_range.x + u_size * (float)local_x;
      const float v_start = v_range.x + v_size * (float)local_y;      
      const float fu = u_start + u * u_size;
      const float fv = v_start + v * v_size;

      color = getTexel3f(lcgbp_scene->map_Kd,1-fu,fv);
      //color = Vec3fa(fu,fv,1.0f-fu-fv);
    }
    else if (mode == RENDER_DEBUG_CLUSTER_ID)
    {
      color =  randomColor(ray.primID);    
    }
    else if (mode == RENDER_DEBUG_LOD_LEVEL)
    {
      const LossyCompressedMeshCluster &cluster = lcgbp_scene->lcm_cluster[ ray.primID ];      
      color =  Vec3fa(1.0f / (cluster.lod_level),0,0);    
    }
    
    
    return Vec3fa(abs(dot(ray.dir,normalize(ray.Ng)))) * color;
  }
  

  void renderPixelStandard(const TutorialData& data,
                           int x, int y, 
                           int* pixels,
                           const unsigned int width,
                           const unsigned int height,
                           const float time,
                           const ISPCCamera& camera,
                           LCG_Scene *lcgbp_scene,
                           const RenderMode mode,
                           const unsigned int spp)
  {
    RandomSampler sampler;

    Vec3fa color(0.0f);
    const float inv_spp = 1.0f / (float)spp;
    
    for (unsigned int i=0;i<spp;i++)
    {
      float fx = x; 
      float fy = y; 
      if (i >= 1)
      {
        RandomSampler_init(sampler, 0, 0, i);
        fx += RandomSampler_get1D(sampler);
        fy += RandomSampler_get1D(sampler);
      }
#if 1      
      RenderMode new_mode = RENDER_PRIMARY;
      if (x > width/2) new_mode = mode;
#endif
      
      /* calculate pixel color */
      if (new_mode == RENDER_PRIMARY)
        color += renderPixelPrimary(data, (float)fx,(float)fy,camera, width, height, lcgbp_scene);
      else
        color += renderPixelDebug(data, (float)fx,(float)fy,camera, width, height, lcgbp_scene, new_mode);
    }
    color *= inv_spp;
    
    /* write color to framebuffer */
    unsigned int r = (unsigned int) (255.0f * clamp(color.x,0.0f,1.0f));
    unsigned int g = (unsigned int) (255.0f * clamp(color.y,0.0f,1.0f));
    unsigned int b = (unsigned int) (255.0f * clamp(color.z,0.0f,1.0f));
    pixels[y*width+x] = (b << 16) + (g << 8) + r;
  }

  


  template<typename T>
  static __forceinline unsigned int atomic_add_global(T *dest, const T count=1)
  {
    sycl::atomic_ref<T, sycl::memory_order::relaxed, sycl::memory_scope::device,sycl::access::address_space::global_space> counter(*dest);        
    return counter.fetch_add(count);      
  }


  extern "C" void device_gui()
  {
    const unsigned int numTrianglesPerGrid9x9 = 8*8*2;
    const unsigned int numTrianglesPerGrid33x33 = 32*32*2;
    //ImGui::Text("BVH Build Time: %4.4f ms",avg_bvh_build_time.get());
    ImGui::Text("SPP:                %d",user_spp);        
    ImGui::Text("Per Frame Overhead: %4.4f ms",avg_bvh_build_time.get());
    ImGui::Text("LOD Selection Time: %4.4f ms",avg_lod_selection_time.get());
    ImGui::DragInt("LOD Threshold",(int*)&g_lod_threshold,1,2,1000);
    
    RenderMode rendering_mode = user_rendering_mode;
    if (rendering_mode == RENDER_PATH_TRACER_DENOISE)
      ImGui::Text("Denoising Time: %4.4f ms",avg_denoising_time.get());

    
    if (global_lcgbp_scene->numLCGBP)
    {
      ImGui::Text("numGrids9x9:   %d (out of %d)",global_lcgbp_scene->numCurrentLCGBPStates,global_lcgbp_scene->numLCGBP*(1<<(LCG_Scene::LOD_LEVELS+1)));
      ImGui::Text("numGrids33x33: %d ",global_lcgbp_scene->numLCGBP);
      ImGui::Text("numTriangles:  %d (out of %d)",global_lcgbp_scene->numCurrentLCGBPStates*numTrianglesPerGrid9x9,global_lcgbp_scene->numLCGBP*numTrianglesPerGrid33x33);
      ImGui::Text("Blocks / Frame:          %d (out of %d)",global_lcgbp_scene->numLCMeshClusterBlocksPerFrame,global_lcgbp_scene->numLCBlocksTotal);                  
    }
    if (global_lcgbp_scene->numLCMeshClusters)
    {
      ImGui::Text("Root Clusters:                 %d            ",global_lcgbp_scene->numLCMeshClusterRoots);      
      ImGui::Text("Active Clusters / Frame:       %d (out of %d)",global_lcgbp_scene->numLCMeshClusterRootsPerFrame,global_lcgbp_scene->numLCMeshClustersMaxRes);
      ImGui::Text("Quads / Frame:                 %d (out of %d)",global_lcgbp_scene->numLCMeshClusterQuadsPerFrame,global_lcgbp_scene->numLCQuadsTotal);
      ImGui::Text("Triangles / Frame:             %d (out of %d)",global_lcgbp_scene->numLCMeshClusterQuadsPerFrame*2,global_lcgbp_scene->numLCQuadsTotal*2);      
      ImGui::Text("64-bytes QBVH6 Blocks / Frame: %d (out of %d)",global_lcgbp_scene->numLCMeshClusterBlocksPerFrame,global_lcgbp_scene->numLCBlocksTotal);            
    }

  }
    
  
/* called by the C++ code to render */
  extern "C" void device_render (int* pixels,
                                 const unsigned int width,
                                 const unsigned int height,
                                 const float time,
                                 const ISPCCamera& camera)
  {
    if (!denoiser)
      denoiser = new Denoiser(width,height);

    double t0_lod = getSeconds();
    
#if defined(EMBREE_SYCL_TUTORIAL)    
    
    LCG_Scene *local_lcgbp_scene = global_lcgbp_scene;
    if ( local_lcgbp_scene->numLCGBP )
      select_clusters_lod_grid_tree(local_lcgbp_scene,width,height,camera);
    
    if (local_lcgbp_scene->numLCMeshClusters)
    {
#if ENABLE_DAG == 1
      select_clusters_lod_mesh_dag(local_lcgbp_scene,width,height,camera);      
#else
      select_clusters_lod_mesh_tree(local_lcgbp_scene,width,height,camera);            
#endif      
    }

    waitOnQueueAndCatchException(*global_gpu_queue);  // FIXME            
    

#if 0
    uint less96 = 0;
    size_t readBytes = 0;
    for (uint i=0;i<local_lcgbp_scene->numLCMeshClusterRootsPerFrame;i++)
    {
      const uint clusterID = local_lcgbp_scene->lcm_cluster_roots_IDs_per_frame[i];
      LossyCompressedMeshCluster &cur = local_lcgbp_scene->lcm_cluster[clusterID];
      if (cur.numQuads <= 96)
        less96++;
      readBytes += sizeof(LossyCompressedMeshCluster);
      readBytes += cur.tmp*64;
    }
    PRINT3(readBytes,less96,local_lcgbp_scene->numLCMeshClusterRootsPerFrame);
#endif
    
    double dt0_lod = (getSeconds()-t0_lod)*1000.0;

    rtcSetLCData(local_lcgbp_scene->geometry, local_lcgbp_scene->numCurrentLCGBPStates, local_lcgbp_scene->lcgbp_state, local_lcgbp_scene->lcm_cluster, local_lcgbp_scene->numLCMeshClusterRootsPerFrame,local_lcgbp_scene->lcm_cluster_roots_IDs_per_frame);
    
    avg_lod_selection_time.add(dt0_lod);
    
    double t0_bvh = getSeconds();

    
    rtcCommitGeometry(local_lcgbp_scene->geometry);

    /* commit changes to scene */
    rtcCommitScene (data.g_scene);
    
    double dt0_bvh = (getSeconds()-t0_bvh)*1000.0;
                                            
    avg_bvh_build_time.add(dt0_bvh);
    
    
#endif
  }


  sycl::event renderFramePathTracer (int* pixels,
                                     const unsigned int width,
                                     const unsigned int height,
                                     const float time,
                                     const ISPCCamera& camera,
                                     TutorialData &data,
                                     unsigned int user_spp,
                                     GBuffer *gbuffer,                              
                                     bool denoise);
  


  extern "C" void renderFrameStandard (int* pixels,
                                       const unsigned int width,
                                       const unsigned int height,
                                       const float time,
                                       const ISPCCamera& _camera)
  {
    ISPCCamera camera = _camera;
    
    if (camera_mode == 1)
      camera_path.push_back(_camera);
    else if (camera_mode == 2)
      camera = camera_path[ (frameIndex++) % camera_path.size() ];    
        
    /* render all pixels */
#if defined(EMBREE_SYCL_TUTORIAL)
    RenderMode rendering_mode = user_rendering_mode;
    if (rendering_mode != RENDER_PATH_TRACER && rendering_mode != RENDER_PATH_TRACER_DENOISE)
    {
      LCG_Scene *lcgbp_scene = global_lcgbp_scene;
      unsigned int spp = user_spp;
      TutorialData ldata = data;
      sycl::event event = global_gpu_queue->submit([=](sycl::handler& cgh){
        const sycl::nd_range<2> nd_range = make_nd_range(height,width);
        cgh.parallel_for(nd_range,[=](sycl::nd_item<2> item) EMBREE_SYCL_SIMD(16) {
          const unsigned int x = item.get_global_id(1); if (x >= width ) return;
          const unsigned int y = item.get_global_id(0); if (y >= height) return;
          renderPixelStandard(ldata,x,y,pixels,width,height,time,camera,lcgbp_scene,rendering_mode,spp);
        });
      });
      waitOnEventAndCatchException(event);
      const double dt = gpu::getDeviceExecutionTiming(event);
      ((ISPCCamera*)&camera)->render_time = dt * 1E-3;        
    }
    else
    {
      GBuffer *gBuffer  = denoiser->gBuffer;      
      GBufferOutput *output  = denoiser->outputBuffer;

      bool denoise = rendering_mode == RENDER_PATH_TRACER_DENOISE;
      sycl::event event = renderFramePathTracer(pixels,width,height,time,camera,data,user_spp,gBuffer,denoise);
      waitOnEventAndCatchException(event);
      
      const double dt = gpu::getDeviceExecutionTiming(event);
      ((ISPCCamera*)&camera)->render_time = dt * 1E-3;        
      
      if (denoise)
      {
        double t0 = getSeconds();                                                    
        denoiser->execute();
        
        sycl::event event_denoising = global_gpu_queue->submit([=](sycl::handler& cgh) {
          const sycl::nd_range<2> nd_range = make_nd_range(height,width);
          cgh.parallel_for(nd_range,[=](sycl::nd_item<2> item) EMBREE_SYCL_SIMD(16) {
            const unsigned int x = item.get_global_id(1); if (x >= width ) return;
            const unsigned int y = item.get_global_id(0); if (y >= height) return;
            Vec3f c = fp_convert(output[y*width+x]);
            //Vec3f c = albedo[y*width+x];
            
            unsigned int r = (unsigned int) (255.01f * clamp(c.x,0.0f,1.0f));
            unsigned int g = (unsigned int) (255.01f * clamp(c.y,0.0f,1.0f));
            unsigned int b = (unsigned int) (255.01f * clamp(c.z,0.0f,1.0f));
            pixels[y*width+x] = (b << 16) + (g << 8) + r;                        
          });
        });
        waitOnEventAndCatchException(event_denoising);

        double dt0 = (getSeconds()-t0)*1000.0;                                            
        avg_denoising_time.add(dt0);
        
      }          
    }
#endif
  }
  

/* called by the C++ code for cleanup */
  extern "C" void device_cleanup ()
  {
    PRINT( camera_mode );
    if (camera_mode == 1 && camera_file)
    {
      PRINT(camera_path.size());
    
      std::ofstream output(camera_file,std::ios::out|std::ios::binary);
      if (!output) FATAL("cannot open camera file");
      for (uint i=0;i<camera_path.size();i++)
        output.write((char*)&camera_path[i],sizeof(ISPCCamera));
      output.close();
    }
    
    TutorialData_Destructor(&data);
  }

  

} // namespace embree
