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

#include "../../kernels/rthwif/builder/gpu/lcgbp.h"
#include "../../kernels/rthwif/builder/gpu/morton.h"


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


  
#define FEATURE_MASK                            \
  RTC_FEATURE_FLAG_TRIANGLE |                   \
  RTC_FEATURE_FLAG_INSTANCE
  
  RTCScene g_scene  = nullptr;
  TutorialData data;

  Denoiser *denoiser = nullptr;
    
  extern "C" RenderMode user_rendering_mode = RENDER_PRIMARY;
  extern "C" unsigned int user_spp = 1;
  extern "C" unsigned int g_max_path_length = 2;
  extern "C" unsigned int g_lod_threshold = 30;
  
  Averaged<double> avg_bvh_build_time(64,1.0);
  Averaged<double> avg_lod_selection_time(64,1.0);
  Averaged<double> avg_denoising_time(64,1.0);

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

  __forceinline bool frustumCullPlane(const Vec3f &lower, const Vec3f &upper, const Vec3f &normal)
  {
    const Vec3f p( normal.x <= 0.0f ? lower.x : upper.x,
                   normal.y <= 0.0f ? lower.y : upper.y,
                   normal.z <= 0.0f ? lower.z : upper.z);
    //PRINT4(normal,lower,upper,p);    
    //PRINT( dot(p,normal) );
    return dot(p,normal) < 0.0f;
                 
  }
  __forceinline bool frustumCull(const Vec3f &lower, const Vec3f &upper, const Vec3f &vx, const Vec3f &vy, const Vec3f &vz)
  {
    // FIXME plane normal;
    const Vec3f vn = cross(vx,vy);
    if (frustumCullPlane(lower,upper,vn)) return true;
    const Vec3f A = vz;
    const Vec3f B = vz + vx;
    const Vec3f C = vz + vx + vy;
    const Vec3f D = vz + vy;
    const Vec3f nAB = cross(A,B);
    const Vec3f nBC = cross(B,C);
    const Vec3f nCD = cross(C,D);
    const Vec3f nDA = cross(D,A);
    if ( frustumCullPlane(lower,upper,nAB) ||
         frustumCullPlane(lower,upper,nBC) ||
         frustumCullPlane(lower,upper,nCD) ||
         frustumCullPlane(lower,upper,nDA) ) return true;
    return false;
  }
  
  static const unsigned int LOD_LEVELS = 3;
  //static const unsigned int NUM_TOTAL_QUAD_NODES_PER_RTC_LCG = (1-(1<<(2*LOD_LEVELS)))/(1-4);

  struct LODPatchLevel
  {
    unsigned int level;
    float blend;

    __forceinline LODPatchLevel(const unsigned int level, const float blend) : level(level), blend(blend) {}
  };


  __forceinline LODPatchLevel getLODPatchLevel(const float MIN_LOD_DISTANCE,LCGBP &current,const ISPCCamera& camera, const unsigned int width, const unsigned int height)
  {
    const float minDistance = MIN_LOD_DISTANCE;
    const unsigned int startRange[LOD_LEVELS+1] = { 0,1,3,7};
    const unsigned int   endRange[LOD_LEVELS+1] = { 1,3,7,15};    
    
    const Vec3f v0 = current.patch.v0;
    const Vec3f v1 = current.patch.v1;
    const Vec3f v2 = current.patch.v2;
    const Vec3f v3 = current.patch.v3;

    const Vec3f center = lerp(lerp(v0,v1,0.5f),lerp(v2,v3,0.5f),0.5f);
    const Vec3f org = camera.xfm.p;

    const float dist = fabs(length(center-org));
    const float dist_minDistance = dist/minDistance;
    const unsigned int dist_level = floorf(dist_minDistance);

    unsigned int segment = -1;
    for (unsigned int i=0;i<LOD_LEVELS;i++)
      if (startRange[i] <= dist_level && dist_level < endRange[i])
      {          
        segment = i;
        break;
      }
    float blend = 0.0f;
    if (segment == -1)
      segment = LOD_LEVELS-1;
    else if (segment != 0)
    {
      blend = min((dist_minDistance-startRange[segment])/(endRange[segment]-startRange[segment]),1.0f);
      segment--;
    }    
    return LODPatchLevel(LOD_LEVELS-1-segment,blend);    
  }

  

  __forceinline Vec2f projectVertexToPlane(const Vec3f &p, const Vec3f &vx, const Vec3f &vy, const Vec3f &vz, const unsigned int width, const unsigned int height, const bool clip=true)
  {
    const Vec3f vn = cross(vx,vy);    
    const float distance = (float)dot(vn,vz) / (float)dot(vn,p);
    Vec3f pip = p * distance;
    if (distance < 0.0f)
      pip = vz;
    float a = dot((pip-vz),vx);
    float b = dot((pip-vz),vy);
    if (clip)
    {
      a = min(max(a,0.0f),(float)width);
      b = min(max(b,0.0f),(float)height);
    }
    return Vec2f(a,b);
  }
  
  __forceinline LODEdgeLevel getLODEdgeLevels(LCGBP &current,const ISPCCamera& camera, const unsigned int width, const unsigned int height)
  {
    const Vec3f v0 = current.patch.v0;
    const Vec3f v1 = current.patch.v1;
    const Vec3f v2 = current.patch.v2;
    const Vec3f v3 = current.patch.v3;

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
    
    i0 = min(max(0,i0),(int)LOD_LEVELS-1);
    i1 = min(max(0,i1),(int)LOD_LEVELS-1);
    i2 = min(max(0,i2),(int)LOD_LEVELS-1);
    i3 = min(max(0,i3),(int)LOD_LEVELS-1);

#if 0
    i0 = i1 = i2 = i3 = 2;
#endif    
    LODEdgeLevel lod_levels(i0,i1,i2,i3);
    return lod_levels;
  }
  
  inline Vec3fa getVertex(const unsigned int x, const unsigned int y, const Vec3fa *const vtx, const unsigned int grid_resX, const unsigned int grid_resY)
  {
    const unsigned int px = min(x,grid_resX-1);
    const unsigned int py = min(y,grid_resY-1);    
    return vtx[py*grid_resX + px];
  }
  
  // ==============================================================================================
  // ==============================================================================================
  // ==============================================================================================
  
  struct __aligned(64) LCG_Scene {
    static const unsigned int LOD_LEVELS = 3;

    /* --- general data --- */
    BBox3f bounds;
    
    /* --- lossy compressed bilinear patches --- */
    unsigned int numAllocatedLCGBP;
    unsigned int numAllocatedLCGBPStates;    
    unsigned int numLCGBP;
    unsigned int numCurrentLCGBPStates;        
    LCGBP *lcgbp;
    LCGBP_State *lcgbp_state;    
    unsigned int numCrackFixQuadNodes;

    /* --- lossy compressed meshes --- */
    unsigned int numLCQuadsTotal;
    unsigned int numLCBlocksTotal;    
    unsigned int numLCMeshClustersMaxRes;    
    unsigned int numLCMeshClusters;
    unsigned int numLCMeshClusterRoots;
    unsigned int numLCMeshClusterRootsPerFrame;
    unsigned int numLCMeshClusterQuadsPerFrame;
    unsigned int numLCMeshClusterBlocksPerFrame;
    
    //LossyCompressedMesh *lcm;
    LossyCompressedMeshCluster  *lcm_cluster;
    unsigned int *lcm_cluster_roots_IDs;
    unsigned int *lcm_cluster_roots_IDs_per_frame;
    
    /* --- embree geometry --- */
    RTCGeometry geometry;
    unsigned int geomID;

    /* --- texture handle --- */
    Texture* map_Kd;

    /* --- LOD settings --- */
    float minLODDistance;
    
    LCG_Scene(const unsigned int maxNumLCGBP);

    void addGrid(const unsigned int gridResX, const unsigned int gridResY, const Vec3fa *const vtx);
  };
  
  LCG_Scene::LCG_Scene(const unsigned int maxNumLCGBP)
  {
    bounds = BBox3f(empty);
    minLODDistance = 1.0f;
    /* --- lossy compressed bilinear patches --- */
    numLCGBP = 0;
    numCurrentLCGBPStates = 0;    
    numAllocatedLCGBP = maxNumLCGBP; 
    numAllocatedLCGBPStates = (1<<(2*(LOD_LEVELS-1))) * maxNumLCGBP;
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
      global_lcgbp_scene->minLODDistance = length(scene_bounds.size()) / RELATIVE_MIN_LOD_DISTANCE_FACTOR;
      
      global_lcgbp_scene->numLCMeshClusters = lcm_clusters.size();
      global_lcgbp_scene->numLCMeshClusterRoots = lcm_clusterRootIDs.size();

      
      EmbreeUSMMode mode = EmbreeUSMMode::EMBREE_DEVICE_READ_WRITE;
      //EmbreeUSMMode mode = EmbreeUSMMode::EMBREE_USM_SHARED;
      
      global_lcgbp_scene->lcm_cluster = (LossyCompressedMeshCluster*)alignedUSMMalloc(sizeof(LossyCompressedMeshCluster)*global_lcgbp_scene->numLCMeshClusters,64,mode);
      global_lcgbp_scene->lcm_cluster_roots_IDs = (unsigned int*)alignedUSMMalloc(sizeof(unsigned int)*global_lcgbp_scene->numLCMeshClusterRoots,64,mode);
      global_lcgbp_scene->lcm_cluster_roots_IDs_per_frame = (unsigned int*)alignedUSMMalloc(sizeof(unsigned int)*global_lcgbp_scene->numLCMeshClusters,64,mode);


      global_gpu_queue->memcpy(global_lcgbp_scene->lcm_cluster,&*lcm_clusters.begin(),sizeof(LossyCompressedMeshCluster)*global_lcgbp_scene->numLCMeshClusters);
      global_gpu_queue->memcpy(global_lcgbp_scene->lcm_cluster_roots_IDs,&*lcm_clusterRootIDs.begin(),sizeof(unsigned int)*global_lcgbp_scene->numLCMeshClusterRoots);

      gpu::waitOnQueueAndCatchException(*global_gpu_queue);        
      
      //for (unsigned int i=0;i<global_lcgbp_scene->numLCMeshClusters;i++)
      //  global_lcgbp_scene->lcm_cluster[i] = lcm_clusters[i];

      unsigned int numLODQuads = 0;
      for (unsigned int i=0;i<global_lcgbp_scene->numLCMeshClusterRoots;i++)
        numLODQuads += lcm_clusters[ lcm_clusterRootIDs[i] ].numQuads;
      
      
      global_lcgbp_scene->geometry = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_LOSSY_COMPRESSED_GEOMETRY);
      rtcCommitGeometry(global_lcgbp_scene->geometry);
      global_lcgbp_scene->geomID = rtcAttachGeometry(data.g_scene,global_lcgbp_scene->geometry);
      //rtcReleaseGeometry(geom);
      global_lcgbp_scene->map_Kd = nullptr;

      PRINT( global_lcgbp_scene->numLCMeshClusterRoots );      
      PRINT(global_lcgbp_scene->numLCMeshClusters);
      PRINT3(numQuadMeshes,numQuads,numLODQuads);
      PRINT3(totalCompressedSize,(float)totalCompressedSize/numQuads,(float)totalCompressedSize/numQuads*0.5f);
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
    if (ray.geomID == RTC_INVALID_GEOMETRY_ID)
      color = Vec3fa(0.0f);
    else
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
    
      /* calculate pixel color */
      if (mode == RENDER_PRIMARY)
        color += renderPixelPrimary(data, (float)fx,(float)fy,camera, width, height, lcgbp_scene);
      else
        color += renderPixelDebug(data, (float)fx,(float)fy,camera, width, height, lcgbp_scene, mode);
    }
    color *= inv_spp;
    
    /* write color to framebuffer */
    unsigned int r = (unsigned int) (255.0f * clamp(color.x,0.0f,1.0f));
    unsigned int g = (unsigned int) (255.0f * clamp(color.y,0.0f,1.0f));
    unsigned int b = (unsigned int) (255.0f * clamp(color.z,0.0f,1.0f));
    pixels[y*width+x] = (b << 16) + (g << 8) + r;
  }

  

  __forceinline size_t alignTo(const unsigned int size, const unsigned int alignment)
  {
    return ((size+alignment-1)/alignment)*alignment;
  }

  __forceinline void waitOnQueueAndCatchException(sycl::queue &gpu_queue)
  {
    try {
      gpu_queue.wait_and_throw();
    } catch (sycl::exception const& e) {
      std::cout << "Caught synchronous SYCL exception:\n"
                << e.what() << std::endl;
      FATAL("SYCL Exception");     
    }      
  }

  __forceinline void waitOnEventAndCatchException(sycl::event &event)
  {
    try {
      event.wait_and_throw();
    } catch (sycl::exception const& e) {
      std::cout << "Caught synchronous SYCL exception:\n"
                << e.what() << std::endl;
      FATAL("SYCL Exception");     
    }      
  }

  __forceinline float getDeviceExecutionTiming(sycl::event &queue_event)
  {
    const auto t0 = queue_event.template get_profiling_info<sycl::info::event_profiling::command_start>();
    const auto t1 = queue_event.template get_profiling_info<sycl::info::event_profiling::command_end>();
    return (float)((t1-t0)*1E-6);      
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
    ImGui::Text("SPP: %d",user_spp);    
    ImGui::Text("BVH Build Time: %4.4f ms",avg_bvh_build_time.get());
    ImGui::Text("LOD Selection Time: %4.4f ms",avg_lod_selection_time.get());
    ImGui::DragInt("",(int*)&g_lod_threshold,1,2,1000);
    
    if (global_lcgbp_scene->numLCGBP)
    {
      ImGui::Text("numGrids9x9:   %d (out of %d)",global_lcgbp_scene->numCurrentLCGBPStates,global_lcgbp_scene->numLCGBP*(1<<(LOD_LEVELS+1)));
      ImGui::Text("numGrids33x33: %d ",global_lcgbp_scene->numLCGBP);
      ImGui::Text("numTriangles:  %d (out of %d)",global_lcgbp_scene->numCurrentLCGBPStates*numTrianglesPerGrid9x9,global_lcgbp_scene->numLCGBP*numTrianglesPerGrid33x33);
    }
    if (global_lcgbp_scene->numLCMeshClusters)
    {
      ImGui::Text("Active Clusters: %d (out of %d)",global_lcgbp_scene->numLCMeshClusterRootsPerFrame,global_lcgbp_scene->numLCMeshClustersMaxRes);
      ImGui::Text("Quads:           %d (out of %d)",global_lcgbp_scene->numLCMeshClusterQuadsPerFrame,global_lcgbp_scene->numLCQuadsTotal);
      ImGui::Text("Blocks:          %d (out of %d)",global_lcgbp_scene->numLCMeshClusterBlocksPerFrame,global_lcgbp_scene->numLCBlocksTotal);            
    }

    RenderMode rendering_mode = user_rendering_mode;
    if (rendering_mode == RENDER_PATH_TRACER_DENOISE)
    {
      ImGui::Text("Denoising Time: %4.4f ms",avg_denoising_time.get());
      
    }
    
  }


  __forceinline bool subdivideLOD(const Vec2f &lower, const Vec2f &upper, const int width, const int height, const float THRESHOLD)
  {
    bool subdivide = true;
    const float l = length(upper - lower);    
    if (l <= THRESHOLD) subdivide = false;
    return subdivide;
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
    
    const unsigned int wgSize = 16*1;
    LCG_Scene *local_lcgbp_scene = global_lcgbp_scene;
    const unsigned int numLCGBP = local_lcgbp_scene->numLCGBP;
    const unsigned int numLCMeshClusters = local_lcgbp_scene->numLCMeshClusters;
    if ( numLCGBP )
    {
      sycl::event init_event =  global_gpu_queue->submit([&](sycl::handler &cgh) {
        cgh.single_task([=]() {
          local_lcgbp_scene->numCurrentLCGBPStates = 0;
        });
      });

      waitOnEventAndCatchException(init_event);

      void *lcg_ptr = nullptr;
      //unsigned int lcg_num_prims = 0;
    
      const float minLODDistance = local_lcgbp_scene->minLODDistance;
      
      const sycl::nd_range<1> nd_range1(alignTo(numLCGBP,wgSize),sycl::range<1>(wgSize));              
      sycl::event compute_lod_event = global_gpu_queue->submit([=](sycl::handler& cgh){
        cgh.depends_on(init_event);                                                   
        cgh.parallel_for(nd_range1,[=](sycl::nd_item<1> item) {
          const unsigned int i = item.get_global_id(0);
          if (i < numLCGBP)
          {
            const Vec3f org = camera.xfm.p;
            const Vec3f vx = camera.xfm.l.vx;
            const Vec3f vy = camera.xfm.l.vy;
            const Vec3f vz = camera.xfm.l.vz;
            
            LCGBP &current = local_lcgbp_scene->lcgbp[i];
            LODPatchLevel patchLevel = getLODPatchLevel(minLODDistance,current,camera,width,height);

            const BBox3f patch_bounds = current.patch.bounds();

            bool cull = frustumCull( patch_bounds.lower-org,patch_bounds.upper-org,vx*width,vy*height,vz);
            if (cull && 1)
            {
              patchLevel.level = 0;
              patchLevel.blend = 0.0f;
            }

            const unsigned int lod_level = patchLevel.level;
                                                                                             
            unsigned int lod_level_top    = lod_level;
            unsigned int lod_level_right  = lod_level;
            unsigned int lod_level_bottom = lod_level;
            unsigned int lod_level_left   = lod_level;

            LODPatchLevel patchLevel_top    = patchLevel;
            LODPatchLevel patchLevel_right  = patchLevel;
            LODPatchLevel patchLevel_bottom = patchLevel;
            LODPatchLevel patchLevel_left   = patchLevel;
                                                                                            
            if (current.neighbor_top    != -1)
            {
              patchLevel_top   = getLODPatchLevel(minLODDistance,local_lcgbp_scene->lcgbp[current.neighbor_top],camera,width,height);
              lod_level_top    = patchLevel_top.level;
            }
                                                                                               
            if (current.neighbor_right  != -1)
            {
              patchLevel_right  = getLODPatchLevel(minLODDistance,local_lcgbp_scene->lcgbp[current.neighbor_right],camera,width,height);
              lod_level_right  = patchLevel_right.level;
            }
                                                                                             
            if (current.neighbor_bottom != -1)
            {
              patchLevel_bottom = getLODPatchLevel(minLODDistance,local_lcgbp_scene->lcgbp[current.neighbor_bottom],camera,width,height);
              lod_level_bottom = patchLevel_bottom.level;
            }
                                                                                             
            if (current.neighbor_left   != -1)
            {
              patchLevel_left   = getLODPatchLevel(minLODDistance,local_lcgbp_scene->lcgbp[current.neighbor_left],camera,width,height);
              lod_level_left   = patchLevel_left.level;
            }
                                                                                             
            LODEdgeLevel edgeLevels(lod_level);
                                                                                             
            edgeLevels.top    = min(edgeLevels.top,(uchar)lod_level_top);
            edgeLevels.right  = min(edgeLevels.right,(uchar)lod_level_right);
            edgeLevels.bottom = min(edgeLevels.bottom,(uchar)lod_level_bottom);
            edgeLevels.left   = min(edgeLevels.left,(uchar)lod_level_left);
                                                                                             
            unsigned int blend = (unsigned int)floorf(255.0f * patchLevel.blend);
                                                                                             
            const unsigned int numGrids9x9 = 1<<(2*lod_level);
            //const unsigned int offset = ((1<<(2*lod_level))-1)/(4-1);
            const unsigned int offset = atomic_add_global(&local_lcgbp_scene->numCurrentLCGBPStates,numGrids9x9);
            unsigned int index = 0;
            if (lod_level == 0)
            {
              local_lcgbp_scene->lcgbp_state[offset+index] = LCGBP_State(&current,0,0,4,index,lod_level,edgeLevels,blend);
              index++;
            }
            else if (lod_level == 1)
            {
              for (unsigned int y=0;y<2;y++)
                for (unsigned int x=0;x<2;x++)
                {
                  local_lcgbp_scene->lcgbp_state[offset+index] = LCGBP_State(&current,x*16,y*16,2,index,lod_level,edgeLevels,blend);
                  index++;
                }
            }
            else
            {
              for (unsigned int y=0;y<4;y++)
                for (unsigned int x=0;x<4;x++)
                {
                  local_lcgbp_scene->lcgbp_state[offset+index] = LCGBP_State(&current,x*8,y*8,1,index,lod_level,edgeLevels,blend);
                  index++;
                }
            }
          }
                                                                                           
        });
      });
      waitOnEventAndCatchException(compute_lod_event);
      //rtcSetGeometryUserData(local_lcgbp_scene->geometry,lcg_ptr);      
    }   

    if (numLCMeshClusters)
    {
      const unsigned int numRootsTotal = local_lcgbp_scene->numLCMeshClusterRoots;
      
      sycl::event init_event =  global_gpu_queue->submit([&](sycl::handler &cgh) {
        cgh.single_task([=]() {
          local_lcgbp_scene->numLCMeshClusterRootsPerFrame = 0;
          local_lcgbp_scene->numLCMeshClusterQuadsPerFrame = 0;
          local_lcgbp_scene->numLCMeshClusterBlocksPerFrame = 0;                    
        });
      });
      
      const float lod_threshold = g_lod_threshold;
      const sycl::nd_range<1> nd_range1(alignTo(numRootsTotal,wgSize),sycl::range<1>(wgSize));              
      sycl::event compute_lod_event = global_gpu_queue->submit([=](sycl::handler& cgh){
        sycl::local_accessor< uint      ,  0> _cluster_counter(cgh);
        sycl::local_accessor< uint      ,  0> _quad_counter(cgh);
        sycl::local_accessor< uint      ,  0> _block_counter(cgh);
        
        //sycl::local_accessor< uint      ,  1> _local_clusterIDs(sycl::range<1>(),cgh);       
        
        cgh.depends_on(init_event);                                                   
        cgh.parallel_for(nd_range1,[=](sycl::nd_item<1> item) {
          const unsigned int i = item.get_global_id(0);
          uint &cluster_counter    = *_cluster_counter.get_pointer();
          uint &quad_counter       = *_quad_counter.get_pointer();
          uint &block_counter      = *_block_counter.get_pointer();

          cluster_counter = 0;
          quad_counter = 0;
          block_counter = 0;

          item.barrier(sycl::access::fence_space::local_space);
                              
          if (i < local_lcgbp_scene->numLCMeshClusterRoots)
          {
            const unsigned int clusterID = local_lcgbp_scene->lcm_cluster_roots_IDs[i];            
#if 1
            const Vec3f org = camera.xfm.p;
            const Vec3f vx = camera.xfm.l.vx;
            const Vec3f vy = camera.xfm.l.vy;
            const Vec3f vz = camera.xfm.l.vz;
            
            const LossyCompressedMeshCluster &root_cluster = local_lcgbp_scene->lcm_cluster[ clusterID ];
        
            LossyCompressedMesh *mesh = root_cluster.mesh;
            const Vec3f lower = mesh->bounds.lower;
            const Vec3f diag = mesh->bounds.size() * (1.0f / CompressedVertex::RES_PER_DIM);

            bool cull = frustumCull( root_cluster.bounds.lower.decompress(lower,diag)-org,root_cluster.bounds.upper.decompress(lower,diag)-org,vx*width,vy*height,vz);
            if (cull)
            {
              const unsigned int numQuads = root_cluster.numQuads;
              gpu::atomic_add_local(&quad_counter,(unsigned int)numQuads);                                          
              const unsigned int destID = gpu::atomic_add_global(&local_lcgbp_scene->numLCMeshClusterRootsPerFrame,(unsigned int)1);                            
              local_lcgbp_scene->lcm_cluster_roots_IDs_per_frame[destID] = clusterID;              
            }
            else
            {
              const unsigned int STACK_SIZE = 16;
              unsigned int numStackEntries = 1;
              unsigned int stack[STACK_SIZE];
              stack[0] = clusterID;
              while(numStackEntries)
              {
                numStackEntries--;
                const unsigned int currentID = stack[numStackEntries];
                const LossyCompressedMeshCluster &cur = local_lcgbp_scene->lcm_cluster[ currentID ];              
                if (!cur.hasChildren()) // leaf
                {
#if ENABLE_DAG == 1
                  if (cur.hasNeighbor())
                  {                    
                    const uint neighborID = currentID + cur.neighborID;
                    const LossyCompressedMeshCluster &neighbor = local_lcgbp_scene->lcm_cluster[ neighborID ];                                  
                    gpu::atomic_add_local(&quad_counter,(unsigned int)(cur.numQuads + neighbor.numQuads));
                    gpu::atomic_add_local(&block_counter,(unsigned int)(cur.numBlocks + neighbor.numBlocks));
                    
                    const unsigned int destID = gpu::atomic_add_global(&local_lcgbp_scene->numLCMeshClusterRootsPerFrame,(unsigned int)2);
                    local_lcgbp_scene->lcm_cluster_roots_IDs_per_frame[destID+0] =  currentID;
                    local_lcgbp_scene->lcm_cluster_roots_IDs_per_frame[destID+1] =  neighborID;                    
                  }
                  else                  
#endif                  
                  {
                    gpu::atomic_add_local(&quad_counter,(unsigned int)cur.numQuads);
                    gpu::atomic_add_local(&block_counter,(unsigned int)cur.numBlocks);                    
                    const unsigned int destID = gpu::atomic_add_global(&local_lcgbp_scene->numLCMeshClusterRootsPerFrame,(unsigned int)1);
                    local_lcgbp_scene->lcm_cluster_roots_IDs_per_frame[destID] =  currentID;
                  }
                }
                else
                {
                  const Vec3f bounds_lower = cur.bounds.lower.decompress(lower,diag)-org;
                  const Vec3f bounds_upper = cur.bounds.upper.decompress(lower,diag)-org;

                  const Vec2f plane_bounds_lower = projectVertexToPlane(bounds_lower,vx,vy,vz,width,height,true);
                  const Vec2f plane_bounds_upper = projectVertexToPlane(bounds_upper,vx,vy,vz,width,height,true);
                
                  bool subdivide = subdivideLOD(min(plane_bounds_lower,plane_bounds_upper),max(plane_bounds_lower,plane_bounds_upper),width,height,lod_threshold);
                    
                  if (subdivide && (numStackEntries+2 <= STACK_SIZE))
                  {
                    const uint lID = (int)currentID + cur.lodLeftID;
                    const uint rID = (int)currentID + cur.lodRightID;
                    stack[numStackEntries+0] = lID;
                    stack[numStackEntries+1] = rID;
                    numStackEntries+=2;
                    
                  }
                  else
                  {
#if ENABLE_DAG == 1
                    if (cur.hasNeighbor())
                    {                    
                      const uint neighborID = currentID + cur.neighborID;
                      const LossyCompressedMeshCluster &neighbor = local_lcgbp_scene->lcm_cluster[ neighborID ];                                  
                      gpu::atomic_add_local(&quad_counter,(unsigned int)(cur.numQuads + neighbor.numQuads));
                      gpu::atomic_add_local(&block_counter,(unsigned int)(cur.numBlocks + neighbor.numBlocks));                      
                      const unsigned int destID = gpu::atomic_add_global(&local_lcgbp_scene->numLCMeshClusterRootsPerFrame,(unsigned int)2);
                      local_lcgbp_scene->lcm_cluster_roots_IDs_per_frame[destID+0] =  currentID;
                      local_lcgbp_scene->lcm_cluster_roots_IDs_per_frame[destID+1] =  neighborID;                    
                    }
                    else                    
#endif
                    {
                      gpu::atomic_add_local(&quad_counter,(unsigned int)cur.numQuads);
                      gpu::atomic_add_local(&block_counter,(unsigned int)cur.numBlocks);                                          
                      const unsigned int destID = gpu::atomic_add_global(&local_lcgbp_scene->numLCMeshClusterRootsPerFrame,(unsigned int)1);
                      local_lcgbp_scene->lcm_cluster_roots_IDs_per_frame[destID] =  currentID;
                    }
                  }                                
                }              
              }
            }
#else              
            {
              const Vec3f org = camera.xfm.p;
              const Vec3f vx = camera.xfm.l.vx;
              const Vec3f vy = camera.xfm.l.vy;
              const Vec3f vz = camera.xfm.l.vz;
              
              const LossyCompressedMeshCluster &cluster = local_lcgbp_scene->lcm_cluster[ clusterID ];
              LossyCompressedMesh *mesh = cluster.mesh;
              const Vec3f lower = mesh->bounds.lower;
              const Vec3f diag = mesh->bounds.size() * (1.0f / CompressedVertex::RES_PER_DIM);

              const Vec3f bounds_lower = cluster.bounds.lower.decompress(lower,diag)-org;
              const Vec3f bounds_upper = cluster.bounds.upper.decompress(lower,diag)-org;

              bool cull = frustumCull(bounds_lower,bounds_upper,vx*width,vy*height,vz);

              if (!cull)
              {
                gpu::atomic_add_local(&quad_counter,(unsigned int)cluster.numQuads);
                gpu::atomic_add_local(&block_counter,(unsigned int)cluster.numBlocks);                
                const unsigned int destID = gpu::atomic_add_global(&local_lcgbp_scene->numLCMeshClusterRootsPerFrame,(unsigned int)1);                            
                local_lcgbp_scene->lcm_cluster_roots_IDs_per_frame[destID] = clusterID;
              }
            }
#endif              
            
          }

          item.barrier(sycl::access::fence_space::local_space);
          
          const uint localID = item.get_local_id(0);
          if (localID == 0)
          {
            if (quad_counter > 0)
              gpu::atomic_add_global(&local_lcgbp_scene->numLCMeshClusterQuadsPerFrame,(unsigned int)quad_counter);
            if (block_counter > 0)
              gpu::atomic_add_global(&local_lcgbp_scene->numLCMeshClusterBlocksPerFrame,(unsigned int)block_counter);
            
          }
        });
      });
      waitOnEventAndCatchException(compute_lod_event);

      waitOnQueueAndCatchException(*global_gpu_queue);  // FIXME            
    }

    rtcSetLCData(local_lcgbp_scene->geometry, local_lcgbp_scene->numCurrentLCGBPStates, local_lcgbp_scene->lcgbp_state, local_lcgbp_scene->lcm_cluster, local_lcgbp_scene->numLCMeshClusterRootsPerFrame,local_lcgbp_scene->lcm_cluster_roots_IDs_per_frame);

    double dt0_lod = (getSeconds()-t0_lod)*1000.0;                                            
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
                                       const ISPCCamera& camera)
  {
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
        cgh.parallel_for(nd_range,[=](sycl::nd_item<2> item) {
          const unsigned int x = item.get_global_id(1); if (x >= width ) return;
          const unsigned int y = item.get_global_id(0); if (y >= height) return;
          renderPixelStandard(ldata,x,y,pixels,width,height,time,camera,lcgbp_scene,rendering_mode,spp);
        });
      });
      waitOnEventAndCatchException(event);
      const auto t0 = event.template get_profiling_info<sycl::info::event_profiling::command_start>();
      const auto t1 = event.template get_profiling_info<sycl::info::event_profiling::command_end>();
      const double dt = (t1-t0)*1E-9;
      ((ISPCCamera*)&camera)->render_time = dt;        
    }
    else
    {
      GBuffer *gBuffer  = denoiser->gBuffer;      
      GBufferOutput *output  = denoiser->outputBuffer;

      bool denoise = rendering_mode == RENDER_PATH_TRACER_DENOISE;
      sycl::event event = renderFramePathTracer(pixels,width,height,time,camera,data,user_spp,gBuffer,denoise);
      waitOnEventAndCatchException(event);
      
      const auto t0 = event.template get_profiling_info<sycl::info::event_profiling::command_start>();
      const auto t1 = event.template get_profiling_info<sycl::info::event_profiling::command_end>();
      const double dt = (t1-t0)*1E-9;
      ((ISPCCamera*)&camera)->render_time = dt;
      
      if (denoise)
      {
        double t0 = getSeconds();                                                    
        denoiser->execute();
        
        sycl::event event_denoising = global_gpu_queue->submit([=](sycl::handler& cgh) {
          const sycl::nd_range<2> nd_range = make_nd_range(height,width);
          cgh.parallel_for(nd_range,[=](sycl::nd_item<2> item) {
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
    TutorialData_Destructor(&data);
  }

  

} // namespace embree
