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

#define RELATIVE_MIN_LOD_DISTANCE_FACTOR 32.0f
//#define RELATIVE_MIN_LOD_DISTANCE_FACTOR 16.0f

#include "../../kernels/rthwif/builder/gpu/lcgbp.h"
#include "../../kernels/rthwif/builder/gpu/morton.h"


#include "../common/tutorial/optics.h"
#include "../common/lights/ambient_light.cpp"
#include "../common/lights/directional_light.cpp"
#include "../common/lights/point_light.cpp"
#include "../common/lights/quad_light.cpp"
#include "../common/lights/spot_light.cpp"

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


  extern "C" RenderMode user_rendering_mode = RENDER_PRIMARY;
  extern "C" uint user_spp = 1;

  Averaged<double> avg_bvh_build_time(64,1.0);
  Averaged<double> avg_lod_selection_crack_fixing_time(64,1.0);

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
  
  static const uint LOD_LEVELS = 3;
  //static const uint NUM_TOTAL_QUAD_NODES_PER_RTC_LCG = (1-(1<<(2*LOD_LEVELS)))/(1-4);

  struct LODPatchLevel
  {
    uint level;
    float blend;

    __forceinline LODPatchLevel(const uint level, const float blend) : level(level), blend(blend) {}
  };


  __forceinline LODPatchLevel getLODPatchLevel(const float MIN_LOD_DISTANCE,LCGBP &current,const ISPCCamera& camera, const uint width, const uint height)
  {
    const float minDistance = MIN_LOD_DISTANCE;
    const uint startRange[LOD_LEVELS+1] = { 0,1,3,7};
    const uint   endRange[LOD_LEVELS+1] = { 1,3,7,15};    
    
    const Vec3f v0 = current.patch.v0;
    const Vec3f v1 = current.patch.v1;
    const Vec3f v2 = current.patch.v2;
    const Vec3f v3 = current.patch.v3;

    const Vec3f center = lerp(lerp(v0,v1,0.5f),lerp(v2,v3,0.5f),0.5f);
    const Vec3f org = camera.xfm.p;

    const float dist = fabs(length(center-org));
    const float dist_minDistance = dist/minDistance;
    const uint dist_level = floorf(dist_minDistance);

    uint segment = -1;
    for (uint i=0;i<LOD_LEVELS;i++)
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
  

  __forceinline Vec2f projectVertexToPlane(const Vec3f &p, const Vec3f &vx, const Vec3f &vy, const Vec3f &vz, const uint width, const uint height)
  {
    const Vec3f vn = cross(vx,vy);    
    const float distance = (float)dot(vn,vz) / (float)dot(vn,p);
    Vec3f pip = p * distance;
    if (distance < 0.0f)
      pip = vz;
    float a = dot((pip-vz),vx);
    float b = dot((pip-vz),vy);
    a = min(max(a,0.0f),(float)width);
    b = min(max(b,0.0f),(float)height);    
    return Vec2f(a,b);
  }
  
  __forceinline LODEdgeLevel getLODEdgeLevels(LCGBP &current,const ISPCCamera& camera, const uint width, const uint height)
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
  
  inline Vec3fa getVertex(const uint x, const uint y, const Vec3fa *const vtx, const uint grid_resX, const uint grid_resY)
  {
    const uint px = min(x,grid_resX-1);
    const uint py = min(y,grid_resY-1);    
    return vtx[py*grid_resX + px];
  }
  
  // ==============================================================================================
  // ==============================================================================================
  // ==============================================================================================
  
  struct __aligned(64) LCG_Scene {
    static const uint LOD_LEVELS = 3;

    /* --- general data --- */
    BBox3f bounds;
    
    /* --- lossy compressed bilinear patches --- */
    uint numAllocatedLCGBP;
    uint numAllocatedLCGBPStates;    
    uint numLCGBP;
    uint numCurrentLCGBPStates;        
    LCGBP *lcgbp;
    LCGBP_State *lcgbp_state;    
    uint numCrackFixQuadNodes;

    /* --- lossy compressed meshes --- */
    //uint numLCMeshes;        
    uint numLCMeshClusters;    
    //LossyCompressedMesh *lcm;
    LossyCompressedMeshCluster *lcm_cluster;

    /* --- embree geometry --- */
    RTCGeometry geometry;
    uint geomID;

    /* --- texture handle --- */
    Texture* map_Kd;

    /* --- LOD settings --- */
    float minLODDistance;
    
    LCG_Scene(const uint maxNumLCGBP);

    void addGrid(const uint gridResX, const uint gridResY, const Vec3fa *const vtx);
  };
  
  LCG_Scene::LCG_Scene(const uint maxNumLCGBP)
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
    numLCMeshClusters = 0;
    lcm_cluster = 0;
    
    if (maxNumLCGBP)
    {
      lcgbp       = (LCGBP*)alignedUSMMalloc(sizeof(LCGBP)*numAllocatedLCGBP,64,EMBREE_USM_SHARED /*EmbreeUSMMode::EMBREE_DEVICE_READ_WRITE*/);
      lcgbp_state = (LCGBP_State*)alignedUSMMalloc(sizeof(LCGBP_State)*numAllocatedLCGBPStates,64,EMBREE_USM_SHARED/*EmbreeUSMMode::EMBREE_DEVICE_READ_WRITE*/);
      PRINT2(numAllocatedLCGBP,numAllocatedLCGBP*sizeof(LCGBP));
      PRINT2(numAllocatedLCGBPStates,numAllocatedLCGBPStates*sizeof(LCGBP_State));
    }

    PRINT(numLCMeshClusters);
    
    // if (numLCMeshes)
    // {
    //   lcm = (LossyCompressedMesh*)alignedUSMMalloc(sizeof(LossyCompressedMesh)*numLCMeshes,64,EMBREE_USM_SHARED /*EmbreeUSMMode::EMBREE_DEVICE_READ_WRITE*/);      
    // }

    // if (numLCMeshClusters)
    // {
    //   lcm_cluster = (LossyCompressedMeshCluster*)alignedUSMMalloc(sizeof(LossyCompressedMeshCluster)*numLCMeshClusters,64,EMBREE_USM_SHARED /*EmbreeUSMMode::EMBREE_DEVICE_READ_WRITE*/);            
    // }    
  }

  void LCG_Scene::addGrid(const uint gridResX, const uint gridResY, const Vec3fa *const vtx)
  {
    double avg_error = 0.0;
    double max_error = 0.0;
    uint num_error = 0;

    PRINT(gridResX);
    PRINT(gridResY);

    const uint lcg_resX = ((gridResX-1) / LCGBP::GRID_RES_QUAD);
    const uint lcg_resY = ((gridResY-1) / LCGBP::GRID_RES_QUAD);

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

        const uint current_x = start_x / LCGBP::GRID_RES_QUAD;
        const uint current_y = start_y / LCGBP::GRID_RES_QUAD;        
        
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
  }

  LCG_Scene *global_lcgbp_scene = nullptr;

  // ==============================================================================================
  // ==============================================================================================
  // ==============================================================================================


  void extractRanges(const gpu::Range &current, const gpu::MortonCodePrimitive64x32Bits3D *const mcodes, std::vector<gpu::Range> &ranges, const uint threshold)
  {
    if (current.size() < threshold)
    {
      ranges.push_back(current);
    }
    else
    {
      gpu::Range left, right;
      splitRange(current,mcodes,left,right);
      extractRanges(left,mcodes,ranges,threshold);
      extractRanges(right,mcodes,ranges,threshold);      
    }
  }

  __forceinline uint remap_vtx_index(const uint v, std::map<uint,uint> &index_map, uint &numLocalIndices)
  {
    auto e = index_map.find(v);
    if (e != index_map.end()) return e->second;
    const uint ID = numLocalIndices++;
    index_map[v] = ID;
    return ID;
  }
    

  void convertISPCQuadMesh(ISPCQuadMesh* mesh, RTCScene scene, ISPCOBJMaterial *material,const uint geomID,std::vector<LossyCompressedMesh*> &lcm_ptrs,std::vector<LossyCompressedMeshCluster> &lcm_clusters, size_t &totalCompressedSize, size_t &numDecompressedBlocks)
  {
    const uint lcm_ID = lcm_ptrs.size();
    const uint numQuads = mesh->numQuads;
    const uint INITIAL_CREATE_RANGE_THRESHOLD = LossyCompressedMeshCluster::MAX_QUADS_PER_CLUSTER;
    // === get centroid and geometry bounding boxes ===
    
    BBox3fa centroidBounds(empty);
    BBox3fa geometryBounds(empty);
    
    for (uint i=0;i<numQuads;i++)
    {
      const uint v0 = mesh->quads[i].v0;
      const uint v1 = mesh->quads[i].v1;
      const uint v2 = mesh->quads[i].v2;
      const uint v3 = mesh->quads[i].v3;

      const Vec3fa &vtx0 = mesh->positions[0][v0];
      const Vec3fa &vtx1 = mesh->positions[0][v1];
      const Vec3fa &vtx2 = mesh->positions[0][v2];
      const Vec3fa &vtx3 = mesh->positions[0][v3];

      BBox3fa quadBounds(empty);
      quadBounds.extend(vtx0);
      quadBounds.extend(vtx1);
      quadBounds.extend(vtx2);
      quadBounds.extend(vtx3);
      centroidBounds.extend(quadBounds.center());
      geometryBounds.extend(quadBounds);
    }

    // === create morton codes for quads ===
    
    const Vec3f lower = centroidBounds.lower;
    const Vec3f diag = centroidBounds.size();
    const Vec3f inv_diag  = diag != Vec3fa(0.0f) ? Vec3fa(1.0f) / diag : Vec3fa(0.0f);

    std::vector<gpu::MortonCodePrimitive64x32Bits3D> mcodes;
    std::vector<gpu::Range> ranges;
    
    for (uint i=0;i<numQuads;i++)
    {
      const uint v0 = mesh->quads[i].v0;
      const uint v1 = mesh->quads[i].v1;
      const uint v2 = mesh->quads[i].v2;
      const uint v3 = mesh->quads[i].v3;

      const Vec3fa &vtx0 = mesh->positions[0][v0];
      const Vec3fa &vtx1 = mesh->positions[0][v1];
      const Vec3fa &vtx2 = mesh->positions[0][v2];
      const Vec3fa &vtx3 = mesh->positions[0][v3];

      BBox3fa quadBounds(empty);
      quadBounds.extend(vtx0);
      quadBounds.extend(vtx1);
      quadBounds.extend(vtx2);
      quadBounds.extend(vtx3);
            
      const uint grid_size = 1 << 21; // 3*21 = 63
      const Vec3f grid_base = lower;
      const Vec3f grid_extend = diag;
      
      const Vec3f grid_scale = ((float)grid_size * 0.99f) * inv_diag;
      const Vec3f centroid =  quadBounds.center();

      const Vec3f gridpos_f = (centroid-grid_base)*grid_scale;                                                                      
      const uint gx = (uint)gridpos_f.x;
      const uint gy = (uint)gridpos_f.y;
      const uint gz = (uint)gridpos_f.z;
      const uint64_t code = bitInterleave64<uint64_t>(gx,gy,gz);
      mcodes.push_back(gpu::MortonCodePrimitive64x32Bits3D(code,i));      
    }

    // === sort morton codes ===
    
    std::sort(mcodes.begin(), mcodes.end()); 

    // === extract ranges ===
    
    gpu::Range current(0,mcodes.size());
    extractRanges(current,&*mcodes.begin(),ranges,INITIAL_CREATE_RANGE_THRESHOLD);

    // === test range if it fullfills requirements, split if necessary ===

    uint numTotalVertices = 0;

    for (uint i=0;i<ranges.size();i++)
    {      
      while(1)
      {
        std::map<uint,uint> index_map;
        uint numLocalIndices = 0;
        bool split = false;
        for (uint j=ranges[i].start;j<ranges[i].end;j++)
        {
          const uint index = mcodes[j].getIndex();
          const uint v0 = mesh->quads[index].v0;
          const uint v1 = mesh->quads[index].v1;
          const uint v2 = mesh->quads[index].v2;
          const uint v3 = mesh->quads[index].v3;

          remap_vtx_index(v0,index_map,numLocalIndices);
          remap_vtx_index(v1,index_map,numLocalIndices);
          remap_vtx_index(v2,index_map,numLocalIndices);
          remap_vtx_index(v3,index_map,numLocalIndices);
          if (index_map.size() > 256)
          {
            gpu::Range left,right;
            const uint center = (ranges[i].start + ranges[i].end)/2;
            left.start  = ranges[i].start;
            left.end    = center;
            right.start = center;
            right.end = ranges[i].end;
            ranges[i] = left;
            ranges.push_back(right);
            split = true;
            break;
          }
        }
        if (!split)
        {
          numTotalVertices += index_map.size();
          break;
        }
      }      
    }
    
    // === allocate LossyCompressedMesh in USM ===
    
    LossyCompressedMesh *lcm = (LossyCompressedMesh *)alignedUSMMalloc(sizeof(LossyCompressedMesh),64);
    lcm_ptrs.push_back(lcm);
  
    lcm->bounds             = geometryBounds;
    lcm->numQuads           = numQuads;
    lcm->numVertices        = mesh->numVertices;
    lcm->geomID             = geomID; 
    lcm->compressedVertices = (CompressedVertex*)alignedUSMMalloc(sizeof(CompressedVertex)*numTotalVertices,64); // FIXME
    lcm->compressedIndices  = (CompressedQuadIndices*)alignedUSMMalloc(sizeof(CompressedQuadIndices)*numQuads,64); //FIXME    
           
    uint globalCompressedVertexOffset = 0;
    uint globalCompressedIndexOffset = 0;

    // === quantize vertices with respect to geometry bounding box ===
    
    const Vec3f geometry_lower    = geometryBounds.lower;
    const Vec3f geometry_diag     = geometryBounds.size();
    const Vec3f geometry_inv_diag = geometry_diag != Vec3fa(0.0f) ? Vec3fa(1.0f) / geometry_diag : Vec3fa(0.0f);
    
    for (uint i=0;i<ranges.size();i++)
    {
      
      LossyCompressedMeshCluster cluster;
      cluster.numQuads  = ranges[i].size();
      cluster.numBlocks = LossyCompressedMeshCluster::getDecompressedSizeInBytes(ranges[i].size())/64;
      cluster.ID = i;
      cluster.offsetIndices  = globalCompressedIndexOffset;      
      cluster.offsetVertices = globalCompressedVertexOffset;
      cluster.mesh = lcm;

      std::map<uint,uint> index_map;
      uint numLocalIndices = 0;

      // === remap vertices relative to cluster ===
      
      for (uint j=ranges[i].start;j<ranges[i].end;j++)
      {
        const uint index = mcodes[j].getIndex();
        const uint v0 = mesh->quads[index].v0;
        const uint v1 = mesh->quads[index].v1;
        const uint v2 = mesh->quads[index].v2;
        const uint v3 = mesh->quads[index].v3;

        const uint remaped_v0 =  remap_vtx_index(v0,index_map,numLocalIndices);
        const uint remaped_v1 =  remap_vtx_index(v1,index_map,numLocalIndices);
        const uint remaped_v2 =  remap_vtx_index(v2,index_map,numLocalIndices);
        const uint remaped_v3 =  remap_vtx_index(v3,index_map,numLocalIndices);

        lcm->compressedIndices[ globalCompressedIndexOffset++ ] = CompressedQuadIndices(remaped_v0,remaped_v1,remaped_v2,remaped_v3);
      }

      for (std::map<uint,uint>::iterator i=index_map.begin(); i != index_map.end(); i++)
      {
        const uint old_v = (*i).first;
        const uint new_v = (*i).second;
        lcm->compressedVertices[ globalCompressedVertexOffset + new_v ] = CompressedVertex(mesh->positions[0][old_v],geometry_lower,geometry_inv_diag);        
      }
      
      globalCompressedVertexOffset += index_map.size();
      
      if (index_map.size() > 256) FATAL("index_map"); // byte indices
      if (globalCompressedVertexOffset > numTotalVertices) FATAL("numTotalVertices");
      
      lcm_clusters.push_back(cluster);
      numDecompressedBlocks += cluster.numBlocks;      
    }

    const size_t uncompressedSizeMeshBytes = mesh->numVertices * sizeof(Vec3f) + mesh->numQuads * sizeof(uint) * 4;
    const size_t compressedSizeMeshBytes = sizeof(CompressedVertex)*numTotalVertices + sizeof(CompressedQuadIndices)*numQuads;
    const size_t clusterSizeBytes = ranges.size()*sizeof(LossyCompressedMeshCluster);
    PRINT5(lcm_ID,uncompressedSizeMeshBytes,compressedSizeMeshBytes,(float)compressedSizeMeshBytes/uncompressedSizeMeshBytes,clusterSizeBytes);

    totalCompressedSize += compressedSizeMeshBytes + clusterSizeBytes;
  }  
  
  void convertISPCGridMesh(ISPCGridMesh* grid, RTCScene scene, ISPCOBJMaterial *material)
  {
    uint numLCGBP = 0;
    
    /* --- count lcgbp --- */
    for (uint i=0;i<grid->numGrids;i++)
    {
      PRINT3(i,grid->grids[i].resX,grid->grids[i].resY);      
      const uint grid_resX = grid->grids[i].resX;
      const uint grid_resY = grid->grids[i].resY;
      const uint numInitialSubGrids = ((grid_resX-1) / LCGBP::GRID_RES_QUAD) * ((grid_resY-1) / LCGBP::GRID_RES_QUAD);
      //PRINT(numInitialSubGrids);
      numLCGBP  += numInitialSubGrids;
    }
    PRINT(numLCGBP);

    /* --- allocate global LCGBP --- */
    global_lcgbp_scene = (LCG_Scene*)alignedUSMMalloc(sizeof(LCG_Scene),64);
    new (global_lcgbp_scene) LCG_Scene(numLCGBP);
    
    /* --- fill array of LCGBP --- */
    for (uint i=0;i<grid->numGrids;i++)
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
  

  void generateGrid(RTCScene scene, const uint gridResX, const uint gridResY)
  {
    const uint numLCGBP = ((gridResX-1) / LCGBP::GRID_RES_QUAD) * ((gridResY-1) / LCGBP::GRID_RES_QUAD);

    /* --- allocate global LCGBP --- */
    global_lcgbp_scene = (LCG_Scene*)alignedUSMMalloc(sizeof(LCG_Scene),64);
    new (global_lcgbp_scene) LCG_Scene(numLCGBP);

    const uint vertices = gridResX*gridResY;
    Vec3fa *vtx = (Vec3fa*)malloc(sizeof(Vec3fa)*vertices);

    const FileName fileNameDisplacement("Rock_Mossy_02_height.png");
    Texture *displacement = new Texture(loadImage(fileNameDisplacement),fileNameDisplacement);
    PRINT2(displacement->width,displacement->height);
    
    for (uint y=0;y<gridResY;y++)
      for (uint x=0;x<gridResX;x++)
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

    uint numGridMeshes = 0;
    uint numQuadMeshes = 0;
    uint numQuads      = 0;
    for (unsigned int geomID=0; geomID<g_ispc_scene->numGeometries; geomID++)
    {
      ISPCGeometry* geometry = g_ispc_scene->geometries[geomID];
      if (geometry->type == GRID_MESH) numGridMeshes++;
      else if (geometry->type == QUAD_MESH) { numQuadMeshes++; numQuads+= ((ISPCQuadMesh*)geometry)->numQuads; }
    }

    global_lcgbp_scene = (LCG_Scene*)alignedUSMMalloc(sizeof(LCG_Scene),64);
    new (global_lcgbp_scene) LCG_Scene(0);    

    std::vector<LossyCompressedMesh*> lcm_ptrs;
    std::vector<LossyCompressedMeshCluster> lcm_clusters;
    size_t totalCompressedSize = 0;
    size_t numDecompressedBlocks = 0;
    
    for (unsigned int geomID=0; geomID<g_ispc_scene->numGeometries; geomID++)
    {
      ISPCGeometry* geometry = g_ispc_scene->geometries[geomID];
      if (geometry->type == GRID_MESH)
        convertISPCGridMesh((ISPCGridMesh*)geometry,data.g_scene, (ISPCOBJMaterial*)g_ispc_scene->materials[geomID]);
      else if (geometry->type == QUAD_MESH)
        convertISPCQuadMesh((ISPCQuadMesh*)geometry,data.g_scene, (ISPCOBJMaterial*)g_ispc_scene->materials[geomID],geomID,lcm_ptrs,lcm_clusters,totalCompressedSize,numDecompressedBlocks);
    }
    
    // === finalize quad meshes ===
    if (numQuadMeshes)
    {
      global_lcgbp_scene->numLCMeshClusters = lcm_clusters.size();
      global_lcgbp_scene->lcm_cluster = (LossyCompressedMeshCluster*)alignedUSMMalloc(sizeof(LossyCompressedMeshCluster)*global_lcgbp_scene->numLCMeshClusters,64,EMBREE_USM_SHARED /*EmbreeUSMMode::EMBREE_DEVICE_READ_WRITE*/);
      for (uint i=0;i<global_lcgbp_scene->numLCMeshClusters;i++)
        global_lcgbp_scene->lcm_cluster[i] = lcm_clusters[i];
      
      global_lcgbp_scene->geometry = rtcNewGeometry (g_device, RTC_GEOMETRY_TYPE_LOSSY_COMPRESSED_GEOMETRY);
      rtcCommitGeometry(global_lcgbp_scene->geometry);
      global_lcgbp_scene->geomID = rtcAttachGeometry(data.g_scene,global_lcgbp_scene->geometry);
      //rtcReleaseGeometry(geom);
      global_lcgbp_scene->map_Kd = nullptr;
      
      PRINT(global_lcgbp_scene->numLCMeshClusters);
      PRINT2(numQuadMeshes,numQuads);
      PRINT3(totalCompressedSize,(float)totalCompressedSize/numQuads,(float)totalCompressedSize/numQuads*0.5f);
      PRINT3(numDecompressedBlocks,numDecompressedBlocks*64,(float)numDecompressedBlocks*64/totalCompressedSize);
      
    }
    
#else
    const uint gridResX = 16*1024;
    const uint gridResY = 16*1024;    
    generateGrid(data.g_scene,gridResX,gridResY);
#endif    
    //exit(0);
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

    const uint localID = ray.primID & (((uint)1<<RTC_LOSSY_COMPRESSED_GRID_LOCAL_ID_SHIFT)-1);
    const uint primID = ray.primID >> RTC_LOSSY_COMPRESSED_GRID_LOCAL_ID_SHIFT;
    
    Vec3f color(1.0f,1.0f,1.0f);
    
    if (mode == RENDER_DEBUG_QUADS)
    {
      const float LINE_THRESHOLD = 0.1f;
      if (ray.u <= LINE_THRESHOLD ||
          ray.v <= LINE_THRESHOLD ||
          ray.u + ray.v <= LINE_THRESHOLD)
        color = Vec3fa(1.0f,0.0f,0.0f);      
    }
    else if (mode == RENDER_DEBUG_SUBGRIDS)
    {
      const uint gridID = lcgbp_scene->lcgbp_state[primID].lcgbp->ID;
      const uint subgridID = lcgbp_scene->lcgbp_state[primID].localID;    
      color = randomColor(gridID*(16+4+1)+subgridID);   
    }    
    else if (mode == RENDER_DEBUG_GRIDS)
    {
      const uint gridID = lcgbp_scene->lcgbp_state[primID].lcgbp->ID;      
      color = randomColor(gridID);   
    }
    else if (mode == RENDER_DEBUG_LOD)
    {
      const uint step = lcgbp_scene->lcgbp_state[primID].step; 
      if (step == 4)
        color = Vec3fa(0,0,1);
      else if (step == 2)
        color = Vec3fa(0,1,0);
      else if (step == 1)
        color = Vec3fa(1,0,0);            
    }
    else if (mode == RENDER_DEBUG_CRACK_FIXING)
    {
      const uint cracks_fixed = lcgbp_scene->lcgbp_state[primID].cracksFixed();
      if (cracks_fixed)
        color = Vec3fa(1,0,1);      
    }
    else if (mode == RENDER_DEBUG_CLOD)
    {
      const uint step = lcgbp_scene->lcgbp_state[primID].step; 
      if (step == 4)
        color = Vec3fa(0,0,1);
      else if (step == 2)
        color = Vec3fa(0,1,0);
      else if (step == 1)
        color = Vec3fa(1,0,0);                  
      const uint blend = (uint)lcgbp_scene->lcgbp_state[primID].blend;
      if (blend)
        color = Vec3fa(1,1,0);      
    }    
    else if (mode == RENDER_DEBUG_TEXTURE)
    {
      const uint flip_uv = localID & 1;
      const uint localQuadID = localID>>1;
      const uint local_y = localQuadID /  RTC_LOSSY_COMPRESSED_GRID_QUAD_RES;
      const uint local_x = localQuadID %  RTC_LOSSY_COMPRESSED_GRID_QUAD_RES;

      const LCGBP_State &state = lcgbp_scene->lcgbp_state[primID];
      const LCGBP &current = *state.lcgbp;
      const uint start_x = state.start_x;
      const uint start_y = state.start_y;
      const uint end_x = state.start_x + state.step*8;
      const uint end_y = state.start_y + state.step*8;

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
                           const uint spp)
  {
    RandomSampler sampler;

    Vec3fa color(0.0f);
    const float inv_spp = 1.0f / (float)spp;
    
    for (uint i=0;i<spp;i++)
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

  void renderPixelPathTracer(const TutorialData& data,
                             int x, int y,
                             int* pixels,
                             const unsigned int width,
                             const unsigned int height,
                             const float time,
                             const ISPCCamera& camera,
                             RayStats& stats,
                             const RTCFeatureFlags features);


  extern "C" void renderFrameStandard (int* pixels,
                                       const unsigned int width,
                                       const unsigned int height,
                                       const float time,
                                       const ISPCCamera& camera)
  {
    /* render all pixels */
#if defined(EMBREE_SYCL_TUTORIAL)
    RenderMode rendering_mode = user_rendering_mode;
    if (rendering_mode != RENDER_PATH_TRACER)
    {
      LCG_Scene *lcgbp_scene = global_lcgbp_scene;
      uint spp = user_spp;
      TutorialData ldata = data;
      sycl::event event = global_gpu_queue->submit([=](sycl::handler& cgh){
        const sycl::nd_range<2> nd_range = make_nd_range(height,width);
        cgh.parallel_for(nd_range,[=](sycl::nd_item<2> item) {
          const unsigned int x = item.get_global_id(1); if (x >= width ) return;
          const unsigned int y = item.get_global_id(0); if (y >= height) return;
          renderPixelStandard(ldata,x,y,pixels,width,height,time,camera,lcgbp_scene,rendering_mode,spp);
        });
      });
      global_gpu_queue->wait_and_throw();
      const auto t0 = event.template get_profiling_info<sycl::info::event_profiling::command_start>();
      const auto t1 = event.template get_profiling_info<sycl::info::event_profiling::command_end>();
      const double dt = (t1-t0)*1E-9;
      ((ISPCCamera*)&camera)->render_time = dt;        
    }
    else
    {
      TutorialData ldata = data;
      ldata.spp = user_spp;

      int numMaterials = ldata.ispc_scene->numMaterials;

#if 0      
      PRINT( numMaterials );
      PRINT(  ((ISPCOBJMaterial*)ldata.ispc_scene->materials[0])->Kd );
      PRINT(  ((ISPCOBJMaterial*)ldata.ispc_scene->materials[0])->Ks );   
#endif
      
      sycl::event event = global_gpu_queue->submit([=](sycl::handler& cgh) {
        const sycl::nd_range<2> nd_range = make_nd_range(height,width);
        cgh.parallel_for(nd_range,[=](sycl::nd_item<2> item) {
          const unsigned int x = item.get_global_id(1); if (x >= width ) return;
          const unsigned int y = item.get_global_id(0); if (y >= height) return;
          RayStats stats;
          const RTCFeatureFlags feature_mask = RTC_FEATURE_FLAG_ALL;
          renderPixelPathTracer(ldata,x,y,pixels,width,height,time,camera,stats,feature_mask);
        });
      });
      global_gpu_queue->wait_and_throw();

      const auto t0 = event.template get_profiling_info<sycl::info::event_profiling::command_start>();
      const auto t1 = event.template get_profiling_info<sycl::info::event_profiling::command_end>();
      const double dt = (t1-t0)*1E-9;
      ((ISPCCamera*)&camera)->render_time = dt;
      
    }
#endif
  }

  __forceinline size_t alignTo(const uint size, const uint alignment)
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
  static __forceinline uint atomic_add_global(T *dest, const T count=1)
  {
    sycl::atomic_ref<T, sycl::memory_order::relaxed, sycl::memory_scope::device,sycl::access::address_space::global_space> counter(*dest);        
    return counter.fetch_add(count);      
  }


  extern "C" void device_gui()
  {
    const uint numTrianglesPerGrid9x9 = 8*8*2;
    const uint numTrianglesPerGrid33x33 = 32*32*2;
    ImGui::Text("SPP: %d",user_spp);    
    ImGui::Text("BVH Build Time: %4.4f ms",avg_bvh_build_time.get());
    if (global_lcgbp_scene->numLCGBP)
    {
      ImGui::Text("numGrids9x9:   %d (out of %d)",global_lcgbp_scene->numCurrentLCGBPStates,global_lcgbp_scene->numLCGBP*(1<<(LOD_LEVELS+1)));
      ImGui::Text("numGrids33x33: %d ",global_lcgbp_scene->numLCGBP);
      ImGui::Text("numTriangles: %d (out of %d)",global_lcgbp_scene->numCurrentLCGBPStates*numTrianglesPerGrid9x9,global_lcgbp_scene->numLCGBP*numTrianglesPerGrid33x33);
    }
  }
  
  
/* called by the C++ code to render */
  extern "C" void device_render (int* pixels,
                                 const unsigned int width,
                                 const unsigned int height,
                                 const float time,
                                 const ISPCCamera& camera)
  {
#if defined(EMBREE_SYCL_TUTORIAL)
    
    LCG_Scene *local_lcgbp_scene = global_lcgbp_scene;
    sycl::event init_event =  global_gpu_queue->submit([&](sycl::handler &cgh) {
      cgh.single_task([=]() {
        local_lcgbp_scene->numCurrentLCGBPStates = 0;
      });
    });

    waitOnEventAndCatchException(init_event);

    void *lcg_ptr = nullptr;
    uint lcg_num_prims = 0;
    
    const uint wgSize = 64;
    const uint numLCGBP = local_lcgbp_scene->numLCGBP;
    if ( numLCGBP )
    {
      const sycl::nd_range<1> nd_range1(alignTo(numLCGBP,wgSize),sycl::range<1>(wgSize));              
      sycl::event compute_lod_event = global_gpu_queue->submit([=](sycl::handler& cgh){
        cgh.depends_on(init_event);                                                   
        cgh.parallel_for(nd_range1,[=](sycl::nd_item<1> item) {
          const uint i = item.get_global_id(0);
          if (i < numLCGBP)
          {
            LCGBP &current = local_lcgbp_scene->lcgbp[i];
            const float minLODDistance = local_lcgbp_scene->minLODDistance;
            LODPatchLevel patchLevel = getLODPatchLevel(minLODDistance,current,camera,width,height);
            const uint lod_level = patchLevel.level;
                                                                                             
            uint lod_level_top    = lod_level;
            uint lod_level_right  = lod_level;
            uint lod_level_bottom = lod_level;
            uint lod_level_left   = lod_level;

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
                                                                                             
            uint blend = (uint)floorf(255.0f * patchLevel.blend);
                                                                                             
            const uint numGrids9x9 = 1<<(2*lod_level);
            //const uint offset = ((1<<(2*lod_level))-1)/(4-1);
            const uint offset = atomic_add_global(&local_lcgbp_scene->numCurrentLCGBPStates,numGrids9x9);
            uint index = 0;
            if (lod_level == 0)
            {
              local_lcgbp_scene->lcgbp_state[offset+index] = LCGBP_State(&current,0,0,4,index,lod_level,edgeLevels,blend);
              index++;
            }
            else if (lod_level == 1)
            {
              for (uint y=0;y<2;y++)
                for (uint x=0;x<2;x++)
                {
                  local_lcgbp_scene->lcgbp_state[offset+index] = LCGBP_State(&current,x*16,y*16,2,index,lod_level,edgeLevels,blend);
                  index++;
                }
            }
            else
            {
              for (uint y=0;y<4;y++)
                for (uint x=0;x<4;x++)
                {
                  local_lcgbp_scene->lcgbp_state[offset+index] = LCGBP_State(&current,x*8,y*8,1,index,lod_level,edgeLevels,blend);
                  index++;
                }
            }
          }
                                                                                           
        });
      });
      waitOnEventAndCatchException(compute_lod_event);
    }

    double t0 = getSeconds();
    
    rtcSetGeometryUserData(local_lcgbp_scene->geometry,lcg_ptr);

    rtcSetLCData(local_lcgbp_scene->geometry, local_lcgbp_scene->numCurrentLCGBPStates, local_lcgbp_scene->lcgbp_state, local_lcgbp_scene->numLCMeshClusters,local_lcgbp_scene->lcm_cluster);
    
    rtcCommitGeometry(local_lcgbp_scene->geometry);
    
    /* commit changes to scene */
    rtcCommitScene (data.g_scene);

    double dt0 = (getSeconds()-t0)*1000.0;
                                            
    avg_bvh_build_time.add(dt0);
#endif    
  }

/* called by the C++ code for cleanup */
  extern "C" void device_cleanup ()
  {
    TutorialData_Destructor(&data);
  }

  // =================================================================================================================================================================================
  // ======================================================================== Simple Path Tracer =====================================================================================
  // =================================================================================================================================================================================  

  Light_SampleRes Lights_sample(const Light* self,
                                const DifferentialGeometry& dg, /*! point to generate the sample for >*/
                                const Vec2f s)                /*! random numbers to generate the sample >*/
  {
    TutorialLightType ty = self->type;
    switch (ty) {
    case LIGHT_AMBIENT    : return AmbientLight_sample(self,dg,s);
    case LIGHT_POINT      : return PointLight_sample(self,dg,s);
    case LIGHT_DIRECTIONAL: return DirectionalLight_sample(self,dg,s);
    case LIGHT_SPOT       : return SpotLight_sample(self,dg,s);
    case LIGHT_QUAD       : return QuadLight_sample(self,dg,s);
    default: {
      Light_SampleRes res;
      res.weight = Vec3fa(0,0,0);
      res.dir = Vec3fa(0,0,0);
      res.dist = 0;
      res.pdf = inf;
      return res;
    }
    }
  }
  
  Light_EvalRes Lights_eval(const Light* self,
                            const DifferentialGeometry& dg,
                            const Vec3fa& dir)
  {
    TutorialLightType ty = self->type;
    switch (ty) {
    case LIGHT_AMBIENT     : return AmbientLight_eval(self,dg,dir);
    case LIGHT_POINT       : return PointLight_eval(self,dg,dir);
    case LIGHT_DIRECTIONAL : return DirectionalLight_eval(self,dg,dir);
    case LIGHT_SPOT        : return SpotLight_eval(self,dg,dir);
    case LIGHT_QUAD        : return QuadLight_eval(self,dg,dir);
    default: {
      Light_EvalRes res;
      res.value = Vec3fa(0,0,0);
      res.dist = inf;
      res.pdf = 0.f;
      return res;
    }
    }
  }

////////////////////////////////////////////////////////////////////////////////
//                                 BRDF                                       //
////////////////////////////////////////////////////////////////////////////////

  struct BRDF
  {
    float Ns;               /*< specular exponent */
    float Ni;               /*< optical density for the surface (index of refraction) */
    Vec3fa Ka;              /*< ambient reflectivity */
    Vec3fa Kd;              /*< diffuse reflectivity */
    Vec3fa Ks;              /*< specular reflectivity */
    Vec3fa Kt;              /*< transmission filter */
    float dummy[30];
  };

  struct Medium
  {
    Vec3fa transmission; //!< Transmissivity of medium.
    float eta;             //!< Refraction index of medium.
  };

  inline Medium make_Medium(const Vec3fa& transmission, const float eta)
  {
    Medium m;
    m.transmission = transmission;
    m.eta = eta;
    return m;
  }

  inline Medium make_Medium_Vacuum() {
    return make_Medium(Vec3fa((float)1.0f),1.0f);
  }

  inline bool eq(const Medium& a, const Medium& b) {
    return (a.eta == b.eta) && eq(a.transmission, b.transmission);
  }

  inline Vec3fa sample_component2(const Vec3fa& c0, const Sample3f& wi0, const Medium& medium0,
                                  const Vec3fa& c1, const Sample3f& wi1, const Medium& medium1,
                                  const Vec3fa& Lw, Sample3f& wi_o, Medium& medium_o, const float s)
  {
    const Vec3fa m0 = Lw*c0/wi0.pdf;
    const Vec3fa m1 = Lw*c1/wi1.pdf;

    const float C0 = wi0.pdf == 0.0f ? 0.0f : max(max(m0.x,m0.y),m0.z);
    const float C1 = wi1.pdf == 0.0f ? 0.0f : max(max(m1.x,m1.y),m1.z);
    const float C  = C0 + C1;

    if (C == 0.0f) {
      wi_o = make_Sample3f(Vec3fa(0,0,0),0);
      return Vec3fa(0,0,0);
    }

    const float CP0 = C0/C;
    const float CP1 = C1/C;
    if (s < CP0) {
      wi_o = make_Sample3f(wi0.v,wi0.pdf*CP0);
      medium_o = medium0; return c0;
    }
    else {
      wi_o = make_Sample3f(wi1.v,wi1.pdf*CP1);
      medium_o = medium1; return c1;
    }
  }



////////////////////////////////////////////////////////////////////////////////
//                          OBJ Material                                      //
////////////////////////////////////////////////////////////////////////////////

  void OBJMaterial__preprocess(ISPCOBJMaterial* material, BRDF& brdf, const Vec3fa& wo, const DifferentialGeometry& dg, const Medium& medium)
  {
    float d = material->d;
    if (material->map_d) d *= getTextureTexel1f(material->map_d,dg.u,dg.v);
    brdf.Ka = Vec3fa(material->Ka);
    //if (material->map_Ka) { brdf.Ka *= material->map_Ka->get(dg.st); }
    brdf.Kd = d * Vec3fa(material->Kd);
    //if (material->map_Kd) brdf.Kd = brdf.Kd * getTextureTexel3f(material->map_Kd,dg.u,dg.v);
    brdf.Ks = d * Vec3fa(material->Ks);
    //if (material->map_Ks) brdf.Ks *= material->map_Ks->get(dg.st);
    brdf.Ns = material->Ns;
    //if (material->map_Ns) { brdf.Ns *= material->map_Ns.get(dg.st); }
    brdf.Kt = (1.0f-d)*Vec3fa(material->Kt);
    brdf.Ni = material->Ni;
  }

  Vec3fa OBJMaterial__eval(ISPCOBJMaterial* material, const BRDF& brdf, const Vec3fa& wo, const DifferentialGeometry& dg, const Vec3fa& wi)
  {
    Vec3fa R = Vec3fa(0.0f);
    const float Md = max(max(brdf.Kd.x,brdf.Kd.y),brdf.Kd.z);
    const float Ms = max(max(brdf.Ks.x,brdf.Ks.y),brdf.Ks.z);
    const float Mt = max(max(brdf.Kt.x,brdf.Kt.y),brdf.Kt.z);
    if (Md > 0.0f) {
      R = R + (1.0f/float(M_PI)) * clamp(dot(wi,dg.Ns)) * brdf.Kd;
    }
    if (Ms > 0.0f) {
      const Sample3f refl = make_Sample3f(reflect(wo,dg.Ns),1.0f);
      if (dot(refl.v,wi) > 0.0f)
        R = R + (brdf.Ns+2) * float(one_over_two_pi) * powf(max(1e-10f,dot(refl.v,wi)),brdf.Ns) * clamp(dot(wi,dg.Ns)) * brdf.Ks;
    }
    if (Mt > 0.0f) {
    }
    return R;
  }

  Vec3fa OBJMaterial__sample(ISPCOBJMaterial* material, const BRDF& brdf, const Vec3fa& Lw, const Vec3fa& wo, const DifferentialGeometry& dg, Sample3f& wi_o, Medium& medium, const Vec2f& s)
  {
    Vec3fa cd = Vec3fa(0.0f);
    Sample3f wid = make_Sample3f(Vec3fa(0.0f),0.0f);
    if (max(max(brdf.Kd.x,brdf.Kd.y),brdf.Kd.z) > 0.0f) {
      wid = cosineSampleHemisphere(s.x,s.y,dg.Ns);
      cd = float(one_over_pi) * clamp(dot(wid.v,dg.Ns)) * brdf.Kd;
    }

    Vec3fa cs = Vec3fa(0.0f);
    Sample3f wis = make_Sample3f(Vec3fa(0.0f),0.0f);
    if (max(max(brdf.Ks.x,brdf.Ks.y),brdf.Ks.z) > 0.0f)
    {
      const Sample3f refl = make_Sample3f(reflect(wo,dg.Ns),1.0f);
      wis.v = powerCosineSampleHemisphere(brdf.Ns,s);
      wis.pdf = powerCosineSampleHemispherePDF(wis.v,brdf.Ns);
      wis.v = frame(refl.v) * wis.v;
      cs = (brdf.Ns+2) * float(one_over_two_pi) * powf(max(dot(refl.v,wis.v),1e-10f),brdf.Ns) * clamp(dot(wis.v,dg.Ns)) * brdf.Ks;
    }

    Vec3fa ct = Vec3fa(0.0f);
    Sample3f wit = make_Sample3f(Vec3fa(0.0f),0.0f);
    if (max(max(brdf.Kt.x,brdf.Kt.y),brdf.Kt.z) > 0.0f)
    {
      wit = make_Sample3f(neg(wo),1.0f);
      ct = brdf.Kt;
    }

    const Vec3fa md = Lw*cd/wid.pdf;
    const Vec3fa ms = Lw*cs/wis.pdf;
    const Vec3fa mt = Lw*ct/wit.pdf;

    const float Cd = wid.pdf == 0.0f ? 0.0f : max(max(md.x,md.y),md.z);
    const float Cs = wis.pdf == 0.0f ? 0.0f : max(max(ms.x,ms.y),ms.z);
    const float Ct = wit.pdf == 0.0f ? 0.0f : max(max(mt.x,mt.y),mt.z);
    const float C  = Cd + Cs + Ct;

    if (C == 0.0f) {
      wi_o = make_Sample3f(Vec3fa(0,0,0),0);
      return Vec3fa(0,0,0);
    }

    const float CPd = Cd/C;
    const float CPs = Cs/C;
    const float CPt = Ct/C;

    if (s.x < CPd) {
      wi_o = make_Sample3f(wid.v,wid.pdf*CPd);
      return cd;
    }
    else if (s.x < CPd + CPs)
    {
      wi_o = make_Sample3f(wis.v,wis.pdf*CPs);
      return cs;
    }
    else
    {
      wi_o = make_Sample3f(wit.v,wit.pdf*CPt);
      return ct;
    }
  }


////////////////////////////////////////////////////////////////////////////////
//                              Material                                      //
////////////////////////////////////////////////////////////////////////////////

  inline void Material__preprocess(ISPCMaterial** materials, unsigned int materialID, unsigned int numMaterials, BRDF& brdf, const Vec3fa& wo, const DifferentialGeometry& dg, const Medium& medium)
  {
    auto id = materialID;
    {
      if (id < numMaterials) // FIXME: workaround for ISPC bug, location reached with empty execution mask
      {
        ISPCMaterial* material = materials[id];

        switch (material->type) {
        case MATERIAL_OBJ  : OBJMaterial__preprocess  ((ISPCOBJMaterial*)  material,brdf,wo,dg,medium); break;
        default: break;
        }
      }
    }
  }

  inline Vec3fa Material__eval(ISPCMaterial** materials, unsigned int materialID, unsigned int numMaterials, const BRDF& brdf, const Vec3fa& wo, const DifferentialGeometry& dg, const Vec3fa& wi)
  {
    Vec3fa c = Vec3fa(0.0f);
    auto id = materialID;
    {
      if (id < numMaterials) // FIXME: workaround for ISPC bug, location reached with empty execution mask
      {
        ISPCMaterial* material = materials[id];
        switch (material->type) {
        case MATERIAL_OBJ  : c = OBJMaterial__eval  ((ISPCOBJMaterial*)  material, brdf, wo, dg, wi); break;
        default: c = Vec3fa(0.0f);
        }
      }
    }
    return c;
  }

  inline Vec3fa Material__sample(ISPCMaterial** materials, unsigned int materialID, unsigned int numMaterials, const BRDF& brdf, const Vec3fa& Lw, const Vec3fa& wo, const DifferentialGeometry& dg, Sample3f& wi_o, Medium& medium, const Vec2f& s)
  {
    Vec3fa c = Vec3fa(0.0f);
    auto id = materialID;
    {
      if (id < numMaterials) // FIXME: workaround for ISPC bug, location reached with empty execution mask
      {
        ISPCMaterial* material = materials[id];
        switch (material->type) {
        case MATERIAL_OBJ  : c = OBJMaterial__sample  ((ISPCOBJMaterial*)  material, brdf, Lw, wo, dg, wi_o, medium, s); break;
        default: wi_o = make_Sample3f(Vec3fa(0.0f),0.0f); c = Vec3fa(0.0f); break;
        }
      }
    }
    return c;
  }

    
  inline int postIntersect(const TutorialData& data, const Ray& ray, DifferentialGeometry& dg)
  {
    dg.eps = 32.0f*1.19209e-07f*max(max(abs(dg.P.x),abs(dg.P.y)),max(abs(dg.P.z),ray.tfar));   
    int materialID = 0;
    return materialID;
  }

  inline Vec3fa face_forward(const Vec3fa& dir, const Vec3fa& _Ng) {
    const Vec3fa Ng = _Ng;
    return dot(dir,Ng) < 0.0f ? Ng : neg(Ng);
  }


  Vec3fa renderPixelFunction(const TutorialData& data, float x, float y, RandomSampler& sampler, const ISPCCamera& camera, RayStats& stats, const RTCFeatureFlags features)
  {
    /* radiance accumulator and weight */
    Vec3fa L = Vec3fa(0.0f);
    Vec3fa Lw = Vec3fa(1.0f);
    Medium medium = make_Medium_Vacuum();
    float time = RandomSampler_get1D(sampler);

    /* initialize ray */
    Ray ray(Vec3fa(camera.xfm.p),
            Vec3fa(normalize(x*camera.xfm.l.vx + y*camera.xfm.l.vy + camera.xfm.l.vz)),0.0f,inf,time);

    DifferentialGeometry dg;
 
    /* iterative path tracer loop */
    for (int i=0; i<data.max_path_length; i++)
    {
      /* terminate if contribution too low */
      if (max(Lw.x,max(Lw.y,Lw.z)) < 0.01f)
        break;

      /* intersect ray with scene */
      RayQueryContext context;
      InitIntersectionContext(&context);
      context.tutorialData = (void*) &data;
    
      RTCIntersectArguments args;
      rtcInitIntersectArguments(&args);
      args.context = &context.context;
      args.feature_mask = features;
  
      rtcIntersect1(data.g_scene,RTCRayHit_(ray),&args);
      RayStats_addRay(stats);
      const Vec3fa wo = neg(ray.dir);

      /* invoke environment lights if nothing hit */
      if (ray.geomID == RTC_INVALID_GEOMETRY_ID)
      {
        //L = L + Lw*Vec3fa(1.0f);

        /* iterate over all lights */
        for (unsigned int i=0; i<data.ispc_scene->numLights; i++)
        {
          const Light* l = data.ispc_scene->lights[i];
          //Light_EvalRes le = l->eval(l,dg,ray.dir);
          Light_EvalRes le = Lights_eval(l,dg,ray.dir);
          L = L + Lw*le.value;
        }

        break;
      }

      Vec3fa Ns = normalize(ray.Ng);

      /* compute differential geometry */
    
      dg.geomID = ray.geomID;
      dg.primID = ray.primID;
      dg.u = ray.u;
      dg.v = ray.v;
      dg.P  = ray.org+ray.tfar*ray.dir;
      dg.Ng = ray.Ng;
      dg.Ns = Ns;
      int materialID = postIntersect(data,ray,dg);
      dg.Ng = face_forward(ray.dir,normalize(dg.Ng));
      dg.Ns = face_forward(ray.dir,normalize(dg.Ns));

      /*! Compute  simple volumetric effect. */
      Vec3fa c = Vec3fa(1.0f);
      const Vec3fa transmission = medium.transmission;
      if (ne(transmission,Vec3fa(1.0f)))
        c = c * pow(transmission,ray.tfar);

      /* calculate BRDF */
      BRDF brdf;
      int numMaterials = data.ispc_scene->numMaterials;
      ISPCMaterial** material_array = &data.ispc_scene->materials[0];
      Material__preprocess(material_array,materialID,numMaterials,brdf,wo,dg,medium);

      /* sample BRDF at hit point */
      Sample3f wi1;
      c = c * Material__sample(material_array,materialID,numMaterials,brdf,Lw, wo, dg, wi1, medium, RandomSampler_get2D(sampler));

      /* iterate over lights */
      for (unsigned int i=0; i<data.ispc_scene->numLights; i++)
      {
        const Light* l = data.ispc_scene->lights[i];
        //Light_SampleRes ls = l->sample(l,dg,RandomSampler_get2D(sampler));
        Light_SampleRes ls = Lights_sample(l,dg,RandomSampler_get2D(sampler));
        if (ls.pdf <= 0.0f) continue;
        Vec3fa transparency = Vec3fa(1.0f);
        Ray shadow(dg.P,ls.dir,dg.eps,ls.dist,time);
        context.userRayExt = &transparency;

        RTCOccludedArguments sargs;
        rtcInitOccludedArguments(&sargs);
        sargs.context = &context.context;
        sargs.feature_mask = features;
        rtcOccluded1(data.g_scene,RTCRay_(shadow),&sargs);
        RayStats_addShadowRay(stats);
        if (shadow.tfar > 0.0f)
          L = L + Lw*ls.weight*transparency*Material__eval(material_array,materialID,numMaterials,brdf,wo,dg,ls.dir);
      }

      if (wi1.pdf <= 1E-4f /* 0.0f */) break;
      Lw = Lw*c/wi1.pdf;

      /* setup secondary ray */
      float sign = dot(wi1.v,dg.Ng) < 0.0f ? -1.0f : 1.0f;
      dg.P = dg.P + sign*dg.eps*dg.Ng;
      init_Ray(ray, dg.P,normalize(wi1.v),dg.eps,inf,time);
    }
    return L;
  }



  void renderPixelPathTracer(const TutorialData& data,
                             int x, int y,
                             int* pixels,
                             const unsigned int width,
                             const unsigned int height,
                             const float time,
                             const ISPCCamera& camera,
                             RayStats& stats,
                             const RTCFeatureFlags features)
  {
    RandomSampler sampler;

    Vec3fa L = Vec3fa(0.0f);

    for (int i=0; i<data.spp; i++)
    {
      RandomSampler_init(sampler, x, y, data.spp+i);

      /* calculate pixel color */
      float fx = x + RandomSampler_get1D(sampler);
      float fy = y + RandomSampler_get1D(sampler);
      L = L + renderPixelFunction(data,fx,fy,sampler,camera,stats,features);
    }
    L = L/(float)data.spp;

    /* write color to framebuffer */
    //Vec3ff accu_color = data.accu[y*width+x] + Vec3ff(L.x,L.y,L.z,1.0f); data.accu[y*width+x] = accu_color;
    //float f = rcp(max(0.001f,accu_color.w));
    Vec3ff accu_color = Vec3ff(L.x,L.y,L.z,1.0f);
    float f = 1.0f;
    unsigned int r = (unsigned int) (255.01f * clamp(accu_color.x*f,0.0f,1.0f));
    unsigned int g = (unsigned int) (255.01f * clamp(accu_color.y*f,0.0f,1.0f));
    unsigned int b = (unsigned int) (255.01f * clamp(accu_color.z*f,0.0f,1.0f));
    pixels[y*width+x] = (b << 16) + (g << 8) + r;
  }
  

} // namespace embree
