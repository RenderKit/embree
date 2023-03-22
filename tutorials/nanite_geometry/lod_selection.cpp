#include "nanite_geometry_device.h"

#define FORCE_ROOT_LEVEL 0

namespace embree {

  extern "C" unsigned int g_lod_threshold;
  extern LCG_Scene *global_lcgbp_scene;
  extern TutorialData data;

  
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
  
  //static const unsigned int NUM_TOTAL_QUAD_NODES_PER_RTC_LCG = (1-(1<<(2*LCG_Scene::LOD_LEVELS)))/(1-4);

  struct LODPatchLevel
  {
    unsigned int level;
    float blend;

    __forceinline LODPatchLevel(const unsigned int level, const float blend) : level(level), blend(blend) {}
  };


  __forceinline LODPatchLevel getLODPatchLevel(const float MIN_LOD_DISTANCE,LCGBP &current,const ISPCCamera& camera, const unsigned int width, const unsigned int height)
  {
    const float minDistance = MIN_LOD_DISTANCE;
    const unsigned int startRange[LCG_Scene::LOD_LEVELS+1] = { 0,1,3,7};
    const unsigned int   endRange[LCG_Scene::LOD_LEVELS+1] = { 1,3,7,15};    
    
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
    for (unsigned int i=0;i<LCG_Scene::LOD_LEVELS;i++)
      if (startRange[i] <= dist_level && dist_level < endRange[i])
      {          
        segment = i;
        break;
      }
    float blend = 0.0f;
    if (segment == -1)
      segment = LCG_Scene::LOD_LEVELS-1;
    else if (segment != 0)
    {
      blend = min((dist_minDistance-startRange[segment])/(endRange[segment]-startRange[segment]),1.0f);
      segment--;
    }    
    return LODPatchLevel(LCG_Scene::LOD_LEVELS-1-segment,blend);    
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

  __forceinline Vec2f projectVertexToPlane(const Vec3f &p, const Vec3f &vx, const Vec3f &vy, const Vec3f &vz)
  {
    const Vec3f vn = cross(vx,vy);    
    const float distance = (float)dot(vn,vz) / (float)dot(vn,p);
    Vec3f pip = p * distance;
    if (distance < 0.0f)
      pip = vz;
    const float a = dot((pip-vz),vx);
    const float b = dot((pip-vz),vy);
    return Vec2f(a,b);    
  }
  

  __forceinline Vec2f projectBBox3fToPlane(const BBox3f &bounds, const Vec3f &vx, const Vec3f &vy, const Vec3f &vz, const unsigned int width, const unsigned int height, const bool clip=true)
  {
    const Vec3f v0(bounds.lower.x,bounds.lower.y,bounds.lower.z);
    const Vec3f v1(bounds.upper.x,bounds.lower.y,bounds.lower.z);
    const Vec3f v2(bounds.lower.x,bounds.upper.y,bounds.lower.z);
    const Vec3f v3(bounds.upper.x,bounds.upper.y,bounds.lower.z);
    const Vec3f v4(bounds.lower.x,bounds.lower.y,bounds.upper.z);
    const Vec3f v5(bounds.upper.x,bounds.lower.y,bounds.upper.z);
    const Vec3f v6(bounds.lower.x,bounds.upper.y,bounds.upper.z);
    const Vec3f v7(bounds.upper.x,bounds.upper.y,bounds.upper.z);
    
    const Vec2f p0 = projectVertexToPlane(v0,vx,vy,vz);
    const Vec2f p1 = projectVertexToPlane(v1,vx,vy,vz);
    const Vec2f p2 = projectVertexToPlane(v2,vx,vy,vz);
    const Vec2f p3 = projectVertexToPlane(v3,vx,vy,vz);
    const Vec2f p4 = projectVertexToPlane(v4,vx,vy,vz);
    const Vec2f p5 = projectVertexToPlane(v5,vx,vy,vz);
    const Vec2f p6 = projectVertexToPlane(v6,vx,vy,vz);
    const Vec2f p7 = projectVertexToPlane(v7,vx,vy,vz);

    BBox2f bounds2D(empty);
    bounds2D.extend(p0);
    bounds2D.extend(p1);
    bounds2D.extend(p2);
    bounds2D.extend(p3);
    bounds2D.extend(p4);
    bounds2D.extend(p5);
    bounds2D.extend(p6);
    bounds2D.extend(p7);

    BBox2f image2D(Vec2f(0,0),Vec2f(width,height));

    if (clip)
    {
      bounds2D = intersect(bounds2D,image2D);
    }
    return bounds2D.size();
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
    
    i0 = min(max(0,i0),(int)LCG_Scene::LOD_LEVELS-1);
    i1 = min(max(0,i1),(int)LCG_Scene::LOD_LEVELS-1);
    i2 = min(max(0,i2),(int)LCG_Scene::LOD_LEVELS-1);
    i3 = min(max(0,i3),(int)LCG_Scene::LOD_LEVELS-1);

#if 0
    i0 = i1 = i2 = i3 = 2;
#endif    
    LODEdgeLevel lod_levels(i0,i1,i2,i3);
    return lod_levels;
  }

  __forceinline bool subdivideLOD(const Vec2f &lower, const Vec2f &upper, const float THRESHOLD)
  {
    bool subdivide = true;
    const float l = length(upper - lower); //FIXME ^2   
    if (l <= THRESHOLD) subdivide = false; 
    return subdivide;
  }

  __forceinline bool subdivideLOD(const BBox3f &bounds, const Vec3f &vx, const Vec3f &vy, const Vec3f &vz, const uint width, const uint height, const float THRESHOLD)
  {
#if 1
    const Vec2f diag = projectBBox3fToPlane( bounds, vx,vy,vz, width,height,true);
    const float l = dot(diag,diag); // length^2
    if (l <= THRESHOLD*THRESHOLD) return false;     
#else
    const Vec3f c = bounds.center();
    const Vec3f delta = bounds.size(); // diagonal of AABB;
    const float S0 = delta.y * delta.z;
    const float S1 = delta.x * delta.z;
    const float S2 = delta.x * delta.y;
    const Vec3f abs_c = abs(c);
    const float d = sqrt(dot(c,c)); // ||p-c||
    const float scale = 1.0f / (d*d*d);
    const float l = scale * (abs_c.x * S0 + abs_c.y * S1 + abs_c.z * S2);
    if (l <= THRESHOLD) return false;         
#endif    
    return true;
  }
  


  void select_clusters_lod_grid_tree(LCG_Scene *local_lcgbp_scene,
                                     const unsigned int width,
                                     const unsigned int height,
                                     const ISPCCamera& camera)
  {

    const unsigned int wgSize = 16*1;
    const unsigned int numLCGBP = local_lcgbp_scene->numLCGBP;
    //const unsigned int numLCMeshClusters = local_lcgbp_scene->numLCMeshClusters;
    sycl::event init_event =  global_gpu_queue->submit([&](sycl::handler &cgh) {
      cgh.single_task([=]() {
        local_lcgbp_scene->numCurrentLCGBPStates = 0;
      });
    });

    waitOnEventAndCatchException(init_event);

    //void *lcg_ptr = nullptr;
    //unsigned int lcg_num_prims = 0;
    
    const float minLODDistance = local_lcgbp_scene->minLODDistance;
      
    const sycl::nd_range<1> nd_range1(alignTo(numLCGBP,wgSize),sycl::range<1>(wgSize));              
    sycl::event compute_lod_event = global_gpu_queue->submit([=](sycl::handler& cgh){
      cgh.depends_on(init_event);                                                   
      cgh.parallel_for(nd_range1,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(16) {
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
          const unsigned int offset = gpu::atomic_add_global(&local_lcgbp_scene->numCurrentLCGBPStates,numGrids9x9);
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
    
  }

  __forceinline uint writeSubgroup(uint *dest, const uint value, const bool cond)
  {
    const uint count = cond ? 1 : 0;
    const uint exclusive_scan  = sub_group_exclusive_scan(count, std::plus<uint>());
    const uint reduction       = sub_group_reduce(count, std::plus<uint>());
    dest[exclusive_scan] = value;
    sub_group_barrier();                                       
    return reduction;
  }
  
  void select_clusters_lod_mesh_dag(LCG_Scene *local_lcgbp_scene,
                                    const unsigned int width,
                                    const unsigned int height,
                                    const ISPCCamera& camera)
  {
    const unsigned int wgSize = 16*1;

    const unsigned int numLCMeshClusters = local_lcgbp_scene->numLCMeshClusters;
    const unsigned int numRootsTotal = local_lcgbp_scene->numLCMeshClusterRoots;
    uchar *const active_state = local_lcgbp_scene->lcm_cluster_active_state_per_frame;      

    // ================================================================================================================================
    // ================================================================================================================================
    // ================================================================================================================================
    
    sycl::event init_event =  global_gpu_queue->submit([&](sycl::handler &cgh) {
      cgh.single_task([=]() {
        local_lcgbp_scene->numLCMeshClusterRootsPerFrame = 0;
        local_lcgbp_scene->numLCMeshClusterQuadsPerFrame = 0;
        local_lcgbp_scene->numLCMeshClusterBlocksPerFrame = 0;
      });
    });

    // ================================================================================================================================
    // ================================================================================================================================
    // ================================================================================================================================
    

    sycl::event memset_event = global_gpu_queue->memset(active_state,0,numLCMeshClusters);
    //waitOnEventAndCatchException(memset_event);
          
    // ================================================================================================================================
    // ================================================================================================================================
    // ================================================================================================================================

    const float lod_threshold = g_lod_threshold;

#if 1

    const uint wgSizeComputeLOD = 16;
    const sycl::nd_range<1> nd_range1(alignTo(numRootsTotal,wgSizeComputeLOD),sycl::range<1>(wgSize));
    sycl::event compute_lod_event = global_gpu_queue->submit([=](sycl::handler& cgh){
      cgh.depends_on(memset_event);        
      cgh.depends_on(init_event);
      sycl::local_accessor< uint, 1> _localIDs(sycl::range<1>(256),cgh);
      
      cgh.parallel_for(nd_range1,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(16) {
        const unsigned int i = item.get_global_id(0);
        const uint subgroupLocalID = get_sub_group_local_id();
        const uint subgroupSize    = get_sub_group_size();            

        uint *const localIDs = _localIDs.get_pointer();
        
        uint clusterID = -1;

        const Vec3f org = camera.xfm.p;
        const Vec3f vx = camera.xfm.l.vx;
        const Vec3f vy = camera.xfm.l.vy;
        const Vec3f vz = camera.xfm.l.vz;

#if FORCE_ROOT_LEVEL == 1
        if (i < local_lcgbp_scene->numLCMeshClusterRoots)        
        {
          clusterID = local_lcgbp_scene->lcm_cluster_roots_IDs[i];            
          const LossyCompressedMeshCluster &root_cluster = local_lcgbp_scene->lcm_cluster[ clusterID ];
          LossyCompressedMesh *mesh = root_cluster.mesh;
          active_state[clusterID] = 1;                
          if (root_cluster.hasNeighbor()) active_state[root_cluster.neighborID] = 1;              
        }
#else        
        if (i < local_lcgbp_scene->numLCMeshClusterRoots)
        {
          clusterID = local_lcgbp_scene->lcm_cluster_roots_IDs[i];            
          const LossyCompressedMeshCluster &root_cluster = local_lcgbp_scene->lcm_cluster[ clusterID ];

          LossyCompressedMesh *mesh = root_cluster.mesh;
          const Vec3f lower = mesh->bounds.lower;
          const Vec3f diag = mesh->bounds.size() * (1.0f / CompressedVertex::RES_PER_DIM);

          const bool cull = frustumCull( root_cluster.bounds.lower.decompress(lower,diag)-org,root_cluster.bounds.upper.decompress(lower,diag)-org,vx*width,vy*height,vz);
          if (cull)
          {
            active_state[clusterID] = 1;                
            if (root_cluster.hasNeighbor()) active_state[root_cluster.neighborID] = 1;
            clusterID = -1;
          }          
        }

        int numIDs = writeSubgroup(localIDs,clusterID,clusterID!=-1);
        
        while(numIDs)
        {
          const int cur_startID = std::max(numIDs-(int)subgroupSize,0);
          const int cur_numIDs = numIDs-cur_startID;
          const int cur_index = std::min(cur_startID+(int)subgroupLocalID,numIDs-1);
          const uint currentID = localIDs[cur_index];
          const bool active = (cur_startID + subgroupLocalID) < numIDs;
          numIDs -= cur_numIDs;
          
          bool write = false;
          const LossyCompressedMeshCluster &cur = local_lcgbp_scene->lcm_cluster[ currentID ];              
          
          if (active)
          {
            LossyCompressedMesh *mesh = cur.mesh;
            const Vec3f lower = mesh->bounds.lower;
            const Vec3f diag = mesh->bounds.size() * (1.0f / CompressedVertex::RES_PER_DIM);
            
            const Vec3f bounds_lower = cur.bounds.lower.decompress(lower,diag)-org;
            const Vec3f bounds_upper = cur.bounds.upper.decompress(lower,diag)-org;
            const bool subdivide = subdivideLOD(BBox3f(bounds_lower,bounds_upper),vx,vy,vz,width,height,lod_threshold);
            if (subdivide && cur.hasChildren())
              write = true;
            //write |= cur.hasChildren();
          }
          
          if (!write && active)
          {
            active_state[currentID] = 1;                
            if (cur.hasNeighbor()) active_state[cur.neighborID] = 1;
          }                        

          numIDs += writeSubgroup(&localIDs[numIDs],cur.leftID,write);          
          numIDs += writeSubgroup(&localIDs[numIDs],cur.rightID,write);
        }
#endif        
      });
    });


#else    
    const uint wgSizeComputeLOD = 16;
    const sycl::nd_range<1> nd_range1(alignTo(numRootsTotal,wgSizeComputeLOD),sycl::range<1>(wgSize));              
    sycl::event compute_lod_event = global_gpu_queue->submit([=](sycl::handler& cgh){
      cgh.depends_on(memset_event);        
      cgh.depends_on(init_event);        
      cgh.parallel_for(nd_range1,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(16) {
        const unsigned int i = item.get_global_id(0);
                              
        if (i < local_lcgbp_scene->numLCMeshClusterRoots)
        {
          const unsigned int clusterID = local_lcgbp_scene->lcm_cluster_roots_IDs[i];            
#if FORCE_ROOT_LEVEL == 0          
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
            active_state[clusterID] = 1;                
            if (root_cluster.hasNeighbor()) active_state[root_cluster.neighborID] = 1;
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

              const Vec3f bounds_lower = cur.bounds.lower.decompress(lower,diag)-org;
              const Vec3f bounds_upper = cur.bounds.upper.decompress(lower,diag)-org;
              bool subdivide = subdivideLOD(BBox3f(bounds_lower,bounds_upper),vx,vy,vz,width,height,lod_threshold);
              if (subdivide && cur.hasChildren() && (numStackEntries+2 <= STACK_SIZE))
              {
                stack[numStackEntries+0] = cur.leftID;
                stack[numStackEntries+1] = cur.rightID;
                numStackEntries+=2;
              }                    
              else
              {
                active_state[currentID] = 1;                
                if (cur.hasNeighbor()) active_state[cur.neighborID] = 1;
              }              
            }
          }
#else              
          {
            const LossyCompressedMeshCluster &cluster = local_lcgbp_scene->lcm_cluster[ clusterID ];
            LossyCompressedMesh *mesh = cluster.mesh;
            active_state[clusterID] = 1;                
            if (root_cluster.hasNeighbor()) active_state[root_cluster.neighborID] = 1;              
          }
#endif            
        }
      });
    });
#endif    

    // ================================================================================================================================
    // ================================================================================================================================
    // ================================================================================================================================
    

    const sycl::nd_range<1> nd_range2(alignTo(numLCMeshClusters,wgSize),sycl::range<1>(wgSize));              
    sycl::event select_clusterIDs_event = global_gpu_queue->submit([=](sycl::handler& cgh){
      sycl::local_accessor< uint      ,  0> _cluster_counter(cgh);
      sycl::local_accessor< uint      ,  0> _quad_counter(cgh);
      sycl::local_accessor< uint      ,  0> _block_counter(cgh);

      cgh.depends_on(compute_lod_event);        
        
      cgh.parallel_for(nd_range2,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(16) {
        const unsigned int i = item.get_global_id(0);
        uint &cluster_counter    = *_cluster_counter.get_pointer();
        uint &quad_counter       = *_quad_counter.get_pointer();
        uint &block_counter      = *_block_counter.get_pointer();

        cluster_counter = 0;
        quad_counter = 0;
        block_counter = 0;

        item.barrier(sycl::access::fence_space::local_space);
                              
        if (i < local_lcgbp_scene->numLCMeshClusters)
        {
          if (active_state[i])
          {
            const unsigned int destID = gpu::atomic_add_global(&local_lcgbp_scene->numLCMeshClusterRootsPerFrame,(unsigned int)1);
            local_lcgbp_scene->lcm_cluster_roots_IDs_per_frame[destID] = i;            
            const LossyCompressedMeshCluster &cur = local_lcgbp_scene->lcm_cluster[ i ];              
            gpu::atomic_add_local(&quad_counter,(unsigned int)cur.numQuads);
            gpu::atomic_add_local(&block_counter,(unsigned int)cur.numBlocks);                                                      
          }
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
    waitOnEventAndCatchException(select_clusterIDs_event);
    
    //PRINT3(gpu::getDeviceExecutionTiming(memset_event),gpu::getDeviceExecutionTiming(compute_lod_event),gpu::getDeviceExecutionTiming(select_clusterIDs_event));
  }


  void select_clusters_lod_mesh_tree(LCG_Scene *local_lcgbp_scene,
                                     const unsigned int width,
                                     const unsigned int height,
                                     const ISPCCamera& camera)
  {
    const unsigned int wgSize = 16*1;
    
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

      cgh.depends_on(init_event);
        
      cgh.parallel_for(nd_range1,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(16) {
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

              const Vec3f bounds_lower = cur.bounds.lower.decompress(lower,diag)-org;
              const Vec3f bounds_upper = cur.bounds.upper.decompress(lower,diag)-org;

              bool subdivide = subdivideLOD(BBox3f(bounds_lower,bounds_upper),vx,vy,vz,width,height,lod_threshold);
              if (subdivide && cur.hasChildren() && (numStackEntries+2 <= STACK_SIZE))
              {
                const uint lID = cur.leftID;
                const uint rID = cur.rightID;
                stack[numStackEntries+0] = lID;
                stack[numStackEntries+1] = rID;
                numStackEntries+=2;
              }                
              else
              {
                gpu::atomic_add_local(&quad_counter,(unsigned int)cur.numQuads);
                gpu::atomic_add_local(&block_counter,(unsigned int)cur.numBlocks);                                          
                const unsigned int destID = gpu::atomic_add_global(&local_lcgbp_scene->numLCMeshClusterRootsPerFrame,(unsigned int)1);
                local_lcgbp_scene->lcm_cluster_roots_IDs_per_frame[destID] =  currentID;
              }              
            }
          }
#else              
          {
            const LossyCompressedMeshCluster &cluster = local_lcgbp_scene->lcm_cluster[ clusterID ];
            LossyCompressedMesh *mesh = cluster.mesh;              
            gpu::atomic_add_local(&quad_counter,(unsigned int)cluster.numQuads);
            gpu::atomic_add_local(&block_counter,(unsigned int)cluster.numBlocks);                
            const unsigned int destID = gpu::atomic_add_global(&local_lcgbp_scene->numLCMeshClusterRootsPerFrame,(unsigned int)1);                            
            local_lcgbp_scene->lcm_cluster_roots_IDs_per_frame[destID] = clusterID;
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
    
  }


  extern "C" bool device_pick(const float x, const float y, const ISPCCamera& camera, Vec3fa& hitPos)
  {
    LCG_Scene *lcgbp_scene = global_lcgbp_scene;

    TutorialData ldata = data;
    sycl::event event = global_gpu_queue->submit([=](sycl::handler& cgh){
      const sycl::nd_range<2> nd_range = make_nd_range(1,1);
      cgh.parallel_for(nd_range,[=](sycl::nd_item<2> item) EMBREE_SYCL_SIMD(16) {

        RTCIntersectArguments args;
        rtcInitIntersectArguments(&args);
        args.feature_mask = (RTCFeatureFlags) (FEATURE_MASK);
  
        /* initialize ray */
        const Vec3fa org = Vec3fa(camera.xfm.p);
        const Vec3fa dir = Vec3fa(normalize(x*camera.xfm.l.vx + y*camera.xfm.l.vy + camera.xfm.l.vz));
        Ray ray(org, dir, 0.0f, inf);

        /* intersect ray with scene */
        rtcIntersect1(ldata.g_scene,RTCRayHit_(ray),&args);

        if (ray.geomID != RTC_INVALID_GEOMETRY_ID)
        {
          lcgbp_scene->pick_primID = ray.primID;
          lcgbp_scene->pick_geomID = ray.geomID;
          lcgbp_scene->pick_pos    = org + ray.tfar * dir;
        }
        else
        {
          lcgbp_scene->pick_primID = -1;
          lcgbp_scene->pick_geomID = -1;
          lcgbp_scene->pick_pos    = Vec3fa(0,0,0);          
        }
      });
    });
    gpu::waitOnQueueAndCatchException(*global_gpu_queue);        

    hitPos = lcgbp_scene->pick_pos;    
    const bool hit = lcgbp_scene->pick_primID != -1;
    //const float lod_threshold = g_lod_threshold;
    if (hit)
    {
      PRINT2(lcgbp_scene->pick_primID,lcgbp_scene->pick_geomID);
      PRINT(g_lod_threshold);
      const Vec3f org = camera.xfm.p;
      const Vec3f vx = camera.xfm.l.vx;
      const Vec3f vy = camera.xfm.l.vy;
      const Vec3f vz = camera.xfm.l.vz;
      
      const uint clusterID = lcgbp_scene->pick_primID;
      const LossyCompressedMeshCluster &cluster = lcgbp_scene->lcm_cluster[ clusterID ];
      PRINT3((int)cluster.numQuads,(int)cluster.numBlocks,(int)cluster.lod_level);
      PRINT3((int)cluster.leftID,(int)cluster.rightID,(int)cluster.neighborID);

      LossyCompressedMesh *mesh = cluster.mesh;
      const Vec3f lower = mesh->bounds.lower;
      const Vec3f diag = mesh->bounds.size() * (1.0f / CompressedVertex::RES_PER_DIM);
      // const uint width = 1024;
      // const uint height = 1024;
      const BBox3f cluster_bounds(cluster.bounds.lower.decompress(lower,diag),cluster.bounds.upper.decompress(lower,diag));
      PRINT2(cluster_bounds,area(cluster_bounds));
      // const Vec2f diag2 = projectBBox3fToPlane( BBox3f(cluster_bounds.lower-org,cluster_bounds.upper-org), vx,vy,vz, width,height,true);
      // PRINT3(diag2,length(diag2),length(diag2)<lod_threshold);
      // bool subdivide = subdivideLOD(BBox3f(cluster_bounds.lower-org,cluster_bounds.upper-org),vx,vy,vz, width,height,lod_threshold);
      // PRINT(subdivide);
    }

                
    return hit;
  }
  
  
};
