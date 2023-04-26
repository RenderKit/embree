#include "nanite_geometry_device.h"

#define FORCE_ROOT_LEVEL 0

namespace sycl {
  template<> struct sycl::is_device_copyable<embree::Vec3f> : std::true_type {};
}

namespace embree {
  extern "C" unsigned int g_lod_threshold;
  extern LCG_Scene *global_lcgbp_scene;
  extern TutorialData data;

  class BezierBasis
  {
  public:

    template<typename T>
      static __forceinline Vec4<T> eval(const T& u) 
    {
      const T t1 = u;
      const T t0 = 1.0f-t1;
      const T B0 = t0 * t0 * t0;
      const T B1 = 3.0f * t1 * (t0 * t0);
      const T B2 = 3.0f * (t1 * t1) * t0;
      const T B3 = t1 * t1 * t1;
      return Vec4<T>(B0,B1,B2,B3);
    }
    
    template<typename T>
      static __forceinline Vec4<T>  derivative(const T& u)
    {
      const T t1 = u;
      const T t0 = 1.0f-t1;
      const T B0 = -(t0*t0);
      const T B1 = madd(-2.0f,t0*t1,t0*t0);
      const T B2 = msub(+2.0f,t0*t1,t1*t1);
      const T B3 = +(t1*t1);
      return T(3.0f)*Vec4<T>(B0,B1,B2,B3);
    }

    template<typename T>
      static __forceinline Vec4<T>  derivative2(const T& u)
    {
      const T t1 = u;
      const T t0 = 1.0f-t1;
      const T B0 = t0;
      const T B1 = madd(-2.0f,t0,t1);
      const T B2 = madd(-2.0f,t1,t0);
      const T B3 = t1;
      return T(6.0f)*Vec4<T>(B0,B1,B2,B3);
    }
  };  

  class BSplineBasis
  {
  public:

    template<typename T>
      static __forceinline Vec4<T> eval(const T& u) 
    {
      const T t  = u;
      const T s  = T(1.0f) - u;
      const T n0 = s*s*s;
      const T n1 = (4.0f*(s*s*s)+(t*t*t)) + (12.0f*((s*t)*s) + 6.0f*((t*s)*t));
      const T n2 = (4.0f*(t*t*t)+(s*s*s)) + (12.0f*((t*s)*t) + 6.0f*((s*t)*s));
      const T n3 = t*t*t;
      return T(1.0f/6.0f)*Vec4<T>(n0,n1,n2,n3);
    }
    
    template<typename T>
      static __forceinline Vec4<T>  derivative(const T& u)
    {
      const T t  =  u;
      const T s  =  1.0f - u;
      const T n0 = -s*s;
      const T n1 = -t*t - 4.0f*(t*s);
      const T n2 =  s*s + 4.0f*(s*t);
      const T n3 =  t*t;
      return T(0.5f)*Vec4<T>(n0,n1,n2,n3);
    }

    template<typename T>
      static __forceinline Vec4<T>  derivative2(const T& u)
    {
      const T t  =  u;
      const T s  =  1.0f - u;
      const T n0 = s;
      const T n1 = t - 2.0f*s;
      const T n2 = s - 2.0f*t;
      const T n3 = t;
      return Vec4<T>(n0,n1,n2,n3);
    }
  };

  __forceinline Vec3f bilinear(const Vec4f Bu, const Vec3f matrix[4][4], const Vec4f Bv)
  {
    const Vec3f M0 = madd(Bu.x,matrix[0][0],madd(Bu.y,matrix[0][1],madd(Bu.z,matrix[0][2],Bu.w * matrix[0][3]))); 
    const Vec3f M1 = madd(Bu.x,matrix[1][0],madd(Bu.y,matrix[1][1],madd(Bu.z,matrix[1][2],Bu.w * matrix[1][3])));
    const Vec3f M2 = madd(Bu.x,matrix[2][0],madd(Bu.y,matrix[2][1],madd(Bu.z,matrix[2][2],Bu.w * matrix[2][3])));
    const Vec3f M3 = madd(Bu.x,matrix[3][0],madd(Bu.y,matrix[3][1],madd(Bu.z,matrix[3][2],Bu.w * matrix[3][3])));
    return madd(Bv.x,M0,madd(Bv.y,M1,madd(Bv.z,M2,Bv.w*M3)));
  }
  
  __forceinline Vec3f evalPatch(const Patch& patch, const float uu, const float vv)
  {
#if 0    
    const Vec4f v_n = BSplineBasis::eval(vv);
    const Vec3f curve0 = madd(v_n[0],patch.v[0][0],madd(v_n[1],patch.v[1][0],madd(v_n[2],patch.v[2][0],v_n[3] * patch.v[3][0])));
    const Vec3f curve1 = madd(v_n[0],patch.v[0][1],madd(v_n[1],patch.v[1][1],madd(v_n[2],patch.v[2][1],v_n[3] * patch.v[3][1])));
    const Vec3f curve2 = madd(v_n[0],patch.v[0][2],madd(v_n[1],patch.v[1][2],madd(v_n[2],patch.v[2][2],v_n[3] * patch.v[3][2])));
    const Vec3f curve3 = madd(v_n[0],patch.v[0][3],madd(v_n[1],patch.v[1][3],madd(v_n[2],patch.v[2][3],v_n[3] * patch.v[3][3])));    
    const Vec4f u_n = BSplineBasis::eval(uu);
    return madd(u_n[0],curve0,madd(u_n[1],curve1,madd(u_n[2],curve2,u_n[3] * curve3)));
#else
    const Vec4f Bu = BezierBasis::eval(uu);
    const Vec4f Bv = BezierBasis::eval(vv);
    return bilinear(Bu,patch.v,Bv);    
#endif    
  }

  void select_clusters_lod_patches(LCG_Scene *local_lcgbp_scene,
                                   const unsigned int width,
                                   const unsigned int height,
                                   const ISPCCamera* const _camera)
  {
    const uint numSubdivPatches = local_lcgbp_scene->numSubdivPatches;    
    const BBox3f geometryBounds   = local_lcgbp_scene->patch_mesh->bounds;
    const Vec3f geometry_lower    = geometryBounds.lower;
    const Vec3f geometry_diag     = geometryBounds.size();
    const Vec3f geometry_inv_diag = geometry_diag != Vec3fa(0.0f) ? Vec3fa(1.0f) / geometry_diag : Vec3fa(0.0f);

#if 0    
    local_lcgbp_scene->numLCMeshClusterRootsPerFrame = local_lcgbp_scene->numSubdivPatches;
    local_lcgbp_scene->patch_mesh->numVertices = 0;
    local_lcgbp_scene->patch_mesh->numQuads = 0;
#else
     sycl::event init_event =  global_gpu_queue->submit([&](sycl::handler &cgh) {
      cgh.single_task([=]() {
        local_lcgbp_scene->numLCMeshClusterRootsPerFrame = local_lcgbp_scene->numSubdivPatches;
        local_lcgbp_scene->patch_mesh->numVertices = 0;
        local_lcgbp_scene->patch_mesh->numQuads = 0;
      });
    });

    waitOnEventAndCatchException(init_event);   
#endif    


    const unsigned int wgSize = 256;
    
    //for (uint i=0;i<local_lcgbp_scene->numSubdivPatches;i++)
    const sycl::nd_range<1> nd_range1(alignTo(numSubdivPatches,wgSize),sycl::range<1>(wgSize));              
    sycl::event compute_lod_event = global_gpu_queue->submit([=](sycl::handler& cgh){
      cgh.depends_on(init_event);                                                   
      cgh.parallel_for(nd_range1,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(16) {
          const unsigned int i = item.get_global_id(0);
          if (i < numSubdivPatches)
          {
            const Patch &patch = local_lcgbp_scene->patches[i];

            //PRINT5(i,patch.v[1][1],patch.v[1][2],patch.v[2][2],patch.v[2][1]);

            const unsigned int type = patch.type;
            //if (type != 0) FATAL("TYPE");
            const unsigned int numVertices = (type == Patch::BEZIER) ? 16 : 4;
            const unsigned int numQuads = (type == Patch::BEZIER) ? 9 : 1;
      
            const unsigned int offsetVertex = gpu::atomic_add_global(&local_lcgbp_scene->patch_mesh->numVertices,numVertices);
            const unsigned int offsetQuad   = gpu::atomic_add_global(&local_lcgbp_scene->patch_mesh->numQuads,numQuads);
            
            //local_lcgbp_scene->patch_mesh->numVertices += numVertices;
            //local_lcgbp_scene->patch_mesh->numQuads += numQuads;

            LossyCompressedMeshCluster &cluster = local_lcgbp_scene->lcm_cluster[i];
      
            CompressedVertex *const cv = &local_lcgbp_scene->patch_mesh->compressedVertices[offsetVertex];
            CompressedQuadIndices *const cq = &local_lcgbp_scene->patch_mesh->compressedIndices[offsetQuad];

            CompressedAABB3f bounds;
            bounds.init();

#if 1            
            if (type == Patch::BEZIER)
            {
              for (int y=0;y<4;y++)
                for (int x=0;x<4;x++)
                {
                  const float p_u = (float)x / 3;
                  const float p_v = (float)y / 3;
                  const Vec3f vtx = evalPatch(patch,p_u,p_v);
                  const CompressedVertex c_vtx = CompressedVertex(vtx,geometry_lower,geometry_inv_diag);
                  cv[y*4+x] = c_vtx;
                  bounds.extend(c_vtx);
                }

              for (int y=0;y<3;y++)
                for (int x=0;x<3;x++)
                {
                  const unsigned int v0 = (y+0)*4+(x+0);
                  const unsigned int v1 = (y+0)*4+(x+1);
                  const unsigned int v2 = (y+1)*4+(x+1);
                  const unsigned int v3 = (y+1)*4+(x+0);          
                  cq[y*3+x] = CompressedQuadIndices(v0,v1,v2,v3);
                }
            }
            else
#endif              
            {
              cv[0] = CompressedVertex(patch.v[1][1],geometry_lower,geometry_inv_diag);
              cv[1] = CompressedVertex(patch.v[1][2],geometry_lower,geometry_inv_diag);
              cv[2] = CompressedVertex(patch.v[2][2],geometry_lower,geometry_inv_diag);
              cv[3] = CompressedVertex(patch.v[2][1],geometry_lower,geometry_inv_diag);
              bounds.extend(cv[0]);
              bounds.extend(cv[1]);
              bounds.extend(cv[2]);
              bounds.extend(cv[3]);        
              cq[0] = CompressedQuadIndices(0,1,2,3);        
            }
            
            cluster.numQuads = type == Patch::BEZIER ? 9 : 1;
            cluster.numBlocks = LossyCompressedMeshCluster::getDecompressedSizeInBytes(cluster.numQuads)/64;
            cluster.lod_level = 0;
            cluster.tmp = 0;
            cluster.bounds = bounds;      
            cluster.mesh = local_lcgbp_scene->patch_mesh;
            cluster.leftID = -1;
            cluster.rightID = -1;
            cluster.neighborID = -1;
            cluster.offsetVertices = offsetVertex;
            cluster.offsetIndices  = offsetQuad;
          }
        });
    });
    waitOnEventAndCatchException(compute_lod_event);

    //PRINT(local_lcgbp_scene->patch_mesh->numVertices);
    //PRINT(local_lcgbp_scene->patch_mesh->numQuads);

    //gpu::waitOnQueueAndCatchException(*global_gpu_queue);        
    
  }  
  
  
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
                                     const ISPCCamera* const _camera)
  {
    const unsigned int wgSize = 16*1;
    const unsigned int numLCGBP = local_lcgbp_scene->numLCGBP;
    const float minLODDistance = local_lcgbp_scene->minLODDistance;    
    //const unsigned int numLCMeshClusters = local_lcgbp_scene->numLCMeshClusters;
    sycl::event init_event =  global_gpu_queue->submit([&](sycl::handler &cgh) {
      cgh.single_task([=]() {
        local_lcgbp_scene->numCurrentLCGBPStates = 0;
      });
    });

    waitOnEventAndCatchException(init_event);

    //void *lcg_ptr = nullptr;
    //unsigned int lcg_num_prims = 0;
      
    const sycl::nd_range<1> nd_range1(alignTo(numLCGBP,wgSize),sycl::range<1>(wgSize));              
    sycl::event compute_lod_event = global_gpu_queue->submit([=](sycl::handler& cgh){
      cgh.depends_on(init_event);                                                   
      cgh.parallel_for(nd_range1,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(16) {
        const unsigned int i = item.get_global_id(0);
        if (i < numLCGBP)
        {
          const ISPCCamera& camera = *_camera;
          
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
                                                                                             
          edgeLevels.top    = min(edgeLevels.top,(unsigned char)lod_level_top);
          edgeLevels.right  = min(edgeLevels.right,(unsigned char)lod_level_right);
          edgeLevels.bottom = min(edgeLevels.bottom,(unsigned char)lod_level_bottom);
          edgeLevels.left   = min(edgeLevels.left,(unsigned char)lod_level_left);
                                                                                             
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
#if 0
    static double total_sum = 0.0f;
    static uint entries = 0;
    total_sum += gpu::getDeviceExecutionTiming(compute_lod_event) + gpu::getDeviceExecutionTiming(init_event);
    entries++;
    if (entries % 4096) PRINT(total_sum / entries);
    //PRINT4(gpu::getDeviceExecutionTiming(memset_event),gpu::getDeviceExecutionTiming(compute_lod_event),gpu::getDeviceExecutionTiming(select_clusterIDs_event),total);
#endif    
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
                                    const ISPCCamera* const _camera)
  {
    const unsigned int wgSize = 16*1;

    const unsigned int numLCMeshClusters = local_lcgbp_scene->numLCMeshClusters;
    const unsigned int numRootsTotal = local_lcgbp_scene->numLCMeshClusterRoots;
    unsigned char *const active_state = local_lcgbp_scene->lcm_cluster_active_state_per_frame;      

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
        const ISPCCamera& camera = *_camera;
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
            
            Vec3f bounds_lower = cur.bounds.lower.decompress(lower,diag)-org;
            Vec3f bounds_upper = cur.bounds.upper.decompress(lower,diag)-org;

            const bool cull = frustumCull( cur.bounds.lower.decompress(lower,diag)-org,cur.bounds.upper.decompress(lower,diag)-org,vx*width,vy*height,vz);
            if (!cull)
            {
              const bool subdivide = subdivideLOD(BBox3f(bounds_lower,bounds_upper),vx,vy,vz,width,height,lod_threshold);
              if (subdivide && cur.hasChildren())
                write = true;
            }
          }
          
          if (!write && active)
            active_state[currentID] = 1;                
          
          numIDs += writeSubgroup(&localIDs[numIDs],cur.leftID,write);          
          numIDs += writeSubgroup(&localIDs[numIDs],cur.rightID,write);
        }
#endif        
      });
    });



    // ================================================================================================================================
    // ================================================================================================================================
    // ================================================================================================================================

#if 0
    waitOnQueueAndCatchException(*global_gpu_queue);

    for (uint i=0;i<local_lcgbp_scene->numLCMeshClusters;i++)
    {
      const LossyCompressedMeshCluster &cur = local_lcgbp_scene->lcm_cluster[ i ];              
        
      if (active_state[i])
      {
        if (cur.hasChildren())
        {
          //const LossyCompressedMeshCluster &neighbor = local_lcgbp_scene->lcm_cluster[ cur.neighborID ];              
          if (active_state[ cur.leftID ] || active_state[ cur.rightID ])
          {
            PRINT6(i,cur.neighborID,cur.leftID,cur.rightID,(uint)active_state[ cur.leftID],(uint)active_state[ cur.rightID]);
          }
        }
      }
    }
    waitOnQueueAndCatchException(*global_gpu_queue);    
#endif
    
    const uint wgSize_select = 512;
    const sycl::nd_range<1> nd_range2(alignTo(numLCMeshClusters,wgSize_select),sycl::range<1>(wgSize_select));              
    sycl::event select_clusterIDs_event = global_gpu_queue->submit([=](sycl::handler& cgh){
      sycl::local_accessor< uint      ,  0> _cluster_counter(cgh);
      sycl::local_accessor< uint      ,  0> _global_offset(cgh);      
      sycl::local_accessor< uint      ,  0> _quad_counter(cgh);
      sycl::local_accessor< uint      ,  0> _block_counter(cgh);
      sycl::local_accessor< uint, 1> _localIDs(sycl::range<1>(512),cgh);
      

      cgh.depends_on(compute_lod_event);        
        
      cgh.parallel_for(nd_range2,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(16) {
        const unsigned int i = item.get_global_id(0);
        uint &cluster_counter  = *_cluster_counter.get_pointer();
        uint &global_offset    = *_global_offset.get_pointer();        
        uint &quad_counter     = *_quad_counter.get_pointer();
        uint &block_counter    = *_block_counter.get_pointer();
        uint *const localIDs   = _localIDs.get_pointer();

        cluster_counter = 0;
        quad_counter = 0;
        block_counter = 0;

        item.barrier(sycl::access::fence_space::local_space);
                              
        if (i < local_lcgbp_scene->numLCMeshClusters)
        {
          if (active_state[i])
          {
            //const unsigned int destID = gpu::atomic_add_global(&local_lcgbp_scene->numLCMeshClusterRootsPerFrame,(unsigned int)1);
            //const unsigned int destID = gpu::atomic_add_global_sub_group_varying(&local_lcgbp_scene->numLCMeshClusterRootsPerFrame,(unsigned int)1);
            //local_lcgbp_scene->lcm_cluster_roots_IDs_per_frame[destID] = i;
            const unsigned int destID = gpu::atomic_add_local(&cluster_counter,(unsigned int)1);
            localIDs[destID] = i;
            const LossyCompressedMeshCluster &cur = local_lcgbp_scene->lcm_cluster[ i ];              
            gpu::atomic_add_local(&quad_counter,(unsigned int)cur.numQuads);
            gpu::atomic_add_local(&block_counter,(unsigned int)cur.numBlocks);                                                      
          }
        }

        item.barrier(sycl::access::fence_space::local_space);
          
        const uint localID = item.get_local_id(0);
        const uint groupSize = wgSize_select; //item.get_group_size(0);

        if (localID == 0)
        {
          if (cluster_counter)
            global_offset = gpu::atomic_add_global(&local_lcgbp_scene->numLCMeshClusterRootsPerFrame,(unsigned int)cluster_counter);            
          if (quad_counter > 0)
            gpu::atomic_add_global(&local_lcgbp_scene->numLCMeshClusterQuadsPerFrame,(unsigned int)quad_counter);
          if (block_counter > 0)
            gpu::atomic_add_global(&local_lcgbp_scene->numLCMeshClusterBlocksPerFrame,(unsigned int)block_counter);            
        }

        item.barrier(sycl::access::fence_space::local_space);

        for (uint i=localID;i<cluster_counter;i+=groupSize)
          local_lcgbp_scene->lcm_cluster_roots_IDs_per_frame[global_offset+i] = localIDs[i];
        
      });
    });
    waitOnEventAndCatchException(select_clusterIDs_event);

#if 0
    static double total_sum = 0.0f;
    static uint entries = 0;
    total_sum += gpu::getDeviceExecutionTiming(memset_event)+gpu::getDeviceExecutionTiming(compute_lod_event)+gpu::getDeviceExecutionTiming(select_clusterIDs_event);
    entries++;
    if (entries % (4*4096)) PRINT2("LOD selection",total_sum / entries);
    //PRINT4(gpu::getDeviceExecutionTiming(memset_event),gpu::getDeviceExecutionTiming(compute_lod_event),gpu::getDeviceExecutionTiming(select_clusterIDs_event),total);
#endif    
  }


  void select_clusters_lod_mesh_tree(LCG_Scene *local_lcgbp_scene,
                                     const unsigned int width,
                                     const unsigned int height,
                                     const ISPCCamera* const _camera)
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
          const ISPCCamera& camera = *_camera;          
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
#if ALLOC_DEVICE_MEMORY  == 0    
    if (hit)
    {
      PRINT2(lcgbp_scene->pick_primID,lcgbp_scene->pick_geomID);
      PRINT(g_lod_threshold);

      if (lcgbp_scene->numLCMeshClusters)
      {
        const uint clusterID = lcgbp_scene->pick_primID;
        const LossyCompressedMeshCluster &cluster = lcgbp_scene->lcm_cluster[ clusterID ];
        PRINT3((int)cluster.numQuads,(int)cluster.numBlocks,(int)cluster.lod_level);
        PRINT3((int)cluster.leftID,(int)cluster.rightID,(int)cluster.neighborID);
#if 0        
        LossyCompressedMesh *mesh = cluster.mesh;
        const Vec3f lower = mesh->bounds.lower;
        const Vec3f diag = mesh->bounds.size() * (1.0f / CompressedVertex::RES_PER_DIM);
        // const uint width = 1024;
        // const uint height = 1024;
        const Vec3f org = camera.xfm.p;
        const Vec3f vx = camera.xfm.l.vx;
        const Vec3f vy = camera.xfm.l.vy;
        const Vec3f vz = camera.xfm.l.vz;
        
        const BBox3f cluster_bounds(cluster.bounds.lower.decompress(lower,diag),cluster.bounds.upper.decompress(lower,diag));
        PRINT2(cluster_bounds,area(cluster_bounds));
#endif        
        // const Vec2f diag2 = projectBBox3fToPlane( BBox3f(cluster_bounds.lower-org,cluster_bounds.upper-org), vx,vy,vz, width,height,true);
        // PRINT3(diag2,length(diag2),length(diag2)<lod_threshold);
        // bool subdivide = subdivideLOD(BBox3f(cluster_bounds.lower-org,cluster_bounds.upper-org),vx,vy,vz, width,height,lod_threshold);
        // PRINT(subdivide);
      }
    }
#endif    

                
    return hit;
  }
  
  
};
