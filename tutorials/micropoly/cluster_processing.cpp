#include "micropoly_device.h"
#include "../../kernels/rthwif/rtbuild/gpu/lcgbp.h"
#include "../../kernels/rthwif/rtbuild/gpu/morton.h"

#if 0
#define DBG_PRINT(x) PRINT(x)
#define DBG_PRINT2(x0,x1) PRINT2(x0,x1)
#define DBG_PRINT3(x0,x1,x2) PRINT3(x0,x1,x2)
#define DBG_PRINT4(x0,x1,x2,x3) PRINT4(x0,x1,x2,x3)
#define DBG_PRINT5(x0,x1,x2,x3,x4) PRINT5(x0,x1,x2,x3,x4)
#else
#define DBG_PRINT(x) 
#define DBG_PRINT2(x0,x1) 
#define DBG_PRINT3(x0,x1,x2) 
#define DBG_PRINT4(x0,x1,x2,x3)
#define DBG_PRINT5(x0,x1,x2,x3,x4)
#endif

#define UNLOCK_BORDER 0
#define MIN_AREA_DISTANCE_FCT 1
#define ENABLE_MT_PREPROCESS 1

#define GENERATE_LODS 1
#define ENABLE_INVALID_MERGE_IDS 1


namespace embree {

  typedef gpu::MortonCodePrimitive64x32Bits3D SpaceCurveType;

  uint32_t findVertex(std::vector<Vec3f> &vertices, const Vec3f &cv)
  {
    for (uint32_t i=0;i<vertices.size();i++)
      if (cv == vertices[i])
        return i;
    vertices.push_back(cv);
    return vertices.size()-1;
  }

  void countVertexIDs(std::vector<uint32_t> &vertices, const uint32_t cv)
  {
    for (uint32_t i=0;i<vertices.size();i++)
      if (cv == vertices[i])
        return;
    vertices.push_back(cv);
  }
  
  struct Triangle {
    uint32_t v0,v1,v2;

    __forceinline Triangle () {}
    __forceinline Triangle (const uint32_t v0, const uint32_t v1, const uint32_t v2) : v0(v0), v1(v1), v2(v2) {}

    __forceinline bool valid()
    {
      if (v0 != v1 && v1 != v2 && v2 != v0) return true;
      return false;
    }
  };

  struct Quad {
    uint32_t v0,v1,v2,v3;

    __forceinline Quad () {}
    __forceinline Quad (const uint32_t v0, const uint32_t v1, const uint32_t v2, const uint32_t v3) : v0(v0), v1(v1), v2(v2), v3(v3)  {}
  };

  struct BVH2Node
  {
    BBox3f bounds;
    uint32_t leftID,rightID;
    uint32_t numLeafPrims;
    uint32_t depth;
    
    BVH2Node() {}
    
    BVH2Node(const BBox3f &bounds, uint32_t ID) : bounds(bounds),leftID(ID),rightID(-1),numLeafPrims(1),depth(1) {}

    BVH2Node(const BVH2Node &left, const BVH2Node &right, uint32_t lID, uint32_t rID) {
      bounds = left.bounds;
      bounds.extend(right.bounds);
      leftID = lID;
      rightID = rID;
      numLeafPrims = left.numLeafPrims + right.numLeafPrims;
      depth = std::max(left.depth,right.depth)+1;
    }
    
    __forceinline bool isLeaf() { return rightID == -1; }
    __forceinline uint32_t leafID() { return leftID; }
    __forceinline uint32_t items() { return numLeafPrims; }
    
  };

  void extractIDs(const uint32_t currentID, BVH2Node *bvh, std::vector<uint32_t> &IDs)
  {
    if (bvh[currentID].isLeaf())
    {
      IDs.push_back(bvh[currentID].leafID());
    }
    else
    {
      extractIDs(bvh[currentID].leftID,bvh,IDs);
      extractIDs(bvh[currentID].rightID,bvh,IDs);      
    }
  }
  
  
  struct TriangleMesh {
    std::vector<Triangle> triangles;
    std::vector<Vec3f> vertices;
  };
  
  struct QuadMeshCluster {
    BBox3f bounds;
    bool lod_root;
    uint32_t leftID, rightID, neighborID;
    uint32_t depth;
    std::vector<Quad> quads;
    std::vector<Vec3f> vertices;
    std::vector<uint32_t> invalidMergeIDs;
    
    __forceinline QuadMeshCluster() : bounds(empty),lod_root(false),leftID(-1), rightID(-1), neighborID(-1), depth(0) {}

    __forceinline bool isLeaf() { return leftID == -1 || rightID == -1; }

    __forceinline void initBounds()
    {
      bounds = BBox3f(empty);
      for (uint32_t i=0;i<vertices.size();i++)
        bounds.extend(vertices[i]);
    }
    
    void reorderPLOC();
    
    uint32_t computeTriangleStrip();
    bool split(QuadMeshCluster &left, QuadMeshCluster &right);

    __forceinline void addInvalidMergeID(const uint32_t ID) { invalidMergeIDs.push_back(ID); }
    __forceinline bool isInvalidMergeID(const uint32_t ID)
    {
      for (uint32_t i=0;i<invalidMergeIDs.size();i++)
        if (invalidMergeIDs[i] == ID) return true;
      return false;
    }
  };
  
  void QuadMeshCluster::reorderPLOC()
  {
    const uint32_t numQuads = quads.size();
    Quad  *new_quads = new Quad[numQuads];
    BBox3f   *bounds = new BBox3f[numQuads];
    BVH2Node   *bvh2 = new BVH2Node[numQuads*2];

    uint32_t *index_buffer = new uint32_t[numQuads];
    uint32_t *tmp_buffer = new uint32_t[numQuads];
    uint32_t *nearest_neighborID = new uint32_t[numQuads];
    
    BBox3f centroidBounds(empty);

    for (uint32_t i=0;i<numQuads;i++)
    {
      const uint32_t v0 = quads[i].v0;
      const uint32_t v1 = quads[i].v1;
      const uint32_t v2 = quads[i].v2;
      const uint32_t v3 = quads[i].v3;

      const Vec3f &vtx0 = vertices[v0];
      const Vec3f &vtx1 = vertices[v1];
      const Vec3f &vtx2 = vertices[v2];
      const Vec3f &vtx3 = vertices[v3];

      BBox3f quadBounds(empty);
      quadBounds.extend(vtx0);
      quadBounds.extend(vtx1);
      quadBounds.extend(vtx2);
      quadBounds.extend(vtx3);
      centroidBounds.extend(quadBounds.center());
    }

    const Vec3f lower = centroidBounds.lower;
    const Vec3f diag = centroidBounds.size();
    const Vec3f inv_diag  = diag != Vec3fa(0.0f) ? Vec3fa(1.0f) / diag : Vec3fa(0.0f);

    std::vector<SpaceCurveType> mcodes;

    for (uint32_t i=0;i<numQuads;i++)
    {
      const uint32_t v0 = quads[i].v0;
      const uint32_t v1 = quads[i].v1;
      const uint32_t v2 = quads[i].v2;
      const uint32_t v3 = quads[i].v3;

      const Vec3f &vtx0 = vertices[v0];
      const Vec3f &vtx1 = vertices[v1];
      const Vec3f &vtx2 = vertices[v2];
      const Vec3f &vtx3 = vertices[v3];

      BBox3fa quadBounds(empty);
      quadBounds.extend(vtx0);
      quadBounds.extend(vtx1);
      quadBounds.extend(vtx2);
      quadBounds.extend(vtx3);

      bounds[i] = quadBounds;
            
      const uint32_t grid_size = 1 << 21; // 3*21 = 63
      const Vec3f grid_base = lower;
      const Vec3f grid_extend = diag;
      
      const Vec3f grid_scale = ((float)grid_size * 0.99f) * inv_diag;
      const Vec3f centroid =  quadBounds.center();

      const Vec3f gridpos_f = (centroid-grid_base)*grid_scale;                                                                      
      const uint32_t gx = (uint32_t)gridpos_f.x;
      const uint32_t gy = (uint32_t)gridpos_f.y;
      const uint32_t gz = (uint32_t)gridpos_f.z;
      const uint64_t code = bitInterleave64<uint64_t>(gx,gy,gz);
      mcodes.push_back(SpaceCurveType(code,i));      
    }

    std::sort(mcodes.begin(), mcodes.end());


    // ==============

    uint32_t numPrims = numQuads;

    for (uint32_t i=0;i<numPrims;i++)
    {
      const uint32_t ID = mcodes[i].getIndex();
      index_buffer[i] = i;
      bvh2[i] = BVH2Node(bounds[ID],ID);
    }

    const int SEARCH_RADIUS = 16;

    uint32_t numPrimitivesAlloc = numPrims;

    uint32_t cur_numPrims = numPrims;
    while(cur_numPrims > 1)
    {
      DBG_PRINT(cur_numPrims);
      for (uint32_t i=0;i<cur_numPrims;i++)
        nearest_neighborID[i] = -1;

      for (uint32_t c=0;c<cur_numPrims;c++)
      {
        // find nearest neighbor
        const uint32_t ID = index_buffer[c];
        const BBox3f bounds = bvh2[ID].bounds;
        float min_area = pos_inf;
        int nn = -1;
        for (int i=std::max((int)c-SEARCH_RADIUS,0);i<std::min((int)c+SEARCH_RADIUS+1,(int)cur_numPrims);i++)
          if (i != c && index_buffer[i] != -1)
          {
            const uint32_t merge_ID = index_buffer[i];
            BBox3f merged_bounds = bvh2[merge_ID].bounds;
            merged_bounds.extend(bounds);
            const float areaBounds = area(merged_bounds);
            if (areaBounds < min_area)
            {
              min_area = areaBounds;
              nn = i;
            }
          }
        nearest_neighborID[c] = nn;                  
      }

      for (uint32_t i=0;i<cur_numPrims;i++)
      {
        if (nearest_neighborID[i] != -1)
        {
          if ( nearest_neighborID[ nearest_neighborID[i] ] == i)
          {
            if ( i < nearest_neighborID[i])
            {
              const uint32_t leftID = index_buffer[i];
              const uint32_t rightID = index_buffer[nearest_neighborID[i]];
              const uint32_t newID = numPrimitivesAlloc++;
              bvh2[newID] = BVH2Node(bvh2[leftID],bvh2[rightID],leftID,rightID);
              tmp_buffer[i] = newID;
              //tmp_buffer[nearest_neighborID[i]] = -1;                            
            }
            else
              tmp_buffer[i] = -1;              
            
          }
          else
            tmp_buffer[i] = index_buffer[i];              
        }
        else
          tmp_buffer[i] = index_buffer[i];                      
      }
      
      uint32_t new_cur_numPrims = 0;
      for (uint32_t i=0;i<cur_numPrims;i++)
        if (tmp_buffer[i] != -1)
          index_buffer[new_cur_numPrims++] = tmp_buffer[i];

      if (cur_numPrims == new_cur_numPrims)
        FATAL("NO PLOC PRIM REDUCTION IN ITERATION");
      
      cur_numPrims = new_cur_numPrims;      
    }
    
    const uint32_t rootID = index_buffer[0];
    // ==============

    std::vector<uint32_t> IDs;

    extractIDs(rootID,bvh2,IDs);

    if (IDs.size() != numQuads)
    {
      PRINT2(IDs.size(),numQuads);
      FATAL("IDs.size() != numQuads");
    }
    
    for (uint32_t i=0;i<numQuads;i++)
      new_quads[i] = quads[IDs[i]];

    for (uint32_t i=0;i<numQuads;i++)
      quads[i] = new_quads[i];

    delete [] bounds;
    
    delete [] nearest_neighborID;        
    delete [] tmp_buffer;    
    delete [] index_buffer;

    delete [] bvh2;
    delete [] new_quads;
  }
  

  uint32_t QuadMeshCluster::computeTriangleStrip()
  {
    TriangleMesh mesh;
        
    // === cluster0 ===
    for (uint32_t i=0;i<quads.size();i++)
    {
      uint32_t v0 = findVertex(mesh.vertices, vertices[ quads[i].v0 ]);
      uint32_t v1 = findVertex(mesh.vertices, vertices[ quads[i].v1 ]);
      uint32_t v2 = findVertex(mesh.vertices, vertices[ quads[i].v2 ]);
      uint32_t v3 = findVertex(mesh.vertices, vertices[ quads[i].v3 ]);

      Triangle tri0(v0,v1,v3);
      Triangle tri1(v1,v2,v3);
      if (tri0.valid()) mesh.triangles.push_back(tri0);
      if (tri1.valid()) mesh.triangles.push_back(tri1);            
    }

    const uint32_t numTriangles = mesh.triangles.size();
    const uint32_t numVertices  = mesh.vertices.size();
    const uint32_t numIndices   = numTriangles*3;

    uint32_t *old_indices = (uint32_t*)&*mesh.triangles.begin();    
    std::vector<unsigned int> new_index_list(meshopt_stripifyBound(numIndices));
    uint32_t *new_indices = (uint32_t*)&*new_index_list.begin();    

#if 0
    meshopt_optimizeVertexCacheStrip(new_indices, old_indices, numIndices, numVertices);
    for (uint32_t i=0;i<numIndices;i++)
      old_indices[i] = new_indices[i];
#endif
    
    unsigned int restart_index = ~0u;
    size_t strip_size = meshopt_stripify(new_indices, old_indices, numIndices, numVertices, restart_index);
    return strip_size;
  }

  bool QuadMeshCluster::split(QuadMeshCluster &left, QuadMeshCluster &right)
  {
    reorderPLOC();
    
    uint32_t mid = quads.size() / 2;
    for (uint32_t i=0;i<mid;i++)
    {
      uint32_t v0 = findVertex(left.vertices, vertices[ quads[i].v0 ]);
      uint32_t v1 = findVertex(left.vertices, vertices[ quads[i].v1 ]);
      uint32_t v2 = findVertex(left.vertices, vertices[ quads[i].v2 ]);
      uint32_t v3 = findVertex(left.vertices, vertices[ quads[i].v3 ]);

      left.quads.push_back(Quad(v0,v1,v2,v3));
    }

    for (uint32_t i=mid;i<quads.size();i++)
    {
      uint32_t v0 = findVertex(right.vertices, vertices[ quads[i].v0 ]);
      uint32_t v1 = findVertex(right.vertices, vertices[ quads[i].v1 ]);
      uint32_t v2 = findVertex(right.vertices, vertices[ quads[i].v2 ]);
      uint32_t v3 = findVertex(right.vertices, vertices[ quads[i].v3 ]);

      right.quads.push_back(Quad(v0,v1,v2,v3));
    }

    if (left.vertices.size() <= 256 && left.quads.size() <= LossyCompressedMeshCluster::MAX_QUADS_PER_CLUSTER &&
        right.vertices.size() <= 256 && right.quads.size() <= LossyCompressedMeshCluster::MAX_QUADS_PER_CLUSTER)
      return true;
    else
      return false;
  }
  
  

  __forceinline std::pair<int,int> quad_index2(int p, int a0, int a1, int b0, int b1)
  {
    if      (b0 == a0) return std::make_pair(p-1,b1);
    else if (b0 == a1) return std::make_pair(p+0,b1);
    else if (b1 == a0) return std::make_pair(p-1,b0);
    else if (b1 == a1) return std::make_pair(p+0,b0);
    else return std::make_pair(0,-1);
  }
  
  __forceinline std::pair<int,int> quad_index3(int a0, int a1, int a2, int b0, int b1, int b2)
  {
    if      (b0 == a0) return quad_index2(0,a2,a1,b1,b2);
    else if (b0 == a1) return quad_index2(1,a0,a2,b1,b2);
    else if (b0 == a2) return quad_index2(2,a1,a0,b1,b2);
    else if (b1 == a0) return quad_index2(0,a2,a1,b0,b2);
    else if (b1 == a1) return quad_index2(1,a0,a2,b0,b2);
    else if (b1 == a2) return quad_index2(2,a1,a0,b0,b2);
    else if (b2 == a0) return quad_index2(0,a2,a1,b0,b1);
    else if (b2 == a1) return quad_index2(1,a0,a2,b0,b1);
    else if (b2 == a2) return quad_index2(2,a1,a0,b0,b1);
    else return std::make_pair(0,-1);
  }

  bool findSharedVertex(const std::vector<Vec3f> &vertices, const Vec3f &cv)
  {
    for (uint32_t i=0;i<vertices.size();i++)
      if (cv == vertices[i])
        return true;
    return false;
  }

  uint32_t getNumSharedBorderVertices(const QuadMeshCluster &cluster0,const QuadMeshCluster &cluster1)
  {
    const BBox3f sharedBounds = intersect(cluster0.bounds,cluster1.bounds);
    if (sharedBounds.empty()) return 0;
    
    std::vector<Vec3f> cluster0_vertices;
    std::vector<Vec3f> cluster1_vertices;

    for (uint32_t i=0;i<cluster0.vertices.size();i++)
      if (inside(sharedBounds,cluster0.vertices[i]))
        cluster0_vertices.push_back(cluster0.vertices[i]);

    for (uint32_t i=0;i<cluster1.vertices.size();i++)
      if (inside(sharedBounds,cluster1.vertices[i]))
        cluster1_vertices.push_back(cluster1.vertices[i]);

    uint32_t numSharedVertices = 0;
    for (uint32_t i=0;i<cluster0_vertices.size();i++)
      numSharedVertices += findSharedVertex(cluster1_vertices,cluster0_vertices[i]) ? 1 : 0;

    return numSharedVertices;
  }
  

  float getSimplificationRatio(QuadMeshCluster &cluster0,QuadMeshCluster &cluster1)
  {
    TriangleMesh mesh;    
    
    // === cluster0 ===
    for (uint32_t i=0;i<cluster0.quads.size();i++)
    {
      uint32_t v0 = findVertex(mesh.vertices, cluster0.vertices[ cluster0.quads[i].v0 ]);
      uint32_t v1 = findVertex(mesh.vertices, cluster0.vertices[ cluster0.quads[i].v1 ]);
      uint32_t v2 = findVertex(mesh.vertices, cluster0.vertices[ cluster0.quads[i].v2 ]);
      uint32_t v3 = findVertex(mesh.vertices, cluster0.vertices[ cluster0.quads[i].v3 ]);

      Triangle tri0(v0,v1,v3);
      Triangle tri1(v1,v2,v3);
      if (tri0.valid()) mesh.triangles.push_back(tri0);
      if (tri1.valid()) mesh.triangles.push_back(tri1);            
    }

    // === cluster1 ===
    for (uint32_t i=0;i<cluster1.quads.size();i++)
    {
      uint32_t v0 = findVertex(mesh.vertices, cluster1.vertices[ cluster1.quads[i].v0 ]);
      uint32_t v1 = findVertex(mesh.vertices, cluster1.vertices[ cluster1.quads[i].v1 ]);
      uint32_t v2 = findVertex(mesh.vertices, cluster1.vertices[ cluster1.quads[i].v2 ]);
      uint32_t v3 = findVertex(mesh.vertices, cluster1.vertices[ cluster1.quads[i].v3 ]);

      Triangle tri0(v0,v1,v3);
      Triangle tri1(v1,v2,v3);
      if (tri0.valid()) mesh.triangles.push_back(tri0);
      if (tri1.valid()) mesh.triangles.push_back(tri1);            
    }

    const uint32_t numTriangles = mesh.triangles.size();
    const uint32_t numVertices  = mesh.vertices.size();
    const uint32_t numIndices   = numTriangles*3;

    Triangle *new_triangles = new Triangle[numTriangles];    
    Triangle *triangles     = &*mesh.triangles.begin();
    Vec3f *vertices         = &*mesh.vertices.begin();

    const float REDUCTION_FACTOR = 0.5f;
    const uint32_t expectedTriangles = ceilf(numTriangles * REDUCTION_FACTOR); 
    
    const float max_error = 0.1f;
    
    float result_error = 0.0f;
    const uint32_t opts = meshopt_SimplifyLockBorder;
    const size_t new_numIndices = meshopt_simplify((uint32_t*)new_triangles,(uint32_t*)triangles,numIndices,(float*)vertices,numVertices,sizeof(Vec3f),expectedTriangles*3,max_error,opts,&result_error);
    const size_t new_numTriangles = new_numIndices/3;                
    
    delete [] new_triangles;

    return (float)new_numTriangles / numTriangles;
  }


  bool mergeSimplifyQuadMeshCluster(QuadMeshCluster &cluster0,QuadMeshCluster &cluster1, std::vector<QuadMeshCluster> &quadMeshes)
  {
    QuadMeshCluster quadMesh;
    TriangleMesh mesh;
    
    
    // === cluster0 ===
    for (uint32_t i=0;i<cluster0.quads.size();i++)
    {
      uint32_t v0 = findVertex(mesh.vertices, cluster0.vertices[ cluster0.quads[i].v0 ]);
      uint32_t v1 = findVertex(mesh.vertices, cluster0.vertices[ cluster0.quads[i].v1 ]);
      uint32_t v2 = findVertex(mesh.vertices, cluster0.vertices[ cluster0.quads[i].v2 ]);
      uint32_t v3 = findVertex(mesh.vertices, cluster0.vertices[ cluster0.quads[i].v3 ]);

      Triangle tri0(v0,v1,v3);
      Triangle tri1(v1,v2,v3);
      if (tri0.valid()) mesh.triangles.push_back(tri0);
      if (tri1.valid()) mesh.triangles.push_back(tri1);            
    }

    // === cluster1 ===
    for (uint32_t i=0;i<cluster1.quads.size();i++)
    {
      uint32_t v0 = findVertex(mesh.vertices, cluster1.vertices[ cluster1.quads[i].v0 ]);
      uint32_t v1 = findVertex(mesh.vertices, cluster1.vertices[ cluster1.quads[i].v1 ]);
      uint32_t v2 = findVertex(mesh.vertices, cluster1.vertices[ cluster1.quads[i].v2 ]);
      uint32_t v3 = findVertex(mesh.vertices, cluster1.vertices[ cluster1.quads[i].v3 ]);

      Triangle tri0(v0,v1,v3);
      Triangle tri1(v1,v2,v3);
      if (tri0.valid()) mesh.triangles.push_back(tri0);
      if (tri1.valid()) mesh.triangles.push_back(tri1);            
    }
    
    DBG_PRINT(mesh.vertices.size());
    DBG_PRINT(mesh.triangles.size());

    const uint32_t numTriangles = mesh.triangles.size();
    const uint32_t numVertices  = mesh.vertices.size();
    const uint32_t numIndices   = numTriangles*3;

    Triangle *new_triangles = new Triangle[numTriangles];    
    Triangle *triangles     = &*mesh.triangles.begin();
    Vec3f *vertices         = &*mesh.vertices.begin();

    const float REDUCTION_FACTOR = 0.5f;
    uint32_t expectedTriangles = floorf((LossyCompressedMeshCluster::MAX_QUADS_PER_CLUSTER * 4) * REDUCTION_FACTOR);
    
    uint32_t iterations = 0;
    while(1)
    {
      iterations++;

      uint32_t opts = meshopt_SimplifyLockBorder;
#if UNLOCK_BORDER == 1
      if (iterations > 5)
      {
        opts = 0;
      }
#endif            
      if (iterations > 10) return false;
      bool retry = false;      
      float result_error = 0.0f;
      const size_t new_numIndices = meshopt_simplify((uint32_t*)new_triangles,(uint32_t*)triangles,numIndices,(float*)vertices,numVertices,sizeof(Vec3f),expectedTriangles*3,0.1f,opts,&result_error);

      const size_t new_numTriangles = new_numIndices/3;
      DBG_PRINT3(expectedTriangles,new_numTriangles,result_error);

      for (size_t i=0; i<new_numTriangles; i++)
      {
        const int a0 = findVertex(quadMesh.vertices, mesh.vertices[ new_triangles[i+0].v0 ]);
        const int a1 = findVertex(quadMesh.vertices, mesh.vertices[ new_triangles[i+0].v1 ]);
        const int a2 = findVertex(quadMesh.vertices, mesh.vertices[ new_triangles[i+0].v2 ]);      
        if (i+1 == new_numTriangles) {
          quadMesh.quads.push_back(Quad(a0,a1,a2,a2));
          continue;
        }

        const int b0 = findVertex(quadMesh.vertices, mesh.vertices[ new_triangles[i+1].v0 ]);
        const int b1 = findVertex(quadMesh.vertices, mesh.vertices[ new_triangles[i+1].v1 ]);
        const int b2 = findVertex(quadMesh.vertices, mesh.vertices[ new_triangles[i+1].v2 ]);      
        const std::pair<int,int> q = quad_index3(a0,a1,a2,b0,b1,b2);
        const int a3 = q.second;
        if (a3 == -1) {
          quadMesh.quads.push_back(Quad(a0,a1,a2,a2));
          continue;
        }
      
        if      (q.first == -1) quadMesh.quads.push_back(Quad(a1,a2,a3,a0));
        else if (q.first ==  0) quadMesh.quads.push_back(Quad(a3,a1,a2,a0));
        else if (q.first ==  1) quadMesh.quads.push_back(Quad(a0,a1,a3,a2));
        else if (q.first ==  2) quadMesh.quads.push_back(Quad(a1,a2,a3,a0)); 
        i++;
      }

      if (quadMesh.quads.size() > LossyCompressedMeshCluster::MAX_QUADS_PER_CLUSTER) { DBG_PRINT2("RETRY quadMesh.quads.size()",quadMesh.quads.size()); retry = true; }
      
      if (quadMesh.vertices.size() > 256) {
        //PRINT("RETRY quadMesh.vertices.size()");
        retry = true;
      }
      
      if (retry)
      {
        quadMesh.vertices.clear();
        quadMesh.quads.clear();
        DBG_PRINT2(quadMesh.vertices.size(),quadMesh.quads.size());
        expectedTriangles -= std::max((uint32_t)2,expectedTriangles/10);
      }
      else
        break;
    }

    DBG_PRINT2(quadMesh.quads.size(),quadMesh.vertices.size());

    delete [] new_triangles;

    quadMesh.reorderPLOC();
    quadMeshes.push_back(quadMesh);
    return true;
  }



  // ==========================================================

  void mergeQuadMeshCluster(QuadMeshCluster &cluster0,QuadMeshCluster &cluster1, std::vector<QuadMeshCluster> &quadMeshes)
  {
    QuadMeshCluster quadMesh = cluster0;

    for (uint32_t i=0;i<cluster1.quads.size();i++)
    {
      uint32_t v0 = findVertex(quadMesh.vertices, cluster1.vertices[ cluster1.quads[i].v0 ]);
      uint32_t v1 = findVertex(quadMesh.vertices, cluster1.vertices[ cluster1.quads[i].v1 ]);
      uint32_t v2 = findVertex(quadMesh.vertices, cluster1.vertices[ cluster1.quads[i].v2 ]);
      uint32_t v3 = findVertex(quadMesh.vertices, cluster1.vertices[ cluster1.quads[i].v3 ]);

      Quad quad(v0,v1,v2,v3);
      quadMesh.quads.push_back(quad);
    }
    quadMeshes.push_back(quadMesh);
  }
  
  bool mergeSimplifyQuadMeshClusterDAG(QuadMeshCluster &cluster0,QuadMeshCluster &cluster1, std::vector<QuadMeshCluster> &quadMeshes)
  {
    DBG_PRINT3("CLUSTER MERGING",cluster0.quads.size(),cluster1.quads.size());
    QuadMeshCluster quadMesh;
    TriangleMesh mesh;
    
    
    // === cluster0 ===
    for (uint32_t i=0;i<cluster0.quads.size();i++)
    {
      uint32_t v0 = findVertex(mesh.vertices, cluster0.vertices[ cluster0.quads[i].v0 ]);
      uint32_t v1 = findVertex(mesh.vertices, cluster0.vertices[ cluster0.quads[i].v1 ]);
      uint32_t v2 = findVertex(mesh.vertices, cluster0.vertices[ cluster0.quads[i].v2 ]);
      uint32_t v3 = findVertex(mesh.vertices, cluster0.vertices[ cluster0.quads[i].v3 ]);

      Triangle tri0(v0,v1,v3);
      Triangle tri1(v1,v2,v3);
      if (tri0.valid()) mesh.triangles.push_back(tri0);
      if (tri1.valid()) mesh.triangles.push_back(tri1);            
    }

    // === cluster1 ===
    for (uint32_t i=0;i<cluster1.quads.size();i++)
    {
      uint32_t v0 = findVertex(mesh.vertices, cluster1.vertices[ cluster1.quads[i].v0 ]);
      uint32_t v1 = findVertex(mesh.vertices, cluster1.vertices[ cluster1.quads[i].v1 ]);
      uint32_t v2 = findVertex(mesh.vertices, cluster1.vertices[ cluster1.quads[i].v2 ]);
      uint32_t v3 = findVertex(mesh.vertices, cluster1.vertices[ cluster1.quads[i].v3 ]);

      Triangle tri0(v0,v1,v3);
      Triangle tri1(v1,v2,v3);
      if (tri0.valid()) mesh.triangles.push_back(tri0);
      if (tri1.valid()) mesh.triangles.push_back(tri1);            
    }
    
    DBG_PRINT(mesh.vertices.size());
    DBG_PRINT(mesh.triangles.size());

    const uint32_t numTriangles = mesh.triangles.size();
    const uint32_t numVertices  = mesh.vertices.size();
    const uint32_t numIndices   = numTriangles*3;

    Triangle *new_triangles = new Triangle[numTriangles];    
    Triangle *triangles     = &*mesh.triangles.begin();
    Vec3f *vertices         = &*mesh.vertices.begin();

    const float REDUCTION_FACTOR = 0.5f;
    uint32_t expectedTriangles = ceilf(numTriangles * REDUCTION_FACTOR); //floorf((LossyCompressedMeshCluster::MAX_QUADS_PER_CLUSTER * 4) * REDUCTION_FACTOR);
    
    uint32_t iterations = 0;
    float max_error = 0.1f;
    const float REDUCTION_THRESHOLD = 0.8f;
    //while(1)
    {
      iterations++;

      uint32_t opts = meshopt_SimplifyLockBorder;

      bool retry = false;
      size_t new_numIndices = 0;
      while(max_error < 1.0f) {
        float result_error = 0.0f;
        new_numIndices = meshopt_simplify((uint32_t*)new_triangles,(uint32_t*)triangles,numIndices,(float*)vertices,numVertices,sizeof(Vec3f),expectedTriangles*3,max_error,opts,&result_error);
        const size_t new_numTriangles = new_numIndices/3;                
        DBG_PRINT5("SIMPLIFY",new_numTriangles,numTriangles,expectedTriangles,(float)new_numTriangles / numTriangles);
        if ((float)new_numTriangles / numTriangles <= REDUCTION_THRESHOLD) break;        
        //expectedTriangles += std::max(expectedTriangles/10,(uint32_t)1);
        expectedTriangles += expectedTriangles/10;
        max_error *= 2;
      } 

      const size_t new_numTriangles = new_numIndices/3;

      if ((float)new_numTriangles / numTriangles > REDUCTION_THRESHOLD) { DBG_PRINT5("NOT ENOUGH REDUCTION",numTriangles,new_numTriangles,expectedTriangles,iterations); return false; }
      
      DBG_PRINT2(expectedTriangles,new_numTriangles);

      for (size_t i=0; i<new_numTriangles; i++)
      {
        const int a0 = findVertex(quadMesh.vertices, mesh.vertices[ new_triangles[i+0].v0 ]);
        const int a1 = findVertex(quadMesh.vertices, mesh.vertices[ new_triangles[i+0].v1 ]);
        const int a2 = findVertex(quadMesh.vertices, mesh.vertices[ new_triangles[i+0].v2 ]);      
        if (i+1 == new_numTriangles) {
          quadMesh.quads.push_back(Quad(a0,a1,a2,a2));
          continue;
        }

        const int b0 = findVertex(quadMesh.vertices, mesh.vertices[ new_triangles[i+1].v0 ]);
        const int b1 = findVertex(quadMesh.vertices, mesh.vertices[ new_triangles[i+1].v1 ]);
        const int b2 = findVertex(quadMesh.vertices, mesh.vertices[ new_triangles[i+1].v2 ]);      
        const std::pair<int,int> q = quad_index3(a0,a1,a2,b0,b1,b2);
        const int a3 = q.second;
        if (a3 == -1) {
          quadMesh.quads.push_back(Quad(a0,a1,a2,a2));
          continue;
        }
      
        if      (q.first == -1) quadMesh.quads.push_back(Quad(a1,a2,a3,a0));
        else if (q.first ==  0) quadMesh.quads.push_back(Quad(a3,a1,a2,a0));
        else if (q.first ==  1) quadMesh.quads.push_back(Quad(a0,a1,a3,a2));
        else if (q.first ==  2) quadMesh.quads.push_back(Quad(a1,a2,a3,a0)); 
        i++;
      }


      if (quadMesh.quads.size() > LossyCompressedMeshCluster::MAX_QUADS_PER_CLUSTER) { DBG_PRINT2("RETRY quadMesh.quads.size()",quadMesh.quads.size()); retry = true; }

      if (quadMesh.vertices.size() > 256) {
        DBG_PRINT("RETRY quadMesh.vertices.size()");
        retry = true;
      }
      
      if (retry)
      {
        DBG_PRINT("SPLIT DAG");
        QuadMeshCluster left, right;
        const bool split_success = quadMesh.split(left,right);
        DBG_PRINT3(split_success,left.quads.size(),right.quads.size());
        
        if (split_success)
        {
          quadMeshes.push_back(left);
          quadMeshes.push_back(right);
        }
        else
        {
          DBG_PRINT("SPLIT FAILED");
          return false;
        }
      }
      else
        quadMeshes.push_back(quadMesh);        
    }

    DBG_PRINT(quadMeshes.size());
    for (uint32_t i=0;i<quadMeshes.size();i++)
      DBG_PRINT2(quadMeshes[i].quads.size(),quadMeshes[i].vertices.size());
      
    for (uint32_t i=0;i<quadMeshes.size();i++)      
      quadMeshes[i].reorderPLOC();
      
    delete [] new_triangles;

    
    return true;
  }
  


  // ==========================================================
  
  
  
  
  __forceinline uint32_t remap_vtx_index(const uint32_t v, std::map<uint32_t,uint32_t> &index_map, uint32_t &numLocalIndices)
  {
    auto e = index_map.find(v);
    if (e != index_map.end()) return e->second;
    const uint32_t ID = numLocalIndices++;
    index_map[v] = ID;
    return ID;
  }

  struct HierarchyRange
  {
    gpu::Range range;
    uint32_t parent, left, right;
    uint32_t counter, clusterID;

    __forceinline HierarchyRange(const gpu::Range &range, const uint32_t parent = -1) : range(range), parent(parent), left(-1), right(-1), counter(0), clusterID(-1) {}

    __forceinline bool isLeaf() { return left == -1 || right == -1; }
  };
  

  
  void extractClusters(const uint32_t currentID, BVH2Node *bvh, std::vector<QuadMeshCluster> &clusters, ISPCQuadMesh* mesh, const uint32_t threshold)
  {
    if (bvh[currentID].items() < threshold || bvh[currentID].isLeaf())
    {
      std::vector<uint32_t> IDs;
      extractIDs(currentID,bvh,IDs);

      std::map<uint32_t,uint32_t> index_map;
      uint32_t numLocalIndices = 0;
      bool fits = true;
      int min_index_delta = 0;
      int max_index_delta = 0;
      
      for (uint32_t j=0;j<IDs.size();j++)
      {
        const uint32_t index = IDs[j];
        const uint32_t v0 = mesh->quads[index].v0;
        const uint32_t v1 = mesh->quads[index].v1;
        const uint32_t v2 = mesh->quads[index].v2;
        const uint32_t v3 = mesh->quads[index].v3;
        
        const uint32_t new_v0 = remap_vtx_index(v0,index_map,numLocalIndices);
        const uint32_t new_v1 = remap_vtx_index(v1,index_map,numLocalIndices);
        const uint32_t new_v2 = remap_vtx_index(v2,index_map,numLocalIndices);
        const uint32_t new_v3 = remap_vtx_index(v3,index_map,numLocalIndices);

        min_index_delta = std::min(min_index_delta,(int)new_v1 - (int)new_v0);
        min_index_delta = std::min(min_index_delta,(int)new_v2 - (int)new_v0);
        min_index_delta = std::min(min_index_delta,(int)new_v3 - (int)new_v0);

        max_index_delta = std::max(max_index_delta,(int)new_v1 - (int)new_v0);
        max_index_delta = std::max(max_index_delta,(int)new_v2 - (int)new_v0);
        max_index_delta = std::max(max_index_delta,(int)new_v3 - (int)new_v0);
        
      }

      if (index_map.size() > 256)
      {
        DBG_PRINT2(IDs.size(),index_map.size());
        DBG_PRINT2(min_index_delta,max_index_delta);
        //FATAL("256");
        fits = false;
      }
      
      if (fits)
      {
        QuadMeshCluster cluster;

        for (uint32_t j=0;j<IDs.size();j++)
        {
          const uint32_t index = IDs[j];
          const uint32_t v0 = mesh->quads[index].v0;
          const uint32_t v1 = mesh->quads[index].v1;
          const uint32_t v2 = mesh->quads[index].v2;
          const uint32_t v3 = mesh->quads[index].v3;

          const uint32_t remaped_v0 =  remap_vtx_index(v0,index_map,numLocalIndices);
          const uint32_t remaped_v1 =  remap_vtx_index(v1,index_map,numLocalIndices);
          const uint32_t remaped_v2 =  remap_vtx_index(v2,index_map,numLocalIndices);
          const uint32_t remaped_v3 =  remap_vtx_index(v3,index_map,numLocalIndices);

          cluster.quads.push_back(Quad(remaped_v0,remaped_v1,remaped_v2,remaped_v3));
          
        }

        cluster.vertices.resize(numLocalIndices);
      
        for (std::map<uint32_t,uint32_t>::iterator i=index_map.begin(); i != index_map.end(); i++)
        {
          const uint32_t old_v = (*i).first;
          const uint32_t new_v = (*i).second;
          cluster.vertices[new_v] = mesh->positions[0][old_v];
        }
                
        cluster.initBounds();
        cluster.depth = 1;
        cluster.lod_root = true;
        
        if (cluster.quads.size() > LossyCompressedMeshCluster::MAX_QUADS_PER_CLUSTER) FATAL("cluster.quads");
        if (numLocalIndices > 256) FATAL("cluster.vertices");

        clusters.push_back(cluster);
        return;
      }      
    }

    extractClusters(bvh[currentID].leftID,bvh,clusters,mesh,threshold);
    extractClusters(bvh[currentID].rightID,bvh,clusters,mesh,threshold);      
  }  

  // ======================================================================================================================================================================================
  // ======================================================================================================================================================================================
  // ======================================================================================================================================================================================

  template<typename SpaceCurveType>
  std::vector<QuadMeshCluster> extractRangesPLOC(std::vector<SpaceCurveType> &mcodes,const std::vector<BBox3f> &plocBounds, ISPCQuadMesh* mesh, const uint32_t threshold)
  {
    std::vector<QuadMeshCluster> clusters;
    
    const uint32_t numPrims = mcodes.size();
    DBG_PRINT(numPrims);
    
    uint32_t *index_buffer = new uint32_t[numPrims];
    uint32_t *tmp_buffer = new uint32_t[numPrims];
    uint32_t *nearest_neighborID = new uint32_t[numPrims];

    BVH2Node *bvh2 = new BVH2Node[numPrims*2];

    for (uint32_t i=0;i<numPrims;i++)
    {
      const uint32_t ID = mcodes[i].getIndex();
      index_buffer[i] = i;
      bvh2[i] = BVH2Node(plocBounds[ID],ID);
    }

    const int SEARCH_RADIUS = 16;

    std::atomic<uint32_t> numPrimitives(numPrims);

    uint32_t cur_numPrims = numPrims;
    while(cur_numPrims > 1)
    {
      DBG_PRINT(cur_numPrims);
      for (uint32_t i=0;i<cur_numPrims;i++)
        nearest_neighborID[i] = -1;

#if ENABLE_MT_PREPROCESS == 1      
      parallel_for((uint32_t)0, cur_numPrims, [&] (const range<uint32_t>& r)
      {
        for (uint32_t c=r.begin();c<r.end();c++)
#else
        for (uint32_t c=0;c<cur_numPrims;c++)
#endif          
        {
          // find nearest neighbor
          const uint32_t ID = index_buffer[c];
          const BBox3f bounds = bvh2[ID].bounds;
          float min_area = pos_inf;
          int nn = -1;
          int start = std::max((int)c-SEARCH_RADIUS,0);
          int end   = std::min((int)c+SEARCH_RADIUS+1,(int)cur_numPrims);
          int plus  = 1;
          for (int i=start;i!=end;i+=plus)            
            if (i != c && index_buffer[i] != -1)
            {
              const uint32_t merge_ID = index_buffer[i];
              BBox3f merged_bounds = bvh2[merge_ID].bounds;
              merged_bounds.extend(bounds);
              const float areaBounds = area(merged_bounds);
              if (areaBounds < min_area)
              {
                min_area = areaBounds;
                nn = i;
              }              
            }
          nearest_neighborID[c] = nn;                  
        }
#if ENABLE_MT_PREPROCESS == 1              
      });
#endif      

#if ENABLE_MT_PREPROCESS == 1            
      parallel_for((uint32_t)0, cur_numPrims, [&] (const range<uint32_t>& r)
      {
        for (uint32_t i=r.begin();i<r.end();i++)
#else
        for (uint32_t i=0;i<cur_numPrims;i++)          
#endif          
        {
          if (nearest_neighborID[i] != -1)
          {
            if ( nearest_neighborID[ nearest_neighborID[i] ] == i)
            {
              if ( i < nearest_neighborID[i])
              {
                const uint32_t leftID = index_buffer[i];
                const uint32_t rightID = index_buffer[nearest_neighborID[i]];
                const uint32_t newID = numPrimitives.fetch_add(1);
                bvh2[newID] = BVH2Node(bvh2[leftID],bvh2[rightID],leftID,rightID);
                tmp_buffer[i] = newID;
                //tmp_buffer[nearest_neighborID[i]] = -1;                            
              }
              else
                tmp_buffer[i] = -1;              
            
            }
            else
              tmp_buffer[i] = index_buffer[i];              
          }
          else
            tmp_buffer[i] = index_buffer[i];                      
        }
#if ENABLE_MT_PREPROCESS == 1                      
      });
#endif      
      
      uint32_t new_cur_numPrims = 0;
      for (uint32_t i=0;i<cur_numPrims;i++)
        if (tmp_buffer[i] != -1)
          index_buffer[new_cur_numPrims++] = tmp_buffer[i];

      if (cur_numPrims == new_cur_numPrims)
        FATAL("NO PLOC PRIM REDUCTION IN ITERATION");
      
      cur_numPrims = new_cur_numPrims;      
    }

    uint32_t rootID = index_buffer[0];
    

    DBG_PRINT("BVH2 DONE");

    extractClusters(rootID, bvh2, clusters, mesh, threshold);

    DBG_PRINT(clusters.size());
    DBG_PRINT("CLUSTER EXTRACTION DONE");
    
    delete [] bvh2;  
    delete [] nearest_neighborID;        
    delete [] tmp_buffer;    
    delete [] index_buffer;

    return clusters;
  }
  
  

  // ======================================================================================================================================================================================
  // ======================================================================================================================================================================================
  // ======================================================================================================================================================================================
  


  // ======================================================================================================================================================================================
  // ======================================================================================================================================================================================
  // ======================================================================================================================================================================================
  
  void extractRanges(const uint32_t currentID, const SpaceCurveType *const mcodes, std::vector<HierarchyRange> &ranges, std::vector<uint32_t> &leafIDs, ISPCQuadMesh* mesh, uint32_t &numTotalVertices, const uint32_t threshold)
  {
    if (ranges[currentID].range.size() < threshold)
    {
      std::map<uint32_t,uint32_t> index_map;
      uint32_t numLocalIndices = 0;
      bool fits = true;
      for (uint32_t j=ranges[currentID].range.start;j<ranges[currentID].range.end;j++)
      {
        const uint32_t index = mcodes[j].getIndex();
        const uint32_t v0 = mesh->quads[index].v0;
        const uint32_t v1 = mesh->quads[index].v1;
        const uint32_t v2 = mesh->quads[index].v2;
        const uint32_t v3 = mesh->quads[index].v3;

        remap_vtx_index(v0,index_map,numLocalIndices);
        remap_vtx_index(v1,index_map,numLocalIndices);
        remap_vtx_index(v2,index_map,numLocalIndices);
        remap_vtx_index(v3,index_map,numLocalIndices);
        if (index_map.size() > 256)
        {
          fits = false;
          break;
        }
      }
      
      if (fits)
      {
        //DBG_PRINT4(ranges[currentID].range.start,ranges[currentID].range.end,ranges[currentID].range.size(),ranges[currentID].parent);
        leafIDs.push_back(currentID);
        numTotalVertices += index_map.size();
        return;
      }
    }
    
    gpu::Range left, right;
    splitRange(ranges[currentID].range,mcodes,left,right);

    const uint32_t leftID = ranges.size();
    ranges.push_back(HierarchyRange(left,currentID));
    const uint32_t rightID = ranges.size();
    ranges.push_back(HierarchyRange(right,currentID));

    ranges[currentID].left = leftID;
    ranges[currentID].right = rightID;
    
    extractRanges(leftID,mcodes,ranges,leafIDs,mesh,numTotalVertices,threshold);
    extractRanges(rightID,mcodes,ranges,leafIDs,mesh,numTotalVertices,threshold);      
  }

  void extractClusterRootIDs(const uint32_t currentID, std::vector<HierarchyRange> &ranges, std::vector<uint32_t> &clusterRootIDs)
  {
    if (ranges[currentID].isLeaf()  || ranges[currentID].counter == 2 )
    {
      if (ranges[currentID].clusterID == -1) FATAL("ranges[currentID].clusterID");
      clusterRootIDs.push_back(ranges[currentID].clusterID);
    }
    else
    {
      if (ranges[currentID].left != -1)
        extractClusterRootIDs(ranges[currentID].left,ranges,clusterRootIDs);
      if (ranges[currentID].right != -1)
        extractClusterRootIDs(ranges[currentID].right,ranges,clusterRootIDs);      
    }
    
  }

      
  Vec2i convertISPCQuadMesh(ISPCQuadMesh* mesh, RTCScene scene, ISPCOBJMaterial *material,const uint32_t geomID,std::vector<LossyCompressedMesh*> &lcm_ptrs,std::vector<LossyCompressedMeshCluster> &lcm_clusters, std::vector<uint32_t> &lcm_clusterRootIDs, size_t &totalCompressedSize, size_t &numDecompressedBlocks, sycl::queue &queue)
  {
    const uint32_t numQuads = mesh->numQuads;
    const uint32_t INITIAL_CREATE_RANGE_THRESHOLD = LossyCompressedMeshCluster::MAX_QUADS_PER_CLUSTER;

    const uint32_t global_start_lcm_clusterID = lcm_clusters.size();      

    static size_t uncompressedSize = 0;
    static size_t uncompressedTriangles = 0;
    
    PRINT(mesh->numQuads);
    PRINT(mesh->numVertices);    
    PRINT(mesh->numQuads*sizeof(ISPCTriangle)*2);
    PRINT(mesh->numVertices*sizeof(Vec3f));
    uncompressedSize += mesh->numQuads*sizeof(ISPCTriangle)*2;
    uncompressedSize += mesh->numVertices*sizeof(Vec3f);
    uncompressedTriangles += mesh->numQuads*2;

    PRINT3(uncompressedTriangles,uncompressedSize,(double)uncompressedSize / uncompressedTriangles);
    
    // === get centroid and geometry bounding boxes ===
    
    BBox3fa centroidBounds(empty);
    BBox3fa geometryBounds(empty);
    
    for (uint32_t i=0;i<numQuads;i++)
    {
      const uint32_t v0 = mesh->quads[i].v0;
      const uint32_t v1 = mesh->quads[i].v1;
      const uint32_t v2 = mesh->quads[i].v2;
      const uint32_t v3 = mesh->quads[i].v3;

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
    
    std::vector<SpaceCurveType> mcodes;
    std::vector<HierarchyRange> ranges;
    std::vector<uint32_t> leafIDs;
    std::vector<uint32_t> clusterRootIDs;
    std::vector<BBox3f> plocBounds;
    mcodes.reserve(numQuads);
    
    for (uint32_t i=0;i<numQuads;i++)
    {
      const uint32_t v0 = mesh->quads[i].v0;
      const uint32_t v1 = mesh->quads[i].v1;
      const uint32_t v2 = mesh->quads[i].v2;
      const uint32_t v3 = mesh->quads[i].v3;

      const Vec3fa &vtx0 = mesh->positions[0][v0];
      const Vec3fa &vtx1 = mesh->positions[0][v1];
      const Vec3fa &vtx2 = mesh->positions[0][v2];
      const Vec3fa &vtx3 = mesh->positions[0][v3];

      BBox3fa quadBounds(empty);
      quadBounds.extend(vtx0);
      quadBounds.extend(vtx1);
      quadBounds.extend(vtx2);
      quadBounds.extend(vtx3);
            
      const uint32_t grid_size = 1 << 21; // 3*21 = 63
      const Vec3f grid_base = lower;
      const Vec3f grid_extend = diag;
      
      const Vec3f grid_scale = ((float)grid_size * 0.99f) * inv_diag;
      const Vec3f centroid =  quadBounds.center();

      const Vec3f gridpos_f = (centroid-grid_base)*grid_scale;                                                                      
      const uint32_t gx = (uint32_t)gridpos_f.x;
      const uint32_t gy = (uint32_t)gridpos_f.y;
      const uint32_t gz = (uint32_t)gridpos_f.z;
      const uint64_t code = bitInterleave64<uint64_t>(gx,gy,gz);
      mcodes.push_back(SpaceCurveType(code,i));
      
      plocBounds.push_back(quadBounds);
    }

    // === sort morton codes ===
    
    std::sort(mcodes.begin(), mcodes.end()); 

    // === extract ranges, test range if it fullfills requirements, split if necessary ===

    std::vector<QuadMeshCluster> extractClusters = extractRangesPLOC<SpaceCurveType>(mcodes,plocBounds,mesh,INITIAL_CREATE_RANGE_THRESHOLD);
    const uint32_t MAX_NUM_MESH_CLUSTERS = extractClusters.size()*4;
    QuadMeshCluster *clusters = new QuadMeshCluster[MAX_NUM_MESH_CLUSTERS]; //FIXME
    std::atomic<uint32_t> numClusters(extractClusters.size());
    
    for (uint32_t i=0;i<numClusters;i++)
      clusters[i] = extractClusters[i];
    
    const uint32_t numClustersMaxRes = numClusters;
        
    // === bottom-up merging and creation of new clusters ===

    uint32_t *index_buffer = new uint32_t[numClusters];
    uint32_t *tmp_buffer = new uint32_t[numClusters];
    uint32_t *nearest_neighborID = new uint32_t[numClusters];

    const int SEARCH_RADIUS = 16;

    //uint32_t numClusterQuads = 0;
    for (uint32_t i=0;i<numClusters;i++)
    {
      index_buffer[i] = i;
      clusters[i].lod_root = true;
      //numClusterQuads += clusters[i].quads.size();
    }

    //uint32_t iteration = 0;
    
    const uint32_t MAX_DEPTH_LIMIT = 16;

    uint32_t current_numClusters = numClusters;
    
#if GENERATE_LODS == 1    
    bool generateLODs = true;
#else
    bool generateLODs = false;    
#endif
    
    while(current_numClusters > 1 && generateLODs)
    {      
      for (uint32_t i=0;i<current_numClusters;i++)
        nearest_neighborID[i] = -1;

#if ENABLE_MT_PREPROCESS == 1            
      parallel_for((uint32_t)0, current_numClusters, [&] (const range<uint32_t>& r)
      {
        for (uint32_t c=r.begin();c<r.end();c++)
#else
          for (uint32_t c=0;c<current_numClusters;c++)
#endif          
        {
          // find nearest neighbor
          const uint32_t clusterID = index_buffer[c];
          const BBox3f cluster_bounds = clusters[clusterID].bounds;
          int nn = -1;
#if MIN_AREA_DISTANCE_FCT == 1          
          float min_area = pos_inf;

          int start = std::max((int)c-SEARCH_RADIUS,0);
          int end   = std::min((int)c+SEARCH_RADIUS+1,(int)current_numClusters);
          int plus  = 1;
          for (int i=start;i!=end;i+=plus)            
            if (i != c && index_buffer[i] != -1)
            {
              const uint32_t merge_clusterID = index_buffer[i];
              BBox3f bounds = clusters[merge_clusterID].bounds;
              if (!intersect(cluster_bounds,bounds).empty() && !clusters[clusterID].isInvalidMergeID(merge_clusterID))
              {
                bounds.extend(cluster_bounds);
                const float areaBounds = area(bounds);
                if (areaBounds < min_area && clusters[clusterID].neighborID != merge_clusterID)
                {
                  min_area = areaBounds;
                  nn = i;
                }
              }
            }
              
#else
          uint32_t numSharedBorderVertices = 0;
          for (int i=std::max((int)c-SEARCH_RADIUS,0);i<std::min((int)c+SEARCH_RADIUS+1,(int)current_numClusters);i++)
            if (i != c && index_buffer[i] != -1)
            {
              const uint32_t merge_clusterID = index_buffer[i];
              const BBox3f bounds = clusters[merge_clusterID].bounds;
              if (!intersect(cluster_bounds,bounds).empty() && clusters[clusterID].neighborID != merge_clusterID)
              {
                const uint32_t num = getNumSharedBorderVertices(clusters[clusterID],clusters[merge_clusterID]);
                if (num > numSharedBorderVertices)
                {
                  numSharedBorderVertices = num;
                  nn = i;
                }
              }
            }
#endif          
          nearest_neighborID[c] = nn;                  
        }
#if ENABLE_MT_PREPROCESS == 1                    
      });
#endif      

      bool merged_pair = false;

#if ENABLE_MT_PREPROCESS == 1                  
      parallel_for(current_numClusters, [&] (uint32_t i)
#else        
                   for (uint32_t i=0;i<current_numClusters;i++)
#endif                     
      {
        if (nearest_neighborID[i] != -1)
        {
          if ( nearest_neighborID[ nearest_neighborID[i] ] == i)
          {
            if ( i < nearest_neighborID[i])
            {
              const uint32_t leftClusterID = index_buffer[i];
              const uint32_t rightClusterID = index_buffer[nearest_neighborID[i]];
              
              DBG_PRINT4("MERGE",leftClusterID,rightClusterID,getSimplificationRatio(clusters[leftClusterID], clusters[rightClusterID]));
              const uint32_t newDepth =  std::max(clusters[leftClusterID].depth,clusters[rightClusterID].depth)+1;
              
              std::vector<QuadMeshCluster> new_clusters;
#if ENABLE_DAG == 1
              bool success = mergeSimplifyQuadMeshClusterDAG( clusters[leftClusterID], clusters[rightClusterID], new_clusters);              
#else              
              bool success = mergeSimplifyQuadMeshCluster( clusters[leftClusterID], clusters[rightClusterID], new_clusters);
#endif              
              DBG_PRINT2(success,newDepth);
              
              if (success && newDepth <= MAX_DEPTH_LIMIT)
              {
                merged_pair = true;
                
                QuadMeshCluster &new_cluster0 = new_clusters[0];
                clusters[leftClusterID].lod_root = false;
                clusters[rightClusterID].lod_root = false;
                
                new_cluster0.depth = newDepth;
                new_cluster0.leftID  = leftClusterID;
                new_cluster0.rightID = rightClusterID;
                new_cluster0.neighborID = -1;
                new_cluster0.initBounds();
                new_cluster0.lod_root = true;
                
                const uint32_t newClusterID0 = numClusters.fetch_add(1);
                if (newClusterID0 >= MAX_NUM_MESH_CLUSTERS) FATAL("MAX_NUM_MESH_CLUSTERS");
                
                clusters[newClusterID0] = new_cluster0;

                tmp_buffer[i] = newClusterID0;

                DBG_PRINT4("MERGE",newClusterID0,leftClusterID,rightClusterID);

                if (new_clusters.size() == 2)
                {
                  DBG_PRINT("SPLIT CASE");
                  QuadMeshCluster &new_cluster1 = new_clusters[1];
                  clusters[leftClusterID].lod_root = false;
                  clusters[rightClusterID].lod_root = false;
                  new_cluster1.depth = newDepth;
                  new_cluster1.leftID  = leftClusterID;
                  new_cluster1.rightID = rightClusterID;
                  new_cluster1.neighborID = newClusterID0;
                  new_cluster1.initBounds();
                  new_cluster1.lod_root = true; // set root flag of neighbor cluster to false to avoid duplicates on LOD selection
                  const uint32_t newClusterID1 = numClusters.fetch_add(1);
                  DBG_PRINT2(newClusterID0,newClusterID1);
                  
                  if (newClusterID1 >= MAX_NUM_MESH_CLUSTERS) FATAL("MAX_NUM_MESH_CLUSTERS");
                  
                  clusters[newClusterID1] = new_cluster1;
                  tmp_buffer[nearest_neighborID[i]] = newClusterID1;

                  /* update neighbor */
                  clusters[newClusterID0].neighborID = newClusterID1;


#if ENABLE_INVALID_MERGE_IDS == 1
                  clusters[newClusterID1].addInvalidMergeID( newClusterID0 );
                  clusters[newClusterID0].addInvalidMergeID( newClusterID1 );                  
#endif
                  
                }
                else                                
                  tmp_buffer[nearest_neighborID[i]] = -1;
              }
              else
              {
                DBG_PRINT4("CANNOT MERGE", leftClusterID, rightClusterID,newDepth);
#if ENABLE_INVALID_MERGE_IDS == 1                
                clusters[ index_buffer[i] ].addInvalidMergeID( index_buffer[nearest_neighborID[i]] );
                clusters[ index_buffer[nearest_neighborID[i]] ].addInvalidMergeID( index_buffer[i] );
#endif
                
                tmp_buffer[i] = index_buffer[i];
                tmp_buffer[nearest_neighborID[i]] = index_buffer[nearest_neighborID[i]];                
                //nearest_neighborID[nearest_neighborID[i]] = -1;
              }
            }
          }
          else
            tmp_buffer[i] = index_buffer[i];              
        }
        else
          tmp_buffer[i] = index_buffer[i];                      
      }
#if ENABLE_MT_PREPROCESS == 1                                     
        );
#endif      
      
      
      uint32_t new_numClusters = 0;
      for (uint32_t i=0;i<current_numClusters;i++)
        if (tmp_buffer[i] != -1)
          index_buffer[new_numClusters++] = tmp_buffer[i];



      uint32_t numTmpRoots = 0;
      for (uint32_t i=0;i<numClusters;i++)      
        if (clusters[i].lod_root)
        {
          numTmpRoots++;
        }
          
      //iteration++;

      if (!merged_pair)
      {
        for (uint32_t i=0;i<current_numClusters;i++)
        {
          if (nearest_neighborID[i] != -1 &&  nearest_neighborID[ nearest_neighborID[i] ] == i)
          {
            DBG_PRINT4(i,nearest_neighborID[i],index_buffer[i],index_buffer[nearest_neighborID[i]]);
            DBG_PRINT(clusters[index_buffer[i]].quads.size());
            DBG_PRINT(clusters[index_buffer[nearest_neighborID[i]]].quads.size());                        
            DBG_PRINT(clusters[index_buffer[i]].neighborID);
          }
          
        }
        break; // couldn't merge any clusters anymore
      }
      
      current_numClusters = new_numClusters;

      if (numTmpRoots != current_numClusters)
      {
        PRINT2(numTmpRoots,current_numClusters);
#if ENABLE_DAG == 0        
        FATAL("numTmpRoots != current_numClusters");
#endif        
      }
    }

    DBG_PRINT("MERGING DONE");    
    // exit(0);
    
    delete [] nearest_neighborID;        
    delete [] tmp_buffer;    
    delete [] index_buffer;
    
    uint32_t numTotalQuadsAllocate = 0;
    uint32_t numTotalVerticesAllocate = 0;
    for (uint32_t i=0;i<numClusters;i++)
    {
      numTotalQuadsAllocate += clusters[i].quads.size();
      numTotalVerticesAllocate += clusters[i].vertices.size();
    }
    DBG_PRINT2(numTotalQuadsAllocate,numTotalVerticesAllocate);
    
    // === allocate LossyCompressedMesh in USM ===
    
    LossyCompressedMesh *lcm = (LossyCompressedMesh *)alignedUSMMalloc(sizeof(LossyCompressedMesh),64,EMBREE_USM_SHARED);
    
    lcm->bounds             = geometryBounds;
    lcm->numQuads           = numQuads;
    lcm->numVertices        = mesh->numVertices;
    lcm->geomID             = geomID;
    
#if ALLOC_DEVICE_MEMORY == 1    
    EmbreeUSMMode mode = EmbreeUSMMode::EMBREE_DEVICE_READ_WRITE;
#else    
    EmbreeUSMMode mode = EmbreeUSMMode::EMBREE_USM_SHARED;
#endif
    
    lcm->compressedVertices = (CompressedVertex*)alignedUSMMalloc(sizeof(CompressedVertex)*numTotalVerticesAllocate,64,mode); // FIXME
    lcm->compressedIndices  = (CompressedQuadIndices*)alignedUSMMalloc(sizeof(CompressedQuadIndices)*numTotalQuadsAllocate,64,mode); //FIXME    


    PRINT( sizeof(CompressedVertex)*numTotalVerticesAllocate + sizeof(CompressedQuadIndices)*numTotalQuadsAllocate);
    
    CompressedVertex* compressedVertices = new CompressedVertex[numTotalVerticesAllocate];
    CompressedQuadIndices *compressedIndices = new CompressedQuadIndices[numTotalQuadsAllocate];
        
    
    uint32_t globalCompressedVertexOffset = 0;
    uint32_t globalCompressedIndexOffset = 0;

    // === quantize vertices with respect to geometry bounding box ===
    
    const Vec3f geometry_lower    = geometryBounds.lower;
    const Vec3f geometry_diag     = geometryBounds.size();
    const Vec3f geometry_inv_diag = geometry_diag != Vec3fa(0.0f) ? Vec3fa(1.0f) / geometry_diag : Vec3fa(0.0f);

    uint32_t roots = 0;
    uint32_t maxDepth = 0;
    for (uint32_t c=0;c<numClusters;c++)
    {
      maxDepth = std::max(clusters[c].depth,maxDepth);
      LossyCompressedMeshCluster compressed_cluster;      
      compressed_cluster.numQuads  = clusters[c].quads.size();
      compressed_cluster.numBlocks = LossyCompressedMeshCluster::getDecompressedSizeInBytes(compressed_cluster.numQuads)/64;

      //PRINT2( (int)compressed_cluster.numQuads, (int)compressed_cluster.numBlocks );
      
      BBox3f cluster_bounds = clusters[c].bounds;

#if ENABLE_DAG == 1      
      if (clusters[c].neighborID != -1)
      {
        cluster_bounds.extend( clusters[ clusters[c].neighborID ].bounds );
      }
#endif
      
      compressed_cluster.lod_level = clusters[c].depth;
      compressed_cluster.tmp = 0;

#if 0      
      compressed_cluster.bounds = CompressedAABB3f( CompressedVertex(cluster_bounds.lower,geometry_lower,geometry_inv_diag),
                                                    CompressedVertex(cluster_bounds.upper,geometry_lower,geometry_inv_diag) );
#else
      compressed_cluster.bounds.init(); 
#endif      
      
      compressed_cluster.leftID = (clusters[c].leftID != -1) ? global_start_lcm_clusterID+clusters[c].leftID : -1;
      compressed_cluster.rightID = (clusters[c].rightID != -1) ? global_start_lcm_clusterID+clusters[c].rightID : -1;
      compressed_cluster.neighborID = (clusters[c].neighborID != -1) ? global_start_lcm_clusterID+clusters[c].neighborID : -1;

      if (compressed_cluster.leftID == -1 && compressed_cluster.rightID != -1) FATAL("leftID,rightID");
      if (compressed_cluster.leftID != -1 && compressed_cluster.rightID == -1) FATAL("rightID,leftID");
        
      if (clusters[c].neighborID != -1 && clusters[ clusters[c].neighborID ].neighborID != c)
        FATAL("clusters[c].neighborID");


      if (clusters[c].neighborID != -1 && (clusters[ clusters[c].neighborID ].leftID != clusters[c].leftID || clusters[ clusters[c].neighborID ].rightID != clusters[c].rightID))
        FATAL("clusters[c].neighborID");
      
      compressed_cluster.offsetIndices  = globalCompressedIndexOffset;      
      compressed_cluster.offsetVertices = globalCompressedVertexOffset;
      compressed_cluster.mesh = lcm;

      for (uint32_t i=0;i<clusters[c].quads.size();i++)
      {
        const uint32_t v0 = clusters[c].quads[i].v0;
        const uint32_t v1 = clusters[c].quads[i].v1;
        const uint32_t v2 = clusters[c].quads[i].v2;
        const uint32_t v3 = clusters[c].quads[i].v3;        
        compressedIndices[ globalCompressedIndexOffset++ ] = CompressedQuadIndices(v0,v1,v2,v3);
        if ( globalCompressedIndexOffset > numTotalQuadsAllocate ) FATAL("numTotalQuadsAllocate");        
      }
      compressed_cluster.tmp =  clusters[c].vertices.size(); //(clusters[c].quads.size()*sizeof(CompressedQuadIndices) + clusters[c].vertices.size()*sizeof(CompressedVertex)+63)/64;
      for (uint32_t i=0;i<clusters[c].vertices.size();i++)
      {

        const CompressedVertex v = CompressedVertex(clusters[c].vertices[i],geometry_lower,geometry_inv_diag);
        compressed_cluster.bounds.extend(v);        
        compressedVertices[ globalCompressedVertexOffset++ ] = v;
        if ( globalCompressedVertexOffset > numTotalVerticesAllocate ) FATAL("numTotalVerticesAllocate");        
      }
      
      
      lcm_ptrs.push_back(lcm);      
      const uint32_t lcm_clusterID = lcm_clusters.size();      
      lcm_clusters.push_back(compressed_cluster);
      if (clusters[c].lod_root)
      {
        lcm_clusterRootIDs.push_back(lcm_clusterID);
        roots++;
      }
      
      numDecompressedBlocks += compressed_cluster.numBlocks;      
    }
    if (numTotalVerticesAllocate != globalCompressedVertexOffset) FATAL("numTotalVerticesAllocate != globalCompressedVertexOffset");
    if (numTotalQuadsAllocate    != globalCompressedIndexOffset) FATAL("numTotalQuadsAllocate != globalCompressedIndexOffset");

    DBG_PRINT3(compressedVertices,compressedVertices,roots);
    queue.memcpy(lcm->compressedVertices,compressedVertices,sizeof(CompressedVertex)*globalCompressedVertexOffset);
    queue.memcpy(lcm->compressedIndices,compressedIndices,sizeof(CompressedQuadIndices)*globalCompressedIndexOffset);
    
    gpu::waitOnQueueAndCatchException(*global_gpu_queue);        

    //const size_t uncompressedSizeMeshBytes = mesh->numVertices * sizeof(Vec3f) + mesh->numQuads * sizeof(uint32_t) * 4;
    const size_t compressedSizeMeshBytes = sizeof(CompressedVertex)*numTotalVerticesAllocate + sizeof(CompressedQuadIndices)*numTotalQuadsAllocate;
    const size_t clusterSizeBytes = numClusters*sizeof(LossyCompressedMeshCluster);
    PRINT3(numTotalQuadsAllocate,numTotalVerticesAllocate,numClusters);
    DBG_PRINT4(uncompressedSizeMeshBytes,compressedSizeMeshBytes,(float)compressedSizeMeshBytes/uncompressedSizeMeshBytes,clusterSizeBytes);

    totalCompressedSize += compressedSizeMeshBytes + clusterSizeBytes;
    PRINT2(globalCompressedIndexOffset*sizeof(CompressedQuadIndices),globalCompressedVertexOffset*sizeof(CompressedVertex));
    PRINT(globalCompressedIndexOffset*sizeof(CompressedQuadIndices) + globalCompressedVertexOffset*sizeof(CompressedVertex));
    
    DBG_PRINT2(roots,maxDepth);

    delete [] compressedIndices;
    delete [] compressedVertices;    
    delete [] clusters;

    DBG_PRINT("CLUSTER PROCESSING DONE");    
    //exit(0);
    
    return Vec2i(numClustersMaxRes,numDecompressedBlocks);
  }  


  uint32_t findVertex(std::vector<CompressedVertex> &vertices, const CompressedVertex &cv)
  {
    for (uint32_t i=0;i<vertices.size();i++)
      if (cv == vertices[i])
        return i;
    vertices.push_back(cv);
    return vertices.size()-1;
  }
  
  std::vector<Quad> extractQuads(TriangleMesh &mesh)
  {
    std::vector<Quad> quads;

    for (size_t i=0; i<mesh.triangles.size(); i++)
    {
      const int a0 = mesh.triangles[i+0].v0;
      const int a1 = mesh.triangles[i+0].v1;
      const int a2 = mesh.triangles[i+0].v2;
      if (i+1 == mesh.triangles.size()) {
        quads.push_back(Quad(a0,a1,a2,a2));
        continue;
      }
      
      const int b0 = mesh.triangles[i+1].v0;
      const int b1 = mesh.triangles[i+1].v1;
      const int b2 = mesh.triangles[i+1].v2;
      const std::pair<int,int> q = quad_index3(a0,a1,a2,b0,b1,b2);
      const int a3 = q.second;
      if (a3 == -1) {
        quads.push_back(Quad(a0,a1,a2,a2));
        continue;
      }
      
      if      (q.first == -1) quads.push_back(Quad(a1,a2,a3,a0));
      else if (q.first ==  0) quads.push_back(Quad(a3,a1,a2,a0));
      else if (q.first ==  1) quads.push_back(Quad(a0,a1,a3,a2));
      else if (q.first ==  2) quads.push_back(Quad(a1,a2,a3,a0)); 
      i++;
    }
    return quads;
  }  
  
} // namespace embree
