#include "nanite_geometry_device.h"
#include "../../kernels/rthwif/builder/gpu/lcgbp.h"
#include "../../kernels/rthwif/builder/gpu/morton.h"

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


namespace embree {

  uint findVertex(std::vector<Vec3f> &vertices, const Vec3f &cv)
  {
    for (uint i=0;i<vertices.size();i++)
      if (cv == vertices[i])
        return i;
    vertices.push_back(cv);
    return vertices.size()-1;
  }

  void countVertexIDs(std::vector<uint> &vertices, const uint cv)
  {
    for (uint i=0;i<vertices.size();i++)
      if (cv == vertices[i])
        return;
    vertices.push_back(cv);
  }
  
  struct Triangle {
    uint v0,v1,v2;

    __forceinline Triangle () {}
    __forceinline Triangle (const uint v0, const uint v1, const uint v2) : v0(v0), v1(v1), v2(v2) {}

    __forceinline bool valid()
    {
      if (v0 != v1 && v1 != v2 && v2 != v0) return true;
      return false;
    }
  };

  struct Quad {
    uint v0,v1,v2,v3;

    __forceinline Quad () {}
    __forceinline Quad (const uint v0, const uint v1, const uint v2, const uint v3) : v0(v0), v1(v1), v2(v2), v3(v3)  {}
  };

  struct TriangleMesh {
    std::vector<Triangle> triangles;
    std::vector<Vec3f> vertices;
  };
  
  struct QuadMeshCluster {
    BBox3f bounds;
    bool lod_root;
    uint leftID, rightID, neighborID;
    uint depth;
    std::vector<Quad> quads;
    std::vector<Vec3f> vertices;

    __forceinline QuadMeshCluster() : bounds(empty),lod_root(false),leftID(-1), rightID(-1), neighborID(-1), depth(0) {}

    __forceinline bool isLeaf() { return leftID == -1 || rightID == -1; }

    __forceinline void initBounds()
    {
      bounds = BBox3f(empty);
      for (uint i=0;i<vertices.size();i++)
        bounds.extend(vertices[i]);
    }
    
    void reorderMorton();
    uint computeTriangleStrip();
    bool split(QuadMeshCluster &left, QuadMeshCluster &right);
    
  };

  
  void QuadMeshCluster::reorderMorton()
  {
    const uint numQuads = quads.size();
    Quad *new_quads = new Quad[numQuads];
    BBox3f centroidBounds(empty);

    for (uint i=0;i<numQuads;i++)
    {
      const uint v0 = quads[i].v0;
      const uint v1 = quads[i].v1;
      const uint v2 = quads[i].v2;
      const uint v3 = quads[i].v3;

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

    std::vector<gpu::MortonCodePrimitive64x32Bits3D> mcodes;

    for (uint i=0;i<numQuads;i++)
    {
      const uint v0 = quads[i].v0;
      const uint v1 = quads[i].v1;
      const uint v2 = quads[i].v2;
      const uint v3 = quads[i].v3;

      const Vec3f &vtx0 = vertices[v0];
      const Vec3f &vtx1 = vertices[v1];
      const Vec3f &vtx2 = vertices[v2];
      const Vec3f &vtx3 = vertices[v3];

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

    std::sort(mcodes.begin(), mcodes.end()); 
    for (uint i=0;i<numQuads;i++)
      new_quads[i] = quads[mcodes[i].getIndex()];

    for (uint i=0;i<numQuads;i++)
      quads[i] = new_quads[i];
    delete [] new_quads;
  }

  uint QuadMeshCluster::computeTriangleStrip()
  {
    TriangleMesh mesh;
        
    // === cluster0 ===
    for (uint i=0;i<quads.size();i++)
    {
      uint v0 = findVertex(mesh.vertices, vertices[ quads[i].v0 ]);
      uint v1 = findVertex(mesh.vertices, vertices[ quads[i].v1 ]);
      uint v2 = findVertex(mesh.vertices, vertices[ quads[i].v2 ]);
      uint v3 = findVertex(mesh.vertices, vertices[ quads[i].v3 ]);

      Triangle tri0(v0,v1,v3);
      Triangle tri1(v1,v2,v3);
      if (tri0.valid()) mesh.triangles.push_back(tri0);
      if (tri1.valid()) mesh.triangles.push_back(tri1);            
    }

    const uint numTriangles = mesh.triangles.size();
    const uint numVertices  = mesh.vertices.size();
    const uint numIndices   = numTriangles*3;

    uint *old_indices = (uint*)&*mesh.triangles.begin();    
    std::vector<unsigned int> new_index_list(meshopt_stripifyBound(numIndices));
    uint *new_indices = (uint*)&*new_index_list.begin();    

#if 0
    meshopt_optimizeVertexCacheStrip(new_indices, old_indices, numIndices, numVertices);
    for (uint i=0;i<numIndices;i++)
      old_indices[i] = new_indices[i];
#endif
    
    unsigned int restart_index = ~0u;
    size_t strip_size = meshopt_stripify(new_indices, old_indices, numIndices, numVertices, restart_index);
    return strip_size;
  }

  bool QuadMeshCluster::split(QuadMeshCluster &left, QuadMeshCluster &right)
  {
    reorderMorton();
    
    uint mid = quads.size() / 2;
    for (uint i=0;i<mid;i++)
    {
      uint v0 = findVertex(left.vertices, vertices[ quads[i].v0 ]);
      uint v1 = findVertex(left.vertices, vertices[ quads[i].v1 ]);
      uint v2 = findVertex(left.vertices, vertices[ quads[i].v2 ]);
      uint v3 = findVertex(left.vertices, vertices[ quads[i].v3 ]);

      left.quads.push_back(Quad(v0,v1,v2,v3));
    }

    for (uint i=mid;i<quads.size();i++)
    {
      uint v0 = findVertex(right.vertices, vertices[ quads[i].v0 ]);
      uint v1 = findVertex(right.vertices, vertices[ quads[i].v1 ]);
      uint v2 = findVertex(right.vertices, vertices[ quads[i].v2 ]);
      uint v3 = findVertex(right.vertices, vertices[ quads[i].v3 ]);

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


  float getSimplificationRatio(QuadMeshCluster &cluster0,QuadMeshCluster &cluster1)
  {
    TriangleMesh mesh;    
    
    // === cluster0 ===
    for (uint i=0;i<cluster0.quads.size();i++)
    {
      uint v0 = findVertex(mesh.vertices, cluster0.vertices[ cluster0.quads[i].v0 ]);
      uint v1 = findVertex(mesh.vertices, cluster0.vertices[ cluster0.quads[i].v1 ]);
      uint v2 = findVertex(mesh.vertices, cluster0.vertices[ cluster0.quads[i].v2 ]);
      uint v3 = findVertex(mesh.vertices, cluster0.vertices[ cluster0.quads[i].v3 ]);

      Triangle tri0(v0,v1,v3);
      Triangle tri1(v1,v2,v3);
      if (tri0.valid()) mesh.triangles.push_back(tri0);
      if (tri1.valid()) mesh.triangles.push_back(tri1);            
    }

    // === cluster1 ===
    for (uint i=0;i<cluster1.quads.size();i++)
    {
      uint v0 = findVertex(mesh.vertices, cluster1.vertices[ cluster1.quads[i].v0 ]);
      uint v1 = findVertex(mesh.vertices, cluster1.vertices[ cluster1.quads[i].v1 ]);
      uint v2 = findVertex(mesh.vertices, cluster1.vertices[ cluster1.quads[i].v2 ]);
      uint v3 = findVertex(mesh.vertices, cluster1.vertices[ cluster1.quads[i].v3 ]);

      Triangle tri0(v0,v1,v3);
      Triangle tri1(v1,v2,v3);
      if (tri0.valid()) mesh.triangles.push_back(tri0);
      if (tri1.valid()) mesh.triangles.push_back(tri1);            
    }

    const uint numTriangles = mesh.triangles.size();
    const uint numVertices  = mesh.vertices.size();
    const uint numIndices   = numTriangles*3;

    Triangle *new_triangles = new Triangle[numTriangles];    
    Triangle *triangles     = &*mesh.triangles.begin();
    Vec3f *vertices         = &*mesh.vertices.begin();

    const float REDUCTION_FACTOR = 0.5f;
    const uint expectedTriangles = ceilf(numTriangles * REDUCTION_FACTOR); 
    
    const float max_error = 0.1f;
    
    float result_error = 0.0f;
    const uint opts = meshopt_SimplifyLockBorder;
    const size_t new_numIndices = meshopt_simplify((uint*)new_triangles,(uint*)triangles,numIndices,(float*)vertices,numVertices,sizeof(Vec3f),expectedTriangles*3,max_error,opts,&result_error);
    const size_t new_numTriangles = new_numIndices/3;                
    
    delete [] new_triangles;

    return (float)new_numTriangles / numTriangles;
  }


  bool mergeSimplifyQuadMeshCluster(QuadMeshCluster &cluster0,QuadMeshCluster &cluster1, std::vector<QuadMeshCluster> &quadMeshes)
  {
    QuadMeshCluster quadMesh;
    TriangleMesh mesh;
    
    
    // === cluster0 ===
    for (uint i=0;i<cluster0.quads.size();i++)
    {
      uint v0 = findVertex(mesh.vertices, cluster0.vertices[ cluster0.quads[i].v0 ]);
      uint v1 = findVertex(mesh.vertices, cluster0.vertices[ cluster0.quads[i].v1 ]);
      uint v2 = findVertex(mesh.vertices, cluster0.vertices[ cluster0.quads[i].v2 ]);
      uint v3 = findVertex(mesh.vertices, cluster0.vertices[ cluster0.quads[i].v3 ]);

      Triangle tri0(v0,v1,v3);
      Triangle tri1(v1,v2,v3);
      if (tri0.valid()) mesh.triangles.push_back(tri0);
      if (tri1.valid()) mesh.triangles.push_back(tri1);            
    }

    // === cluster1 ===
    for (uint i=0;i<cluster1.quads.size();i++)
    {
      uint v0 = findVertex(mesh.vertices, cluster1.vertices[ cluster1.quads[i].v0 ]);
      uint v1 = findVertex(mesh.vertices, cluster1.vertices[ cluster1.quads[i].v1 ]);
      uint v2 = findVertex(mesh.vertices, cluster1.vertices[ cluster1.quads[i].v2 ]);
      uint v3 = findVertex(mesh.vertices, cluster1.vertices[ cluster1.quads[i].v3 ]);

      Triangle tri0(v0,v1,v3);
      Triangle tri1(v1,v2,v3);
      if (tri0.valid()) mesh.triangles.push_back(tri0);
      if (tri1.valid()) mesh.triangles.push_back(tri1);            
    }
    
    DBG_PRINT(mesh.vertices.size());
    DBG_PRINT(mesh.triangles.size());

    const uint numTriangles = mesh.triangles.size();
    const uint numVertices  = mesh.vertices.size();
    const uint numIndices   = numTriangles*3;

    Triangle *new_triangles = new Triangle[numTriangles];    
    Triangle *triangles     = &*mesh.triangles.begin();
    Vec3f *vertices         = &*mesh.vertices.begin();

    const float REDUCTION_FACTOR = 0.5f;
    uint expectedTriangles = floorf((LossyCompressedMeshCluster::MAX_QUADS_PER_CLUSTER * 4) * REDUCTION_FACTOR);
    
    uint iterations = 0;
    while(1)
    {
      iterations++;

      uint opts = meshopt_SimplifyLockBorder;
#if UNLOCK_BORDER == 1
      if (iterations > 5)
      {
        opts = 0;
      }
#endif            
      if (iterations > 10) return false;
      bool retry = false;      
      float result_error = 0.0f;
      const size_t new_numIndices = meshopt_simplify((uint*)new_triangles,(uint*)triangles,numIndices,(float*)vertices,numVertices,sizeof(Vec3f),expectedTriangles*3,0.1f,opts,&result_error);

      const size_t new_numTriangles = new_numIndices/3;
      DBG_PRINT3(expectedTriangles,new_numTriangles,result_error);

      std::vector<uint> new_vertices;
      for (uint i=0;i<new_numTriangles;i++)
      {
        countVertexIDs(new_vertices, new_triangles[i].v0);
        countVertexIDs(new_vertices, new_triangles[i].v1);
        countVertexIDs(new_vertices, new_triangles[i].v2);      
      }      
      if (new_vertices.size() > 256)
      {
        DBG_PRINT2("new_vertices.size()",new_vertices.size());
        retry = true;
      }


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
      if (quadMesh.vertices.size() > 256) { DBG_PRINT("RETRY quadMesh.vertices.size()"); retry = true; }
      if (retry)
      {
        quadMesh.vertices.clear();
        quadMesh.quads.clear();
        DBG_PRINT2(quadMesh.vertices.size(),quadMesh.quads.size());
        expectedTriangles -= std::max((uint)2,expectedTriangles/10);
      }
      else
        break;
    }
    //exit(0);
    DBG_PRINT2(quadMesh.quads.size(),quadMesh.vertices.size());

    delete [] new_triangles;

    quadMeshes.push_back(quadMesh);
    return true;
  }



  // ==========================================================

  bool mergeSimplifyQuadMeshClusterDAG(QuadMeshCluster &cluster0,QuadMeshCluster &cluster1, std::vector<QuadMeshCluster> &quadMeshes)
  {
    DBG_PRINT3("CLUSTER MERGING",cluster0.quads.size(),cluster1.quads.size());
    QuadMeshCluster quadMesh;
    TriangleMesh mesh;
    
    
    // === cluster0 ===
    for (uint i=0;i<cluster0.quads.size();i++)
    {
      uint v0 = findVertex(mesh.vertices, cluster0.vertices[ cluster0.quads[i].v0 ]);
      uint v1 = findVertex(mesh.vertices, cluster0.vertices[ cluster0.quads[i].v1 ]);
      uint v2 = findVertex(mesh.vertices, cluster0.vertices[ cluster0.quads[i].v2 ]);
      uint v3 = findVertex(mesh.vertices, cluster0.vertices[ cluster0.quads[i].v3 ]);

      Triangle tri0(v0,v1,v3);
      Triangle tri1(v1,v2,v3);
      if (tri0.valid()) mesh.triangles.push_back(tri0);
      if (tri1.valid()) mesh.triangles.push_back(tri1);            
    }

    // === cluster1 ===
    for (uint i=0;i<cluster1.quads.size();i++)
    {
      uint v0 = findVertex(mesh.vertices, cluster1.vertices[ cluster1.quads[i].v0 ]);
      uint v1 = findVertex(mesh.vertices, cluster1.vertices[ cluster1.quads[i].v1 ]);
      uint v2 = findVertex(mesh.vertices, cluster1.vertices[ cluster1.quads[i].v2 ]);
      uint v3 = findVertex(mesh.vertices, cluster1.vertices[ cluster1.quads[i].v3 ]);

      Triangle tri0(v0,v1,v3);
      Triangle tri1(v1,v2,v3);
      if (tri0.valid()) mesh.triangles.push_back(tri0);
      if (tri1.valid()) mesh.triangles.push_back(tri1);            
    }
    
    DBG_PRINT(mesh.vertices.size());
    DBG_PRINT(mesh.triangles.size());

    const uint numTriangles = mesh.triangles.size();
    const uint numVertices  = mesh.vertices.size();
    const uint numIndices   = numTriangles*3;

    Triangle *new_triangles = new Triangle[numTriangles];    
    Triangle *triangles     = &*mesh.triangles.begin();
    Vec3f *vertices         = &*mesh.vertices.begin();

    const float REDUCTION_FACTOR = 0.5f;
    uint expectedTriangles = ceilf(numTriangles * REDUCTION_FACTOR); //floorf((LossyCompressedMeshCluster::MAX_QUADS_PER_CLUSTER * 4) * REDUCTION_FACTOR);
    
    uint iterations = 0;
    float max_error = 0.1f;
    const float REDUCTION_THRESHOLD = 0.9f;
    //while(1)
    {
      iterations++;

      uint opts = meshopt_SimplifyLockBorder;

      bool retry = false;
      size_t new_numIndices = 0;
      while(max_error < 1.0f) {
        float result_error = 0.0f;
        new_numIndices = meshopt_simplify((uint*)new_triangles,(uint*)triangles,numIndices,(float*)vertices,numVertices,sizeof(Vec3f),expectedTriangles*3,max_error,opts,&result_error);
        const size_t new_numTriangles = new_numIndices/3;                
        DBG_PRINT5("SIMPLIFY",new_numTriangles,numTriangles,expectedTriangles,(float)new_numTriangles / numTriangles);
        if ((float)new_numTriangles / numTriangles <= REDUCTION_THRESHOLD) break;        
        //expectedTriangles += std::max(expectedTriangles/10,(uint)1);
        max_error *= 2;
      } 

      const size_t new_numTriangles = new_numIndices/3;

      if ((float)new_numTriangles / numTriangles > REDUCTION_THRESHOLD) { PRINT4("NOT ENOUGH REDUCTION",numTriangles,new_numTriangles,expectedTriangles); return false; }
      
      DBG_PRINT2(expectedTriangles,new_numTriangles);

      std::vector<uint> new_vertices;
      for (uint i=0;i<new_numTriangles;i++)
      {
        countVertexIDs(new_vertices, new_triangles[i].v0);
        countVertexIDs(new_vertices, new_triangles[i].v1);
        countVertexIDs(new_vertices, new_triangles[i].v2);      
      }      
      if (new_vertices.size() > 256)
      {
        DBG_PRINT2("new_vertices.size()",new_vertices.size());
        retry = true;
      }


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
      if (quadMesh.vertices.size() > 256) { DBG_PRINT("RETRY quadMesh.vertices.size()"); retry = true; }
      if (retry)
      {
        DBG_PRINT("SPLIT DAG");
        QuadMeshCluster left, right;
        const bool split_success = quadMesh.split(left,right);
        PRINT3(split_success,left.quads.size(),right.quads.size());
        
        if (split_success)
        {
          quadMeshes.push_back(left);
          quadMeshes.push_back(right);
        }
        else
          FATAL("SPLIT FAILED");                  
      }
      else
        quadMeshes.push_back(quadMesh);        
      // else
      //   break;
    }
    //exit(0);
    DBG_PRINT(quadMeshes.size());
      for (uint i=0;i<quadMeshes.size();i++)
        DBG_PRINT2(quadMeshes[i].quads.size(),quadMeshes[i].vertices.size());

    delete [] new_triangles;

    
    return true;
  }
  


  // ==========================================================
  
  
  
  
  __forceinline uint remap_vtx_index(const uint v, std::map<uint,uint> &index_map, uint &numLocalIndices)
  {
    auto e = index_map.find(v);
    if (e != index_map.end()) return e->second;
    const uint ID = numLocalIndices++;
    index_map[v] = ID;
    return ID;
  }

  struct HierarchyRange
  {
    gpu::Range range;
    uint parent, left, right;
    uint counter, clusterID;

    __forceinline HierarchyRange(const gpu::Range &range, const uint parent = -1) : range(range), parent(parent), left(-1), right(-1), counter(0), clusterID(-1) {}

    __forceinline bool isLeaf() { return left == -1 || right == -1; }
  };
  
  struct BVH2Ploc
  {
    BBox3f bounds;
    uint leftID,rightID;
    uint numLeafPrims;

    BVH2Ploc() {}
    
    BVH2Ploc(const BBox3f &bounds, uint ID) : bounds(bounds),leftID(ID),rightID(-1),numLeafPrims(1) {}

    BVH2Ploc(const BVH2Ploc &left, const BVH2Ploc &right, uint lID, uint rID) {
      bounds = left.bounds;
      bounds.extend(right.bounds);
      leftID = lID;
      rightID = rID;
      numLeafPrims = left.numLeafPrims + right.numLeafPrims;
    }
    
    __forceinline bool isLeaf() { return rightID == -1; }
    __forceinline uint leafID() { return leftID; }
    __forceinline uint items() { return numLeafPrims; }
    
  };

  void extractIDs(const uint currentID, BVH2Ploc *bvh, std::vector<uint> &IDs)
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
  
  void extractClusters(const uint currentID, BVH2Ploc *bvh, std::vector<QuadMeshCluster> &clusters, ISPCQuadMesh* mesh, const uint threshold)
  {
    if (bvh[currentID].items() < threshold || bvh[currentID].isLeaf())
    {
      std::vector<uint> IDs;
      extractIDs(currentID,bvh,IDs);

      std::map<uint,uint> index_map;
      uint numLocalIndices = 0;
      bool fits = true;
      for (uint j=0;j<IDs.size();j++)
      {
        const uint index = IDs[j];
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
          fits = false;
          break;
        }
      }
      
      if (fits)
      {
        QuadMeshCluster cluster;

        for (uint j=0;j<IDs.size();j++)
        {
          const uint index = IDs[j];
          const uint v0 = mesh->quads[index].v0;
          const uint v1 = mesh->quads[index].v1;
          const uint v2 = mesh->quads[index].v2;
          const uint v3 = mesh->quads[index].v3;

          const uint remaped_v0 =  remap_vtx_index(v0,index_map,numLocalIndices);
          const uint remaped_v1 =  remap_vtx_index(v1,index_map,numLocalIndices);
          const uint remaped_v2 =  remap_vtx_index(v2,index_map,numLocalIndices);
          const uint remaped_v3 =  remap_vtx_index(v3,index_map,numLocalIndices);

          cluster.quads.push_back(Quad(remaped_v0,remaped_v1,remaped_v2,remaped_v3));
          
        }

        cluster.vertices.resize(numLocalIndices);
      
        for (std::map<uint,uint>::iterator i=index_map.begin(); i != index_map.end(); i++)
        {
          const uint old_v = (*i).first;
          const uint new_v = (*i).second;
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

  std::vector<QuadMeshCluster> extractRangesPLOC(std::vector<gpu::MortonCodePrimitive64x32Bits3D> &mcodes,const std::vector<BBox3f> &plocBounds, ISPCQuadMesh* mesh, const uint threshold)
  {
    std::vector<QuadMeshCluster> clusters;
    
    const uint numPrims = mcodes.size();
    DBG_PRINT(numPrims);
    
    uint *index_buffer = new uint[numPrims];
    uint *tmp_buffer = new uint[numPrims];
    uint *nearest_neighborID = new uint[numPrims];

    BVH2Ploc *bvh2 = new BVH2Ploc[numPrims*2];

    for (uint i=0;i<numPrims;i++)
    {
      const uint ID = mcodes[i].getIndex();
      index_buffer[i] = i;
      bvh2[i] = BVH2Ploc(plocBounds[ID],ID);
    }

    const int SEARCH_RADIUS = 16;

    std::atomic<uint> numPrimitives(numPrims);

    uint cur_numPrims = numPrims;
    while(cur_numPrims > 1)
    {
      DBG_PRINT(cur_numPrims);
      for (uint i=0;i<cur_numPrims;i++)
        nearest_neighborID[i] = -1;

      parallel_for((uint)0, cur_numPrims, [&] (const range<uint>& r)
      {
        for (uint c=r.begin();c<r.end();c++)
        {
          // find nearest neighbor
          const uint ID = index_buffer[c];
          const BBox3f bounds = bvh2[ID].bounds;
          float min_area = pos_inf;
          int nn = -1;
          for (int i=std::max((int)c-SEARCH_RADIUS,0);i<std::min((int)c+SEARCH_RADIUS+1,(int)cur_numPrims);i++)
            if (i != c && index_buffer[i] != -1)
            {
              const uint merge_ID = index_buffer[i];
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
      });

      parallel_for((uint)0, cur_numPrims, [&] (const range<uint>& r)
      {
        for (uint i=r.begin();i<r.end();i++)
        {
          if (nearest_neighborID[i] != -1)
          {
            if ( nearest_neighborID[ nearest_neighborID[i] ] == i)
            {
              if ( i < nearest_neighborID[i])
              {
                const uint leftID = index_buffer[i];
                const uint rightID = index_buffer[nearest_neighborID[i]];
                const uint newID = numPrimitives.fetch_add(1);
                bvh2[newID] = BVH2Ploc(bvh2[leftID],bvh2[rightID],leftID,rightID);
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
      });
      
      uint new_cur_numPrims = 0;
      for (uint i=0;i<cur_numPrims;i++)
        if (tmp_buffer[i] != -1)
          index_buffer[new_cur_numPrims++] = tmp_buffer[i];

      if (cur_numPrims == new_cur_numPrims)
        FATAL("NO PLOC PRIM REDUCTION IN ITERATION");
      
      cur_numPrims = new_cur_numPrims;      
    }

    uint rootID = index_buffer[0];
    

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
  
  


  void extractRanges(const uint currentID, const gpu::MortonCodePrimitive64x32Bits3D *const mcodes, std::vector<HierarchyRange> &ranges, std::vector<uint> &leafIDs, ISPCQuadMesh* mesh, uint &numTotalVertices, const uint threshold)
  {
    if (ranges[currentID].range.size() < threshold)
    {
      std::map<uint,uint> index_map;
      uint numLocalIndices = 0;
      bool fits = true;
      for (uint j=ranges[currentID].range.start;j<ranges[currentID].range.end;j++)
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

    const uint leftID = ranges.size();
    ranges.push_back(HierarchyRange(left,currentID));
    const uint rightID = ranges.size();
    ranges.push_back(HierarchyRange(right,currentID));

    ranges[currentID].left = leftID;
    ranges[currentID].right = rightID;
    
    extractRanges(leftID,mcodes,ranges,leafIDs,mesh,numTotalVertices,threshold);
    extractRanges(rightID,mcodes,ranges,leafIDs,mesh,numTotalVertices,threshold);      
  }

  void extractClusterRootIDs(const uint currentID, std::vector<HierarchyRange> &ranges, std::vector<uint> &clusterRootIDs)
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

      
  uint convertISPCQuadMesh(ISPCQuadMesh* mesh, RTCScene scene, ISPCOBJMaterial *material,const uint geomID,std::vector<LossyCompressedMesh*> &lcm_ptrs,std::vector<LossyCompressedMeshCluster> &lcm_clusters, std::vector<uint> &lcm_clusterRootIDs, size_t &totalCompressedSize, size_t &numDecompressedBlocks, sycl::queue &queue)
  {
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
    std::vector<HierarchyRange> ranges;
    std::vector<uint> leafIDs;
    std::vector<uint> clusterRootIDs;
    std::vector<BBox3f> plocBounds;
    mcodes.reserve(numQuads);
    
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
      plocBounds.push_back(quadBounds);
    }

    // === sort morton codes ===
    
    std::sort(mcodes.begin(), mcodes.end()); 

    // === extract ranges, test range if it fullfills requirements, split if necessary ===
    uint numTotalVertices = 0;

#if 0
    ranges.push_back(HierarchyRange(gpu::Range(0,mcodes.size())));
    extractRanges(0,&*mcodes.begin(),ranges,leafIDs,mesh,numTotalVertices,INITIAL_CREATE_RANGE_THRESHOLD);
    DBG_PRINT(ranges.size());
    DBG_PRINT(leafIDs.size());
    // === create leaf clusters ===
    //size_t totalSizeMicroMesh = 0;
    //size_t totalSizeMicroStrip = 0;

    const uint MAX_NUM_MESH_CLUSTERS = leafIDs.size()*4;
    QuadMeshCluster *clusters = new QuadMeshCluster[MAX_NUM_MESH_CLUSTERS]; //FIXME
    std::atomic<uint> numClusters(0);

    for (uint i=0;i<leafIDs.size();i++) //FIXME leafIDs
    {
      const uint ID = leafIDs[i];
      QuadMeshCluster &cluster = clusters[i];

      std::map<uint,uint> index_map;
      uint numLocalIndices = 0;

      // === remap vertices relative to cluster ===
      //DBG_PRINT2(ranges[ID].range.start,ranges[ID].range.end);
      
      for (uint j=ranges[ID].range.start;j<ranges[ID].range.end;j++)
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

        cluster.quads.push_back(Quad(remaped_v0,remaped_v1,remaped_v2,remaped_v3));
      }
      if (cluster.quads.size() > LossyCompressedMeshCluster::MAX_QUADS_PER_CLUSTER) FATAL("cluster.quads");
      if (numLocalIndices > 256) FATAL("cluster.vertices");
      
      cluster.vertices.resize(numLocalIndices);
      
      for (std::map<uint,uint>::iterator i=index_map.begin(); i != index_map.end(); i++)
      {
        const uint old_v = (*i).first;
        const uint new_v = (*i).second;
        cluster.vertices[new_v] = mesh->positions[0][old_v];
      }
      ranges[ID].clusterID = numClusters;

      cluster.reorderMorton();

#if 0      
      const uint strip_indices = cluster.computeTriangleStrip();
      const uint sizeMicroMesh = cluster.quads.size()*4 + cluster.vertices.size()*6;
      const uint sizeMicroStrip = cluster.vertices.size()*6 + strip_indices; //strip_indices*6;
      //PRINT4(cluster.quads.size()*4,strip_indices,sizeMicroMesh,sizeMicroStrip);
      totalSizeMicroMesh += sizeMicroMesh;
      totalSizeMicroStrip += sizeMicroStrip;
#endif      
      cluster.initBounds();
      cluster.depth = 1;
      //DBG_PRINT2(cluster.quads.size(),cluster.vertices.size());
      numClusters++;
    }
    //PRINT(numClusters);
    if (numClusters != leafIDs.size()) FATAL("numClusters != leafIDs.size()");

#else
    std::vector<QuadMeshCluster> extractClusters = extractRangesPLOC(mcodes,plocBounds,mesh,INITIAL_CREATE_RANGE_THRESHOLD);
    const uint MAX_NUM_MESH_CLUSTERS = extractClusters.size()*4;
    QuadMeshCluster *clusters = new QuadMeshCluster[MAX_NUM_MESH_CLUSTERS]; //FIXME
    std::atomic<uint> numClusters(extractClusters.size());
    
    for (uint i=0;i<numClusters;i++)
      clusters[i] = extractClusters[i];
    
#endif    
    

    const uint numNumClustersMaxRes = numClusters;
    
    //PRINT2(totalSizeMicroMesh,totalSizeMicroStrip);
    
    // === bottom-up merging and creation of new clusters ===

    uint *index_buffer = new uint[numClusters];
    uint *tmp_buffer = new uint[numClusters];
    uint *nearest_neighborID = new uint[numClusters];

    const int SEARCH_RADIUS = 16;

    //uint numClusterQuads = 0;
    for (uint i=0;i<numClusters;i++)
    {
      index_buffer[i] = i;
      clusters[i].lod_root = true;
      //numClusterQuads += clusters[i].quads.size();
    }

    uint iteration = 0;
    
    const uint MAX_DEPTH_LIMIT = 16;

    uint current_numClusters = numClusters;
    
    while(current_numClusters > 1)
    {
      for (uint i=0;i<current_numClusters;i++)
        nearest_neighborID[i] = -1;

      parallel_for((uint)0, current_numClusters, [&] (const range<uint>& r)
      {
        for (uint c=r.begin();c<r.end();c++)
        {
          // find nearest neighbor
          const uint clusterID = index_buffer[c];
          const BBox3f cluster_bounds = clusters[clusterID].bounds;          
          float min_area = pos_inf;
          int nn = -1;
            for (int i=std::max((int)c-SEARCH_RADIUS,0);i<std::min((int)c+SEARCH_RADIUS+1,(int)current_numClusters);i++)
              if (i != c && index_buffer[i] != -1)
              {
                const uint merge_clusterID = index_buffer[i];
                BBox3f bounds = clusters[merge_clusterID].bounds;
                if (!intersect(cluster_bounds,bounds).empty())
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
          nearest_neighborID[c] = nn;                  
        }
      });

      bool merged_pair = false;
      
      parallel_for(current_numClusters, [&] (uint i)
      //for (uint i=0;i<current_numClusters;i++)
      {
        if (nearest_neighborID[i] != -1)
        {
          if ( nearest_neighborID[ nearest_neighborID[i] ] == i)
          {
            if ( i < nearest_neighborID[i])
            {
              const uint leftClusterID = index_buffer[i];
              const uint rightClusterID = index_buffer[nearest_neighborID[i]];

              // if (intersect(clusters[leftClusterID].bounds,clusters[rightClusterID].bounds).empty())
              // {
              //   PRINT2(leftClusterID,rightClusterID);
              // }
              
              DBG_PRINT4("MERGE",leftClusterID,rightClusterID,getSimplificationRatio(clusters[leftClusterID], clusters[rightClusterID]));
              const uint newDepth =  std::max(clusters[leftClusterID].depth,clusters[rightClusterID].depth)+1;
            
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
                
                const uint newClusterID0 = numClusters.fetch_add(1);
                if (newClusterID0 >= MAX_NUM_MESH_CLUSTERS) FATAL("MAX_NUM_MESH_CLUSTERS");
                
                clusters[newClusterID0] = new_cluster0;

                tmp_buffer[i] = newClusterID0;


                if (new_clusters.size() == 2)
                {
                  PRINT("SPLIT CASE");
                  QuadMeshCluster &new_cluster1 = new_clusters[1];
                  clusters[leftClusterID].lod_root = false;
                  clusters[rightClusterID].lod_root = false;
                  new_cluster1.depth = newDepth;
                  new_cluster1.leftID  = leftClusterID;
                  new_cluster1.rightID = rightClusterID;
                  new_cluster1.neighborID = newClusterID0;
                  new_cluster1.initBounds();
                  new_cluster1.lod_root = true;              
                  const uint newClusterID1 = numClusters.fetch_add(1);
                  if (newClusterID1 >= MAX_NUM_MESH_CLUSTERS) FATAL("MAX_NUM_MESH_CLUSTERS");
                  
                  clusters[newClusterID1] = new_cluster1;
                  tmp_buffer[nearest_neighborID[i]] = newClusterID1;

                  /* update neighbor */
                  clusters[newClusterID0].neighborID = newClusterID1;
                  
                }
                else                                
                  tmp_buffer[nearest_neighborID[i]] = -1;
              }
              else
              {
                DBG_PRINT4("CANNOT MERGE", leftClusterID, rightClusterID,newDepth);
                tmp_buffer[i] = index_buffer[i];
                tmp_buffer[nearest_neighborID[i]] = index_buffer[nearest_neighborID[i]];                
                //nearest_neighborID[nearest_neighborID[i]] = -1;
                //exit(0);
              }
            }
            //else
            //  tmp_buffer[i] = -1;
          }
          else
            tmp_buffer[i] = index_buffer[i];              
        }
        else
          tmp_buffer[i] = index_buffer[i];                      
      });

      
      uint new_numClusters = 0;
      for (uint i=0;i<current_numClusters;i++)
        if (tmp_buffer[i] != -1)
          index_buffer[new_numClusters++] = tmp_buffer[i];


      // uint new_numClusterQuads = 0;
      // for (uint i=0;i<new_numClusters;i++)
      //   new_numClusterQuads += clusters[ index_buffer[i] ].quads.size();

      uint numTmpRoots = 0;
      for (uint i=0;i<numClusters;i++)      
        if (clusters[i].lod_root) numTmpRoots++;        
          
      iteration++;
      //numClusterQuads = new_numClusterQuads;
      
      DBG_PRINT3(iteration,new_numClusters,current_numClusters);


      if (!merged_pair) break; // couldn't merge any clusters anymore
      //if (current_numClusters == new_numClusters) break; // couldn't merge any clusters anymore
      
      current_numClusters = new_numClusters;

      if (numTmpRoots != current_numClusters)
      {
        PRINT2(numTmpRoots,current_numClusters);
        FATAL("numTmpRoots != current_numClusters");
      }      
    }

    DBG_PRINT("MERGING DONE");    
    
    delete [] nearest_neighborID;        
    delete [] tmp_buffer;    
    delete [] index_buffer;
    
    uint numTotalQuadsAllocate = 0;
    uint numTotalVerticesAllocate = 0;
    for (uint i=0;i<numClusters;i++)
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
    EmbreeUSMMode mode = EmbreeUSMMode::EMBREE_DEVICE_READ_WRITE;
    //EmbreeUSMMode mode = EmbreeUSMMode::EMBREE_USM_SHARED;

    lcm->compressedVertices = (CompressedVertex*)alignedUSMMalloc(sizeof(CompressedVertex)*numTotalVerticesAllocate,64,mode); // FIXME
    lcm->compressedIndices  = (CompressedQuadIndices*)alignedUSMMalloc(sizeof(CompressedQuadIndices)*numTotalQuadsAllocate,64,mode); //FIXME    


    CompressedVertex* compressedVertices = new CompressedVertex[numTotalVerticesAllocate];
    CompressedQuadIndices *compressedIndices = new CompressedQuadIndices[numTotalQuadsAllocate];
        
    
    uint globalCompressedVertexOffset = 0;
    uint globalCompressedIndexOffset = 0;

    // === quantize vertices with respect to geometry bounding box ===
    
    const Vec3f geometry_lower    = geometryBounds.lower;
    const Vec3f geometry_diag     = geometryBounds.size();
    const Vec3f geometry_inv_diag = geometry_diag != Vec3fa(0.0f) ? Vec3fa(1.0f) / geometry_diag : Vec3fa(0.0f);

    uint maxDepth = 0;
    for (uint c=0;c<numClusters;c++)
    {
      maxDepth = std::max(clusters[c].depth,maxDepth);
      LossyCompressedMeshCluster compressed_cluster;      
      compressed_cluster.numQuads  = clusters[c].quads.size();
      compressed_cluster.numBlocks = LossyCompressedMeshCluster::getDecompressedSizeInBytes(compressed_cluster.numQuads)/64;
      BBox3f cluster_bounds = clusters[c].bounds;

#if ENABLE_DAG == 1      
      if (clusters[c].neighborID != -1)
      {
        cluster_bounds.extend( clusters[ clusters[c].neighborID ].bounds );
      }
#endif
      
      compressed_cluster.lod_level = clusters[c].depth;
      compressed_cluster.tmp = 0;
      
      compressed_cluster.bounds = CompressedAABB3f( CompressedVertex(cluster_bounds.lower,geometry_lower,geometry_inv_diag),
                                                    CompressedVertex(cluster_bounds.upper,geometry_lower,geometry_inv_diag) );
      
      compressed_cluster.lodLeftID = (clusters[c].leftID != -1) ? (clusters[c].leftID-c) : -1;
      compressed_cluster.lodRightID = (clusters[c].rightID != -1) ? (clusters[c].rightID-c) : -1;
      compressed_cluster.neighborID = (clusters[c].neighborID != -1) ? (clusters[c].neighborID-c) : -1;
      if (clusters[c].neighborID != -1 && clusters[ clusters[c].neighborID ].neighborID != c)
        FATAL("clusters[c].neighborID");
       
      compressed_cluster.offsetIndices  = globalCompressedIndexOffset;      
      compressed_cluster.offsetVertices = globalCompressedVertexOffset;
      compressed_cluster.mesh = lcm;

      for (uint i=0;i<clusters[c].quads.size();i++)
      {
        const uint v0 = clusters[c].quads[i].v0;
        const uint v1 = clusters[c].quads[i].v1;
        const uint v2 = clusters[c].quads[i].v2;
        const uint v3 = clusters[c].quads[i].v3;        
        compressedIndices[ globalCompressedIndexOffset++ ] = CompressedQuadIndices(v0,v1,v2,v3);
        if ( globalCompressedIndexOffset > numTotalQuadsAllocate ) FATAL("numTotalQuadsAllocate");        
      }
        
      for (uint i=0;i<clusters[c].vertices.size();i++)
      {
        compressedVertices[ globalCompressedVertexOffset++ ] = CompressedVertex(clusters[c].vertices[i],geometry_lower,geometry_inv_diag);
        if ( globalCompressedVertexOffset > numTotalVerticesAllocate ) FATAL("numTotalVerticesAllocate");                
      }

      
      lcm_ptrs.push_back(lcm);      
      const uint lcm_clusterID = lcm_clusters.size();      
      lcm_clusters.push_back(compressed_cluster);
      if (clusters[c].lod_root)
        lcm_clusterRootIDs.push_back(lcm_clusterID);
      
      numDecompressedBlocks += compressed_cluster.numBlocks;      
    }
    if (numTotalVerticesAllocate != globalCompressedVertexOffset) FATAL("numTotalVerticesAllocate != globalCompressedVertexOffset");
    if (numTotalQuadsAllocate    != globalCompressedIndexOffset) FATAL("numTotalQuadsAllocate != globalCompressedIndexOffset");

    DBG_PRINT2(compressedVertices,compressedVertices);
    queue.memcpy(lcm->compressedVertices,compressedVertices,sizeof(CompressedVertex)*globalCompressedVertexOffset);
    queue.memcpy(lcm->compressedIndices,compressedIndices,sizeof(CompressedQuadIndices)*globalCompressedIndexOffset);
    
    gpu::waitOnQueueAndCatchException(*global_gpu_queue);        

    const size_t uncompressedSizeMeshBytes = mesh->numVertices * sizeof(Vec3f) + mesh->numQuads * sizeof(uint) * 4;
    const size_t compressedSizeMeshBytes = sizeof(CompressedVertex)*numTotalVertices + sizeof(CompressedQuadIndices)*numQuads;
    const size_t clusterSizeBytes = numClusters*sizeof(LossyCompressedMeshCluster);
    DBG_PRINT4(uncompressedSizeMeshBytes,compressedSizeMeshBytes,(float)compressedSizeMeshBytes/uncompressedSizeMeshBytes,clusterSizeBytes);

    totalCompressedSize += compressedSizeMeshBytes + clusterSizeBytes;
    DBG_PRINT(maxDepth);

    delete [] compressedIndices;
    delete [] compressedVertices;    
    delete [] clusters;

    PRINT("CLUSTER PROCESSING DONE");    
    //exit(0);
    
    return numNumClustersMaxRes;
  }  


  uint findVertex(std::vector<CompressedVertex> &vertices, const CompressedVertex &cv)
  {
    for (uint i=0;i<vertices.size();i++)
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
