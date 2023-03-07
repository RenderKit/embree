#include "nanite_geometry_device.h"
#include "../../kernels/rthwif/builder/gpu/lcgbp.h"
#include "../../kernels/rthwif/builder/gpu/morton.h"


namespace embree {

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
  
  struct Mesh {
    std::vector<Triangle> triangles;
    std::vector<CompressedVertex> vertices;
  };


  struct TriangleMesh {
    std::vector<Triangle> triangles;
    std::vector<Vec3f> vertices;
  };
  
  struct QuadMeshCluster {
    bool lod_root;
    uint leftID, rightID;
    std::vector<Quad> quads;
    std::vector<Vec3f> vertices;

    __forceinline QuadMeshCluster() : leftID(-1), rightID(-1), lod_root(false) {}

    __forceinline bool isLeaf() { return leftID == -1 || rightID == -1; }    
  };

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


  bool mergeSimplifyQuadMeshCluster(QuadMeshCluster &cluster0,QuadMeshCluster &cluster1, QuadMeshCluster &quadMesh)
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
    
    PRINT(mesh.vertices.size());
    PRINT(mesh.triangles.size());

    const uint numTriangles = mesh.triangles.size();
    const uint numVertices  = mesh.vertices.size();
    const uint numIndices   = numTriangles*3;

    Triangle *new_triangles = new Triangle[numTriangles];    
    Triangle *triangles     = &*mesh.triangles.begin();
    Vec3f *vertices         = &*mesh.vertices.begin();

    const float REDUCTION_FACTOR = 0.5f;
    uint expectedTriangles = floorf((LossyCompressedMeshCluster::MAX_QUADS_PER_CLUSTER * 3) * REDUCTION_FACTOR);
    
    uint iterations = 0;
    while(1)
    {
      iterations++;
      if (iterations > 10) return false;
      bool retry = false;      
      float result_error = 0.0f;
      const size_t new_numIndices = meshopt_simplify((uint*)new_triangles,(uint*)triangles,numIndices,(float*)vertices,numVertices,sizeof(Vec3f),expectedTriangles*3,0.05f,meshopt_SimplifyLockBorder,&result_error);

      const size_t new_numTriangles = new_numIndices/3;
      PRINT3(expectedTriangles,new_numTriangles,result_error);

      std::vector<uint> new_vertices;
      for (uint i=0;i<new_numTriangles;i++)
      {
        countVertexIDs(new_vertices, new_triangles[i].v0);
        countVertexIDs(new_vertices, new_triangles[i].v1);
        countVertexIDs(new_vertices, new_triangles[i].v2);      
      }      
      if (new_vertices.size() > 256)
      {
        PRINT2("new_vertices.size()",new_vertices.size());
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


      if (quadMesh.quads.size() > LossyCompressedMeshCluster::MAX_QUADS_PER_CLUSTER) { PRINT2("RETRY quadMesh.quads.size()",quadMesh.quads.size()); retry = true; }
      if (quadMesh.vertices.size() > 256) { PRINT("RETRY quadMesh.vertices.size()"); retry = true; }
      if (retry)
      {
        quadMesh.vertices.clear();
        quadMesh.quads.clear();
        PRINT2(quadMesh.vertices.size(),quadMesh.quads.size());
        expectedTriangles -= std::max((uint)2,expectedTriangles/10);
      }
      else
        break;
    }
    //exit(0);
    PRINT2(quadMesh.quads.size(),quadMesh.vertices.size());

    delete [] new_triangles;
    
    return true;
  }
  
  
  
  
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
        //PRINT4(ranges[currentID].range.start,ranges[currentID].range.end,ranges[currentID].range.size(),ranges[currentID].parent);
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

    
  
  void convertISPCQuadMesh(ISPCQuadMesh* mesh, RTCScene scene, ISPCOBJMaterial *material,const uint geomID,std::vector<LossyCompressedMesh*> &lcm_ptrs,std::vector<LossyCompressedMeshCluster> &lcm_clusters, std::vector<uint> &lcm_clusterRootIDs, size_t &totalCompressedSize, size_t &numDecompressedBlocks)
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
    std::vector<HierarchyRange> ranges;
    std::vector<uint> leafIDs;
    std::vector<QuadMeshCluster> clusters;
    std::vector<uint> clusterRootIDs;
    
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

    // === extract ranges, test range if it fullfills requirements, split if necessary ===
    uint numTotalVertices = 0;
    
    ranges.push_back(HierarchyRange(gpu::Range(0,mcodes.size())));
    extractRanges(0,&*mcodes.begin(),ranges,leafIDs,mesh,numTotalVertices,INITIAL_CREATE_RANGE_THRESHOLD);
    PRINT(ranges.size());
    PRINT(leafIDs.size());

    const uint numRanges = leafIDs.size();

    // === create leaf clusters ===
    
    for (uint i=0;i<leafIDs.size();i++)
    {
      const uint ID = leafIDs[i];
      QuadMeshCluster cluster;

      std::map<uint,uint> index_map;
      uint numLocalIndices = 0;

      // === remap vertices relative to cluster ===
      //PRINT2(ranges[ID].range.start,ranges[ID].range.end);
      
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
      ranges[ID].clusterID = clusters.size();
      clusters.push_back(cluster);
      //PRINT2(cluster.quads.size(),cluster.vertices.size());
    }

    // === bottom-up merging and creation of new clusters ===
    
    for (uint i=0;i<leafIDs.size();i++)
    {
      const uint ID = leafIDs[i];
      const uint parentID = ranges[ID].parent;
      //PRINT2(ID,parentID);
      if (parentID != -1)
      {        
        ranges[parentID].counter++;
        if (ranges[parentID].counter == 2)
        {
          const uint leftID = ranges[parentID].left;
          const uint rightID = ranges[parentID].right;
          //PRINT2(leftID,rightID);
          if (leftID == -1 || rightID == -1) FATAL("leftID, rightID");
          //PRINT5(parentID,ranges[leftID].range.start,ranges[leftID].range.end,ranges[rightID].range.start,ranges[rightID].range.end);
          // === merge ranges ===
          const uint  leftClusterID = ranges[ leftID].clusterID;
          const uint rightClusterID = ranges[rightID].clusterID;
          PRINT(parentID);
          QuadMeshCluster new_cluster;
          bool success = mergeSimplifyQuadMeshCluster( clusters[leftClusterID], clusters[rightClusterID], new_cluster);
          if (success)
          {
            PRINT(new_cluster.quads.size());
            PRINT(new_cluster.vertices.size());          
            const uint mergedClusterID = clusters.size();
            new_cluster.leftID  = leftClusterID;
            new_cluster.rightID = rightClusterID;          
            clusters.push_back(new_cluster);
            ranges[parentID].clusterID = mergedClusterID; 
          }
          else
          {
            ranges[parentID].counter = 0;
          }
        }
      }
    }

    extractClusterRootIDs(0,ranges,clusterRootIDs);
    PRINT(clusterRootIDs.size());
    for (uint i=0;i<clusterRootIDs.size();i++)
    {
      uint ID = clusterRootIDs[i];
      clusters[ID].lod_root = true;
    }
    
    uint numTotalQuadsAllocate = 0;
    uint numTotalVerticesAllocate = 0;

    for (uint i=0;i<clusters.size();i++)
    {
      numTotalQuadsAllocate += clusters[i].quads.size();
      numTotalVerticesAllocate += clusters[i].vertices.size();      
    }
    PRINT2(numTotalQuadsAllocate,numTotalVerticesAllocate);
    
    // === allocate LossyCompressedMesh in USM ===
    
    LossyCompressedMesh *lcm = (LossyCompressedMesh *)alignedUSMMalloc(sizeof(LossyCompressedMesh),64);
    lcm_ptrs.push_back(lcm);
  
    lcm->bounds             = geometryBounds;
    lcm->numQuads           = numQuads;
    lcm->numVertices        = mesh->numVertices;
    lcm->geomID             = geomID; 
    lcm->compressedVertices = (CompressedVertex*)alignedUSMMalloc(sizeof(CompressedVertex)*numTotalVerticesAllocate,64); // FIXME
    lcm->compressedIndices  = (CompressedQuadIndices*)alignedUSMMalloc(sizeof(CompressedQuadIndices)*numTotalQuadsAllocate,64); //FIXME    
           
    uint globalCompressedVertexOffset = 0;
    uint globalCompressedIndexOffset = 0;

    // === quantize vertices with respect to geometry bounding box ===
    
    const Vec3f geometry_lower    = geometryBounds.lower;
    const Vec3f geometry_diag     = geometryBounds.size();
    const Vec3f geometry_inv_diag = geometry_diag != Vec3fa(0.0f) ? Vec3fa(1.0f) / geometry_diag : Vec3fa(0.0f);

    const uint global_lcm_cluster_startID = lcm_clusters.size();
      
    for (uint c=0;c<clusters.size();c++)
    {
      LossyCompressedMeshCluster compressed_cluster;
      compressed_cluster.numQuads  = clusters[c].quads.size();
      compressed_cluster.numBlocks = LossyCompressedMeshCluster::getDecompressedSizeInBytes(compressed_cluster.numQuads)/64;
      compressed_cluster.ID = c;
      compressed_cluster.lodLeftID = (clusters[c].leftID != -1) ? global_lcm_cluster_startID + clusters[c].leftID : -1;
      compressed_cluster.lodRightID = (clusters[c].rightID != -1) ? global_lcm_cluster_startID + clusters[c].rightID : -1;
      compressed_cluster.offsetIndices  = globalCompressedIndexOffset;      
      compressed_cluster.offsetVertices = globalCompressedVertexOffset;
      compressed_cluster.mesh = lcm;

      for (uint i=0;i<clusters[c].quads.size();i++)
      {
        const uint v0 = clusters[c].quads[i].v0;
        const uint v1 = clusters[c].quads[i].v1;
        const uint v2 = clusters[c].quads[i].v2;
        const uint v3 = clusters[c].quads[i].v3;        
        lcm->compressedIndices[ globalCompressedIndexOffset++ ] = CompressedQuadIndices(v0,v1,v2,v3);
        if ( globalCompressedIndexOffset > numTotalQuadsAllocate ) FATAL("numTotalQuadsAllocate");        
      }
        
      for (uint i=0;i<clusters[c].vertices.size();i++)
      {
        lcm->compressedVertices[ globalCompressedVertexOffset++ ] = CompressedVertex(clusters[c].vertices[i],geometry_lower,geometry_inv_diag);
        if ( globalCompressedVertexOffset > numTotalVerticesAllocate ) FATAL("numTotalVerticesAllocate");                
      }
      
      compressed_cluster.numVertices           = clusters[c].vertices.size();

      const uint lcm_clusterID = lcm_clusters.size();
      lcm_clusters.push_back(compressed_cluster);

      if (clusters[c].lod_root)
      {
        lcm_clusterRootIDs.push_back(lcm_clusterID);
      }
      
      numDecompressedBlocks += compressed_cluster.numBlocks;      
    }

    const size_t uncompressedSizeMeshBytes = mesh->numVertices * sizeof(Vec3f) + mesh->numQuads * sizeof(uint) * 4;
    const size_t compressedSizeMeshBytes = sizeof(CompressedVertex)*numTotalVertices + sizeof(CompressedQuadIndices)*numQuads;
    const size_t clusterSizeBytes = numRanges*sizeof(LossyCompressedMeshCluster);
    PRINT5(lcm_ID,uncompressedSizeMeshBytes,compressedSizeMeshBytes,(float)compressedSizeMeshBytes/uncompressedSizeMeshBytes,clusterSizeBytes);

    totalCompressedSize += compressedSizeMeshBytes + clusterSizeBytes;
  }  


  uint findVertex(std::vector<CompressedVertex> &vertices, const CompressedVertex &cv)
  {
    for (uint i=0;i<vertices.size();i++)
      if (cv == vertices[i])
        return i;
    vertices.push_back(cv);
    return vertices.size()-1;
  }

  
  std::vector<Quad> extractQuads(Mesh &mesh)
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

  Mesh convertToTriangleMesh(LossyCompressedMeshCluster &cluster)
  {
    Mesh mesh;
    uint numQuads = cluster.numQuads;

    CompressedQuadIndices *compressedIndices = cluster.mesh->compressedIndices + cluster.offsetIndices;
    CompressedVertex *compressedVertices = cluster.mesh->compressedVertices + cluster.offsetVertices;

    for (uint i=0;i<numQuads;i++)
    {
      uint v0 = findVertex(mesh.vertices, compressedVertices[ compressedIndices[i].v0 ]);
      uint v1 = findVertex(mesh.vertices ,compressedVertices[ compressedIndices[i].v1 ]);
      uint v2 = findVertex(mesh.vertices, compressedVertices[ compressedIndices[i].v2 ]);
      uint v3 = findVertex(mesh.vertices, compressedVertices[ compressedIndices[i].v3 ]);

      Triangle tri0(v0,v1,v3);
      Triangle tri1(v1,v2,v3);
      if (tri0.valid()) mesh.triangles.push_back(tri0);
      if (tri1.valid()) mesh.triangles.push_back(tri1);            
    }

    PRINT(mesh.vertices.size());
    PRINT(mesh.triangles.size());
    return mesh;
  }

  __forceinline uint64_t makeUint64Edge(uint a, uint b)
  {
    if (a > b) std::swap(a,b);
    return ((uint64_t)b << 32) | a;
    
  }
  
  uint getEdgeCount(Mesh &mesh, const uint a, const uint b)
  {
    uint64_t edge = makeUint64Edge(a,b);
    uint count = 0;
    for (uint i=0;i<mesh.triangles.size();i++)
    {
      uint v0 = mesh.triangles[i].v0;
      uint v1 = mesh.triangles[i].v1;
      uint v2 = mesh.triangles[i].v2;

      count += makeUint64Edge(v0,v1) == edge ? 1 : 0;
      count += makeUint64Edge(v1,v2) == edge ? 1 : 0;
      count += makeUint64Edge(v2,v0) == edge ? 1 : 0;
    }
    return count;
  }
  
  Mesh simplifyTriangleMesh(Mesh &mesh)
  {
    const uint numVertices = mesh.vertices.size();
    const uint numTriangles = mesh.triangles.size();

    bool borderVertices[numVertices];
    for (uint i=0;i<numVertices;i++)
      borderVertices[i] = false;
    
    bool borderTriangle[numTriangles];

    uint numBorderTriangles = 0;
    for (uint i=0;i<numTriangles;i++)
    {
      //PRINT(i);
      uint v0 = mesh.triangles[i].v0;
      uint v1 = mesh.triangles[i].v1;
      uint v2 = mesh.triangles[i].v2;
      const uint count_v0v1= getEdgeCount(mesh,v0,v1);
      const uint count_v1v2= getEdgeCount(mesh,v1,v2);
      const uint count_v2v0= getEdgeCount(mesh,v2,v0);
      //PRINT4(v0,mesh.vertices[v0].x,mesh.vertices[v0].y,mesh.vertices[v0].z);
      //PRINT4(v1,mesh.vertices[v1].x,mesh.vertices[v1].y,mesh.vertices[v1].z);
      //PRINT4(v2,mesh.vertices[v2].x,mesh.vertices[v2].y,mesh.vertices[v2].z);
      
      //PRINT3(count_v0v1,count_v1v2,count_v2v0);
      
      if ( count_v0v1 == 1 ) { borderVertices[v0] = true; borderVertices[v1] = true; }
      if ( count_v1v2 == 1 ) { borderVertices[v1] = true; borderVertices[v2] = true; }
      if ( count_v2v0 == 1 ) { borderVertices[v2] = true; borderVertices[v0] = true; }
    }

    for (uint i=0;i<numTriangles;i++)
    {
      //PRINT(i);
      uint v0 = mesh.triangles[i].v0;
      uint v1 = mesh.triangles[i].v1;
      uint v2 = mesh.triangles[i].v2;
      borderTriangle[i] = false;
      
      if ( borderVertices[v0] || borderVertices[v1] || borderVertices[v2])
      {
        numBorderTriangles++;
        borderTriangle[i] = true;
      }
    }
    
    PRINT2(numTriangles,numBorderTriangles);
    for (uint i=0;i<numTriangles;i++)
      if (!borderTriangle[i])
      {
        //PRINT2(i,borderTriangle[i]);
        const CompressedVertex c = mesh.vertices[ mesh.triangles[i].v0 ];
        mesh.vertices[ mesh.triangles[i].v1 ] = c;
        mesh.vertices[ mesh.triangles[i].v2 ] = c;        
      }


    Mesh new_mesh;
    for (uint i=0;i<numTriangles;i++)
    {     
      uint v0 = findVertex(new_mesh.vertices, mesh.vertices[ mesh.triangles[i].v0 ]);
      uint v1 = findVertex(new_mesh.vertices, mesh.vertices[ mesh.triangles[i].v1 ]);
      uint v2 = findVertex(new_mesh.vertices, mesh.vertices[ mesh.triangles[i].v2 ]);

      Triangle tri0(v0,v1,v2);
      if (tri0.valid()) new_mesh.triangles.push_back(tri0);
    }

    PRINT(new_mesh.vertices.size());
    PRINT(new_mesh.triangles.size());
    return new_mesh;
  }
  
} // namespace embree
