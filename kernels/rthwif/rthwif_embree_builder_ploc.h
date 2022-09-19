#pragma once

#include "builder/qbvh6.h"
#include "builder/gpu/AABB3f.h"
#include "builder/gpu/sort.h"
#include "builder/gpu/morton.h"
#include <memory>

#if defined(EMBREE_DPCPP_SUPPORT)

#define SEARCH_RADIUS_SHIFT       4
#define SEARCH_RADIUS             (1<<SEARCH_RADIUS_SHIFT)
#define SINGLE_R_PATH             1
#define BVH_BRANCHING_FACTOR      6

#define FATLEAF_THRESHOLD         6
#define USE_NEW_OPENING           0

namespace embree
{  
  // ===================================================================================================================================================================================
  // =============================================================================== General ===========================================================================================
  // ===================================================================================================================================================================================

  const uint LEAF_TYPE_SHIFT = 30;
  const uint LEAF_TYPE_MASK_LOW   = ~(3 << LEAF_TYPE_SHIFT);
  const uint LEAF_TYPE_MASK_HIGH  =   3 << LEAF_TYPE_SHIFT;
  
  enum {
    LEAF_TYPE_QUAD       = 0 << LEAF_TYPE_SHIFT,
    LEAF_TYPE_PROCEDURAL = 1 << LEAF_TYPE_SHIFT,
    LEAF_TYPE_INSTANCE   = 2 << LEAF_TYPE_SHIFT,
    LEAF_TYPE_UNUSED     = 3 << LEAF_TYPE_SHIFT        
  };
  
  struct __aligned(64) PLOCGlobals
  {
    gpu::AABB3f geometryBounds;
    gpu::AABB3f centroidBounds; 
    char *qbvh_base_pointer;
    uint node_mem_allocator_start;
    uint node_mem_allocator_cur;
    uint64_t wgState;        
    // === 64 bytes ===
    uint bvh2_index_allocator;    
    uint leaf_mem_allocator_start;
    uint leaf_mem_allocator_cur;
    uint range_start;
    uint range_end;
    uint numPrimitives;
    uint numOrgPrimitives;
    uint numBuildRecords;   
    uint sync;
    uint totalAllocatedMem;
    uint rootIndex;
    uint wgID;
    uint numLeaves;
    uint padd;
    // === 128 bytes ===

    __forceinline void reset()
    {
      geometryBounds.init();
      centroidBounds.init();
      qbvh_base_pointer = nullptr;
      wgState                    = 0;      
      node_mem_allocator_cur     = 0;
      node_mem_allocator_start   = 0;
      bvh2_index_allocator       = 0;
      leaf_mem_allocator_cur     = 0;
      leaf_mem_allocator_start   = 0;
      range_start                = 0;
      range_end                  = 0;
      numPrimitives              = 0;
      numOrgPrimitives           = 0;      
      numBuildRecords            = 0;
      sync                       = 0;      
      totalAllocatedMem          = 0;
      rootIndex                  = 0;
      wgID                       = 0;
      numLeaves                  = 0;
    }
      
    /* allocate data in the node memory section */
    __forceinline char* atomic_allocNode(const uint bytes)
    {
      uint32_t blocks = (uint32_t)bytes / 64;
      const uint current = gpu::atomic_add_global(&node_mem_allocator_cur,blocks);      
      char* ptr = qbvh_base_pointer + 64 * (size_t)current;
      return ptr;
    }

    /* allocate memory in the leaf memory section */
    __forceinline char* atomic_allocLeaf(const uint bytes)
    {
      uint32_t blocks = (uint32_t)bytes / 64;      
      const uint current = gpu::atomic_add_global(&leaf_mem_allocator_cur,blocks);            
      char* ptr = qbvh_base_pointer + 64 * (size_t)current;
      return ptr;
    }
      
    __forceinline char* basePtr() {
      return qbvh_base_pointer;
    }

    __forceinline char* nodeBlockPtr(const uint blockID) {
      return qbvh_base_pointer + 64 * size_t(blockID);
    }

    __forceinline char* leafLocalPtr(const uint localID=0) {
      return qbvh_base_pointer + 64 * size_t(leaf_mem_allocator_start + localID);
    }

    __forceinline uint getBlockIDFromPtr(const char *addr) {
      return (addr - qbvh_base_pointer)/64;
    }
    

    __forceinline uint sub_group_shared_varying_atomic_allocNodeID(const uint bytes)
    {
      uint32_t blocks = (uint32_t)bytes / 64;
      return gpu::sub_group_shared_varying_atomic_add_global(&node_mem_allocator_cur,blocks);      
    }

    __forceinline char* sub_group_shared_varying_atomic_allocNode(const uint bytes)
    {
      const uint current = sub_group_shared_varying_atomic_allocNodeID(bytes);
      char* ptr = qbvh_base_pointer + 64 * (size_t)current;
      return ptr;
    }    
    
    __forceinline char* sub_group_shared_varying_atomic_allocLeaf(const uint bytes)
    {
      uint32_t blocks = (uint32_t)bytes / 64;      
      const uint current = gpu::sub_group_shared_varying_atomic_add_global(&leaf_mem_allocator_cur,blocks);            
      char* ptr = qbvh_base_pointer + 64 * (size_t)current;
      return ptr;
    }      
            
    __forceinline void resetGlobalCounters()
    {
      node_mem_allocator_cur = node_mem_allocator_start;
      leaf_mem_allocator_cur = leaf_mem_allocator_start;
      numBuildRecords = 0;	
    }
      
    __forceinline ulong alloc_node_mem(const uint size)
    {
      const uint aligned_size = ((size+63)/64)*64; /* allocate in 64 bytes blocks */
      return gpu::atomic_add_global(&node_mem_allocator_cur,aligned_size);
    }

    __forceinline ulong alloc_leaf_mem(const uint size)
    {
      const uint aligned_size = ((size+15)/16)*16; /* allocate in 16 bytes blocks */
      return gpu::atomic_add_global(&leaf_mem_allocator_cur,aligned_size);
    }
      
  };
      
  class __aligned(8) TriQuadMesh {
    
  private:
    static const uint QUAD_TYPE      = (uint)1 << 31;
    static const uint QUAD_TYPE_MASK = ~QUAD_TYPE;
    
    uint numPrimitives;
    uint numVertices;
    union {
      TriangleMesh::Triangle* triangles;
      QuadMesh::Quad* quads;
      void *ptr;
    };
    Vec3fa* vertices;

  public:
    __forceinline TriQuadMesh(const uint numPrimitives, const uint numVertices, void *ptr, Vec3fa *vertices, bool isQuad = false) : numPrimitives(numPrimitives|(isQuad?QUAD_TYPE:0)), numVertices(numVertices), ptr(ptr), vertices(vertices)
    {
    }

    __forceinline uint getNumPrimitives() const { return numPrimitives & QUAD_TYPE_MASK; }   
    __forceinline uint getNumVertices()   const { return numVertices; }
    __forceinline TriangleMesh::Triangle *getTrianglesPtr() const  { return triangles; }
    __forceinline QuadMesh::Quad         *getQuadsPtr()     const  { return quads; }
    __forceinline Vec3fa                 *getVerticesPtr()  const  { return vertices; }

    __forceinline bool isTriangleMesh() const { return (numPrimitives & QUAD_TYPE) == 0; } 
    __forceinline bool isQuadMesh()     const { return (numPrimitives & QUAD_TYPE) != 0 ; } 
        
  };
  
  struct __aligned(64) QBVHNodeN
  {
    float bounds_lower[3];
    int offset;

      struct {
        uint8_t type;
        uint8_t pad;
        char exp[3];
        uint8_t instMask;
        uint8_t childData[6];
      };

      struct {
        uint8_t lower_x[BVH_BRANCHING_FACTOR];
        uint8_t upper_x[BVH_BRANCHING_FACTOR];
        uint8_t lower_y[BVH_BRANCHING_FACTOR];
        uint8_t upper_y[BVH_BRANCHING_FACTOR];
        uint8_t lower_z[BVH_BRANCHING_FACTOR];
        uint8_t upper_z[BVH_BRANCHING_FACTOR];
      };


    __forceinline float3 start() const { return float3(bounds_lower[0],bounds_lower[1],bounds_lower[2]); }
    
    static __forceinline const gpu::AABB3f quantize_bounds(const float3 &start, const char exp_x, const char exp_y, const char exp_z, const gpu::AABB3f &fbounds) 
    {
      const float3 lower = fbounds.lower()-start;
      const float3 upper = fbounds.upper()-start;
      float qlower_x = ldexpf(lower.x(), -exp_x + 8); 
      float qlower_y = ldexpf(lower.y(), -exp_y + 8); 
      float qlower_z = ldexpf(lower.z(), -exp_z + 8); 
      float qupper_x = ldexpf(upper.x(), -exp_x + 8); 
      float qupper_y = ldexpf(upper.y(), -exp_y + 8); 
      float qupper_z = ldexpf(upper.z(), -exp_z + 8); 
      assert(qlower_x >= 0.0f && qlower_x <= 255.0f);
      assert(qlower_y >= 0.0f && qlower_y <= 255.0f);
      assert(qlower_z >= 0.0f && qlower_z <= 255.0f);
      assert(qupper_x >= 0.0f && qupper_x <= 255.0f);
      assert(qupper_y >= 0.0f && qupper_y <= 255.0f);
      assert(qupper_z >= 0.0f && qupper_z <= 255.0f); 
      qlower_x = min(max(sycl::floor(qlower_x),0.0f),255.0f);
      qlower_y = min(max(sycl::floor(qlower_y),0.0f),255.0f);
      qlower_z = min(max(sycl::floor(qlower_z),0.0f),255.0f);
      qupper_x = min(max(sycl::ceil(qupper_x),0.0f),255.0f);
      qupper_y = min(max(sycl::ceil(qupper_y),0.0f),255.0f);
      qupper_z = min(max(sycl::ceil(qupper_z),0.0f),255.0f);
      gpu::AABB3f qbounds(float3(qlower_x, qlower_y, qlower_z), float3(qupper_x, qupper_y, qupper_z));
      return qbounds;
    }    
  };

  struct LeafGenerationData {

    __forceinline LeafGenerationData() {}
    __forceinline LeafGenerationData(const uint blockID, const uint primID, const uint geomID) : blockID(blockID),primID(primID),geomID(geomID) {}
    
    uint blockID;
    union {
      uint primID;
      uint bvh2_index;
    };
    union {
      uint geomID;
      uint data;
    };
  };

  struct __aligned(16) TmpNodeState { //TODO: additional store left/right or primID/geomID
    uint header;
    uint bvh2_index;

    __forceinline TmpNodeState() {}
    __forceinline TmpNodeState(const uint _bvh2_index) : header(0x7fffffff), bvh2_index(_bvh2_index) {}
    __forceinline void init(const uint _bvh2_index) { header = 0x7fffffff; bvh2_index = _bvh2_index;  }
  };


  struct GeometryTypeRanges
  {
    uint triQuad_end;
    uint procedural_end;    
    uint instances_end;

    __forceinline GeometryTypeRanges(const uint triQuads, const uint numProcedurals, const uint numInstances) 
    {
      triQuad_end    = triQuads;
      procedural_end = triQuads + numProcedurals;      
      instances_end  = triQuads + numProcedurals + numInstances;
    }
    
    __forceinline bool isTriQuad(const uint index)    const { return index >= 0              && index < triQuad_end;    }
    __forceinline bool isProcedural(const uint index) const { return index >= triQuad_end    && index < procedural_end; }    
    __forceinline bool isInstance(const uint index)   const { return index >= procedural_end && index < instances_end;  }
    
  };
  
  // ===================================================================================================================================================================================
  // ================================================================================= BVH2 ============================================================================================
  // ===================================================================================================================================================================================
  
  class __aligned(32) BVH2Ploc
  {
  public:
    
    static const uint FATLEAF_SHIFT      =  29;    
    static const uint FATLEAF_BITS       =  (uint)7<<FATLEAF_SHIFT;
    static const uint BIT_MASK           = ~(FATLEAF_BITS);
    
    
    uint left;   // 4 bytes
    uint right;  // 4 bytes

    gpu::AABB3f bounds;       // 24 bytes
    
    __forceinline BVH2Ploc() {}

    __forceinline uint leftIndex()  const { return left & BIT_MASK;  }
    __forceinline uint getLeafIndex()  const { return left & BIT_MASK;  }
    __forceinline uint rightIndex() const { return right & BIT_MASK; }

    __forceinline uint geomID() const { return left & BIT_MASK & 0x00ffffff; }    
    __forceinline uint primID() const { return right & BIT_MASK; }
    __forceinline uint primID1() const { return ((left & BIT_MASK) >> 24) + primID(); }
    __forceinline uint primID1_offset() const { return ((left & BIT_MASK) >> 24); }
    
    __forceinline void init(const uint _left, const uint _right, const gpu::AABB3f &_bounds, const uint subtree_size_left, const uint subtree_size_right)
    {
      left    = _left  | ((subtree_size_left  <= FATLEAF_THRESHOLD) ? subtree_size_left  : 0)<<FATLEAF_SHIFT;
      right   = _right | ((subtree_size_right <= FATLEAF_THRESHOLD) ? subtree_size_right : 0)<<FATLEAF_SHIFT ;

      // better coalescing
#if 1
      bounds.lower_x = _bounds.lower_x;
      bounds.lower_y = _bounds.lower_y;
      bounds.lower_z = _bounds.lower_z;
      bounds.upper_x = _bounds.upper_x;
      bounds.upper_y = _bounds.upper_y;
      bounds.upper_z = _bounds.upper_z;                  
#else
      bounds  = _bounds;      
#endif      
    }
    
    __forceinline void initLeaf(const uint _geomID, const uint _primID, const gpu::AABB3f &_bounds)
    {
      left    = _geomID;      
      right   = _primID;

      // better coalescing      
#if 1
      bounds.lower_x = _bounds.lower_x;
      bounds.lower_y = _bounds.lower_y;
      bounds.lower_z = _bounds.lower_z;
      bounds.upper_x = _bounds.upper_x;
      bounds.upper_y = _bounds.upper_y;
      bounds.upper_z = _bounds.upper_z;                  
#else
      bounds  = _bounds;      
#endif      
      
    }


    __forceinline void store(BVH2Ploc *dest)
    {
      //dest = (BVH2Ploc *)__builtin_assume_aligned(dest,32);
      //uint *dest2 = (uint *)__builtin_assume_aligned((uint*)dest,32);
      dest->left  = left;
      dest->right = right;
      dest->bounds.lower_x = bounds.lower_x;
      dest->bounds.lower_y = bounds.lower_y;
      dest->bounds.lower_z = bounds.lower_z;
      dest->bounds.upper_x = bounds.upper_x;
      dest->bounds.upper_y = bounds.upper_y;
      dest->bounds.upper_z = bounds.upper_z;            
    }

    static  __forceinline bool isFatLeaf(const uint index) { return index & FATLEAF_BITS;  }
    static  __forceinline uint numFatChildren(const uint index) { return index >> FATLEAF_SHIFT;  }
    
    static  __forceinline uint getIndex(const uint index)  { return index & BIT_MASK;  }
    static  __forceinline bool isLeaf(const uint index, const uint numPrimitives) { return getIndex(index) < numPrimitives;  }

    static  __forceinline uint makeFatLeaf(const uint index, const uint numChildren) { return index | (numChildren<<FATLEAF_SHIFT);  }

    __forceinline operator const gpu::AABB3f &() const { return bounds; }

    friend __forceinline embree_ostream operator<<(embree_ostream cout, const BVH2Ploc &n)
    {
      cout << "left " << n.leftIndex() << " right " << n.rightIndex() << " geomID " << n.geomID() << " primID0 " << n.primID() << " primID1 " << n.primID1() << " AABB3f { ";
      cout << "  lower = (" << n.bounds.lower_x << ", " << n.bounds.lower_y << ", " << n.bounds.lower_z << ") ";
      cout << "  upper = (" << n.bounds.upper_x << ", " << n.bounds.upper_y << ", " << n.bounds.upper_z << ") ";
      return cout << "}";
    }    
      
  };


  struct __aligned(64) InstancePrimitive
  {
    uint32_t shaderIndex_geomMask; // : 24 shader index used to calculate instancing shader in case of software instancing
                                   // : 8; geometry mask used for ray masking
      
    uint32_t instanceContributionToHitGroupIndex; // : 24;

    // uint32_t type : 1; // enables/disables opaque culling
    // uint32_t geomFlags : 2; // unused for instances
      
    uint64_t startNodePtr_instFlags; // : 48;  start node where to continue traversal of the instanced object
      
    Vec3f world2obj_vx;   // 1st column of Worl2Obj transform
    Vec3f world2obj_vy;   // 2nd column of Worl2Obj transform
    Vec3f world2obj_vz;   // 3rd column of Worl2Obj transform
    Vec3f obj2world_p;    // translation of Obj2World transform (on purpose in first 64 bytes)

    uint64_t bvhPtr;   // pointer to BVH where start node belongs too
      
    uint32_t instanceID;    // user defined value per DXR spec
    uint32_t instanceIndex; // geometry index of the instance (n'th geometry in scene)
      
    Vec3f obj2world_vx;   // 1st column of Obj2World transform
    Vec3f obj2world_vy;   // 2nd column of Obj2World transform
    Vec3f obj2world_vz;   // 3rd column of Obj2World transform
    Vec3f world2obj_p;    // translation of World2Obj transform

    __forceinline InstancePrimitive() {}
    __forceinline InstancePrimitive(AffineSpace3f obj2world, uint64_t startNodePtr, uint32_t instID, uint8_t instMask)
    {
      shaderIndex_geomMask = instMask << 24; 
      instanceContributionToHitGroupIndex = ((uint)PrimLeafDesc::TYPE_OPACITY_CULLING_ENABLED << 29) | ((uint)GeometryFlags::NONE << 30);
    
      startNodePtr_instFlags = startNodePtr;
      
      instanceID = instID;
      instanceIndex = instID;
      bvhPtr = (uint64_t) 0;

      obj2world_vx = obj2world.l.vx;
      obj2world_vy = obj2world.l.vy;
      obj2world_vz = obj2world.l.vz;
      obj2world_p = obj2world.p;
      
      const AffineSpace3f world2obj = rcp(obj2world);
      world2obj_vx = world2obj.l.vx;
      world2obj_vy = world2obj.l.vy;
      world2obj_vz = world2obj.l.vz;
      world2obj_p = world2obj.p;      
    }
  };
  
  static_assert(sizeof(InstancePrimitive) == 128, "InstanceLeaf must be 128 bytes large");
 
  // ===================================================================================================================================================================================
  // ============================================================================== Quadifier ==========================================================================================
  // ===================================================================================================================================================================================

  __forceinline bool isValidTriangle(const TriQuadMesh& mesh, const uint i, uint3 &indices)
  {
    const TriangleMesh::Triangle &tri = mesh.getTrianglesPtr()[i];
    const uint numVertices = mesh.getNumVertices();
    indices.x() = tri.v[0];
    indices.y() = tri.v[1];
    indices.z() = tri.v[2];    
    if (max(tri.v[0],max(tri.v[1],tri.v[2])) >= numVertices) return false;
    const Vec3fa v0 = mesh.getVerticesPtr()[tri.v[0]];
    const Vec3fa v1 = mesh.getVerticesPtr()[tri.v[1]];
    const Vec3fa v2 = mesh.getVerticesPtr()[tri.v[2]];
    const float max_v0 = max(fabsf(v0.x),fabsf(v0.y),fabsf(v0.z));
    const float max_v1 = max(fabsf(v1.x),fabsf(v1.y),fabsf(v1.z));
    const float max_v2 = max(fabsf(v2.x),fabsf(v2.y),fabsf(v2.z));    
    const static float FLT_LARGE = 1.844E18f;
    const float max_value = max(max_v0,max(max_v1,max_v2));
    return max_value < FLT_LARGE && sycl::isfinite(max_value);
  }

  __forceinline bool isValidQuad(const TriQuadMesh& mesh, const uint i, uint4 &indices)
  {
    const QuadMesh::Quad &quad = mesh.getQuadsPtr()[i];
    const uint numVertices = mesh.getNumVertices();
    indices.x() = quad.v[0];
    indices.y() = quad.v[1];
    indices.z() = quad.v[2];
    indices.w() = quad.v[3];    
    if (max(max(quad.v[0],quad.v[1]),max(quad.v[2],quad.v[3])) >= numVertices) return false;
    const Vec3fa v0 = mesh.getVerticesPtr()[quad.v[0]];
    const Vec3fa v1 = mesh.getVerticesPtr()[quad.v[1]];
    const Vec3fa v2 = mesh.getVerticesPtr()[quad.v[2]];
    const Vec3fa v3 = mesh.getVerticesPtr()[quad.v[3]];    
    const float max_v0 = max(fabsf(v0.x),fabsf(v0.y),fabsf(v0.z));
    const float max_v1 = max(fabsf(v1.x),fabsf(v1.y),fabsf(v1.z));
    const float max_v2 = max(fabsf(v2.x),fabsf(v2.y),fabsf(v2.z));
    const float max_v3 = max(fabsf(v3.x),fabsf(v3.y),fabsf(v3.z));        
    const static float FLT_LARGE = 1.844E18f;
    const float max_value = max(max(max_v0,max_v1),max(max_v2,max_v3));
    return max_value < FLT_LARGE && sycl::isfinite(max_value);
  }
  

  __forceinline bool isValidTriangle(const TriQuadMesh& mesh, const uint i, uint3 &indices, gpu::AABB3f &bounds)
  {
    const TriangleMesh::Triangle &tri = mesh.getTrianglesPtr()[i];
    const uint numVertices = mesh.getNumVertices();
    indices.x() = tri.v[0];
    indices.y() = tri.v[1];
    indices.z() = tri.v[2];    
    if (max(tri.v[0],max(tri.v[1],tri.v[2])) >= numVertices) return false;
    const Vec3fa v0 = mesh.getVerticesPtr()[tri.v[0]];
    const Vec3fa v1 = mesh.getVerticesPtr()[tri.v[1]];
    const Vec3fa v2 = mesh.getVerticesPtr()[tri.v[2]];
    const float max_v0 = max(fabsf(v0.x),fabsf(v0.y),fabsf(v0.z));
    const float max_v1 = max(fabsf(v1.x),fabsf(v1.y),fabsf(v1.z));
    const float max_v2 = max(fabsf(v2.x),fabsf(v2.y),fabsf(v2.z));    
    const static float FLT_LARGE = 1.844E18f;
    const float max_value = max(max_v0,max(max_v1,max_v2));    
    if (max_value >= FLT_LARGE || !sycl::isfinite(max_value)) return false;
    float3 vtx0(v0.x,v0.y,v0.z);
    float3 vtx1(v1.x,v1.y,v1.z);
    float3 vtx2(v2.x,v2.y,v2.z);
    bounds.extend(vtx0);
    bounds.extend(vtx1);
    bounds.extend(vtx2);    
    return true;
  }

  __forceinline bool isValidQuad(const TriQuadMesh& mesh, const uint i, uint4 &indices, gpu::AABB3f &bounds)
  {
    const QuadMesh::Quad &quad = mesh.getQuadsPtr()[i];
    const uint numVertices = mesh.getNumVertices();
    indices.x() = quad.v[0];
    indices.y() = quad.v[1];
    indices.z() = quad.v[2];
    indices.w() = quad.v[3];    
    if (max(max(quad.v[0],quad.v[1]),max(quad.v[2],quad.v[3])) >= numVertices) return false;
    const Vec3fa v0 = mesh.getVerticesPtr()[quad.v[0]];
    const Vec3fa v1 = mesh.getVerticesPtr()[quad.v[1]];
    const Vec3fa v2 = mesh.getVerticesPtr()[quad.v[2]];
    const Vec3fa v3 = mesh.getVerticesPtr()[quad.v[3]];    
    const float max_v0 = max(fabsf(v0.x),fabsf(v0.y),fabsf(v0.z));
    const float max_v1 = max(fabsf(v1.x),fabsf(v1.y),fabsf(v1.z));
    const float max_v2 = max(fabsf(v2.x),fabsf(v2.y),fabsf(v2.z));
    const float max_v3 = max(fabsf(v3.x),fabsf(v3.y),fabsf(v3.z));        
    const static float FLT_LARGE = 1.844E18f;
    const float max_value = max(max(max_v0,max_v1),max(max_v2,max_v3));
    if (max_value >= FLT_LARGE && !sycl::isfinite(max_value)) return false;
    float3 vtx0(v0.x,v0.y,v0.z);
    float3 vtx1(v1.x,v1.y,v1.z);
    float3 vtx2(v2.x,v2.y,v2.z);
    float3 vtx3(v3.x,v3.y,v3.z);    
    bounds.extend(vtx0);
    bounds.extend(vtx1);
    bounds.extend(vtx2);    
    bounds.extend(vtx3);
    return true;
  }
  

  __forceinline uint try_pair_triangles(const uint3 &a, const uint3 &b, uint& lb0, uint& lb1, uint &lb2)    
  {
    lb0 = 3;
    lb1 = 3;
    lb2 = 3;
    
    lb0 = ( b.x() == a.x() ) ? 0 : lb0;
    lb1 = ( b.y() == a.x() ) ? 0 : lb1;
    lb2 = ( b.z() == a.x() ) ? 0 : lb2;
    
    lb0 = ( b.x() == a.y() ) ? 1 : lb0;
    lb1 = ( b.y() == a.y() ) ? 1 : lb1;
    lb2 = ( b.z() == a.y() ) ? 1 : lb2;
    
    lb0 = ( b.x() == a.z() ) ? 2 : lb0;
    lb1 = ( b.y() == a.z() ) ? 2 : lb1;
    lb2 = ( b.z() == a.z() ) ? 2 : lb2;    
    if ((lb0 == 3) + (lb1 == 3) + (lb2 == 3) <= 1)
    {
      uint p3_index = 0;
      p3_index = (lb1 == 3) ? 1 : p3_index;
      p3_index = (lb2 == 3) ? 2 : p3_index;
      return p3_index;
    }
    else
      return -1;
  }

  __forceinline bool try_pair_triangles(const uint3 &a, const uint3 &b)    
  {
    uint lb0,lb1,lb2;
    lb0 = 3;
    lb1 = 3;
    lb2 = 3;
    
    lb0 = ( b.x() == a.x() ) ? 0 : lb0;
    lb1 = ( b.y() == a.x() ) ? 0 : lb1;
    lb2 = ( b.z() == a.x() ) ? 0 : lb2;
    
    lb0 = ( b.x() == a.y() ) ? 1 : lb0;
    lb1 = ( b.y() == a.y() ) ? 1 : lb1;
    lb2 = ( b.z() == a.y() ) ? 1 : lb2;
    
    lb0 = ( b.x() == a.z() ) ? 2 : lb0;
    lb1 = ( b.y() == a.z() ) ? 2 : lb1;
    lb2 = ( b.z() == a.z() ) ? 2 : lb2;    
    if ((lb0 == 3) + (lb1 == 3) + (lb2 == 3) <= 1)
      return true;
    else
      return false;
  }
  
  __forceinline uint find_geomID_from_blockID(const uint *const prefix_sum, const uint numBlocks, const uint blockID)
  {
    uint l = 0;
    uint r = numBlocks;

    while (r - l > 1)
    {
      const uint m = (l + r) / 2;
      const uint k = prefix_sum[m];
      l = (blockID >= k) ? m : l;
      r = (blockID  < k) ? m : r;
    }
    return l;
  }  

  __forceinline void prefixsumOverGeometryCounts (sycl::queue &gpu_queue, const uint numGeoms, uint *const quads_per_geom_prefix_sum, uint *host_device_tasks, double &iteration_time, const bool verbose)    
  {
    static const uint GEOM_PREFIX_SUB_GROUP_WIDTH = 16;
    static const uint GEOM_PREFIX_WG_SIZE  = 1024;

    sycl::event queue_event =  gpu_queue.submit([&](sycl::handler &cgh) {
                                                  sycl::accessor< uint       , 1, sycl_read_write, sycl_local> counts(sycl::range<1>((GEOM_PREFIX_WG_SIZE/GEOM_PREFIX_SUB_GROUP_WIDTH)),cgh);
                                                  sycl::accessor< uint       , 1, sycl_read_write, sycl_local> counts_prefix_sum(sycl::range<1>((GEOM_PREFIX_WG_SIZE/GEOM_PREFIX_SUB_GROUP_WIDTH)),cgh);
                                                  const sycl::nd_range<1> nd_range(GEOM_PREFIX_WG_SIZE,sycl::range<1>(GEOM_PREFIX_WG_SIZE));		  
                                                  cgh.parallel_for(nd_range,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(GEOM_PREFIX_SUB_GROUP_WIDTH) {
                                                      const uint subgroupID      = get_sub_group_id();
                                                      const uint subgroupLocalID = get_sub_group_local_id();                                                        
                                                      const uint localID        = item.get_local_id(0);
                                                      const uint localSize       = item.get_local_range().size();            
                                                      const uint aligned_numGeoms = gpu::alignTo(numGeoms,GEOM_PREFIX_WG_SIZE);

                                                      uint total_offset = 0;
                                                      for (uint t=localID;t<aligned_numGeoms;t+=localSize)
                                                      {
                                                        item.barrier(sycl::access::fence_space::local_space);
                                                          
                                                        uint count = 0;
                                                        if (t < numGeoms)
                                                          count = quads_per_geom_prefix_sum[t];

                                                        const uint exclusive_scan = sub_group_exclusive_scan(count, std::plus<uint>());
                                                        const uint reduction = sub_group_reduce(count, std::plus<uint>());                                                     
                                                        counts[subgroupID] = reduction;                                                             

                                                        item.barrier(sycl::access::fence_space::local_space);

                                                        uint total_reduction = 0;
                                                        for (uint j=subgroupLocalID;j<GEOM_PREFIX_WG_SIZE/GEOM_PREFIX_SUB_GROUP_WIDTH;j+=GEOM_PREFIX_SUB_GROUP_WIDTH)
                                                        {
                                                          const uint subgroup_counts = counts[j];
                                                          const uint sums_exclusive_scan = sub_group_exclusive_scan(subgroup_counts, std::plus<uint>());
                                                          const uint reduction = sub_group_broadcast(subgroup_counts,GEOM_PREFIX_SUB_GROUP_WIDTH-1) + sub_group_broadcast(sums_exclusive_scan,GEOM_PREFIX_SUB_GROUP_WIDTH-1);
                                                          counts_prefix_sum[j] = sums_exclusive_scan + total_reduction;
                                                          total_reduction += reduction;
                                                        }
                                                        item.barrier(sycl::access::fence_space::local_space);

                                                        const uint sums_prefix_sum = counts_prefix_sum[subgroupID];                                                                 
                                                        const uint p_sum = total_offset + sums_prefix_sum + exclusive_scan;
                                                        total_offset += total_reduction;
                                                          
                                                        if (t < numGeoms)
                                                          quads_per_geom_prefix_sum[t] = p_sum;

                                                      }
                                                        
                                                      if (localID == 0)
                                                      {
                                                        quads_per_geom_prefix_sum[numGeoms] = total_offset;                                                          
                                                        *host_device_tasks = total_offset;
                                                      }
                                                        
                                                    });
                                                });
    gpu::waitOnQueueAndCatchException(gpu_queue);
    double dt = gpu::getDeviceExecutionTiming(queue_event);
    iteration_time += dt;
    if (unlikely(verbose))
      std::cout << "Prefix sum over quad counts over geometries " << dt << " ms" << std::endl;    
  }
  
  
  __forceinline void countQuadsPerGeometry (sycl::queue &gpu_queue, const TriQuadMesh *const triQuadMesh, const uint numGeoms, uint *quads_per_geom_prefix_sum, double &iteration_time, const bool verbose)    
  {
    static const uint COUNT_QUADS_PER_GEOMETRY_SEARCH_WG_SIZE  = 1024;
    
    const sycl::nd_range<1> nd_range1(numGeoms*COUNT_QUADS_PER_GEOMETRY_SEARCH_WG_SIZE,sycl::range<1>(COUNT_QUADS_PER_GEOMETRY_SEARCH_WG_SIZE));
    sycl::event queue_event = gpu_queue.submit([&](sycl::handler &cgh) {
        sycl::accessor< uint      ,  0, sycl_read_write, sycl_local> _active_counter(cgh);                                                   
        cgh.parallel_for(nd_range1,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(16)      
                         {
                           const uint localID         = item.get_local_id(0);                                                                      
                           const uint subgroupLocalID = get_sub_group_local_id();
                           const uint subgroupSize    = get_sub_group_size();        
                           const uint geomID          = item.get_group(0);
                           uint &active_counter       = *_active_counter.get_pointer();

                           active_counter = 0;
                           item.barrier(sycl::access::fence_space::local_space);

                           const TriQuadMesh &mesh = triQuadMesh[geomID];
                           
                           // ====================                           
                           // === TriangleMesh ===
                           // ====================
                           
                           if (mesh.isTriangleMesh())
                           {
                             const uint numTriangles = mesh.getNumPrimitives();
                             const uint numBlocks    = (numTriangles + COUNT_QUADS_PER_GEOMETRY_SEARCH_WG_SIZE - 1) / COUNT_QUADS_PER_GEOMETRY_SEARCH_WG_SIZE;
                                                                        
                             for (uint blockID = 0; blockID < numBlocks; blockID++)
                             {                                                                          
                               const uint startPrimID  = blockID*COUNT_QUADS_PER_GEOMETRY_SEARCH_WG_SIZE;
                               const uint endPrimID    = min(startPrimID+COUNT_QUADS_PER_GEOMETRY_SEARCH_WG_SIZE,numTriangles);
                               const uint ID           = (startPrimID + localID) < endPrimID ? startPrimID + localID : -1;
                               uint3 tri_indices;
                               bool valid = ID < endPrimID ? isValidTriangle(mesh,ID,tri_indices) : false;
                               bool paired = false;
                               uint numQuads = 0;
                               uint active_mask = sub_group_ballot(valid);

                               while(active_mask)
                               {
                                 active_mask = sub_group_broadcast(active_mask,0);
                                                                              
                                 const uint broadcast_lane = sycl::ctz(active_mask);
                                 if (subgroupLocalID == broadcast_lane) valid = false;

                                 active_mask &= active_mask-1;
                                                                              
                                 const bool broadcast_paired = sub_group_broadcast(paired, broadcast_lane);
                                 const uint broadcast_ID     = sub_group_broadcast(ID    , broadcast_lane);

                                 if (!broadcast_paired)
                                 {
                                   const uint3 tri_indices_broadcast(sub_group_broadcast(tri_indices.x(),broadcast_lane),
                                                                     sub_group_broadcast(tri_indices.y(),broadcast_lane),
                                                                     sub_group_broadcast(tri_indices.z(),broadcast_lane));
                                   bool pairable = false;
                                   if (ID != broadcast_ID && !paired && valid)
                                     pairable = try_pair_triangles(tri_indices_broadcast,tri_indices);
                                                                            
                                   const uint first_paired_lane = sycl::ctz(sub_group_ballot(pairable));
                                   if (first_paired_lane < subgroupSize)
                                   {
                                     active_mask &= ~((uint)1 << first_paired_lane);
                                     if (subgroupLocalID == first_paired_lane) { valid = false; }
                                   }
                                 }
                                 numQuads++;
                               }                                                                          
                               sycl::atomic_ref<uint, sycl::memory_order::relaxed, sycl::memory_scope::work_group,sycl::access::address_space::local_space> counter(active_counter);
                               if (subgroupLocalID == 0)
                                 counter.fetch_add(numQuads);
                             }
                           }
                           // ================                           
                           // === QuadMesh ===
                           // ================
                           else
                           {
                             const uint numQuads  = mesh.getNumPrimitives();
                             const uint numBlocks = (numQuads + COUNT_QUADS_PER_GEOMETRY_SEARCH_WG_SIZE - 1) / COUNT_QUADS_PER_GEOMETRY_SEARCH_WG_SIZE;
                                                                        
                             for (uint blockID = 0; blockID < numBlocks; blockID++)
                             {                                                                          
                               const uint startPrimID  = blockID*COUNT_QUADS_PER_GEOMETRY_SEARCH_WG_SIZE;
                               const uint endPrimID    = min(startPrimID+COUNT_QUADS_PER_GEOMETRY_SEARCH_WG_SIZE,numQuads);
                               const uint ID           = (startPrimID + localID) < endPrimID ? startPrimID + localID : -1;
                               uint4 quad_indices;
                               bool valid = ID < endPrimID ? isValidQuad(mesh,ID,quad_indices) : false;
                               uint active_mask = sub_group_ballot(valid);
                               uint numQuads = sycl::popcount(active_mask);

                               sycl::atomic_ref<uint, sycl::memory_order::relaxed, sycl::memory_scope::work_group,sycl::access::address_space::local_space> counter(active_counter);
                               if (subgroupLocalID == 0)
                                 counter.fetch_add(numQuads);                             
                             }
                           }
                             
                           item.barrier(sycl::access::fence_space::local_space);
                                                                        
                           if (localID == 0)                                                                          
                             quads_per_geom_prefix_sum[geomID] = active_counter;
                           
                         });
		  
      });
    gpu::waitOnQueueAndCatchException(gpu_queue);
    double dt = gpu::getDeviceExecutionTiming(queue_event);      
    iteration_time += dt;
    if (unlikely(verbose)) PRINT2("count quads per geometry", (float)dt);
  }

  __forceinline void createQuads_initPLOCPrimRefs(sycl::queue &gpu_queue, const TriQuadMesh *const triQuadMesh, const uint numGeoms, const uint *const quads_per_geom_prefix_sum, BVH2Ploc *const bvh2, const uint prim_type_offset, double &iteration_time, const bool verbose)    
  {
    static const uint MERGE_TRIANGLES_TO_QUADS_SEARCH_WG_SIZE  = 1024;
    static const uint MERGE_TRIANGLES_TO_QUADS_SUB_GROUP_WIDTH = 16;
    const sycl::nd_range<1> nd_range1(numGeoms*MERGE_TRIANGLES_TO_QUADS_SEARCH_WG_SIZE,sycl::range<1>(MERGE_TRIANGLES_TO_QUADS_SEARCH_WG_SIZE));
    sycl::event queue_event = gpu_queue.submit([&](sycl::handler &cgh) {
        sycl::accessor< uint      ,  0, sycl_read_write, sycl_local> _active_counter(cgh);
        sycl::accessor< uint       , 1, sycl_read_write, sycl_local> counts(sycl::range<1>((MERGE_TRIANGLES_TO_QUADS_SEARCH_WG_SIZE/MERGE_TRIANGLES_TO_QUADS_SUB_GROUP_WIDTH)),cgh);       
        cgh.parallel_for(nd_range1,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(MERGE_TRIANGLES_TO_QUADS_SUB_GROUP_WIDTH)      
                         {
                           const uint localID         = item.get_local_id(0);
                           const uint subgroupID      = get_sub_group_id();
                           const uint subgroupLocalID = get_sub_group_local_id();
                           const uint subgroupSize    = get_sub_group_size();        
                           const uint geomID          = item.get_group(0);
                           uint &active_counter       = *_active_counter.get_pointer();

                           active_counter = 0;

                           const uint startQuadOffset = quads_per_geom_prefix_sum[geomID] + prim_type_offset;
                           uint total_offset = 0;
                                                                    
                           item.barrier(sycl::access::fence_space::local_space);

                           const TriQuadMesh &mesh = triQuadMesh[geomID];
                           
                           // ====================                           
                           // === TriangleMesh ===
                           // ====================                           
                           if (mesh.isTriangleMesh())
                           {
                             const uint numTriangles = mesh.getNumPrimitives();
                             const uint numBlocks    = (numTriangles + MERGE_TRIANGLES_TO_QUADS_SEARCH_WG_SIZE - 1) / MERGE_TRIANGLES_TO_QUADS_SEARCH_WG_SIZE;
                                                                        
                             for (uint blockID = 0; blockID < numBlocks; blockID++)
                             {                                                                          
                               const uint startPrimID  = blockID*MERGE_TRIANGLES_TO_QUADS_SEARCH_WG_SIZE;
                               const uint endPrimID    = min(startPrimID+MERGE_TRIANGLES_TO_QUADS_SEARCH_WG_SIZE,numTriangles);
                               const uint ID           = (startPrimID + localID) < endPrimID ? startPrimID + localID : -1;
                               {
                                 uint3 tri_indices;
                                 gpu::AABB3f bounds;
                                 bounds.init();
                                 bool valid = ID < endPrimID ? isValidTriangle(mesh,ID,tri_indices,bounds) : false;
                                 bool paired = false;
                                 uint paired_ID = -1;
                                 uint active_mask = sub_group_ballot(valid);

                                 while(active_mask)
                                 {
                                   active_mask = sub_group_broadcast(active_mask,0);
                                                                              
                                   const uint broadcast_lane = sycl::ctz(active_mask);

                                   if (subgroupLocalID == broadcast_lane) valid = false;
                                                                            
                                   active_mask &= active_mask-1;
                                                                              
                                   const bool broadcast_paired = sub_group_broadcast(paired, broadcast_lane);
                                   const uint broadcast_ID     = sub_group_broadcast(ID    , broadcast_lane);

                                   if (!broadcast_paired)
                                   {
                                     const uint3 tri_indices_broadcast(sub_group_broadcast(tri_indices.x(),broadcast_lane),
                                                                       sub_group_broadcast(tri_indices.y(),broadcast_lane),
                                                                       sub_group_broadcast(tri_indices.z(),broadcast_lane));
                                     bool pairable = false;
                                     if (ID != broadcast_ID && !paired && valid)
                                       pairable = try_pair_triangles(tri_indices_broadcast,tri_indices);
                                                                            
                                     const uint first_paired_lane = sycl::ctz(sub_group_ballot(pairable));
                                     if (first_paired_lane < subgroupSize)
                                     {
                                       active_mask &= ~((uint)1 << first_paired_lane);
                                       if (subgroupLocalID == first_paired_lane) { valid = false; }
                                       const uint secondID = sub_group_broadcast(ID,first_paired_lane);
                                       gpu::AABB3f second_bounds = bounds.sub_group_broadcast(first_paired_lane);
                                       if (subgroupLocalID == broadcast_lane)  {
                                         paired_ID = secondID;
                                         bounds.extend(second_bounds);
                                       }
                                     }
                                     else
                                       if (subgroupLocalID == broadcast_lane)
                                         paired_ID = ID;
                                                                                
                                   }
                                 }
                                                                          
                                 const uint flag = paired_ID != -1 ? 1 : 0;
                                 const uint ps = ID < endPrimID ? flag : 0;
                                 const uint exclusive_scan = sub_group_exclusive_scan(ps, std::plus<uint>());
                                 const uint reduction = sub_group_reduce(ps, std::plus<uint>());
                                 counts[subgroupID] = reduction;
                                                                          
                                 item.barrier(sycl::access::fence_space::local_space);

                                 /* -- prefix sum over reduced sub group counts -- */
        
                                 uint total_reduction = 0;
                                 uint p_sum = 0;
                                 for (uint j=0;j<MERGE_TRIANGLES_TO_QUADS_SEARCH_WG_SIZE/subgroupSize;j++)
                                 {
                                   if (j<subgroupID) p_sum += counts[j];
                                   total_reduction += counts[j];
                                 }
                                                                          
                                 item.barrier(sycl::access::fence_space::local_space);

                                 const uint dest_offset = startQuadOffset + total_offset + p_sum + exclusive_scan;
                                 total_offset += total_reduction;

                                 /* --- store cluster representative into destination array --- */
                                 if (ID < endPrimID)
                                   if (paired_ID != -1)
                                   {
                                     const uint pair_offset = paired_ID - ID;
                                     const uint pair_geomID = (pair_offset << 24) | geomID;
                                     //bvh2[dest_offset].initLeaf(pair_geomID,ID,bounds); // need to consider pair_offset
                                     BVH2Ploc node;
                                     node.initLeaf(pair_geomID,ID,bounds); // need to consider pair_offset
                                     node.store(&bvh2[dest_offset]);
                                   }                                                                          
                               }
                             }                                                                        
                           }
                           // ================                           
                           // === QuadMesh ===
                           // ================                           
                           else
                           {
                             const uint numQuads = mesh.getNumPrimitives();
                             const uint numBlocks    = (numQuads + MERGE_TRIANGLES_TO_QUADS_SEARCH_WG_SIZE - 1) / MERGE_TRIANGLES_TO_QUADS_SEARCH_WG_SIZE;
                             for (uint blockID = 0; blockID < numBlocks; blockID++)
                             {                                                                          
                               const uint startPrimID  = blockID*MERGE_TRIANGLES_TO_QUADS_SEARCH_WG_SIZE;
                               const uint endPrimID    = min(startPrimID+MERGE_TRIANGLES_TO_QUADS_SEARCH_WG_SIZE,numQuads);
                               const uint ID           = (startPrimID + localID) < endPrimID ? startPrimID + localID : -1;
                               {
                                 uint4 quad_indices;
                                 gpu::AABB3f bounds;
                                 bounds.init();
                                 const bool valid = ID < endPrimID ? isValidQuad(mesh,ID,quad_indices,bounds) : false;                                                                          
                                 const uint ps = valid ? 1 : 0;
                                 const uint exclusive_scan = sub_group_exclusive_scan(ps, std::plus<uint>());
                                 const uint reduction = sub_group_reduce(ps, std::plus<uint>());
                                 counts[subgroupID] = reduction;
                                                                          
                                 item.barrier(sycl::access::fence_space::local_space);

                                 /* -- prefix sum over reduced sub group counts -- */
        
                                 uint total_reduction = 0;
                                 uint p_sum = 0;
                                 for (uint j=0;j<MERGE_TRIANGLES_TO_QUADS_SEARCH_WG_SIZE/subgroupSize;j++)
                                 {
                                   if (j<subgroupID) p_sum += counts[j];
                                   total_reduction += counts[j];
                                 }
                                                                          
                                 item.barrier(sycl::access::fence_space::local_space);

                                 const uint dest_offset = startQuadOffset + total_offset + p_sum + exclusive_scan;
                                 total_offset += total_reduction;

                                 /* --- store cluster representative into destination array --- */
                                 if (ID < endPrimID)
                                   if (valid)
                                   {
                                     BVH2Ploc node;
                                     node.initLeaf(geomID,ID,bounds); // need to consider pair_offset
                                     node.store(&bvh2[dest_offset]);
                                   }                                                                          
                               }
                             }                                                                                                     
                           }
                             
                                                                                                                                                                     
                         });
		  
      });
    gpu::waitOnQueueAndCatchException(gpu_queue);
    double dt = gpu::getDeviceExecutionTiming(queue_event);      
    iteration_time += dt;
    if (unlikely(verbose)) PRINT2("merge triangles per geometry and write out quads", (float)dt);
  }
  


  __forceinline void createInstances_initPLOCPrimRefs(sycl::queue &gpu_queue, Scene *const scene, const uint numGeoms, BVH2Ploc *const bvh2, const uint prim_type_offset, double &iteration_time, const bool verbose)    
  {
    static const uint CREATE_INSTANCES_SEARCH_WG_SIZE  = 256;
    const sycl::nd_range<1> nd_range1(gpu::alignTo(numGeoms,CREATE_INSTANCES_SEARCH_WG_SIZE),sycl::range<1>(CREATE_INSTANCES_SEARCH_WG_SIZE));
    sycl::event queue_event = gpu_queue.submit([&](sycl::handler &cgh) {
        cgh.parallel_for(nd_range1,[=](sycl::nd_item<1> item)       
                         {
                           const uint instID = item.get_global_id(0);
                           if (instID < numGeoms)
                           {
                             const bool isInstance = scene->geometries[instID]->getTypeMask() & Instance::geom_type;
                             Instance* instance = scene->get<Instance>(instID);                             
                             gpu::AABB3f bounds(float3(0.0f));
                             BVH2Ploc node;                             
                             if (isInstance)
                             {
                               BBox3fa instance_bounds(instance->bounds(0));
                               bounds = gpu::AABB3f(instance_bounds.lower.x,instance_bounds.lower.y,instance_bounds.lower.z,
                                                    instance_bounds.upper.x,instance_bounds.upper.y,instance_bounds.upper.z);
                             }
                             node.initLeaf(0,instID,bounds);                               
                             node.store(&bvh2[prim_type_offset + instID]);
                           }
                         });
		  
      });
    gpu::waitOnQueueAndCatchException(gpu_queue);
    double dt = gpu::getDeviceExecutionTiming(queue_event);      
    iteration_time += dt;
    if (unlikely(verbose)) PRINT2("write out instance bounds", (float)dt);
  }

  __forceinline void createUserGeometries_initPLOCPrimRefs(sycl::queue &gpu_queue, Scene *const scene, const uint numGeoms, BVH2Ploc *const bvh2, const uint prim_type_offset, double &iteration_time, const bool verbose)    
  {
    //static const uint CREATE_INSTANCES_SEARCH_WG_SIZE  = 256;
    uint ID = 0;
    for (uint userGeomID=0;userGeomID<numGeoms;userGeomID++)
    {
      UserGeometry* userGeometry = scene->getSafe<UserGeometry>(userGeomID);                             
      if (userGeometry)
      {
        for (uint i=0;i<userGeometry->size();i++)
        {
          BBox3fa userGeometry_bounds(userGeometry->bounds(i));
          gpu::AABB3f bounds = gpu::AABB3f(userGeometry_bounds.lower.x,userGeometry_bounds.lower.y,userGeometry_bounds.lower.z,
                                           userGeometry_bounds.upper.x,userGeometry_bounds.upper.y,userGeometry_bounds.upper.z);
          BVH2Ploc node;                             
          node.initLeaf(userGeomID,ID,bounds);                               
          node.store(&bvh2[prim_type_offset + ID]);
          ID++;
        }          
      }
    }
#if 0    
    const sycl::nd_range<1> nd_range1(gpu::alignTo(numGeoms,CREATE_INSTANCES_SEARCH_WG_SIZE),sycl::range<1>(CREATE_INSTANCES_SEARCH_WG_SIZE));
    sycl::event queue_event = gpu_queue.submit([&](sycl::handler &cgh) {
        cgh.parallel_for(nd_range1,[=](sycl::nd_item<1> item)       
                         {
                           const uint userGeomID = item.get_global_id(0);
                           if (userGeomID < numGeoms)
                           {
                             const bool isUserGeometry = scene->geometries[userGeomID]->getTypeMask() & UserGeometry::geom_type;
                             UserGeometry* userGeometry = scene->get<UserGeometry>(userGeomID);                             
                             gpu::AABB3f bounds(float3(0.0f));
                             BVH2Ploc node;                             
                             if (isUserGeometry)
                             {
                               BBox3fa userGeometry_bounds(userGeometry->bounds(0));
                               bounds = gpu::AABB3f(userGeometry_bounds.lower.x,userGeometry_bounds.lower.y,userGeometry_bounds.lower.z,
                                                    userGeometry_bounds.upper.x,userGeometry_bounds.upper.y,userGeometry_bounds.upper.z);
                             }
                             node.initLeaf(0,userGeomID,bounds);                               
                             node.store(&bvh2[prim_type_offset + userGeomID]);
                           }
                         });
		  
      });
    gpu::waitOnQueueAndCatchException(gpu_queue);
    double dt = gpu::getDeviceExecutionTiming(queue_event);      
    iteration_time += dt;
    if (unlikely(verbose)) PRINT2("write out user geom/procedurals bounds", (float)dt);
#endif    
  }
  
  

 

  // ===================================================================================================================================================================================
  // =========================================================================== DISTANCE FUNCTION =====================================================================================
  // ===================================================================================================================================================================================

  __forceinline float distanceFct(const gpu::AABB3f &bounds0,const gpu::AABB3f &bounds1)
  {
    const gpu::AABB3f bounds = gpu::merge(bounds0,bounds1);
    return bounds.halfArea();
  }
  

  // ====================================================================================================================================================================================
  // ================================================================================= SETUP ============================================================================================
  // ====================================================================================================================================================================================
  

  __forceinline void computeCentroidGeometryBounds(sycl::queue &gpu_queue, gpu::AABB3f *geometryBounds, gpu::AABB3f *centroidBounds, const BVH2Ploc *const bvh2, const uint numPrimitives, double &iteration_time, const bool verbose)
  {
    const uint wgSize = 1024;
    const sycl::nd_range<1> nd_range1(gpu::alignTo(numPrimitives,wgSize),sycl::range<1>(wgSize));          
    sycl::event queue_event = gpu_queue.submit([&](sycl::handler &cgh) {
        sycl::accessor< gpu::AABB3f, 0, sycl_read_write, sycl_local> _local_geometry_aabb(cgh);
        sycl::accessor< gpu::AABB3f, 0, sycl_read_write, sycl_local> _local_centroid_aabb(cgh);
                                                       
        cgh.parallel_for(nd_range1,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(16)
                         {
                           const uint localID        = item.get_local_id(0);
                           const uint subgroupLocalID = get_sub_group_local_id();
                           const uint ID = item.get_global_id(0);                                                                                                               
                           gpu::AABB3f *local_geometry_aabb = _local_geometry_aabb.get_pointer();
                           gpu::AABB3f *local_centroid_aabb = _local_centroid_aabb.get_pointer();
                                                                                    
                           gpu::AABB3f geometry_aabb;
                           gpu::AABB3f centroid_aabb;
                           geometry_aabb.init();
                           centroid_aabb.init();

                           if (ID < numPrimitives)
                           {                             
                             const gpu::AABB3f aabb_geom = bvh2[ID].bounds;
                             const gpu::AABB3f aabb_centroid(aabb_geom.centroid2()); // FIXME
                             geometry_aabb.extend(aabb_geom);
                             centroid_aabb.extend(aabb_centroid);		      
                                                                                    
                             if (localID == 0)
                             {
                               local_geometry_aabb->init();
                               local_centroid_aabb->init();                                                                                      
                             }
                           }
                           item.barrier(sycl::access::fence_space::local_space);

                           geometry_aabb = geometry_aabb.sub_group_reduce(); 
                           centroid_aabb = centroid_aabb.sub_group_reduce(); 

                            if (subgroupLocalID == 0) 
                           {
                             geometry_aabb.atomic_merge_local(*local_geometry_aabb);
                             centroid_aabb.atomic_merge_local(*local_centroid_aabb);
                           }
                           
                           item.barrier(sycl::access::fence_space::local_space);

                           if (localID == 0)
                           {
                             local_geometry_aabb->atomic_merge_global(*geometryBounds);
                             local_centroid_aabb->atomic_merge_global(*centroidBounds);
                           }
                         });
		  
      });
    gpu::waitOnQueueAndCatchException(gpu_queue);
    double dt = gpu::getDeviceExecutionTiming(queue_event);
    iteration_time += dt;
    if (unlikely(verbose)) PRINT2("computeCentroidGeometryBounds", (float)dt);
  }
  
  template<typename type>
    __forceinline void computeMortonCodes64Bit(sycl::queue &gpu_queue, const gpu::AABB3f *const _centroidBounds, type *const mc0, const BVH2Ploc *const bvh2, const uint numPrimitives, const uint shift, const uint64_t mask, double &iteration_time, const bool verbose)    
  {
    const uint wgSize = 16;
    const sycl::nd_range<1> nd_range1(gpu::alignTo(numPrimitives,wgSize),sycl::range<1>(wgSize));              
    sycl::event queue_event = gpu_queue.submit([&](sycl::handler &cgh) {
        cgh.parallel_for(nd_range1,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(32)
                         {
                           const uint globalID     = item.get_global_id(0);                                                                    
                           if (globalID < numPrimitives)
                           {
                             const gpu::AABB3f centroidBounds(*_centroidBounds);
                             const uint i = globalID;

                             const float3 lower = centroidBounds.lower();                                                                      
                             const uint   grid_size   = 1 << type::GRID_SHIFT;
                             const float3 grid_base(lower.x(),lower.y(),lower.z());
                             const float3 grid_extend(centroidBounds.maxDiagDim());                             
                             const float3 grid_scale = cselect( int3(grid_extend != 0.0f), (grid_size * 0.99f)/grid_extend, float3(0.0f)); // FIXME: 0.99f!!!!!

                             /* calculate and store morton code */
                             const gpu::AABB3f bounds3f = bvh2[i].bounds;
                             const float3 centroid = bounds3f.centroid2();
                             const float3 gridpos_f = (centroid-grid_base)*grid_scale;                                                                      
                             const sycl::uint3 gridpos = gridpos_f.convert<sycl::uint,sycl::rounding_mode::rtz>();                             
                             const uint64_t code = (gpu::bitInterleave3D_64bits(gridpos)>>shift) & mask;
                             mc0[i] = type(code,i); 
                           }
                         });
		  
      });
    gpu::waitOnQueueAndCatchException(gpu_queue);
    double dt = gpu::getDeviceExecutionTiming(queue_event);      
    if (unlikely(verbose)) PRINT2("compute 3D morton codes ",(float)dt);
    iteration_time += dt;
  }

  template<typename type>
    __forceinline void computeMortonCodes64Bit_SaveMSBBits(sycl::queue &gpu_queue, const gpu::AABB3f *const _centroidBounds, type *const mc0, const BVH2Ploc *const bvh2, uint *const high, const uint numPrimitives, double &iteration_time, const bool verbose)    
  {
    const uint wgSize = 16; //256;
    const sycl::nd_range<1> nd_range1(gpu::alignTo(numPrimitives,wgSize),sycl::range<1>(wgSize));              
    sycl::event queue_event = gpu_queue.submit([&](sycl::handler &cgh) {
        cgh.parallel_for(nd_range1,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(32)
                         {
                           const uint globalID     = item.get_global_id(0);                                                                    
                           if (globalID < numPrimitives)
                           {
                             const gpu::AABB3f centroidBounds(*_centroidBounds);
                             const uint i = globalID;

                             const float3 lower = centroidBounds.lower();                                                                      
                             const uint   grid_size   = 1 << type::GRID_SHIFT;
                             const float3 grid_base(lower.x(),lower.y(),lower.z());
                             const float3 grid_extend(centroidBounds.maxDiagDim());                             
                             const float3 grid_scale = cselect( int3(grid_extend != 0.0f), (grid_size * 0.99f)/grid_extend, float3(0.0f)); // FIXME: 0.99f!!!!!

                             /* calculate and store morton code */
                             const gpu::AABB3f bounds3f = bvh2[i].bounds;
                             const float3 centroid = bounds3f.centroid2();
                             const float3 gridpos_f = (centroid-grid_base)*grid_scale;                                                                      
                             const sycl::uint3 gridpos = gridpos_f.convert<sycl::uint,sycl::rounding_mode::rtz>();
                             const uint64_t mask = (((uint64_t)1 << 32)-1);
                             const uint64_t code = gpu::bitInterleave3D_64bits(gridpos);
                             high[i] = code >> 32;
                             mc0[i]  = type(code & mask,i); 
                           }
                         });
		  
      });
    gpu::waitOnQueueAndCatchException(gpu_queue);
    double dt = gpu::getDeviceExecutionTiming(queue_event);      
    if (unlikely(verbose)) PRINT2("compute 3D morton codes ",(float)dt);
    iteration_time += dt;
  }
  
  template<typename type>
    __forceinline void restoreMSBBits(sycl::queue &gpu_queue, type *const mc0, uint *const high, const uint numPrimitives, double &iteration_time, const bool verbose)    
  {
    const uint wgSize = 16; 
    const sycl::nd_range<1> nd_range1(gpu::alignTo(numPrimitives,wgSize),sycl::range<1>(wgSize));              
    sycl::event queue_event = gpu_queue.submit([&](sycl::handler &cgh) {
        cgh.parallel_for(nd_range1,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(32)
                         {
                           const uint globalID     = item.get_global_id(0);                                                                    
                           if (globalID < numPrimitives)
                           {
                             const uint index = mc0[globalID].getIndex();
                             const uint64_t code = high[index];                             
                             mc0[globalID] = type(code,index);                                        
                           }
                         });
		  
      });
    gpu::waitOnQueueAndCatchException(gpu_queue);
    double dt = gpu::getDeviceExecutionTiming(queue_event);      
    if (unlikely(verbose)) PRINT2("restore Morton Code MSB Bits ",(float)dt);
    iteration_time += dt;
  }
  

  template<typename type>  
    __forceinline void initClusters(sycl::queue &gpu_queue, type *const mc0, const BVH2Ploc *const bvh2, uint *const cluster_index, uint *const bvh2_subtree_size, const uint numPrimitives, double &iteration_time, const bool verbose)    
  {
    static const uint INIT_CLUSTERS_WG_SIZE = 256;    
    const sycl::nd_range<1> nd_range1(sycl::range<1>(gpu::alignTo(numPrimitives,INIT_CLUSTERS_WG_SIZE)),sycl::range<1>(INIT_CLUSTERS_WG_SIZE)); 
    sycl::event queue_event = gpu_queue.submit([&](sycl::handler &cgh) {
        cgh.parallel_for(nd_range1,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(16)
                         {
                           const uint globalID     = item.get_global_id(0);
                           if (globalID < numPrimitives)
                           {
                             const uint index = mc0[globalID].getIndex();
                             bvh2_subtree_size[globalID] = 1;
                             cluster_index[globalID] = index;
                           }
                         });
                                                       
      });
    gpu::waitOnQueueAndCatchException(gpu_queue);
    double dt = gpu::getDeviceExecutionTiming(queue_event);      
    if (unlikely(verbose)) PRINT2("init leaf clusters",(float)dt);
    iteration_time += dt; 
  }


  // ====================================================================================================================================================================================
  // ====================================================================================================================================================================================
  // ====================================================================================================================================================================================


  __forceinline  uint encodeRelativeOffset(const int localID, const int neighbor)
  {
    const int sOffset = neighbor - localID;
    const uint uOffset = sycl::abs(sOffset)-1;
    return (uOffset<<1) | ((uint)sOffset>>31);
  }

  __forceinline int decodeRelativeOffset(const int localID, const uint offset)
  {
    const uint off = (offset>>1)+1;
    return localID + ((offset % 2 == 0) ? (int)off : -(int)off);
  }
  
  
  __forceinline void iteratePLOC (sycl::queue &gpu_queue, PLOCGlobals *const globals, BVH2Ploc *const bvh2, uint *const cluster_index_source, uint *const cluster_index_dest, uint *const bvh2_subtree_size, uint *const scratch_mem, const uint numPrims, const int RADIUS, const uint NN_SEARCH_WG_NUM, uint *host_device_tasks, double &iteration_time, const bool verbose)    
  {
    static const uint NN_SEARCH_SUB_GROUP_WIDTH = 16;
    static const uint NN_SEARCH_WG_SIZE         = 1024;
    uint *const bvh2_index_allocator = &globals->bvh2_index_allocator;
    
    sycl::event queue_event = gpu_queue.submit([&](sycl::handler &cgh) {
        sycl::accessor< gpu::AABB3f, 1, sycl_read_write, sycl_local> cached_bounds  (sycl::range<1>(NN_SEARCH_WG_SIZE),cgh);
        sycl::accessor< uint       , 1, sycl_read_write, sycl_local> cached_neighbor(sycl::range<1>(NN_SEARCH_WG_SIZE),cgh);
        sycl::accessor< uint       , 1, sycl_read_write, sycl_local> cached_clusterID(sycl::range<1>(NN_SEARCH_WG_SIZE),cgh);        
        sycl::accessor< uint       , 1, sycl_read_write, sycl_local> counts(sycl::range<1>((NN_SEARCH_WG_SIZE/NN_SEARCH_SUB_GROUP_WIDTH)),cgh);
        sycl::accessor< uint       , 1, sycl_read_write, sycl_local> counts_prefix_sum(sycl::range<1>((NN_SEARCH_WG_SIZE/NN_SEARCH_SUB_GROUP_WIDTH)),cgh);        
        sycl::accessor< uint      ,  0, sycl_read_write, sycl_local> _wgID(cgh);
        sycl::accessor< uint      ,  0, sycl_read_write, sycl_local> _global_count_prefix_sum(cgh);
        
        
        const sycl::nd_range<1> nd_range(sycl::range<1>(NN_SEARCH_WG_NUM*NN_SEARCH_WG_SIZE),sycl::range<1>(NN_SEARCH_WG_SIZE));		  
        cgh.parallel_for(nd_range,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(NN_SEARCH_SUB_GROUP_WIDTH) {
            const uint localID        = item.get_local_id(0);
            const uint localSize       = item.get_local_range().size();            
            const uint subgroupID      = get_sub_group_id();                                                                                                                          
            const uint subgroupLocalID = get_sub_group_local_id();
            const uint subgroupSize    = NN_SEARCH_SUB_GROUP_WIDTH;                                                                                                                       
            const uint WORKING_WG_SIZE = NN_SEARCH_WG_SIZE - 4*RADIUS; /* reducing working group set size to 1024 - 4 * radius to avoid loops */
            

            uint &wgID = *_wgID.get_pointer();
            uint &global_count_prefix_sum = *_global_count_prefix_sum.get_pointer();

            if (localID == 0)
              wgID = gpu::atomic_add_global(&globals->wgID,(uint)1);

            item.barrier(sycl::access::fence_space::local_space);
            
            const uint groupID        = wgID;                                                             
            const uint startID        = (groupID + 0)*numPrims / NN_SEARCH_WG_NUM;
            const uint endID          = (groupID + 1)*numPrims / NN_SEARCH_WG_NUM;
            const uint sizeID         = endID-startID;
            const uint aligned_sizeID = gpu::alignTo(sizeID,WORKING_WG_SIZE);
            
            
            uint total_offset = 0;                                                     
            for (uint t=0;t<aligned_sizeID;t+=WORKING_WG_SIZE)
            {
              /* -------------------------------------------------------- */                                                       
              /* --- copy AABBs from cluster representatives into SLM --- */
              /* -------------------------------------------------------- */
                                                       
              const int local_window_start = max((int)(startID+t                )-2*RADIUS  ,(int)0);
              const int local_window_end   = min((int)(startID+t+WORKING_WG_SIZE)+2*RADIUS+1,(int)numPrims);
              const int local_window_size = local_window_end - local_window_start;
              const uint ID = startID + localID + t;
              const uint maxID = min(startID + t + WORKING_WG_SIZE,endID);

              const uint clusterID = cluster_index_source[min(local_window_start+(int)localID,local_window_end-1)];
              cached_bounds[localID] = bvh2[clusterID].bounds;
              cached_clusterID[localID] = clusterID;

#if SINGLE_R_PATH == 1              
              cached_neighbor[localID] = -1;              
#endif              
              item.barrier(sycl::access::fence_space::local_space);

              /* ---------------------------------------------------------- */                                                       
              /* --- compute nearest neighbor and store result into SLM --- */
              /* ---------------------------------------------------------- */

#if SINGLE_R_PATH == 0

              if (localID < local_window_size)
              {                                                         
                const int start = max((int)localID-RADIUS,(int)0);
                const int end = min((int)localID+RADIUS+1,(int)local_window_size);
                const gpu::AABB3f bounds = cached_bounds[localID];                  
                uint area_min = -1; 
                int area_min_index = -1; 
                
                /* ------------------------------------------------------------------------ */                
                /* --- fallback case for equal cluster representatives with equal bounds -- */
                /* ------------------------------------------------------------------------ */
                if (bounds.area() == 0.0f)
                {
                  if (localID < local_window_size - 1)
                  {
                    const uint next = (ID % 2 == 0) ? localID+1 : localID-1;
                    const float next_merged_area = distanceFct(bounds,cached_bounds[next]);
                    if (next_merged_area == 0.0f)
                    {
                      area_min_index = next;
                      area_min = gpu::as_uint(next_merged_area);
                    }
                  }
                }

                /* ---------------------------------------------- */                
                /* --- scan search radius for nearest neighbor -- */
                /* ---------------------------------------------- */                                
                
                for (int s=start;s<end;s++)
                {
                  const uint new_area = gpu::as_uint(distanceFct(bounds,cached_bounds[s]));
                  const bool update = s != localID && new_area < area_min;                                                                    
                  area_min = cselect(update,new_area,area_min); 
                  area_min_index = cselect(update,s,area_min_index);
                }                    
                
                /* ------------------------------ */                
                /* --- update nearest neighbor -- */
                /* ------------------------------ */                                
                cached_neighbor[localID] = local_window_start + (uint)area_min_index;                
              }
#else

              
              const uint encode_mask = ~(((uint)1<<(SEARCH_RADIUS_SHIFT+1))-1);
              const uint decode_mask =  (((uint)1<<(SEARCH_RADIUS_SHIFT+1))-1);
              
              uint min_area_index = -1;
              const gpu::AABB3f bounds0 = cached_bounds[localID];              
              for (uint r=1;r<=RADIUS && (localID + r < local_window_size) ;r++)
              {
                const gpu::AABB3f bounds1 = cached_bounds[localID+r];
                const float new_area = distanceFct(bounds0,bounds1);
                const uint new_area_i = (gpu::as_uint(new_area) << 1) & encode_mask;
                const uint encode0 = encodeRelativeOffset(localID  ,localID+r);
                const uint encode1 = encodeRelativeOffset(localID+r,localID);
                const uint new_area_index0 = new_area_i | encode0;
                const uint new_area_index1 = new_area_i | encode1;                  
                min_area_index = min(min_area_index,new_area_index0);                  
                gpu::atomic_min_local(&cached_neighbor[localID+r],new_area_index1);                  
              }

              /* --- work around for neighboring zero area bounds --- */
              const uint zero_min_area_mask = sub_group_ballot(min_area_index == 0);
              const uint first_zero_min_area_mask_index = sycl::ctz(zero_min_area_mask);
              if (min_area_index != 0 || (ID % 2 == first_zero_min_area_mask_index % 2))
                gpu::atomic_min_local(&cached_neighbor[localID],min_area_index);                                                    
#endif              
                                                       
              item.barrier(sycl::access::fence_space::local_space);
              
              /* ---------------------------------------------------------- */                                                       
              /* --- merge valid nearest neighbors and create bvh2 node --- */
              /* ---------------------------------------------------------- */
              
              uint new_cluster_index = -1;
              if (ID < maxID)
              {                
                new_cluster_index = cluster_index_source[ID];
#if SINGLE_R_PATH == 1

                const uint n_i     = decodeRelativeOffset(ID -local_window_start,cached_neighbor[ID -local_window_start] & decode_mask) + local_window_start;
                const uint n_i_n_i = decodeRelativeOffset(n_i-local_window_start,cached_neighbor[n_i-local_window_start] & decode_mask) + local_window_start;

                const gpu::AABB3f bounds = cached_bounds[ID - local_window_start];
                                                
#else
                const gpu::AABB3f bounds = cached_bounds[ID -local_window_start];                
                const uint n_i = cached_neighbor[ID-local_window_start];
                const uint n_i_n_i = cached_neighbor[n_i-local_window_start];
#endif                
                if (ID == n_i_n_i)  
                {
                  if (ID < n_i)
                  {
                    const uint leftIndex  = cached_clusterID[ID-local_window_start]; //cluster_index_source[ID];
                    const uint rightIndex = cached_clusterID[n_i-local_window_start]; //cluster_index_source[n_i];
                    const gpu::AABB3f &leftBounds  = bounds;
                    const gpu::AABB3f &rightBounds = cached_bounds[n_i-local_window_start];

                    /* --- reduce per subgroup to lower pressure on global atomic counter --- */                                                         
                    sycl::atomic_ref<uint, sycl::memory_order::relaxed, sycl::memory_scope::device,sycl::access::address_space::global_space> bvh2_counter(*bvh2_index_allocator);
                    const uint bvh2_index = gpu::sub_group_shared_global_atomic(bvh2_counter,1);

                    /* --- store new BVH2 node --- */
                    const uint new_size = bvh2_subtree_size[leftIndex] + bvh2_subtree_size[rightIndex];
                    bvh2[bvh2_index].init(leftIndex,rightIndex,gpu::merge(leftBounds,rightBounds),bvh2_subtree_size[leftIndex],bvh2_subtree_size[rightIndex]);                                                         
                    bvh2_subtree_size[bvh2_index] = new_size;
                    new_cluster_index = bvh2_index;
                  }
                  else
                    new_cluster_index = -1; /* --- second item of pair with the larger index disables the slot --- */
                }
              }

              const uint flag = new_cluster_index != -1 ? 1 : 0;
              const uint ps = ID < maxID ? flag : 0;
              const uint exclusive_scan = sub_group_exclusive_scan(ps, std::plus<uint>());
              const uint reduction = sub_group_reduce(ps, std::plus<uint>());
                                                     
              counts[subgroupID] = reduction;
                                                             
              item.barrier(sycl::access::fence_space::local_space);
              
              /* -- prefix sum over reduced sub group counts -- */
        
              uint total_reduction = 0;
              for (uint j=subgroupLocalID;j<NN_SEARCH_WG_SIZE/subgroupSize;j+=subgroupSize)
              {
                const uint subgroup_counts = counts[j];
                const uint sums_exclusive_scan = sub_group_exclusive_scan(subgroup_counts, std::plus<uint>());
                const uint reduction = sub_group_broadcast(subgroup_counts,subgroupSize-1) + sub_group_broadcast(sums_exclusive_scan,subgroupSize-1);
                counts_prefix_sum[j] = sums_exclusive_scan + total_reduction;
                total_reduction += reduction;
              }


              item.barrier(sycl::access::fence_space::local_space);

              const uint sums_prefix_sum = counts_prefix_sum[subgroupID];                                                                 
              const uint p_sum = startID + total_offset + sums_prefix_sum + exclusive_scan;

              /* --- store cluster representative into destination array --- */                                                                 
              if (ID < maxID)
                if (new_cluster_index != -1)
                  cluster_index_dest[p_sum] = new_cluster_index;
          
              total_offset += total_reduction;                                          
            }

            /* ----------------------------------------------------------------------------------------- */                                                       
            /* --- store number of valid cluster representatives into scratch mem and set valid flag --- */
            /* ----------------------------------------------------------------------------------------- */

            const uint flag = (uint)1<<31;            
            const uint mask = ~flag;
            
            if (localID == 0)
            {
              sycl::atomic_ref<uint,sycl::memory_order::relaxed, sycl::memory_scope::device,sycl::access::address_space::global_space> scratch_mem_counter(scratch_mem[groupID]);
              scratch_mem_counter.store(total_offset | flag);              
            }

            global_count_prefix_sum = 0;            
            
            item.barrier(sycl::access::fence_space::global_and_local);

            // =======================================            
            // wait until earlier WGs finished as well
            // =======================================

            if (localID < groupID /*NN_SEARCH_WG_NUM */)
            {
              sycl::atomic_ref<uint, sycl::memory_order::acq_rel, sycl::memory_scope::device,sycl::access::address_space::global_space> global_state(scratch_mem[localID]);
              uint c = 0;
              while( ((c = global_state.load()) & flag) == 0 );
              if (c)
                gpu::atomic_add_local(&global_count_prefix_sum,(uint)c & mask);
            }
            
            item.barrier(sycl::access::fence_space::local_space);

            /* ---------------------------------------------------- */                                                       
            /* --- prefix sum over per WG counts in scratch mem --- */
            /* ---------------------------------------------------- */
            const uint active_count = total_offset;
            const uint global_offset = global_count_prefix_sum;            
              
            for (uint t=localID;t<active_count;t+=localSize)
              cluster_index_source[global_offset + t] = cluster_index_dest[startID + t];                                                               

            /* -------------------------------------------------- */                                                       
            /* --- update number of clusters after compaction --- */
            /* -------------------------------------------------- */
                                         
            if (localID == 0 && groupID == NN_SEARCH_WG_NUM-1) // need to be the last group as only this one waits until all previous are done              
              *host_device_tasks = global_offset + active_count;
            
            /* -------------------------------- */                                                       
            /* --- last WG does the cleanup --- */
            /* -------------------------------- */

            if (localID == 0)
            {
              const uint syncID = gpu::atomic_add_global(&globals->sync,(uint)1);
              if (syncID == NN_SEARCH_WG_NUM-1)
              {
                /* --- reset atomics --- */
                globals->wgID = 0;
                globals->sync = 0;
                /* --- reset scratch_mem --- */
                for (uint i=0;i<NN_SEARCH_WG_NUM;i++) 
                  scratch_mem[i] = 0;                
              }
            }            
                                                     
          });		  
      });
    gpu::waitOnQueueAndCatchException(gpu_queue);
    double dt = gpu::getDeviceExecutionTiming(queue_event);      
    if (unlikely(verbose)) PRINT2("nearest neighbor search ",(float)dt);
    iteration_time += dt;
  }


  __forceinline void computePrefixSum_compactClusterReferences(sycl::queue &gpu_queue, uint *const numBuildRecords, const uint *const cluster_index_source, uint *const cluster_index_dest, uint *const scratch_mem, const uint numPrims, const uint PREFIX_SUM_WG_NUM, double &iteration_time, const bool verbose)    
  {
    static const uint PREFIX_SUM_SUB_GROUP_WIDTH = 32;
    static const uint PREFIX_SUM_WG_SIZE = PREFIX_SUM_SUB_GROUP_WIDTH*PREFIX_SUM_SUB_GROUP_WIDTH; 

    sycl::event queue_event;
    queue_event = gpu_queue.submit([&](sycl::handler &cgh) {
        const sycl::nd_range<1> nd_range(PREFIX_SUM_WG_NUM*PREFIX_SUM_WG_SIZE,sycl::range<1>(PREFIX_SUM_WG_SIZE));
        /* local variables */
        sycl::accessor< uint   , 1, sycl_read_write, sycl_local> global_wg_prefix_sum(sycl::range<1>(PREFIX_SUM_WG_NUM),cgh);
                                                         
        cgh.parallel_for(nd_range,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(PREFIX_SUM_SUB_GROUP_WIDTH) {
            const uint localID         = item.get_local_id(0);
            const uint groupID         = item.get_group(0);                                                             
            const uint localSize       = item.get_local_range().size();
            const uint subgroupLocalID = get_sub_group_local_id();
            const uint subgroupSize    = get_sub_group_size();                                                                                                                          
            const uint startID = (groupID + 0)*numPrims / PREFIX_SUM_WG_NUM;

            /* ---------------------------------------------------- */                                                       
            /* --- prefix sum over per WG counts in scratch mem --- */
            /* ---------------------------------------------------- */
                                         
            uint global_total = 0;
            for (uint i=0;i<PREFIX_SUM_WG_NUM;i+=subgroupSize)
            {
              const uint subgroup_counts     = i+subgroupLocalID < PREFIX_SUM_WG_NUM ? scratch_mem[i+subgroupLocalID] : 0;

              const uint total               = sub_group_reduce(subgroup_counts, std::plus<uint>());
              const uint sums_exclusive_scan = sub_group_exclusive_scan(subgroup_counts, std::plus<uint>());

              global_wg_prefix_sum[i+subgroupLocalID] = sums_exclusive_scan + global_total;
              global_total += total;                                           
            }

            
            const uint active_count = scratch_mem[groupID];
            item.barrier(sycl::access::fence_space::local_space);

            const uint global_offset = global_wg_prefix_sum[groupID];
              
            for (uint t=localID;t<active_count;t+=localSize)
              cluster_index_dest[global_offset + t] = cluster_index_source[startID + t];                                                               

            /* -------------------------------------------------- */                                                       
            /* --- update number of clusters after compaction --- */
            /* -------------------------------------------------- */
                                         
            if (localID == 0 && groupID == 0)
            {
              *numBuildRecords = global_total;              
            }
          });		  
      });
    gpu::waitOnQueueAndCatchException(gpu_queue);
    double dt = gpu::getDeviceExecutionTiming(queue_event);      
    if (unlikely(verbose)) PRINT2("prefix sum and cluster reference compaction",(float)dt);
    iteration_time += dt;
  }
  
  
  // ====================================================================================================================================================================================
  // ====================================================================================================================================================================================
  // ====================================================================================================================================================================================
  
  
  __forceinline uint wgBuild(sycl::nd_item<1> &item,
                             uint *const bvh2_index_allocator,
                             const uint startID,
                             const uint endID,
                             BVH2Ploc *const bvh2,
                             uint *const global_cluster_index_source,
                             uint *const global_cluster_index_dest,
                             uint *const bvh2_subtree_size,
                             gpu::AABB3f *const cached_bounds,
                             uint *const cached_neighbor,
                             uint *const cached_clusterID,                             
                             uint *const counts,
                             uint *const counts_prefix_sum,                             
                             uint &active_counter,
                             const uint BOTTOM_UP_THRESHOLD,
                             const int LOCAL_SEARCH_RADIUS = SEARCH_RADIUS,
                             const uint SINGLE_WG_SIZE = 1024)
  {
    const uint localID         = item.get_local_id(0);
    const uint localSize       = item.get_local_range().size();
    const uint subgroupLocalID = get_sub_group_local_id();
    const uint subgroupID      = get_sub_group_id();                                                                                                                          
    const uint subgroupSize    = get_sub_group_size();
    const uint WORKING_WG_SIZE = SINGLE_WG_SIZE - 4*LOCAL_SEARCH_RADIUS; 

    uint *const cluster_index_source = &global_cluster_index_source[startID];
    uint *const cluster_index_dest   = &global_cluster_index_dest[startID];
    
    uint numPrims = endID-startID; 
                                                     
    while(numPrims>BOTTOM_UP_THRESHOLD)
    {

      const uint aligned_numPrims = gpu::alignTo(numPrims,WORKING_WG_SIZE);

      uint total_offset = 0;
      for (uint t=0;t<aligned_numPrims;t+=WORKING_WG_SIZE)
      {
        /* -------------------------------------------------------- */                                                       
        /* --- copy AABBs from cluster representatives into SLM --- */
        /* -------------------------------------------------------- */
                                                         
        const int local_window_start = max((int)(t                )-2*LOCAL_SEARCH_RADIUS  ,(int)0);
        const int local_window_end   = min((int)(t+WORKING_WG_SIZE)+2*LOCAL_SEARCH_RADIUS+1,(int)numPrims);
        const int local_window_size = local_window_end - local_window_start;
        const uint ID    = localID + t;                                                             
        const uint maxID = min(t + WORKING_WG_SIZE,numPrims);                                                         

        /* --- fill the SLM bounds cache --- */
        
        const uint clusterID = cluster_index_source[min(local_window_start+(int)localID,local_window_end-1)];
        cached_bounds[localID] = bvh2[clusterID].bounds;
        cached_clusterID[localID] = clusterID;
        
#if SINGLE_R_PATH == 1              
        cached_neighbor[localID] = -1;              
#endif              
        
        item.barrier(sycl::access::fence_space::local_space);

        /* ---------------------------------------------------------- */                                                       
        /* --- compute nearest neighbor and store result into SLM --- */
        /* ---------------------------------------------------------- */

#if SINGLE_R_PATH == 0        
        if (localID < local_window_size)                                                           
        {
          const int start = max((int)localID-SEARCH_RADIUS,(int)0);
          const int end = min((int)localID+SEARCH_RADIUS+1,(int)local_window_size);
          const gpu::AABB3f bounds = cached_bounds[localID];                  
          uint area_min = -1; 
          int area_min_index = -1; 
                
          /* ------------------------------------------------------------------------ */                
          /* --- fallback case for equal cluster representatives with equal bounds -- */
          /* ------------------------------------------------------------------------ */

          if (bounds.area() == 0.0f)
          {
            if (localID < local_window_size - 1)
            {
              const uint next = (ID % 2 == 0) ? localID+1 : localID-1;
              const float next_merged_area = distanceFct(bounds,cached_bounds[next]);
              if (next_merged_area == 0.0f)
              {
                area_min_index = next;
                area_min = gpu::as_uint(next_merged_area);
              }
            }
          }

          /* ---------------------------------------------- */                
          /* --- scan search radius for nearest neighbor -- */
          /* ---------------------------------------------- */                                

          for (int s=start;s<end;s++)
          {
            const uint new_area = gpu::as_uint(distanceFct(bounds,cached_bounds[s]));
            const bool update = s != localID && new_area < area_min;                                                                    
            area_min = cselect(update,new_area,area_min); 
            area_min_index = cselect(update,s,area_min_index);
          }                    
                
          /* ------------------------------ */                
          /* --- update nearest neighbor -- */
          /* ------------------------------ */                                
          cached_neighbor[localID] = local_window_start + (uint)area_min_index;
        }
#else
        const uint encode_mask = ~(((uint)1<<(SEARCH_RADIUS_SHIFT+1))-1);
        const uint decode_mask =  (((uint)1<<(SEARCH_RADIUS_SHIFT+1))-1);
              
        uint min_area_index = -1;
        const gpu::AABB3f bounds0 = cached_bounds[localID];
        {              
          for (uint r=1;r<=SEARCH_RADIUS && (localID + r < local_window_size) ;r++)
          {
            const gpu::AABB3f bounds1 = cached_bounds[localID+r];
            const float new_area = distanceFct(bounds0,bounds1);
            const uint new_area_i = (gpu::as_uint(new_area) << 1) & encode_mask;
            const uint encode0 = encodeRelativeOffset(localID  ,localID+r);
            const uint encode1 = encodeRelativeOffset(localID+r,localID);
            const uint new_area_index0 = new_area_i | encode0;
            const uint new_area_index1 = new_area_i | encode1;                  
            min_area_index = min(min_area_index,new_area_index0);                  
            gpu::atomic_min_local(&cached_neighbor[localID+r],new_area_index1);
                  
          }
        }

        /* --- work around for neighboring zero area bounds --- */        
        const uint zero_min_area_mask = sub_group_ballot(min_area_index == 0);
        const uint first_zero_min_area_mask_index = sycl::ctz(zero_min_area_mask);
        if (min_area_index != 0 || (ID % 2 == first_zero_min_area_mask_index % 2))
          gpu::atomic_min_local(&cached_neighbor[localID],min_area_index);                      
#endif        

                                                                                                          
        item.barrier(sycl::access::fence_space::local_space); 

        
        /* ---------------------------------------------------------- */                                                       
        /* --- merge valid nearest neighbors and create bvh2 node --- */
        /* ---------------------------------------------------------- */

        uint new_cluster_index = -1;
        
        if (ID < maxID)
        {
          new_cluster_index = cluster_index_source[ID];

#if SINGLE_R_PATH == 1
          const uint n_i     = decodeRelativeOffset(ID -local_window_start,cached_neighbor[ID -local_window_start] & decode_mask) + local_window_start;
          const uint n_i_n_i = decodeRelativeOffset(n_i-local_window_start,cached_neighbor[n_i-local_window_start] & decode_mask) + local_window_start;                
          const gpu::AABB3f bounds = cached_bounds[ID -local_window_start];
          
#else
          const gpu::AABB3f bounds = cached_bounds[ID -local_window_start];          
          const uint n_i = cached_neighbor[ID-local_window_start];
          const uint n_i_n_i = cached_neighbor[n_i-local_window_start];
#endif
          if (ID == n_i_n_i)                  
          {
            if (ID < n_i)
            {
              const uint leftIndex  = cached_clusterID[ID-local_window_start]; 
              const uint rightIndex = cached_clusterID[n_i-local_window_start]; 
              const gpu::AABB3f &leftBounds  = bounds;
              const gpu::AABB3f &rightBounds = cached_bounds[n_i-local_window_start];
              
              
              /* --- reduce per subgroup to lower pressure on global atomic counter --- */                                                         
              sycl::atomic_ref<uint, sycl::memory_order::relaxed, sycl::memory_scope::device,sycl::access::address_space::global_space> bvh2_counter(*bvh2_index_allocator);
              const uint bvh2_index = gpu::sub_group_shared_global_atomic(bvh2_counter,1);

              /* --- store new BVH2 node --- */
              const uint new_size = bvh2_subtree_size[leftIndex] + bvh2_subtree_size[rightIndex];              
              bvh2[bvh2_index].init(leftIndex,rightIndex,gpu::merge(leftBounds,rightBounds),bvh2_subtree_size[leftIndex],bvh2_subtree_size[rightIndex]);                                                         
              bvh2_subtree_size[bvh2_index] = new_size;
              new_cluster_index = bvh2_index;
            }
            else
              new_cluster_index = -1; /* --- second item of pair with the larger index disables the slot --- */
          }
        }

        const uint flag = new_cluster_index != -1 ? 1 : 0;
        const uint ps = ID < maxID ? flag : 0;
        const uint exclusive_scan = sub_group_exclusive_scan(ps, std::plus<uint>());
        const uint reduction = sub_group_reduce(ps, std::plus<uint>());
                                                     
        counts[subgroupID] = reduction;
                                                             
        item.barrier(sycl::access::fence_space::local_space);

        /* -- prefix sum over reduced sub group counts -- */
        
        uint total_reduction = 0;
        for (uint j=subgroupLocalID;j<SINGLE_WG_SIZE/subgroupSize;j+=subgroupSize)
        {
          const uint subgroup_counts = counts[j];
          const uint sums_exclusive_scan = sub_group_exclusive_scan(subgroup_counts, std::plus<uint>());
          const uint reduction = sub_group_broadcast(subgroup_counts,subgroupSize-1) + sub_group_broadcast(sums_exclusive_scan,subgroupSize-1);
          counts_prefix_sum[j] = sums_exclusive_scan + total_reduction;
          total_reduction += reduction;
        }

        item.barrier(sycl::access::fence_space::local_space);

        const uint sums_prefix_sum = counts_prefix_sum[subgroupID];                                                                 
        const uint p_sum = total_offset + sums_prefix_sum + exclusive_scan;

        /* --- store cluster representative into destination array --- */                                                                 
        if (ID < maxID)
          if (new_cluster_index != -1)
            cluster_index_dest[p_sum] = new_cluster_index;
          
        total_offset += total_reduction;              
      }

      item.barrier(sycl::access::fence_space::global_and_local);

      /*-- copy elements back from dest to source -- */
      
      for (uint t=localID;t<total_offset;t+=localSize)
        cluster_index_source[t] = cluster_index_dest[t];      

      item.barrier(sycl::access::fence_space::local_space);
      
      numPrims = total_offset;      
    }
    return numPrims; /* return number of remaining cluster reps */
  }

  // ====================================================================================================================================================================================
  // ====================================================================================================================================================================================
  // ====================================================================================================================================================================================
  
  __forceinline void singleWGBuild(sycl::queue &gpu_queue, PLOCGlobals *const globals, BVH2Ploc *const bvh2, uint *const cluster_index_source, uint *const cluster_index_dest, uint *const bvh2_subtree_size, const uint numPrimitives, double &iteration_time, const bool verbose)
  {
    static const uint SINGLE_WG_SUB_GROUP_WIDTH = 16;
    static const uint SINGLE_WG_SIZE = 1024;
    uint *const bvh2_index_allocator = &globals->bvh2_index_allocator;
    
    sycl::event queue_event = gpu_queue.submit([&](sycl::handler &cgh) {
        const sycl::nd_range<1> nd_range(SINGLE_WG_SIZE,sycl::range<1>(SINGLE_WG_SIZE));

        /* local variables */
        sycl::accessor< gpu::AABB3f, 1, sycl_read_write, sycl_local> cached_bounds  (sycl::range<1>(SINGLE_WG_SIZE),cgh);
        sycl::accessor< uint       , 1, sycl_read_write, sycl_local> cached_neighbor(sycl::range<1>(SINGLE_WG_SIZE),cgh);
        sycl::accessor< uint       , 1, sycl_read_write, sycl_local> cached_clusterID(sycl::range<1>(SINGLE_WG_SIZE),cgh);        
        sycl::accessor< uint       , 1, sycl_read_write, sycl_local> counts(sycl::range<1>((SINGLE_WG_SIZE/SINGLE_WG_SUB_GROUP_WIDTH)),cgh);
        sycl::accessor< uint       , 1, sycl_read_write, sycl_local> counts_prefix_sum(sycl::range<1>((SINGLE_WG_SIZE/SINGLE_WG_SUB_GROUP_WIDTH)),cgh);
        sycl::accessor< uint      ,  0, sycl_read_write, sycl_local> _active_counter(cgh);
                                             
        cgh.parallel_for(nd_range,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(SINGLE_WG_SUB_GROUP_WIDTH) {
            uint &active_counter = *_active_counter.get_pointer();
            wgBuild(item, bvh2_index_allocator, 0, numPrimitives, bvh2, cluster_index_source, cluster_index_dest, bvh2_subtree_size, cached_bounds.get_pointer(), cached_neighbor.get_pointer(), cached_clusterID.get_pointer(), counts.get_pointer(),  counts_prefix_sum.get_pointer(), active_counter, 1, SEARCH_RADIUS, SINGLE_WG_SIZE);

            const uint localID        = item.get_local_id(0);                                                                 
            if (localID == 0) globals->rootIndex = globals->bvh2_index_allocator-1;
          });		  
      });            
    gpu::waitOnQueueAndCatchException(gpu_queue);
    double dt = gpu::getDeviceExecutionTiming(queue_event);      
    iteration_time += dt;
    if (unlikely(verbose)) PRINT2("single WG build ",(float)dt);    
  }

  

  __forceinline void parallelWGBuild(sycl::queue &gpu_queue, PLOCGlobals *const globals, BVH2Ploc *const bvh2, uint *const cluster_index_source, uint *const cluster_index_dest, uint *const bvh2_subtree_size, gpu::Range *const global_ranges, const uint numRanges, uint BOTTOM_UP_THRESHOLD, double &iteration_time, const bool verbose)
  {
    static const uint SINGLE_WG_SUB_GROUP_WIDTH = 16; 
    static const uint SINGLE_WG_SIZE = 1024;
    uint *const bvh2_index_allocator = &globals->bvh2_index_allocator;

    sycl::event queue_event = gpu_queue.submit([&](sycl::handler &cgh) {
        const sycl::nd_range<1> nd_range(sycl::range<1>(numRanges*SINGLE_WG_SIZE),sycl::range<1>(SINGLE_WG_SIZE));		  

        /* local variables */
        sycl::accessor< gpu::AABB3f, 1, sycl_read_write, sycl_local> cached_bounds  (sycl::range<1>(SINGLE_WG_SIZE),cgh);
        sycl::accessor< uint       , 1, sycl_read_write, sycl_local> cached_neighbor(sycl::range<1>(SINGLE_WG_SIZE),cgh);
        sycl::accessor< uint       , 1, sycl_read_write, sycl_local> cached_clusterID(sycl::range<1>(SINGLE_WG_SIZE),cgh);        
        sycl::accessor< uint       , 1, sycl_read_write, sycl_local> counts(sycl::range<1>((SINGLE_WG_SIZE/SINGLE_WG_SUB_GROUP_WIDTH)),cgh);
        sycl::accessor< uint       , 1, sycl_read_write, sycl_local> counts_prefix_sum(sycl::range<1>((SINGLE_WG_SIZE/SINGLE_WG_SUB_GROUP_WIDTH)),cgh);
        sycl::accessor< uint      ,  0, sycl_read_write, sycl_local> _active_counter(cgh);
                                             
        cgh.parallel_for(nd_range,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(SINGLE_WG_SUB_GROUP_WIDTH) {
            uint &active_counter = *_active_counter.get_pointer();
            const uint groupID        = item.get_group(0);
            const uint localID        = item.get_local_id(0);                                                     
            const uint localSize      = item.get_local_range().size();
            const uint startID = global_ranges[groupID].start;
            const uint endID   = global_ranges[groupID].end;

            const uint newNumPrims = wgBuild(item, bvh2_index_allocator, startID, endID, bvh2, cluster_index_source, cluster_index_dest, bvh2_subtree_size, cached_bounds.get_pointer(), cached_neighbor.get_pointer(), cached_clusterID.get_pointer(), counts.get_pointer(),  counts_prefix_sum.get_pointer(), active_counter,  BOTTOM_UP_THRESHOLD, SEARCH_RADIUS, SINGLE_WG_SIZE);
            
            /* --- copy current reps to dest array --- */
            global_ranges[groupID].end = startID + newNumPrims;
            item.barrier(sycl::access::fence_space::local_space);
            for (uint i=startID+localID;i<startID+newNumPrims;i+=localSize)
              cluster_index_dest[i] = cluster_index_source[i];
          });		  
      });            
    gpu::waitOnQueueAndCatchException(gpu_queue);
    double dt = gpu::getDeviceExecutionTiming(queue_event);      
    iteration_time += dt;
    if (unlikely(verbose)) PRINT2("parallel WG build ",(float)dt);    
  }

  __forceinline void singleWGTopLevelBuild(sycl::queue &gpu_queue, PLOCGlobals *const globals, BVH2Ploc *const bvh2, uint *const cluster_index_source, uint *const cluster_index_dest, uint *const bvh2_subtree_size, gpu::Range *const global_ranges, const uint numRanges, const uint SEARCH_RADIUS_TOP_LEVEL, double &iteration_time, const bool verbose)
  {
    static const uint SINGLE_WG_SUB_GROUP_WIDTH = 16;
    static const uint SINGLE_WG_SIZE = 1024;
    uint *const bvh2_index_allocator = &globals->bvh2_index_allocator;
    
    sycl::event queue_event = gpu_queue.submit([&](sycl::handler &cgh) {
        const sycl::nd_range<1> nd_range(SINGLE_WG_SIZE,sycl::range<1>(SINGLE_WG_SIZE));

        /* local variables */
        sycl::accessor< gpu::AABB3f, 1, sycl_read_write, sycl_local> cached_bounds  (sycl::range<1>(SINGLE_WG_SIZE),cgh);
        sycl::accessor< uint       , 1, sycl_read_write, sycl_local> cached_neighbor(sycl::range<1>(SINGLE_WG_SIZE),cgh);
        sycl::accessor< uint       , 1, sycl_read_write, sycl_local> cached_clusterID(sycl::range<1>(SINGLE_WG_SIZE),cgh);        
        sycl::accessor< uint       , 1, sycl_read_write, sycl_local> counts(sycl::range<1>((SINGLE_WG_SIZE/SINGLE_WG_SUB_GROUP_WIDTH)),cgh);
        sycl::accessor< uint       , 1, sycl_read_write, sycl_local> counts_prefix_sum(sycl::range<1>((SINGLE_WG_SIZE/SINGLE_WG_SUB_GROUP_WIDTH)),cgh);
        sycl::accessor< uint      ,  0, sycl_read_write, sycl_local> _active_counter(cgh);
                                             
        cgh.parallel_for(nd_range,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(SINGLE_WG_SUB_GROUP_WIDTH) {
            const uint localID        = item.get_local_id(0);
            const uint localSize      = item.get_local_range().size();                                                     
            uint &active_counter = *_active_counter.get_pointer();

            uint items = 0;
            for (uint r=0;r<numRanges;r++)
            {
              const uint startID = global_ranges[r].start;
              const uint size    = global_ranges[r].size();
                                                       
              /* copy from scratch mem */
              for (uint t=localID;t<size;t+=localSize)
                cluster_index_source[items + t] = cluster_index_dest[startID + t];
              items += size;
                                                       
            }
            
            item.barrier(sycl::access::fence_space::local_space);
              
            wgBuild(item, bvh2_index_allocator, 0, items, bvh2, cluster_index_source, cluster_index_dest, bvh2_subtree_size, cached_bounds.get_pointer(), cached_neighbor.get_pointer(), cached_clusterID.get_pointer(), counts.get_pointer(), counts_prefix_sum.get_pointer(), active_counter, 1, SEARCH_RADIUS_TOP_LEVEL, SINGLE_WG_SIZE);

            if (localID == 0) globals->rootIndex = globals->bvh2_index_allocator-1;
          });		  
      });
    gpu::waitOnQueueAndCatchException(gpu_queue);
    double dt = gpu::getDeviceExecutionTiming(queue_event);      
    iteration_time += dt;
    if (unlikely(verbose)) PRINT2("single WG top level build ",(float)dt);    
  }


  
  
  template<typename type>
    __forceinline void extractRanges(sycl::queue &gpu_queue, uint *const numBuildRecords, const type *const mc0, gpu::Range *const global_ranges, const uint numPrimitives, const uint RANGE_THRESHOLD, double &iteration_time, const bool verbose)
  {
    static const uint EXTRACT_WG_SIZE = 256;
    static const uint MAX_RANGES = 1024;
    
    sycl::event queue_event = gpu_queue.submit([&](sycl::handler &cgh) {
        const sycl::nd_range<1> nd_range(EXTRACT_WG_SIZE,sycl::range<1>(EXTRACT_WG_SIZE));

        /* local variables */
        sycl::accessor< gpu::Range, 1, sycl_read_write, sycl_local> ranges  (sycl::range<1>(MAX_RANGES),cgh);
        sycl::accessor< uint     ,  0, sycl_read_write, sycl_local> _numRanges(cgh);
                                             
        cgh.parallel_for(nd_range,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(16) {
            uint &num_ranges      = *_numRanges.get_pointer();

            const uint localID        = item.get_local_id(0);
            const uint localSize      = item.get_local_range().size();
                                                     
            num_ranges = 1;
            ranges[0] = gpu::Range(0,numPrimitives);

            uint iteration = 0;
            while(iteration < 32)
            {
              const uint current_num_ranges = num_ranges;
                                                       
              item.barrier(sycl::access::fence_space::local_space);
                                                       
              for (uint i=localID;i<current_num_ranges;i+=localSize)
                if (ranges[i].size() > RANGE_THRESHOLD)
                {
                  gpu::Range left,right;
                  splitRange(ranges[i],mc0,left,right);
                  if (left.size() > SEARCH_RADIUS && right.size() > SEARCH_RADIUS)
                  {
                    sycl::atomic_ref<uint, sycl::memory_order::relaxed, sycl::memory_scope::work_group,sycl::access::address_space::local_space> counter(num_ranges);
                    const uint new_index = counter.fetch_add(1);
                    ranges[i] = left;
                    ranges[new_index] = right;
                  }
                }
                                                     
              item.barrier(sycl::access::fence_space::local_space);
              iteration++;
              if (num_ranges == current_num_ranges) break;
            }

            for (uint i=localID;i<num_ranges;i+=localSize)
              global_ranges[i] = ranges[i];

            if (localID == 0)
              *numBuildRecords = num_ranges;
          });		  
      });
            
    gpu::waitOnQueueAndCatchException(gpu_queue);
    double dt = gpu::getDeviceExecutionTiming(queue_event);      
    iteration_time += dt;
    if (unlikely(verbose)) PRINT2("extract ranges ",(float)dt);
  }

  // ===================================================================================================================================================================================
  // ====================================================================== BVH2 -> QBVH6 conversion ===================================================================================
  // ===================================================================================================================================================================================


  __forceinline void getLeafIndices(const uint first_index, const BVH2Ploc *const bvh_nodes, uint *const dest, uint &indexID, const uint numPrimitives)
  {
    dest[0] = first_index;
    uint old_indexID = 0;
    indexID = 1;
    while(old_indexID != indexID)
    {
      old_indexID = indexID;
      for (uint i=0;i<old_indexID;i++)
        if (!BVH2Ploc::isLeaf(dest[i],numPrimitives))
        {
          const uint left = bvh_nodes[BVH2Ploc::getIndex(dest[i])].left;
          const uint right = bvh_nodes[BVH2Ploc::getIndex(dest[i])].right;
          dest[i]         = left;
          dest[indexID++] = right;
        }
    }
  }

  __forceinline void getLeafIndicesOrder(const uint first_index, const BVH2Ploc *const bvh_nodes, uint *const dest, uint &indexID, const uint numPrimitives)
  {
    dest[0] = first_index;
    uint old_indexID = 0;
    indexID = 1;
    while(old_indexID != indexID)
    {
      old_indexID = indexID;
      for (uint i=0;i<old_indexID;i++)
        if (!BVH2Ploc::isLeaf(dest[i],numPrimitives))
        {
          for (int j = old_indexID-1; j>i; j--)
            dest[j+1] = dest[j];
          const uint left = bvh_nodes[BVH2Ploc::getIndex(dest[i])].left;
          const uint right = bvh_nodes[BVH2Ploc::getIndex(dest[i])].right;
          dest[i+0] = left;
          dest[i+1] = right;
          indexID++;
          break;
        }
    }
  }
  

  __forceinline void writeNode(void *_dest, const uint relative_block_offset, const gpu::AABB3f &parent_bounds, const uint numChildren, uint indices[BVH_BRANCHING_FACTOR], const BVH2Ploc *const bvh2, const uint numPrimitives, const NodeType type, const GeometryTypeRanges &geometryTypeRanges, const bool forceFatLeaves = false)
  {
    uint *dest = (uint*)_dest;
    
    //dest = (uint*) __builtin_assume_aligned(dest,16);
    if (relative_block_offset == 0) PRINT2("FATAL",relative_block_offset);
    
    const float _ulp = std::numeric_limits<float>::epsilon();
    const float up = 1.0f + float(_ulp);  
    const gpu::AABB3f conservative_bounds = parent_bounds.conservativeBounds();
    const float3 len = conservative_bounds.size() * up;
      
    int _exp_x; float mant_x = frexp(len.x(), &_exp_x); _exp_x += (mant_x > 255.0f / 256.0f);
    int _exp_y; float mant_y = frexp(len.y(), &_exp_y); _exp_y += (mant_y > 255.0f / 256.0f);
    int _exp_z; float mant_z = frexp(len.z(), &_exp_z); _exp_z += (mant_z > 255.0f / 256.0f);
    _exp_x = max(-128,_exp_x); // enlarge too tight bounds
    _exp_y = max(-128,_exp_y);
    _exp_z = max(-128,_exp_z);

    const float3 lower(conservative_bounds.lower_x,conservative_bounds.lower_y,conservative_bounds.lower_z);
    
    dest[0]  = gpu::as_uint(lower.x());
    dest[1]  = gpu::as_uint(lower.y());
    dest[2]  = gpu::as_uint(lower.z());
    dest[3]  = relative_block_offset; 

    uint8_t tmp[48];
    
    tmp[0]         = type; // type
    tmp[1]         = 0;    // pad 
    tmp[2]         = _exp_x; assert(_exp_x >= -128 && _exp_x <= 127); 
    tmp[3]         = _exp_y; assert(_exp_y >= -128 && _exp_y <= 127);
    tmp[4]         = _exp_z; assert(_exp_z >= -128 && _exp_z <= 127);
    tmp[5]         = 0xff; //instanceMode ? 1 : 0xff;
    
#pragma nounroll
    for (uint i=0;i<BVH_BRANCHING_FACTOR;i++)
    {
      const uint index = BVH2Ploc::getIndex(indices[sycl::min(i,numChildren-1)]);
      // === default is invalid ===
      uint8_t lower_x = 0x80;
      uint8_t lower_y = 0x80;
      uint8_t lower_z = 0x80;    
      uint8_t upper_x = 0x00;
      uint8_t upper_y = 0x00;
      uint8_t upper_z = 0x00;
      uint8_t data    = 0x00;      
      // === determine leaf type ===
      const bool isLeaf = index < numPrimitives; // && !forceFatLeaves;      
      const bool isInstance   = geometryTypeRanges.isInstance(index);  
      const bool isProcedural = geometryTypeRanges.isProcedural(index);
      //PRINT3(isLeaf,isInstance,isProcedural);
      
      const uint numBlocks    = isInstance ? 2 : 1;
      NodeType leaf_type = NODE_TYPE_QUAD;
      leaf_type = isInstance ? NODE_TYPE_INSTANCE   : leaf_type;
      leaf_type = isProcedural ? NODE_TYPE_PROCEDURAL : leaf_type;      
      data = (i<numChildren) ? numBlocks : 0;
      data |= (isLeaf ? leaf_type : NODE_TYPE_INTERNAL) << 2;
      const gpu::AABB3f childBounds = bvh2[index].bounds; //.conservativeBounds();
      // === bounds valid ? ====
      uint equal_dims = childBounds.lower_x == childBounds.upper_x ? 1 : 0;
      equal_dims += childBounds.lower_y == childBounds.upper_y ? 1 : 0;
      equal_dims += childBounds.lower_z == childBounds.upper_z ? 1 : 0;
      const bool write = (i<numChildren) && equal_dims <= 1;
      // === quantize bounds ===
      const gpu::AABB3f  qbounds    = QBVHNodeN::quantize_bounds(lower, _exp_x, _exp_y, _exp_z, childBounds);
      // === updated discretized bounds ===
      lower_x = write ? (uint8_t)qbounds.lower_x : lower_x;
      lower_y = write ? (uint8_t)qbounds.lower_y : lower_y;
      lower_z = write ? (uint8_t)qbounds.lower_z : lower_z;
      upper_x = write ? (uint8_t)qbounds.upper_x : upper_x;
      upper_y = write ? (uint8_t)qbounds.upper_y : upper_y;
      upper_z = write ? (uint8_t)qbounds.upper_z : upper_z;
      // === init child in node ===
      tmp[ 6+i] = data;
      tmp[12+i] = lower_x;
      tmp[18+i] = upper_x;      
      tmp[24+i] = lower_y;
      tmp[30+i] = upper_y;      
      tmp[36+i] = lower_z;
      tmp[42+i] = upper_z;      
    }
    // === write out second part of 64 bytes node ===
    for (uint i=0;i<12;i++)
      dest[4+i] = tmp[i*4+0] | (tmp[i*4+1]<<8) | (tmp[i*4+2]<<16) | (tmp[i*4+3]<<24);
  }


  __forceinline uint openBVH2MaxAreaSortChildren(const uint index, uint indices[BVH_BRANCHING_FACTOR], const BVH2Ploc *const bvh2, const uint numPrimitives)
  {
    float areas[BVH_BRANCHING_FACTOR];
                                                                          
    const uint _left  = bvh2[BVH2Ploc::getIndex(index)].left;
    const uint _right = bvh2[BVH2Ploc::getIndex(index)].right;
                                                                          
    indices[0] = _left;
    indices[1] = _right;
#if USE_NEW_OPENING == 1
    const uint _left_numChildren  = BVH2Ploc::numFatChildren(_left);
    const uint _right_numChildren = BVH2Ploc::numFatChildren(_right);    
    areas[0]  = (_left_numChildren == 0  || _left_numChildren == 2 ) ? bvh2[BVH2Ploc::getIndex( _left)].bounds.area() : neg_inf;
    areas[1]  = (_right_numChildren == 0 || _right_numChildren == 2 ) ? bvh2[BVH2Ploc::getIndex(_right)].bounds.area() : neg_inf;
#else    
    areas[0]  = (!BVH2Ploc::isFatLeaf( _left)) ? bvh2[BVH2Ploc::getIndex( _left)].bounds.area() : neg_inf;    
    areas[1]  = (!BVH2Ploc::isFatLeaf(_right)) ? bvh2[BVH2Ploc::getIndex(_right)].bounds.area() : neg_inf; 
#endif
    
    uint numChildren = 2;
    while (numChildren < BVH_BRANCHING_FACTOR)
    {
      /*! find best child to split */
      float bestArea = areas[0];
      uint bestChild = 0;
      for (uint i=1;i<numChildren;i++)
        if (areas[i] > bestArea)
        {
          bestArea = areas[i];
          bestChild = i;
        }
            
      if (areas[bestChild] < 0.0f)
      {
#if USE_NEW_OPENING == 1        
        const uint free_space = BVH_BRANCHING_FACTOR - numChildren + 1;
        bestChild = -1;
        //if (free_space >= 3)
        {
          for (uint i=0;i<numChildren;i++)
            if (BVH2Ploc::numFatChildren(indices[i]) > 2 && BVH2Ploc::numFatChildren(indices[i]) <= free_space)
            {
              bestChild = i;
              break;
            }
        }
        if (bestChild == -1)
#endif          
          break; // nothing left to open
      }

      const uint bestNodeID = indices[bestChild];                                                                            
      const uint left  = bvh2[BVH2Ploc::getIndex(bestNodeID)].left;
      const uint right = bvh2[BVH2Ploc::getIndex(bestNodeID)].right;

#if USE_NEW_OPENING == 1
      const uint left_numChildren  = BVH2Ploc::numFatChildren(left);
      const uint right_numChildren = BVH2Ploc::numFatChildren(right);          
      areas[bestChild]     = (left_numChildren == 0  || left_numChildren == 2 ) ? bvh2[BVH2Ploc::getIndex(left)].bounds.area() : neg_inf;
      areas[numChildren]   = (right_numChildren == 0 || right_numChildren == 2) ? bvh2[BVH2Ploc::getIndex(right)].bounds.area() : neg_inf;      
#else      
      areas[bestChild]     = (!BVH2Ploc::isFatLeaf(left)) ? bvh2[BVH2Ploc::getIndex(left)].bounds.area() : neg_inf; 
      areas[numChildren]   = (!BVH2Ploc::isFatLeaf(right)) ? bvh2[BVH2Ploc::getIndex(right)].bounds.area() : neg_inf; 
#endif      
      indices[bestChild]   = left;      
      indices[numChildren] = right;                                                                            
      numChildren++;
    }            
    
    for (uint i=0;i<numChildren;i++)
      areas[i] = fabs(areas[i]);

    for (uint m=0; m<numChildren-1; m++)
      for (uint n=m+1; n<numChildren; n++)
        if (areas[m] < areas[n])
        {
          std::swap(areas[m],areas[n]);
          std::swap(indices[m],indices[n]);
        }

    return numChildren;
  }  

  __forceinline void write(const QuadLeaf &q, float16 *out) 
  {
    out[0].s0() = gpu::as_float(q.header[0]);      
    out[0].s1() = gpu::as_float(q.header[1]);
    out[0].s2() = gpu::as_float(q.header[2]);
    out[0].s3() = gpu::as_float(q.header[3]);
    out[0].s4() = q.v0.x;
    out[0].s5() = q.v0.y;
    out[0].s6() = q.v0.z;
    out[0].s7() = q.v1.x;
    out[0].s8() = q.v1.y;
    out[0].s9() = q.v1.z;
    out[0].sA() = q.v2.x;
    out[0].sB() = q.v2.y;
    out[0].sC() = q.v2.z;
    out[0].sD() = q.v3.x;
    out[0].sE() = q.v3.y;
    out[0].sF() = q.v3.z;      
  }


  __forceinline float convertBVH2toQBVH6(sycl::queue &gpu_queue, PLOCGlobals *globals, uint *host_device_tasks, TriQuadMesh* triQuadMesh, Scene *const scene, QBVH6 *qbvh, const BVH2Ploc *const bvh2, LeafGenerationData *leafGenData, const uint numPrimitives, const bool instanceMode, const GeometryTypeRanges &geometryTypeRanges, const bool verbose)
  {
    static const uint STOP_THRESHOLD = 1296;    
    double total_time = 0.0f;    
    uint iteration = 0;

    const bool forceFatLeaves = numPrimitives <= BVH_BRANCHING_FACTOR;
    
    host_device_tasks[0] = 0;
    host_device_tasks[1] = 0;

    /* ---- Phase I: single WG generates enough work for the breadth-first phase --- */
    {
      const uint wgSize = 1024;
      const sycl::nd_range<1> nd_range1(wgSize,sycl::range<1>(wgSize));                    
      sycl::event queue_event = gpu_queue.submit([&](sycl::handler &cgh) {
                                                   sycl::accessor< uint      ,  0, sycl_read_write, sycl_local> _node_mem_allocator_cur(cgh);                                                   
                                                   cgh.parallel_for(nd_range1,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(16)      
                                                                    {
                                                                      const uint localID     = item.get_local_id(0);
                                                                      const uint localSize   = item.get_local_range().size();                                                                      
                                                                      uint &node_mem_allocator_cur = *_node_mem_allocator_cur.get_pointer();

                                                                      const uint node_start = 2;
                                                                      const uint node_end   = 3;
                                                                      
                                                                      if (localID == 0)
                                                                      {
                                                                        /* init globals */
                                                                        globals->node_mem_allocator_start = node_start;
                                                                        globals->node_mem_allocator_cur   = node_end;
                                                                        globals->qbvh_base_pointer        = (char*)qbvh;
                                                                        TmpNodeState *root_state = (TmpNodeState*)((char*)qbvh + 64 * node_start);
                                                                        uint rootIndex = globals->rootIndex;

                                                                        if (numPrimitives <= FATLEAF_THRESHOLD)
                                                                          rootIndex = BVH2Ploc::makeFatLeaf(rootIndex,numPrimitives);
                                                                        
                                                                        root_state->init( rootIndex );
                                                                        
                                                                        node_mem_allocator_cur = node_end;
                                                                      }

                                                                      item.barrier(sycl::access::fence_space::global_and_local);

                                                                      uint startBlockID = node_start;
                                                                      uint endBlockID   = node_mem_allocator_cur;

                                                                      while(1)
                                                                      {                                                                        
                                                                        item.barrier(sycl::access::fence_space::local_space);
                               
                                                                        if (startBlockID == endBlockID || endBlockID-startBlockID>STOP_THRESHOLD) break;
                               
                                                                        for (uint innerID=startBlockID+localID;innerID<endBlockID;innerID+=localSize)
                                                                        {
                                                                          TmpNodeState *state = (TmpNodeState *)globals->nodeBlockPtr(innerID);
                                                                          const uint header = state->header;                                                                        
                                                                          const uint index  = state->bvh2_index;
                                                                          char* curAddr = (char*)state;

                                                                          if (header == 0x7fffffff)
                                                                          {
                                                                            if (!BVH2Ploc::isLeaf(index,numPrimitives))
                                                                            {
                                                                              
                                                                              if (!BVH2Ploc::isFatLeaf(index))
                                                                              {
                                                                                uint indices[BVH_BRANCHING_FACTOR];                                                                                
                                                                                const uint numChildren = openBVH2MaxAreaSortChildren(index,indices,bvh2,numPrimitives);
                                                                                uint numBlocks = 0;
                                                                                for (uint i=0;i<numChildren;i++)
                                                                                  numBlocks += geometryTypeRanges.isInstance(BVH2Ploc::getIndex(indices[i])) ? 2 : 1; 
                                                                                    
                                                                                const uint allocID = gpu::atomic_add_local(&node_mem_allocator_cur,/*numChildren*/numBlocks);
                                                                                
                                                                                char* childAddr = (char*)globals->qbvh_base_pointer + 64 * allocID;
                                                                                writeNode(curAddr,allocID-innerID,bvh2[BVH2Ploc::getIndex(index)].bounds,numChildren,indices,bvh2,numPrimitives,NODE_TYPE_MIXED,geometryTypeRanges,instanceMode);
                                                                                uint offset = 0;
                                                                                for (uint j=0;j<numChildren;j++)
                                                                                {
                                                                                  TmpNodeState *childState = (TmpNodeState *)(childAddr + offset*64);
                                                                                  childState->init(indices[j]);
                                                                                  
                                                                                  const bool isInstance = geometryTypeRanges.isInstance(BVH2Ploc::getIndex(indices[j]));
                                                                                  if (isInstance)
                                                                                    *(uint*)(childAddr + offset*64 + 64) = 0; // invalid header for second cache line in instance case
                                                                                  offset += isInstance ? 2 : 1;                                                                                  
                                                                                }
                                                                              }
                                                                            }
                                                                          }
                                                                        }

                                                                        item.barrier(sycl::access::fence_space::global_and_local);
                                                                        
                                                                        startBlockID = endBlockID;
                                                                        endBlockID = node_mem_allocator_cur;
                                                                      }
                                                                      // write out local node allocator to globals 
                                                                      if (localID == 0)
                                                                      {
                                                                        startBlockID = globals->node_mem_allocator_start;
                                                                        globals->range_start = startBlockID;
                                                                        globals->range_end   = endBlockID;                                                                        
                                                                        globals->node_mem_allocator_cur = node_mem_allocator_cur;
                                                                        host_device_tasks[0] = endBlockID - startBlockID;
                                                                        host_device_tasks[1] = endBlockID - globals->node_mem_allocator_start;                           
                                                                      }
                                                                      
                                                                    });
                                                 });
      gpu::waitOnQueueAndCatchException(gpu_queue);
      double dt = gpu::getDeviceExecutionTiming(queue_event);
      total_time += dt;
      if (unlikely(verbose))
        PRINT4("initial iteration ",iteration,(float)dt,(float)total_time);
    }

    /* ---- Phase II: full breadth-first phase until only fat leaves or single leaves remain--- */

#if 1  
    struct __aligned(64) LocalNodeData {
      uint v[16];
    };
    while(1)
    {      
      const uint blocks = host_device_tasks[0]; // = endBlockID-startBlockID;
     
      if (blocks == 0) break;
      
      iteration++;
      const uint wgSize = 256;
      const sycl::nd_range<1> nd_range1(gpu::alignTo(blocks,wgSize),sycl::range<1>(wgSize));              
      sycl::event queue_event = gpu_queue.submit([&](sycl::handler &cgh) {
                                                   sycl::accessor< LocalNodeData, 1, sycl_read_write, sycl_local> _localNodeData(sycl::range<1>(wgSize),cgh);                        
                                                   cgh.parallel_for(nd_range1,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(16)      
                                                                    {
                                                                      const uint localID   = item.get_local_id(0);                                                                      
                                                                      const uint globalID  = item.get_global_id(0);
                                                                      const uint numGroups   = item.get_group_range(0);
                             
                                                                      uint startBlockID = globals->range_start; 
                                                                      uint endBlockID   = globals->range_end; 
                             
                                                                      const uint innerID   = startBlockID + globalID;
                                                                      char* curAddr = nullptr;
                                                                      bool valid = false;

                                                                      LocalNodeData *localNodeData = _localNodeData.get_pointer();
                                                                      
                                                                      if (innerID < endBlockID)
                                                                      {
                                                                        TmpNodeState *state = (TmpNodeState *)globals->nodeBlockPtr(innerID);
                                                                        const uint header = state->header;                                                                        
                                                                        const uint index  = state->bvh2_index;
                                                                        curAddr = (char*)state;
                                                                        if (header == 0x7fffffff)
                                                                        {
                                                                          if (!BVH2Ploc::isLeaf(index,numPrimitives))
                                                                          {                                                                            
                                                                            if (!BVH2Ploc::isFatLeaf(index))
                                                                            {                                                                              
                                                                              uint indices[BVH_BRANCHING_FACTOR];
                                                                              const uint numChildren = openBVH2MaxAreaSortChildren(index,indices,bvh2,numPrimitives);
                                                                              uint numBlocks = 0;
                                                                              for (uint i=0;i<numChildren;i++)
                                                                                numBlocks += geometryTypeRanges.isInstance(BVH2Ploc::getIndex(indices[i])) ? 2 : 1; 
                                                                              
                                                                              char *const childAddr = globals->sub_group_shared_varying_atomic_allocNode(sizeof(QBVH6::InternalNode6)*numBlocks /*numChildren*/); //FIXME: subgroup
                                                                              valid = true;
                                                                              writeNode(localNodeData[localID].v,(childAddr-curAddr)/64,bvh2[BVH2Ploc::getIndex(index)].bounds,numChildren,indices,bvh2,numPrimitives,NODE_TYPE_MIXED,geometryTypeRanges,instanceMode);
                                                                              uint offset = 0;
                                                                              for (uint j=0;j<numChildren;j++)
                                                                              {
                                                                                //TmpNodeState *childState = (TmpNodeState *)(childAddr + 64 * j);
                                                                                TmpNodeState *childState = (TmpNodeState *)(childAddr + offset*64);
                                                                                childState->init(indices[j]);
                                                                                const bool isInstance = geometryTypeRanges.isInstance(BVH2Ploc::getIndex(indices[j]));
                                                                                if (isInstance)
                                                                                  *(uint*)(childAddr + offset*64 + 64) = 0; // invalid header for second cache line in instance case
                                                                                offset += isInstance ? 2 : 1;                                                                                 
                                                                              }
                                                                            }
                                                                          }
                                                                        }
                                                                      }

                                                                      item.barrier(sycl::access::fence_space::local_space);

                                                                      
                                                                      const uint subgroupLocalID = get_sub_group_local_id();
                                                                      uint mask = sub_group_ballot(valid);
                                                                      while(mask)
                                                                      {
                                                                        const uint index = sycl::ctz(mask);
                                                                        mask &= mask-1;
                                                                        uint ID = sub_group_broadcast(localID,index);
                                                                        uint* dest = sub_group_broadcast((uint*)curAddr,index);
                                                                        const uint v = localNodeData[ID].v[subgroupLocalID];                                                                        
                                                                        sub_group_store(dest,v);
                                                                      }
                                                                      
                                                                      /* -------------------------------- */                                                       
                                                                      /* --- last WG does the cleanup --- */
                                                                      /* -------------------------------- */

                                                                      if (localID == 0)
                                                                      {
                                                                        const uint syncID = gpu::atomic_add_global(&globals->sync,(uint)1);
                                                                        if (syncID == numGroups-1)
                                                                        {
                                                                          /* --- reset atomics --- */
                                                                          globals->sync = 0;
                                                                          const uint new_startBlockID = globals->range_end;
                                                                          const uint new_endBlockID   = globals->node_mem_allocator_cur;
                                                                          globals->range_start = new_startBlockID;
                                                                          globals->range_end   = new_endBlockID;
                                                                          host_device_tasks[0] = new_endBlockID - new_startBlockID;
                                                                          host_device_tasks[1] = new_endBlockID - globals->node_mem_allocator_start;
                                                                        }
                                                                      }
                             
                                                                    });
		  
                                                 });
      gpu::waitOnQueueAndCatchException(gpu_queue);
      double dt = gpu::getDeviceExecutionTiming(queue_event);
      total_time += dt;
      if (unlikely(verbose))      
        PRINT5("flattening iteration ",iteration,blocks,(float)dt,(float)total_time);
    }

#endif    
    
    /* ---- Phase III: fill in mixed leafs and generate inner node for fatleaves plus storing primID, geomID pairs for final phase --- */
    const uint blocks = host_device_tasks[1];
    
    if (blocks)
    {
      /* PRINT(blocks); */
      /* PRINT(globals->node_mem_allocator_start); */
      /* PRINT(globals->node_mem_allocator_cur);       */
      /* PRINT(globals->leaf_mem_allocator_start); */
      /* PRINT(globals->leaf_mem_allocator_cur); */
      
      const uint wgSize = 256;
      const sycl::nd_range<1> nd_range1(gpu::alignTo(blocks,wgSize),sycl::range<1>(wgSize));              
      sycl::event queue_event = gpu_queue.submit([&](sycl::handler &cgh) {
                                                   sycl::accessor< uint  ,  0, sycl_read_write, sycl_local> _local_numBlocks(cgh);
                                                   sycl::accessor< uint  ,  0, sycl_read_write, sycl_local> _local_numLeaves(cgh);
                                                   
                                                   sycl::accessor< uint  ,  0, sycl_read_write, sycl_local> _global_blockID(cgh);
                                                   sycl::accessor< uint  ,  0, sycl_read_write, sycl_local> _global_numLeafID(cgh);
                                                   
                                                   sycl::accessor< LeafGenerationData, 1, sycl_read_write, sycl_local> _local_leafGenData(sycl::range<1>(wgSize*BVH_BRANCHING_FACTOR),cgh);

                                                   sycl::accessor< uint, 1, sycl_read_write, sycl_local> _local_indices(sycl::range<1>(wgSize*BVH_BRANCHING_FACTOR),cgh);                                                                           
                                                   cgh.parallel_for(nd_range1,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(16)      
                                                                    {
                                                                      const uint localID   = item.get_local_id(0);                             
                                                                      const uint globalID  = item.get_global_id(0);
                                                                      const uint numGroups   = item.get_group_range(0);
                                                                      const uint localSize   = item.get_local_range().size();                                                                      
                                                                      

                                                                      const uint startBlockID = globals->node_mem_allocator_start;
                                                                      const uint endBlockID   = globals->node_mem_allocator_cur;                             
                                                                      const uint innerID      = startBlockID + globalID;

                                                                      uint &local_numBlocks   = *_local_numBlocks.get_pointer();
                                                                      uint &local_numLeaves   = *_local_numLeaves.get_pointer();                                                                      
                                                                      uint &global_blockID    = *_global_blockID.get_pointer();
                                                                      uint &global_leafID     = *_global_numLeafID.get_pointer();
                                                                      
                                                                      LeafGenerationData* local_leafGenData = _local_leafGenData.get_pointer();

                                                                      uint *indices = _local_indices.get_pointer() + BVH_BRANCHING_FACTOR * localID;
                                                                      
                                                                      if (localID == 0)
                                                                      {
                                                                        local_numBlocks = 0;
                                                                        local_numLeaves = 0;
                                                                      }
                                                                      
                                                                      item.barrier(sycl::access::fence_space::local_space);
                                                                      
                                                                      char* curAddr      = nullptr;
                                                                      uint numChildren   = 0;
                                                                      bool isFatLeaf     = false;
                                                                      uint local_blockID = 0;
                                                                      uint local_leafID  = 0;
                                                                      uint index         = 0;
                                                                      
                                                                      if (innerID < endBlockID)                                                                        
                                                                      {
                                                                        TmpNodeState *state = (TmpNodeState *)globals->nodeBlockPtr(innerID);
                                                                        index   = state->bvh2_index;

                                                                        
                                                                        curAddr = (char*)state;
                                                                        if (state->header == 0x7fffffff) // not processed yet
                                                                        {
                                                                          isFatLeaf = !BVH2Ploc::isLeaf(index,numPrimitives) || forceFatLeaves;
                                                                          
                                                                          uint numBlocks = 0;
                                                                          if (isFatLeaf) // fatleaf, generate internal node and numChildren x LeafGenData 
                                                                          {
                                                                            numChildren = 0;
                                                                            getLeafIndices(index,bvh2,indices,numChildren,numPrimitives);
                                                                            for (uint i=0;i<numChildren;i++)
                                                                              numBlocks += geometryTypeRanges.isInstance(BVH2Ploc::getIndex(indices[i])) ? 2 : 1;
                                                                          }
                                                                          else
                                                                          {
                                                                            numChildren = 1;
                                                                            numBlocks = 0; //geometryTypeRanges.isInstance(BVH2Ploc::getIndex(index)) ? 2 : 1; 
                                                                            indices[0] = index;
                                                                          }
                                                                          local_blockID = gpu::atomic_add_local(&local_numBlocks,numBlocks);
                                                                          local_leafID  = gpu::atomic_add_local(&local_numLeaves,numChildren);
                                                                        }
                                                                      }

                                                                      
                                                                      item.barrier(sycl::access::fence_space::local_space);
                                                                      
                                                                      const uint numBlocks = local_numBlocks;
                                                                      const uint numLeaves = local_numLeaves;                                                                                                                                              
                                                                      if (localID == 0)
                                                                      {
                                                                        global_blockID = gpu::atomic_add_global(&globals->leaf_mem_allocator_cur,numBlocks);
                                                                        global_leafID  = gpu::atomic_add_global(&globals->numLeaves,numLeaves);
                                                                      }
                                                                      
                                                                      item.barrier(sycl::access::fence_space::local_space);

                                                                      const uint blockID = global_blockID + local_blockID;
                                                                      const uint leafID  = global_leafID; //  + local_leafID;
                                                                      
                                                                      
                                                                      if (isFatLeaf)
                                                                        writeNode(curAddr,blockID-innerID,bvh2[BVH2Ploc::getIndex(index)].bounds,numChildren,indices,bvh2,numPrimitives,NODE_TYPE_MIXED,geometryTypeRanges);
                                                                                                                                            
                                                                      /* --- write to SLM frist --- */
                                                                      
                                                                      const uint local_leafDataID = local_leafID;
                                                                      uint node_blockID = 0;
                                                                      for (uint j=0;j<numChildren;j++)
                                                                      {
                                                                        const uint index_j = BVH2Ploc::getIndex(indices[j]);
                                                                        const uint geomID = bvh2[index_j].left;
                                                                        const uint primID = bvh2[index_j].right;
                                                                        const uint bID = isFatLeaf ? (blockID + node_blockID) : innerID;
                                                                        const bool isInstance   = geometryTypeRanges.isInstance(index_j);
                                                                        const bool isProcedural = geometryTypeRanges.isProcedural(index_j);
                                                                        uint primID_type = (primID & LEAF_TYPE_MASK_LOW) | LEAF_TYPE_QUAD;
                                                                        primID_type |= isProcedural ? LEAF_TYPE_PROCEDURAL : 0;
                                                                        primID_type |= isInstance   ? LEAF_TYPE_INSTANCE   : 0;
                                                                        local_leafGenData[local_leafDataID+j].blockID = bID;
                                                                        local_leafGenData[local_leafDataID+j].primID = primID_type;
                                                                        local_leafGenData[local_leafDataID+j].geomID = geomID;
                                                                        node_blockID += isInstance  ? 2 : 1;
                                                                      }
                                                                                                                                           
                                                                      item.barrier(sycl::access::fence_space::local_space);

                                                                      /* --- write out all local entries to global memory --- */
                                                                      
                                                                       for (uint i=localID;i<numLeaves;i+=localSize) 
                                                                         leafGenData[leafID+i] = local_leafGenData[i]; 
                                                                      
                                                                      if (localID == 0)
                                                                      {                                                                        
                                                                        const uint syncID = gpu::atomic_add_global(&globals->sync,(uint)1);
                                                                        if (syncID == numGroups-1)
                                                                        {
                                                                          /* --- reset atomics --- */
                                                                          globals->sync = 0;
                                                                          host_device_tasks[0] = globals->numLeaves;
                                                                        }
                                                                      }
                             
                                                                    });
		  
                                                 });
      gpu::waitOnQueueAndCatchException(gpu_queue);
      double dt = gpu::getDeviceExecutionTiming(queue_event);      
      total_time += dt;
      if (unlikely(verbose))      
        PRINT3("final flattening iteration ",(float)dt,(float)total_time);
    }    
    
    /* ---- Phase IV: for each primID, geomID pair generate corresponding leaf data --- */
    const uint leaves = host_device_tasks[0]; // = globals->leaf_mem_allocator_cur - globals->leaf_mem_allocator_start;
    
    if (leaves)
    {
      //for (uint i=0;i<leaves;i++) 
      // PRINT5(i,leafGenData[i].blockID,leafGenData[i].primID,leafGenData[i].primID & BVH2Ploc::BIT_MASK,leafGenData[i].geomID & 0x00ffffff); 
      
      {
        const uint wgSize = 256;
        const sycl::nd_range<1> nd_range1(gpu::alignTo(leaves,wgSize),sycl::range<1>(wgSize));              
        sycl::event queue_event = gpu_queue.submit([&](sycl::handler &cgh) {
            sycl::accessor< QuadLeaf, 1, sycl_read_write, sycl_local> _localLeaf(sycl::range<1>(wgSize),cgh);
            cgh.parallel_for(nd_range1,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(16)      
                             {
                               const uint globalID = item.get_global_id(0);
                               const uint localID  = item.get_local_id(0);
                               QuadLeaf *localLeaf = _localLeaf.get_pointer();
                               bool valid = false;
                               QuadLeaf *qleaf = nullptr;
                                                                      
                               if (globalID < leaves)                                                                        
                               {
                                 qleaf = (QuadLeaf *)globals->nodeBlockPtr(leafGenData[globalID].blockID);
                                 const uint geomID = leafGenData[globalID].geomID & 0x00ffffff;
                                 const uint primID0 = leafGenData[globalID].primID & LEAF_TYPE_MASK_LOW;
                                 const uint primID1 = primID0 + ((leafGenData[globalID].geomID & 0x7fffffff) >> 24);
                                 
                                 if ((leafGenData[globalID].primID & LEAF_TYPE_MASK_HIGH) == LEAF_TYPE_QUAD)
                                 {
                                   TriQuadMesh &mesh = triQuadMesh[geomID];

                                   valid = true;
                                   // ====================                           
                                   // === TriangleMesh ===
                                   // ====================                                                                        
                                   if (mesh.isTriangleMesh())
                                   {
                                     const TriangleMesh::Triangle tri = mesh.getTrianglesPtr()[primID0];
                                     const Vec3f p0 = mesh.getVerticesPtr()[tri.v[0]];
                                     const Vec3f p1 = mesh.getVerticesPtr()[tri.v[1]];
                                     const Vec3f p2 = mesh.getVerticesPtr()[tri.v[2]];
                                     Vec3f p3 = p2;
                                     uint lb0 = 0,lb1 = 0, lb2 = 0;
                
                                     /* handle paired triangle */
                                     if (primID0 != primID1)
                                     {
                                       const TriangleMesh::Triangle tri1 = mesh.getTrianglesPtr()[primID1];
          
                                       const uint p3_index = try_pair_triangles(uint3(tri.v[0],tri.v[1],tri.v[2]),uint3(tri1.v[0],tri1.v[1],tri1.v[2]),lb0,lb1,lb2);
                                       p3 = mesh.getVerticesPtr()[tri1.v[p3_index]];                                   
                                     }

                                     localLeaf[localID] = QuadLeaf( p0,p1,p2,p3, lb0,lb1,lb2, 0, geomID, primID0, primID1, GeometryFlags::OPAQUE, 0xFF, /*i == (numChildren-1)*/ true );
                                     //write(leaf,(float16*)qleaf);
                                   }
                                   // ================                           
                                   // === QuadMesh ===
                                   // ================
                                   else
                                   {
                                     const QuadMesh::Quad quad = mesh.getQuadsPtr()[primID0];
                                     const Vec3f p0 = mesh.getVerticesPtr()[quad.v[0]];
                                     const Vec3f p1 = mesh.getVerticesPtr()[quad.v[1]];
                                     const Vec3f p2 = mesh.getVerticesPtr()[quad.v[2]];
                                     const Vec3f p3 = mesh.getVerticesPtr()[quad.v[3]];
                                     localLeaf[localID] = QuadLeaf( p0,p1,p3,p2, 3,2,1, 0, geomID, primID0, primID0, GeometryFlags::OPAQUE, 0xFF, /*i == (numChildren-1)*/ true );                                                                          
                                   }
                                 }
                                 else if ((leafGenData[globalID].primID & LEAF_TYPE_MASK_HIGH) == LEAF_TYPE_INSTANCE)
                                 {
                                   InstancePrimitive *dest = (InstancePrimitive *)qleaf;         
                                   const uint instID = primID0;
                                   Instance* instance = scene->get<Instance>(instID);
                                   void* accel = static_cast<Scene*>(instance->object)->hwaccel.data();
                                   const AffineSpace3fa local2world = instance->getLocal2World();
                                   const uint64_t root = (uint64_t)accel + 128; //static_cast<QBVH6*>(accel)->root();
                                   *dest = InstancePrimitive(local2world,root,instID,mask32_to_mask8(instance->mask));
                                 }
                                 else if ((leafGenData[globalID].primID & LEAF_TYPE_MASK_HIGH) == LEAF_TYPE_PROCEDURAL) // FIXME use valid
                                 {
                                   const uint userGeomID = geomID;
                                   const uint mask32 = mask32_to_mask8(scene->get(userGeomID)->mask);
                                   ProceduralLeaf *dest = (ProceduralLeaf *)qleaf;
                                   PrimLeafDesc leafDesc(0,geomID,GeometryFlags::NONE,mask32,PrimLeafDesc::TYPE_OPACITY_CULLING_ENABLED);
                                   *dest = ProceduralLeaf(leafDesc,primID0,true);
                                 }
                               }

                               /* ================================== */                                                                      
                               /* === write out to global memory === */
                               /* ================================== */
                                                                      
                               const uint subgroupLocalID = get_sub_group_local_id();
                               uint mask = sub_group_ballot(valid);
                               while(mask)
                               {
                                 const uint index = sycl::ctz(mask);
                                 mask &= mask-1;
                                 uint ID = sub_group_broadcast(localID,index);
                                 uint* dest = sub_group_broadcast((uint*)qleaf,index);
                                 uint* source = (uint*)&localLeaf[ID];
                                 const uint v = source[subgroupLocalID];                                                                        
                                 sub_group_store(dest,v);
                               }                                                                                                            
                             });
		  
          });
        gpu::waitOnQueueAndCatchException(gpu_queue);
        double dt = gpu::getDeviceExecutionTiming(queue_event);      
        total_time += dt;
        if (unlikely(verbose))      
          PRINT3("final leaf generation ",(float)dt,(float)total_time);
      }
    }    
    
    return (float)total_time;
  }


  __forceinline float convertBVH2toQBVH6_2(sycl::queue &gpu_queue, PLOCGlobals *globals, uint *host_device_tasks, TriQuadMesh* triQuadMesh, Scene *const scene, QBVH6 *qbvh, const BVH2Ploc *const bvh2, LeafGenerationData *leafGenData, const uint numPrimitives, const bool instanceMode, const GeometryTypeRanges &geometryTypeRanges, const bool verbose)
  {
    static const uint STOP_THRESHOLD = 1296;    
    double total_time = 0.0f;    
    uint iteration = 0;

    const bool forceFatLeaves = instanceMode || numPrimitives <= BVH_BRANCHING_FACTOR;
    
    host_device_tasks[0] = 0;
    host_device_tasks[1] = 0;
    
    /* ---- Phase I: single WG generates enough work for the breadth-first phase --- */
    {
      const uint wgSize = 1024;
      const sycl::nd_range<1> nd_range1(wgSize,sycl::range<1>(wgSize));                    
      sycl::event queue_event = gpu_queue.submit([&](sycl::handler &cgh) {
                                                   sycl::accessor< uint        ,  0, sycl_read_write, sycl_local> _node_mem_allocator_cur(cgh);
                                                   sycl::accessor< uint        ,  0, sycl_read_write, sycl_local> _numBuildRecords(cgh);
                                                   sycl::accessor< uint , 1, sycl_read_write, sycl_local> _local_blockID(sycl::range<1>(8000),cgh);
                                                   sycl::accessor< uint , 1, sycl_read_write, sycl_local> _local_bvh2_index(sycl::range<1>(8000),cgh);
                                                  
                                                   cgh.parallel_for(nd_range1,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(16)      
                                                                    {
                                                                      const uint localID     = item.get_local_id(0);
                                                                      const uint localSize   = item.get_local_range().size();                                                                      
                                                                      uint &node_mem_allocator_cur = *_node_mem_allocator_cur.get_pointer();
                                                                      uint &numBuildRecords        = *_numBuildRecords.get_pointer();
                                                                      uint *local_blockID    = _local_blockID.get_pointer();
                                                                      uint *local_bvh2_index = _local_bvh2_index.get_pointer();
                                                                      
                                                                      const uint node_start = 2;
                                                                      const uint node_end   = 3;
                                                                      
                                                                      if (localID == 0)
                                                                      {
                                                                        /* init globals */
                                                                        globals->node_mem_allocator_start = node_start;
                                                                        globals->node_mem_allocator_cur   = node_end;
                                                                        globals->qbvh_base_pointer        = (char*)qbvh;
                                                                        //TmpNodeState *root_state = (TmpNodeState*)((char*)qbvh + 64 * node_start);
                                                                        uint rootIndex = globals->rootIndex;

                                                                        if (numPrimitives <= FATLEAF_THRESHOLD)
                                                                          rootIndex = BVH2Ploc::makeFatLeaf(rootIndex,numPrimitives);

                                                                        numBuildRecords = 1;
                                                                        //root_state->init( rootIndex );

                                                                        local_blockID[0] = node_start;
                                                                        local_bvh2_index[0] = rootIndex;
                                                                        
                                                                        node_mem_allocator_cur = node_end;
                                                                      }

                                                                      item.barrier(sycl::access::fence_space::global_and_local);

                                                                      uint current_numBuildRecords = -1;                                                                      
                                                                      while(current_numBuildRecords != numBuildRecords)
                                                                      {
                                                                        current_numBuildRecords = numBuildRecords;
                                                                        item.barrier(sycl::access::fence_space::local_space);
                               
                                                                        if (current_numBuildRecords>STOP_THRESHOLD) break; // FIXME: old_numRecords;
                               
                                                                        for (uint innerID=localID;innerID<current_numBuildRecords;innerID+=localSize)
                                                                        {
                                                                          const uint index   = local_bvh2_index[innerID];
                                                                          const uint blockID = local_blockID[innerID];
                                                                          
                                                                          char* curAddr = (char*)globals->qbvh_base_pointer + 64 * blockID;
                                                                          if (!BVH2Ploc::isLeaf(index,numPrimitives))
                                                                          {
                                                                            if (!BVH2Ploc::isFatLeaf(index))
                                                                            {
                                                                              uint indices[BVH_BRANCHING_FACTOR];                                                                                
                                                                              const uint numChildren = openBVH2MaxAreaSortChildren(index,indices,bvh2,numPrimitives);
                                                                              const uint allocID  = gpu::atomic_add_local(&node_mem_allocator_cur,numChildren);
                                                                              const uint recordID = gpu::atomic_add_local(&numBuildRecords,numChildren-1);
                                                                              
                                                                              writeNode(curAddr,allocID-blockID,bvh2[BVH2Ploc::getIndex(index)].bounds,numChildren,indices,bvh2,numPrimitives,NODE_TYPE_MIXED,geometryTypeRanges,instanceMode);
                                                                              local_blockID[innerID] = allocID;
                                                                              local_bvh2_index[innerID] = indices[0];
                                                                              
                                                                              for (uint j=1;j<numChildren;j++)
                                                                              {
                                                                                local_blockID[recordID+j-1] = allocID + j;
                                                                                local_bvh2_index[recordID+j-1] = indices[j];
                                                                              }
                                                                            }
                                                                          }
                                                                        }

                                                                        item.barrier(sycl::access::fence_space::global_and_local);
                                                                      }

                                                                      for (uint ID=localID;ID<numBuildRecords;ID+=localSize)
                                                                        leafGenData[ID] = LeafGenerationData(local_blockID[ID],local_bvh2_index[ID],0);
                                                                      
                                                                      // write out local node allocator to globals 
                                                                      if (localID == 0)
                                                                      {
                                                                        globals->node_mem_allocator_cur = node_mem_allocator_cur;
                                                                        globals->numBuildRecords = numBuildRecords;
                                                                        host_device_tasks[0] = numBuildRecords;
                                                                      }
                                                                      
                                                                    });
                                                 });
      gpu::waitOnQueueAndCatchException(gpu_queue);
      double dt = gpu::getDeviceExecutionTiming(queue_event);
      total_time += dt;
      if (unlikely(verbose))
        PRINT4("initial iteration ",iteration,(float)dt,(float)total_time);
    }
        
    /* ---- Phase II: full breadth-first phase until only fat leaves or single leaves remain--- */
    
    struct __aligned(64) LocalNodeData {
      uint v[16];
    };

    uint old_numBlocks = -1;
    while(1)
    {      
      const uint blocks = host_device_tasks[0]; // = endBlockID-startBlockID;
      //PRINT3(iteration,blocks,old_numBlocks);
      
      if (blocks == old_numBlocks) break;
      old_numBlocks = blocks;

      /* for (uint i=0;i<blocks;i++)   */
      /*   PRINT4(i,leafGenData[i].blockID,leafGenData[i].primID,leafGenData[i].primID & BVH2Ploc::BIT_MASK); */
      
      
      iteration++;
      const uint wgSize = 256;
      const sycl::nd_range<1> nd_range1(gpu::alignTo(blocks,wgSize),sycl::range<1>(wgSize));              
      sycl::event queue_event = gpu_queue.submit([&](sycl::handler &cgh) {
                                                   sycl::accessor< LocalNodeData, 1, sycl_read_write, sycl_local> _localNodeData(sycl::range<1>(wgSize),cgh);                        
                                                   cgh.parallel_for(nd_range1,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(16)      
                                                                    {
                                                                      const uint localID   = item.get_local_id(0);                                                                      
                                                                      const uint globalID  = item.get_global_id(0);
                                                                      const uint numGroups   = item.get_group_range(0);                                                          
                                                                      const uint innerID   = globalID;
                                                                      char* curAddr = nullptr;
                                                                      bool valid = false;

                                                                      LocalNodeData *localNodeData = _localNodeData.get_pointer();
                                                                      
                                                                      if (innerID < blocks)
                                                                      {
                                                                        const uint index = leafGenData[innerID].bvh2_index;
                                                                        const int blockID  = leafGenData[innerID].blockID;
                                                                        curAddr = (char*)globals->qbvh_base_pointer + 64 * blockID;

                                                                        if (!BVH2Ploc::isLeaf(index,numPrimitives))
                                                                        {                                                                            
                                                                          if (!BVH2Ploc::isFatLeaf(index))
                                                                          {                                                                              
                                                                            uint indices[BVH_BRANCHING_FACTOR];
                                                                            const uint numChildren = openBVH2MaxAreaSortChildren(index,indices,bvh2,numPrimitives);                                                                              
                                                                            const uint allocID = globals->sub_group_shared_varying_atomic_allocNodeID(sizeof(QBVH6::InternalNode6)*numChildren); //FIXME: subgroup
                                                                            const uint recordID = gpu::atomic_add_global(&globals->numBuildRecords,numChildren-1);
                                                                            
                                                                            valid = true;
                                                                            writeNode(localNodeData[localID].v,allocID-blockID,bvh2[BVH2Ploc::getIndex(index)].bounds,numChildren,indices,bvh2,numPrimitives,NODE_TYPE_MIXED,geometryTypeRanges,instanceMode);
                                                                            
                                                                            leafGenData[innerID] = LeafGenerationData(allocID,indices[0],0);
                                                                            
                                                                            for (uint j=1;j<numChildren;j++)
                                                                            leafGenData[recordID+j-1] = LeafGenerationData(allocID + j,indices[j],0);
                                                                          }
                                                                        }
                                                                      }

                                                                      item.barrier(sycl::access::fence_space::local_space);

                                                                      
                                                                      const uint subgroupLocalID = get_sub_group_local_id();
                                                                      uint mask = sub_group_ballot(valid);
                                                                      while(mask)
                                                                      {
                                                                        const uint index = sycl::ctz(mask);
                                                                        mask &= mask-1;
                                                                        uint ID = sub_group_broadcast(localID,index);
                                                                        uint* dest = sub_group_broadcast((uint*)curAddr,index);
                                                                        const uint v = localNodeData[ID].v[subgroupLocalID];                                                                        
                                                                        sub_group_store(dest,v);
                                                                      }
                                                                      
                                                                      /* -------------------------------- */                                                       
                                                                      /* --- last WG does the cleanup --- */
                                                                      /* -------------------------------- */

                                                                      if (localID == 0)
                                                                      {
                                                                        const uint syncID = gpu::atomic_add_global(&globals->sync,(uint)1);
                                                                        if (syncID == numGroups-1)
                                                                        {
                                                                          /* --- reset atomics --- */
                                                                          globals->sync = 0;
                                                                          host_device_tasks[0] = globals->numBuildRecords;
                                                                        }
                                                                      }
                             
                                                                    });
		  
                                                 });
      gpu::waitOnQueueAndCatchException(gpu_queue);
      double dt = gpu::getDeviceExecutionTiming(queue_event);
      total_time += dt;
      if (unlikely(verbose))      
        PRINT5("flattening iteration ",iteration,blocks,(float)dt,(float)total_time);
    }
    
    /* ---- Phase III: fill in mixed leafs and generate inner node for fatleaves plus storing primID, geomID pairs for final phase --- */
    const uint blocks = host_device_tasks[0];
    PRINT(blocks);
    
    if (blocks)
    {

      const uint wgSize = 256;
      const sycl::nd_range<1> nd_range1(gpu::alignTo(blocks,wgSize),sycl::range<1>(wgSize));              
      sycl::event queue_event = gpu_queue.submit([&](sycl::handler &cgh) {
                                                   sycl::accessor< uint  ,  0, sycl_read_write, sycl_local> _local_numBlocks(cgh);
                                                   sycl::accessor< uint  ,  0, sycl_read_write, sycl_local> _local_numLeaves(cgh);
                                                   
                                                   sycl::accessor< uint  ,  0, sycl_read_write, sycl_local> _global_blockID(cgh);
                                                   sycl::accessor< uint  ,  0, sycl_read_write, sycl_local> _global_numLeafID(cgh);
                                                   
                                                   //sycl::accessor< LeafGenerationData, 1, sycl_read_write, sycl_local> _local_leafGenData(sycl::range<1>(wgSize*BVH_BRANCHING_FACTOR),cgh);

                                                   sycl::accessor< uint, 1, sycl_read_write, sycl_local> _local_indices(sycl::range<1>(wgSize*BVH_BRANCHING_FACTOR),cgh);                                                                                             sycl::accessor< uint, 1, sycl_read_write, sycl_local> _local_blockIDs(sycl::range<1>(wgSize*BVH_BRANCHING_FACTOR),cgh);                                                                           
                                                   cgh.parallel_for(nd_range1,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(16)      
                                                                    {
                                                                      const uint localID   = item.get_local_id(0);                             
                                                                      const uint globalID  = item.get_global_id(0);
                                                                      const uint groupID   = item.get_group(0);
                                                                      const uint numGroups   = item.get_group_range(0);
                                                                      const uint localSize   = item.get_local_range().size();                                                                                                                                            
                                                                      const uint group_startID = groupID*wgSize;
                                                                      const uint innerID = globalID;
                                                                        
                                                                      uint &local_numBlocks   = *_local_numBlocks.get_pointer();
                                                                      uint &local_numLeaves   = *_local_numLeaves.get_pointer();                                                                      
                                                                      uint &global_blockID    = *_global_blockID.get_pointer();
                                                                      uint &global_leafID     = *_global_numLeafID.get_pointer();
                                                                      
                                                                      //LeafGenerationData* local_leafGenData = _local_leafGenData.get_pointer();

                                                                      //uint *indices = _local_indices.get_pointer() + BVH_BRANCHING_FACTOR * localID;

                                                                      uint *local_indices  = _local_indices.get_pointer();
                                                                      uint *local_blockIDs = _local_blockIDs.get_pointer();
                                                                      
                                                                      
                                                                      if (localID == 0)
                                                                      {
                                                                        local_numBlocks = 0;
                                                                        local_numLeaves = 0;
                                                                      }
                                                                      
                                                                      item.barrier(sycl::access::fence_space::local_space);
                                                                      
                                                                      char* curAddr      = nullptr;
                                                                      uint numChildren   = 0;
                                                                      bool isFatLeaf     = false;
                                                                      uint local_blockID = 0;
                                                                      uint local_leafID  = 0;
                                                                      uint index         = 0;
                                                                      uint blockID       = 0;
                                                                      uint indices[BVH_BRANCHING_FACTOR];
                                                                      
                                                                      if (innerID < blocks)                                                                        
                                                                      {
                                                                        index = leafGenData[innerID].bvh2_index;
                                                                        blockID  = leafGenData[innerID].blockID;
                                                                        curAddr = (char*)globals->qbvh_base_pointer + 64 * blockID;

                                                                        // =============================================                                                                          
                                                                        // === forcing fatleaf mode in instance mode ===
                                                                        // =============================================
                                                                          
                                                                        isFatLeaf = !BVH2Ploc::isLeaf(index,numPrimitives) || forceFatLeaves;
                                                                        uint numBlocks = 0;
                                                                        if (isFatLeaf) // fatleaf, generate internal node and numChildren x LeafGenData 
                                                                        {
                                                                          numChildren = 0;
                                                                          getLeafIndices(index,bvh2,indices,numChildren,numPrimitives);
                                                                          for (uint i=0;i<numChildren;i++)
                                                                            numBlocks += geometryTypeRanges.isInstance(BVH2Ploc::getIndex(indices[i])) ? 2 : 1;
                                                                        }
                                                                        else
                                                                        {
                                                                          numChildren = 1;
                                                                          numBlocks = geometryTypeRanges.isInstance(BVH2Ploc::getIndex(index)) ? 2 : 1; 
                                                                          indices[0] = index;
                                                                        }
                                                                        local_blockID = gpu::atomic_add_local(&local_numBlocks,numBlocks);
                                                                        local_leafID  = gpu::atomic_add_local(&local_numLeaves,numChildren);  // full numChildren
                                                                      }

                                                                      item.barrier(sycl::access::fence_space::local_space);
                                                                      
                                                                      const uint already_processed_leaves = sycl::min(blocks-group_startID,localSize);
                                                                      
                                                                      if (localID == 0)
                                                                      {
                                                                        const uint numBlocks = local_numBlocks;
                                                                        const uint allocate_numLeaves = local_numLeaves-already_processed_leaves;
                                                                        
                                                                        global_blockID = gpu::atomic_add_global(&globals->leaf_mem_allocator_cur,numBlocks);
                                                                        global_leafID  = gpu::atomic_add_global(&globals->numBuildRecords,allocate_numLeaves);
                                                                      }
                                                                      
                                                                      item.barrier(sycl::access::fence_space::local_space);

                                                                      const uint leaf_blockID = global_blockID + local_blockID;
                                                                      //const uint leafID  = global_leafID; // + local_leafID;

                                                                      uint node_blockID = 0;                                                                        
                                                                      for (uint j=0;j<numChildren;j++)
                                                                      {
                                                                        const uint index_j = BVH2Ploc::getIndex(indices[j]);
                                                                        const uint bID = isFatLeaf ? (leaf_blockID + node_blockID) : blockID;
                                                                        local_indices[local_leafID+j] = indices[j];
                                                                        local_blockIDs[local_blockID+j] = bID;
                                                                        const bool isInstance   = geometryTypeRanges.isInstance(index_j);
                                                                        node_blockID += isInstance  ? 2 : 1;
                                                                      }
                                                                                                                                            
                                                                      
                                                                      if (isFatLeaf)
                                                                        writeNode(curAddr,leaf_blockID-blockID,bvh2[BVH2Ploc::getIndex(index)].bounds,numChildren,indices,bvh2,numPrimitives,NODE_TYPE_MIXED,geometryTypeRanges);

                                                                      item.barrier(sycl::access::fence_space::local_space);
                                                                      
                                                                      for (uint j=localID;j<local_numLeaves;j+=localSize)
                                                                      {
                                                                        const uint index_j = BVH2Ploc::getIndex(local_indices[j]);
                                                                        const uint geomID = bvh2[index_j].left;
                                                                        const uint primID = bvh2[index_j].right;
                                                                        const uint bID = local_blockIDs[j];
                                                                        const bool isInstance   = geometryTypeRanges.isInstance(index_j);
                                                                        const bool isProcedural = geometryTypeRanges.isProcedural(index_j);
                                                                        uint primID_type = (primID & LEAF_TYPE_MASK_LOW) | LEAF_TYPE_QUAD;
                                                                        primID_type |= isProcedural ? LEAF_TYPE_PROCEDURAL : 0;
                                                                        primID_type |= isInstance   ? LEAF_TYPE_INSTANCE   : 0;
                                                                        const uint destID = j < already_processed_leaves ? group_startID+localID : global_leafID + j - already_processed_leaves;                                                                                           leafGenData[destID].blockID = bID; 
                                                                        leafGenData[destID].primID = primID_type; 
                                                                        leafGenData[destID].geomID = geomID; 
                                                                      }
#if 0                                                                      

                                                                      /* --- write to SLM frist --- */

                                                                      //const uint local_leafDataID = local_leafID;
                                                                      uint node_blockID = 0;
                                                                      for (uint j=0;j<numChildren;j++)
                                                                      {
                                                                        const uint index_j = BVH2Ploc::getIndex(indices[j]);
                                                                        const uint geomID = bvh2[index_j].left;
                                                                        const uint primID = bvh2[index_j].right;
                                                                        const uint bID = isFatLeaf ? (leaf_blockID + node_blockID) : blockID;
                                                                        const bool isInstance   = geometryTypeRanges.isInstance(index_j);
                                                                        const bool isProcedural = geometryTypeRanges.isProcedural(index_j);
                                                                        uint primID_type = (primID & LEAF_TYPE_MASK_LOW) | LEAF_TYPE_QUAD;
                                                                        primID_type |= isProcedural ? LEAF_TYPE_PROCEDURAL : 0;
                                                                        primID_type |= isInstance   ? LEAF_TYPE_INSTANCE   : 0;
                                                                        //local_leafGenData[local_leafDataID+j].blockID = bID;
                                                                        //local_leafGenData[local_leafDataID+j].primID = primID_type;
                                                                        //local_leafGenData[local_leafDataID+j].geomID = geomID;

                                                                        const uint destID = j == 0 ? innerID : leafID+j-1;                                                                         
                                                                        leafGenData[destID].blockID = bID; 
                                                                        leafGenData[destID].primID = primID_type; 
                                                                        leafGenData[destID].geomID = geomID; 

                                                                        /* if (j == 0) */
                                                                        /* { */
                                                                        /*   leafGenData[innerID].blockID = bID;  */
                                                                        /*   leafGenData[innerID].primID = primID_type;  */
                                                                        /*   leafGenData[innerID].geomID = geomID;                                                                            */
                                                                        /* } */
                                                                        /* else */
                                                                        /* { */
                                                                        /*   local_leafGenData[local_leafID+j-1].blockID = bID;  */
                                                                        /*   local_leafGenData[local_leafID+j-1].primID = primID_type;  */
                                                                        /*   local_leafGenData[local_leafID+j-1].geomID = geomID;                                                                            */
                                                                        /* } */
                                                                        
                                                                        node_blockID += isInstance  ? 2 : 1;
                                                                      }

#endif                                                                      
                                                                                                                                           
                                                                      item.barrier(sycl::access::fence_space::local_space);

                                                                      /* --- write out all local entries to global memory --- */

                                                                      /* for (uint i=localID;i<numLeaves;i+=localSize)   */
                                                                      /*   leafGenData[leafID+i] = local_leafGenData[i];   */
                                                                      
                                                                      if (localID == 0)
                                                                      {
                                                                        const uint syncID = gpu::atomic_add_global(&globals->sync,(uint)1);
                                                                        if (syncID == numGroups-1)
                                                                        {
                                                                          /* --- reset atomics --- */
                                                                          globals->sync = 0;
                                                                          host_device_tasks[0] = globals->numBuildRecords;
                                                                        }
                                                                      }
                             
                                                                    });
		  
                                                 });
      gpu::waitOnQueueAndCatchException(gpu_queue);
      double dt = gpu::getDeviceExecutionTiming(queue_event);      
      total_time += dt;
      if (unlikely(verbose))      
        PRINT3("final flattening iteration ",(float)dt,(float)total_time);
    }

    /* ---- Phase IV: for each primID, geomID pair generate corresponding leaf data --- */
    const uint leaves = host_device_tasks[0]; // = globals->leaf_mem_allocator_cur - globals->leaf_mem_allocator_start;
   
    if (leaves)
    {        
      /* for (uint i=0;i<leaves;i++) */
      /*   PRINT5(i,leafGenData[i].blockID,leafGenData[i].primID,leafGenData[i].primID & BVH2Ploc::BIT_MASK,leafGenData[i].geomID & 0x00ffffff); */
      
      {
        const uint wgSize = 256;
        const sycl::nd_range<1> nd_range1(gpu::alignTo(leaves,wgSize),sycl::range<1>(wgSize));              
        sycl::event queue_event = gpu_queue.submit([&](sycl::handler &cgh) {
            sycl::accessor< QuadLeaf, 1, sycl_read_write, sycl_local> _localLeaf(sycl::range<1>(wgSize),cgh);
            cgh.parallel_for(nd_range1,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(16)      
                             {
                               const uint globalID = item.get_global_id(0);
                               const uint localID  = item.get_local_id(0);
                               QuadLeaf *localLeaf = _localLeaf.get_pointer();
                               bool valid = false;
                               QuadLeaf *qleaf = nullptr;
                                                                      
                               if (globalID < leaves)                                                                        
                               {
                                 qleaf = (QuadLeaf *)globals->nodeBlockPtr(leafGenData[globalID].blockID);
                                 const uint geomID = leafGenData[globalID].geomID & 0x00ffffff;
                                 const uint primID0 = leafGenData[globalID].primID & LEAF_TYPE_MASK_LOW;
                                 const uint primID1 = primID0 + ((leafGenData[globalID].geomID & 0x7fffffff) >> 24);
                                 
                                 if ((leafGenData[globalID].primID & LEAF_TYPE_MASK_HIGH) == LEAF_TYPE_QUAD)
                                 {
                                   TriQuadMesh &mesh = triQuadMesh[geomID];

                                   valid = true;
                                   // ====================                           
                                   // === TriangleMesh ===
                                   // ====================                                                                        
                                   if (mesh.isTriangleMesh())
                                   {
                                     const TriangleMesh::Triangle tri = mesh.getTrianglesPtr()[primID0];
                                     const Vec3f p0 = mesh.getVerticesPtr()[tri.v[0]];
                                     const Vec3f p1 = mesh.getVerticesPtr()[tri.v[1]];
                                     const Vec3f p2 = mesh.getVerticesPtr()[tri.v[2]];
                                     Vec3f p3 = p2;
                                     uint lb0 = 0,lb1 = 0, lb2 = 0;
                
                                     /* handle paired triangle */
                                     if (primID0 != primID1)
                                     {
                                       const TriangleMesh::Triangle tri1 = mesh.getTrianglesPtr()[primID1];
          
                                       const uint p3_index = try_pair_triangles(uint3(tri.v[0],tri.v[1],tri.v[2]),uint3(tri1.v[0],tri1.v[1],tri1.v[2]),lb0,lb1,lb2);
                                       p3 = mesh.getVerticesPtr()[tri1.v[p3_index]];                                   
                                     }

                                     localLeaf[localID] = QuadLeaf( p0,p1,p2,p3, lb0,lb1,lb2, 0, geomID, primID0, primID1, GeometryFlags::OPAQUE, 0xFF, /*i == (numChildren-1)*/ true );
                                     //write(leaf,(float16*)qleaf);
                                   }
                                   // ================                           
                                   // === QuadMesh ===
                                   // ================
                                   else
                                   {
                                     const QuadMesh::Quad quad = mesh.getQuadsPtr()[primID0];
                                     const Vec3f p0 = mesh.getVerticesPtr()[quad.v[0]];
                                     const Vec3f p1 = mesh.getVerticesPtr()[quad.v[1]];
                                     const Vec3f p2 = mesh.getVerticesPtr()[quad.v[2]];
                                     const Vec3f p3 = mesh.getVerticesPtr()[quad.v[3]];
                                     localLeaf[localID] = QuadLeaf( p0,p1,p3,p2, 3,2,1, 0, geomID, primID0, primID0, GeometryFlags::OPAQUE, 0xFF, /*i == (numChildren-1)*/ true );                                                                          
                                   }
                                 }
                                 else if ((leafGenData[globalID].primID & LEAF_TYPE_MASK_HIGH) == LEAF_TYPE_INSTANCE)
                                 {
                                   InstancePrimitive *dest = (InstancePrimitive *)qleaf;         
                                   const uint instID = primID0;
                                   Instance* instance = scene->get<Instance>(instID);
                                   void* accel = static_cast<Scene*>(instance->object)->hwaccel.data();
                                   const AffineSpace3fa local2world = instance->getLocal2World();
                                   const uint64_t root = (uint64_t)accel + 128; //static_cast<QBVH6*>(accel)->root();
                                   *dest = InstancePrimitive(local2world,root,instID,mask32_to_mask8(instance->mask));                                             
                                 }
                                 else if ((leafGenData[globalID].primID & LEAF_TYPE_MASK_HIGH) == LEAF_TYPE_PROCEDURAL) // FIXME use valid
                                 {
                                   const uint userGeomID = geomID;
                                   const uint mask32 = mask32_to_mask8(scene->get(userGeomID)->mask);
                                   ProceduralLeaf *dest = (ProceduralLeaf *)qleaf;
                                   PrimLeafDesc leafDesc(0,geomID,GeometryFlags::NONE,mask32,PrimLeafDesc::TYPE_OPACITY_CULLING_ENABLED);
                                   *dest = ProceduralLeaf(leafDesc,primID0,true);
                                 }
                               }

                               /* ================================== */                                                                      
                               /* === write out to global memory === */
                               /* ================================== */
                                                                      
                               const uint subgroupLocalID = get_sub_group_local_id();
                               uint mask = sub_group_ballot(valid);
                               while(mask)
                               {
                                 const uint index = sycl::ctz(mask);
                                 mask &= mask-1;
                                 uint ID = sub_group_broadcast(localID,index);
                                 uint* dest = sub_group_broadcast((uint*)qleaf,index);
                                 uint* source = (uint*)&localLeaf[ID];
                                 const uint v = source[subgroupLocalID];                                                                        
                                 sub_group_store(dest,v);
                               }                                                                                                            
                             });
		  
          });
        gpu::waitOnQueueAndCatchException(gpu_queue);
        double dt = gpu::getDeviceExecutionTiming(queue_event);      
        total_time += dt;
        if (unlikely(verbose))      
          PRINT3("final leaf generation ",(float)dt,(float)total_time);
      }
    } 
    
    return (float)total_time;
  }  
  
  
}

#endif
