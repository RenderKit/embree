// ======================================================================== //
// Copyright 2009-2015 Intel Corporation                                    //
//                                                                          //
// Licensed under the Apache License, Version 2.0 (the "License");          //
// you may not use this file except in compliance with the License.         //
// You may obtain a copy of the License at                                  //
//                                                                          //
//     http://www.apache.org/licenses/LICENSE-2.0                           //
//                                                                          //
// Unless required by applicable law or agreed to in writing, software      //
// distributed under the License is distributed on an "AS IS" BASIS,        //
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. //
// See the License for the specific language governing permissions and      //
// limitations under the License.                                           //
// ======================================================================== //

#include "../bvh/bvh.h"
#include "../bvh/bvh_statistics.h"
#include "bvh4_rotate.h"
#include "../../common/profile.h"
#include "../../algorithms/parallel_prefix_sum.h"
#include "../../algorithms/parallel_for_for_prefix_sum.h"

#include "../builders/primrefgen.h"
#include "../builders/bvh_builder_morton.h"

#include "../geometry/triangle.h"
#include "../geometry/trianglev.h"
#include "../geometry/trianglei.h"

#define ROTATE_TREE 1 // specifies number of tree rotation rounds to perform
#define PROFILE 0
#define BLOCK_SIZE 4096

namespace embree 
{
  namespace isa
  {
    struct AllocBVH4Node
    {
      __forceinline BVH4::Node* operator() (MortonBuildRecord<BVH4::NodeRef>& current, MortonBuildRecord<BVH4::NodeRef>* children, size_t numChildren, FastAllocator::ThreadLocal2* alloc)
      {
        BVH4::Node* node = (BVH4::Node*) alloc->alloc0.malloc(sizeof(BVH4::Node),BVH4::byteNodeAlignment); node->clear();
        *current.parent = BVH4::encodeNode(node);
        for (size_t i=0; i<numChildren; i++)
          children[i].parent = &node->child(i);
        return node;
      }
    };
    
    struct SetBVH4Bounds
    {
      BVH4* bvh;
      __forceinline SetBVH4Bounds (BVH4* bvh) : bvh(bvh) {}

      __forceinline BBox3fa operator() (BVH4::Node* node, const BBox3fa* bounds, size_t N)
      {
        BBox3fa res = empty;
        for (size_t i=0; i<N; i++) {
          const BBox3fa b = bounds[i];
          res.extend(b);
          node->set(i,b);
        }

#if ROTATE_TREE
        size_t n = 0;
        for (size_t i=0; i<N; i++) 
          n += bounds[i].lower.a;

        if (n >= 4096) {
          for (size_t i=0; i<N; i++) {
            if (bounds[i].lower.a < 4096) {
              for (int j=0; j<ROTATE_TREE; j++) 
                BVH4Rotate::rotate(node->child(i)); 
              node->child(i).setBarrier();
            }
          }
        }
        res.lower.a = n;
#endif

        return res;
      }
    };

    template<typename Primitive>
    struct CreateMortonLeaf;

    template<> struct CreateMortonLeaf<Triangle4>
    {
      __forceinline CreateMortonLeaf (TriangleMesh* mesh, MortonID32Bit* morton)
        : mesh(mesh), morton(morton) {}

      void operator() (MortonBuildRecord<BVH4::NodeRef>& current, FastAllocator::ThreadLocal2* alloc, BBox3fa& box_o)
      {
        vfloat4 lower(pos_inf);
        vfloat4 upper(neg_inf);
        size_t items = current.size();
        size_t start = current.begin;
        assert(items<=4);
        
        /* allocate leaf node */
        Triangle4* accel = (Triangle4*) alloc->alloc1.malloc(sizeof(Triangle4));
        *current.parent = BVH4::encodeLeaf((char*)accel,1);
        
        vint4 vgeomID = -1, vprimID = -1;
        Vec3vf4 v0 = zero, v1 = zero, v2 = zero;
        
        for (size_t i=0; i<items; i++)
        {
          const size_t index = morton[start+i].index;
          const size_t primID = index; 
          const size_t geomID = this->mesh->id;
          const TriangleMesh* mesh = this->mesh;
          const TriangleMesh::Triangle& tri = mesh->triangle(primID);
          const Vec3fa& p0 = mesh->vertex(tri.v[0]);
          const Vec3fa& p1 = mesh->vertex(tri.v[1]);
          const Vec3fa& p2 = mesh->vertex(tri.v[2]);
          lower = min(lower,(vfloat4)p0,(vfloat4)p1,(vfloat4)p2);
          upper = max(upper,(vfloat4)p0,(vfloat4)p1,(vfloat4)p2);
          vgeomID [i] = geomID;
          vprimID [i] = primID;
          v0.x[i] = p0.x; v0.y[i] = p0.y; v0.z[i] = p0.z;
          v1.x[i] = p1.x; v1.y[i] = p1.y; v1.z[i] = p1.z;
          v2.x[i] = p2.x; v2.y[i] = p2.y; v2.z[i] = p2.z;
        }
        Triangle4::store_nt(accel,Triangle4(v0,v1,v2,vgeomID,vprimID));
        box_o = BBox3fa((Vec3fa)lower,(Vec3fa)upper);
#if ROTATE_TREE
        box_o.lower.a = current.size();
#endif
      }
    
    private:
      TriangleMesh* mesh;
      MortonID32Bit* morton;
    };
    
#if defined(__AVX__)
    
    template<> struct CreateMortonLeaf<Triangle8>
    {
      __forceinline CreateMortonLeaf (TriangleMesh* mesh, MortonID32Bit* morton)
        : mesh(mesh), morton(morton) {}
      
      void operator() (MortonBuildRecord<BVH4::NodeRef>& current, FastAllocator::ThreadLocal2* alloc, BBox3fa& box_o)
      {
        vfloat4 lower(pos_inf);
        vfloat4 upper(neg_inf);
        size_t items = current.size();
        size_t start = current.begin;
        assert(items<=8);
        
        /* allocate leaf node */
        Triangle8* accel = (Triangle8*) alloc->alloc1.malloc(sizeof(Triangle8));
        *current.parent = BVH4::encodeLeaf((char*)accel,1);
        
        vint8 vgeomID = -1, vprimID = -1;
        Vec3vf8 v0 = zero, v1 = zero, v2 = zero;
        
        for (size_t i=0; i<items; i++)
        {
          const size_t index = morton[start+i].index;
          const size_t primID = index; 
          const size_t geomID = this->mesh->id;
          const TriangleMesh* mesh = this->mesh;
          const TriangleMesh::Triangle& tri = mesh->triangle(primID);
          const Vec3fa& p0 = mesh->vertex(tri.v[0]);
          const Vec3fa& p1 = mesh->vertex(tri.v[1]);
          const Vec3fa& p2 = mesh->vertex(tri.v[2]);
          lower = min(lower,(vfloat4)p0,(vfloat4)p1,(vfloat4)p2);
          upper = max(upper,(vfloat4)p0,(vfloat4)p1,(vfloat4)p2);
          vgeomID [i] = geomID;
          vprimID [i] = primID;
          v0.x[i] = p0.x; v0.y[i] = p0.y; v0.z[i] = p0.z;
          v1.x[i] = p1.x; v1.y[i] = p1.y; v1.z[i] = p1.z;
          v2.x[i] = p2.x; v2.y[i] = p2.y; v2.z[i] = p2.z;
        }
        Triangle8::store_nt(accel,Triangle8(v0,v1,v2,vgeomID,vprimID));
        box_o = BBox3fa((Vec3fa)lower,(Vec3fa)upper);
#if ROTATE_TREE
        box_o.lower.a = current.size();
#endif
      }

    private:
      TriangleMesh* mesh;
      MortonID32Bit* morton;
    };
#endif
    
    template<> struct CreateMortonLeaf<Triangle4v>
    {
      __forceinline CreateMortonLeaf (TriangleMesh* mesh, MortonID32Bit* morton)
        : mesh(mesh), morton(morton) {}
      
      void operator() (MortonBuildRecord<BVH4::NodeRef>& current, FastAllocator::ThreadLocal2* alloc, BBox3fa& box_o)
      {
        vfloat4 lower(pos_inf);
        vfloat4 upper(neg_inf);
        size_t items = current.size();
        size_t start = current.begin;
        assert(items<=4);
        
        /* allocate leaf node */
        Triangle4v* accel = (Triangle4v*) alloc->alloc1.malloc(sizeof(Triangle4v));
        *current.parent = BVH4::encodeLeaf((char*)accel,1);
        
        vint4 vgeomID = -1, vprimID = -1;
        Vec3vf4 v0 = zero, v1 = zero, v2 = zero;

        for (size_t i=0; i<items; i++)
        {
          const size_t index = morton[start+i].index;
          const size_t primID = index; 
          const size_t geomID = this->mesh->id;
          const TriangleMesh* mesh = this->mesh;
          const TriangleMesh::Triangle& tri = mesh->triangle(primID);
          const Vec3fa& p0 = mesh->vertex(tri.v[0]);
          const Vec3fa& p1 = mesh->vertex(tri.v[1]);
          const Vec3fa& p2 = mesh->vertex(tri.v[2]);
          lower = min(lower,(vfloat4)p0,(vfloat4)p1,(vfloat4)p2);
          upper = max(upper,(vfloat4)p0,(vfloat4)p1,(vfloat4)p2);
          vgeomID [i] = geomID;
          vprimID [i] = primID;
          v0.x[i] = p0.x; v0.y[i] = p0.y; v0.z[i] = p0.z;
          v1.x[i] = p1.x; v1.y[i] = p1.y; v1.z[i] = p1.z;
          v2.x[i] = p2.x; v2.y[i] = p2.y; v2.z[i] = p2.z;
        }
        Triangle4v::store_nt(accel,Triangle4v(v0,v1,v2,vgeomID,vprimID));
        box_o = BBox3fa((Vec3fa)lower,(Vec3fa)upper);
#if ROTATE_TREE
        box_o.lower.a = current.size();
#endif
      }
    private:
      TriangleMesh* mesh;
      MortonID32Bit* morton;
    };

    template<> struct CreateMortonLeaf<Triangle4i>
    {
      __forceinline CreateMortonLeaf (TriangleMesh* mesh, MortonID32Bit* morton)
        : mesh(mesh), morton(morton) {}
      
      void operator() (MortonBuildRecord<BVH4::NodeRef>& current, FastAllocator::ThreadLocal2* alloc, BBox3fa& box_o)
      {
        vfloat4 lower(pos_inf);
        vfloat4 upper(neg_inf);
        size_t items = current.size();
        size_t start = current.begin;
        assert(items<=4);
        
        /* allocate leaf node */
        Triangle4i* accel = (Triangle4i*) alloc->alloc1.malloc(sizeof(Triangle4i));
        *current.parent = BVH4::encodeLeaf((char*)accel,1);
        
        vint4 vgeomID = -1, vprimID = -1;
        Vec3f* v0[4] = { nullptr, nullptr, nullptr, nullptr };
        vint4 v1 = zero, v2 = zero;
        
        for (size_t i=0; i<items; i++)
        {
          const size_t index = morton[start+i].index;
          const size_t primID = index; 
          const size_t geomID = this->mesh->id;
          const TriangleMesh* mesh = this->mesh;
          const TriangleMesh::Triangle& tri = mesh->triangle(primID);
          const Vec3fa& p0 = mesh->vertex(tri.v[0]);
          const Vec3fa& p1 = mesh->vertex(tri.v[1]);
          const Vec3fa& p2 = mesh->vertex(tri.v[2]);
          lower = min(lower,(vfloat4)p0,(vfloat4)p1,(vfloat4)p2);
          upper = max(upper,(vfloat4)p0,(vfloat4)p1,(vfloat4)p2);
          vgeomID[i] = geomID;
          vprimID[i] = primID;
          v0[i] = (Vec3f*) mesh->vertexPtr(tri.v[0]); 
          v1[i] = (int*)   mesh->vertexPtr(tri.v[1])-(int*)v0[i]; 
          v2[i] = (int*)   mesh->vertexPtr(tri.v[2])-(int*)v0[i]; 
        }
        
        for (size_t i=items; i<4; i++)
        {
          vgeomID[i] = -1;
          vprimID[i] = -1;
          v0[i] = v0[0];
          v1[i] = 0; 
          v2[i] = 0;
        }
        
        new (accel) Triangle4i(v0,v1,v2,vgeomID,vprimID);
        box_o = BBox3fa((Vec3fa)lower,(Vec3fa)upper);
#if ROTATE_TREE
        box_o.lower.a = current.size();
#endif
      }
    private:
      TriangleMesh* mesh;
      MortonID32Bit* morton;
    };
    
    template<typename Mesh>
    struct CalculateMeshBounds
    {
      __forceinline CalculateMeshBounds (Mesh* mesh)
        : mesh(mesh) {}
      
      __forceinline const BBox3fa operator() (const MortonID32Bit& morton) {
        return mesh->bounds(morton.index);
      }
      
    private:
      Mesh* mesh;
    };        
    
    template<typename Mesh, typename Primitive>
    class BVH4MeshBuilderMorton : public Builder
    {
    public:
      
      BVH4MeshBuilderMorton (BVH4* bvh, Mesh* mesh, const size_t minLeafSize, const size_t maxLeafSize)
        : bvh(bvh), mesh(mesh), minLeafSize(minLeafSize), maxLeafSize(maxLeafSize), morton(bvh->device), numPrimitives(0) {}
      
      /*! Destruction */
      ~BVH4MeshBuilderMorton () {
        //bvh->shrink();
      }
      
      /* build function */
      void build(size_t threadIndex, size_t threadCount) 
      {
        /* We have to clear the allocator to guarantee that we can
         * temporarily use the first allocation block for sorting the
         * morton codes. */
        const size_t numNewPrimitives = mesh->size();
        if (numNewPrimitives != numPrimitives) bvh->alloc.clear();
        numPrimitives = numNewPrimitives;
        
        /* skip build for empty scene */
        if (numPrimitives == 0) {
          bvh->set(BVH4::emptyNode,empty,0);
          return;
        }
        
        auto progress = [&] (size_t dn) { bvh->scene->progressMonitor(dn); };
        
        /* preallocate arrays */
        morton.resize(numPrimitives);
        size_t bytesAllocated = numPrimitives*sizeof(BVH4::Node)/(4*BVH4::N) + size_t(1.2f*Primitive::blocks(numPrimitives)*sizeof(Primitive));
        bytesAllocated = max(bytesAllocated,numPrimitives*sizeof(MortonID32Bit)); // the first allocation block is reused to sort the morton codes
        bvh->alloc.init(bytesAllocated,2*bytesAllocated);
        
        /* compute scene bounds */
        ParallelPrefixSumState<size_t> pstate;
        const BBox3fa centBounds = parallel_reduce 
          ( size_t(0), numPrimitives, size_t(BLOCK_SIZE), BBox3fa(empty), [&](const range<size_t>& r) -> BBox3fa
            {
              BBox3fa bounds(empty);
              for (size_t i=r.begin(); i<r.end(); i++) bounds.extend(center2(mesh->bounds(i)));
              return bounds;
            }, [] (const BBox3fa& a, const BBox3fa& b) { return merge(a,b); });
        
        /* compute morton codes */
        MortonID32Bit* dest = (MortonID32Bit*) bvh->alloc.ptr();
        MortonCodeGenerator::MortonCodeMapping mapping(centBounds);
        size_t numPrimitivesGen = parallel_prefix_sum( pstate, size_t(0), numPrimitives, size_t(BLOCK_SIZE), size_t(0), [&](const range<size_t>& r, const size_t base) -> size_t {
            size_t N = 0;
            MortonCodeGenerator generator(mapping,&morton.data()[r.begin()]);
            for (ssize_t j=r.begin(); j<r.end(); j++)
            {
              BBox3fa bounds = empty;
              if (!mesh->valid(j,&bounds)) continue;
              generator(bounds,j);
              N++;
            }
            return N;
          }, std::plus<size_t>());
        
        /* fallback in case some primitives were invalid */
        if (numPrimitivesGen != numPrimitives)
        {
          numPrimitivesGen = parallel_prefix_sum( pstate, size_t(0), numPrimitives, size_t(BLOCK_SIZE), size_t(0), [&](const range<size_t>& r, const size_t base) -> size_t {
              size_t N = 0;
              MortonCodeGenerator generator(mapping,&morton.data()[base]);
              for (ssize_t j=r.begin(); j<r.end(); j++)
              {
                BBox3fa bounds = empty;
                if (!mesh->valid(j,&bounds)) continue;
                generator(bounds,j);
                N++;
              }
              return N;
            }, std::plus<size_t>());
        }
        
        /* create BVH */
        AllocBVH4Node allocNode;
        SetBVH4Bounds setBounds(bvh);
        CreateMortonLeaf<Primitive> createLeaf(mesh,morton.data());
        CalculateMeshBounds<Mesh> calculateBounds(mesh);
        auto node_bounds = bvh_builder_morton_internal<BVH4::NodeRef>(
          BVH4::CreateAlloc(bvh), BBox3fa(empty),
          allocNode,setBounds,createLeaf,calculateBounds,progress,
          morton.data(),dest,numPrimitivesGen,BVH4::N,BVH4::maxBuildDepth,minLeafSize,maxLeafSize);
        
        bvh->set(node_bounds.first,node_bounds.second,numPrimitives);
        
#if ROTATE_TREE
        for (int i=0; i<ROTATE_TREE; i++) 
          BVH4Rotate::rotate(bvh->root);
        bvh->clearBarrier(bvh->root);
#endif
        
        /* clear temporary data for static geometry */
        if (mesh->isStatic()) 
        {
          morton.clear();
          bvh->shrink();
        }
        bvh->cleanup();
      }
      
      void clear() {
        morton.clear();
      }
      
    private:
      BVH4* bvh;
      Mesh* mesh;
      const size_t minLeafSize;
      const size_t maxLeafSize;
      size_t numPrimitives;
      mvector<MortonID32Bit> morton;
    };
    
    Builder* BVH4Triangle4MeshBuilderMortonGeneral  (void* bvh, TriangleMesh* mesh, size_t mode) { return new class BVH4MeshBuilderMorton<TriangleMesh,Triangle4> ((BVH4*)bvh,mesh,4,4*BVH4::maxLeafBlocks); }
#if defined(__AVX__)
    Builder* BVH4Triangle8MeshBuilderMortonGeneral  (void* bvh, TriangleMesh* mesh, size_t mode) { return new class BVH4MeshBuilderMorton<TriangleMesh,Triangle8> ((BVH4*)bvh,mesh,8,8*BVH4::maxLeafBlocks); }
#endif
    Builder* BVH4Triangle4vMeshBuilderMortonGeneral (void* bvh, TriangleMesh* mesh, size_t mode) { return new class BVH4MeshBuilderMorton<TriangleMesh,Triangle4v>((BVH4*)bvh,mesh,4,4*BVH4::maxLeafBlocks); }
    Builder* BVH4Triangle4iMeshBuilderMortonGeneral (void* bvh, TriangleMesh* mesh, size_t mode) { return new class BVH4MeshBuilderMorton<TriangleMesh,Triangle4i>((BVH4*)bvh,mesh,4,4*BVH4::maxLeafBlocks); }
  }
}

