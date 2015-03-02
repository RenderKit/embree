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

#include "bvh4.h"
#include "bvh4_rotate.h"
#include "common/profile.h"
#include "algorithms/parallel_prefix_sum.h"
#include "algorithms/parallel_for_for_prefix_sum.h"

#include "builders_new/primrefgen.h"
#include "builders_new/bvh_builder_morton.h"

#include "geometry/triangle1.h"
#include "geometry/triangle4.h"
#include "geometry/triangle8.h"
#include "geometry/triangle1v.h"
#include "geometry/triangle4v.h"
#include "geometry/triangle4i.h"

#define ROTATE_TREE 5

namespace embree 
{
  namespace isa
  {
    struct AllocBVH4Node
    {
      __forceinline BVH4::Node* operator() (MortonBuildRecord<BVH4::NodeRef>& current, MortonBuildRecord<BVH4::NodeRef>* children, size_t numChildren, FastAllocator::ThreadLocal2* alloc)
      {
        BVH4::Node* node = (BVH4::Node*) alloc->alloc0.malloc(sizeof(BVH4::Node)); node->clear();
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
                BVH4Rotate::rotate(bvh,node->child(i)); 
              node->child(i).setBarrier();
            }
          }
        }
        res.lower.a = n;
#endif

        return res;
      }
    };

    struct CreateTriangle1Leaf
    {
      __forceinline CreateTriangle1Leaf (Scene* scene, MortonID32Bit* morton, size_t encodeShift, size_t encodeMask)
        : scene(scene), mesh(NULL), morton(morton), encodeShift(encodeShift), encodeMask(encodeMask) {}

       __forceinline CreateTriangle1Leaf (TriangleMesh* mesh, MortonID32Bit* morton)
         : scene(NULL), mesh(mesh), morton(morton), encodeShift(0), encodeMask(-1) {}

      void operator() (MortonBuildRecord<BVH4::NodeRef>& current, FastAllocator::ThreadLocal2* alloc, BBox3fa& box_o)
      {
        ssef lower(pos_inf);
        ssef upper(neg_inf);
        size_t items = current.size();
        size_t start = current.begin;
        
        /* allocate leaf node */
        Triangle1* accel = (Triangle1*) alloc->alloc1.malloc(items*sizeof(Triangle1));
        *current.parent = BVH4::encodeLeaf((char*)accel,items);
        
        for (size_t i=0; i<items; i++) 
        {	
          const size_t index = morton[start+i].index;
          const size_t primID = index & encodeMask; 
          const size_t geomID = this->mesh ? this->mesh->id : (index >> encodeShift); 
          const TriangleMesh* mesh = this->mesh ? this->mesh : scene->getTriangleMesh(geomID);
          const TriangleMesh::Triangle& tri = mesh->triangle(primID);
          
          const ssef v0 = select(0x7,(ssef)mesh->vertex(tri.v[0]),zero);
          const ssef v1 = select(0x7,(ssef)mesh->vertex(tri.v[1]),zero);
          const ssef v2 = select(0x7,(ssef)mesh->vertex(tri.v[2]),zero);
          
          lower = min(lower,v0,v1,v2);
          upper = max(upper,v0,v1,v2);
          
          const ssef e1 = v0 - v1;
          const ssef e2 = v2 - v0;	     
          const ssef normal = cross(e1,e2);
          
          store4f_nt(&accel[i].v0,cast(insert<3>(cast(v0),primID)));
          store4f_nt(&accel[i].v1,cast(insert<3>(cast(v1),geomID)));
          store4f_nt(&accel[i].v2,cast(insert<3>(cast(v2),mesh->mask)));
          store4f_nt(&accel[i].Ng,cast(insert<3>(cast(normal),0)));
        }
        box_o = BBox3fa((Vec3fa)lower,(Vec3fa)upper);
#if ROTATE_TREE
        box_o.lower.a = current.size();
#endif
      }

    private:
      Scene* scene;
      TriangleMesh* mesh;
      MortonID32Bit* morton;
      size_t encodeShift;
      size_t encodeMask;
    };
    
    struct CreateTriangle4Leaf
    {
      __forceinline CreateTriangle4Leaf (Scene* scene, MortonID32Bit* morton, size_t encodeShift, size_t encodeMask)
        : scene(scene), mesh(NULL), morton(morton), encodeShift(encodeShift), encodeMask(encodeMask) {}
      
      __forceinline CreateTriangle4Leaf (TriangleMesh* mesh, MortonID32Bit* morton)
        : scene(NULL), mesh(mesh), morton(morton), encodeShift(0), encodeMask(-1) {}

      void operator() (MortonBuildRecord<BVH4::NodeRef>& current, FastAllocator::ThreadLocal2* alloc, BBox3fa& box_o)
      {
        ssef lower(pos_inf);
        ssef upper(neg_inf);
        size_t items = current.size();
        size_t start = current.begin;
        assert(items<=4);
        
        /* allocate leaf node */
        Triangle4* accel = (Triangle4*) alloc->alloc1.malloc(sizeof(Triangle4));
        *current.parent = BVH4::encodeLeaf((char*)accel,1);
        
        ssei vgeomID = -1, vprimID = -1, vmask = -1;
        sse3f v0 = zero, v1 = zero, v2 = zero;
        
        for (size_t i=0; i<items; i++)
        {
          const size_t index = morton[start+i].index;
          const size_t primID = index & encodeMask; 
          const size_t geomID = this->mesh ? this->mesh->id : (index >> encodeShift); 
          const TriangleMesh* mesh = this->mesh ? this->mesh : scene->getTriangleMesh(geomID);
          const TriangleMesh::Triangle& tri = mesh->triangle(primID);
          const Vec3fa& p0 = mesh->vertex(tri.v[0]);
          const Vec3fa& p1 = mesh->vertex(tri.v[1]);
          const Vec3fa& p2 = mesh->vertex(tri.v[2]);
          lower = min(lower,(ssef)p0,(ssef)p1,(ssef)p2);
          upper = max(upper,(ssef)p0,(ssef)p1,(ssef)p2);
          vgeomID [i] = geomID;
          vprimID [i] = primID;
          vmask   [i] = mesh->mask;
          v0.x[i] = p0.x; v0.y[i] = p0.y; v0.z[i] = p0.z;
          v1.x[i] = p1.x; v1.y[i] = p1.y; v1.z[i] = p1.z;
          v2.x[i] = p2.x; v2.y[i] = p2.y; v2.z[i] = p2.z;
        }
        Triangle4::store_nt(accel,Triangle4(v0,v1,v2,vgeomID,vprimID,vmask,false));
        box_o = BBox3fa((Vec3fa)lower,(Vec3fa)upper);
#if ROTATE_TREE
        box_o.lower.a = current.size();
#endif
      }
    
    private:
      Scene* scene;
      TriangleMesh* mesh;
      MortonID32Bit* morton;
      size_t encodeShift;
      size_t encodeMask;
    };
    
#if defined(__AVX__)
    
    struct CreateTriangle8Leaf
    {
      __forceinline CreateTriangle8Leaf (Scene* scene, MortonID32Bit* morton, size_t encodeShift, size_t encodeMask)
        : scene(scene), mesh(NULL), morton(morton), encodeShift(encodeShift), encodeMask(encodeMask) {}
      
      __forceinline CreateTriangle8Leaf (TriangleMesh* mesh, MortonID32Bit* morton)
        : scene(NULL), mesh(mesh), morton(morton), encodeShift(0), encodeMask(-1) {}
      
      void operator() (MortonBuildRecord<BVH4::NodeRef>& current, FastAllocator::ThreadLocal2* alloc, BBox3fa& box_o)
      {
        ssef lower(pos_inf);
        ssef upper(neg_inf);
        size_t items = current.size();
        size_t start = current.begin;
        assert(items<=8);
        
        /* allocate leaf node */
        Triangle8* accel = (Triangle8*) alloc->alloc1.malloc(sizeof(Triangle8));
        *current.parent = BVH4::encodeLeaf((char*)accel,1);
        
        avxi vgeomID = -1, vprimID = -1, vmask = -1;
        avx3f v0 = zero, v1 = zero, v2 = zero;
        
        for (size_t i=0; i<items; i++)
        {
          const size_t index = morton[start+i].index;
          const size_t primID = index & encodeMask; 
          const size_t geomID = this->mesh ? this->mesh->id : (index >> encodeShift); 
          const TriangleMesh* mesh = this->mesh ? this->mesh : scene->getTriangleMesh(geomID);
          const TriangleMesh::Triangle& tri = mesh->triangle(primID);
          const Vec3fa& p0 = mesh->vertex(tri.v[0]);
          const Vec3fa& p1 = mesh->vertex(tri.v[1]);
          const Vec3fa& p2 = mesh->vertex(tri.v[2]);
          lower = min(lower,(ssef)p0,(ssef)p1,(ssef)p2);
          upper = max(upper,(ssef)p0,(ssef)p1,(ssef)p2);
          vgeomID [i] = geomID;
          vprimID [i] = primID;
          vmask   [i] = mesh->mask;
          v0.x[i] = p0.x; v0.y[i] = p0.y; v0.z[i] = p0.z;
          v1.x[i] = p1.x; v1.y[i] = p1.y; v1.z[i] = p1.z;
          v2.x[i] = p2.x; v2.y[i] = p2.y; v2.z[i] = p2.z;
        }
        new (accel) Triangle8(v0,v1,v2,vgeomID,vprimID,vmask,false); // FIXME: use storent
        box_o = BBox3fa((Vec3fa)lower,(Vec3fa)upper);
#if ROTATE_TREE
        box_o.lower.a = current.size();
#endif
      }

    private:
      Scene* scene;
      TriangleMesh* mesh;
      MortonID32Bit* morton;
      size_t encodeShift;
      size_t encodeMask;
    };
#endif
    
    struct CreateTriangle1vLeaf
    {
      __forceinline CreateTriangle1vLeaf (Scene* scene, MortonID32Bit* morton, size_t encodeShift, size_t encodeMask)
        : scene(scene), mesh(NULL), morton(morton), encodeShift(encodeShift), encodeMask(encodeMask) {}
      
      __forceinline CreateTriangle1vLeaf (TriangleMesh* mesh, MortonID32Bit* morton)
        : scene(NULL), mesh(mesh), morton(morton), encodeShift(0), encodeMask(-1) {}
      
      void operator() (MortonBuildRecord<BVH4::NodeRef>& current, FastAllocator::ThreadLocal2* alloc, BBox3fa& box_o)
      {
        ssef lower(pos_inf);
        ssef upper(neg_inf);
        size_t items = current.size();
        size_t start = current.begin;
        
        /* allocate leaf node */
        Triangle1v* accel = (Triangle1v*) alloc->alloc1.malloc(items*sizeof(Triangle1v));
        *current.parent = BVH4::encodeLeaf((char*)accel,items);
        
        for (size_t i=0; i<items; i++) 
        {	
          const size_t index = morton[start+i].index;
          const size_t primID = index & encodeMask; 
          const size_t geomID = this->mesh ? this->mesh->id : (index >> encodeShift); 
          const TriangleMesh* mesh = this->mesh ? this->mesh : scene->getTriangleMesh(geomID);
          const TriangleMesh::Triangle& tri = mesh->triangle(primID);
          
          const ssef v0 = select(0x7,(ssef)mesh->vertex(tri.v[0]),zero);
          const ssef v1 = select(0x7,(ssef)mesh->vertex(tri.v[1]),zero);
          const ssef v2 = select(0x7,(ssef)mesh->vertex(tri.v[2]),zero);
          
          lower = min(lower,v0,v1,v2);
          upper = max(upper,v0,v1,v2);
          
          const ssef e1 = v0 - v1;
          const ssef e2 = v2 - v0;	     
          const ssef normal = cross(e1,e2);
          
          store4f_nt(&accel[i].v0,cast(insert<3>(cast(v0),primID)));
          store4f_nt(&accel[i].v1,cast(insert<3>(cast(v1),geomID)));
          store4f_nt(&accel[i].v2,cast(insert<3>(cast(v2),mesh->mask)));
        }
        box_o = BBox3fa((Vec3fa)lower,(Vec3fa)upper);
#if ROTATE_TREE
        box_o.lower.a = current.size();
#endif
      }
    private:
      Scene* scene;
      TriangleMesh* mesh;
      MortonID32Bit* morton;
      size_t encodeShift;
      size_t encodeMask;
    };

    struct CreateTriangle4vLeaf
    {
      __forceinline CreateTriangle4vLeaf (Scene* scene, MortonID32Bit* morton, size_t encodeShift, size_t encodeMask)
        : scene(scene), mesh(NULL), morton(morton), encodeShift(encodeShift), encodeMask(encodeMask) {}
      
      __forceinline CreateTriangle4vLeaf (TriangleMesh* mesh, MortonID32Bit* morton)
        : scene(NULL), mesh(mesh), morton(morton), encodeShift(0), encodeMask(-1) {}
      
      void operator() (MortonBuildRecord<BVH4::NodeRef>& current, FastAllocator::ThreadLocal2* alloc, BBox3fa& box_o)
      {
        ssef lower(pos_inf);
        ssef upper(neg_inf);
        size_t items = current.size();
        size_t start = current.begin;
        assert(items<=4);
        
        /* allocate leaf node */
        Triangle4v* accel = (Triangle4v*) alloc->alloc1.malloc(sizeof(Triangle4v));
        *current.parent = BVH4::encodeLeaf((char*)accel,1);
        
        ssei vgeomID = -1, vprimID = -1, vmask = -1;
        sse3f v0 = zero, v1 = zero, v2 = zero;

        for (size_t i=0; i<items; i++)
        {
          const size_t index = morton[start+i].index;
          const size_t primID = index & encodeMask; 
          const size_t geomID = this->mesh ? this->mesh->id : (index >> encodeShift); 
          const TriangleMesh* mesh = this->mesh ? this->mesh : scene->getTriangleMesh(geomID);
          const TriangleMesh::Triangle& tri = mesh->triangle(primID);
          const Vec3fa& p0 = mesh->vertex(tri.v[0]);
          const Vec3fa& p1 = mesh->vertex(tri.v[1]);
          const Vec3fa& p2 = mesh->vertex(tri.v[2]);
          lower = min(lower,(ssef)p0,(ssef)p1,(ssef)p2);
          upper = max(upper,(ssef)p0,(ssef)p1,(ssef)p2);
          vgeomID [i] = geomID;
          vprimID [i] = primID;
          vmask   [i] = mesh->mask;
          v0.x[i] = p0.x; v0.y[i] = p0.y; v0.z[i] = p0.z;
          v1.x[i] = p1.x; v1.y[i] = p1.y; v1.z[i] = p1.z;
          v2.x[i] = p2.x; v2.y[i] = p2.y; v2.z[i] = p2.z;
        }
        Triangle4v::store_nt(accel,Triangle4v(v0,v1,v2,vgeomID,vprimID,vmask,false));
        box_o = BBox3fa((Vec3fa)lower,(Vec3fa)upper);
#if ROTATE_TREE
        box_o.lower.a = current.size();
#endif
      }
    private:
      Scene* scene;
      TriangleMesh* mesh;
      MortonID32Bit* morton;
      size_t encodeShift;
      size_t encodeMask;
    };

    struct CreateTriangle4iLeaf
    {
      __forceinline CreateTriangle4iLeaf (Scene* scene, MortonID32Bit* morton, size_t encodeShift, size_t encodeMask)
        : scene(scene), mesh(NULL), morton(morton), encodeShift(encodeShift), encodeMask(encodeMask) {}
      
      __forceinline CreateTriangle4iLeaf (TriangleMesh* mesh, MortonID32Bit* morton)
        : scene(NULL), mesh(mesh), morton(morton), encodeShift(0), encodeMask(-1) {}
      
      void operator() (MortonBuildRecord<BVH4::NodeRef>& current, FastAllocator::ThreadLocal2* alloc, BBox3fa& box_o)
      {
        ssef lower(pos_inf);
        ssef upper(neg_inf);
        size_t items = current.size();
        size_t start = current.begin;
        assert(items<=4);
        
        /* allocate leaf node */
        Triangle4i* accel = (Triangle4i*) alloc->alloc1.malloc(sizeof(Triangle4i));
        *current.parent = BVH4::encodeLeaf((char*)accel,1);
        
        ssei vgeomID = -1, vprimID = -1;
        Vec3f* v0[4] = { NULL, NULL, NULL, NULL };
        ssei v1 = zero, v2 = zero;
        
        for (size_t i=0; i<items; i++)
        {
          const size_t index = morton[start+i].index;
          const size_t primID = index & encodeMask; 
          const size_t geomID = this->mesh ? this->mesh->id : (index >> encodeShift); 
          const TriangleMesh* mesh = this->mesh ? this->mesh : scene->getTriangleMesh(geomID);
          const TriangleMesh::Triangle& tri = mesh->triangle(primID);
          const Vec3fa& p0 = mesh->vertex(tri.v[0]);
          const Vec3fa& p1 = mesh->vertex(tri.v[1]);
          const Vec3fa& p2 = mesh->vertex(tri.v[2]);
          lower = min(lower,(ssef)p0,(ssef)p1,(ssef)p2);
          upper = max(upper,(ssef)p0,(ssef)p1,(ssef)p2);
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
        
        new (accel) Triangle4i(v0,v1,v2,vgeomID,vprimID,false);
        box_o = BBox3fa((Vec3fa)lower,(Vec3fa)upper);
#if ROTATE_TREE
        box_o.lower.a = current.size();
#endif
      }
    private:
      Scene* scene;
      TriangleMesh* mesh;
      MortonID32Bit* morton;
      size_t encodeShift;
      size_t encodeMask;
    };

    struct CalculateBounds
    {
      __forceinline CalculateBounds (Scene* scene, size_t encodeShift, size_t encodeMask)
        : scene(scene), encodeShift(encodeShift), encodeMask(encodeMask) {}
      
      __forceinline const BBox3fa operator() (const MortonID32Bit& morton)
      {
        const size_t index = morton.index;
        const size_t primID = index & encodeMask; 
        const size_t geomID = index >> encodeShift; 
        const TriangleMesh* mesh = scene->getTriangleMesh(geomID);
        return mesh->bounds(primID);
      }
      
    private:
      Scene* scene;
      size_t encodeShift;
      size_t encodeMask;
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
    

    template<typename Mesh, typename CreateLeaf>
      class BVH4MeshBuilderMortonGeneral2 : public Builder
    {
    public:
      
      BVH4MeshBuilderMortonGeneral2 (BVH4* bvh, Mesh* mesh, const size_t minLeafSize, const size_t maxLeafSize)
        : bvh(bvh), mesh(mesh), minLeafSize(minLeafSize), maxLeafSize(maxLeafSize), morton(NULL), bytesMorton(0), numPrimitives(0) {}
      

      /*! Destruction */
      ~BVH4MeshBuilderMortonGeneral2 ()
      {
        if (morton) os_free(morton,bytesMorton);
        bvh->alloc.shrink();
      }
      
      /* build function */
      void build(size_t threadIndex, size_t threadCount) 
      {
        /* calculate size of scene */
        size_t numPrimitivesOld = numPrimitives;
        numPrimitives = mesh->size();
        
        /* preallocate arrays */
        if (numPrimitivesOld != numPrimitives)
        {
          bvh->init(sizeof(BVH4::Node),numPrimitives,threadCount);
          if (morton) os_free(morton,bytesMorton);
          bytesMorton = ((numPrimitives+4)&(-4)) * sizeof(MortonID32Bit);
          morton = (MortonID32Bit* ) os_malloc(bytesMorton); memset(morton,0,bytesMorton); 
        }
        
        /* skip build for empty scene */
        if (numPrimitives == 0) {
          bvh->set(BVH4::emptyNode,empty,0);
          return;
        }
      
        /* verbose mode */
        if (g_verbose >= 1)
	  std::cout << "building BVH4<" << bvh->primTy.name << "> with " << TOSTRING(isa) "::BVH4MeshBuilderMortonGeneral ... " << std::flush;

	double t0 = 0.0f, dt = 0.0f;
	//profile(2,20,numPrimitives,[&] (ProfileTimer& timer) {
	    
            if (g_verbose >= 1) t0 = getSeconds();
	    
            //bvh->alloc.init(numPrimitives*sizeof(BVH4::Node),numPrimitives*sizeof(BVH4::Node));
            size_t bytesAllocated = (numPrimitives+7)/8*sizeof(BVH4::Node) + size_t(1.2f*(numPrimitives+3)/4)*sizeof(Triangle4);
            bvh->alloc2.init(bytesAllocated,2*bytesAllocated); // FIXME: not working if scene size changes, initial block has to get reallocated as used as temporary data

#if 0
            /* compute scene bounds */
            const BBox3fa centBounds = parallel_reduce 
              ( size_t(0), numPrimitives, BBox3fa(empty), [&](const range<size_t>& r) -> BBox3fa
                {
                  BBox3fa bounds(empty);
                  for (size_t i=r.begin(); i<r.end(); i++) bounds.extend(center2(mesh->bounds(i)));
                  return bounds;
                }, [] (const BBox3fa& a, const BBox3fa& b) { return merge(a,b); });
           
            //timer("compute_bounds");

            /* compute morton codes */
            MortonID32Bit* dest = (MortonID32Bit*) bvh->alloc2.ptr();
            MortonCodeGenerator::MortonCodeMapping mapping(centBounds);
            parallel_for
              ( size_t(0), numPrimitives, [&](const range<size_t>& r) 
                {
                  MortonCodeGenerator generator(mapping,&dest[r.begin()]);
                  for (size_t i=r.begin(); i<r.end(); i++) {
                    generator(mesh->bounds(i),i);
                  }
                });

#else 

            ParallelPrefixSumState<PrimInfo> pstate;
      
            /* compute scene bounds */
            PrimInfo pinfo = parallel_prefix_sum( pstate, size_t(0), mesh->size(), size_t(1024), PrimInfo(empty), [&](const range<size_t>& r, const PrimInfo& base) -> PrimInfo
            {
              PrimInfo pinfo(empty);
              for (size_t j=r.begin(); j<r.end(); j++)
              {
                BBox3fa bounds = empty;
                if (!mesh->valid(j,&bounds)) continue;
                pinfo.add(bounds);
              }
              return pinfo;
            }, [](const PrimInfo& a, const PrimInfo& b) -> PrimInfo { return PrimInfo::merge(a,b); });
            const BBox3fa centBounds = pinfo.centBounds;

            /* compute morton codes */
            MortonID32Bit* dest = (MortonID32Bit*) bvh->alloc2.ptr();
            MortonCodeGenerator::MortonCodeMapping mapping(centBounds);
            pinfo = parallel_prefix_sum( pstate, size_t(0), numPrimitives, size_t(1024), PrimInfo(empty), [&](const range<size_t>& r, const PrimInfo& base) -> PrimInfo
            {
              PrimInfo pinfo(empty);
              MortonCodeGenerator generator(mapping,&dest[base.size()]);
              for (ssize_t j=r.begin(); j<r.end(); j++)
              {
                BBox3fa bounds = empty;
                if (!mesh->valid(j,&bounds)) continue;
                pinfo.add(bounds);
                generator(bounds,j);
              }
              return pinfo;
            }, [](const PrimInfo& a, const PrimInfo& b) -> PrimInfo { return PrimInfo::merge(a,b); });

#endif
            //timer("compute_morton_codes");

            /* create BVH */
            AllocBVH4Node allocNode;
            SetBVH4Bounds setBounds(bvh);
            CreateLeaf createLeaf(mesh,morton);
            CalculateMeshBounds<Mesh> calculateBounds(mesh);
            auto node_bounds = bvh_builder_center_internal<BVH4::NodeRef>(
              [&] () { return bvh->alloc2.threadLocal2(); },
              BBox3fa(empty),
              allocNode,setBounds,createLeaf,calculateBounds,
              dest,morton,pinfo.size(),4,BVH4::maxBuildDepth,minLeafSize,maxLeafSize);
            bvh->set(node_bounds.first,node_bounds.second,numPrimitives);

#if ROTATE_TREE
            for (int i=0; i<ROTATE_TREE; i++) 
              BVH4Rotate::rotate(bvh,bvh->root);
            bvh->clearBarrier(bvh->root);
#endif

            //timer("compute_tree");
            //timer("bvh4_builder_morton_new");

            if (g_verbose >= 1) dt = getSeconds()-t0;
            
            //});
        
        /* clear temporary data for static geometry */
	//bool staticGeom = mesh ? mesh->isStatic() : scene->isStatic(); // FIXME: implement
	//if (staticGeom) prims.resize(0,true);
        bvh->alloc2.cleanup();
	
	/* verbose mode */
	if (g_verbose >= 1)
	  std::cout << "[DONE] " << 1000.0f*dt << "ms (" << numPrimitives/dt*1E-6 << " Mprim/s)" << std::endl;
	if (g_verbose >= 2)
	  bvh->printStatistics();
      }
      
    public:
      BVH4* bvh;               //!< Output BVH
      Mesh* mesh;
      const size_t minLeafSize;
      const size_t maxLeafSize;
      
    public:
      MortonID32Bit* morton; // FIXME: use vector_t class
      size_t bytesMorton;
      size_t numPrimitives;
    };
    
    Builder* BVH4Triangle1MeshBuilderMortonGeneral  (void* bvh, TriangleMesh* mesh, size_t mode) { return new class BVH4MeshBuilderMortonGeneral2<TriangleMesh,CreateTriangle1Leaf> ((BVH4*)bvh,mesh,4,1*BVH4::maxLeafBlocks); }
    Builder* BVH4Triangle4MeshBuilderMortonGeneral  (void* bvh, TriangleMesh* mesh, size_t mode) { return new class BVH4MeshBuilderMortonGeneral2<TriangleMesh,CreateTriangle4Leaf> ((BVH4*)bvh,mesh,4,4*BVH4::maxLeafBlocks); }
#if defined(__AVX__)
    Builder* BVH4Triangle8MeshBuilderMortonGeneral  (void* bvh, TriangleMesh* mesh, size_t mode) { return new class BVH4MeshBuilderMortonGeneral2<TriangleMesh,CreateTriangle8Leaf> ((BVH4*)bvh,mesh,8,8*BVH4::maxLeafBlocks); }
#endif
    Builder* BVH4Triangle1vMeshBuilderMortonGeneral (void* bvh, TriangleMesh* mesh, size_t mode) { return new class BVH4MeshBuilderMortonGeneral2<TriangleMesh,CreateTriangle1vLeaf>((BVH4*)bvh,mesh,4,1*BVH4::maxLeafBlocks); }
    Builder* BVH4Triangle4vMeshBuilderMortonGeneral (void* bvh, TriangleMesh* mesh, size_t mode) { return new class BVH4MeshBuilderMortonGeneral2<TriangleMesh,CreateTriangle4vLeaf>((BVH4*)bvh,mesh,4,4*BVH4::maxLeafBlocks); }
    Builder* BVH4Triangle4iMeshBuilderMortonGeneral (void* bvh, TriangleMesh* mesh, size_t mode) { return new class BVH4MeshBuilderMortonGeneral2<TriangleMesh,CreateTriangle4iLeaf>((BVH4*)bvh,mesh,4,4*BVH4::maxLeafBlocks); }


    template<typename Mesh, typename CreateLeaf>
      class BVH4SceneBuilderMortonGeneral2 : public Builder
    {
    public:
      
      BVH4SceneBuilderMortonGeneral2 (BVH4* bvh, Scene* scene, const size_t minLeafSize, const size_t maxLeafSize)
        : bvh(bvh), scene(scene), minLeafSize(minLeafSize), maxLeafSize(maxLeafSize), encodeShift(0), encodeMask(-1), morton(NULL), bytesMorton(0), numPrimitives(0) {}
      
      /*! Destruction */
      ~BVH4SceneBuilderMortonGeneral2 ()
      {
        if (morton) os_free(morton,bytesMorton);
        bvh->alloc.shrink();
      }
      
      /* build function */
      void build(size_t threadIndex, size_t threadCount) 
      {
        /* calculate size of scene */
        size_t numPrimitivesOld = numPrimitives;
        numPrimitives = scene->getNumPrimitives<Mesh,1>();
        
        /* calculate groupID, primID encoding */ // FIXME: does not work for all scenes, use 64 bit IDs !!!
        Scene::Iterator<Mesh,1> iter2(scene);
        size_t numGroups = iter2.size();
        size_t maxPrimsPerGroup = iter2.maxPrimitivesPerGeometry();
        encodeShift = __bsr(maxPrimsPerGroup) + 1;
        encodeMask = ((size_t)1 << encodeShift)-1;
        size_t maxGroups = ((size_t)1 << (31-encodeShift))-1;
        if (maxPrimsPerGroup > encodeMask || numGroups > maxGroups) 
          THROW_RUNTIME_ERROR("encoding error in morton builder");
        
        /* preallocate arrays */
        if (numPrimitivesOld != numPrimitives)
        {
          bvh->init(sizeof(BVH4::Node),numPrimitives,threadCount);
          if (morton) os_free(morton,bytesMorton);
          bytesMorton = ((numPrimitives+4)&(-4)) * sizeof(MortonID32Bit);
          morton = (MortonID32Bit* ) os_malloc(bytesMorton); memset(morton,0,bytesMorton); 
        }

        /* skip build for empty scene */
        if (numPrimitives == 0) {
          bvh->set(BVH4::emptyNode,empty,0);
          return;
        }
      
        /* verbose mode */
        if (g_verbose >= 1)
	  std::cout << "building BVH4<" << bvh->primTy.name << "> with " << TOSTRING(isa) "::BVH4SceneBuilderMortonGeneral ... " << std::flush;

	double t0 = 0.0f, dt = 0.0f;
	//profile(2,20,numPrimitives,[&] (ProfileTimer& timer) {
        
            if (g_verbose >= 1) t0 = getSeconds();
	    
            //bvh->alloc.init(numPrimitives*sizeof(BVH4::Node),numPrimitives*sizeof(BVH4::Node));
            size_t bytesAllocated = (numPrimitives+7)/8*sizeof(BVH4::Node) + size_t(1.2f*(numPrimitives+3)/4)*sizeof(Triangle4);
            bvh->alloc2.init(bytesAllocated,2*bytesAllocated); // FIXME: not working if scene size changes, initial block has to get reallocated as used as temporary data

#if 0
            
            /* compute scene bounds */
            Scene::Iterator<Mesh,1> iter1(scene);
            const BBox3fa centBounds = parallel_for_for_reduce 
              ( iter1, BBox3fa(empty), [&](Mesh* mesh, const range<size_t>& r, size_t k) -> BBox3fa
                {
                  BBox3fa bounds(empty);
                  for (size_t i=r.begin(); i<r.end(); i++) bounds.extend(center2(mesh->bounds(i)));
                  return bounds;
                }, [] (const BBox3fa& a, const BBox3fa& b) { return merge(a,b); });
            
            //timer("compute_bounds");

            /* compute morton codes */
            Scene::Iterator<Mesh,1> iter(scene);
            MortonID32Bit* dest = (MortonID32Bit*) bvh->alloc2.ptr();
            MortonCodeGenerator::MortonCodeMapping mapping(centBounds);
            parallel_for_for
              ( iter, [&](Mesh* mesh, const range<size_t>& r, size_t k) 
                {
                  MortonCodeGenerator generator(mapping,&dest[k]);
                  for (size_t i=r.begin(); i<r.end(); i++) {
                    generator(mesh->bounds(i),i | (mesh->id << encodeShift));
                  }
                });
            
            //timer("compute_morton_codes");

#else 

            Scene::Iterator<Mesh,1> iter1(scene);
            ParallelForForPrefixSumState<PrimInfo> pstate;
            pstate.init(iter1,size_t(1024));

            /* compute scene bounds */
            PrimInfo pinfo = parallel_for_for_prefix_sum( pstate, iter1, PrimInfo(empty), [&](Mesh* mesh, const range<size_t>& r, size_t k, const PrimInfo& base) -> PrimInfo
            {
              PrimInfo pinfo(empty);
              for (size_t j=r.begin(); j<r.end(); j++)
              {
                BBox3fa bounds = empty;
                if (!mesh->valid(j,&bounds)) continue;
                pinfo.add(bounds);
              }
              return pinfo;
            }, [](const PrimInfo& a, const PrimInfo& b) -> PrimInfo { return PrimInfo::merge(a,b); });
            const BBox3fa centBounds = pinfo.centBounds;

            //timer("compute_bounds");

            /* compute morton codes */
            MortonID32Bit* dest = (MortonID32Bit*) bvh->alloc2.ptr();
            MortonCodeGenerator::MortonCodeMapping mapping(centBounds);
            pinfo = parallel_for_for_prefix_sum( pstate, iter1, PrimInfo(empty), [&](Mesh* mesh, const range<size_t>& r, size_t k, const PrimInfo& base) -> PrimInfo
            {
              PrimInfo pinfo(empty);
              MortonCodeGenerator generator(mapping,&dest[base.size()]);
              for (ssize_t j=r.begin(); j<r.end(); j++)
              {
                BBox3fa bounds = empty;
                if (!mesh->valid(j,&bounds)) continue;
                pinfo.add(bounds);
                generator(bounds,j | (mesh->id << encodeShift));
              }
              return pinfo;
            }, [](const PrimInfo& a, const PrimInfo& b) -> PrimInfo { return PrimInfo::merge(a,b); });
            
            //timer("compute_morton_codes");
            
#endif

            /* create BVH */
            AllocBVH4Node allocNode;
            SetBVH4Bounds setBounds(bvh);
            CreateLeaf createLeaf(scene,morton,encodeShift,encodeMask);
            CalculateBounds calculateBounds(scene,encodeShift,encodeMask);
            auto node_bounds = bvh_builder_center_internal<BVH4::NodeRef>(
              [&] () { return bvh->alloc2.threadLocal2(); },
                BBox3fa(empty),
                allocNode,setBounds,createLeaf,calculateBounds,
                  dest,morton,pinfo.size(),4,BVH4::maxBuildDepth,minLeafSize,maxLeafSize);
            bvh->set(node_bounds.first,node_bounds.second,numPrimitives);

#if ROTATE_TREE
            for (int i=0; i<ROTATE_TREE; i++) 
              BVH4Rotate::rotate(bvh,bvh->root);
            bvh->clearBarrier(bvh->root);
#endif
            //timer("compute_tree");

            if (g_verbose >= 1) dt = getSeconds()-t0;

            //});
        
        /* clear temporary data for static geometry */
	//bool staticGeom = mesh ? mesh->isStatic() : scene->isStatic(); // FIXME: implement
	//if (staticGeom) prims.resize(0,true);
        bvh->alloc2.cleanup();

	/* verbose mode */
	if (g_verbose >= 1)
	  std::cout << "[DONE] " << 1000.0f*dt << "ms (" << numPrimitives/dt*1E-6 << " Mprim/s)" << std::endl;
	if (g_verbose >= 2)
	  bvh->printStatistics();
      }
      
    public:
      BVH4* bvh;               //!< Output BVH
      Scene* scene;
      const size_t minLeafSize;
      const size_t maxLeafSize;
      size_t encodeShift;
      size_t encodeMask;
      
    public:
      MortonID32Bit* morton; // FIXME: use vector_t class
      size_t bytesMorton;
      size_t numPrimitives;
    };

    Builder* BVH4Triangle1SceneBuilderMortonGeneral  (void* bvh, Scene* scene, size_t mode) { return new class BVH4SceneBuilderMortonGeneral2<TriangleMesh,CreateTriangle1Leaf> ((BVH4*)bvh,scene,4,1*BVH4::maxLeafBlocks); }
    Builder* BVH4Triangle4SceneBuilderMortonGeneral  (void* bvh, Scene* scene, size_t mode) { return new class BVH4SceneBuilderMortonGeneral2<TriangleMesh,CreateTriangle4Leaf> ((BVH4*)bvh,scene,4,4*BVH4::maxLeafBlocks); }
#if defined(__AVX__)
    Builder* BVH4Triangle8SceneBuilderMortonGeneral  (void* bvh, Scene* scene, size_t mode) { return new class BVH4SceneBuilderMortonGeneral2<TriangleMesh,CreateTriangle8Leaf> ((BVH4*)bvh,scene,8,8*BVH4::maxLeafBlocks); }
#endif
    Builder* BVH4Triangle1vSceneBuilderMortonGeneral (void* bvh, Scene* scene, size_t mode) { return new class BVH4SceneBuilderMortonGeneral2<TriangleMesh,CreateTriangle1vLeaf>((BVH4*)bvh,scene,4,1*BVH4::maxLeafBlocks); }
    Builder* BVH4Triangle4vSceneBuilderMortonGeneral (void* bvh, Scene* scene, size_t mode) { return new class BVH4SceneBuilderMortonGeneral2<TriangleMesh,CreateTriangle4vLeaf>((BVH4*)bvh,scene,4,4*BVH4::maxLeafBlocks); }
    Builder* BVH4Triangle4iSceneBuilderMortonGeneral (void* bvh, Scene* scene, size_t mode) { return new class BVH4SceneBuilderMortonGeneral2<TriangleMesh,CreateTriangle4iLeaf>((BVH4*)bvh,scene,4,4*BVH4::maxLeafBlocks); }

  }
}

