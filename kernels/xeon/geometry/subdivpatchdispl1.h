// ======================================================================== //
// Copyright 2009-2014 Intel Corporation                                    //
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

#pragma once

#include "primitive.h"
#include "quadquad4x4.h"
#include "common/scene_subdivision.h"
#include "bvh4/bvh4.h"
#include "bvh4/bvh4_builder_fast.h"

namespace embree
{
  struct SubdivPatchDispl1
  {
    struct Type : public PrimitiveType 
    {
      Type ();
      size_t blocks(size_t x) const; 
      size_t size(const char* This) const;
    };

    static Type type;

    /*! branching width of the tree */
    static const size_t N = 4;

    const std::pair<BBox3fa,BVH4::NodeRef> build(LinearAllocatorPerThread::ThreadAllocator& alloc, unsigned x, unsigned y, unsigned l, unsigned maxDepth)
    {
      if (l == maxDepth) {
        new (&leaves(x,y)) QuadQuad4x4(8*x,8*y,v,3,geomID(),primID());
        const BBox3fa bounds = leaves(x,y).build();
        return std::pair<BBox3fa,BVH4::NodeRef>(bounds,bvh.encodeLeaf(&leaves(x,y),0));
      }
      BVH4::Node* node = bvh.allocNode(alloc);
      const std::pair<BBox3fa,BVH4::NodeRef> b00 = build(alloc,2*x+0,2*y+0,l+1,maxDepth); node->set(0,b00.first,b00.second);
      const std::pair<BBox3fa,BVH4::NodeRef> b10 = build(alloc,2*x+1,2*y+0,l+1,maxDepth); node->set(1,b10.first,b10.second);
      const std::pair<BBox3fa,BVH4::NodeRef> b01 = build(alloc,2*x+0,2*y+1,l+1,maxDepth); node->set(2,b01.first,b01.second);
      const std::pair<BBox3fa,BVH4::NodeRef> b11 = build(alloc,2*x+1,2*y+1,l+1,maxDepth); node->set(3,b11.first,b11.second);
      const BBox3fa bounds = merge(b00.first,b10.first,b01.first,b11.first);
      return std::pair<BBox3fa,BVH4::NodeRef>(bounds,bvh.encodeNode(node));
    }

    struct MakeLeaf : public isa::BVH4BuilderFastGeneric::MakeLeaf
    {
      MakeLeaf (SubdivPatchDispl1* This) 
        : This(This) {}

      BVH4::NodeRef operator() (LinearAllocatorPerThread::ThreadAllocator& alloc, const PrimRef* prims, size_t N) const
      {
        assert(N == 1);
        const size_t x = prims->geomID();
        const size_t y = prims->primID();
        return This->bvh.encodeLeaf(&This->leaves(x,y),0);
      }

      SubdivPatchDispl1* This;
    };

    void build(unsigned levels)
    {
      size_t N = 1<<levels;
      size_t K = N/8;
      std::vector<PrimRef> prims;
      prims.resize(K*K);
      for (size_t y=0; y<K; y++) {
        for (size_t x=0; x<K; x++) {
          QuadQuad4x4& leaf = leaves(x,y);
          new (&leaf) QuadQuad4x4(8*x,8*y,v,3,geomID(),primID());
          prims[y*K+x] = PrimRef(leaf.build(),x,y);
        }
      }
      bool listMode = true; // FIXME
      isa::BVH4BuilderFastGeneric builder(&bvh,&prims[0],K*K,MakeLeaf(this),listMode,4,4,false,0,1,1);
      builder.build(0,0);
    }

  public:
    
    /*! Construction from vertices and IDs. */
    __forceinline SubdivPatchDispl1 (BVH4::NodeRef& parent,
                                     Scene* scene,
                                     const SubdivMesh::HalfEdge* h, 
                                     const Vec3fa* vertices, 
                                     const unsigned int geom, 
                                     const unsigned int prim, 
                                     const unsigned int level,
                                     const bool last)
      : initializing(0), initialized(0), scene(scene), parent(parent), bvh(PrimitiveType2<QuadQuad4x4,SubdivPatchDispl1>::type,scene,false), 
        h(h), vertices(vertices), geom(geom), prim(prim), levels(level) { assert(last); }

    size_t initialize()
    {
      if (atomic_add(&initializing,1) != 0) {
        while (!initialized) __pause();
        return (size_t)parent;
      }

      /* create subdivided patch */
      IrregularCatmullClarkPatch patch(h,vertices);
      SubdivideIrregularCatmullClarkPatch(patch,levels,v);

      /* displace points */
      size_t N = 1<<levels;
      size_t M = N+1;
      SubdivMesh* mesh = (SubdivMesh*) scene->get(geom);
      if (mesh->displFunc) 
      {
        Vec2f* uv = new Vec2f[M];
        Vec3fa* displ = new Vec3fa[M];
        for (size_t y=0; y<M; y++) {
          float fy = float(y)/float(M);
          for (size_t x=0; x<M; x++) {
            float fx = float(x)/float(M);
            uv[x] = Vec2f(fx,fy);
          }
          mesh->displFunc(mesh->userPtr,geom,prim,(RTCFloat2*)uv,(RTCFloat3a*)displ,M);
          for (size_t x=0; x<M; x++) {
            const Vec3fa dP = displ[x];
#if defined(DEBUG)
            if (!inside(mesh->displBounds,dP))
              THROW_RUNTIME_ERROR("displacement out of bounds");
#endif
            v(x,y) += dP;
          }
        }
        delete uv;
        delete displ;
      }

      /* build sub-BVH */
      leaves.init(N/8,N/8);
#if 1
      bvh.init(sizeof(BVH4::Node),(N/8)*(N/8),1);
      LinearAllocatorPerThread::ThreadAllocator nodeAlloc(&bvh.alloc);
      const std::pair<BBox3fa,BVH4::NodeRef> root = build(nodeAlloc,0,0,0,levels-3);
      bvh.bounds = root.first;
      bvh.root   = root.second;
#else
      build(levels);
#endif

      /* link to sub-BVH */
      parent = bvh.root;
      __memory_barrier();
      initialized = 1;
      return (size_t)bvh.root;
    }

    // FIXME: destructor gets never called !

    /*! return geometry ID */
    __forceinline unsigned int geomID() const { return geom; }

    /*! return primitive ID */
    __forceinline unsigned int primID() const { return prim; }

  public:
    volatile atomic_t initializing;
    volatile size_t initialized;
    BVH4::NodeRef& parent;

  public:
    const SubdivMesh::HalfEdge* h;
    const Vec3fa* vertices;
    size_t levels;

  public:
    BVH4 bvh;
    Array2D<QuadQuad4x4> leaves;
    Array2D<Vec3fa> v;

  public:
    Scene* scene;
    unsigned geom;
    unsigned prim;
  };
}
