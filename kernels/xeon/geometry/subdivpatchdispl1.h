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

    const std::pair<BBox3fa,BVH4::NodeRef> leaf(FastAllocator& alloc,
                                                const IrregularCatmullClarkPatch& patch,
                                                unsigned x, unsigned y, unsigned l,
                                                bool Tt, bool Tr, bool Tb, bool Tl) // tagged transition edges
    {
      QuadQuad4x4* leaf = (QuadQuad4x4*) alloc.malloc(sizeof(QuadQuad4x4),16);
      new (leaf) QuadQuad4x4(8*x,8*y,8*(1<<l),geomID(),primID());
#if 1
      SubdivideIrregularCatmullClarkPatch subdivided2(patch,2);
      SubdivideIrregularCatmullClarkPatch subdivided3(patch,3);
      const BBox3fa bounds = leaf->build(scene,subdivided2.v,subdivided3.v,Tt,Tr,Tb,Tl);
#else
      const RegularCatmullClarkPatch regular(patch);
      const BBox3fa bounds = leaf->build(scene,regular);
#endif
      return std::pair<BBox3fa,BVH4::NodeRef>(bounds,BVH4::encodeTypedLeaf(leaf,0));
    }

    const std::pair<BBox3fa,BVH4::NodeRef> build(FastAllocator& alloc,
                                                  const IrregularCatmullClarkPatch& patch,
                                                 unsigned x, unsigned y, int l, int maxDepth,
                                                 bool Tt, bool Tr, bool Tb, bool Tl) // tagged transition edges
    {
      if (unlikely(l == maxDepth))
        return leaf(alloc,patch,x,y,l,false,false,false,false);

      IrregularCatmullClarkPatch patches[4]; 
      patch.subdivide(patches);

      const bool noleaf = (l+1) < maxDepth;
      const bool subdivide0 = noleaf && !patches[0].dicable();
      const bool subdivide1 = noleaf && !patches[1].dicable();
      const bool subdivide2 = noleaf && !patches[2].dicable();
      const bool subdivide3 = noleaf && !patches[3].dicable();

      BVH4::Node* node = (BVH4::Node*) alloc.malloc(sizeof(BVH4::Node),16);

      std::pair<BBox3fa,BVH4::NodeRef> b00;
      if (subdivide0) b00 = build(alloc,patches[0],2*x+0,2*y+0,l+1,maxDepth,false,!subdivide1,!subdivide3,false);
      else            b00 = leaf (alloc,patches[0],2*x+0,2*y+0,l+1,Tt,false,false,Tl);
      node->set(0,b00.first,b00.second);

      std::pair<BBox3fa,BVH4::NodeRef> b10;
      if (subdivide1) b10 = build(alloc,patches[1],2*x+1,2*y+0,l+1,maxDepth,false,false,!subdivide2,!subdivide0); 
      else            b10 = leaf (alloc,patches[1],2*x+1,2*y+0,l+1,Tt,Tr,false,false);
      node->set(1,b10.first,b10.second);

      std::pair<BBox3fa,BVH4::NodeRef> b11;
      if (subdivide2) b11 = build(alloc,patches[2],2*x+1,2*y+1,l+1,maxDepth,!subdivide1,false,false,!subdivide3); 
      else            b11 = leaf (alloc,patches[2],2*x+1,2*y+1,l+1,false,Tr,Tb,false);
      node->set(2,b11.first,b11.second);

      std::pair<BBox3fa,BVH4::NodeRef> b01;
      if (subdivide3) b01 = build(alloc,patches[3],2*x+0,2*y+1,l+1,maxDepth,!subdivide0,!subdivide2,false,false); 
      else            b01 = leaf (alloc,patches[3],2*x+0,2*y+1,l+1,false,false,Tb,Tl);
      node->set(3,b01.first,b01.second);

      const BBox3fa bounds = merge(b00.first,b10.first,b01.first,b11.first);
      return std::pair<BBox3fa,BVH4::NodeRef>(bounds,BVH4::encodeNode2(node));
    }

  public:
    
    /*! Construction from vertices and IDs. */
    __forceinline SubdivPatchDispl1 (FastAllocator& alloc,
                                     BVH4::NodeRef& parent,
                                     Scene* scene,
                                     const SubdivMesh::HalfEdge* h, 
                                     const Vec3fa* vertices, 
                                     const unsigned int geom, 
                                     const unsigned int prim, 
                                     const unsigned int level,
                                     const bool last)
    : initializing(0), initialized(0), scene(scene), alloc(alloc), parent(parent),
      h(h), vertices(vertices), geom(geom), prim(prim), levels(level) { assert(last); }

    size_t initialize()
    {
      /* let only one thread lazily build this object */
      if (atomic_add(&initializing,1) != 0) {
        while (!initialized) __pause();
        return (size_t)parent;
      }

      /* create patch and build sub-BVH */
      IrregularCatmullClarkPatch patch(h,vertices);
      const std::pair<BBox3fa,BVH4::NodeRef> root = build(alloc,patch,0,0,0,(int)levels-3,false,false,false,false);

      /* link to sub-BVH */
      parent = root.second;
      __memory_barrier();
      initialized = 1;
      return (size_t)root.second;
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
    size_t levels; // FIXME: not required

  public:
    FastAllocator& alloc;
    Scene* scene;
    unsigned geom;
    unsigned prim;
  };
}

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//  FIXME: old code

#if 0

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

#endif
