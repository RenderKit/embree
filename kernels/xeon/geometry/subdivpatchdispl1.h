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
  class CatmullClark1Edge
  {
  public:
    void init(size_t N, CatmullClark1Ring& r0, CatmullClark1Ring& r1)
    {
      v.init(N,3);
      for (size_t i=0; i<N; i++) v(i,0) = v(i,1) = v(i,2) = Vec3fa(nan);
      this->K = 1;
      init(r0,r1);
    }

    __forceinline void subdivide(CatmullClark1Ring& r0, CatmullClark1Ring& r1) 
    {
      assert(2*K+1 <= v.width());
      size_t K0 = K;
      size_t K1 = 2*K;
      
      Vec3fa v20 = zero;
      Vec3fa v21 = zero;
      Vec3fa v22 = zero;
      
      Vec3fa v10 = v((K+1)-1,0);
      Vec3fa v11 = v((K+1)-1,1);
      Vec3fa v12 = v((K+1)-1,2);
      
      for (ssize_t x=K-1; x>=0; x--) 
	{
	  /* load next column */
	  Vec3fa v00 = v(x,0);
	  Vec3fa v01 = v(x,1);
	  Vec3fa v02 = v(x,2);
        
	  /* calculate face points and edge centers */
	  const Vec3fa c00 = 0.25f*(v00+v10+v01+v11);
	  const Vec3fa c10 = 0.50f*(v10+v11);
	  const Vec3fa c20 = 0.25f*(v10+v20+v11+v21);
	  const Vec3fa c01 = 0.50f*(v01+v11);
	  const Vec3fa c21 = 0.50f*(v11+v21);
	  const Vec3fa c02 = 0.25f*(v01+v11+v02+v12);
	  const Vec3fa c12 = 0.50f*(v11+v12);
	  const Vec3fa c22 = 0.25f*(v11+v21+v12+v22);
        
	  /* store face points and edge point at 2*x+1 */
	  v(2*x+1,0) = c00;
	  v(2*x+1,1) = 0.5f*(c01+0.5f*(c00+c02));
	  v(2*x+1,2) = c02;
        
	  /* store face points and edge point at 2*x+2 */
	  const Vec3fa F = 0.25f*(c00+c20+c02+c22);
	  const Vec3fa R = 0.25f*(c10+c01+c21+c12);
	  const Vec3fa P = v11;
	  v(2*x+2,0) = 0.5f*(c10+0.5f*(c00+c20));
	  v(2*x+2,1) = 0.25*F + 0.5*R + 0.25*P;
	  v(2*x+2,2) = 0.5f*(c12+0.5f*(c02+c22));
        
	  /* propagate points to next iteration */
	  v20 = v10; v10 = v00;
	  v21 = v11; v11 = v01;
	  v22 = v12; v12 = v02;
	}        

      K = 2*K;
      init(r0,r1);
    }
    
    __forceinline void init(CatmullClark1Ring& ring0, CatmullClark1Ring& ring1) 
    {
      v(0,0) = ring0.front(2);
      v(0,1) = ring0.vtx;
      v(0,2) = ring0.back(1);
      if (!ring0.has_first_patch()) v(0,0) = 2*v(0,1) - v(0,2);

      v(K,0) = ring1.back(3);
      v(K,1) = ring1.vtx;
      v(K,2) = ring1.front(0);
      if (!ring1.has_prelast_patch()) v(K,0) = 2*v(K,1) - v(K,2);
    }
    
    friend __forceinline std::ostream &operator<<(std::ostream& out, const CatmullClark1Edge& edge)
    {
      out << std::endl;
      for (size_t y=0; y<3; y++) {
        for (size_t x=0; x<edge.K+1; x++) {
          //out << patch.v(x,y) << " ";
          out << std::setw(10) << edge.v(x,y).x << " ";
        }
        out << std::endl;
      }
      return out;
    } 
    
  public:
    Array2D<Vec3fa> v;
    size_t K;
  };

  struct SubdivideIrregularCatmullClarkPatch
  {
    SubdivideIrregularCatmullClarkPatch (const IrregularCatmullClarkPatch& patch, const unsigned int levels)
    : K(1)
    {
      size_t N = 1<<levels;
      size_t M = N+1;
      v.init(M,M,Vec3fa(nan));
      ring00 = patch.ring[0];
      ring10 = patch.ring[1];
      ring11 = patch.ring[2];
      ring01 = patch.ring[3];
      edgeT.init(M,ring00,ring10);
      edgeR.init(M,ring10,ring11);
      edgeB.init(M,ring11,ring01);
      edgeL.init(M,ring01,ring00);
      init();

      for (size_t l=0; l<levels; l++)
        subdivide();
    }

      void subdivide_points()
      {
        size_t K0 = K;
        size_t K1 = 2*K;

        assert(2*K+1 <= v.width());
        assert(2*K+1 <= v.height());

        for (ssize_t y=K; y>=0; y--) {
          for (ssize_t x=K; x>=0; x--) {
            v(2*x+0,2*y+0) = v(x,y);
          }
        }
        for (ssize_t y=0; y<K; y++) {
          for (ssize_t x=0; x<K; x++) {
            v(2*x+1,2*y+0) = Vec3fa(nan);
            v(2*x+0,2*y+1) = Vec3fa(nan);
            v(2*x+1,2*y+1) = Vec3fa(nan);
          }
        }

        for (ssize_t x=K-1; x>=0; x--) 
        {
          const Vec3fa c00 = v(2*x+0,0);
          const Vec3fa c10 = v(2*x+2,0);
          const Vec3fa c01 = v(2*x+0,2);
          const Vec3fa c11 = v(2*x+2,2);
          v(2*x+1,1) = 0.25f*(c00+c01+c10+c11);
          v(2*x+2,1) = 0.50f*(c10+c11);
        }
        
        for (size_t y=1; y<K; y++)
        {
          //Vec3fa v20 = zero;
          Vec3fa c20 = zero;
          Vec3fa v21 = zero;
          Vec3fa v22 = zero;

          //Vec3fa v10 = v(2*K,0);
          Vec3fa v11 = v(2*K,2*y);
          Vec3fa v12 = v(2*K,2*y+2);

          for (ssize_t x=K-1; x>=0; x--) 
          {
            /* load next column */
            //Vec3fa v00 = v(2*x,2*y-2);
            Vec3fa v01 = v(2*x,2*y+0);
            Vec3fa v02 = v(2*x,2*y+2);
            
            /* calculate face points and edge centers */
            const Vec3fa c00 = v(2*x+1,2*y-1);
            const Vec3fa c10 = v(2*x+2,2*y-1);
            //const Vec3fa c20 = v(2*x+3,2*y-1);
            const Vec3fa c01 = 0.50f*(v01+v11);
            const Vec3fa c21 = 0.50f*(v11+v21);
            const Vec3fa c02 = 0.25f*(v01+v11+v02+v12);
            const Vec3fa c12 = 0.50f*(v11+v12);
            const Vec3fa c22 = 0.25f*(v11+v21+v12+v22);

            /* store face points and edge point at 2*x+1 */
            //v(2*x+1,2*y-1) = c00;
            v(2*x+1,2*y+0) = 0.5f*(c01+0.5f*(c00+c02));
            v(2*x+1,2*y+1) = c02;

            /* store face points and edge point at 2*x+2 */
            const Vec3fa F = 0.25f*(c00+c20+c02+c22);
            const Vec3fa R = 0.25f*(c10+c01+c21+c12);
            const Vec3fa P = v11;
            v(2*x+2,2*y-1) = 0.5f*(c10+0.5f*(c00+c20));
            v(2*x+2,2*y+0) = 0.25*F + 0.5*R + 0.25*P;
            v(2*x+2,2*y+1) = c12;
            
            /* propagate points to next iteration */
            //v20 = v10; v10 = v00;
            v21 = v11; v11 = v01;
            v22 = v12; v12 = v02;
            c20 = c00;
          }        
        }

        for (ssize_t x=1; x<K; x++) 
          v(2*x,2*K-1) = 0.5f*(v(2*x,2*K-1)+0.5f*(v(2*x-1,2*K-1)+v(2*x+1,2*K-1)));

        K=2*K;
        init();
      }

      void init()
      {
        for (size_t i=0; i<(K+1); i++)
        {
          v(i,0) = edgeT.v(i,1);
          v(K,i) = edgeR.v(i,1);
          v(i,K) = edgeB.v(K-i,1);
          v(0,i) = edgeL.v(K-i,1);
        }
      }

      void subdivide()
      {
        CatmullClark1Ring ring00a; ring00.update(ring00a); ring00 = ring00a;
        CatmullClark1Ring ring01a; ring01.update(ring01a); ring01 = ring01a;
        CatmullClark1Ring ring10a; ring10.update(ring10a); ring10 = ring10a;
        CatmullClark1Ring ring11a; ring11.update(ring11a); ring11 = ring11a;
        edgeT.subdivide(ring00,ring10);
        edgeR.subdivide(ring10,ring11);
        edgeB.subdivide(ring11,ring01);
        edgeL.subdivide(ring01,ring00);
        subdivide_points();
      }

  public:
    size_t K;
    CatmullClark1Ring ring00,ring01,ring10,ring11;
    CatmullClark1Edge edgeT, edgeB, edgeL, edgeR; 
    Array2D<Vec3fa> v;
  };

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

#if 0
      SubdivideIrregularCatmullClarkPatch subdivided2(patch,2);
      SubdivideIrregularCatmullClarkPatch subdivided3(patch,3);
      const BBox3fa bounds = leaf->build(scene,subdivided2.v,subdivided3.v,Tt,Tr,Tb,Tl);
#endif

#if 0
      const RegularCatmullClarkPatch regular(patch);
      const BBox3fa bounds = leaf->build(scene,regular);
#endif

#if 1
      GregoryPatch gregory; gregory.init(patch);
      const BBox3fa bounds = leaf->build(scene,gregory);
#endif
      return std::pair<BBox3fa,BVH4::NodeRef>(bounds,BVH4::encodeTypedLeaf(leaf,0));
    }

    const std::pair<BBox3fa,BVH4::NodeRef> build(FastAllocator& alloc,
                                                  const IrregularCatmullClarkPatch& patch,
                                                 unsigned x, unsigned y, int l, int maxDepth,
                                                 bool Tt, bool Tr, bool Tb, bool Tl) // tagged transition edges
    {
      //if (unlikely(l == maxDepth))
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

    const std::pair<BBox3fa,BVH4::NodeRef> build(FastAllocator& alloc, const GeneralIrregularCatmullClarkPatch& patch, int maxDepth)
    {
      size_t N;
      IrregularCatmullClarkPatch patches[GeneralIrregularCatmullClarkPatch::SIZE]; 
      patch.subdivide(patches,N);

      BVH4::Node* node = (BVH4::Node*) alloc.malloc(sizeof(BVH4::Node),16);
      BBox3fa bounds = empty;
      for (size_t i=0, j=0; i<N; i++, j++) {
        if (j == 4) {
          BVH4::Node* node1 = (BVH4::Node*) alloc.malloc(sizeof(BVH4::Node),16);
          node1->set(0,bounds,BVH4::encodeNode2(node));
          node = node1;
          j = 1;
        }
        std::pair<BBox3fa,BVH4::NodeRef> b = build(alloc,patches[i],0,0,1,maxDepth,false,false,false,false);
        bounds.extend(b.first);
        node->set(j,b.first,b.second);
      }
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
#if 0
      IrregularCatmullClarkPatch patch(h,vertices);
      const std::pair<BBox3fa,BVH4::NodeRef> root = build(alloc,patch,0,0,0,(int)levels-3,false,false,false,false);
      //const std::pair<BBox3fa,BVH4::NodeRef> root = build(alloc,patch,0,0,0,1,false,false,false,false);
#else
      GeneralIrregularCatmullClarkPatch patch(h,vertices);
      const std::pair<BBox3fa,BVH4::NodeRef> root = build(alloc,patch,(int)levels-3);
#endif

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
