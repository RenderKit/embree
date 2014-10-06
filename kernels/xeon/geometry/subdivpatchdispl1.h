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
#include "common/scene_subdivision.h"
#include "bvh4/bvh4.h"
#include "bvh4/bvh4_builder_fast.h"

#define SUBDIVISION_LEVEL_DISPL 8

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

    struct SmallNode4
    {
      /*! Clears the node. */
      __forceinline void clear() {
        for (size_t i=0; i<4; i++) {
          lower_x[i] = lower_y[i] = lower_z[i] = pos_inf; 
          upper_x[i] = upper_y[i] = upper_z[i] = neg_inf;
        }
      }
      
      /*! Sets bounding box of child. */
      __forceinline void set(size_t j, size_t i, const BBox3fa& bounds) 
      {
        assert(i < N);
        lower_x[j][i] = bounds.lower.x; lower_y[j][i] = bounds.lower.y; lower_z[j][i] = bounds.lower.z;
        upper_x[j][i] = bounds.upper.x; upper_y[j][i] = bounds.upper.y; upper_z[j][i] = bounds.upper.z;
      }
      
      /*! intersection with single rays */
      template<bool robust>
      __forceinline size_t intersect(size_t i, size_t _nearX, size_t _nearY, size_t _nearZ,
                                     const sse3f& org, const sse3f& rdir, const sse3f& org_rdir, const ssef& tnear, const ssef& tfar) const
      {
        const size_t nearX = 4*_nearX, nearY = 4*_nearY, nearZ = 4*_nearZ; 
        const size_t farX  = nearX ^ (4*sizeof(ssef)), farY  = nearY ^ (4*sizeof(ssef)), farZ  = nearZ ^ (4*sizeof(ssef));
#if defined (__AVX2__)
        const ssef tNearX = msub(load4f((const char*)&lower_x[i]+nearX), rdir.x, org_rdir.x);
        const ssef tNearY = msub(load4f((const char*)&lower_x[i]+nearY), rdir.y, org_rdir.y);
        const ssef tNearZ = msub(load4f((const char*)&lower_x[i]+nearZ), rdir.z, org_rdir.z);
        const ssef tFarX  = msub(load4f((const char*)&lower_x[i]+farX ), rdir.x, org_rdir.x);
        const ssef tFarY  = msub(load4f((const char*)&lower_x[i]+farY ), rdir.y, org_rdir.y);
        const ssef tFarZ  = msub(load4f((const char*)&lower_x[i]+farZ ), rdir.z, org_rdir.z);
#else
        const ssef tNearX = (load4f((const char*)&lower_x[i]+nearX) - org.x) * rdir.x;
        const ssef tNearY = (load4f((const char*)&lower_x[i]+nearY) - org.y) * rdir.y;
        const ssef tNearZ = (load4f((const char*)&lower_x[i]+nearZ) - org.z) * rdir.z;
        const ssef tFarX  = (load4f((const char*)&lower_x[i]+farX ) - org.x) * rdir.x;
        const ssef tFarY  = (load4f((const char*)&lower_x[i]+farY ) - org.y) * rdir.y;
        const ssef tFarZ  = (load4f((const char*)&lower_x[i]+farZ ) - org.z) * rdir.z;
#endif

        if (robust) {
          const float round_down = 1.0f-2.0f*float(ulp);
          const float round_up   = 1.0f+2.0f*float(ulp);
          const ssef tNear = max(tNearX,tNearY,tNearZ,tnear);
          const ssef tFar  = min(tFarX ,tFarY ,tFarZ ,tfar);
          const sseb vmask = round_down*tNear <= round_up*tFar;
          const size_t mask = movemask(vmask);
          return mask;
        }
        
#if defined(__SSE4_1__)
        const ssef tNear = maxi(maxi(tNearX,tNearY),maxi(tNearZ,tnear));
        const ssef tFar  = mini(mini(tFarX ,tFarY ),mini(tFarZ ,tfar ));
        const sseb vmask = cast(tNear) > cast(tFar);
        const size_t mask = movemask(vmask)^0xf;
#else
        const ssef tNear = max(tNearX,tNearY,tNearZ,tnear);
        const ssef tFar  = min(tFarX ,tFarY ,tFarZ ,tfar);
        const sseb vmask = tNear <= tFar;
        const size_t mask = movemask(vmask);
#endif
        return mask;
      }

      template<bool robust>
      __forceinline size_t intersect(size_t nearX, size_t nearY, size_t nearZ,
                                     const sse3f& org, const sse3f& rdir, const sse3f& org_rdir, const ssef& tnear, const ssef& tfar) const
      {
        const size_t mask0 = intersect<robust>(0, nearX, nearY, nearZ, org, rdir, org_rdir, tnear, tfar);
        const size_t mask1 = intersect<robust>(1, nearX, nearY, nearZ, org, rdir, org_rdir, tnear, tfar);
        const size_t mask2 = intersect<robust>(2, nearX, nearY, nearZ, org, rdir, org_rdir, tnear, tfar);
        const size_t mask3 = intersect<robust>(3, nearX, nearY, nearZ, org, rdir, org_rdir, tnear, tfar);
        return mask0 | (mask1 << 4) | (mask2 << 8) | (mask3 << 12);
      }
      
    public:
      ssef lower_x[4];           //!< X dimension of lower bounds of all 4 children.
      ssef upper_x[4];           //!< X dimension of upper bounds of all 4 children.
      ssef lower_y[4];           //!< Y dimension of lower bounds of all 4 children.
      ssef upper_y[4];           //!< Y dimension of upper bounds of all 4 children.
      ssef lower_z[4];           //!< Z dimension of lower bounds of all 4 children.
      ssef upper_z[4];           //!< Z dimension of upper bounds of all 4 children.
    };
    
    struct QuadQuad4x4
    {
    public:
      
      __forceinline QuadQuad4x4(unsigned x, unsigned y, Array2D<Vec3fa>& vertices, unsigned levels, unsigned geomID, unsigned primID)
        : bx(x), by(y), vertices(vertices), levels(levels-1), geomID(geomID), primID(primID) {}

      const BBox3fa leafBounds(size_t x, size_t y)
      {
        BBox3fa bounds = empty;
        x *= 2; y *= 2;
        bounds.extend(vertices(bx+x+0,by+y+0));
        bounds.extend(vertices(bx+x+1,by+y+0));
        bounds.extend(vertices(bx+x+2,by+y+0));
        bounds.extend(vertices(bx+x+0,by+y+1));
        bounds.extend(vertices(bx+x+1,by+y+1));
        bounds.extend(vertices(bx+x+2,by+y+1));
        bounds.extend(vertices(bx+x+0,by+y+2));
        bounds.extend(vertices(bx+x+1,by+y+2));
        bounds.extend(vertices(bx+x+2,by+y+2));
        return bounds;
      }
      
      const BBox3fa build()
      {
        BBox3fa bounds = empty;

        const BBox3fa bounds00_0 = leafBounds(0,0); n.set(0,0,bounds00_0); bounds.extend(bounds00_0);
        const BBox3fa bounds00_1 = leafBounds(1,0); n.set(0,1,bounds00_1); bounds.extend(bounds00_1);
        const BBox3fa bounds00_2 = leafBounds(0,1); n.set(0,2,bounds00_2); bounds.extend(bounds00_2);
        const BBox3fa bounds00_3 = leafBounds(1,1); n.set(0,3,bounds00_3); bounds.extend(bounds00_3);

        const BBox3fa bounds10_0 = leafBounds(2,0); n.set(1,0,bounds10_0); bounds.extend(bounds10_0);
        const BBox3fa bounds10_1 = leafBounds(3,0); n.set(1,1,bounds10_1); bounds.extend(bounds10_1);
        const BBox3fa bounds10_2 = leafBounds(2,1); n.set(1,2,bounds10_2); bounds.extend(bounds10_2);
        const BBox3fa bounds10_3 = leafBounds(3,1); n.set(1,3,bounds10_3); bounds.extend(bounds10_3);

        const BBox3fa bounds01_0 = leafBounds(0,2); n.set(2,0,bounds01_0); bounds.extend(bounds01_0);
        const BBox3fa bounds01_1 = leafBounds(1,2); n.set(2,1,bounds01_1); bounds.extend(bounds01_1);
        const BBox3fa bounds01_2 = leafBounds(0,3); n.set(2,2,bounds01_2); bounds.extend(bounds01_2);
        const BBox3fa bounds01_3 = leafBounds(1,3); n.set(2,3,bounds01_3); bounds.extend(bounds01_3);

        const BBox3fa bounds11_0 = leafBounds(2,2); n.set(3,0,bounds11_0); bounds.extend(bounds11_0);
        const BBox3fa bounds11_1 = leafBounds(3,2); n.set(3,1,bounds11_1); bounds.extend(bounds11_1);
        const BBox3fa bounds11_2 = leafBounds(2,3); n.set(3,2,bounds11_2); bounds.extend(bounds11_2);
        const BBox3fa bounds11_3 = leafBounds(3,3); n.set(3,3,bounds11_3); bounds.extend(bounds11_3);

        return bounds;
      }
      
    public:
      unsigned bx,by;            //!< coordinates of subtree
      Array2D<Vec3fa>& vertices; //!< pointer to vertices
      unsigned levels;           //!< number of stored levels
      unsigned primID;
      unsigned geomID;
      SmallNode4 n;  //!< child nodes
      //SmallNode n00, n10, n01, n11;  //!< child nodes
    };
    
    const std::pair<BBox3fa,BVH4::NodeRef> build(LinearAllocatorPerThread::ThreadAllocator& alloc, unsigned x, unsigned y, unsigned l, unsigned maxDepth)
    {
      if (l == maxDepth) {
        new (&leaves(x,y)) QuadQuad4x4(8*x,8*y,v,3,geomID<true>(),primID<true>());
        const BBox3fa bounds = leaves(x,y).build();
        return std::pair<BBox3fa,BVH4::NodeRef>(bounds,bvh.encodeLeaf(&leaves(x,y),1));
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
        return This->bvh.encodeLeaf(&This->leaves(x,y),1);
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
          new (&leaf) QuadQuad4x4(8*x,8*y,v,3,geomID<true>(),primID<true>());
          prims[y*K+x] = PrimRef(leaf.build(),x,y);
        }
      }
      bool listMode = true; // FIXME
      isa::BVH4BuilderFastGeneric builder(&bvh,&prims[0],K*K,MakeLeaf(this),listMode,4,4,false,0,1,1);
      builder.build(0,0);
    }

  public:
    
    /*! Construction from vertices and IDs. */
    __forceinline SubdivPatchDispl1 (Scene* scene,
                                     const SubdivMesh::HalfEdge* h, 
                                     const Vec3fa* vertices, 
                                     const unsigned int geom, 
                                     const unsigned int prim, 
                                     const unsigned int level,
                                     const bool last)
      : bvh(SubdivPatchDispl1::type,scene,false), K(1), geom(geom), prim(prim | (last << 31)), levels(level) // FIXME: set list mode
    {
      size_t N = 1<<level;
      size_t M = N+1;
      v.init(M,M,Vec3fa(nan));
        
      ring00.init(h,vertices); h = h->next();
      ring10.init(h,vertices); h = h->next();
      ring11.init(h,vertices); h = h->next();
      ring01.init(h,vertices); h = h->next();
      edgeT.init(M,ring00,ring10);
      edgeR.init(M,ring10,ring11);
      edgeB.init(M,ring11,ring01);
      edgeL.init(M,ring01,ring00);
      init();
      
      for (size_t l=0; l<level; l++)
        subdivide();

      /* displace points */
      for (size_t y=0; y<M; y++) {
        for (size_t x=0; x<M; x++) {
          const Vec3fa p = v(x,y);
          v(x,y) += 0.05f*Vec3fa(1.0f+sinf(40.0f*p.z),1.0f+sinf(40.0f*p.x),1.0f+sinf(40.0f*p.y));
        }
      }

      size_t S = 0;
      for (ssize_t i=0; i<level-3; i++) S += (1<<i)*(1<<i);

      leaves.init(N/8,N/8);
#if 0
      LinearAllocatorPerThread::ThreadAllocator nodeAlloc(&bvh.alloc);
      const std::pair<BBox3fa,BVH4::NodeRef> root = build(nodeAlloc,0,0,0,level-3);
      bvh.bounds = root.first;
      bvh.root   = root.second;
#else
      build(level);
#endif
    }

    // FIXME: destruction

    /*! returns required number of primitive blocks for N primitives */
    static __forceinline size_t blocks(size_t N) { return N; }

    /*! return geometry ID */
    template<bool list>
    __forceinline unsigned int geomID() const { 
      return geom; 
    }

    /*! return primitive ID */
    template<bool list>
    __forceinline unsigned int primID() const { 
      if (list) return prim & 0x7FFFFFFF; 
      else      return prim; 
    }

    /*! checks if this is the last primitive in list leaf mode */
    __forceinline int last() const { 
      return prim & 0x80000000; 
    }

    /*! builder interface to fill primitive */
    __forceinline void fill(atomic_set<PrimRefBlock>::block_iterator_unsafe& prims, Scene* scene, const bool list)
    {
      const PrimRef& prim = *prims;
      prims++;

      const unsigned int last   = list && !prims;
      const unsigned int geomID = prim.geomID();
      const unsigned int primID = prim.primID();
      const SubdivMesh* const subdiv_mesh = scene->getSubdivMesh(geomID);
      new (this) SubdivPatchDispl1(scene,
                                   &subdiv_mesh->getHalfEdgeForQuad( primID ),
                                   subdiv_mesh->getVertexPositionPtr(),
                                   geomID,
                                   primID,
                                   SUBDIVISION_LEVEL_DISPL,
                                   last); 
    }

    /*! builder interface to fill primitive */
    __forceinline void fill(const PrimRef* prims, size_t& i, size_t end, Scene* scene, const bool list)
    {
      const PrimRef& prim = prims[i];
      i++;

      const unsigned int last = list && i >= end;
      const unsigned int geomID = prim.geomID();
      const unsigned int primID = prim.primID();
      const SubdivMesh* const subdiv_mesh = scene->getSubdivMesh(geomID);
      new (this) SubdivPatchDispl1(scene,
                                   &subdiv_mesh->getHalfEdgeForQuad( primID ),
                                   subdiv_mesh->getVertexPositionPtr(),
                                   geomID,
                                   primID,
                                   SUBDIVISION_LEVEL_DISPL,
                                   last); 
    }
    
      __forceinline size_t size() const { return K+1; }
      __forceinline Vec3fa& get(size_t x, size_t y) { assert(x<=K); assert(y<=K); return v(x,y); }

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

      friend __forceinline std::ostream &operator<<(std::ostream& out, const SubdivPatchDispl1& patch)
      {
        size_t N = patch.K+1;
        out << "ring00 = " << patch.ring00 << std::endl;
        out << "ring01 = " << patch.ring01 << std::endl;
        out << "ring10 = " << patch.ring10 << std::endl;
        out << "ring11 = " << patch.ring11 << std::endl;
        out << "edgeT  = " << patch.edgeT << std::endl;
        out << "edgeR  = " << patch.edgeR << std::endl;
        out << "edgeB  = " << patch.edgeB << std::endl;
        out << "edgeL  = " << patch.edgeL << std::endl;

        out << std::endl;
        out << "              ";
        for (size_t x=0; x<N; x++) out << std::setw(10) << patch.edgeT.v(x,0).x << " ";
        out << std::endl;
        out << std::endl;
        for (size_t y=0; y<N; y++) {
          out << std::setw(10) << patch.edgeL.v(N-1-y,0).x << "    ";
          for (size_t x=0; x<N; x++) out << std::setw(10) << patch.v(x,y).x << " ";
          out << std::setw(10) << "    " << patch.edgeR.v(y,0).x;
          out << std::endl;
        }
        out << std::endl;
        out << "              ";
        for (size_t x=0; x<N; x++) out << std::setw(10) << patch.edgeB.v(N-1-x,0).x << " ";
        out << std::endl;
        return out;
      } 

  public:
    size_t K;
    size_t levels;
    CatmullClark1Ring ring00,ring01,ring10,ring11;
    CatmullClark1Edge edgeT, edgeB, edgeL, edgeR; 

  public:
    BVH4 bvh;
    Array2D<QuadQuad4x4> leaves;
    Array2D<Vec3fa> v;

  public:
    unsigned geom;
    unsigned prim;
  };
}
