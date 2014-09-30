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

    struct Node
    {
      /*! Clears the node. */
      __forceinline void clear() {
        lower_x = lower_y = lower_z = pos_inf; 
        upper_x = upper_y = upper_z = neg_inf;
      }
      
      /*! Sets bounding box of child. */
      __forceinline void set(size_t i, const BBox3fa& bounds) 
      {
        assert(i < N);
        lower_x[i] = bounds.lower.x; lower_y[i] = bounds.lower.y; lower_z[i] = bounds.lower.z;
        upper_x[i] = bounds.upper.x; upper_y[i] = bounds.upper.y; upper_z[i] = bounds.upper.z;
      }
      
      /*! Returns bounds of specified child. */
      __forceinline BBox3fa bounds(size_t i) const 
      {
        assert(i < N);
        const Vec3fa lower(lower_x[i],lower_y[i],lower_z[i]);
        const Vec3fa upper(upper_x[i],upper_y[i],upper_z[i]);
        return BBox3fa(lower,upper);
      }

      /*! intersection with single rays */
      template<bool robust>
      __forceinline size_t intersect(size_t nearX, size_t nearY, size_t nearZ,
                                     const sse3f& org, const sse3f& rdir, const sse3f& org_rdir, const ssef& tnear, const ssef& tfar, 
                                     ssef& dist) const
      {
        const size_t farX  = nearX ^ sizeof(ssef), farY  = nearY ^ sizeof(ssef), farZ  = nearZ ^ sizeof(ssef);
#if defined (__AVX2__)
        const ssef tNearX = msub(load4f((const char*)&lower_x+nearX), rdir.x, org_rdir.x);
        const ssef tNearY = msub(load4f((const char*)&lower_x+nearY), rdir.y, org_rdir.y);
        const ssef tNearZ = msub(load4f((const char*)&lower_x+nearZ), rdir.z, org_rdir.z);
        const ssef tFarX  = msub(load4f((const char*)&lower_x+farX ), rdir.x, org_rdir.x);
        const ssef tFarY  = msub(load4f((const char*)&lower_x+farY ), rdir.y, org_rdir.y);
        const ssef tFarZ  = msub(load4f((const char*)&lower_x+farZ ), rdir.z, org_rdir.z);
#else
        const ssef tNearX = (load4f((const char*)&lower_x+nearX) - org.x) * rdir.x;
        const ssef tNearY = (load4f((const char*)&lower_x+nearY) - org.y) * rdir.y;
        const ssef tNearZ = (load4f((const char*)&lower_x+nearZ) - org.z) * rdir.z;
        const ssef tFarX  = (load4f((const char*)&lower_x+farX ) - org.x) * rdir.x;
        const ssef tFarY  = (load4f((const char*)&lower_x+farY ) - org.y) * rdir.y;
        const ssef tFarZ  = (load4f((const char*)&lower_x+farZ ) - org.z) * rdir.z;
#endif

        if (robust) {
          const float round_down = 1.0f-2.0f*float(ulp);
          const float round_up   = 1.0f+2.0f*float(ulp);
          const ssef tNear = max(tNearX,tNearY,tNearZ,tnear);
          const ssef tFar  = min(tFarX ,tFarY ,tFarZ ,tfar);
          const sseb vmask = round_down*tNear <= round_up*tFar;
          const size_t mask = movemask(vmask);
          dist = tNear;
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
        dist = tNear;
        return mask;
      }
      
    public:
      ssef lower_x;           //!< X dimension of lower bounds of all 4 children.
      ssef upper_x;           //!< X dimension of upper bounds of all 4 children.
      ssef lower_y;           //!< Y dimension of lower bounds of all 4 children.
      ssef upper_y;           //!< Y dimension of upper bounds of all 4 children.
      ssef lower_z;           //!< Z dimension of lower bounds of all 4 children.
      ssef upper_z;           //!< Z dimension of upper bounds of all 4 children.
    };
    
    struct SubTree
    {
    public:
      
      __forceinline SubTree(unsigned x, unsigned y, Array2D<Vec3fa>& vertices, unsigned levels, unsigned geomID, unsigned primID)
        : bx(x), by(y), vertices(vertices), levels(levels-1), geomID(geomID), primID(primID)
      {
        build(nodes,0,0,0);
      }
      
      const BBox3fa build(Node* node, unsigned x, unsigned y, unsigned l)
      {
        if (l == levels) {
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
        const size_t w = 1<<l;
        const BBox3fa b00 = build(node+w*w,2*x+0,2*y+0,l+1); node[y*w+x].set(0,b00);
        const BBox3fa b10 = build(node+w*w,2*x+1,2*y+0,l+1); node[y*w+x].set(1,b10);
        const BBox3fa b01 = build(node+w*w,2*x+0,2*y+1,l+1); node[y*w+x].set(2,b01);
        const BBox3fa b11 = build(node+w*w,2*x+1,2*y+1,l+1); node[y*w+x].set(3,b11);
        return merge(b00,b10,b01,b11);
      }
      
    public:
      unsigned bx,by;            //!< coordinates of subtree
      Array2D<Vec3fa>& vertices; //!< pointer to vertices
      unsigned levels;           //!< number of stored levels
      unsigned primID;
      unsigned geomID;
      Node nodes[];
      //Node n0;                  //!< root node
      //Node n00, n01, n10, n11;  //!< child nodes
    };
    
  public:
    
    /*! Default constructor. */
    __forceinline SubdivPatchDispl1 () {}

    /*! Construction from vertices and IDs. */
    __forceinline SubdivPatchDispl1 (const SubdivMesh::HalfEdge* h, 
                                     const Vec3fa* vertices, 
                                     const unsigned int geom, 
                                     const unsigned int prim, 
                                     const unsigned int level,
                                     const bool last)
      : K(1), geom(geom), prim(prim | (last << 31)), levels(level)
    {
      size_t M = (1<<level)+1;
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
      for (size_t i=0; i<level; i++) S += (1<<i)*(1<<i);
      
      subtree = (SubTree*) malloc(sizeof(SubTree)+S*sizeof(Node));
      new (subtree) SubTree(0,0,v,level,geomID<true>(),primID<true>());
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
      new (this) SubdivPatchDispl1(&subdiv_mesh->getHalfEdgeForQuad( primID ),
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
      new (this) SubdivPatchDispl1(&subdiv_mesh->getHalfEdgeForQuad( primID ),
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
    //const SubdivMesh::HalfEdge* first_half_edge;  //!< pointer to first half edge of this patch
    //const Vec3fa* vertices;                       //!< pointer to vertex array
    //unsigned int subdivision_level;
    //unsigned int flags;
    //unsigned int geom;                            //!< geometry ID of the subdivision mesh this patch belongs to
    //unsigned int prim;                            //!< primitive ID of this subdivision patch

    size_t K;
    CatmullClark1Ring ring00,ring01,ring10,ring11;
    CatmullClark1Edge edgeT, edgeB, edgeL, edgeR; 
    Array2D<Vec3fa> v;
    size_t levels;
    SubTree* subtree;
    unsigned geom;
    unsigned prim;
  };
}
