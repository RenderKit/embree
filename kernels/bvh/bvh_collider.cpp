// ======================================================================== //
// Copyright 2009-2016 Intel Corporation                                    //
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

#include "bvh_collider.h"

namespace embree
{ 
  namespace isa
  {
#define CSTAT(x) x

    size_t parallel_depth_threshold = 5;
    std::atomic<size_t> bvh_collide_traversal_steps(0);
    std::atomic<size_t> bvh_collide_leaf_pairs(0);
    std::atomic<size_t> bvh_collide_leaf_iterations(0);
    std::atomic<size_t> bvh_collide_prim_intersections1(0);
    std::atomic<size_t> bvh_collide_prim_intersections2(0);
    std::atomic<size_t> bvh_collide_prim_intersections3(0);
    std::atomic<size_t> bvh_collide_prim_intersections4(0);
    std::atomic<size_t> bvh_collide_prim_intersections5(0);
    std::atomic<size_t> bvh_collide_prim_intersections(0);

    Scene* scene0 = nullptr; // FIXME: hack
    Scene* scene1 = nullptr;

    struct Collision
    {
      __forceinline Collision() {}

      __forceinline Collision (unsigned geomID0, unsigned primID0, unsigned geomID1, unsigned primID1)
        : geomID0(geomID0), primID0(primID0), geomID1(geomID1), primID1(primID1) {}

      unsigned geomID0;
      unsigned primID0;
      unsigned geomID1;
      unsigned primID1;
    };

    __forceinline float T(float pa0, float pa1, float da0, float da1) {
      return pa0 + (pa1-pa0)*da0/(da0-da1);
    }

    __forceinline bool point_line_side(const Vec2f& p, const Vec2f& a0, const Vec2f& a1) {
      return det(p-a0,p-a1) >= 0.0f;
    }

    __forceinline bool point_inside_triangle(const Vec2f& p, const Vec2f& a, const Vec2f& b, const Vec2f& c) 
    {
      const bool pab = point_line_side(p,a,b); 
      const bool pbc = point_line_side(p,b,c);
      const bool pca = point_line_side(p,c,a);
      return pab == pbc && pab == pca;
    }

    __forceinline bool intersect_line_line(const Vec2f& a0, const Vec2f& a1, const Vec2f& b0, const Vec2f& b1)
    {
      const bool different_sides0 = point_line_side(b0,a0,a1) != point_line_side(b1,a0,a1);
      const bool different_sides1 = point_line_side(a0,b0,b1) != point_line_side(a1,b0,b1);
      return different_sides0 && different_sides1;
    }

    bool intersect_triangle_triangle (const Vec2f& a0, const Vec2f& a1, const Vec2f& a2, 
                                      const Vec2f& b0, const Vec2f& b1, const Vec2f& b2)
    {
      const bool a01_b01 = intersect_line_line(a0,a1,b0,b1); 
      if (a01_b01) return true;
      const bool a01_b12 = intersect_line_line(a0,a1,b1,b2);
      if (a01_b12) return true;
      const bool a01_b20 = intersect_line_line(a0,a1,b2,b0);
      if (a01_b20) return true;
      const bool a12_b01 = intersect_line_line(a1,a2,b0,b1);
      if (a12_b01) return true;
      const bool a12_b12 = intersect_line_line(a1,a2,b1,b2);
      if (a12_b12) return true;
      const bool a12_b20 = intersect_line_line(a1,a2,b2,b0);
      if (a12_b20) return true;
      const bool a20_b01 = intersect_line_line(a2,a0,b0,b1);
      if (a20_b01) return true;
      const bool a20_b12 = intersect_line_line(a2,a0,b1,b2);
      if (a20_b12) return true;
      const bool a20_b20 = intersect_line_line(a2,a0,b2,b0);
      if (a20_b20) return true;

      bool a_in_b = point_inside_triangle(a0,b0,b1,b2) && point_inside_triangle(a1,b0,b1,b2) && point_inside_triangle(a2,b0,b1,b2);
      if (a_in_b) return true;

      bool b_in_a = point_inside_triangle(b0,a0,a1,a2) && point_inside_triangle(b1,a0,a1,a2) && point_inside_triangle(b2,a0,a1,a2);
      if (b_in_a) return true;

      return false;
    }

    bool intersect_triangle_triangle (const Vec3fa& a0, const Vec3fa& a1, const Vec3fa& a2,
                                      const Vec3fa& b0, const Vec3fa& b1, const Vec3fa& b2)
    {
      const float eps = 1E-5f;

      /* calculate triangle planes */
      const Vec3fa Na = cross(a1-a0,a2-a0);
      const float  Ca = dot(Na,a0);
      const Vec3fa Nb = cross(b1-b0,b2-b0);
      const float  Cb = dot(Nb,b0);
      
      /* project triangle A onto plane B */
      const float da0 = dot(Nb,a0)-Cb;
      const float da1 = dot(Nb,a1)-Cb;
      const float da2 = dot(Nb,a2)-Cb;
      if (max(da0,da1,da2) < -eps) return false;
      if (min(da0,da1,da2) > +eps) return false;
      CSTAT(bvh_collide_prim_intersections4++);

      /* project triangle B onto plane A */
      const float db0 = dot(Na,b0)-Ca;
      const float db1 = dot(Na,b1)-Ca;
      const float db2 = dot(Na,b2)-Ca;
      if (max(db0,db1,db2) < -eps) return false;
      if (min(db0,db1,db2) > +eps) return false;
      CSTAT(bvh_collide_prim_intersections5++);

      if ((da0 == 0.0f && da1 == 0.0f && da2 == 0.0f) ||
          (db0 == 0.0f && db1 == 0.0f && db2 == 0.0f))
      {
        const size_t dz = maxDim(Na);
        const size_t dx = (dz+1)%3;
        const size_t dy = (dx+1)%3;
        const Vec2f A0(a0[dx],a0[dy]);
        const Vec2f A1(a1[dx],a1[dy]);
        const Vec2f A2(a2[dx],a2[dy]);
        const Vec2f B0(b0[dx],b0[dy]);
        const Vec2f B1(b1[dx],b1[dy]);
        const Vec2f B2(b2[dx],b2[dy]);
        return intersect_triangle_triangle(A0,A1,A2,B0,B1,B2);
      }

      const Vec3fa D = cross(Na,Nb);
      const float pa0 = dot(D,a0);
      const float pa1 = dot(D,a1);
      const float pa2 = dot(D,a2);
      const float pb0 = dot(D,b0);
      const float pb1 = dot(D,b1);
      const float pb2 = dot(D,b2);

      BBox1f ba = empty;
      if (min(da0,da1) <= 0.0f && max(da0,da1) >= 0.0f && abs(da0-da1) > 0.0f) ba.extend(T(pa0,pa1,da0,da1));
      if (min(da1,da2) <= 0.0f && max(da1,da2) >= 0.0f && abs(da1-da2) > 0.0f) ba.extend(T(pa1,pa2,da1,da2));
      if (min(da2,da0) <= 0.0f && max(da2,da0) >= 0.0f && abs(da2-da0) > 0.0f) ba.extend(T(pa2,pa0,da2,da0));

      BBox1f bb = empty;
      if (min(db0,db1) <= 0.0f && max(db0,db1) >= 0.0f && abs(db0-db1) > 0.0f) bb.extend(T(pb0,pb1,db0,db1));
      if (min(db1,db2) <= 0.0f && max(db1,db2) >= 0.0f && abs(db1-db2) > 0.0f) bb.extend(T(pb1,pb2,db1,db2));
      if (min(db2,db0) <= 0.0f && max(db2,db0) >= 0.0f && abs(db2-db0) > 0.0f) bb.extend(T(pb2,pb0,db2,db0));

      return conjoint(ba,bb);
    }                              

    bool intersect_triangle_triangle (Scene* scene0, unsigned geomID0, unsigned primID0, Scene* scene1, unsigned geomID1, unsigned primID1)
    {
      CSTAT(bvh_collide_prim_intersections1++);
      const TriangleMesh* mesh0 = scene0->getTriangleMesh(geomID0);
      const TriangleMesh* mesh1 = scene1->getTriangleMesh(geomID1);
      const TriangleMesh::Triangle& tri0 = mesh0->triangle(primID0);
      const TriangleMesh::Triangle& tri1 = mesh1->triangle(primID1);

      /* special culling for scene intersection with itself */
      if (scene0 == scene1 && geomID0 == geomID1)
      {
        /* ignore self intersections */
        if (primID0 == primID1)
          return false;
      }
      CSTAT(bvh_collide_prim_intersections2++);
      
      if (scene0 == scene1 && geomID0 == geomID1)
      {
        /* ignore intersection with topological neighbors */
        const vint4 t0(tri0.v[0],tri0.v[1],tri0.v[2],tri0.v[2]);
        if (any(vint4(tri1.v[0]) == t0)) return false;
        if (any(vint4(tri1.v[1]) == t0)) return false;
        if (any(vint4(tri1.v[2]) == t0)) return false;
      }
      CSTAT(bvh_collide_prim_intersections3++);
      
      const Vec3fa a0 = mesh0->vertex(tri0.v[0]);
      const Vec3fa a1 = mesh0->vertex(tri0.v[1]);
      const Vec3fa a2 = mesh0->vertex(tri0.v[2]);
      const Vec3fa b0 = mesh1->vertex(tri1.v[0]);
      const Vec3fa b1 = mesh1->vertex(tri1.v[1]);
      const Vec3fa b2 = mesh1->vertex(tri1.v[2]);

      return intersect_triangle_triangle(a0,a1,a2,b0,b1,b2);
    }

    
    template<int N>
    __forceinline size_t overlap(const BBox3fa& box0, const typename BVHN<N>::Node& node1)
    {
      const vfloat<N> lower_x = max(vfloat<N>(box0.lower.x),node1.lower_x);
      const vfloat<N> lower_y = max(vfloat<N>(box0.lower.y),node1.lower_y);
      const vfloat<N> lower_z = max(vfloat<N>(box0.lower.z),node1.lower_z);
      const vfloat<N> upper_x = min(vfloat<N>(box0.upper.x),node1.upper_x);
      const vfloat<N> upper_y = min(vfloat<N>(box0.upper.y),node1.upper_y);
      const vfloat<N> upper_z = min(vfloat<N>(box0.upper.z),node1.upper_z);
      return movemask((lower_x <= upper_x) & (lower_y <= upper_y) & (lower_z <= upper_z));
    }

    template<int N>
    __forceinline size_t overlap(const BBox3fa& box0, const BBox<Vec3<vfloat<N>>>& box1)
    {
      const vfloat<N> lower_x = max(vfloat<N>(box0.lower.x),box1.lower.x);
      const vfloat<N> lower_y = max(vfloat<N>(box0.lower.y),box1.lower.y);
      const vfloat<N> lower_z = max(vfloat<N>(box0.lower.z),box1.lower.z);
      const vfloat<N> upper_x = min(vfloat<N>(box0.upper.x),box1.upper.x);
      const vfloat<N> upper_y = min(vfloat<N>(box0.upper.y),box1.upper.y);
      const vfloat<N> upper_z = min(vfloat<N>(box0.upper.z),box1.upper.z);
      return movemask((lower_x <= upper_x) & (lower_y <= upper_y) & (lower_z <= upper_z));
    }

    template<int N>
    __forceinline size_t overlap(const BBox<Vec3<vfloat<N>>>& box0, size_t i, const BBox<Vec3<vfloat<N>>>& box1)
    {
      const vfloat<N> lower_x = max(vfloat<N>(box0.lower.x[i]),box1.lower.x);
      const vfloat<N> lower_y = max(vfloat<N>(box0.lower.y[i]),box1.lower.y);
      const vfloat<N> lower_z = max(vfloat<N>(box0.lower.z[i]),box1.lower.z);
      const vfloat<N> upper_x = min(vfloat<N>(box0.upper.x[i]),box1.upper.x);
      const vfloat<N> upper_y = min(vfloat<N>(box0.upper.y[i]),box1.upper.y);
      const vfloat<N> upper_z = min(vfloat<N>(box0.upper.z[i]),box1.upper.z);
      return movemask((lower_x <= upper_x) & (lower_y <= upper_y) & (lower_z <= upper_z));
    }

    template<int N>
    __forceinline void BVHNCollider<N>::processLeaf(const Triangle4v& __restrict__ tris0, const Triangle4v& __restrict__ tris1, RTCCollideFunc callback, void* userPtr)
    {
      Collision collisions[4*4];
      size_t num_collisions = 0;

      size_t size0 = tris0.size();
      size_t size1 = tris1.size();
      BBox<Vec3vf4> bounds0(min(tris0.v0,tris0.v1,tris0.v2),max(tris0.v0,tris0.v1,tris0.v2));
      BBox<Vec3vf4> bounds1(min(tris1.v0,tris1.v1,tris1.v2),max(tris1.v0,tris1.v1,tris1.v2));
     
      if (size0 < size1)
      {
        for (size_t i=0; i<size0; i++) 
        {
          CSTAT(bvh_collide_leaf_iterations++);
          size_t mask = movemask(tris1.valid()) & overlap(bounds0,i,bounds1);
          for (size_t m=mask, j=__bsf(m); m!=0; m=__btc(m,j), j=__bsf(m)) 
          {
            const unsigned geomID0 = tris0.geomID(i);
            const unsigned primID0 = tris0.primID(i);
            const unsigned geomID1 = tris1.geomID(j);
            const unsigned primID1 = tris1.primID(j);
            if (intersect_triangle_triangle(scene0,geomID0,primID0,scene1,geomID1,primID1)) {
              CSTAT(bvh_collide_prim_intersections++);
              collisions[num_collisions++] = Collision(geomID0,primID0,geomID1,primID1);
            }
          }
        }
      } 
      else 
      {
        for (size_t j=0; j<size1; j++) 
        {
          CSTAT(bvh_collide_leaf_iterations++);
          size_t mask = movemask(tris0.valid()) & overlap(bounds1,j,bounds0);
          for (size_t m=mask, i=__bsf(m); m!=0; m=__btc(m,i), i=__bsf(m)) {
            const unsigned geomID0 = tris0.geomID(i);
            const unsigned primID0 = tris0.primID(i);
            const unsigned geomID1 = tris1.geomID(j);
            const unsigned primID1 = tris1.primID(j);
            if (intersect_triangle_triangle(scene0,geomID0,primID0,scene1,geomID1,primID1)) {
              CSTAT(bvh_collide_prim_intersections++);
              collisions[num_collisions++] = Collision(geomID0,primID0,geomID1,primID1);
            }
          }
        }
      }
      if (num_collisions)
        callback(userPtr,(RTCCollision*)&collisions,num_collisions);
    }

    template<int N>
    __forceinline void BVHNCollider<N>::processLeaf(NodeRef node0, NodeRef node1, RTCCollideFunc callback, void* userPtr)
    {
      size_t N0; Triangle4v* leaf0 = (Triangle4v*) node0.leaf(N0);
      size_t N1; Triangle4v* leaf1 = (Triangle4v*) node1.leaf(N1);
      for (size_t i=0; i<N0; i++)
        for (size_t j=0; j<N1; j++)
          processLeaf(leaf0[i],leaf1[j],callback,userPtr);
    }

    template<int N>
    void BVHNCollider<N>::collide_recurse(NodeRef ref0, const BBox3fa& bounds0, NodeRef ref1, const BBox3fa& bounds1, RTCCollideFunc callback, void* userPtr, size_t depth)
    {
      CSTAT(bvh_collide_traversal_steps++);
      if (unlikely(ref0.isLeaf())) {
        if (unlikely(ref1.isLeaf())) {
          CSTAT(bvh_collide_leaf_pairs++);
          processLeaf(ref0,ref1,callback,userPtr);
          return;
        } else goto recurse_node1;
        
      } else {
        if (unlikely(ref1.isLeaf())) {
          goto recurse_node0;
        } else {
          if (area(bounds0) > area(bounds1)) {
            goto recurse_node0;
          } else {
            goto recurse_node1;
          }
        }
      }

      {
      recurse_node0:
        Node* node0 = ref0.node();
        size_t mask = overlap<N>(bounds1,*node0);
        //for (size_t m=mask, i=__bsf(m); m!=0; m=__btc(m,i), i=__bsf(m)) {
        //for (size_t i=0; i<N; i++) {
        if (depth < parallel_depth_threshold) 
        {
          parallel_for(size_t(N), [&] ( size_t i ) {
              if (mask & ( 1 << i)) {
                node0->child(i).prefetch(BVH_FLAG_ALIGNED_NODE);
                collide_recurse(node0->child(i),node0->bounds(i),ref1,bounds1,callback,userPtr,depth+1);
              }
            });
        } 
        else 
        {
          for (size_t m=mask, i=__bsf(m); m!=0; m=__btc(m,i), i=__bsf(m)) {
            node0->child(i).prefetch(BVH_FLAG_ALIGNED_NODE);
            collide_recurse(node0->child(i),node0->bounds(i),ref1,bounds1,callback,userPtr,depth+1);
          }
        }
        return;
      }
      
      {
      recurse_node1:
        Node* node1 = ref1.node();
        size_t mask = overlap<N>(bounds0,*node1);
        //for (size_t m=mask, i=__bsf(m); m!=0; m=__btc(m,i), i=__bsf(m)) {
        //for (size_t i=0; i<N; i++) {
        if (depth < parallel_depth_threshold) 
        {
          parallel_for(size_t(N), [&] ( size_t i ) {
              if (mask & ( 1 << i)) {
                node1->child(i).prefetch(BVH_FLAG_ALIGNED_NODE);
                collide_recurse(ref0,bounds0,node1->child(i),node1->bounds(i),callback,userPtr,depth+1);
              }
            });
        }
        else
        {
          for (size_t m=mask, i=__bsf(m); m!=0; m=__btc(m,i), i=__bsf(m)) {
            node1->child(i).prefetch(BVH_FLAG_ALIGNED_NODE);
            collide_recurse(ref0,bounds0,node1->child(i),node1->bounds(i),callback,userPtr,depth+1);
          }
        }
        return;
      }
    }

    template<int N>
    void BVHNCollider<N>::collide(BVH* __restrict__ bvh0, BVH* __restrict__ bvh1, RTCCollideFunc callback, void* userPtr)
    {
      assert(intersect_triangle_triangle (Vec3fa(0,0,0),Vec3fa(1,0,0),Vec3fa(0,1,0), Vec3fa(0,0,0),Vec3fa(1,0,0),Vec3fa(0,1,0)) == true);
      assert(intersect_triangle_triangle (Vec3fa(0,0,0),Vec3fa(1,0,0),Vec3fa(0,1,0), Vec3fa(0,0,1),Vec3fa(1,0,1),Vec3fa(0,1,1)) == false);
      assert(intersect_triangle_triangle (Vec3fa(0,0,0),Vec3fa(1,0,0),Vec3fa(0,1,0), Vec3fa(0,0,1),Vec3fa(1,0,0),Vec3fa(0,1,0)) == true);
      assert(intersect_triangle_triangle (Vec3fa(0,0,0),Vec3fa(1,0,0),Vec3fa(0,1,0), Vec3fa(0,0,0),Vec3fa(1,0,1),Vec3fa(0,1,1)) == true);
      assert(intersect_triangle_triangle (Vec3fa(0,0,0),Vec3fa(1,0,0),Vec3fa(0,1,0), Vec3fa(0.1,0.1,0),Vec3fa(1,0,1),Vec3fa(0,1,1)) == true);
      assert(intersect_triangle_triangle (Vec3fa(0,0,0),Vec3fa(1,0,0),Vec3fa(0,1,0), Vec3fa(0.1,0.1,-0.1),Vec3fa(1,0,1),Vec3fa(0,1,1)) == true);
      assert(intersect_triangle_triangle (Vec3fa(0,0,0),Vec3fa(1,0,0),Vec3fa(0,1,0), Vec3fa(0,0,0),Vec3fa(1,0,0),Vec3fa(0,1,0)) == true);
      assert(intersect_triangle_triangle (Vec3fa(0,0,0),Vec3fa(1,0,0),Vec3fa(0,1,0), Vec3fa(0,0,0),Vec3fa(0.5,0,0),Vec3fa(0,0.5,0)) == true);
      assert(intersect_triangle_triangle (Vec3fa(0,0,0),Vec3fa(1,0,0),Vec3fa(0,1,0), Vec3fa(0.1,0.1,0),Vec3fa(0.5,0,0),Vec3fa(0,0.5,0)) == true);
      assert(intersect_triangle_triangle (Vec3fa(0,0,0),Vec3fa(1,0,0),Vec3fa(0,1,0), Vec3fa(0.1,0.1,0),Vec3fa(0.5,0.1,0),Vec3fa(0.1,0.5,0)) == true);
      assert(intersect_triangle_triangle (Vec3fa(0,0,0),Vec3fa(1,0,0),Vec3fa(0,1,0), Vec3fa(0.1,-0.1,0),Vec3fa(0.5,0.1,0),Vec3fa(0.1,0.5,0)) == true);
      assert(intersect_triangle_triangle (Vec3fa(0,0,0),Vec3fa(1,0,0),Vec3fa(0,1,0), Vec3fa(-0.1,0.1,0),Vec3fa(0.5,0.1,0),Vec3fa(0.1,0.5,0)) == true);
      assert(intersect_triangle_triangle (Vec3fa(0,0,0),Vec3fa(1,0,0),Vec3fa(0,1,0), Vec3fa(-1,1,0) + Vec3fa(0,0,0),Vec3fa(-1,1,0) + Vec3fa(0.1,0,0),Vec3fa(-1,1,0) + Vec3fa(0,0.1,0)) == false);
      assert(intersect_triangle_triangle (Vec3fa(0,0,0),Vec3fa(1,0,0),Vec3fa(0,1,0), Vec3fa( 2,0.5,0) + Vec3fa(0,0,0),Vec3fa( 2,0.5,0) + Vec3fa(0.1,0,0),Vec3fa( 2,0.5,0) + Vec3fa(0,0.1,0)) == false);
      assert(intersect_triangle_triangle (Vec3fa(0,0,0),Vec3fa(1,0,0),Vec3fa(0,1,0), Vec3fa(0.5,-2.0f,0) + Vec3fa(0,0,0),Vec3fa(0.5f,-2.0f,0) + Vec3fa(0.1,0,0),Vec3fa(0.5f,-2.0f,0) + Vec3fa(0,0.1,0)) == false);
      
      scene0 = bvh0->scene;
      scene1 = bvh1->scene;
      collide_recurse(bvh0->root,bvh0->bounds,bvh1->root,bvh1->bounds,callback,userPtr,0);
      CSTAT(PRINT(bvh_collide_traversal_steps));
      CSTAT(PRINT(bvh_collide_leaf_pairs));
      CSTAT(PRINT(bvh_collide_leaf_iterations));
      CSTAT(PRINT(bvh_collide_prim_intersections1));
      CSTAT(PRINT(bvh_collide_prim_intersections2));
      CSTAT(PRINT(bvh_collide_prim_intersections3));
      CSTAT(PRINT(bvh_collide_prim_intersections4));
      CSTAT(PRINT(bvh_collide_prim_intersections5));
      CSTAT(PRINT(bvh_collide_prim_intersections));
      AVX_ZERO_UPPER();
    }

    ////////////////////////////////////////////////////////////////////////////////
    /// Collider Definitions
    ////////////////////////////////////////////////////////////////////////////////

    DEFINE_COLLIDER(BVH4Collider,BVHNCollider<4>);

#if defined(__AVX__)
    DEFINE_COLLIDER(BVH8Collider,BVHNCollider<8>);
#endif
  }
}
