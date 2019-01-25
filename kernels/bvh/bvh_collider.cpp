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
#include "../geometry/triangle_triangle_intersector.h"

namespace embree
{ 
  namespace isa
  {
#define CSTAT(x) 

    size_t parallel_depth_threshold = 6;
    std::atomic<size_t> bvh_collide_traversal_steps(0);
    std::atomic<size_t> bvh_collide_leaf_pairs(0);
    std::atomic<size_t> bvh_collide_leaf_iterations(0);
    std::atomic<size_t> bvh_collide_prim_intersections1(0);
    std::atomic<size_t> bvh_collide_prim_intersections2(0);
    std::atomic<size_t> bvh_collide_prim_intersections3(0);
    std::atomic<size_t> bvh_collide_prim_intersections4(0);
    std::atomic<size_t> bvh_collide_prim_intersections5(0);
    std::atomic<size_t> bvh_collide_prim_intersections(0);

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
    
    template<int N>
    __forceinline size_t overlap(const BBox3fa& box0, const typename BVHN<N>::AlignedNode& node1)
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
      
      return TriangleTriangleIntersector::intersect_triangle_triangle(a0,a1,a2,b0,b1,b2);
    }
    
    template<int N>
    __forceinline void BVHNColliderTriangle4v<N>::processLeaf(const Triangle4v& __restrict__ tris0, const Triangle4v& __restrict__ tris1)
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
            if (intersect_triangle_triangle(this->scene0,geomID0,primID0,this->scene1,geomID1,primID1)) {
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
            if (intersect_triangle_triangle(this->scene0,geomID0,primID0,this->scene1,geomID1,primID1)) {
              CSTAT(bvh_collide_prim_intersections++);
              collisions[num_collisions++] = Collision(geomID0,primID0,geomID1,primID1);
            }
          }
        }
      }
      if (num_collisions)
        this->callback(this->userPtr,(RTCCollision*)&collisions,num_collisions);
    }

    template<int N>
    __forceinline void BVHNColliderTriangle4v<N>::processLeaf(NodeRef node0, NodeRef node1)
    {
      size_t N0; Triangle4v* leaf0 = (Triangle4v*) node0.leaf(N0);
      size_t N1; Triangle4v* leaf1 = (Triangle4v*) node1.leaf(N1);
      for (size_t i=0; i<N0; i++)
        for (size_t j=0; j<N1; j++)
          processLeaf(leaf0[i],leaf1[j]);
    }

    template<int N>
    __forceinline void BVHNColliderUserGeom<N>::processLeaf(NodeRef node0, NodeRef node1)
    {
      Collision collisions[16];
      size_t num_collisions = 0;

      size_t N0; Object* leaf0 = (Object*) node0.leaf(N0);
      size_t N1; Object* leaf1 = (Object*) node1.leaf(N1);
      for (size_t i=0; i<N0; i++) {
        for (size_t j=0; j<N1; j++) {
          const unsigned geomID0 = leaf0[i].geomID;
          const unsigned primID0 = leaf0[i].primID;
          const unsigned geomID1 = leaf1[j].geomID;
          const unsigned primID1 = leaf1[j].primID;
          if (this->scene0 == this->scene1 && geomID0 == geomID1 && primID0 == primID1) continue;
          collisions[num_collisions++] = Collision(geomID0,primID0,geomID1,primID1);
          if (num_collisions == 16) {
            this->callback(this->userPtr,(RTCCollision*)&collisions,num_collisions);
            num_collisions = 0;
          }
        }
      }
      if (num_collisions)
        this->callback(this->userPtr,(RTCCollision*)&collisions,num_collisions);
    }

    template<int N>
    void BVHNCollider<N>::collide_recurse(NodeRef ref0, const BBox3fa& bounds0, NodeRef ref1, const BBox3fa& bounds1, size_t depth)
    {
      CSTAT(bvh_collide_traversal_steps++);
      if (unlikely(ref0.isLeaf())) {
        if (unlikely(ref1.isLeaf())) {
          CSTAT(bvh_collide_leaf_pairs++);
          processLeaf(ref0,ref1);
          return;
        } else goto recurse_node1;
        
      } else {
        if (unlikely(ref1.isLeaf())) {
          goto recurse_node0;
        } else {
          //if (area(bounds0) > area(bounds1)) {
          if (depth < parallel_depth_threshold) { 
            goto recurse_node0; // for some reason this parallel work distribution works well
          }
          else
          {
            if (depth % 2) {
              goto recurse_node0;
            } else {
              goto recurse_node1;
            }
          }
        }
      }

      {
      recurse_node0:
        AlignedNode* node0 = ref0.alignedNode();
        size_t mask = overlap<N>(bounds1,*node0);
        //for (size_t m=mask, i=__bsf(m); m!=0; m=__btc(m,i), i=__bsf(m)) {
        //for (size_t i=0; i<N; i++) {
        if (depth < parallel_depth_threshold) 
        {
          parallel_for(size_t(N), [&] ( size_t i ) {
              if (mask & ( 1 << i)) {
                node0->child(i).prefetch(BVH_FLAG_ALIGNED_NODE);
                collide_recurse(node0->child(i),node0->bounds(i),ref1,bounds1,depth+1);
              }
            });
        } 
        else 
        {
          for (size_t m=mask, i=__bsf(m); m!=0; m=__btc(m,i), i=__bsf(m)) {
            node0->child(i).prefetch(BVH_FLAG_ALIGNED_NODE);
            collide_recurse(node0->child(i),node0->bounds(i),ref1,bounds1,depth+1);
          }
        }
        return;
      }
      
      {
      recurse_node1:
        AlignedNode* node1 = ref1.alignedNode();
        size_t mask = overlap<N>(bounds0,*node1);
        //for (size_t m=mask, i=__bsf(m); m!=0; m=__btc(m,i), i=__bsf(m)) {
        //for (size_t i=0; i<N; i++) {
        if (depth < parallel_depth_threshold) 
        {
          parallel_for(size_t(N), [&] ( size_t i ) {
              if (mask & ( 1 << i)) {
                node1->child(i).prefetch(BVH_FLAG_ALIGNED_NODE);
                collide_recurse(ref0,bounds0,node1->child(i),node1->bounds(i),depth+1);
              }
            });
        }
        else
        {
          for (size_t m=mask, i=__bsf(m); m!=0; m=__btc(m,i), i=__bsf(m)) {
            node1->child(i).prefetch(BVH_FLAG_ALIGNED_NODE);
            collide_recurse(ref0,bounds0,node1->child(i),node1->bounds(i),depth+1);
          }
        }
        return;
      }
    }

     template<int N>
    void BVHNCollider<N>::collide_recurse_entry(NodeRef ref0, const BBox3fa& bounds0, NodeRef ref1, const BBox3fa& bounds1, size_t depth)
    {
      CSTAT(bvh_collide_traversal_steps = 0);
      CSTAT(bvh_collide_leaf_pairs = 0);
      CSTAT(bvh_collide_leaf_iterations = 0);
      CSTAT(bvh_collide_prim_intersections1 = 0);
      CSTAT(bvh_collide_prim_intersections2 = 0);
      CSTAT(bvh_collide_prim_intersections3 = 0);
      CSTAT(bvh_collide_prim_intersections4 = 0);
      CSTAT(bvh_collide_prim_intersections5 = 0);
      CSTAT(bvh_collide_prim_intersections = 0);
      collide_recurse(ref0,bounds0,ref1,bounds1,depth);
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
   

    template<int N>
    void BVHNColliderTriangle4v<N>::collide(BVH* __restrict__ bvh0, BVH* __restrict__ bvh1, RTCCollideFunc callback, void* userPtr)
    { 
      BVHNColliderTriangle4v<N>(bvh0->scene,bvh1->scene,callback,userPtr).
        collide_recurse_entry(bvh0->root,bvh0->bounds.bounds(),bvh1->root,bvh1->bounds.bounds(),0);
    }

    template<int N>
    void BVHNColliderUserGeom<N>::collide(BVH* __restrict__ bvh0, BVH* __restrict__ bvh1, RTCCollideFunc callback, void* userPtr)
    { 
      BVHNColliderUserGeom<N>(bvh0->scene,bvh1->scene,callback,userPtr).
        collide_recurse_entry(bvh0->root,bvh0->bounds.bounds(),bvh1->root,bvh1->bounds.bounds(),0);
    }

#if !defined (__SSE4_1__)
    struct collision_regression_test : public RegressionTest
    {
      collision_regression_test(const char* name) : RegressionTest(name) {
        registerRegressionTest(this);
      }
    
      bool run ()
      {
        bool passed = true;
        passed &= TriangleTriangleIntersector::intersect_triangle_triangle (Vec3fa(-0.008815, 0.041848, -2.49875e-06), Vec3fa(-0.008276, 0.053318, -2.49875e-06), Vec3fa(0.003023, 0.048969, -2.49875e-06),
                                               Vec3fa(0.00245, 0.037612, -2.49875e-06), Vec3fa(0.01434, 0.042634, -2.49875e-06), Vec3fa(0.013499, 0.031309, -2.49875e-06)) == false;
        passed &= TriangleTriangleIntersector::intersect_triangle_triangle (Vec3fa(0,0,0),Vec3fa(1,0,0),Vec3fa(0,1,0), Vec3fa(0,0,0),Vec3fa(1,0,0),Vec3fa(0,1,0)) == true;
        passed &= TriangleTriangleIntersector::intersect_triangle_triangle (Vec3fa(0,0,0),Vec3fa(1,0,0),Vec3fa(0,1,0), Vec3fa(0,0,1),Vec3fa(1,0,1),Vec3fa(0,1,1)) == false;
        passed &= TriangleTriangleIntersector::intersect_triangle_triangle (Vec3fa(0,0,0),Vec3fa(1,0,0),Vec3fa(0,1,0), Vec3fa(0,0,1),Vec3fa(1,0,0),Vec3fa(0,1,0)) == true;
        passed &= TriangleTriangleIntersector::intersect_triangle_triangle (Vec3fa(0,0,0),Vec3fa(1,0,0),Vec3fa(0,1,0), Vec3fa(0,0,0),Vec3fa(1,0,1),Vec3fa(0,1,1)) == true;
        passed &= TriangleTriangleIntersector::intersect_triangle_triangle (Vec3fa(0,0,0),Vec3fa(1,0,0),Vec3fa(0,1,0), Vec3fa(0.1,0.1,0),Vec3fa(1,0,1),Vec3fa(0,1,1)) == true;
        passed &= TriangleTriangleIntersector::intersect_triangle_triangle (Vec3fa(0,0,0),Vec3fa(1,0,0),Vec3fa(0,1,0), Vec3fa(0.1,0.1,-0.1),Vec3fa(1,0,1),Vec3fa(0,1,1)) == true;
        passed &= TriangleTriangleIntersector::intersect_triangle_triangle (Vec3fa(0,0,0),Vec3fa(1,0,0),Vec3fa(0,1,0), Vec3fa(0,0,0),Vec3fa(1,0,0),Vec3fa(0,1,0)) == true;
        passed &= TriangleTriangleIntersector::intersect_triangle_triangle (Vec3fa(0,0,0),Vec3fa(1,0,0),Vec3fa(0,1,0), Vec3fa(0,0,0),Vec3fa(0.5,0,0),Vec3fa(0,0.5,0)) == true;
        passed &= TriangleTriangleIntersector::intersect_triangle_triangle (Vec3fa(0,0,0),Vec3fa(1,0,0),Vec3fa(0,1,0), Vec3fa(0.1,0.1,0),Vec3fa(0.5,0,0),Vec3fa(0,0.5,0)) == true;
        passed &= TriangleTriangleIntersector::intersect_triangle_triangle (Vec3fa(0,0,0),Vec3fa(1,0,0),Vec3fa(0,1,0), Vec3fa(0.1,0.1,0),Vec3fa(0.5,0.1,0),Vec3fa(0.1,0.5,0)) == true;
        passed &= TriangleTriangleIntersector::intersect_triangle_triangle (Vec3fa(0,0,0),Vec3fa(1,0,0),Vec3fa(0,1,0), Vec3fa(0.1,-0.1,0),Vec3fa(0.5,0.1,0),Vec3fa(0.1,0.5,0)) == true;
        passed &= TriangleTriangleIntersector::intersect_triangle_triangle (Vec3fa(0,0,0),Vec3fa(1,0,0),Vec3fa(0,1,0), Vec3fa(-0.1,0.1,0),Vec3fa(0.5,0.1,0),Vec3fa(0.1,0.5,0)) == true;
        passed &= TriangleTriangleIntersector::intersect_triangle_triangle (Vec3fa(0,0,0),Vec3fa(1,0,0),Vec3fa(0,1,0), 
                                               Vec3fa(-1,1,0) + Vec3fa(0,0,0),Vec3fa(-1,1,0) + Vec3fa(0.1,0,0),Vec3fa(-1,1,0) + Vec3fa(0,0.1,0)) == false;
        passed &= TriangleTriangleIntersector::intersect_triangle_triangle (Vec3fa(0,0,0),Vec3fa(1,0,0),Vec3fa(0,1,0), 
                                               Vec3fa( 2,0.5,0) + Vec3fa(0,0,0),Vec3fa( 2,0.5,0) + Vec3fa(0.1,0,0),Vec3fa( 2,0.5,0) + Vec3fa(0,0.1,0)) == false;
        passed &= TriangleTriangleIntersector::intersect_triangle_triangle (Vec3fa(0,0,0),Vec3fa(1,0,0),Vec3fa(0,1,0), 
                                               Vec3fa(0.5,-2.0f,0) + Vec3fa(0,0,0),Vec3fa(0.5f,-2.0f,0) + Vec3fa(0.1,0,0),Vec3fa(0.5f,-2.0f,0) + Vec3fa(0,0.1,0)) == false;
        return passed;
      }
    };

    collision_regression_test collision_regression("collision_regression_test");
#endif

    ////////////////////////////////////////////////////////////////////////////////
    /// Collider Definitions
    ////////////////////////////////////////////////////////////////////////////////

    DEFINE_COLLIDER(BVH4ColliderTriangle4v,BVHNColliderTriangle4v<4>);
    DEFINE_COLLIDER(BVH4ColliderUserGeom,BVHNColliderUserGeom<4>);

#if defined(__AVX__)
    DEFINE_COLLIDER(BVH8ColliderTriangle4v,BVHNColliderTriangle4v<8>);
#endif
  }
}
