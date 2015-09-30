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

#include "bvh4_intersector1.h"
#include "bvh4_intersector_node.h"

#include "../geometry/triangle.h"
#include "../geometry/trianglev.h"
#include "../geometry/trianglev_mb.h"
#include "../geometry/trianglei.h"
#include "../geometry/intersector_iterators.h"
#include "../geometry/bezier1v_intersector.h"
#include "../geometry/bezier1i_intersector.h"
#include "../geometry/triangle_intersector_moeller.h"
#include "../geometry/triangle_intersector_pluecker.h"
#include "../geometry/triangle4i_intersector_pluecker.h"
#include "../geometry/subdivpatch1_intersector1.h"
#include "../geometry/subdivpatch1cached_intersector1.h"
#include "../geometry/grid_aos_intersector1.h"
#include "../geometry/object_intersector1.h"

namespace embree
{ 
  namespace isa
  {
    int getISA() { 
      return VerifyMultiTargetLinking::getISA(); 
    }

    template<int N, int types, bool robust>
    __forceinline void traverse_nodes_sort4(BVH4::NodeRef& cur,
                              const Ray& ray,
                              const TravRay<4>& vray,
                              const vfloat4& ray_near,
                              const vfloat4& ray_far,
                              StackItemT<BVH4::NodeRef>* stackBegin,
                              StackItemT<BVH4::NodeRef>*& stackPtr,
                              StackItemT<BVH4::NodeRef>* stackEnd)
    {
      /* downtraversal loop */
      while (true) cont2:
      {
        size_t mask; 
        vfloat4 tNear;
        
        /*! stop if we found a leaf node */
        if (unlikely(cur.isLeaf(types))) break;
        STAT3(normal.trav_nodes,1,1,1);
        
        /* process standard nodes */
        if (likely(cur.isNode(types)))
          mask = intersect_node<N,robust>(cur.node(),vray,ray_near,ray_far,tNear); 
        
        /* process motion blur nodes */
        else if (likely(cur.isNodeMB(types)))
          mask = intersect_node<N>(cur.nodeMB(),vray,ray_near,ray_far,ray.time,tNear); 
        
        /*! process nodes with unaligned bounds */
        else if (unlikely(cur.isUnalignedNode(types)))
          mask = intersect_node<N>(cur.unalignedNode(),vray,ray_near,ray_far,tNear);
        
        /*! process nodes with unaligned bounds and motion blur */
        else if (unlikely(cur.isUnalignedNodeMB(types)))
          mask = intersect_node<N>(cur.unalignedNodeMB(),vray,ray_near,ray_far,ray.time,tNear);
        
        else assert(false);
        
        /*! if no child is hit, pop next node */
        const BVH4::BaseNode* node = cur.baseNode(types);
        if (unlikely(mask == 0)) 
        {
          /*! pop next node */
          while (true)
          {
            if (unlikely(stackPtr == stackBegin)) {
              cur = BVH4::invalidNode; return;
            }
            stackPtr--;
            cur = BVH4::NodeRef(stackPtr->ptr);
            
            /*! if popped node is too far, pop next one */
            if (likely(*(float*)&stackPtr->dist <= ray.tfar))
              goto cont2;
          }
        }
        
        /*! one child is hit, continue with that child */
        size_t r = __bscf(mask);
        if (likely(mask == 0)) {
          cur = node->child(r); cur.prefetch(types);
          assert(cur != BVH4::emptyNode);
          continue;
        }
        
        /*! two children are hit, push far child, and continue with closer child */
        BVH4::NodeRef c0 = node->child(r); c0.prefetch(types); const unsigned int d0 = ((unsigned int*)&tNear)[r];
        r = __bscf(mask);
        BVH4::NodeRef c1 = node->child(r); c1.prefetch(types); const unsigned int d1 = ((unsigned int*)&tNear)[r];
        assert(c0 != BVH4::emptyNode);
        assert(c1 != BVH4::emptyNode);
        if (likely(mask == 0)) {
          assert(stackPtr < stackEnd); 
          if (d0 < d1) { stackPtr->ptr = c1; stackPtr->dist = d1; stackPtr++; cur = c0; continue; }
          else         { stackPtr->ptr = c0; stackPtr->dist = d0; stackPtr++; cur = c1; continue; }
        }
        
        /*! Here starts the slow path for 3 or 4 hit children. We push
         *  all nodes onto the stack to sort them there. */
        assert(stackPtr < stackEnd); 
        stackPtr->ptr = c0; stackPtr->dist = d0; stackPtr++;
        assert(stackPtr < stackEnd); 
        stackPtr->ptr = c1; stackPtr->dist = d1; stackPtr++;
        
        /*! three children are hit, push all onto stack and sort 3 stack items, continue with closest child */
        assert(stackPtr < stackEnd); 
        r = __bscf(mask);
        BVH4::NodeRef c = node->child(r); c.prefetch(types); unsigned int d = ((unsigned int*)&tNear)[r]; stackPtr->ptr = c; stackPtr->dist = d; stackPtr++;
        
        assert(c != BVH4::emptyNode);
        if (likely(mask == 0)) {
          sort(stackPtr[-1],stackPtr[-2],stackPtr[-3]);
          cur = (BVH4::NodeRef) stackPtr[-1].ptr; stackPtr--;
          continue;
        }
        
        /*! four children are hit, push all onto stack and sort 4 stack items, continue with closest child */
        assert(stackPtr < stackEnd); 
        r = __bscf(mask);
        c = node->child(r); c.prefetch(types); d = *(unsigned int*)&tNear[r]; stackPtr->ptr = c; stackPtr->dist = d; stackPtr++;
        assert(c != BVH4::emptyNode);
        sort(stackPtr[-1],stackPtr[-2],stackPtr[-3],stackPtr[-4]);
        cur = (BVH4::NodeRef) stackPtr[-1].ptr; stackPtr--;
      }
    }

    template<int N, int types, bool robust>
    __forceinline void traverse_nodes_sort2(BVH4::NodeRef& cur,
                                            const Ray& ray,
                                            const TravRay<4>& vray,
                                            const vfloat4& ray_near,
                                            const vfloat4& ray_far,
                                            BVH4::NodeRef* stackBegin,
                                            BVH4::NodeRef*& stackPtr,
                                            BVH4::NodeRef* stackEnd)
    {
      
      /* downtraversal loop */
      while (true)
      {
        size_t mask; 
        vfloat4 tNear;
        
        /*! stop if we found a leaf node */
        if (unlikely(cur.isLeaf(types))) break;
        STAT3(shadow.trav_nodes,1,1,1);
        
        /* process standard nodes */
        if (likely(cur.isNode(types)))
          mask = intersect_node<N,robust>(cur.node(),vray,ray_near,ray_far,tNear); 
        
        /* process motion blur nodes */
        else if (likely(cur.isNodeMB(types)))
          mask = intersect_node<N>(cur.nodeMB(),vray.nearX,vray.nearY,vray.nearZ,vray.org,vray.rdir,vray.org_rdir,ray_near,ray_far,ray.time,tNear); 
        
        /*! process nodes with unaligned bounds */
        else if (unlikely(cur.isUnalignedNode(types)))
          mask = intersect_node<N>(cur.unalignedNode(),vray.org,vray.dir,ray_near,ray_far,tNear);
        
        /*! process nodes with unaligned bounds and motion blur */
        else if (unlikely(cur.isUnalignedNodeMB(types)))
          mask = intersect_node<N>(cur.unalignedNodeMB(),vray.org,vray.dir,ray_near,ray_far,ray.time,tNear);
        
        /*! if no child is hit, pop next node */
        const BVH4::BaseNode* node = cur.baseNode(types);
        if (unlikely(mask == 0))
        {
          /*! pop next node */
          if (unlikely(stackPtr == stackBegin)) {
            cur = BVH4::invalidNode; return;
          }
          stackPtr--;
          cur = BVH4::NodeRef(*stackPtr);
          continue;
        }
	
        /*! one child is hit, continue with that child */
        size_t r = __bscf(mask);
        if (likely(mask == 0)) {
          cur = node->child(r); cur.prefetch(types); 
          assert(cur != BVH4::emptyNode);
          continue;
        }
        
        /*! two children are hit, push far child, and continue with closer child */
        BVH4::NodeRef c0 = node->child(r); c0.prefetch(types); const unsigned int d0 = ((unsigned int*)&tNear)[r];
        r = __bscf(mask);
        BVH4::NodeRef c1 = node->child(r); c1.prefetch(types); const unsigned int d1 = ((unsigned int*)&tNear)[r];
        assert(c0 != BVH4::emptyNode);
        assert(c1 != BVH4::emptyNode);
        if (likely(mask == 0)) {
          assert(stackPtr < stackEnd);
          if (d0 < d1) { *stackPtr = c1; stackPtr++; cur = c0; continue; }
          else         { *stackPtr = c0; stackPtr++; cur = c1; continue; }
        }
        assert(stackPtr < stackEnd);
        *stackPtr = c0; stackPtr++;
        assert(stackPtr < stackEnd);
        *stackPtr = c1; stackPtr++;
        
        /*! three children are hit */
        r = __bscf(mask);
        cur = node->child(r); cur.prefetch(types);
        assert(cur != BVH4::emptyNode);
        if (likely(mask == 0)) continue;
        assert(stackPtr < stackEnd);
        *stackPtr = cur; stackPtr++;
        
        /*! four children are hit */
        cur = node->child(3); cur.prefetch(types);
        assert(cur != BVH4::emptyNode);
      }
    }
  
    template<int types, bool robust, typename PrimitiveIntersector>
    void BVH4Intersector1<types,robust,PrimitiveIntersector>::intersect(const BVH4* bvh, Ray& ray)
    {
      enum { N = BVH4::N };
      /*! perform per ray precalculations required by the primitive intersector */
      Precalculations pre(ray,bvh);

      /*! stack state */
      StackItemT<NodeRef> stack[stackSize];           //!< stack of nodes 
      StackItemT<NodeRef>* stackPtr = stack+1;        //!< current stack pointer
      StackItemT<NodeRef>* stackEnd = stack+stackSize;
      stack[0].ptr  = bvh->root;
      stack[0].dist = neg_inf;

      /* filter out invalid rays */
#if defined(RTCORE_IGNORE_INVALID_RAYS)
      if (!ray.valid()) return;
#endif

      /* verify correct input */
      assert(ray.tnear > -FLT_MIN);
      assert(!(types & BVH4::FLAG_NODE_MB) || (ray.time >= 0.0f && ray.time <= 1.0f));

      /*! load the ray into SIMD registers */
      const unsigned int* geomID_to_instID = nullptr;
      TravRay<N> vray(ray.org,ray.dir);
      __aligned(32) char tlray[sizeof(TravRay<N>)];
      new (tlray) TravRay<N>(vray);
      vfloat4 ray_near(ray.tnear);
      vfloat4 ray_far (ray.tfar);

      /* pop loop */
      while (true) 
      {
        /*! pop next node */
        if (unlikely(stackPtr == stack)) break;
        stackPtr--;
        NodeRef cur = NodeRef(stackPtr->ptr);
        
        /*! if popped node is too far, pop next one */
        if (unlikely(*(float*)&stackPtr->dist > ray.tfar))
          continue;
        
        /*! traverse to next leaf node */
#if 0
        while (true)
        {
          switch (cur.type())
          {
          case BVH4::tyNode:
            if (types & BVH4::FLAG_ALIGNED_NODE) traverse_nodes_sort4<N,BVH4::FLAG_ALIGNED_NODE,robust>(cur,ray,vray,ray_near,ray_far,stack,stackPtr,stackEnd);
            break;
          case BVH4::tyNodeMB:
            if (types & BVH4::FLAG_ALIGNED_NODE_MB) traverse_nodes_sort4<N,BVH4::FLAG_ALIGNED_NODE_MB,robust>(cur,ray,vray,ray_near,ray_far,stack,stackPtr,stackEnd);
            break;
          case BVH4::tyUnalignedNode:
            if (types & BVH4::FLAG_UNALIGNED_NODE) traverse_nodes_sort4<N,BVH4::FLAG_UNALIGNED_NODE,robust>(cur,ray,vray,ray_near,ray_far,stack,stackPtr,stackEnd);
            break;
          case BVH4::tyUnalignedNodeMB:
            if (types & BVH4::FLAG_UNALIGNED_NODE_MB) traverse_nodes_sort4<N,BVH4::FLAG_UNALIGNED_NODE_MB,robust>(cur,ray,vray,ray_near,ray_far,stack,stackPtr,stackEnd);
            break;
          default:
            goto break1;
          }
        }
      break1:
#else
        traverse_nodes_sort4<N,types,robust>(cur,ray,vray,ray_near,ray_far,stack,stackPtr,stackEnd);
#endif

        /* return if stack is empty */
        if (unlikely(cur == BVH4::invalidNode)) break;
        
        /* ray transformation support */
        if (types & BVH4::FLAG_TRANSFORM_NODE)
        {
          /*! process transformation nodes */
          if (unlikely(cur.isTransformNode(types))) 
          {
            //STAT3(normal.transform_nodes,1,1,1);
            const BVH4::TransformNode* node = cur.transformNode();
            if (unlikely((ray.mask & node->mask) == 0)) continue;
            const Vec3fa ray_org = xfmPoint (node->world2local,((TravRay<N>&)tlray).org_xyz);
            const Vec3fa ray_dir = xfmVector(node->world2local,((TravRay<N>&)tlray).dir_xyz);
            geomID_to_instID = &node->instID;
            new (&vray) TravRay<N>(ray_org,ray_dir);
            ray.org = ray_org;
            ray.dir = ray_dir;
            stackPtr->ptr = BVH4::popRay; stackPtr->dist = neg_inf; stackPtr++; // FIXME: requires larger stack!
            stackPtr->ptr = node->child;  stackPtr->dist = neg_inf; stackPtr++;
            continue;
          }
          
          /*! restore toplevel ray */
          if (cur == BVH4::popRay) {
            geomID_to_instID = nullptr;
            vray = (TravRay<N>&) tlray; 
            ray.org = ((TravRay<N>&)tlray).org_xyz;
            ray.dir = ((TravRay<N>&)tlray).dir_xyz;
            continue;
          }
        }
        
        /*! this is a leaf node */
	assert(cur != BVH4::emptyNode);
        STAT3(normal.trav_leaves,1,1,1);
        size_t num; Primitive* prim = (Primitive*) cur.leaf(num);
        size_t lazy_node = 0;
        PrimitiveIntersector::intersect(pre,ray,prim,num,bvh->scene,geomID_to_instID,lazy_node);
        ray_far = ray.tfar;

        /*! push lazy node onto stack */
        if (unlikely(lazy_node)) {
          stackPtr->ptr = lazy_node;
          stackPtr->dist = neg_inf;
          stackPtr++;
        }

        // perform stack compaction
        /*StackItemT<NodeRef>* left=stack;
        for (StackItemT<NodeRef>* right=stack; right<stackPtr; right++) 
        {
          if (*(float*)&right->dist >= ray.tfar) continue;
          *left = *right; left++;
        }
        stackPtr = left;*/
      }
      AVX_ZERO_UPPER();
    }
    
    template<int types, bool robust, typename PrimitiveIntersector>
    void BVH4Intersector1<types,robust,PrimitiveIntersector>::occluded(const BVH4* bvh, Ray& ray)
    {
      enum { N = BVH4::N };
      /*! perform per ray precalculations required by the primitive intersector */
      Precalculations pre(ray,bvh);

      /*! stack state */
      NodeRef stack[stackSize];  //!< stack of nodes that still need to get traversed
      NodeRef* stackPtr = stack+1;        //!< current stack pointer
      NodeRef* stackEnd = stack+stackSize;
      stack[0] = bvh->root;
      
      /* filter out invalid rays */
#if defined(RTCORE_IGNORE_INVALID_RAYS)
      if (!ray.valid()) return;
#endif

      /* verify correct input */
      assert(ray.tnear > -FLT_MIN);
      assert(!(types & BVH4::FLAG_NODE_MB) || (ray.time >= 0.0f && ray.time <= 1.0f));

      /*! load the ray into SIMD registers */
      const unsigned int* geomID_to_instID = nullptr;
      TravRay<N> vray(ray.org,ray.dir);
      __aligned(32) char tlray[sizeof(TravRay<N>)];
      new (tlray) TravRay<N>(vray);
      const vfloat4 ray_near(ray.tnear);
      vfloat4 ray_far (ray.tfar);

      /* pop loop */
      while (true) pop:
      {
        /*! pop next node */
        if (unlikely(stackPtr == stack)) break;
        stackPtr--;
        NodeRef cur = (NodeRef) *stackPtr;
        
        /*! traverse to next leaf node */
        traverse_nodes_sort2<N,types,robust>(cur,ray,vray,ray_near,ray_far,stack,stackPtr,stackEnd);
        
        /* return if stack is empty */
        if (unlikely(cur == BVH4::invalidNode)) break;
        
        /* ray transformation support */
        if (types & 0x10000)
        {
          /*! process transformation nodes */
          if (unlikely(cur.isTransformNode(types))) 
          {
            //STAT3(normal.transform_nodes,1,1,1);
            const BVH4::TransformNode* node = cur.transformNode();
            const Vec3fa ray_org = xfmPoint (node->world2local,((TravRay<N>&)tlray).org_xyz);
            const Vec3fa ray_dir = xfmVector(node->world2local,((TravRay<N>&)tlray).dir_xyz);
            geomID_to_instID = &node->instID;
            new (&vray) TravRay<N>(ray_org,ray_dir);
            ray.org = ray_org;
            ray.dir = ray_dir;
            *stackPtr = BVH4::popRay; stackPtr++; // FIXME: requires larger stack!
            *stackPtr = node->child;  stackPtr++;
            goto pop;
          }
          
          /*! restore toplevel ray */
          if (cur == BVH4::popRay) {
            geomID_to_instID = nullptr;
            vray = (TravRay<N>&) tlray; 
            ray.org = ((TravRay<N>&)tlray).org_xyz;
            ray.dir = ((TravRay<N>&)tlray).dir_xyz;
            goto pop;
          }
        }
        /*! this is a leaf node */
        assert(cur != BVH4::emptyNode);
        STAT3(shadow.trav_leaves,1,1,1);
        size_t num; Primitive* prim = (Primitive*) cur.leaf(num);
        size_t lazy_node = 0;
        if (PrimitiveIntersector::occluded(pre,ray,prim,num,bvh->scene,nullptr,lazy_node)) {
          ray.geomID = 0;
          break;
        }
        
        /*! push lazy node onto stack */
        if (unlikely(lazy_node)) {
          *stackPtr = (NodeRef)lazy_node;
          stackPtr++;
        }
      }
      AVX_ZERO_UPPER();
    }

    DEFINE_INTERSECTOR1(BVH4Bezier1vIntersector1,BVH4Intersector1<0x1 COMMA false COMMA ArrayIntersector1<Bezier1vIntersector1> >);
    DEFINE_INTERSECTOR1(BVH4Bezier1iIntersector1,BVH4Intersector1<0x1 COMMA false COMMA ArrayIntersector1<Bezier1iIntersector1> >);
    
    DEFINE_INTERSECTOR1(BVH4Bezier1vIntersector1_OBB,BVH4Intersector1<0x101 COMMA false COMMA ArrayIntersector1<Bezier1vIntersector1> >);
    DEFINE_INTERSECTOR1(BVH4Bezier1iIntersector1_OBB,BVH4Intersector1<0x101 COMMA false COMMA ArrayIntersector1<Bezier1iIntersector1> >);
    DEFINE_INTERSECTOR1(BVH4Bezier1iMBIntersector1_OBB,BVH4Intersector1<0x1010 COMMA false COMMA ArrayIntersector1<Bezier1iIntersector1MB> >);
    DEFINE_INTERSECTOR1(BVH4Triangle4Intersector1Moeller,BVH4Intersector1<0x0001 COMMA false COMMA ArrayIntersector1<TriangleMIntersector1MoellerTrumbore<4 COMMA true> > >);
    DEFINE_INTERSECTOR1(BVH4XfmTriangle4Intersector1Moeller,BVH4Intersector1<0x10001 COMMA false COMMA ArrayIntersector1<TriangleMIntersector1MoellerTrumbore<4 COMMA true> > >);
#if defined(__AVX__)
    DEFINE_INTERSECTOR1(BVH4Triangle8Intersector1Moeller,BVH4Intersector1<0x1 COMMA false COMMA ArrayIntersector1<TriangleMIntersector1MoellerTrumbore<8 COMMA true> > >);
#endif
    DEFINE_INTERSECTOR1(BVH4Triangle4vIntersector1Pluecker,BVH4Intersector1<0x1 COMMA true COMMA ArrayIntersector1<TriangleMvIntersector1Pluecker<4 COMMA true> > >);
    DEFINE_INTERSECTOR1(BVH4Triangle4iIntersector1Pluecker,BVH4Intersector1<0x1 COMMA true COMMA ArrayIntersector1<Triangle4iIntersector1Pluecker<true> > >);

    DEFINE_INTERSECTOR1(BVH4Subdivpatch1Intersector1,BVH4Intersector1<0x1 COMMA true COMMA ArrayIntersector1<SubdivPatch1Intersector1 > >);
    DEFINE_INTERSECTOR1(BVH4Subdivpatch1CachedIntersector1,BVH4Intersector1<0x1 COMMA true COMMA SubdivPatch1CachedIntersector1>);

    DEFINE_INTERSECTOR1(BVH4GridAOSIntersector1,BVH4Intersector1<0x1 COMMA true COMMA GridAOSIntersector1>);

    DEFINE_INTERSECTOR1(BVH4VirtualIntersector1,BVH4Intersector1<0x1 COMMA false COMMA ArrayIntersector1<ObjectIntersector1> >);

    DEFINE_INTERSECTOR1(BVH4Triangle4vMBIntersector1Moeller,BVH4Intersector1<0x10 COMMA false COMMA ArrayIntersector1<TriangleMvMBIntersector1MoellerTrumbore<4 COMMA true> > >);
  }
}
