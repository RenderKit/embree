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

#pragma once

#include "bvh.h"
#include "bvh_intersector_node.h"
#include "../../common/stack_item.h"

namespace embree
{
  namespace isa
  {
    /*! BVH regular node traversal for single rays. */
    template<int N, int types>
    class BVHNNodeTraverser1Hit;

    /* Specialization for BVH4. */
    template<int types>
    class BVHNNodeTraverser1Hit<4,types>
    {
      typedef BVH4 BVH;
      typedef BVH4::NodeRef NodeRef;
      typedef BVH4::BaseNode BaseNode;

    public:
      static __forceinline void traverseClosestHit(NodeRef& cur,
                                                   size_t mask,
                                                   const vfloat4& tNear,
                                                   StackItemT<NodeRef>*& stackPtr,
                                                   StackItemT<NodeRef>* stackEnd)
      {
        const BaseNode* node = cur.baseNode(types);

        /*! one child is hit, continue with that child */
        size_t r = __bscf(mask);
        if (likely(mask == 0)) {
          cur = node->child(r); cur.prefetch(types);
          assert(cur != BVH::emptyNode);
          return;
        }

        /*! two children are hit, push far child, and continue with closer child */
        NodeRef c0 = node->child(r); c0.prefetch(types); const unsigned int d0 = ((unsigned int*)&tNear)[r];
        r = __bscf(mask);
        NodeRef c1 = node->child(r); c1.prefetch(types); const unsigned int d1 = ((unsigned int*)&tNear)[r];
        assert(c0 != BVH::emptyNode);
        assert(c1 != BVH::emptyNode);
        if (likely(mask == 0)) {
          assert(stackPtr < stackEnd);
          if (d0 < d1) { stackPtr->ptr = c1; stackPtr->dist = d1; stackPtr++; cur = c0; return; }
          else         { stackPtr->ptr = c0; stackPtr->dist = d0; stackPtr++; cur = c1; return; }
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
        NodeRef c = node->child(r); c.prefetch(types); unsigned int d = ((unsigned int*)&tNear)[r]; stackPtr->ptr = c; stackPtr->dist = d; stackPtr++;
        assert(c != BVH::emptyNode);
        if (likely(mask == 0)) {
          sort(stackPtr[-1],stackPtr[-2],stackPtr[-3]);
          cur = (NodeRef) stackPtr[-1].ptr; stackPtr--;
          return;
        }

        /*! four children are hit, push all onto stack and sort 4 stack items, continue with closest child */
        assert(stackPtr < stackEnd);
        r = __bscf(mask);
        c = node->child(r); c.prefetch(types); d = *(unsigned int*)&tNear[r]; stackPtr->ptr = c; stackPtr->dist = d; stackPtr++;
        assert(c != BVH::emptyNode);
        sort(stackPtr[-1],stackPtr[-2],stackPtr[-3],stackPtr[-4]);
        cur = (NodeRef) stackPtr[-1].ptr; stackPtr--;
      }

      static __forceinline void traverseAnyHit(NodeRef& cur,
                                               size_t mask,
                                               const vfloat4& tNear,
                                               NodeRef*& stackPtr,
                                               NodeRef* stackEnd)
      {
        const BaseNode* node = cur.baseNode(types);

        /*! one child is hit, continue with that child */
        size_t r = __bscf(mask);
        if (likely(mask == 0)) {
          cur = node->child(r); cur.prefetch(types);
          assert(cur != BVH::emptyNode);
          return;
        }

        /*! two children are hit, push far child, and continue with closer child */
        NodeRef c0 = node->child(r); c0.prefetch(types); const unsigned int d0 = ((unsigned int*)&tNear)[r];
        r = __bscf(mask);
        NodeRef c1 = node->child(r); c1.prefetch(types); const unsigned int d1 = ((unsigned int*)&tNear)[r];
        assert(c0 != BVH::emptyNode);
        assert(c1 != BVH::emptyNode);
        if (likely(mask == 0)) {
          assert(stackPtr < stackEnd);
          if (d0 < d1) { *stackPtr = c1; stackPtr++; cur = c0; return; }
          else         { *stackPtr = c0; stackPtr++; cur = c1; return; }
        }
        assert(stackPtr < stackEnd);
        *stackPtr = c0; stackPtr++;
        assert(stackPtr < stackEnd);
        *stackPtr = c1; stackPtr++;

        /*! three children are hit */
        r = __bscf(mask);
        cur = node->child(r); cur.prefetch(types);
        assert(cur != BVH::emptyNode);
        if (likely(mask == 0)) return;
        assert(stackPtr < stackEnd);
        *stackPtr = cur; stackPtr++;

        /*! four children are hit */
        cur = node->child(3); cur.prefetch(types);
        assert(cur != BVH::emptyNode);
      }
    };

    /* Specialization for BVH8. */
    template<int types>
    class BVHNNodeTraverser1Hit<8,types>
    {
      typedef BVH8 BVH;
      typedef BVH8::NodeRef NodeRef;
      typedef BVH8::BaseNode BaseNode;

    public:
      static __forceinline void traverseClosestHit(NodeRef& cur,
                                                   size_t mask,
                                                   const vfloat8& tNear,
                                                   StackItemT<NodeRef>*& stackPtr,
                                                   StackItemT<NodeRef>* stackEnd)
      {
        const BaseNode* node = cur.baseNode(types);

        /*! one child is hit, continue with that child */
        size_t r = __bscf(mask);
        if (likely(mask == 0)) {
          cur = node->child(r); cur.prefetch(types);
          assert(cur != BVH::emptyNode);
          return;
        }

        /*! two children are hit, push far child, and continue with closer child */
        NodeRef c0 = node->child(r); c0.prefetch(types); const unsigned int d0 = ((unsigned int*)&tNear)[r];
        r = __bscf(mask);
        NodeRef c1 = node->child(r); c1.prefetch(types); const unsigned int d1 = ((unsigned int*)&tNear)[r];
        assert(c0 != BVH::emptyNode);
        assert(c1 != BVH::emptyNode);
        if (likely(mask == 0)) {
          assert(stackPtr < stackEnd);
          if (d0 < d1) { stackPtr->ptr = c1; stackPtr->dist = d1; stackPtr++; cur = c0; return; }
          else         { stackPtr->ptr = c0; stackPtr->dist = d0; stackPtr++; cur = c1; return; }
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
        NodeRef c = node->child(r); c.prefetch(types); unsigned int d = ((unsigned int*)&tNear)[r]; stackPtr->ptr = c; stackPtr->dist = d; stackPtr++;
        assert(c != BVH::emptyNode);
        if (likely(mask == 0)) {
          sort(stackPtr[-1],stackPtr[-2],stackPtr[-3]);
          cur = (NodeRef) stackPtr[-1].ptr; stackPtr--;
          return;
        }

        /*! four children are hit, push all onto stack and sort 4 stack items, continue with closest child */
        assert(stackPtr < stackEnd);
        r = __bscf(mask);
        c = node->child(r); c.prefetch(types); d = *(unsigned int*)&tNear[r]; stackPtr->ptr = c; stackPtr->dist = d; stackPtr++;
        assert(c != BVH::emptyNode);
        if (likely(mask == 0)) {
          sort(stackPtr[-1],stackPtr[-2],stackPtr[-3],stackPtr[-4]);
          cur = (NodeRef) stackPtr[-1].ptr; stackPtr--;
          return;
        }

        /*! fallback case if more than 4 children are hit */
        StackItemT<NodeRef>* stackFirst = stackPtr-4;
        while (1)
        {
          assert(stackPtr < stackEnd);
          r = __bscf(mask);
          c = node->child(r); c.prefetch(types); d = *(unsigned int*)&tNear[r]; stackPtr->ptr = c; stackPtr->dist = d; stackPtr++;
          assert(c != BVH::emptyNode);
          if (unlikely(mask == 0)) break;
        }
        sort(stackFirst,stackPtr);
        cur = (NodeRef) stackPtr[-1].ptr; stackPtr--;
      }

      static __forceinline void traverseAnyHit(NodeRef& cur,
                                               size_t mask,
                                               const vfloat8& tNear,
                                               NodeRef*& stackPtr,
                                               NodeRef* stackEnd)
      {
        const BaseNode* node = cur.baseNode(types);

        /*! one child is hit, continue with that child */
        size_t r = __bscf(mask);
        if (likely(mask == 0)) {
          cur = node->child(r); cur.prefetch(types);
          assert(cur != BVH::emptyNode);
          return;
        }

        /*! two children are hit, push far child, and continue with closer child */
        NodeRef c0 = node->child(r); c0.prefetch(types); const unsigned int d0 = ((unsigned int*)&tNear)[r];
        r = __bscf(mask);
        NodeRef c1 = node->child(r); c1.prefetch(types); const unsigned int d1 = ((unsigned int*)&tNear)[r];
        assert(c0 != BVH::emptyNode);
        assert(c1 != BVH::emptyNode);
        if (likely(mask == 0)) {
          assert(stackPtr < stackEnd);
          if (d0 < d1) { *stackPtr = c1; stackPtr++; cur = c0; return; }
          else         { *stackPtr = c0; stackPtr++; cur = c1; return; }
        }
        assert(stackPtr < stackEnd);
        *stackPtr = c0; stackPtr++;
        assert(stackPtr < stackEnd);
        *stackPtr = c1; stackPtr++;

        /*! three children are hit */
        r = __bscf(mask);
        cur = node->child(r); cur.prefetch(types);
        assert(cur != BVH::emptyNode);
        if (likely(mask == 0)) return;
        assert(stackPtr < stackEnd);
        *stackPtr = cur; stackPtr++;

        /*! fallback case if more than 3 children are hit */
        while (1)
        {
          assert(stackPtr < stackEnd);
          r = __bscf(mask);
          NodeRef c = node->child(r); c.prefetch(types); *stackPtr = c; stackPtr++;
          assert(c != BVH::emptyNode);
          if (unlikely(mask == 0)) break;
        }
        cur = (NodeRef) stackPtr[-1]; stackPtr--;
      }
    };

    /*! BVH transform node traversal for single rays. */
    template<int N, int types, bool transform>
    class BVHNNodeTraverser1Transform;

#define ENABLE_TRANSFORM_CACHE 0

    template<int N, int types>
    class BVHNNodeTraverser1Transform<N,types,true>
    {
      typedef BVHN<N> BVH;
      typedef typename BVH::NodeRef NodeRef;
      typedef typename BVH::TransformNode TransformNode;

    public:
      __forceinline explicit BVHNNodeTraverser1Transform(const TravRay<N>& vray)
#if ENABLE_TRANSFORM_CACHE
        : cacheSlot(0), cacheTag(-1)
#endif
      {
        new (&tlray) TravRay<N>(vray);
      }

      __forceinline bool traverseTransform(NodeRef& cur,
                                           Ray& ray,
                                           TravRay<N>& vray,
                                           size_t& leafType,
                                           const unsigned int*& geomID_to_instID,
                                           StackItemT<NodeRef>*& stackPtr,
                                           StackItemT<NodeRef>* stackEnd)
      {
        /*! process transformation node */
        if (unlikely(cur.isTransformNode(types)))
        {
          STAT3(normal.trav_xfm_nodes,1,1,1);
          const TransformNode* node = cur.transformNode();
#if defined(RTCORE_RAY_MASK)
          if (unlikely((ray.mask & node->mask) == 0)) return true;
#endif          
          leafType = node->type;
          geomID_to_instID = &node->instID;

#if ENABLE_TRANSFORM_CACHE
          const vboolx xfm_hit = cacheTag == vintx(node->xfmID);
          if (likely(any(xfm_hit))) {
            const int slot = __bsf(movemask(xfm_hit));
            vray = cacheEntry[slot];
            ray.org = vray.org_xyz;
            ray.dir = vray.dir_xyz;
          } 
          else 
#endif
          {
            const Vec3fa ray_org = xfmPoint (node->world2local,((TravRay<N>&)tlray).org_xyz);
            const Vec3fa ray_dir = xfmVector(node->world2local,((TravRay<N>&)tlray).dir_xyz);  
            new (&vray) TravRay<N>(ray_org,ray_dir);
#if ENABLE_TRANSFORM_CACHE
            cacheTag  [cacheSlot&(VSIZEX-1)] = node->xfmID;
            cacheEntry[cacheSlot&(VSIZEX-1)] = vray;
            cacheSlot++;
#endif
            ray.org = ray_org;
            ray.dir = ray_dir;
          }
          stackPtr->ptr = BVH::popRay; stackPtr->dist = neg_inf; stackPtr++; // FIXME: requires larger stack!
          stackPtr->ptr = node->child; stackPtr->dist = neg_inf; stackPtr++;
          return true;
        }

        /*! restore toplevel ray */
        if (cur == BVH::popRay)
        {
          leafType = 0;
          geomID_to_instID = nullptr;
          vray = (TravRay<N>&) tlray;
          ray.org = ((TravRay<N>&)tlray).org_xyz;
          ray.dir = ((TravRay<N>&)tlray).dir_xyz;
          return true;
        }

        return false;
      }

      __forceinline bool traverseTransform(NodeRef& cur,
                                           Ray& ray,
                                           TravRay<N>& vray,
                                           size_t& leafType,
                                           const unsigned int*& geomID_to_instID,
                                           NodeRef*& stackPtr,
                                           NodeRef* stackEnd)
      {
        /*! process transformation node */
        if (unlikely(cur.isTransformNode(types)))
        {
          STAT3(shadow.trav_xfm_nodes,1,1,1);
          const TransformNode* node = cur.transformNode();
#if defined(RTCORE_RAY_MASK)
          if (unlikely((ray.mask & node->mask) == 0)) return true;
#endif
          leafType = node->type;
          geomID_to_instID = &node->instID;

#if ENABLE_TRANSFORM_CACHE
          const vboolx xfm_hit = cacheTag == vintx(node->xfmID);
          if (likely(any(xfm_hit))) {
            const int slot = __bsf(movemask(xfm_hit));
            vray = cacheEntry[slot];
            ray.org = vray.org_xyz;
            ray.dir = vray.dir_xyz;
          } 
          else 
#endif
          {
            const Vec3fa ray_org = xfmPoint (node->world2local,((TravRay<N>&)tlray).org_xyz);
            const Vec3fa ray_dir = xfmVector(node->world2local,((TravRay<N>&)tlray).dir_xyz);
            new (&vray) TravRay<N>(ray_org,ray_dir);
#if ENABLE_TRANSFORM_CACHE
            cacheTag  [cacheSlot&(VSIZEX-1)] = node->xfmID;
            cacheEntry[cacheSlot&(VSIZEX-1)] = vray;
            cacheSlot++;
#endif
            ray.org = ray_org;
            ray.dir = ray_dir;
          }
          *stackPtr = BVH::popRay; stackPtr++; // FIXME: requires larger stack!
          *stackPtr = node->child; stackPtr++;
          return true;
        }

        /*! restore toplevel ray */
        if (cur == BVH::popRay)
        {
          leafType = 0;
          geomID_to_instID = nullptr;
          vray = (TravRay<N>&) tlray;
          ray.org = ((TravRay<N>&)tlray).org_xyz;
          ray.dir = ((TravRay<N>&)tlray).dir_xyz;
          return true;
        }

        return false;
      }

    private:
      //__aligned(32) char tlray[sizeof(TravRay<N>)];
      TravRay<N> tlray;
      unsigned int cacheSlot;
      vintx cacheTag;
      TravRay<N> cacheEntry[VSIZEX];
    };

    template<int N, int types>
    class BVHNNodeTraverser1Transform<N,types,false>
    {
      typedef BVHN<N> BVH;
      typedef typename BVH::NodeRef NodeRef;

    public:
      __forceinline explicit BVHNNodeTraverser1Transform(const TravRay<N>& vray) {}

      __forceinline bool traverseTransform(NodeRef& cur,
                                           Ray& ray,
                                           TravRay<N>& vray,
                                           size_t& leafType,
                                           const unsigned int*& geomID_to_instID,
                                           StackItemT<NodeRef>*& stackPtr,
                                           StackItemT<NodeRef>* stackEnd)
      {
        return false;
      }

      __forceinline bool traverseTransform(NodeRef& cur,
                                           Ray& ray,
                                           TravRay<N>& vray,
                                           size_t& leafType,
                                           const unsigned int*& geomID_to_instID,
                                           NodeRef*& stackPtr,
                                           NodeRef* stackEnd)
      {
        return false;
      }
    };

    /*! BVH node traversal for single rays. */
    template<int N, int types>
    class BVHNNodeTraverser1 : public BVHNNodeTraverser1Hit<N, types>, public BVHNNodeTraverser1Transform<N, types, (bool)(types & BVH_FLAG_TRANSFORM_NODE)>
    {
    public:
      __forceinline explicit BVHNNodeTraverser1(const TravRay<N>& vray) : BVHNNodeTraverser1Transform<N, types, (bool)(types & BVH_FLAG_TRANSFORM_NODE)>(vray) {}
    };
  }
}
