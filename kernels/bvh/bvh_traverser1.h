// ======================================================================== //
// Copyright 2009-2017 Intel Corporation                                    //
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
#include "../common/stack_item.h"

namespace embree
{
  namespace isa
  {
    /*! BVH regular node traversal for single rays. */
    template<int N, int Nx, int types>
    class BVHNNodeTraverser1Hit;

    /* Specialization for BVH4. */
    template<int Nx, int types>
      class BVHNNodeTraverser1Hit<4,Nx,types>
    {
      typedef BVH4 BVH;
      typedef BVH4::NodeRef NodeRef;
      typedef BVH4::BaseNode BaseNode;

    public:
      /* Traverses a node with at least one hit child. Optimized for finding the closest hit (intersection). */
      static __forceinline void traverseClosestHit(NodeRef& cur,
                                                   size_t mask,
                                                   const vfloat<Nx>& tNear,
                                                   StackItemT<NodeRef>*& stackPtr,
                                                   StackItemT<NodeRef>* stackEnd)
      {
        assert(mask != 0);
        const BaseNode* node = cur.baseNode(types);

        /*! one child is hit, continue with that child */
        size_t r = __bscf(mask);
        cur = node->child(r);
        cur.prefetch(types);
        if (likely(mask == 0)) {
          assert(cur != BVH::emptyNode);
          return;
        }

        /*! two children are hit, push far child, and continue with closer child */
        NodeRef c0 = cur;
        const unsigned int d0 = ((unsigned int*)&tNear)[r];
        r = __bscf(mask);
        NodeRef c1 = node->child(r);
        c1.prefetch(types);
        const unsigned int d1 = ((unsigned int*)&tNear)[r];
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

      /* Traverses a node with at least one hit child. Optimized for finding any hit (occlusion). */
      static __forceinline void traverseAnyHit(NodeRef& cur,
                                               size_t mask,
                                               const vfloat<Nx>& tNear,
                                               NodeRef*& stackPtr,
                                               NodeRef* stackEnd)
      {
        const BaseNode* node = cur.baseNode(types);

        /*! one child is hit, continue with that child */
        size_t r = __bscf(mask);
        cur = node->child(r); 
        cur.prefetch(types);

        /* simpler in sequence traversal order */
        assert(cur != BVH::emptyNode);
        if (likely(mask == 0)) return;
        assert(stackPtr < stackEnd);
        *stackPtr = cur; stackPtr++;

        for (; ;)
        {
          r = __bscf(mask);
          cur = node->child(r); cur.prefetch(types);
          assert(cur != BVH::emptyNode);
          if (likely(mask == 0)) return;
          assert(stackPtr < stackEnd);
          *stackPtr = cur; stackPtr++;
        }
      }
    };


#if defined(__AVX512F__)
      __forceinline static void isort_update(vfloat16 &dist, vllong8 &ptr, const vfloat16 &d, const vllong8 &p)
      {
        const vfloat16 dist_shift = align_shift_right<15>(dist,dist);
        const vllong8  ptr_shift  = align_shift_right<7>(ptr,ptr);
        const vbool16 m_leq = d <= dist;
        const vbool16 m_leq_shift = m_leq << 1;
        dist = select(m_leq,d,dist);
        ptr  = select(vboold8(m_leq),p,ptr);
        dist = select(m_leq_shift,dist_shift,dist);
        ptr  = select(vboold8(m_leq_shift),ptr_shift,ptr);
      }

      __forceinline static void isort_quick_update(const vbool16 &mask, vfloat16 &dist, vllong8 &ptr, const vfloat16 &d, const vllong8 &p)
      {
        dist = select(mask,d,dist);
        ptr  = select(vboold8(mask),p,ptr);
      }

      __forceinline static vllong8 getPushPermuteVector(const vllong8 &input)
      {
        const vllong8 one(1);
        vllong8 counter(zero);
        const vllong8 input1 = align_shift_right<1>(input,input);
        counter = mask_add(input1 < input,counter,counter,one);
        const vllong8 input2 = align_shift_right<2>(input,input);
        counter = mask_add(input2 < input,counter,counter,one);
        const vllong8 input3 = align_shift_right<3>(input,input);
        counter = mask_add(input3 < input,counter,counter,one);
        const vllong8 input4 = align_shift_right<4>(input,input);
        counter = mask_add(input4 < input,counter,counter,one);
        const vllong8 input5 = align_shift_right<5>(input,input);
        counter = mask_add(input5 < input,counter,counter,one);
        const vllong8 input6 = align_shift_right<6>(input,input);
        counter = mask_add(input6 < input,counter,counter,one);
        const vllong8 input7 = align_shift_right<7>(input,input);
        counter = mask_add(input7 < input,counter,counter,one);
        return counter;
      }

        /* const vfloat16 tNear16(tNear); */
        /* const vllong8 children = vllong8::loadu(node->children);             */
        /* const vuint16 ui_dist = select(vboolf16((int)mask),asUInt((__m512)tNear16),vuint16(0x7fffffff)); */
        /* const vuint16 toLongPerm(0,0,1,1,2,2,3,3,4,4,5,5,6,6,7,7); */
        /* const vuint16 ui_dist_perm = permute(ui_dist,toLongPerm); */
        /* const vllong8 dist = (vllong8((__m512i)ui_dist_perm) << 32) | vllong8(step); */
        /* const vllong8 perm = getPermuteVector(dist); */
        /* vllong8 children_perm = permute(children,perm); */
        /* vllong8 dist_perm = permute(dist,perm) >> 32; */
        /* cur = toScalar(children_perm); */

        /* stackPtr += hits - 1; */
        /* for (size_t i=0;i<hits-1;i++) */
        /* { */
        /*   dist_perm = align_shift_right<1>(dist_perm,dist_perm); */
        /*   children_perm  = align_shift_right<1>(children_perm,children_perm);           */
        /*   stackPtr[-1-i].ptr = toScalar(children_perm); */
        /*   stackPtr[-1-i].dist = (unsigned int)(toScalar(dist_perm)); */
        /* } */

#endif

    /* Specialization for BVH8. */
    template<int Nx, int types>
      class BVHNNodeTraverser1Hit<8,Nx,types>
    {
      typedef BVH8 BVH;
      typedef BVH8::NodeRef NodeRef;
      typedef BVH8::BaseNode BaseNode;

    public:
      static __forceinline void traverseClosestHit(NodeRef& cur,
                                                   size_t mask,
                                                   const vfloat<Nx>& tNear,
                                                   StackItemT<NodeRef>*& stackPtr,
                                                   StackItemT<NodeRef>* stackEnd)
      {
        assert(mask != 0);
        const BaseNode* node = cur.baseNode(types);

#if defined(__AVX512F__)

        /*! one child is hit, continue with that child */
        const size_t hits = __popcnt(mask);

        /* vllong8 c = vllong8::loadu(node->children);             */
        /* c = vllong8::compact((__mmask16)mask,c); */
        /* cur = vllong8::extract64bit(c); */

        STAT3(normal.trav_hit_boxes[hits],1,1,1);

#if 0

        vllong8 c = vllong8::loadu(node->children);   
        c = vllong8::compact((__mmask16)mask,c); 
        cur = toScalar(c); 
        cur.prefetch(types);
        if (likely(hits == 1)) {
          assert(cur != BVH::emptyNode);
          return;
        }

        const vfloat16 tNear16(tNear);
        vuint16 ui_dist = asUInt((__m512)tNear16);
        ui_dist = vuint16::compact((__mmask16)mask,ui_dist);
        const vllong8 dist_index = (zeroExtend32Bit(ui_dist) << 32) | vllong8(step);

        vllong8 broadcast_perm(zero);
        vllong8 ptr(0x7fffffff00000007);

        for (size_t i=0;i<hits;i++)
        {
          const vllong8 element = permute(dist_index,broadcast_perm);
          cur = toScalar(element);
          broadcast_perm += 1;
          const vllong8 ptr_shift  = align_shift_right<7>(ptr,ptr);
          const vboold8 m_leq = element <= ptr;
          const vboold8 m_leq_shift = m_leq << 1;
          ptr  = select(m_leq,element,ptr);
          ptr  = select(m_leq_shift,ptr_shift,ptr);
        }
        const vllong8 perm_index = ptr & 0xffffffff;
        vllong8 c_perm = permute(c,perm_index);
        ptr = ptr >> 32;
        cur = toScalar(c_perm);
        cur.prefetch();
        stackPtr += hits - 1;

        for (size_t i=0;i<hits-1;i++)
          assert(ptr[i] <= ptr[i+1]);

        for (size_t i=0;i<hits-1;i++)
        {
          ptr = align_shift_right<1>(ptr,ptr);
          c_perm  = align_shift_right<1>(c_perm,c_perm);          
          NodeRef cur_store = toScalar(c_perm);
          cur_store.prefetch();
          stackPtr[-1-i].ptr  = cur_store;
          stackPtr[-1-i].dist = toScalar(vuint16((__m512i)ptr));
        }
#else
        size_t r = __bscf(mask);
        cur = node->child(r);
        cur.prefetch(types);
        if (likely(mask == 0)) {
          assert(cur != BVH::emptyNode);
          return;
        }

        const vllong8 c0(cur);
        const vfloat16 d0(tNear[r]);
        r = __bscf(mask);
        cur = node->child(r);
        cur.prefetch(types);
        const vllong8 c1(cur);
        const vfloat16 d1(tNear[r]);

        const vboolf16 m_dist = d0 <= d1;
        const vfloat16 d_near = select(m_dist, d0, d1);
        const vfloat16 d_far  = select(m_dist, d1, d0);
        const vllong8 c_near  = select(vboold8(m_dist), c0, c1);
        const vllong8 c_far   = select(vboold8(m_dist), c1, c0);

        if (likely(mask == 0)) {
          cur = toScalar(c_near);
          *(float*)&stackPtr[0].dist = toScalar(d_far);
          stackPtr[0].ptr  = toScalar(c_far);
          stackPtr++;
          return;
        }


        r = __bscf(mask);
        cur = node->child(r);
        cur.prefetch(types);
        const vllong8 c2(cur);
        const vfloat16 d2(tNear[r]);

        const vboolf16 m_dist1 = d_near <= d2;
        //const vfloat16 d_near1  = select(m_dist1, d_near, d2);
        const vfloat16 d_far1   = select(m_dist1, d2, d_near);
        const vllong8  c_near1  = select(vboold8(m_dist1), c_near, c2);
        const vllong8  c_far1   = select(vboold8(m_dist1), c2, c_near);

        const vboolf16 m_dist2 = d_far <= d_far1;
        const vfloat16 d_near2  = select(m_dist2, d_far , d_far1);
        const vfloat16 d_far2   = select(m_dist2, d_far1, d_far);
        const vllong8  c_near2  = select(vboold8(m_dist2), c_far, c_far1);
        const vllong8  c_far2   = select(vboold8(m_dist2), c_far1, c_far);

        if (likely(mask == 0)) {

          cur = toScalar(c_near1);
          *(float*)&stackPtr[0].dist = toScalar(d_far2);
          stackPtr[0].ptr  = toScalar(c_far2);
          *(float*)&stackPtr[1].dist = toScalar(d_near2);
          stackPtr[1].ptr  = toScalar(c_near2);
          stackPtr+=2;
          return;
        }

        vfloat16 dist(pos_inf);
        vllong8 ptr(zero);

#if 1
        isort_update(dist,ptr,d0,c0);
        isort_update(dist,ptr,d1,c1); 
        isort_update(dist,ptr,d2,c2);
#else
        vbool16 m_insert(1);
        isort_update(dist,ptr,d_near1,c_near1);
        m_insert = m_insert << 1;
        isort_update(dist,ptr,d_near2,c_near2);
        m_insert = m_insert << 1;
        isort_update(dist,ptr,d_far2,c_far2);
#endif
        do {
          const size_t r = __bscf(mask);
          cur = node->child(r);
          const vfloat16 new_dist(tNear[r]);
          const vllong8 new_ptr(cur);
          cur.prefetch(types);
          isort_update(dist,ptr,new_dist,new_ptr);
        } while(mask);

        cur = toScalar(ptr);
        stackPtr += hits - 1;
        for (size_t i=0;i<hits-1;i++)
        {
          dist = align_shift_right<1>(dist,dist);
          ptr  = align_shift_right<1>(ptr,ptr);          
          stackPtr[-1-i].ptr  = toScalar(ptr);
          *(float*)&stackPtr[-1-i].dist = toScalar(dist);
        }
#endif

#else
        /*! one child is hit, continue with that child */
        size_t r = __bscf(mask);
        cur = node->child(r);
        cur.prefetch(types);
        if (likely(mask == 0)) {
          assert(cur != BVH::emptyNode);
          return;
        }

        /*! two children are hit, push far child, and continue with closer child */
        NodeRef c0 = cur;
        const unsigned int d0 = ((unsigned int*)&tNear)[r];
        r = __bscf(mask);
        NodeRef c1 = node->child(r);
        c1.prefetch(types);
        const unsigned int d1 = ((unsigned int*)&tNear)[r];

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
#endif
      }

      static __forceinline void traverseAnyHit(NodeRef& cur,
                                               size_t mask,
                                               const vfloat<Nx>& tNear,
                                               NodeRef*& stackPtr,
                                               NodeRef* stackEnd)
      {
        const BaseNode* node = cur.baseNode(types);

        /*! one child is hit, continue with that child */
        size_t r = __bscf(mask);
        cur = node->child(r);
        cur.prefetch(types);

        /* simpler in sequence traversal order */
        assert(cur != BVH::emptyNode);
        if (likely(mask == 0)) return;
        assert(stackPtr < stackEnd);
        *stackPtr = cur; stackPtr++;

        for (; ;)
        {
          r = __bscf(mask);
          cur = node->child(r); cur.prefetch(types);
          assert(cur != BVH::emptyNode);
          if (likely(mask == 0)) return;
          assert(stackPtr < stackEnd);
          *stackPtr = cur; stackPtr++;
        }
      }
    };


    /*! BVH transform node traversal for single rays. */
    template<int N, int Nx, int types, bool transform>
    class BVHNNodeTraverser1Transform;

#define ENABLE_TRANSFORM_CACHE 0

    template<int N, int Nx, int types>
      class BVHNNodeTraverser1Transform<N,Nx,types,true>
    {
      typedef BVHN<N> BVH;
      typedef typename BVH::NodeRef NodeRef;
      typedef typename BVH::TransformNode TransformNode;

    public:
      __forceinline explicit BVHNNodeTraverser1Transform(const TravRay<N,Nx>& vray)
#if ENABLE_TRANSFORM_CACHE
        : cacheSlot(0), cacheTag(-1)
#endif
      {
        new (&tlray) TravRay<N,Nx>(vray);
      }

      /* If a transform node is passed, traverses the node and returns true. */
      __forceinline bool traverseTransform(NodeRef& cur,
                                           Ray& ray,
                                           TravRay<N,Nx>& vray,
                                           size_t& leafType,
                                           IntersectContext* context,
                                           StackItemT<NodeRef>*& stackPtr,
                                           StackItemT<NodeRef>* stackEnd)
      {
        /*! process transformation node */
        if (unlikely(cur.isTransformNode(types)))
        {
          STAT3(normal.trav_xfm_nodes,1,1,1);
          const TransformNode* node = cur.transformNode();
#if defined(EMBREE_RAY_MASK)
          if (unlikely((ray.mask & node->mask) == 0)) return true;
#endif          
          leafType = node->type;
          //context->geomID_to_instID = &node->instID;
          context->instID = ray.instID;
          context->geomID = ray.geomID;
          ray.instID = node->instID;
          ray.geomID = -1;

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
            //if (likely(!node->identity)) 
          {
            const Vec3fa ray_org = xfmPoint (node->world2local,((TravRay<N,Nx>&)tlray).org_xyz);
            const Vec3fa ray_dir = xfmVector(node->world2local,((TravRay<N,Nx>&)tlray).dir_xyz);  
            new (&vray) TravRay<N,Nx>(ray_org,ray_dir);
            ray.org = ray_org;
            ray.dir = ray_dir;
#if ENABLE_TRANSFORM_CACHE
            cacheTag  [cacheSlot&(VSIZEX-1)] = node->xfmID;
            cacheEntry[cacheSlot&(VSIZEX-1)] = vray;
            cacheSlot++;
#endif
          }
          stackPtr->ptr = BVH::popRay; stackPtr->dist = neg_inf; stackPtr++;
          stackPtr->ptr = node->child; stackPtr->dist = neg_inf; stackPtr++;
          return true;
        }

        /*! restore toplevel ray */
        if (cur == BVH::popRay)
        {
          leafType = 0;
          //context->geomID_to_instID = nullptr;
          vray = (TravRay<N,Nx>&) tlray;
          ray.org = ((TravRay<N,Nx>&)tlray).org_xyz;
          ray.dir = ((TravRay<N,Nx>&)tlray).dir_xyz;
          if (ray.geomID == -1) {
            ray.instID = context->instID;
            ray.geomID = context->geomID;
          }
          return true;
        }

        return false;
      }

      /* If a transform node is passed, traverses the node and returns true. */
      __forceinline bool traverseTransform(NodeRef& cur,
                                           Ray& ray,
                                           TravRay<N,Nx>& vray,
                                           size_t& leafType,
                                           IntersectContext* context,
                                           NodeRef*& stackPtr,
                                           NodeRef* stackEnd)
      {
        /*! process transformation node */
        if (unlikely(cur.isTransformNode(types)))
        {
          STAT3(shadow.trav_xfm_nodes,1,1,1);
          const TransformNode* node = cur.transformNode();
#if defined(EMBREE_RAY_MASK)
          if (unlikely((ray.mask & node->mask) == 0)) return true;
#endif
          leafType = node->type;
          //context->geomID_to_instID = &node->instID;
          context->instID = ray.instID;
          context->geomID = ray.geomID;
          ray.instID = node->instID;
          ray.geomID = -1;

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
            //if (likely(!node->identity)) 
          {
            const Vec3fa ray_org = xfmPoint (node->world2local,((TravRay<N,Nx>&)tlray).org_xyz);
            const Vec3fa ray_dir = xfmVector(node->world2local,((TravRay<N,Nx>&)tlray).dir_xyz);
            new (&vray) TravRay<N,Nx>(ray_org,ray_dir);
            ray.org = ray_org;
            ray.dir = ray_dir;
#if ENABLE_TRANSFORM_CACHE
            cacheTag  [cacheSlot&(VSIZEX-1)] = node->xfmID;
            cacheEntry[cacheSlot&(VSIZEX-1)] = vray;
            cacheSlot++;
#endif
          }
          *stackPtr = BVH::popRay; stackPtr++;
          *stackPtr = node->child; stackPtr++;
          return true;
        }

        /*! restore toplevel ray */
        if (cur == BVH::popRay)
        {
          leafType = 0;
          //context->geomID_to_instID = nullptr;
          vray = (TravRay<N,Nx>&) tlray;
          ray.org = ((TravRay<N,Nx>&)tlray).org_xyz;
          ray.dir = ((TravRay<N,Nx>&)tlray).dir_xyz;
          if (ray.geomID == -1) {
            ray.instID = context->instID;
            ray.geomID = context->geomID;
          }
          return true;
        }

        return false;
      }

    private:
      TravRay<N,Nx> tlray;

#if ENABLE_TRANSFORM_CACHE
    private:
      unsigned int cacheSlot;
      vintx cacheTag;
      TravRay<N,Nx> cacheEntry[VSIZEX];
#endif
    };

    template<int N, int Nx, int types>
      class BVHNNodeTraverser1Transform<N,Nx,types,false>
    {
      typedef BVHN<N> BVH;
      typedef typename BVH::NodeRef NodeRef;

    public:
      __forceinline explicit BVHNNodeTraverser1Transform(const TravRay<N,Nx>& vray) {}

      __forceinline bool traverseTransform(NodeRef& cur,
                                           Ray& ray,
                                           TravRay<N,Nx>& vray,
                                           size_t& leafType,
                                           IntersectContext* context,
                                           StackItemT<NodeRef>*& stackPtr,
                                           StackItemT<NodeRef>* stackEnd)
      {
        return false;
      }

      __forceinline bool traverseTransform(NodeRef& cur,
                                           Ray& ray,
                                           TravRay<N,Nx>& vray,
                                           size_t& leafType,
                                           IntersectContext* context,
                                           NodeRef*& stackPtr,
                                           NodeRef* stackEnd)
      {
        return false;
      }
    };

    /*! BVH node traversal for single rays. */
    template<int N, int Nx, int types>
      class BVHNNodeTraverser1 : public BVHNNodeTraverser1Hit<N, Nx, types>, public BVHNNodeTraverser1Transform<N, Nx, types, (bool)(types & BVH_FLAG_TRANSFORM_NODE)>
    {
    public:
      __forceinline explicit BVHNNodeTraverser1(const TravRay<N,Nx>& vray) : BVHNNodeTraverser1Transform<N, Nx, types, (bool)(types & BVH_FLAG_TRANSFORM_NODE)>(vray) {}
    };
  }
}
