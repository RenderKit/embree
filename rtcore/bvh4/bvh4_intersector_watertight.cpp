// ======================================================================== //
// Copyright 2009-2011 Intel Corporation                                    //
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

#define BACKFACE_CULLING 0
#define DOUBLE_FALLBACK 1
#define WATERTIGHT_TRAVERSAL 1

#include "bvh4_intersector_watertight.h"
#include "../common/stack_item.h"
#include "../triangle/triangle1i_intersector1_watertight.h"

namespace embree
{
  /* rounding constants */
  static const float _one_minus_ulp = float(one)-float(ulp);
  static const float _one_plus_ulp  = float(one)+float(ulp);
  static const ssef one_minus_ulp = ssef(one)-ssef(ulp);
  static const ssef one_plus_ulp  = ssef(one)+ssef(ulp);

  /* conservative up and down rounding */
  __forceinline float up(const float a) { return a>0.0f ? a*_one_plus_ulp  : a*_one_minus_ulp; }
  __forceinline float dn(const float a) { return a>0.0f ? a*_one_minus_ulp : a*_one_plus_ulp;  }

  /*! fast rounding for positive numbers */
  __forceinline float Up(const float a) { return a*_one_plus_ulp; }
  __forceinline float Dn(const float a) { return a*_one_minus_ulp; }

  __forceinline Vec3f Up(const Vec3f& a) { return a*_one_plus_ulp; }
  __forceinline Vec3f Dn(const Vec3f& a) { return a*_one_minus_ulp; }

#if WATERTIGHT_TRAVERSAL

  template<typename TriangleIntersector>
  void BVH4IntersectorWatertight<TriangleIntersector>::intersect(const Ray& ray, Hit& hit) const
  {
    AVX_ZERO_UPPER();
    STAT3(normal.travs,1,1,1);
    
    /*! stack state */
    BVH4::Base* popCur = bvh->root;       //!< pre-popped top node from the stack
    float popDist = neg_inf;              //!< pre-popped distance of top node from the stack
    StackItem stack[1+3*BVH4::maxDepth];  //!< stack of nodes that still need to get traversed
    StackItem* stackPtr = stack+1;        //!< current stack pointer

    /* calculate ray dependent shear */
    const RayShear shear(ray);
    const int kx = shear.kx;
    const int ky = shear.ky;
    const int kz = shear.kz;

    /*! Calculate the offset to the near and far planes for the kx, ky, and kz dimension. */
    Vec3i nearID(0*sizeof(ssef),2*sizeof(ssef),4*sizeof(ssef));
    Vec3i farID (1*sizeof(ssef),3*sizeof(ssef),5*sizeof(ssef));
    size_t nearX = nearID[kx], farX = farID[kx];
    size_t nearY = nearID[ky], farY = farID[ky];
    size_t nearZ = nearID[kz], farZ = farID[kz];
    if (ray.dir[kx] < 0.0f) swap(nearX,farX);
    if (ray.dir[ky] < 0.0f) swap(nearY,farY);
    if (ray.dir[kz] < 0.0f) swap(nearZ,farZ);
    
    const ssef rayNear(ray.near);
    ssef rayFar(ray.far);
    hit.t = min(hit.t,ray.far);

    /* Calculate corrected origin for near and far plane distance
       calculations. Each floating point operation is forced to be
       rounded into the correct direction. */
    const float eps = 5.0f*0.5f*float(ulp);
    const Vec3f lower = Dn(abs(ray.org - bvh->bounds.lower));
    const Vec3f upper = Up(abs(ray.org - bvh->bounds.upper));
    const float max_z = max(lower[kz],upper[kz]);

    const float err_near_x = Up(lower[kx]+max_z);
    const float err_near_y = Up(lower[ky]+max_z);
    ssef org_near_x = up(ray.org[kx] + Up(eps*err_near_x));
    ssef org_near_y = up(ray.org[ky] + Up(eps*err_near_y));
    ssef org_near_z = ray.org[kz];
    
    const float err_far_x = Up(upper[kx]+max_z);
    const float err_far_y = Up(upper[ky]+max_z);
    ssef org_far_x = dn(ray.org[kx] - Up(eps*err_far_x));
    ssef org_far_y = dn(ray.org[ky] - Up(eps*err_far_y));
    ssef org_far_z = ray.org[kz];
    
    if (ray.dir[kx] < 0.0f) swap(org_near_x,org_far_x);
    if (ray.dir[ky] < 0.0f) swap(org_near_y,org_far_y);

    /* Calculate corrected reciprocal direction for near and far plane
       distance calculations. We correct with one additional ulp to
       also correctly round the subtraction inside the traversal, which 
       works as the ray is only allowed to hit geometry in front of it. */
    const ssef rdir_near_x = Dn(Dn(ray.rdir[kx]));
    const ssef rdir_near_y = Dn(Dn(ray.rdir[ky]));
    const ssef rdir_near_z = Dn(Dn(ray.rdir[kz]));

    const ssef rdir_far_x = Up(Up(ray.rdir[kx]));
    const ssef rdir_far_y = Up(Up(ray.rdir[ky]));
    const ssef rdir_far_z = Up(Up(ray.rdir[kz]));

    while (true)
    {
      /*! pop next node */
      if (unlikely(stackPtr == stack)) break;
      stackPtr--;
      BVH4::Base* cur = popCur;
      
      /*! if popped node is too far, pop next one */
      if (unlikely(popDist > hit.t)) {
        popCur  = (BVH4::Base*)stackPtr[-1].ptr;
        popDist = stackPtr[-1].dist;
        continue;
      }

    next:

      /*! we mostly go into the inner node case */
      if (likely(cur->isNode()))
      {
        STAT3(normal.trav_nodes,1,1,1);

        /*! single ray intersection with 4 boxes */
        const Node* node = cur->node();
        const ssef tNearX = (*(ssef*)((const char*)node+nearX) - org_near_x) * rdir_near_x;
        const ssef tNearY = (*(ssef*)((const char*)node+nearY) - org_near_y) * rdir_near_y;
        const ssef tNearZ = (*(ssef*)((const char*)node+nearZ) - org_near_z) * rdir_near_z;
        const ssef tFarX  = (*(ssef*)((const char*)node+farX ) - org_far_x) * rdir_far_x;
        const ssef tFarY  = (*(ssef*)((const char*)node+farY ) - org_far_y) * rdir_far_y;
        const ssef tFarZ  = (*(ssef*)((const char*)node+farZ ) - org_far_z) * rdir_far_z;
        popCur = (BVH4::Base*) stackPtr[-1].ptr;  //!< pre-pop of topmost stack item
        popDist = stackPtr[-1].dist;              //!< pre-pop of distance of topmost stack item
        const ssef tNear = max(tNearX,tNearY,tNearZ,rayNear);
        const ssef tFar  = min(tFarX,tFarY,tFarZ,rayFar);
        size_t _hit = movemask(tNear <= tFar);

        /*! if no child is hit, pop next node */
        if (unlikely(_hit == 0))
          continue;

        /*! one child is hit, continue with that child */
        size_t r = __bsf(_hit); _hit = __btc(_hit,r);
        if (likely(_hit == 0)) {
          cur = node->child[r];
          goto next;
        }

        /*! two children are hit, push far child, and continue with closer child */
        BVH4::Base* c0 = node->child[r]; const float d0 = tNear[r];
        r = __bsf(_hit); _hit = __btc(_hit,r);
        BVH4::Base* c1 = node->child[r]; const float d1 = tNear[r];
        if (likely(_hit == 0)) {
          if (d0 < d1) { stackPtr->ptr = c1; stackPtr->dist = d1; stackPtr++; cur = c0; goto next; }
          else         { stackPtr->ptr = c0; stackPtr->dist = d0; stackPtr++; cur = c1; goto next; }
        }

        /*! Here starts the slow path for 3 or 4 hit children. We push
         *  all nodes onto the stack to sort them there. */
        stackPtr->ptr = c0; stackPtr->dist = d0; stackPtr++;
        stackPtr->ptr = c1; stackPtr->dist = d1; stackPtr++;

        /*! three children are hit, push all onto stack and sort 3 stack items, continue with closest child */
        r = __bsf(_hit); _hit = __btc(_hit,r);
        BVH4::Base* c = node->child[r]; float d = tNear[r]; stackPtr->ptr = c; stackPtr->dist = d; stackPtr++;
        if (likely(_hit == 0)) {
          sort(stackPtr[-1],stackPtr[-2],stackPtr[-3]);
          cur = (BVH4::Base*) stackPtr[-1].ptr; stackPtr--;
          goto next;
        }

        /*! four children are hit, push all onto stack and sort 4 stack items, continue with closest child */
        r = __bsf(_hit); _hit = __btc(_hit,r);
        c = node->child[r]; d = tNear[r]; stackPtr->ptr = c; stackPtr->dist = d; stackPtr++;
        sort(stackPtr[-1],stackPtr[-2],stackPtr[-3],stackPtr[-4]);
        cur = (BVH4::Base*) stackPtr[-1].ptr; stackPtr--;
        goto next;
      }

      /*! this is a leaf node */
      else 
      {
        STAT3(normal.trav_leaves,1,1,1);
        size_t num; Triangle* tri = (Triangle*) cur->leaf(num);
        for (size_t i=0; i<num; i++)
          TriangleIntersector::intersect(ray,shear,hit,tri[i],bvh->vertices);

        popCur = (BVH4::Base*) stackPtr[-1].ptr;    //!< pre-pop of topmost stack item
        popDist = stackPtr[-1].dist;                //!< pre-pop of distance of topmost stack item
        rayFar = hit.t;
      }
    }
    AVX_ZERO_UPPER();
  }

  template<typename TriangleIntersector>
  bool BVH4IntersectorWatertight<TriangleIntersector>::occluded(const Ray& ray) const
  {
    AVX_ZERO_UPPER();
    STAT3(shadow.travs,1,1,1);

    /*! stack state */
    BVH4::Base* stack[1+3*BVH4::maxDepth];  //!< stack of nodes that still need to get traversed
    BVH4::Base** stackPtr = stack+1;        //!< current stack pointer
    stack[0] = bvh->root;                   //!< push first node onto stack

    /* calculate ray dependent shear */
    const RayShear shear(ray);
    const int kx = shear.kx;
    const int ky = shear.ky;
    const int kz = shear.kz;

    /*! Calculate the offset to the near and far planes for the kx, ky, and kz dimension. */
    Vec3i nearID(0*sizeof(ssef),2*sizeof(ssef),4*sizeof(ssef));
    Vec3i farID (1*sizeof(ssef),3*sizeof(ssef),5*sizeof(ssef));
    size_t nearX = nearID[kx], farX = farID[kx];
    size_t nearY = nearID[ky], farY = farID[ky];
    size_t nearZ = nearID[kz], farZ = farID[kz];
    if (ray.dir[kx] < 0.0f) swap(nearX,farX);
    if (ray.dir[ky] < 0.0f) swap(nearY,farY);
    if (ray.dir[kz] < 0.0f) swap(nearZ,farZ);
    
    const ssef rayNear(ray.near);
    ssef rayFar(ray.far);

    /* Calculate corrected origin for near and far plane distance
       calculations. Each floating point operation is forced to be
       rounded into the correct direction. */
    const float eps = 5.0f*0.5f*float(ulp);
    const Vec3f lower = Dn(abs(ray.org - bvh->bounds.lower));
    const Vec3f upper = Up(abs(ray.org - bvh->bounds.upper));
    const float max_z = max(lower[kz],upper[kz]);

    const float err_near_x = Up(lower[kx]+max_z);
    const float err_near_y = Up(lower[ky]+max_z);
    ssef org_near_x = up(ray.org[kx] + Up(eps*err_near_x));
    ssef org_near_y = up(ray.org[ky] + Up(eps*err_near_y));
    ssef org_near_z = ray.org[kz];
    
    const float err_far_x = Up(upper[kx]+max_z);
    const float err_far_y = Up(upper[ky]+max_z);
    ssef org_far_x = dn(ray.org[kx] - Up(eps*err_far_x));
    ssef org_far_y = dn(ray.org[ky] - Up(eps*err_far_y));
    ssef org_far_z = ray.org[kz];
    
    if (ray.dir[kx] < 0.0f) swap(org_near_x,org_far_x);
    if (ray.dir[ky] < 0.0f) swap(org_near_y,org_far_y);

    /* Calculate corrected reciprocal direction for near and far plane
       distance calculations. We correct with one additional ulp to
       also correctly round the subtraction inside the traversal, which 
       works as the ray is only allowed to hit geometry in front of it. */
    const ssef rdir_near_x = Dn(Dn(ray.rdir[kx]));
    const ssef rdir_near_y = Dn(Dn(ray.rdir[ky]));
    const ssef rdir_near_z = Dn(Dn(ray.rdir[kz]));

    const ssef rdir_far_x = Up(Up(ray.rdir[kx]));
    const ssef rdir_far_y = Up(Up(ray.rdir[ky]));
    const ssef rdir_far_z = Up(Up(ray.rdir[kz]));
    
    /*! pop node from stack */
    while (true)
    {
      /* finish when the stack is empty */
      if (unlikely(stackPtr == stack)) break;
      BVH4::Base* cur = *(--stackPtr);

      /*! this is an inner node */
      if (likely(cur->isNode()))
      {
        STAT3(shadow.trav_nodes,1,1,1);
        
        /*! single ray intersection with 4 boxes */
        const Node* node = cur->node();
        const ssef tNearX = (*(ssef*)((const char*)node+nearX) - org_near_x) * rdir_near_x;
        const ssef tNearY = (*(ssef*)((const char*)node+nearY) - org_near_y) * rdir_near_y;
        const ssef tNearZ = (*(ssef*)((const char*)node+nearZ) - org_near_z) * rdir_near_z;
        const ssef tFarX  = (*(ssef*)((const char*)node+farX ) - org_far_x) * rdir_far_x;
        const ssef tFarY  = (*(ssef*)((const char*)node+farY ) - org_far_y) * rdir_far_y;
        const ssef tFarZ  = (*(ssef*)((const char*)node+farZ ) - org_far_z) * rdir_far_z;
        const ssef tNear = max(tNearX,tNearY,tNearZ,rayNear);
        const ssef tFar  = min(tFarX,tFarY,tFarZ,rayFar);
        size_t _hit = movemask(tNear <= tFar);

        /*! push hit nodes onto stack */
        if (likely(_hit == 0)) continue;
        size_t r = __bsf(_hit); _hit = __btc(_hit,r);
        *stackPtr = node->child[r]; stackPtr++;
        if (likely(_hit == 0)) continue;
        r = __bsf(_hit); _hit = __btc(_hit,r);
        *stackPtr = node->child[r]; stackPtr++;
        if (likely(_hit == 0)) continue;
        r = __bsf(_hit); _hit = __btc(_hit,r);
        *stackPtr = node->child[r]; stackPtr++;
        if (likely(_hit == 0)) continue;
        r = __bsf(_hit); _hit = __btc(_hit,r);
        *stackPtr = node->child[r]; stackPtr++;
      }

      /*! this is a leaf node */
      else 
      {
        STAT3(shadow.trav_leaves,1,1,1);
        size_t num; Triangle* tri = (Triangle*) cur->leaf(num);
        for (size_t i=0; i<num; i++)
          if (TriangleIntersector::occluded(ray,shear,tri[i],bvh->vertices)) {
            AVX_ZERO_UPPER();
            return true;
          }
      }
    }
    AVX_ZERO_UPPER();
    return false;
  }

#else

  template<typename TriangleIntersector>
  void BVH4IntersectorWatertight<TriangleIntersector>::intersect(const Ray& ray, Hit& hit) const
  {
    AVX_ZERO_UPPER();
    STAT3(normal.travs,1,1,1);
    
    /*! stack state */
    BVH4::Base* popCur = bvh->root;       //!< pre-popped top node from the stack
    float popDist = neg_inf;              //!< pre-popped distance of top node from the stack
    StackItem stack[1+3*BVH4::maxDepth];  //!< stack of nodes that still need to get traversed
    StackItem* stackPtr = stack+1;        //!< current stack pointer

    /* calculate ray dependent shear */
    const RayShear shear(ray);

    /*! offsets to select the side that becomes the lower or upper bound */
    const size_t nearX = ray.dir.x >= 0 ? 0*sizeof(ssef) : 1*sizeof(ssef);
    const size_t nearY = ray.dir.y >= 0 ? 2*sizeof(ssef) : 3*sizeof(ssef);
    const size_t nearZ = ray.dir.z >= 0 ? 4*sizeof(ssef) : 5*sizeof(ssef);
    const size_t farX  = nearX ^ 16;
    const size_t farY  = nearY ^ 16;
    const size_t farZ  = nearZ ^ 16;

    /*! load the ray into SIMD registers */
    const sse3f norg(-ray.org.x,-ray.org.y,-ray.org.z);
    const sse3f rdir(ray.rdir.x,ray.rdir.y,ray.rdir.z);
    const ssef rayNear(ray.near);
    ssef rayFar(ray.far);
    hit.t = min(hit.t,ray.far);
    
    while (true)
    {
      /*! pop next node */
      if (unlikely(stackPtr == stack)) break;
      stackPtr--;
      BVH4::Base* cur = popCur;
      
      /*! if popped node is too far, pop next one */
      if (unlikely(popDist > hit.t)) {
        popCur  = (BVH4::Base*)stackPtr[-1].ptr;
        popDist = stackPtr[-1].dist;
        continue;
      }

    next:

      /*! we mostly go into the inner node case */
      if (likely(cur->isNode()))
      {
        STAT3(normal.trav_nodes,1,1,1);

        /*! single ray intersection with 4 boxes */
        const Node* node = cur->node();
        const ssef tNearX = (norg.x + *(ssef*)((const char*)node+nearX)) * rdir.x;
        const ssef tNearY = (norg.y + *(ssef*)((const char*)node+nearY)) * rdir.y;
        const ssef tNearZ = (norg.z + *(ssef*)((const char*)node+nearZ)) * rdir.z;
        const ssef tNear = max(tNearX,tNearY,tNearZ,rayNear);
        const ssef tFarX = (norg.x + *(ssef*)((const char*)node+farX)) * rdir.x;
        const ssef tFarY = (norg.y + *(ssef*)((const char*)node+farY)) * rdir.y;
        const ssef tFarZ = (norg.z + *(ssef*)((const char*)node+farZ)) * rdir.z;
        popCur = (BVH4::Base*) stackPtr[-1].ptr;  //!< pre-pop of topmost stack item
        popDist = stackPtr[-1].dist;              //!< pre-pop of distance of topmost stack item
        const ssef tFar = min(tFarX,tFarY,tFarZ,rayFar);
        size_t _hit = movemask(tNear <= tFar);

        /*! if no child is hit, pop next node */
        if (unlikely(_hit == 0))
          continue;

        /*! one child is hit, continue with that child */
        size_t r = __bsf(_hit); _hit = __btc(_hit,r);
        if (likely(_hit == 0)) {
          cur = node->child[r];
          goto next;
        }

        /*! two children are hit, push far child, and continue with closer child */
        BVH4::Base* c0 = node->child[r]; const float d0 = tNear[r];
        r = __bsf(_hit); _hit = __btc(_hit,r);
        BVH4::Base* c1 = node->child[r]; const float d1 = tNear[r];
        if (likely(_hit == 0)) {
          if (d0 < d1) { stackPtr->ptr = c1; stackPtr->dist = d1; stackPtr++; cur = c0; goto next; }
          else         { stackPtr->ptr = c0; stackPtr->dist = d0; stackPtr++; cur = c1; goto next; }
        }

        /*! Here starts the slow path for 3 or 4 hit children. We push
         *  all nodes onto the stack to sort them there. */
        stackPtr->ptr = c0; stackPtr->dist = d0; stackPtr++;
        stackPtr->ptr = c1; stackPtr->dist = d1; stackPtr++;

        /*! three children are hit, push all onto stack and sort 3 stack items, continue with closest child */
        r = __bsf(_hit); _hit = __btc(_hit,r);
        BVH4::Base* c = node->child[r]; float d = tNear[r]; stackPtr->ptr = c; stackPtr->dist = d; stackPtr++;
        if (likely(_hit == 0)) {
          sort(stackPtr[-1],stackPtr[-2],stackPtr[-3]);
          cur = (BVH4::Base*) stackPtr[-1].ptr; stackPtr--;
          goto next;
        }

        /*! four children are hit, push all onto stack and sort 4 stack items, continue with closest child */
        r = __bsf(_hit); _hit = __btc(_hit,r);
        c = node->child[r]; d = tNear[r]; stackPtr->ptr = c; stackPtr->dist = d; stackPtr++;
        sort(stackPtr[-1],stackPtr[-2],stackPtr[-3],stackPtr[-4]);
        cur = (BVH4::Base*) stackPtr[-1].ptr; stackPtr--;
        goto next;
      }

      /*! this is a leaf node */
      else 
      {
        STAT3(normal.trav_leaves,1,1,1);
        size_t num; Triangle* tri = (Triangle*) cur->leaf(num);
        for (size_t i=0; i<num; i++)
          TriangleIntersector::intersect(ray,shear,hit,tri[i],bvh->vertices);

        popCur = (BVH4::Base*) stackPtr[-1].ptr;    //!< pre-pop of topmost stack item
        popDist = stackPtr[-1].dist;                //!< pre-pop of distance of topmost stack item
        rayFar = hit.t;
      }
    }
    AVX_ZERO_UPPER();
  }

  template<typename TriangleIntersector>
  bool BVH4IntersectorWatertight<TriangleIntersector>::occluded(const Ray& ray) const
  {
    AVX_ZERO_UPPER();
    STAT3(shadow.travs,1,1,1);

    /*! stack state */
    BVH4::Base* stack[1+3*BVH4::maxDepth];  //!< stack of nodes that still need to get traversed
    BVH4::Base** stackPtr = stack+1;        //!< current stack pointer
    stack[0] = bvh->root;                   //!< push first node onto stack

    /* calculate ray dependent shear */
    const RayShear shear(ray);

    /*! offsets to select the side that becomes the lower or upper bound */
    const size_t nearX = (ray.dir.x >= 0) ? 0*sizeof(ssef) : 1*sizeof(ssef);
    const size_t nearY = (ray.dir.y >= 0) ? 2*sizeof(ssef) : 3*sizeof(ssef);
    const size_t nearZ = (ray.dir.z >= 0) ? 4*sizeof(ssef) : 5*sizeof(ssef);
    const size_t farX  = nearX ^ 16;
    const size_t farY  = nearY ^ 16;
    const size_t farZ  = nearZ ^ 16;

    /*! load the ray into SIMD registers */
    const sse3f norg(-ray.org.x,-ray.org.y,-ray.org.z);
    const sse3f rdir(ray.rdir.x,ray.rdir.y,ray.rdir.z);
    const ssef rayNear(ray.near);
    const ssef rayFar (ray.far);
    
    /*! pop node from stack */
    while (true)
    {
      /* finish when the stack is empty */
      if (unlikely(stackPtr == stack)) break;
      BVH4::Base* cur = *(--stackPtr);

      /*! this is an inner node */
      if (likely(cur->isNode()))
      {
        STAT3(shadow.trav_nodes,1,1,1);
        
        /*! single ray intersection with 4 boxes */
        const Node* node = cur->node();
        const ssef tNearX = (norg.x + *(ssef*)((const char*)node+nearX)) * rdir.x;
        const ssef tNearY = (norg.y + *(ssef*)((const char*)node+nearY)) * rdir.y;
        const ssef tNearZ = (norg.z + *(ssef*)((const char*)node+nearZ)) * rdir.z;
        const ssef tNear = max(tNearX,tNearY,tNearZ,rayNear);
        const ssef tFarX = (norg.x + *(ssef*)((const char*)node+farX)) * rdir.x;
        const ssef tFarY = (norg.y + *(ssef*)((const char*)node+farY)) * rdir.y;
        const ssef tFarZ = (norg.z + *(ssef*)((const char*)node+farZ)) * rdir.z;
        const ssef tFar = min(tFarX,tFarY,tFarZ,rayFar);
        size_t _hit = movemask(tNear <= tFar);

        /*! push hit nodes onto stack */
        if (likely(_hit == 0)) continue;
        size_t r = __bsf(_hit); _hit = __btc(_hit,r);
        *stackPtr = node->child[r]; stackPtr++;
        if (likely(_hit == 0)) continue;
        r = __bsf(_hit); _hit = __btc(_hit,r);
        *stackPtr = node->child[r]; stackPtr++;
        if (likely(_hit == 0)) continue;
        r = __bsf(_hit); _hit = __btc(_hit,r);
        *stackPtr = node->child[r]; stackPtr++;
        if (likely(_hit == 0)) continue;
        r = __bsf(_hit); _hit = __btc(_hit,r);
        *stackPtr = node->child[r]; stackPtr++;
      }

      /*! this is a leaf node */
      else 
      {
        STAT3(shadow.trav_leaves,1,1,1);
        size_t num; Triangle* tri = (Triangle*) cur->leaf(num);
        for (size_t i=0; i<num; i++)
          if (TriangleIntersector::occluded(ray,shear,tri[i],bvh->vertices)) {
            AVX_ZERO_UPPER();
            return true;
          }
      }
    }
    AVX_ZERO_UPPER();
    return false;
  }
#endif

  /* explicit template instantiation */
  template class BVH4IntersectorWatertight<Triangle1iIntersectorWatertight>;
}
