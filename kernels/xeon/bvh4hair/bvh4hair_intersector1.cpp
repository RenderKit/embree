// ======================================================================== //
// Copyright 2009-2013 Intel Corporation                                    //
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

#include "bvh4hair_intersector1.h"
#include "geometry/bezier1i_intersector1.h"

namespace embree
{ 
  namespace isa
  {
#if 0
    __forceinline bool BVH4HairIntersector1::intersectBox(const BBox3fa& aabb, const Ray& ray, const Vec3fa& org_rdir, const Vec3fa& rdir, float& tNear, float& tFar)
    {
#if defined (__AVX2__)
      const Vec3fa tLowerXYZ = msub(aabb.lower,rdir,org_rdir);
      const Vec3fa tUpperXYZ = msub(aabb.upper,rdir,org_rdir);
#else
      const Vec3fa tLowerXYZ = (aabb.lower - ray.org) * rdir;
      const Vec3fa tUpperXYZ = (aabb.upper - ray.org) * rdir;
#endif

#if defined(__SSE4_1__)
      const Vec3fa tNearXYZ = mini(tLowerXYZ,tUpperXYZ);
      const Vec3fa tFarXYZ  = maxi(tLowerXYZ,tUpperXYZ);
      tNear = max(max(tNearXYZ.x,tNearXYZ.y),max(tNearXYZ.z,tNear)); // FIXME: using mini here makes things slower
      tFar  = min(min(tFarXYZ .x,tFarXYZ .y),min(tFarXYZ .z,tFar ));
      return tNear <= tFar;
#else
      const Vec3fa tNearXYZ = min(tLowerXYZ,tUpperXYZ);
      const Vec3fa tFarXYZ  = max(tLowerXYZ,tUpperXYZ);
      tNear = max(reduce_max(tNearXYZ),tNear);
      tFar  = min(reduce_min(tFarXYZ ),tFar);
      return tNear <= tFar;
#endif
    }
#endif

    __forceinline size_t BVH4HairIntersector1::intersectBox(const AffineSpaceSOA4& naabb, const Ray& ray, ssef& tNear, ssef& tFar)
    {
      const sse3f dir = xfmVector(naabb,sse3f(ray.dir));
      const sse3f rdir = rcp(dir);
      const sse3f org = xfmPoint (naabb,sse3f(ray.org));
      const sse3f tLowerXYZ = - org * rdir;     // (Vec3fa(zero) - org) * rdir;
      const sse3f tUpperXYZ = rdir + tLowerXYZ; // (Vec3fa(one ) - org) * rdir;

//#if defined(__SSE4_1__)
//      const sse3f tNearXYZ = mini(tLowerXYZ,tUpperXYZ);
//      const sse3f tFarXYZ  = maxi(tLowerXYZ,tUpperXYZ);
//#else
      const sse3f tNearXYZ = min(tLowerXYZ,tUpperXYZ);
      const sse3f tFarXYZ  = max(tLowerXYZ,tUpperXYZ);
//#endif

//#if defined(__SSE4_1__)
//        tNear = maxi(maxi(tNearX,tNearY),maxi(tNearZ,tNear));
//        tFar  = mini(mini(tFarX ,tFarY ),mini(tFarZ ,tFar));
//        const sseb vmask = cast(tNear) > cast(tFar);
//        size_t mask = movemask(vmask)^0xf;
//#else
        tNear = max(tNearXYZ.x,tNearXYZ.y,tNearXYZ.z,tNear);
        tFar  = min(tFarXYZ.x ,tFarXYZ.y ,tFarXYZ.z ,tFar);
        const sseb vmask = tNear <= tFar;
        size_t mask = movemask(vmask);
//#endif
      return mask;
    }

    __forceinline void BVH4HairIntersector1::intersectBezier(const LinearSpace3fa& ray_space, Ray& ray, const Bezier1& bezier)
    {
      /* load bezier curve control points */
      STAT3(normal.trav_prims,1,1,1);
      const Vec3fa& v0 = bezier.p0;
      const Vec3fa& v1 = bezier.p1;
      const Vec3fa& v2 = bezier.p2;
      const Vec3fa& v3 = bezier.p3;

      /* transform control points into ray space */
      Vec3fa w0 = xfmVector(ray_space,v0-ray.org); w0.w = v0.w;
      Vec3fa w1 = xfmVector(ray_space,v1-ray.org); w1.w = v1.w;
      Vec3fa w2 = xfmVector(ray_space,v2-ray.org); w2.w = v2.w;
      Vec3fa w3 = xfmVector(ray_space,v3-ray.org); w3.w = v3.w;
      BezierCurve3D curve2D(w0,w1,w2,w3,0.0f,1.0f,4);

      /* subdivide 3 levels at once */ 
      const avx4f p0 = curve2D.eval(coeff0[0],coeff0[1],coeff0[2],coeff0[3]);
      const avx4f p1 = curve2D.eval(coeff1[0],coeff1[1],coeff1[2],coeff1[3]);

      /* approximative intersection with cone */
      const avx4f v = p1-p0;
      const avx4f w = -p0;
      const avxf d0 = w.x*v.x + w.y*v.y;
      const avxf d1 = v.x*v.x + v.y*v.y;
      const avxf u = clamp(d0*rcp(d1),avxf(zero),avxf(one));
      const avx4f p = p0 + u*v;
      const avxf t = p.z;
      const avxf d2 = p.x*p.x + p.y*p.y; 
      const avxf r = p.w; //max(p.w,ray.org.w+ray.dir.w*t);
      const avxf r2 = r*r;
      avxb valid = d2 <= r2 & avxf(ray.tnear) < t & t < avxf(ray.tfar);
    retry:
      if (unlikely(none(valid))) return;
      const float one_over_8 = 1.0f/8.0f;
      size_t i = select_min(valid,t);

      /* update hit information */
      const float uu = (float(i)+u[i])*one_over_8;
      BezierCurve3D curve3D(v0,v1,v2,v3,0.0f,1.0f,0);
      Vec3fa P,T; curve3D.eval(uu,P,T);
      if (T == Vec3fa(zero)) { valid[i] = 0; goto retry; } // ignore denormalized curves
      ray.u = bezier.t0 + uu*bezier.dt;
      ray.v = 0.0f;
      ray.tfar = t[i];
      ray.Ng = T;
      ray.geomID = bezier.geomID;
      ray.primID = bezier.primID;
    }

    void BVH4HairIntersector1::intersect(const BVH4Hair* bvh, Ray& ray)
    {
      /*! perform per ray precalculations required by the primitive intersector */
      //const Precalculations pre(ray);
      const LinearSpace3fa pre(rcp(frame(ray.dir)));

      /*! stack state */
      StackItem stack[stackSize];  //!< stack of nodes 
      StackItem* stackPtr = stack+1;        //!< current stack pointer
      StackItem* stackEnd = stack+stackSize;
      stack[0].ref = bvh->root;
      stack[0].tNear = ray.tnear;
      stack[0].tFar = ray.tfar;
      
      /*! offsets to select the side that becomes the lower or upper bound */
      const size_t nearX = ray.dir.x >= 0.0f ? 0*sizeof(ssef) : 1*sizeof(ssef);
      const size_t nearY = ray.dir.y >= 0.0f ? 2*sizeof(ssef) : 3*sizeof(ssef);
      const size_t nearZ = ray.dir.z >= 0.0f ? 4*sizeof(ssef) : 5*sizeof(ssef);
      
      /*! load the ray into SIMD registers */
      const sse3f norg(-ray.org.x,-ray.org.y,-ray.org.z);
      const Vec3fa ray_rdir = rcp_safe(ray.dir);
      const sse3f rdir(ray_rdir.x,ray_rdir.y,ray_rdir.z);
      const Vec3fa ray_org_rdir = ray.org*ray_rdir;
      const sse3f org_rdir(ray_org_rdir.x,ray_org_rdir.y,ray_org_rdir.z);

      /* pop loop */
      while (true) pop:
      {
        /*! pop next node */
        if (unlikely(stackPtr == stack)) break;
        stackPtr--;
        NodeRef cur = NodeRef(stackPtr->ref);
        ssef tNear = stackPtr->tNear;
        ssef tFar = min(stackPtr->tFar,ray.tfar);
        
        /*! if popped node is too far, pop next one */
        if (unlikely(_mm_cvtss_f32(tNear) > _mm_cvtss_f32(tFar)))
          continue;
        
        /* downtraversal loop */
        while (true)
        {
          /*! stop if we found a leaf */
          size_t mask;
          if (unlikely(cur.isLeaf())) break;
          STAT3(normal.trav_nodes,1,1,1);
          
          /*! process nodes with aligned bounds */
          if (likely(cur.isAlignedNode()))
          {
            /*! single ray intersection with 4 boxes */
            const AlignedNode* node = cur.alignedNode();
            const size_t farX  = nearX ^ 16, farY  = nearY ^ 16, farZ  = nearZ ^ 16;
#if defined (__AVX2__)
            const ssef tNearX = msub(load4f((const char*)&node->lower_x+nearX), rdir.x, org_rdir.x);
            const ssef tNearY = msub(load4f((const char*)&node->lower_x+nearY), rdir.y, org_rdir.y);
            const ssef tNearZ = msub(load4f((const char*)&node->lower_x+nearZ), rdir.z, org_rdir.z);
            const ssef tFarX  = msub(load4f((const char*)&node->lower_x+farX ), rdir.x, org_rdir.x);
            const ssef tFarY  = msub(load4f((const char*)&node->lower_x+farY ), rdir.y, org_rdir.y);
            const ssef tFarZ  = msub(load4f((const char*)&node->lower_x+farZ ), rdir.z, org_rdir.z);
#else
            const ssef tNearX = (norg.x + load4f((const char*)&node->lower_x+nearX)) * rdir.x;
            const ssef tNearY = (norg.y + load4f((const char*)&node->lower_x+nearY)) * rdir.y;
            const ssef tNearZ = (norg.z + load4f((const char*)&node->lower_x+nearZ)) * rdir.z;
            const ssef tFarX  = (norg.x + load4f((const char*)&node->lower_x+farX )) * rdir.x;
            const ssef tFarY  = (norg.y + load4f((const char*)&node->lower_x+farY )) * rdir.y;
            const ssef tFarZ  = (norg.z + load4f((const char*)&node->lower_x+farZ )) * rdir.z;
#endif
            
#if defined(__SSE4_1__)
            tNear = maxi(maxi(tNearX,tNearY),maxi(tNearZ,tNear));
            tFar  = mini(mini(tFarX ,tFarY ),mini(tFarZ ,tFar));
            const sseb vmask = cast(tNear) > cast(tFar);
            mask = movemask(vmask)^0xf;
#else
            tNear = max(tNearX,tNearY,tNearZ,tNear);
            tFar  = min(tFarX ,tFarY ,tFarZ ,tNear);
            const sseb vmask = tNear <= tFar;
            mask = movemask(vmask);
#endif
          }

          /*! process nodes with unaligned bounds */
          else {
            const UnalignedNode* node = cur.unalignedNode();
            mask = intersectBox(node->naabb,ray,tNear,tFar);
          }

          /*! if no child is hit, pop next node */
          const Node* node = cur.node();
          if (unlikely(mask == 0))
            goto pop;
          
          /*! one child is hit, continue with that child */
          size_t r = __bscf(mask);
          if (likely(mask == 0)) {
            cur = node->child(r); tNear = tNear[r]; tFar = tFar[r];
            assert(cur != BVH4::emptyNode);
            continue;
          }
          
          /*! two children are hit, push far child, and continue with closer child */
          NodeRef c0 = node->child(r); const float n0 = tNear[r]; const float f0 = tFar[r]; 
          r = __bscf(mask);
          NodeRef c1 = node->child(r); const float n1 = tNear[r]; const float f1 = tFar[r];
          assert(c0 != BVH4::emptyNode);
          assert(c1 != BVH4::emptyNode);
          if (likely(mask == 0)) {
            assert(stackPtr < stackEnd); 
            if (n0 < n1) { stackPtr->ref = c1; stackPtr->tNear = n1; stackPtr->tFar = f1; stackPtr++; cur = c0; tNear = n0; tFar = f0; continue; }
            else         { stackPtr->ref = c0; stackPtr->tNear = n0; stackPtr->tFar = f0; stackPtr++; cur = c1; tNear = n1; tFar = f1; continue; }
          }
          
          /*! Here starts the slow path for 3 or 4 hit children. We push
           *  all nodes onto the stack to sort them there. */
          assert(stackPtr < stackEnd); 
          stackPtr->ref = c0; stackPtr->tNear = n0; stackPtr->tFar = f0; stackPtr++;
          assert(stackPtr < stackEnd); 
          stackPtr->ref = c1; stackPtr->tNear = n1; stackPtr->tFar = f1; stackPtr++;
          
          /*! three children are hit, push all onto stack and sort 3 stack items, continue with closest child */
          assert(stackPtr < stackEnd); 
          r = __bscf(mask);
          NodeRef c = node->child(r); float n2 = tNear[r]; float f2 = tFar[r]; stackPtr->ref = c; stackPtr->tNear = n2; stackPtr->tFar = f2; stackPtr++;
          assert(c != BVH4::emptyNode);
          if (likely(mask == 0)) {
            sort(stackPtr[-1],stackPtr[-2],stackPtr[-3]);
            cur = (NodeRef) stackPtr[-1].ref; tNear = stackPtr[-1].tNear; tFar = stackPtr[-1].tFar; stackPtr--;
            continue;
          }
          
          /*! four children are hit, push all onto stack and sort 4 stack items, continue with closest child */
          assert(stackPtr < stackEnd); 
          r = __bscf(mask);
          c = node->child(r); float n3 = tNear[r]; float f3 = tFar[r]; stackPtr->ref = c; stackPtr->tNear = n3; stackPtr->tFar = f3; stackPtr++;
          assert(c != BVH4::emptyNode);
          sort(stackPtr[-1],stackPtr[-2],stackPtr[-3],stackPtr[-4]);
          cur = (NodeRef) stackPtr[-1].ref; tNear = stackPtr[-1].tNear; tFar = stackPtr[-1].tFar; stackPtr--;
        }
        
        /*! this is a leaf node */
        STAT3(normal.trav_leaves,1,1,1);
        size_t num; Bezier1* prim = (Bezier1*) cur.leaf(num);
        for (size_t i=0; i<num; i++) intersectBezier(pre,ray,prim[i]);
      }
      AVX_ZERO_UPPER();
    }

    __forceinline bool BVH4HairIntersector1::occludedBezier(const LinearSpace3fa& ray_space, Ray& ray, const Bezier1& bezier)
    {
      /* load bezier curve control points */
      STAT3(normal.trav_prims,1,1,1);
      const Vec3fa& v0 = bezier.p0;
      const Vec3fa& v1 = bezier.p1;
      const Vec3fa& v2 = bezier.p2;
      const Vec3fa& v3 = bezier.p3;

      /* transform control points into ray space */
      Vec3fa w0 = xfmVector(ray_space,v0-ray.org); w0.w = v0.w;
      Vec3fa w1 = xfmVector(ray_space,v1-ray.org); w1.w = v1.w;
      Vec3fa w2 = xfmVector(ray_space,v2-ray.org); w2.w = v2.w;
      Vec3fa w3 = xfmVector(ray_space,v3-ray.org); w3.w = v3.w;
      BezierCurve3D curve2D(w0,w1,w2,w3,0.0f,1.0f,4);

      /* subdivide 3 levels at once */ 
      const avx4f p0 = curve2D.eval(coeff0[0],coeff0[1],coeff0[2],coeff0[3]);
      const avx4f p1 = curve2D.eval(coeff1[0],coeff1[1],coeff1[2],coeff1[3]);

      /* approximative intersection with cone */
      const avx4f v = p1-p0;
      const avx4f w = -p0;
      const avxf d0 = w.x*v.x + w.y*v.y;
      const avxf d1 = v.x*v.x + v.y*v.y;
      const avxf u = clamp(d0*rcp(d1),avxf(zero),avxf(one));
      const avx4f p = p0 + u*v;
      const avxf t = p.z;
      const avxf d2 = p.x*p.x + p.y*p.y; 
      const avxf r = p.w; //max(p.w,ray.org.w+ray.dir.w*t);
      const avxf r2 = r*r;
      const avxb valid = d2 <= r2 & avxf(ray.tnear) < t & t < avxf(ray.tfar);
      return any(valid);
    }
    
    void BVH4HairIntersector1::occluded(const BVH4Hair* bvh, Ray& ray) 
    {
      /*! perform per ray precalculations required by the primitive intersector */
      //const Precalculations pre(ray);
      const LinearSpace3fa pre(rcp(frame(ray.dir)));

      /*! stack state */
      StackItem stack[stackSize];  //!< stack of nodes 
      StackItem* stackPtr = stack+1;        //!< current stack pointer
      StackItem* stackEnd = stack+stackSize;
      stack[0].ref = bvh->root;
      stack[0].tNear = ray.tnear;
      stack[0].tFar = ray.tfar;
      
      /*! offsets to select the side that becomes the lower or upper bound */
      const size_t nearX = ray.dir.x >= 0.0f ? 0*sizeof(ssef) : 1*sizeof(ssef);
      const size_t nearY = ray.dir.y >= 0.0f ? 2*sizeof(ssef) : 3*sizeof(ssef);
      const size_t nearZ = ray.dir.z >= 0.0f ? 4*sizeof(ssef) : 5*sizeof(ssef);
      
      /*! load the ray into SIMD registers */
      const sse3f norg(-ray.org.x,-ray.org.y,-ray.org.z);
      const Vec3fa ray_rdir = rcp_safe(ray.dir);
      const sse3f rdir(ray_rdir.x,ray_rdir.y,ray_rdir.z);
      const Vec3fa ray_org_rdir = ray.org*ray_rdir;
      const sse3f org_rdir(ray_org_rdir.x,ray_org_rdir.y,ray_org_rdir.z);

      /* pop loop */
      while (true) pop:
      {
        /*! pop next node */
        if (unlikely(stackPtr == stack)) break;
        stackPtr--;
        NodeRef cur = NodeRef(stackPtr->ref);
        ssef tNear = stackPtr->tNear;
        ssef tFar = min(stackPtr->tFar,ray.tfar);
        
        /*! if popped node is too far, pop next one */
        if (unlikely(_mm_cvtss_f32(tNear) > _mm_cvtss_f32(tFar)))
          continue;
        
        /* downtraversal loop */
        while (true)
        {
          /*! stop if we found a leaf */
          size_t mask;
          if (unlikely(cur.isLeaf())) break;
          STAT3(normal.trav_nodes,1,1,1);
          
          /*! process nodes with aligned bounds */
          if (likely(cur.isAlignedNode()))
          {
            /*! single ray intersection with 4 boxes */
            const AlignedNode* node = cur.alignedNode();
            const size_t farX  = nearX ^ 16, farY  = nearY ^ 16, farZ  = nearZ ^ 16;
#if defined (__AVX2__)
            const ssef tNearX = msub(load4f((const char*)&node->lower_x+nearX), rdir.x, org_rdir.x);
            const ssef tNearY = msub(load4f((const char*)&node->lower_x+nearY), rdir.y, org_rdir.y);
            const ssef tNearZ = msub(load4f((const char*)&node->lower_x+nearZ), rdir.z, org_rdir.z);
            const ssef tFarX  = msub(load4f((const char*)&node->lower_x+farX ), rdir.x, org_rdir.x);
            const ssef tFarY  = msub(load4f((const char*)&node->lower_x+farY ), rdir.y, org_rdir.y);
            const ssef tFarZ  = msub(load4f((const char*)&node->lower_x+farZ ), rdir.z, org_rdir.z);
#else
            const ssef tNearX = (norg.x + load4f((const char*)&node->lower_x+nearX)) * rdir.x;
            const ssef tNearY = (norg.y + load4f((const char*)&node->lower_x+nearY)) * rdir.y;
            const ssef tNearZ = (norg.z + load4f((const char*)&node->lower_x+nearZ)) * rdir.z;
            const ssef tFarX  = (norg.x + load4f((const char*)&node->lower_x+farX )) * rdir.x;
            const ssef tFarY  = (norg.y + load4f((const char*)&node->lower_x+farY )) * rdir.y;
            const ssef tFarZ  = (norg.z + load4f((const char*)&node->lower_x+farZ )) * rdir.z;
#endif
            
#if defined(__SSE4_1__)
            tNear = maxi(maxi(tNearX,tNearY),maxi(tNearZ,tNear));
            tFar  = mini(mini(tFarX ,tFarY ),mini(tFarZ ,tFar));
            const sseb vmask = cast(tNear) > cast(tFar);
            mask = movemask(vmask)^0xf;
#else
            tNear = max(tNearX,tNearY,tNearZ,tNear);
            tFar  = min(tFarX ,tFarY ,tFarZ ,tNear);
            const sseb vmask = tNear <= tFar;
            mask = movemask(vmask);
#endif
          }

          /*! process nodes with unaligned bounds */
          else {
            const UnalignedNode* node = cur.unalignedNode();
            mask = intersectBox(node->naabb,ray,tNear,tFar);
          }

          /*! if no child is hit, pop next node */
          const Node* node = cur.node();
          if (unlikely(mask == 0))
            goto pop;
          
          /*! one child is hit, continue with that child */
          size_t r = __bscf(mask);
          if (likely(mask == 0)) {
            cur = node->child(r); tNear = tNear[r]; tFar = tFar[r];
            assert(cur != BVH4::emptyNode);
            continue;
          }
          
          /*! two children are hit, push far child, and continue with closer child */
          NodeRef c0 = node->child(r); const float n0 = tNear[r]; const float f0 = tFar[r]; 
          r = __bscf(mask);
          NodeRef c1 = node->child(r); const float n1 = tNear[r]; const float f1 = tFar[r];
          assert(c0 != BVH4::emptyNode);
          assert(c1 != BVH4::emptyNode);
          if (likely(mask == 0)) {
            assert(stackPtr < stackEnd); 
            if (n0 < n1) { stackPtr->ref = c1; stackPtr->tNear = n1; stackPtr->tFar = f1; stackPtr++; cur = c0; tNear = n0; tFar = f0; continue; }
            else         { stackPtr->ref = c0; stackPtr->tNear = n0; stackPtr->tFar = f0; stackPtr++; cur = c1; tNear = n1; tFar = f1; continue; }
          }
          
          /*! Here starts the slow path for 3 or 4 hit children. We push
           *  all nodes onto the stack to sort them there. */
          assert(stackPtr < stackEnd); 
          stackPtr->ref = c0; stackPtr->tNear = n0; stackPtr->tFar = f0; stackPtr++;
          assert(stackPtr < stackEnd); 
          stackPtr->ref = c1; stackPtr->tNear = n1; stackPtr->tFar = f1; stackPtr++;
          
          /*! three children are hit, push all onto stack and sort 3 stack items, continue with closest child */
          assert(stackPtr < stackEnd); 
          r = __bscf(mask);
          NodeRef c = node->child(r); float n2 = tNear[r]; float f2 = tFar[r]; 
          cur = c; tNear = n2; tFar = f2;
          assert(c != BVH4::emptyNode);
          if (likely(mask == 0)) continue;
          assert(stackPtr < stackEnd); 
          stackPtr->ref = c; stackPtr->tNear = n2; stackPtr->tFar = f2; stackPtr++;
          
          /*! four children are hit, push all onto stack and sort 4 stack items, continue with closest child */
          cur = node->child(3); tNear = tNear[3]; tFar = tFar[3]; 
          assert(cur != BVH4::emptyNode);
        }
        
        /*! this is a leaf node */
        STAT3(normal.trav_leaves,1,1,1);
        size_t num; Bezier1* prim = (Bezier1*) cur.leaf(num);
        for (size_t i=0; i<num; i++) {
          if (occludedBezier(pre,ray,prim[i])) {
            ray.geomID = 0;
            break;
          }
        }
      }
      AVX_ZERO_UPPER();
    }

    DEFINE_INTERSECTOR1(BVH4HairIntersector1_,BVH4HairIntersector1);
  }
}
