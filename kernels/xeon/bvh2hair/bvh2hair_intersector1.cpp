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

#include "bvh2hair_intersector1.h"
#include "geometry/bezier1i_intersector1.h"

namespace embree
{ 
  namespace isa
  {
    __forceinline bool BVH2HairIntersector1::intersectBox(const BBox3fa& aabb, const Ray& ray, const Vec3fa& org_rdir, const Vec3fa& rdir, float& tNear, float& tFar)
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
      const Vec3fa tFarXYZ = maxi(tLowerXYZ,tUpperXYZ);
      tNear = max(reduce_max(tNearXYZ),tNear);
      tFar  = min(reduce_min(tFarXYZ ),tFar);
      return tNear <= tFar;
#else
      const Vec3fa tNearXYZ = min(tLowerXYZ,tUpperXYZ);
      const Vec3fa tFarXYZ = max(tLowerXYZ,tUpperXYZ);
      tNear = max(reduce_max(tNearXYZ),tNear);
      tFar  = min(reduce_min(tFarXYZ ),tFar);
      return tNear <= tFar;
#endif
    }

    __forceinline bool BVH2HairIntersector1::intersectBox(const AffineSpace3fa& naabb, const Ray& ray, float& tNear, float& tFar)
    {
      const Vec3fa dir = xfmVector(naabb,ray.dir);
      const Vec3fa rdir = rcp(dir);
      const Vec3fa org = xfmPoint (naabb,ray.org);
      const Vec3fa tLowerXYZ = - org * rdir;
      const Vec3fa tUpperXYZ = (Vec3fa(one) - org) * rdir;
      const Vec3fa tNearXYZ = min(tLowerXYZ,tUpperXYZ);
      const Vec3fa tFarXYZ = max(tLowerXYZ,tUpperXYZ);
      tNear = max(reduce_max(tNearXYZ),tNear);
      tFar  = min(reduce_min(tFarXYZ ),tFar);
      return tNear <= tFar;
    }

    static __forceinline bool intersectLineBoxes(const Ray& ray,const avx4f &p0, const avx4f &p1)
    {
      const avx2f d0(p0.x,p0.y);
      const avx2f d1(p1.x,p1.y);
      const avx2f d_min = min(d0,d1) - avx2f(p0.w);
      const avx2f d_max = max(d0,d1) + avx2f(p1.w);
      const avxb vmask_x = (d_min.x <= avxf(zero)) & (d_max.x >= avxf(zero));
      const avxb vmask_y = (d_min.y <= avxf(zero)) & (d_max.y >= avxf(zero));
      return any(vmask_x & vmask_y);
    }

    __forceinline void BVH2HairIntersector1::intersectBezier(const LinearSpace3fa &ray_space, Ray& ray, const Bezier1& bezier)
    {
      /* load bezier curve control points */
      STAT3(normal.trav_prims,1,1,1);
      const Vec3fa &v0 = bezier.p0;
      const Vec3fa &v1 = bezier.p1;
      const Vec3fa &v2 = bezier.p2;
      const Vec3fa &v3 = bezier.p3;


      /* transform control points into ray space */
      Vec3fa w0 = xfmVector(ray_space,v0-ray.org); w0.w = v0.w;
      Vec3fa w1 = xfmVector(ray_space,v1-ray.org); w1.w = v1.w;
      Vec3fa w2 = xfmVector(ray_space,v2-ray.org); w2.w = v2.w;
      Vec3fa w3 = xfmVector(ray_space,v3-ray.org); w3.w = v3.w;
      BezierCurve3D curve2D(w0,w1,w2,w3,0.0f,1.0f,4);

      /* subdivide 3 levels at once */ 
      const avx4f p0 = curve2D.eval(coeff0[0],coeff0[1],coeff0[2],coeff0[3]);
      const avx4f p1 = curve2D.eval(coeff1[0],coeff1[1],coeff1[2],coeff1[3]);

      //if (!intersectLineBoxes(ray,p0,p1)) { return; }

      /* approximative intersection with cone */
      const avx4f v = p1-p0;
      const avx4f w = -p0;
      const avxf d0 = w.x*v.x + w.y*v.y;
      const avxf d1 = v.x*v.x + v.y*v.y;
      const avxf u = clamp(d0*rcp(d1),avxf(zero),avxf(one));
      const avx4f p = p0 + u*v;
      const avxf t = p.z;
      const avxf d2 = p.x*p.x + p.y*p.y; 
      const avxf r = max(p.w,ray.org.w+ray.dir.w*t);
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

    void BVH2HairIntersector1::intersect(const BVH2Hair* bvh, Ray& ray)
    {
      /*! stack state */
      StackItem stack[stackSize];  //!< stack of nodes 
      StackItem* stackPtr = stack+1;        //!< current stack pointer
      StackItem* stackEnd = stack+stackSize;
      stack[0].ref = bvh->root;
      stack[0].tNear = ray.tnear;
      stack[0].tFar  = ray.tfar;
      const Vec3fa rdir = rcp(ray.dir);
      const Vec3fa org_rdir = ray.org*rdir;

      const LinearSpace3fa ray_space(rcp(frame(ray.dir)));

      /* pop loop */
      while (true) pop:
      {
        /*! pop next node */
        if (unlikely(stackPtr == stack)) break;
        stackPtr--;
        NodeRef cur = stackPtr->ref;
        float tNear = stackPtr->tNear;
        float tFar  = stackPtr->tFar;
        
        /*! if popped node is too far, pop next one */
        if (unlikely(tNear > ray.tfar))
          continue;
        
        /* downtraversal loop */
        while (true)
        {
          /*! stop if we found a leaf */
          if (unlikely(cur.isLeaf())) break;
          STAT3(normal.trav_nodes,1,1,1);

          /*! process nodes with aligned bounds */
          if (likely(cur.isAlignedNode()))
          {
            const AlignedNode* node = cur.alignedNode();

            /*! intersect with both axis aligned boxes */
            float tNear0 = tNear, tFar0 = tFar;
            bool hit0 = intersectBox(node->bounds(0), ray, org_rdir, rdir, tNear0, tFar0);
            float tNear1 = tNear, tFar1 = tFar;
            bool hit1 = intersectBox(node->bounds(1), ray, org_rdir, rdir, tNear1, tFar1);

            /*! if no child is hit, pop next node */
            if (unlikely(!hit0 && !hit1))
              goto pop;
          
            /*! one child is hit, continue with that child */
            if (hit0 != hit1) {
              if (hit0) { cur = node->child(0); tNear = tNear0; tFar = tFar0; }
              else      { cur = node->child(1); tNear = tNear1; tFar = tFar1; }
              assert(cur != BVH2Hair::emptyNode);
              continue;
            }
          
            /*! two children are hit, push far child, and continue with closer child */
            NodeRef c0 = node->child(0);
            NodeRef c1 = node->child(1);
            assert(c0 != BVH2Hair::emptyNode);
            assert(c1 != BVH2Hair::emptyNode);
            assert(stackPtr < stackEnd); 
            if (tNear0 < tNear1) { 
              stackPtr->ref = c1; stackPtr->tNear = tNear1; stackPtr->tFar = tFar1; stackPtr++; 
              cur = c0; tNear = tNear0; tFar = tFar0;
            }
            else { 
              stackPtr->ref = c0; stackPtr->tNear = tNear0; stackPtr->tFar = tFar0; stackPtr++; 
              cur = c1; tNear = tNear1; tFar = tFar1;
            }
          }
          
          /*! process nodes with unaligned bounds */
          else
          {
            const UnalignedNode* node = cur.unalignedNode();
            /*! intersect with both non-axis aligned boxes */
            float tNear0 = tNear, tFar0 = tFar;
            bool hit0 = intersectBox(node->bounds(0), ray, tNear0, tFar0);
            float tNear1 = tNear, tFar1 = tFar;
            bool hit1 = intersectBox(node->bounds(1), ray, tNear1, tFar1);

            /*! if no child is hit, pop next node */
            if (unlikely(!hit0 && !hit1))
              goto pop;
          
            /*! one child is hit, continue with that child */
            if (hit0 != hit1) {
              if (hit0) { cur = node->child(0); tNear = tNear0; tFar = tFar0; }
              else      { cur = node->child(1); tNear = tNear1; tFar = tFar1; }
              assert(cur != BVH2Hair::emptyNode);
              continue;
            }
          
            /*! two children are hit, push far child, and continue with closer child */
            NodeRef c0 = node->child(0);
            NodeRef c1 = node->child(1);
            assert(c0 != BVH2Hair::emptyNode);
            assert(c1 != BVH2Hair::emptyNode);
            assert(stackPtr < stackEnd); 
            if (tNear0 < tNear1) { 

              stackPtr->ref = c1; stackPtr->tNear = tNear1; stackPtr->tFar = tFar1; stackPtr++; 
              cur = c0; tNear = tNear0; tFar = tFar0;
	      
            }
            else { 

              stackPtr->ref = c0; stackPtr->tNear = tNear0; stackPtr->tFar = tFar0; stackPtr++; 
              cur = c1; tNear = tNear1; tFar = tFar1;
            }
          }
        }
        
        /*! this is a leaf node */
        STAT3(normal.trav_leaves,1,1,1);
        size_t num; Bezier1* prim = (Bezier1*) cur.leaf(num);
        for (size_t i=0; i<num; i++) intersectBezier(ray_space,ray,prim[i]);
      }
    }
    
    void BVH2HairIntersector1::occluded(const BVH2Hair* bvh, Ray& ray) {
    }

    DEFINE_INTERSECTOR1(BVH2HairIntersector1_,BVH2HairIntersector1);
  }
}
