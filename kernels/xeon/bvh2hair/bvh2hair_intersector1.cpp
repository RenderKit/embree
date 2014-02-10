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
    __forceinline bool BVH2HairIntersector1::intersectBox(const NAABBox3fa& naabb, const Ray& ray, float& tNear, float& tFar)
    {
      const Vec3fa org = xfmPoint (naabb.space,ray.org);
      const Vec3fa dir = xfmVector(naabb.space,ray.dir);
      const Vec3fa rdir = rcp(dir);
      
      const float tLowerX = (naabb.bounds.lower.x - org.x) * rdir.x;
      const float tLowerY = (naabb.bounds.lower.y - org.y) * rdir.y;
      const float tLowerZ = (naabb.bounds.lower.z - org.z) * rdir.z;

      const float tUpperX = (naabb.bounds.upper.x - org.x) * rdir.x;
      const float tUpperY = (naabb.bounds.upper.y - org.y) * rdir.y;
      const float tUpperZ = (naabb.bounds.upper.z - org.z) * rdir.z;

      const float tNearX = min(tLowerX,tUpperX);
      const float tNearY = min(tLowerY,tUpperY);
      const float tNearZ = min(tLowerZ,tUpperZ);

      const float tFarX = max(tLowerX,tUpperX);
      const float tFarY = max(tLowerY,tUpperY);
      const float tFarZ = max(tLowerZ,tUpperZ);
      
      tNear = max(tNearX,tNearY,tNearZ,tNear);
      tFar  = min(tFarX ,tFarY ,tFarZ ,tFar);
      return tNear <= tFar;
    }

    __forceinline void BVH2HairIntersector1::intersectBezier(Ray& ray, const Bezier1& bezier)
    {
      LinearSpace3f ray_space(rcp(frame(ray.dir)));

      /* load bezier curve control points */
      STAT3(normal.trav_prims,1,1,1);
      const Vec3fa v0 = bezier.p0;
      const Vec3fa v1 = bezier.p1;
      const Vec3fa v2 = bezier.p2;
      const Vec3fa v3 = bezier.p3;

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
      const avxf u = clamp(d0/d1,avxf(zero),avxf(one));
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
          const Node* node = cur.node();

          /*! intersect with both non-axis aligned boxes */
          float tNear0 = tNear, tFar0 = tFar;
          bool hit0 = intersectBox(node->bounds(0), ray, tNear0, tFar0);
          float tNear1 = tNear, tFar1 = tFar;
          bool hit1 = intersectBox(node->bounds(1), ray, tNear1, tFar1);

          /*ray.geomID = 0;
          if (hit0) ray.geomID += 123434;
          if (hit1) ray.geomID += 2344343;
          return;*/
          
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
        
        /*! this is a leaf node */
        STAT3(normal.trav_leaves,1,1,1);
        size_t num; Bezier1* prim = (Bezier1*) cur.leaf(num);
        for (size_t i=0; i<num; i++) intersectBezier(ray,prim[i]);
      }
    }
    
    void BVH2HairIntersector1::occluded(const BVH2Hair* bvh, Ray& ray) {
    }

    DEFINE_INTERSECTOR1(BVH2HairIntersector1_,BVH2HairIntersector1);
  }
}
