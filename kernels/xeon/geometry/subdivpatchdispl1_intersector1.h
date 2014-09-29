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

#include "subdivpatchdispl1.h"
#include "common/ray.h"
#include "geometry/filter.h"

namespace embree
{
  template<bool list>
  struct SubdivPatchDispl1Intersector1
  {
    typedef SubdivPatchDispl1 Primitive;
    typedef SubdivPatchDispl1::Node Node;
    typedef SubdivPatchDispl1::SubTree SubTree;

    struct Precalculations {
      __forceinline Precalculations (const Ray& ray) {}
    };

    /*! Intersect a ray with the triangle and updates the hit. */
    static __forceinline void intersectTriangle(Ray& ray, const Vec3fa& tri_v0, const Vec3fa& tri_v1, const Vec3fa& tri_v2, const unsigned primID, const unsigned geomID)
    {
      /* calculate vertices relative to ray origin */
      STAT3(normal.trav_prims,1,1,1);
      const Vec3fa O = ray.org;
      const Vec3fa D = ray.dir;
      const Vec3fa v0 = tri_v0-O;
      const Vec3fa v1 = tri_v1-O;
      const Vec3fa v2 = tri_v2-O;
        
      /* calculate triangle edges */
      const Vec3fa e0 = v2-v0;
      const Vec3fa e1 = v0-v1;
      const Vec3fa e2 = v1-v2;
      
      /* calculate geometry normal and denominator */
      const Vec3fa Ng1 = cross(e1,e0);
      const Vec3fa Ng = Ng1+Ng1;
      const float den = dot(Ng,D);
      const float absDen = abs(den);
      const float sgnDen = signmsk(den);
      
      /* perform edge tests */
      const float U = xorf(dot(cross(v2+v0,e0),D),sgnDen);
      if (unlikely(U < 0.0f)) return;
      const float V = xorf(dot(cross(v0+v1,e1),D),sgnDen);
      if (unlikely(V < 0.0f)) return;
      const float W = xorf(dot(cross(v1+v2,e2),D),sgnDen);
      if (unlikely(W < 0.0f)) return;
      
      /* perform depth test */
      const float T = xorf(dot(v0,Ng),sgnDen);
      if (unlikely(absDen*float(ray.tfar) < T)) return;
      if (unlikely(T < absDen*float(ray.tnear))) return;
      
      /* calculate hit information */
      const float rcpAbsDen = rcp(absDen);
      const float u = U * rcpAbsDen;
      const float v = V * rcpAbsDen;
      const float t = T * rcpAbsDen;
      
      /* update hit information */
      ray.u = u;
      ray.v = v;
      ray.tfar = t;
      ray.Ng  = Ng;
      ray.geomID = geomID;
      ray.primID = primID;
    }
    
    /*! Intersect a ray with the triangle and updates the hit. */
    static __forceinline void intersect(const Precalculations& pre, Ray& ray, const Primitive& prim_i, const void* geom)
    {
      /* load triangle */
      STAT3(normal.trav_prims,1,1,1);
      SubTree& prim = *prim_i.subtree;

      /*! load the ray into SIMD registers */
      const Vec3fa ray_rdir = rcp_safe(ray.dir);
      const Vec3fa ray_org_rdir = ray.org*ray_rdir;
      const sse3f org(ray.org.x,ray.org.y,ray.org.z);
      const sse3f dir(ray.dir.x,ray.dir.y,ray.dir.z);
      const sse3f rdir(ray_rdir.x,ray_rdir.y,ray_rdir.z);
      const sse3f org_rdir(ray_org_rdir.x,ray_org_rdir.y,ray_org_rdir.z);
      const ssef  ray_tnear(ray.tnear);
      ssef ray_tfar(ray.tfar);
      
      /*! offsets to select the side that becomes the lower or upper bound */
      const size_t nearX = ray_rdir.x >= 0.0f ? 0*sizeof(ssef) : 1*sizeof(ssef);
      const size_t nearY = ray_rdir.y >= 0.0f ? 2*sizeof(ssef) : 3*sizeof(ssef);
      const size_t nearZ = ray_rdir.z >= 0.0f ? 4*sizeof(ssef) : 5*sizeof(ssef);
      
      /*! stack handling */
      struct StackItem {
        __forceinline StackItem() {}
        __forceinline StackItem(unsigned x, unsigned y) : x(x), y(y) {}
        unsigned x,y;
      };
      const size_t N = 128*128; // FIXME: this is unsafe
      StackItem stack[N];
      size_t begin = 0, end = 1;
      stack[0] = StackItem(0,0);
      const Node* base = prim.nodes;
      
      for (size_t l=0; l<prim.levels; l++)
      {
        size_t tail = end;
        size_t w = 1<<l;
        for (size_t i=begin; i<end; i++) 
        {
          const size_t x = stack[i].x;
          const size_t y = stack[i].y;
          ssef dist;
          size_t mask = base[y*w+x].intersect<false>(nearX, nearY, nearZ, org, rdir, org_rdir, ray_tnear, ray_tfar, dist);
          if (mask & 1) stack[tail++] = StackItem(2*x+0,2*y+0);
          if (mask & 2) stack[tail++] = StackItem(2*x+1,2*y+0);
          if (mask & 4) stack[tail++] = StackItem(2*x+0,2*y+1);
          if (mask & 8) stack[tail++] = StackItem(2*x+1,2*y+1);
        }
        begin = end; end = tail; 
        base += w*w;
      }
      
      for (size_t i=begin; i<end; i++) 
      {
        const size_t x = 2*stack[i].x;
        const size_t y = 2*stack[i].y;
        intersectTriangle(ray,prim.vertices(x+0,y+0),prim.vertices(x+1,y+0),prim.vertices(x+0,y+1),prim.geomID,prim.primID);
        intersectTriangle(ray,prim.vertices(x+1,y+1),prim.vertices(x+0,y+1),prim.vertices(x+1,y+0),prim.geomID,prim.primID);
        intersectTriangle(ray,prim.vertices(x+1,y+0),prim.vertices(x+2,y+0),prim.vertices(x+1,y+1),prim.geomID,prim.primID);
        intersectTriangle(ray,prim.vertices(x+2,y+1),prim.vertices(x+1,y+1),prim.vertices(x+2,y+0),prim.geomID,prim.primID);
        intersectTriangle(ray,prim.vertices(x+0,y+1),prim.vertices(x+1,y+1),prim.vertices(x+0,y+2),prim.geomID,prim.primID);
        intersectTriangle(ray,prim.vertices(x+1,y+2),prim.vertices(x+0,y+2),prim.vertices(x+1,y+1),prim.geomID,prim.primID);
        intersectTriangle(ray,prim.vertices(x+1,y+1),prim.vertices(x+2,y+1),prim.vertices(x+1,y+2),prim.geomID,prim.primID);
        intersectTriangle(ray,prim.vertices(x+2,y+2),prim.vertices(x+1,y+2),prim.vertices(x+2,y+1),prim.geomID,prim.primID);
      }
      ray_tfar = ray.tfar;
    }
    
    /*! Test if the ray is occluded by the primitive */
    static __forceinline bool occluded(const Precalculations& pre, Ray& ray, const Primitive& subdiv_patch, const void* geom)
    {
      STAT3(shadow.trav_prims,1,1,1);
      return false;
    }
  };
}
