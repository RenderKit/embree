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

#ifndef __EMBREE_TRIANGLE1I_INTERSECTOR_WATERTIGHT_H__
#define __EMBREE_TRIANGLE1I_INTERSECTOR_WATERTIGHT_H__

#include "triangle1i.h"
#include "../common/ray.h"
#include "../common/hit.h"

namespace embree
{
  /* Additional per ray data of the watertight algorithm. */
  struct RayShear
  {
    __forceinline RayShear (const Ray& ray) 
    {
      const Vec3f dir = ray.dir;
      const Vec3f rdir = ray.rdir;

      /* calculate dimension where ray direction is maximal */
      if (fabsf(dir.x) > fabsf(dir.y)) {
        if (fabsf(dir.x) > fabsf(dir.z)) { kz = 0; kx = 1; ky = 2;} else { kz = 2; kx = 0; ky = 1; }
      } else {
        if (fabsf(dir.y) > fabsf(dir.z)) { kz = 1; kx = 2; ky = 0; } else { kz = 2; kx = 0; ky = 1; }
      }
      
      /* swap kx and ky dimension to preserve winding direction of triangles */
      if (dir[kz] < 0.0f) std::swap(kx,ky);
      
      /* calculate shear constants */
      Sz = rdir[kz];
      Sx = dir[kx]*Sz;
      Sy = dir[ky]*Sz;
    }

    /* data for watertight intersection */
  public:
    int kx,ky,kz;   //!< axis change for watertight intersector
    float Sx,Sy,Sz; //!< ray shear transformation for watertight intersector
  };

  /*! Watertight ray/triangle intersection algorithm. Perform first a ray-dependent 
      shear of the triangle vertices, followed by a watertight edge test. */
  struct Triangle1iIntersectorWatertight
  {
    typedef Triangle1i Triangle;

    /*! Intersect a ray with the 4 triangles and updates the hit. */
    static __forceinline void intersect(const Ray& ray, const RayShear& shear, Hit& hit, const Triangle1i& tri, const Vec3fa* vertices)
    {
      STAT3(normal.trav_tris,1,1,1);

      /* calculate vertices releative to ray origin */
      const Vec3f O = ray.org;
      const Vec3f D = ray.dir;
      const Vec3f A = vertices[tri.v0]-O;
      const Vec3f B = vertices[tri.v1]-O;
      const Vec3f C = vertices[tri.v2]-O;

      /* perform shear and scale of vertices */
      const int   kx = shear.kx, ky = shear.ky, kz = shear.kz;
      const float Sx = shear.Sx, Sy = shear.Sy, Sz = shear.Sz;
      const float Ax = A[kx] - Sx*A[kz];
      const float Ay = A[ky] - Sy*A[kz];
      const float Bx = B[kx] - Sx*B[kz];
      const float By = B[ky] - Sy*B[kz];
      const float Cx = C[kx] - Sx*C[kz];
      const float Cy = C[ky] - Sy*C[kz];
      
      /* calculate scaled barycentric coordinates */
      float U = Cx*By-Cy*Bx; 
      float V = Ax*Cy-Ay*Cx; 
      float W = Bx*Ay-By*Ax; 

/* Perform edge tests. */
#if BACKFACE_CULLING
      if (U<0.0f || V<0.0f || W<0.0f) 
        return;
#else
      if ((U<0.0f || V<0.0f || W<0.0f) && (U>0.0f || V>0.0f || W>0.0f)) 
        return;
#endif

      /* fallback to test against edges using double precision */
#if DOUBLE_FALLBACK
      if (unlikely(U == 0.0f || V == 0.0f || W == 0.0f)) {
        U = (double)Cx*(double)By-(double)Cy*(double)Bx; 
        V = (double)Ax*(double)Cy-(double)Ay*(double)Cx; 
        W = (double)Bx*(double)Ay-(double)By*(double)Ax;

        /* Perform edge tests. */
#if BACKFACE_CULLING
        if (U<0.0f || V<0.0f || W<0.0f) 
          return;
#else
        if ((U<0.0f || V<0.0f || W<0.0f) && (U>0.0f || V>0.0f || W>0.0f)) 
          return;
#endif
      }
#endif

      /* calculate denominator  */
      const float det = U+V+W; 
      if (det == 0.0f) return;

      /* Calculate scaled z-coordinates of vertices and use 
         them to calculate the hit distance. */
      const float Az = Sz*A[kz]; 
      const float Bz = Sz*B[kz];
      const float Cz = Sz*C[kz];
      const float T = U*Az+V*Bz+W*Cz;

      /* perform depth test */
#if BACKFACE_CULLING
      if (unlikely(T < ray.near * det || hit.t * det < T)) 
        return;
#else
      const float sign_det = signmsk(det);
      if (unlikely(xorf(T,sign_det) < ray.near * xorf(det,sign_det) ||
                   hit.t * xorf(det,sign_det) < xorf(T,sign_det))) 
        return;
#endif

      /* store normalized U,V,W, and T */
      const float rcpDet = rcp(det);
      hit.u   = V*rcpDet;
      hit.v   = W*rcpDet;
      hit.t   = T*rcpDet;
      hit.id0 = tri.id0;
      hit.id1 = tri.id1;
    }

    /*! Test if the ray is occluded by one of the triangles. */
    static __forceinline bool occluded(const Ray& ray, const RayShear& shear, const Triangle1i& tri, const Vec3fa* vertices = NULL) 
    {
      STAT3(shadow.trav_tris,1,1,1);

      /* calculate vertices releative to ray origin */
      const Vec3f O = ray.org;
      const Vec3f D = ray.dir;
      const Vec3f A = vertices[tri.v0]-O;
      const Vec3f B = vertices[tri.v1]-O;
      const Vec3f C = vertices[tri.v2]-O;

      /* perform shear and scale of vertices */
      const int   kx = shear.kx, ky = shear.ky, kz = shear.kz;
      const float Sx = shear.Sx, Sy = shear.Sy, Sz = shear.Sz;
      const float Ax = A[kx] - Sx*A[kz];
      const float Ay = A[ky] - Sy*A[kz];
      const float Bx = B[kx] - Sx*B[kz];
      const float By = B[ky] - Sy*B[kz];
      const float Cx = C[kx] - Sx*C[kz];
      const float Cy = C[ky] - Sy*C[kz];
      
      /* calculate scaled barycentric coordinates */
      float U = Cx*By-Cy*Bx; 
      float V = Ax*Cy-Ay*Cx; 
      float W = Bx*Ay-By*Ax; 

      /* Perform edge tests. */
#if BACKFACE_CULLING
      if (U<0.0f || V<0.0f || W<0.0f) 
        return false;
#else
      if ((U<0.0f || V<0.0f || W<0.0f) && (U>0.0f || V>0.0f || W>0.0f)) 
        return false;
#endif

      /* fallback to test against edges using double precision */
#if DOUBLE_FALLBACK
      if (unlikely(U == 0.0f || V == 0.0f || W == 0.0f)) {
        U = (double)Cx*(double)By-(double)Cy*(double)Bx; 
        V = (double)Ax*(double)Cy-(double)Ay*(double)Cx; 
        W = (double)Bx*(double)Ay-(double)By*(double)Ax; 

        /* Perform edge tests. */
#if BACKFACE_CULLING
        if (U<0.0f || V<0.0f || W<0.0f) 
          return false;
#else
        if ((U<0.0f || V<0.0f || W<0.0f) && (U>0.0f || V>0.0f || W>0.0f)) 
          return false;
#endif
      }
#endif
      
      /* calculate denominator  */
      const float det = U+V+W; 
      if (det == 0.0f) return false;

      /* Calculate scaled z-coordinates of vertices and use 
         them to calculate the hit distance. */
      const float Az = Sz*A[kz]; 
      const float Bz = Sz*B[kz];
      const float Cz = Sz*C[kz];
      const float T = U*Az+V*Bz+W*Cz;

      /* perform depth test */
#if BACKFACE_CULLING
      if (unlikely(T < ray.near * det || ray.far * det < T)) 
        return false;
#else
      const float sign_det = signmsk(det);
      if (unlikely(xorf(T,sign_det) < ray.near * xorf(det,sign_det) ||
                   ray.far * xorf(det,sign_det) < xorf(T,sign_det))) 
        return false;
#endif
      return true;
    }
  };
}

#endif
