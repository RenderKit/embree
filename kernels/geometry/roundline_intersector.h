// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "../common/ray.h"
#include "curve_intersector_precalculations.h"


/*
  This file implements the intersection of a ray with a round linear
  curve segment. We define the geometry of such a round linear curve
  segment from point p0 with radius r0 to point p1 with radius r1
  using the cone that touches spheres p0/r0 and p1/r1 tangentially
  plus the sphere p1/r1. We denote the tangentially touching cone from
  p0/r0 to p1/r1 with cone(p0,r0,p1,r1) and the cone plus the ending
  sphere with cone_sphere(p0,r0,p1,r1).

  The method follows Reshetov and Hart "Modeling Hair Strands with
  Roving Capsules", SIGGRAPH 2024,
  https://dl.acm.org/doi/10.1145/3641519.3657450
  See also https://www.shadertoy.com/view/4ffXWs
  The surface is directly described by a sphere (with changing radius)
  swept along a line. First we solve for u, where the ray will intersect
  the sphere, followed by a sphere intersection.

  For multiple connected round linear curve segments this construction
  yield a proper shape when viewed from the outside. Using the
  following CSG we can also handle the interior in most common cases:

     round_linear_curve(pl,rl,p0,r0,p1,r1,pr,rr) =
       cone_sphere(p0,r0,p1,r1) - cone(pl,rl,p0,r0) - cone(p1,r1,pr,rr)

  Thus by subtracting the neighboring cone geometries, we cut away
  parts of the center cone_sphere surface which lie inside the
  combined curve. This approach works as long as geometry of the
  current cone_sphere penetrates into direct neighbor segments only,
  and not into segments further away.

  To construct a cone that touches two spheres at p0 and p1 with r0
  and r1, one has to increase the cone radius at r0 and r1 to obtain
  larger radii w0 and w1, such that the infinite cone properly touches
  the spheres.  From the paper "Ray Tracing Generalized Tube
  Primitives: Method and Applications"
  (https://www.researchgate.net/publication/334378683_Ray_Tracing_Generalized_Tube_Primitives_Method_and_Applications)
  one can derive the following equations for these increased
  radii:

     sr = 1.0f / sqrt(1-sqr(dr)/sqr(p1-p0))
     w0 = sr*r0
     w1 = sr*r1

  Further, we want the cone to start where it touches the sphere at p0
  and to end where it touches sphere at p1.  Therefore, we need to
  construct clipping locations y0 and y1 for the start and end of the
  cone. These start and end clipping location of the cone can get
  calculated as:

     Y0 =               - r0 * (r1-r0) / length(p1-p0)
     Y1 = length(p1-p0) - r1 * (r1-r0) / length(p1-p0)

  Where the cone starts a distance Y0 and ends a distance Y1 away of
  point p0 along the cone center. The distance between Y1-Y0 can get
  calculated as:

    dY = length(p1-p0) - (r1-r0)^2 / length(p1-p0)

  In the code below, Y will always be scaled by length(p1-p0) to
  obtain y and you will find the terms r0*(r1-r0) and
  (p1-p0)^2-(r1-r0)^2.

 */

namespace embree
{
  namespace isa
  {
    template<int M>
      struct RoundLineIntersectorHitM
      {
        __forceinline RoundLineIntersectorHitM() {}

        __forceinline RoundLineIntersectorHitM(const vfloat<M>& u, const vfloat<M>& v, const vfloat<M>& t, const Vec3vf<M>& Ng)
          : vu(u), vv(v), vt(t), vNg(Ng) {}
	
        __forceinline void finalize() {}
	
        __forceinline Vec2f uv (const size_t i) const { return Vec2f(vu[i],vv[i]); }
        __forceinline float t  (const size_t i) const { return vt[i]; }
        __forceinline Vec3fa Ng(const size_t i) const { return Vec3fa(vNg.x[i],vNg.y[i],vNg.z[i]); }

        __forceinline Vec2vf<M> uv() const { return Vec2vf<M>(vu,vv); }
        __forceinline vfloat<M> t () const { return vt; }
        __forceinline Vec3vf<M> Ng() const { return vNg; }

      public:
        vfloat<M> vu;
        vfloat<M> vv;
        vfloat<M> vt;
        Vec3vf<M> vNg;
      };

    namespace __roundline_internal
    {
      template<int M>
        struct ConeGeometry
        {
          ConeGeometry (const Vec4vf<M>& a, const Vec4vf<M>& b)
          : p0(a.xyz()), p1(b.xyz()), dP(p1-p0), dPdP(dot(dP,dP)), r0(a.w), sqr_r0(sqr(r0)), r1(b.w), dr(r1-r0), drdr(dr*dr), r0dr (r0*dr), g(dPdP - drdr) {}

          /*
             This function tests whether a point lies inside the capped cone
             tangential to its ending spheres.

             Therefore one has to check if the point is inside the
             region defined by the cone clipping planes, which is
             performed similar as in the previous function.

             To perform the inside cone test we need to project the
             point onto the line p0->p1:

               dP = p1-p0
               Y = (p-p0)*dP/length(dP)

             This value Y is the distance to the projection point from
             p0. To obtain a parameter value u going from 0 to 1 along
             the line p0->p1 we calculate:

               U = Y/length(dP)

             The radii to use at points p0 and p1 are:

               w0 = sr * r0
               w1 = sr * r1
               dw = w1-w0

             Using these radii and u one can directly test if the point
             lies inside the cone using the formula dP*dP < wy*wy with:

               wy = w0 + u*dw
               py = p0 + u*dP - p

             By multiplying the calculations with length(p1-p0) and
             inserting the definition of w can obtain simpler equations:

               y = (p-p0)*dP
               ry = r0 + y/dP^2 * dr
               wy = sr*ry
               py = p0 + y/dP^2*dP - p
               y0 =      - r0 * dr
               y1 = dP^2 - r1 * dr

             Thus for the in-cone test we get:

                    py^2 < wy^2
               <=>  py^2 < sr^2 * ry^2
               <=>  py^2 * ( dP^2 - dr^2 ) < dP^2 * ry^2

             This can further get simplified to:

               (p0-p)^2 * (dP^2 - dr^2) - y^2 < dP^2 * r0^2 + 2.0f*r0*dr*y;
          */

          __forceinline vbool<M> isInsideCappedCone(const vbool<M>& valid_i, const Vec3vf<M>& p) const
          {
            const Vec3vf<M> p0p = p - p0;
            const vfloat<M> y = dot(p0p,dP);
            const vfloat<M> cap0 = vfloat<M>(ulp) - r0dr;
            const vfloat<M> cap1 = dPdP - r1*dr;

            vbool<M> inside_cone = valid_i & (p0.x != vfloat<M>(inf)) & (p1.x != vfloat<M>(inf));
            inside_cone &= y > cap0;  // start clipping plane
            inside_cone &= y < cap1;  // end clipping plane
            inside_cone &= sqr(p0p)*g - sqr(y) < dPdP * sqr_r0 + 2.0f*r0dr*y; // in cone test
            return inside_cone;
          }

        protected:
          Vec3vf<M> p0;
          Vec3vf<M> p1;
          Vec3vf<M> dP;
          vfloat<M> dPdP;
          vfloat<M> r0;
          vfloat<M> sqr_r0;
          vfloat<M> r1;
          vfloat<M> dr;
          vfloat<M> drdr;
          vfloat<M> r0dr;
          vfloat<M> g;
        };

      template<int M>
        struct ConeGeometryIntersector : public ConeGeometry<M>
      {
        using ConeGeometry<M>::p0;
        using ConeGeometry<M>::p1;
        using ConeGeometry<M>::dP;
        using ConeGeometry<M>::dPdP;
        using ConeGeometry<M>::r0;
        using ConeGeometry<M>::sqr_r0;
        using ConeGeometry<M>::r1;
        using ConeGeometry<M>::dr;
        using ConeGeometry<M>::r0dr;
        using ConeGeometry<M>::g;

        ConeGeometryIntersector (const Vec3vf<M>& ray_org, const Vec3vf<M>& ray_dir, const vfloat<M>& dOdO, const vfloat<M>& rcp_dOdO, const Vec4vf<M>& a, const Vec4vf<M>& b)
          : ConeGeometry<M>(a,b), org(ray_org), P(p0-ray_org), dO(ray_dir),  dOdO(dOdO), rcp_dOdO(rcp_dOdO), OdO(dot(dO,P)), dOdP(dot(dP,dO)), OdO_dOdO(OdO*rcp_dOdO), dOdP_dOdO(dOdP*rcp_dOdO) {
            const Vec3vf<M> l = P - OdO_dOdO * dO; // more precise than dot(P,P) - OdO^2/dOdO (catastrophic cancellation of squared terms)
            G = sqr_r0 - sqr(l);
            C = g - dOdP * dOdP_dOdO;
            F = r0dr - dot(dP,P) + dOdP * OdO_dOdO;
          }

        /*

          This function finds u where the ray will intersect the swept sphere

        */

        __forceinline bool intersectConeU(vbool<M>& valid, vfloat<M>& u_front, vfloat<M>& u_back)
        {
          /* we miss the cone if determinant is smaller than zero */
          const vfloat<M> D = (F*F + G*C) * rcp_dOdO * rcp(g);
          valid &= (D >= 0.0f) | (g <= 0.0f);  // or inside a sphere end

          if (unlikely(none(valid)))
            return false;

          const vfloat<M> Q = dOdP * sqrt(D);
          const vfloat<M> rcp_C = rcp(C);
          vfloat<M> u0 = (F + Q) * rcp_C;
          vfloat<M> u1 = (F - Q) * rcp_C;
          // flip negative caps to other side
          u0 = select(r0 + u0*dr < 0.0f, 1.0f-u0, u0);
          u1 = select(r0 + u1*dr < 0.0f, 1.0f-u1, u1);
          const vfloat<M> uend = select(dr >= 0.0f, vfloat<M>(one), vfloat<M>(zero));
          const vbool<M> swap = (dOdP >= 0.0) != (u1 > u0);
          // already clamp for end sphere (always present)
          u_front = min(1.0f, select(g > 0.0f, select(swap, u1, u0), uend));
#if !defined (EMBREE_BACKFACE_CULLING_CURVES)
          u_back = min(1.0f, select(g > 0.0f, select(swap, u0, u1), uend));
#endif
          return true;
        }

        /*
           This function intersects the ray with the swept sphere centered at u.
        */
        template<bool front>     
        __forceinline void intersectSphere(vbool<M>& valid, const vfloat<M>& u, vfloat<M>& t)
        {
          const vfloat<M> D = G + u * (2.0 * F - u * C);
          valid &= D >= 0.0f;
          const vfloat<M> d = sqrt(D);
          t = p0.z + u * dP.z;
          if (front)
            t -= d;
          else
            t += d;
        }

        /*
           Compute geometry normal of hit as the difference vector from hit
           point to projected hit, which is the interpolated sphere center.
           This normal is valid for the cone as well for the end cap spheres.

             Ng = h - (p0 + u*dP)
        */

        __forceinline Vec3vf<M> Ng(const vfloat<M>& u, const vfloat<M>& t) const
        {
          return t*dO-P - u*dP;
        }

      private:
        Vec3vf<M> org;
        Vec3vf<M> P;
        Vec3vf<M> dO;
        vfloat<M> dOdO;
        vfloat<M> rcp_dOdO;
        vfloat<M> OdO;
        vfloat<M> dOdP;
        vfloat<M> OdO_dOdO;
        vfloat<M> dOdP_dOdO;
        vfloat<M> C;
        vfloat<M> F;
        vfloat<M> G;
      };

      template<int M, typename Epilog, typename ray_tfar_func>
        static __forceinline bool intersectConeSphere(const vbool<M>& valid_i,
                                                      const Vec3vf<M>& ray_org, const Vec3vf<M>& ray_dir,
                                                      const vfloat<M>& ray_tnear, const ray_tfar_func& ray_tfar,
                                                      const Vec4vf<M>& v0, const Vec4vf<M>& v1,
                                                      const Vec4vf<M>& vL, const Vec4vf<M>& vR,
                                                      const Epilog& epilog)
      {
        vbool<M> valid = valid_i;

        const vfloat<M> dOdO = sqr(ray_dir);
        const vfloat<M> rcp_dOdO = rcp(dOdO);

        /* intersect with cone from v0 to v1 */
        ConeGeometryIntersector<M> cone (ray_org, ray_dir, dOdO, rcp_dOdO, v0, v1);
        vfloat<M> u_front, u_back;
        if (unlikely(!cone.intersectConeU(valid, u_front, u_back)))
          return false;

        /* calculate front hit */
        vbool<M> validFront = valid & ((u_front >= 0.0f) | (vL[0] == vfloat<M>(pos_inf)));
        u_front = max(u_front, 0.0f);
        vfloat<M> t_lower;
        cone.template intersectSphere<true>(validFront, u_front, t_lower);

#if !defined (EMBREE_BACKFACE_CULLING_CURVES)
        /* hits inside the neighboring capped cones are inside the geometry and thus ignored */
        const ConeGeometry<M> coneL (v0, vL);
        const ConeGeometry<M> coneR (v1, vR);
        const Vec3vf<M> hit_lower = ray_org + t_lower*ray_dir;
        validFront &= !coneL.isInsideCappedCone(validFront, hit_lower) & !coneR.isInsideCappedCone(validFront, hit_lower);

        /* calculate back hit */
        vbool<M> validBack = valid & ((u_back >= 0.0f) | (vL[0] == vfloat<M>(pos_inf)));
        u_back = max(u_back, 0.0f);
        vfloat<M> t_upper;
        cone.template intersectSphere<false>(validBack, u_back, t_upper);
        const Vec3vf<M> hit_upper = ray_org + t_upper*ray_dir;
        validBack &= !coneL.isInsideCappedCone(validBack, hit_upper) & !coneR.isInsideCappedCone(validBack, hit_upper);

        /* filter out hits that are not in tnear/tfar range */
        const vbool<M> valid_lower = validFront & ray_tnear <= t_lower & t_lower <= ray_tfar();
        const vbool<M> valid_upper = validBack & ray_tnear <= t_upper & t_upper <= ray_tfar();

        /* check if there is a first hit */
        const vbool<M> valid_first = valid_lower | valid_upper;
        if (unlikely(none(valid_first)))
          return false;

        /* construct first hit */
        const vfloat<M> t_first = select(valid_lower, t_lower, t_upper);
        const vfloat<M> u_first = select(valid_lower, u_front, u_back);

        /* invoke intersection filter for first hit */
        RoundLineIntersectorHitM<M> hit(u_first,zero,t_first,cone.Ng(u_first,t_first));
        const bool is_hit_first = epilog(valid_first, hit);

        /* check for possible second hits before potentially accepted hit */
        const vbool<M> valid_second = valid_lower & valid_upper & t_upper <= ray_tfar();
        if (unlikely(none(valid_second)))
          return is_hit_first;

        /* invoke intersection filter for second hit */
        hit = RoundLineIntersectorHitM<M>(u_back,zero,t_upper,cone.Ng(u_back,t_upper));
        const bool is_hit_second = epilog(valid_second, hit);

        return is_hit_first | is_hit_second;
#else
        /* filter out hits that are not in tnear/tfar range */
        const vbool<M> valid_lower = validFront & ray_tnear <= t_lower & t_lower <= ray_tfar();

        /* check if there is a valid hit */
        if (unlikely(none(valid_lower)))
          return false;

        /* construct first hit and invoke intersection filter for first hit */
        RoundLineIntersectorHitM<M> hit(u_front,zero,t_lower,cone.Ng(u_front,t_lower));
        const bool is_hit_first = epilog(valid_lower, hit);

        return is_hit_first;
#endif
      }

    } // end namespace __roundline_internal

    template<int M>
      struct RoundLinearCurveIntersector1
      {
        typedef CurvePrecalculations1 Precalculations;

        template<typename Ray>
        struct ray_tfar {
          Ray& ray;
          __forceinline ray_tfar(Ray& ray) : ray(ray) {}
          __forceinline vfloat<M> operator() () const { return ray.tfar; };
        };
	
        template<typename Ray, typename Epilog>
        static __forceinline bool intersect(const vbool<M>& valid_i,
                                            Ray& ray,
                                            RayQueryContext* context,
                                            const LineSegments* geom,
                                            const Precalculations& pre,
                                            const Vec4vf<M>& v0i, const Vec4vf<M>& v1i,
                                            const Vec4vf<M>& vLi, const Vec4vf<M>& vRi,
                                            const Epilog& epilog)
        {
          const Vec3vf<M> ray_org(ray.org.x, ray.org.y, ray.org.z);
          const Vec3vf<M> ray_dir(ray.dir.x, ray.dir.y, ray.dir.z);
          const vfloat<M> ray_tnear(ray.tnear());
          const Vec4vf<M> v0 = enlargeRadiusToMinWidth<M>(context,geom,ray_org,v0i);
          const Vec4vf<M> v1 = enlargeRadiusToMinWidth<M>(context,geom,ray_org,v1i);
          const Vec4vf<M> vL = enlargeRadiusToMinWidth<M>(context,geom,ray_org,vLi);
          const Vec4vf<M> vR = enlargeRadiusToMinWidth<M>(context,geom,ray_org,vRi);
          return  __roundline_internal::intersectConeSphere<M>(valid_i,ray_org,ray_dir,ray_tnear,ray_tfar<Ray>(ray),v0,v1,vL,vR,epilog);
        }
      };

    template<int M, int K>
      struct RoundLinearCurveIntersectorK
      {
        typedef CurvePrecalculationsK<K> Precalculations;

        struct ray_tfar {
          RayK<K>& ray;
          size_t k;
          __forceinline ray_tfar(RayK<K>& ray, size_t k) : ray(ray), k(k) {}
          __forceinline vfloat<M> operator() () const { return ray.tfar[k]; };
        };

        template<typename Epilog>
        static __forceinline bool intersect(const vbool<M>& valid_i,
                                            RayK<K>& ray, size_t k,
                                            RayQueryContext* context,
                                            const LineSegments* geom,
                                            const Precalculations& pre,
                                            const Vec4vf<M>& v0i, const Vec4vf<M>& v1i,
                                            const Vec4vf<M>& vLi, const Vec4vf<M>& vRi,
                                            const Epilog& epilog)
        {
          const Vec3vf<M> ray_org(ray.org.x[k], ray.org.y[k], ray.org.z[k]);
          const Vec3vf<M> ray_dir(ray.dir.x[k], ray.dir.y[k], ray.dir.z[k]);
          const vfloat<M> ray_tnear = ray.tnear()[k];
          const Vec4vf<M> v0 = enlargeRadiusToMinWidth<M>(context,geom,ray_org,v0i);
          const Vec4vf<M> v1 = enlargeRadiusToMinWidth<M>(context,geom,ray_org,v1i);
          const Vec4vf<M> vL = enlargeRadiusToMinWidth<M>(context,geom,ray_org,vLi);
          const Vec4vf<M> vR = enlargeRadiusToMinWidth<M>(context,geom,ray_org,vRi);
          return __roundline_internal::intersectConeSphere<M>(valid_i,ray_org,ray_dir,ray_tnear,ray_tfar(ray,k),v0,v1,vL,vR,epilog);
        }
      };
  }
}
