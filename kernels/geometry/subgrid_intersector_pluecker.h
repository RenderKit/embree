// Copyright 2009-2020 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "subgrid.h"
#include "quad_intersector_moeller.h"
#include "quad_intersector_pluecker.h"

namespace embree
{
  namespace isa
  {

    template<int M>
    __forceinline void interpolateUV(PlueckerHitM<M,UVIdentity<M>> &hit,const GridMesh::Grid &g, const SubGrid& subgrid, const vint<M> &stepX, const vint<M> &stepY) 
    {
      /* correct U,V interpolation across the entire grid */
      const vint<M> sx((int)subgrid.x());
      const vint<M> sy((int)subgrid.y());
      const vint<M> sxM(sx + stepX);
      const vint<M> syM(sy + stepY);
      const float inv_resX = rcp((float)((int)g.resX-1));
      const float inv_resY = rcp((float)((int)g.resY-1));          
      hit.U = (hit.U + vfloat<M>(sxM) * hit.UVW) * inv_resX;
      hit.V = (hit.V + vfloat<M>(syM) * hit.UVW) * inv_resY;
    }
    
    template<int M, bool filter>
      struct SubGridQuadMIntersector1Pluecker;

    template<int M, bool filter>
      struct SubGridQuadMIntersector1Pluecker
      {
        __forceinline SubGridQuadMIntersector1Pluecker() {}

        __forceinline SubGridQuadMIntersector1Pluecker(const Ray& ray, const void* ptr) {}

        __forceinline void intersect(RayHit& ray, IntersectContext* context,
                                     const Vec3vf<M>& v0, const Vec3vf<M>& v1, const Vec3vf<M>& v2, const Vec3vf<M>& v3,
                                     const GridMesh::Grid &g, const SubGrid& subgrid) const
        {
          UVIdentity<M> mapUV;
          PlueckerHitM<M,UVIdentity<M>> hit(mapUV);
          PlueckerIntersector1<M> intersector(ray,nullptr);
	  
          Intersect1EpilogMU<M,filter> epilog(ray,context,subgrid.geomID(),subgrid.primID());

          /* intersect first triangle */
	  if (intersector.intersect(ray,v0,v1,v3,mapUV,hit)) 
          {
            interpolateUV<M>(hit,g,subgrid,vint<M>(0,1,1,0),vint<M>(0,0,1,1));
            epilog(hit.valid,hit);
          }

          /* intersect second triangle */
	  if (intersector.intersect(ray,v2,v3,v1,mapUV,hit)) 
          {
	    hit.U = hit.UVW - hit.U;
	    hit.V = hit.UVW - hit.V;
            interpolateUV<M>(hit,g,subgrid,vint<M>(0,1,1,0),vint<M>(0,0,1,1));
            epilog(hit.valid,hit);
          }
        }
      
        __forceinline bool occluded(Ray& ray, IntersectContext* context,
                                    const Vec3vf<M>& v0, const Vec3vf<M>& v1, const Vec3vf<M>& v2, const Vec3vf<M>& v3,
                                    const GridMesh::Grid &g, const SubGrid& subgrid) const
        {
          UVIdentity<M> mapUV;
          PlueckerHitM<M,UVIdentity<M>> hit(mapUV);
          PlueckerIntersector1<M> intersector(ray,nullptr);
          Occluded1EpilogMU<M,filter> epilog(ray,context,subgrid.geomID(),subgrid.primID());

          /* intersect first triangle */
	  if (intersector.intersect(ray,v0,v1,v3,mapUV,hit)) 
          {
            interpolateUV<M>(hit,g,subgrid,vint<M>(0,1,1,0),vint<M>(0,0,1,1));
            if (epilog(hit.valid,hit))
	      return true;
          }

          /* intersect second triangle */
	  if (intersector.intersect(ray,v2,v3,v1,mapUV,hit)) 
          {
	    hit.U = hit.UVW - hit.U;
	    hit.V = hit.UVW - hit.V;
            interpolateUV<M>(hit,g,subgrid,vint<M>(0,1,1,0),vint<M>(0,0,1,1));
            if (epilog(hit.valid,hit))
	      return true;
          }
          return false;
        }
      };

#if defined (__AVX__)

    /*! Intersects 4 quads with 1 ray using AVX */
    template<bool filter>
      struct SubGridQuadMIntersector1Pluecker<4,filter>
    {
      __forceinline SubGridQuadMIntersector1Pluecker() {}

      __forceinline SubGridQuadMIntersector1Pluecker(const Ray& ray, const void* ptr) {}
      
      template<typename Epilog>
        __forceinline bool intersect(Ray& ray, const Vec3vf4& v0, const Vec3vf4& v1, const Vec3vf4& v2, const Vec3vf4& v3, const GridMesh::Grid &g, const SubGrid& subgrid, const Epilog& epilog) const
      {
        const Vec3vf8 vtx0(vfloat8(v0.x,v2.x),vfloat8(v0.y,v2.y),vfloat8(v0.z,v2.z));
#if !defined(EMBREE_BACKFACE_CULLING)
        const Vec3vf8 vtx1(vfloat8(v1.x),vfloat8(v1.y),vfloat8(v1.z));
        const Vec3vf8 vtx2(vfloat8(v3.x),vfloat8(v3.y),vfloat8(v3.z));        
#else
        const Vec3vf8 vtx1(vfloat8(v1.x,v3.x),vfloat8(v1.y,v3.y),vfloat8(v1.z,v3.z));
        const Vec3vf8 vtx2(vfloat8(v3.x,v1.x),vfloat8(v3.y,v1.y),vfloat8(v3.z,v1.z));
#endif

        UVIdentity<8> mapUV;
        PlueckerHitM<8,UVIdentity<8>> hit(mapUV);
        PlueckerIntersector1<8> intersector(ray,nullptr);
        //const vbool8 flags(0,0,0,0,1,1,1,1);
        if (unlikely(intersector.intersect(ray,vtx0,vtx1,vtx2,mapUV,hit)))
        {
	  /* correct U,V interpolation across the entire grid */
          interpolateUV<8>(hit,g,subgrid,vint<8>(0,1,1,0,0,1,1,0),vint<8>(0,0,1,1,0,0,1,1));
          if (unlikely(epilog(hit.valid,hit)))
            return true;
        }
        return false;
      }
      
      __forceinline bool intersect(RayHit& ray, IntersectContext* context,
                                   const Vec3vf4& v0, const Vec3vf4& v1, const Vec3vf4& v2, const Vec3vf4& v3, 
                                   const GridMesh::Grid &g, const SubGrid& subgrid) const
      {
          return intersect(ray,v0,v1,v2,v3,g,subgrid,Intersect1EpilogMU<8,filter>(ray,context,subgrid.geomID(),subgrid.primID()));
      }
      
      __forceinline bool occluded(Ray& ray, IntersectContext* context,
                                  const Vec3vf4& v0, const Vec3vf4& v1, const Vec3vf4& v2, const Vec3vf4& v3, 
                                  const GridMesh::Grid &g, const SubGrid& subgrid) const
      {
          return intersect(ray,v0,v1,v2,v3,g,subgrid,Occluded1EpilogMU<8,filter>(ray,context,subgrid.geomID(),subgrid.primID()));
      }
    };

#endif


    /* ----------------------------- */
    /* -- ray packet intersectors -- */
    /* ----------------------------- */

    template<int K>
      struct SubGridQuadHitPlueckerK
      {
         __forceinline SubGridQuadHitPlueckerK(const vfloat<K>& U,
                                               const vfloat<K>& V,
                                               const vfloat<K>& UVW,
                                               const vfloat<K>& t,
                                               const Vec3vf<K>& Ng,
                                               const vbool<K>& flags,
                                               const GridMesh::Grid &g, 
                                               const SubGrid& subgrid,
                                               const unsigned int i)
         : U(U), V(V), UVW(UVW), t(t), flags(flags), tri_Ng(Ng), g(g), subgrid(subgrid), i(i) {}

        __forceinline std::tuple<vfloat<K>,vfloat<K>,vfloat<K>,Vec3vf<K>> operator() () const
        {
          const vbool<K> invalid = abs(UVW) < min_rcp_input;
          const vfloat<K> rcpUVW = select(invalid,vfloat<K>(0.0f),rcp(UVW));
          const vfloat<K> u0 = min(U * rcpUVW,1.0f);
          const vfloat<K> v0 = min(V * rcpUVW,1.0f);
          const vfloat<K> u1 = vfloat<K>(1.0f) - u0;
          const vfloat<K> v1 = vfloat<K>(1.0f) - v0;
          const vfloat<K> uu = select(flags,u1,u0);
          const vfloat<K> vv = select(flags,v1,v0);
          const unsigned int sx = subgrid.x() + (unsigned int)(i % 2);
          const unsigned int sy = subgrid.y() + (unsigned int)(i >>1);
          const float inv_resX = rcp((float)(int)(g.resX-1));
          const float inv_resY = rcp((float)(int)(g.resY-1));
          const vfloat<K> u = (uu + (float)(int)sx) * inv_resX;
          const vfloat<K> v = (vv + (float)(int)sy) * inv_resY;
          const Vec3vf<K> Ng(tri_Ng.x,tri_Ng.y,tri_Ng.z);
          return std::make_tuple(u,v,t,Ng);
        }

      private:
        const vfloat<K> U;
        const vfloat<K> V;
        const vfloat<K> UVW;
        const vfloat<K> t;
        const vfloat<K> absDen;
        const vbool<K> flags;
        const Vec3vf<K> tri_Ng;

        const GridMesh::Grid &g;
        const SubGrid& subgrid;
        const size_t i;
      };


    template<int K>
    __forceinline void interpolateUV(const vbool<K>& valid, PlueckerHitK<K,UVIdentity<K>> &hit,const GridMesh::Grid &g, const SubGrid& subgrid, const unsigned int i) 
    {
      /* correct U,V interpolation across the entire grid */
      const unsigned int sx = subgrid.x() + (unsigned int)(i % 2);
      const unsigned int sy = subgrid.y() + (unsigned int)(i >>1);
      const float inv_resX = rcp((float)(int)(g.resX-1));
      const float inv_resY = rcp((float)(int)(g.resY-1));      
      hit.U = select(valid,(hit.U + vfloat<K>((float)sx) * hit.UVW) * inv_resX,hit.U);
      hit.V = select(valid,(hit.V + vfloat<K>((float)sy) * hit.UVW) * inv_resY,hit.V);
    }
    

    template<int M, int K, bool filter>
      struct SubGridQuadMIntersectorKPlueckerBase
      {
        __forceinline SubGridQuadMIntersectorKPlueckerBase(const vbool<K>& valid, const RayK<K>& ray) {}

#if 1	
        template<typename Epilog>
        __forceinline vbool<K> intersectK(const vbool<K>& valid0,
                                          RayK<K>& ray,
                                          const Vec3vf<K>& tri_v0,
                                          const Vec3vf<K>& tri_v1,
                                          const Vec3vf<K>& tri_v2,
                                          const vbool<K>& flags,
                                          const GridMesh::Grid &g, 
                                          const SubGrid &subgrid,
                                          const unsigned int i,
                                          const Epilog& epilog) const
        { 
	  /* calculate vertices relative to ray origin */
          vbool<K> valid = valid0;
          const Vec3vf<K> O = ray.org;
          const Vec3vf<K> D = ray.dir;
          const Vec3vf<K> v0 = tri_v0-O;
          const Vec3vf<K> v1 = tri_v1-O;
          const Vec3vf<K> v2 = tri_v2-O;
          
          /* calculate triangle edges */
          const Vec3vf<K> e0 = v2-v0;
          const Vec3vf<K> e1 = v0-v1;
          const Vec3vf<K> e2 = v1-v2;
           
          /* perform edge tests */
          const vfloat<K> U = dot(Vec3vf<K>(cross(e0,v2+v0)),D);
          const vfloat<K> V = dot(Vec3vf<K>(cross(e1,v0+v1)),D);
          const vfloat<K> W = dot(Vec3vf<K>(cross(e2,v1+v2)),D);
          const vfloat<K> UVW = U+V+W;
          const vfloat<K> eps = float(ulp)*abs(UVW);
#if defined(EMBREE_BACKFACE_CULLING)
          valid &= max(U,V,W) <= eps;
#else
          valid &= (min(U,V,W) >= -eps) | (max(U,V,W) <= eps);
#endif
          if (unlikely(none(valid))) return false;
          
          /* calculate geometry normal and denominator */
          const Vec3vf<K> Ng = stable_triangle_normal(e0,e1,e2);
          const vfloat<K> den = twice(dot(Vec3vf<K>(Ng),D));

          /* perform depth test */
          const vfloat<K> T = twice(dot(v0,Vec3vf<K>(Ng)));
          const vfloat<K> t = rcp(den)*T;
          valid &= ray.tnear() <= t & t <= ray.tfar;
          valid &= den != vfloat<K>(zero);
          if (unlikely(none(valid))) return false;
          
          /* calculate hit information */
          SubGridQuadHitPlueckerK<K> hit(U,V,UVW,t,Ng,flags,g,subgrid,i);
          return epilog(valid,hit);
        }
#endif      
        template<typename Epilog>
        __forceinline bool intersectK(const vbool<K>& valid, 
                                      RayK<K>& ray,
                                      const Vec3vf<K>& v0,
                                      const Vec3vf<K>& v1,
                                      const Vec3vf<K>& v2,
                                      const Vec3vf<K>& v3,
                                      const GridMesh::Grid &g, 
                                      const SubGrid &subgrid,
                                      const unsigned int i,
                                      const Epilog& epilog) const
        {
	  UVIdentity<K> mapUV;
	  PlueckerHitK<K,UVIdentity<K>> hit(mapUV);
	  PlueckerIntersectorK<M,K> intersector;

#if 1
          const vbool<K> valid0 = intersector.intersectK(valid,ray,v0,v1,v3,mapUV,hit);
	  if (any(valid0))
	    {
	      interpolateUV(valid0,hit,g,subgrid,i);
	      epilog(valid0,hit);
	    }
          const vbool<K> valid1 = intersector.intersectK(valid,ray,v2,v3,v1,mapUV,hit);
	  if (any(valid1))
	    {
	      hit.U = hit.UVW - hit.U;
	      hit.V = hit.UVW - hit.V;	      
	      interpolateUV(valid1,hit,g,subgrid,i);
	      epilog(valid1,hit);
	    }
	  return any(valid0|valid1);
#else    
          intersectK(valid,ray,v0,v1,v3,vbool<K>(false),g,subgrid,i,epilog);
          if (none(valid)) return true;
          intersectK(valid,ray,v2,v3,v1,vbool<K>(true ),g,subgrid,i,epilog);
          return none(valid);
#endif
	  
        }

       template<typename Epilog>
        __forceinline bool occludedK(const vbool<K>& valid, 
				     RayK<K>& ray,
				     const Vec3vf<K>& v0,
				     const Vec3vf<K>& v1,
				     const Vec3vf<K>& v2,
				     const Vec3vf<K>& v3,
				     const GridMesh::Grid &g, 
				     const SubGrid &subgrid,
				     const unsigned int i,
				     const Epilog& epilog) const
        {
	  UVIdentity<K> mapUV;
	  PlueckerHitK<K,UVIdentity<K>> hit(mapUV);
	  PlueckerIntersectorK<M,K> intersector;

	  vbool<K> valid_final = valid;
          const vbool<K> valid0 = intersector.intersectK(valid,ray,v0,v1,v3,mapUV,hit);
	  if (any(valid0))
	    {
	      interpolateUV(valid0,hit,g,subgrid,i);
	      epilog(valid0,hit);
	      valid_final &= !valid0;
	    }
	  if (none(valid_final)) return true;	      	  
          const vbool<K> valid1 = intersector.intersectK(valid,ray,v2,v3,v1,mapUV,hit);
	  if (any(valid1))
	    {
	      hit.U = hit.UVW - hit.U;
	      hit.V = hit.UVW - hit.V;	      
	      interpolateUV(valid1,hit,g,subgrid,i);
	      epilog(valid1,hit);
	      valid_final &= !valid1;	      
	    }
	  return none(valid_final);
        }

	
      };


    

    template<int M, int K, bool filter>
      struct SubGridQuadMIntersectorKPluecker : public SubGridQuadMIntersectorKPlueckerBase<M,K,filter>
    {
      __forceinline SubGridQuadMIntersectorKPluecker(const vbool<K>& valid, const RayK<K>& ray)
        : SubGridQuadMIntersectorKPlueckerBase<M,K,filter>(valid,ray) {}

      __forceinline void intersect1(RayHitK<K>& ray, size_t k, IntersectContext* context,
                                    const Vec3vf<M>& v0, const Vec3vf<M>& v1, const Vec3vf<M>& v2, const Vec3vf<M>& v3, const GridMesh::Grid &g, const SubGrid &subgrid) const
      {
	UVIdentity<M> mapUV;
	PlueckerHitM<M,UVIdentity<M>> hit(mapUV);
        Intersect1KEpilogMU<M,K,filter> epilog(ray,k,context,subgrid.geomID(),subgrid.primID());
	PlueckerIntersectorK<M,K> intersector;
	
	/* intersect first triangle */
	if (intersector.intersect(ray,k,v0,v1,v3,mapUV,hit)) 
          {
            interpolateUV<M>(hit,g,subgrid,vint<M>(0,1,1,0),vint<M>(0,0,1,1));
            epilog(hit.valid,hit);
          }

	/* intersect second triangle */
	if (intersector.intersect(ray,k,v2,v3,v1,mapUV,hit)) 
          {
	    hit.U = hit.UVW - hit.U;
	    hit.V = hit.UVW - hit.V;
            interpolateUV<M>(hit,g,subgrid,vint<M>(0,1,1,0),vint<M>(0,0,1,1));
            epilog(hit.valid,hit);
          }
      }
      
      __forceinline bool occluded1(RayK<K>& ray, size_t k, IntersectContext* context,
                                   const Vec3vf<M>& v0, const Vec3vf<M>& v1, const Vec3vf<M>& v2, const Vec3vf<M>& v3, const GridMesh::Grid &g, const SubGrid &subgrid) const
      {
	UVIdentity<M> mapUV;
	PlueckerHitM<M,UVIdentity<M>> hit(mapUV);
        Occluded1KEpilogMU<M,K,filter> epilog(ray,k,context,subgrid.geomID(),subgrid.primID());	
	PlueckerIntersectorK<M,K> intersector;
	
	/* intersect first triangle */
	if (intersector.intersect(ray,k,v0,v1,v3,mapUV,hit)) 
        {
          interpolateUV<M>(hit,g,subgrid,vint<M>(0,1,1,0),vint<M>(0,0,1,1));
          if (epilog(hit.valid,hit)) return true;
        }

	/* intersect second triangle */
	if (intersector.intersect(ray,k,v2,v3,v1,mapUV,hit)) 
        {
          interpolateUV<M>(hit,g,subgrid,vint<M>(0,1,1,0),vint<M>(0,0,1,1));
          if (epilog(hit.valid,hit)) return true;
        }
        return false;
      }
    };


#if defined (__AVX__)

    /*! Intersects 4 quads with 1 ray using AVX */
    template<int K, bool filter>
      struct SubGridQuadMIntersectorKPluecker<4,K,filter> : public SubGridQuadMIntersectorKPlueckerBase<4,K,filter>
    {
      __forceinline SubGridQuadMIntersectorKPluecker(const vbool<K>& valid, const RayK<K>& ray)
        : SubGridQuadMIntersectorKPlueckerBase<4,K,filter>(valid,ray) {}
      
      template<typename Epilog>
        __forceinline bool intersect1(RayK<K>& ray, size_t k,const Vec3vf4& v0, const Vec3vf4& v1, const Vec3vf4& v2, const Vec3vf4& v3, 
                                      const GridMesh::Grid &g, const SubGrid &subgrid, const Epilog& epilog) const
      {
        const Vec3vf8 vtx0(vfloat8(v0.x,v2.x),vfloat8(v0.y,v2.y),vfloat8(v0.z,v2.z));
#if !defined(EMBREE_BACKFACE_CULLING)
        const Vec3vf8 vtx1(vfloat8(v1.x),vfloat8(v1.y),vfloat8(v1.z));
        const Vec3vf8 vtx2(vfloat8(v3.x),vfloat8(v3.y),vfloat8(v3.z));
#else
        const Vec3vf8 vtx1(vfloat8(v1.x,v3.x),vfloat8(v1.y,v3.y),vfloat8(v1.z,v3.z));
        const Vec3vf8 vtx2(vfloat8(v3.x,v1.x),vfloat8(v3.y,v1.y),vfloat8(v3.z,v1.z));
#endif
	UVIdentity<8> mapUV;
	PlueckerHitM<8,UVIdentity<8>> hit(mapUV);
	PlueckerIntersectorK<8,K> intersector;
	
        if (unlikely(intersector.intersect(ray,k,vtx0,vtx1,vtx2,mapUV,hit)))	
        {
          /* correct U,V interpolation across the entire grid */
          interpolateUV<8>(hit,g,subgrid,vint<8>(0,1,1,0,0,1,1,0),vint<8>(0,0,1,1,0,0,1,1));
          if (unlikely(epilog(hit.valid,hit)))
            return true;
        }
        return false;
      }
      
      __forceinline bool intersect1(RayHitK<K>& ray, size_t k, IntersectContext* context,
                                    const Vec3vf4& v0, const Vec3vf4& v1, const Vec3vf4& v2, const Vec3vf4& v3, const GridMesh::Grid &g, const SubGrid &subgrid) const
      {
        return intersect1(ray,k,v0,v1,v2,v3,g,subgrid,Intersect1KEpilogMU<8,K,filter>(ray,k,context,subgrid.geomID(),subgrid.primID()));
      }
      
      __forceinline bool occluded1(RayK<K>& ray, size_t k, IntersectContext* context,
                                   const Vec3vf4& v0, const Vec3vf4& v1, const Vec3vf4& v2, const Vec3vf4& v3, const GridMesh::Grid &g, const SubGrid &subgrid) const
      {
        return intersect1(ray,k,v0,v1,v2,v3,g,subgrid,Occluded1KEpilogMU<8,K,filter>(ray,k,context,subgrid.geomID(),subgrid.primID()));
      }
    };
#endif

    
  }
}
