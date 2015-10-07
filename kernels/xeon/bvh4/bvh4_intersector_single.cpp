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

#include "bvh4_intersector_single.h"
#include "bvh4_intersector1.h"
#include "../geometry/intersector_iterators.h"
#include "../geometry/bezier1v_intersector.h"
#include "../geometry/bezier1i_intersector.h"
#include "../geometry/subdivpatch1_intersector1.h"
#include "../geometry/subdivpatch1cached_intersector1.h"
#include "../geometry/grid_aos_intersector1.h"

namespace embree
{
  namespace isa
  {
    template<int K, int types, bool robust, typename PrimitiveIntersectorK>
    void BVH4IntersectorKSingle<K,types,robust,PrimitiveIntersectorK>::intersect(vint<K>* valid_i, BVH4* bvh, RayK<K>& ray)
    {
      /* verify correct input */
      vbool<K> valid = *valid_i == -1;
#if defined(RTCORE_IGNORE_INVALID_RAYS)
      valid &= ray.valid();
#endif
      assert(all(valid,ray.tnear > -FLT_MIN));

      /* load ray */
      Vec3vfK ray_org = ray.org;
      Vec3vfK ray_dir = ray.dir;
      vfloat<K> ray_tnear = ray.tnear, ray_tfar  = ray.tfar;
      const Vec3vfK rdir = rcp_safe(ray_dir);
      const Vec3vfK org(ray_org), org_rdir = org * rdir;
      ray_tnear = select(valid,ray_tnear,vfloat<K>(pos_inf));
      ray_tfar  = select(valid,ray_tfar ,vfloat<K>(neg_inf));
      const vfloat<K> inf = vfloat<K>(pos_inf);
      Precalculations pre(valid,ray);

      /* compute near/far per ray */
      Vec3viK nearXYZ;
      nearXYZ.x = select(rdir.x >= 0.0f,vint<K>(0*(int)sizeof(vfloat4)),vint<K>(1*(int)sizeof(vfloat4)));
      nearXYZ.y = select(rdir.y >= 0.0f,vint<K>(2*(int)sizeof(vfloat4)),vint<K>(3*(int)sizeof(vfloat4)));
      nearXYZ.z = select(rdir.z >= 0.0f,vint<K>(4*(int)sizeof(vfloat4)),vint<K>(5*(int)sizeof(vfloat4)));

      /* we have no packet implementation for OBB nodes yet */
      size_t bits = movemask(valid);
      for (size_t i=__bsf(bits); bits!=0; bits=__btc(bits,i), i=__bsf(bits)) {
	intersect1(bvh, bvh->root, i, pre, ray, ray_org, ray_dir, rdir, ray_tnear, ray_tfar, nearXYZ);
      }
      AVX_ZERO_UPPER();
    }

    
    template<int K, int types, bool robust, typename PrimitiveIntersectorK>
    void BVH4IntersectorKSingle<K,types,robust,PrimitiveIntersectorK>::occluded(vint<K>* valid_i, BVH4* bvh, RayK<K>& ray)
    {
      /* verify correct input */
      vbool<K> valid = *valid_i == -1;
#if defined(RTCORE_IGNORE_INVALID_RAYS)
      valid &= ray.valid();
#endif
      assert(all(valid,ray.tnear > -FLT_MIN));

      /* load ray */
      vbool<K> terminated = !valid;
      Vec3vfK ray_org = ray.org, ray_dir = ray.dir;
      vfloat<K> ray_tnear = ray.tnear, ray_tfar  = ray.tfar;
      const Vec3vfK rdir = rcp_safe(ray_dir);
      const Vec3vfK org(ray_org), org_rdir = org * rdir;
      ray_tnear = select(valid,ray_tnear,vfloat<K>(pos_inf));
      ray_tfar  = select(valid,ray_tfar ,vfloat<K>(neg_inf));
      const vfloat<K> inf = vfloat<K>(pos_inf);
      Precalculations pre(valid,ray);

      /* compute near/far per ray */
      Vec3viK nearXYZ;
      nearXYZ.x = select(rdir.x >= 0.0f,vint<K>(0*(int)sizeof(vfloat4)),vint<K>(1*(int)sizeof(vfloat4)));
      nearXYZ.y = select(rdir.y >= 0.0f,vint<K>(2*(int)sizeof(vfloat4)),vint<K>(3*(int)sizeof(vfloat4)));
      nearXYZ.z = select(rdir.z >= 0.0f,vint<K>(4*(int)sizeof(vfloat4)),vint<K>(5*(int)sizeof(vfloat4)));

      /* we have no packet implementation for OBB nodes yet */
      size_t bits = movemask(valid);
      for (size_t i=__bsf(bits); bits!=0; bits=__btc(bits,i), i=__bsf(bits)) {
	if (occluded1(bvh,bvh->root,i,pre,ray,ray_org,ray_dir,rdir,ray_tnear,ray_tfar,nearXYZ))
          set(terminated, i);
      }
      vint<K>::store(valid & terminated,&ray.geomID,0);
      AVX_ZERO_UPPER();
    }

    template<int K, typename Intersector1>
    void BVH4IntersectorKFromIntersector1<K,Intersector1>::intersect(vint<K>* valid_i, BVH4* bvh, RayK<K>& ray)
    {
      Ray rays[K];
      ray.get(rays);
      size_t bits = movemask(*valid_i == -1);
      for (size_t i=__bsf(bits); bits!=0; bits=__btc(bits,i), i=__bsf(bits)) {
	Intersector1::intersect(bvh,rays[i]);
      }
      ray.set(rays);
      AVX_ZERO_UPPER();
    }
    
    template<int K, typename Intersector1>
    void BVH4IntersectorKFromIntersector1<K,Intersector1>::occluded(vint<K>* valid_i, BVH4* bvh, RayK<K>& ray)
    {
      Ray rays[K];
      ray.get(rays);
      size_t bits = movemask(*valid_i == -1);
      for (size_t i=__bsf(bits); bits!=0; bits=__btc(bits,i), i=__bsf(bits)) {
	Intersector1::occluded(bvh,rays[i]);
      }
      ray.set(rays);
      AVX_ZERO_UPPER();
    }

    DEFINE_INTERSECTOR4(BVH4Bezier1vIntersector4Single_OBB, BVH4IntersectorKSingle<4 COMMA 0x101 COMMA false COMMA ArrayIntersectorK_1<4 COMMA Bezier1vIntersectorK<4> > >);
    DEFINE_INTERSECTOR4(BVH4Bezier1iIntersector4Single_OBB, BVH4IntersectorKSingle<4 COMMA 0x101 COMMA false COMMA ArrayIntersectorK_1<4 COMMA Bezier1iIntersectorK<4> > >);
    DEFINE_INTERSECTOR4(BVH4Bezier1iMBIntersector4Single_OBB,BVH4IntersectorKSingle<4 COMMA 0x1010 COMMA false COMMA ArrayIntersectorK_1<4 COMMA Bezier1iIntersectorKMB<4> > >);

    DEFINE_INTERSECTOR4(BVH4Subdivpatch1Intersector4, BVH4IntersectorKFromIntersector1<4 COMMA BVH4Intersector1<0x1 COMMA true COMMA ArrayIntersector1<SubdivPatch1Intersector1 > > >);
    DEFINE_INTERSECTOR4(BVH4Subdivpatch1CachedIntersector4,BVH4IntersectorKFromIntersector1<4 COMMA BVH4Intersector1<0x1 COMMA true COMMA SubdivPatch1CachedIntersector1> >);

    DEFINE_INTERSECTOR4(BVH4GridAOSIntersector4, BVH4IntersectorKFromIntersector1<4 COMMA BVH4Intersector1<0x1 COMMA true COMMA GridAOSIntersector1> >);

#if defined(__AVX__)
    DEFINE_INTERSECTOR8(BVH4Bezier1vIntersector8Single_OBB, BVH4IntersectorKSingle<8 COMMA 0x101 COMMA false COMMA ArrayIntersectorK_1<8 COMMA Bezier1vIntersectorK<8> > >);
    DEFINE_INTERSECTOR8(BVH4Bezier1iIntersector8Single_OBB, BVH4IntersectorKSingle<8 COMMA 0x101 COMMA false COMMA ArrayIntersectorK_1<8 COMMA Bezier1iIntersectorK<8> > >);
    DEFINE_INTERSECTOR8(BVH4Bezier1iMBIntersector8Single_OBB,BVH4IntersectorKSingle<8 COMMA 0x1010 COMMA false COMMA ArrayIntersectorK_1<8 COMMA Bezier1iIntersectorKMB<8> > >);

    DEFINE_INTERSECTOR8(BVH4Subdivpatch1Intersector8, BVH4IntersectorKFromIntersector1<8 COMMA BVH4Intersector1<0x1 COMMA true COMMA ArrayIntersector1<SubdivPatch1Intersector1 > > >);
    //DEFINE_INTERSECTOR8(BVH4Subdivpatch1CachedIntersector8,BVH4IntersectorKFromIntersector1<8 COMMA BVH4Intersector1<0x1 COMMA true COMMA SubdivPatch1CachedIntersector1> >);

    DEFINE_INTERSECTOR8(BVH4GridAOSIntersector8, BVH4IntersectorKFromIntersector1<8 COMMA BVH4Intersector1<0x1 COMMA true COMMA GridAOSIntersector1> >);
#endif

#if defined(__AVX512F__)
    DEFINE_INTERSECTOR16(BVH4Bezier1vIntersector16Single_OBB, BVH4IntersectorKSingle<16 COMMA 0x101 COMMA false COMMA ArrayIntersectorK_1<16 COMMA Bezier1vIntersectorK<16> > >);
    DEFINE_INTERSECTOR16(BVH4Bezier1iIntersector16Single_OBB, BVH4IntersectorKSingle<16 COMMA 0x101 COMMA false COMMA ArrayIntersectorK_1<16 COMMA Bezier1iIntersectorK<16> > >);
    DEFINE_INTERSECTOR16(BVH4Bezier1iMBIntersector16Single_OBB,BVH4IntersectorKSingle<16 COMMA 0x1010 COMMA false COMMA ArrayIntersectorK_1<16 COMMA Bezier1iIntersectorKMB<16> > >);

    DEFINE_INTERSECTOR16(BVH4Subdivpatch1Intersector16, BVH4IntersectorKFromIntersector1<16 COMMA BVH4Intersector1<0x1 COMMA true COMMA ArrayIntersector1<SubdivPatch1Intersector1 > > >);
    DEFINE_INTERSECTOR16(BVH4Subdivpatch1CachedIntersector16,BVH4IntersectorKFromIntersector1<16 COMMA BVH4Intersector1<0x1 COMMA true COMMA SubdivPatch1CachedIntersector1> >);

    DEFINE_INTERSECTOR16(BVH4GridAOSIntersector16, BVH4IntersectorKFromIntersector1<16 COMMA BVH4Intersector1<0x1 COMMA true COMMA GridAOSIntersector1> >);
#endif
  }
}
