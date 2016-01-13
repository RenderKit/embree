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

#include "bvh_intersector_single.h"
#include "../geometry/intersector_iterators.h"
#include "../geometry/linei_intersector.h"
#include "../geometry/pointi_intersector.h"
#include "../geometry/bezier1v_intersector.h"
#include "../geometry/bezier1i_intersector.h"
#include "../geometry/subdivpatch1cached_intersector1.h"
#include "../geometry/grid_aos_intersector.h"

namespace embree
{
  namespace isa
  {
    template<int N, int K, int types, bool robust, typename PrimitiveIntersectorK>
    void BVHNIntersectorKSingle<N,K,types,robust,PrimitiveIntersectorK>::intersect(vint<K>* __restrict__ valid_i, BVH* __restrict__ bvh, RayK<K>& __restrict__ ray)
    {
      /* filter out invalid rays */
      vbool<K> valid = *valid_i == -1;
#if defined(RTCORE_IGNORE_INVALID_RAYS)
      valid &= ray.valid();
#endif

      /* verify correct input */
      assert(all(valid,ray.valid()));
      assert(all(valid,ray.tnear >= 0.0f));
      assert(!(types & BVH_MB) || all(valid,ray.time >= 0.0f & ray.time <= 1.0f));

      /* load ray */
      Vec3vfK ray_org = ray.org;
      Vec3vfK ray_dir = ray.dir;
      vfloat<K> ray_tnear = max(ray.tnear,0.0f);
      vfloat<K> ray_tfar  = max(ray.tfar ,0.0f);
      const Vec3vfK rdir = rcp_safe(ray_dir);
      const Vec3vfK org(ray_org), org_rdir = org * rdir;
      ray_tnear = select(valid,ray_tnear,vfloat<K>(pos_inf));
      ray_tfar  = select(valid,ray_tfar ,vfloat<K>(neg_inf));
      const vfloat<K> inf = vfloat<K>(pos_inf);
      Precalculations pre(valid,ray);

      /* compute near/far per ray */
      Vec3viK nearXYZ;
      nearXYZ.x = select(rdir.x >= 0.0f,vint<K>(0*(int)sizeof(vfloat<N>)),vint<K>(1*(int)sizeof(vfloat<N>)));
      nearXYZ.y = select(rdir.y >= 0.0f,vint<K>(2*(int)sizeof(vfloat<N>)),vint<K>(3*(int)sizeof(vfloat<N>)));
      nearXYZ.z = select(rdir.z >= 0.0f,vint<K>(4*(int)sizeof(vfloat<N>)),vint<K>(5*(int)sizeof(vfloat<N>)));

      /* iterates over all rays in the packet using single ray traversal */
      size_t bits = movemask(valid);
      for (size_t i=__bsf(bits); bits!=0; bits=__btc(bits,i), i=__bsf(bits)) {
	intersect1(bvh, bvh->root, i, pre, ray, ray_org, ray_dir, rdir, ray_tnear, ray_tfar, nearXYZ);
      }
      AVX_ZERO_UPPER();
    }

    
    template<int N, int K, int types, bool robust, typename PrimitiveIntersectorK>
    void BVHNIntersectorKSingle<N,K,types,robust,PrimitiveIntersectorK>::occluded(vint<K>* __restrict__ valid_i, BVH* __restrict__ bvh, RayK<K>& __restrict__ ray)
    {
      /* filter out invalid rays */
      vbool<K> valid = *valid_i == -1;
#if defined(RTCORE_IGNORE_INVALID_RAYS)
      valid &= ray.valid();
#endif

      /* verify correct input */
      assert(all(valid,ray.valid()));
      assert(all(valid,ray.tnear >= 0.0f));
      assert(!(types & BVH_MB) || all(valid,ray.time >= 0.0f & ray.time <= 1.0f));

      /* load ray */
      vbool<K> terminated = !valid;
      Vec3vfK ray_org = ray.org, ray_dir = ray.dir;
      vfloat<K> ray_tnear = max(ray.tnear,0.0f);
      vfloat<K> ray_tfar  = max(ray.tfar ,0.0f);
      const Vec3vfK rdir = rcp_safe(ray_dir);
      const Vec3vfK org(ray_org), org_rdir = org * rdir;
      ray_tnear = select(valid,ray_tnear,vfloat<K>(pos_inf));
      ray_tfar  = select(valid,ray_tfar ,vfloat<K>(neg_inf));
      const vfloat<K> inf = vfloat<K>(pos_inf);
      Precalculations pre(valid,ray);

      /* compute near/far per ray */
      Vec3viK nearXYZ;
      nearXYZ.x = select(rdir.x >= 0.0f,vint<K>(0*(int)sizeof(vfloat<N>)),vint<K>(1*(int)sizeof(vfloat<N>)));
      nearXYZ.y = select(rdir.y >= 0.0f,vint<K>(2*(int)sizeof(vfloat<N>)),vint<K>(3*(int)sizeof(vfloat<N>)));
      nearXYZ.z = select(rdir.z >= 0.0f,vint<K>(4*(int)sizeof(vfloat<N>)),vint<K>(5*(int)sizeof(vfloat<N>)));

      /* iterates over all rays in the packet using single ray traversal */
      size_t bits = movemask(valid);
      for (size_t i=__bsf(bits); bits!=0; bits=__btc(bits,i), i=__bsf(bits)) {
	if (occluded1(bvh,bvh->root,i,pre,ray,ray_org,ray_dir,rdir,ray_tnear,ray_tfar,nearXYZ))
          set(terminated, i);
      }
      vint<K>::store(valid & terminated,&ray.geomID,0);
      AVX_ZERO_UPPER();
    }

    ////////////////////////////////////////////////////////////////////////////////
    /// BVH4Intersector4Single Definitions
    ////////////////////////////////////////////////////////////////////////////////

    DEFINE_INTERSECTOR4(BVH4Line4iIntersector4,  BVHNIntersectorKSingle<4 COMMA 4 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<4 COMMA LineMiIntersectorK  <4 COMMA 4 COMMA 4 COMMA true> > >);
    DEFINE_INTERSECTOR4(BVH4Line4iMBIntersector4,BVHNIntersectorKSingle<4 COMMA 4 COMMA BVH_AN2 COMMA false COMMA ArrayIntersectorK_1<4 COMMA LineMiMBIntersectorK<4 COMMA 4 COMMA 4 COMMA true> > >);

    DEFINE_INTERSECTOR4(BVH4Point4iIntersector4,  BVHNIntersectorKSingle<4 COMMA 4 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<4 COMMA PointMiIntersectorK  <4 COMMA 4 COMMA 4 COMMA true> > >);
    DEFINE_INTERSECTOR4(BVH4Point4iMBIntersector4,BVHNIntersectorKSingle<4 COMMA 4 COMMA BVH_AN2 COMMA false COMMA ArrayIntersectorK_1<4 COMMA PointMiMBIntersectorK<4 COMMA 4 COMMA 4 COMMA true> > >);
   
    DEFINE_INTERSECTOR4(BVH4Bezier1vIntersector4Single, BVHNIntersectorKSingle<4 COMMA 4 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<4 COMMA Bezier1vIntersectorK<4> > >);
    DEFINE_INTERSECTOR4(BVH4Bezier1iIntersector4Single, BVHNIntersectorKSingle<4 COMMA 4 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<4 COMMA Bezier1iIntersectorK<4> > >);

    DEFINE_INTERSECTOR4(BVH4Bezier1vIntersector4Single_OBB, BVHNIntersectorKSingle<4 COMMA 4 COMMA BVH_AN1_UN1 COMMA false COMMA ArrayIntersectorK_1<4 COMMA Bezier1vIntersectorK<4> > >);
    DEFINE_INTERSECTOR4(BVH4Bezier1iIntersector4Single_OBB, BVHNIntersectorKSingle<4 COMMA 4 COMMA BVH_AN1_UN1 COMMA false COMMA ArrayIntersectorK_1<4 COMMA Bezier1iIntersectorK<4> > >);
    DEFINE_INTERSECTOR4(BVH4Bezier1iMBIntersector4Single_OBB,BVHNIntersectorKSingle<4 COMMA 4 COMMA BVH_AN2_UN2 COMMA false COMMA ArrayIntersectorK_1<4 COMMA Bezier1iIntersectorKMB<4> > >);

    DEFINE_INTERSECTOR4(BVH4GridAOSIntersector4, BVHNIntersectorKSingle<4 COMMA 4 COMMA BVH_AN1 COMMA true COMMA GridAOSIntersectorK<4> >);

    ////////////////////////////////////////////////////////////////////////////////
    /// BVH4Intersector8Single Definitions
    ////////////////////////////////////////////////////////////////////////////////

#if defined(__AVX__)
    DEFINE_INTERSECTOR8(BVH4Line4iIntersector8,  BVHNIntersectorKSingle<4 COMMA 8 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<8 COMMA LineMiIntersectorK  <4 COMMA 4 COMMA 8 COMMA true> > >);
    DEFINE_INTERSECTOR8(BVH4Line4iMBIntersector8,BVHNIntersectorKSingle<4 COMMA 8 COMMA BVH_AN2 COMMA false COMMA ArrayIntersectorK_1<8 COMMA LineMiMBIntersectorK<4 COMMA 4 COMMA 8 COMMA true> > >);
   
    DEFINE_INTERSECTOR8(BVH4Point4iIntersector8,  BVHNIntersectorKSingle<4 COMMA 8 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<8 COMMA PointMiIntersectorK  <4 COMMA 4 COMMA 8 COMMA true> > >);
    DEFINE_INTERSECTOR8(BVH4Point4iMBIntersector8,BVHNIntersectorKSingle<4 COMMA 8 COMMA BVH_AN2 COMMA false COMMA ArrayIntersectorK_1<8 COMMA PointMiMBIntersectorK<4 COMMA 4 COMMA 8 COMMA true> > >);
   
    DEFINE_INTERSECTOR8(BVH4Bezier1vIntersector8Single, BVHNIntersectorKSingle<4 COMMA 8 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<8 COMMA Bezier1vIntersectorK<8> > >);
    DEFINE_INTERSECTOR8(BVH4Bezier1iIntersector8Single, BVHNIntersectorKSingle<4 COMMA 8 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<8 COMMA Bezier1iIntersectorK<8> > >);

    DEFINE_INTERSECTOR8(BVH4Bezier1vIntersector8Single_OBB, BVHNIntersectorKSingle<4 COMMA 8 COMMA BVH_AN1_UN1 COMMA false COMMA ArrayIntersectorK_1<8 COMMA Bezier1vIntersectorK<8> > >);
    DEFINE_INTERSECTOR8(BVH4Bezier1iIntersector8Single_OBB, BVHNIntersectorKSingle<4 COMMA 8 COMMA BVH_AN1_UN1 COMMA false COMMA ArrayIntersectorK_1<8 COMMA Bezier1iIntersectorK<8> > >);
    DEFINE_INTERSECTOR8(BVH4Bezier1iMBIntersector8Single_OBB,BVHNIntersectorKSingle<4 COMMA 8 COMMA BVH_AN2_UN2 COMMA false COMMA ArrayIntersectorK_1<8 COMMA Bezier1iIntersectorKMB<8> > >);

    DEFINE_INTERSECTOR8(BVH4GridAOSIntersector8, BVHNIntersectorKSingle<4 COMMA 8 COMMA BVH_AN1 COMMA true COMMA GridAOSIntersectorK<8> >);
#endif

    ////////////////////////////////////////////////////////////////////////////////
    /// BVH4Intersector16Single Definitions
    ////////////////////////////////////////////////////////////////////////////////

#if defined(__AVX512F__)
    DEFINE_INTERSECTOR16(BVH4Line4iIntersector16,  BVHNIntersectorKSingle<4 COMMA 16 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<16 COMMA LineMiIntersectorK  <4 COMMA 4 COMMA 16 COMMA true> > >);
    DEFINE_INTERSECTOR16(BVH4Line4iMBIntersector16,BVHNIntersectorKSingle<4 COMMA 16 COMMA BVH_AN2 COMMA false COMMA ArrayIntersectorK_1<16 COMMA LineMiMBIntersectorK<4 COMMA 4 COMMA 16 COMMA true> > >);
   
    DEFINE_INTERSECTOR16(BVH4Point4iIntersector16,  BVHNIntersectorKSingle<4 COMMA 16 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<16 COMMA PointMiIntersectorK  <4 COMMA 4 COMMA 16 COMMA true> > >);
    DEFINE_INTERSECTOR16(BVH4Point4iMBIntersector16,BVHNIntersectorKSingle<4 COMMA 16 COMMA BVH_AN2 COMMA false COMMA ArrayIntersectorK_1<16 COMMA PointMiMBIntersectorK<4 COMMA 4 COMMA 16 COMMA true> > >);
   
    DEFINE_INTERSECTOR16(BVH4Bezier1vIntersector16Single, BVHNIntersectorKSingle<4 COMMA 16 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<16 COMMA Bezier1vIntersectorK<16> > >);
    DEFINE_INTERSECTOR16(BVH4Bezier1iIntersector16Single, BVHNIntersectorKSingle<4 COMMA 16 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<16 COMMA Bezier1iIntersectorK<16> > >);

    DEFINE_INTERSECTOR16(BVH4Bezier1vIntersector16Single_OBB, BVHNIntersectorKSingle<4 COMMA 16 COMMA BVH_AN1_UN1 COMMA false COMMA ArrayIntersectorK_1<16 COMMA Bezier1vIntersectorK<16> > >);
    DEFINE_INTERSECTOR16(BVH4Bezier1iIntersector16Single_OBB, BVHNIntersectorKSingle<4 COMMA 16 COMMA BVH_AN1_UN1 COMMA false COMMA ArrayIntersectorK_1<16 COMMA Bezier1iIntersectorK<16> > >);
    DEFINE_INTERSECTOR16(BVH4Bezier1iMBIntersector16Single_OBB,BVHNIntersectorKSingle<4 COMMA 16 COMMA BVH_AN2_UN2 COMMA false COMMA ArrayIntersectorK_1<16 COMMA Bezier1iIntersectorKMB<16> > >);

    DEFINE_INTERSECTOR16(BVH4GridAOSIntersector16, BVHNIntersectorKSingle<4 COMMA 16 COMMA BVH_AN1 COMMA true COMMA GridAOSIntersectorK<16> >);
#endif

    ////////////////////////////////////////////////////////////////////////////////
    /// BVH8Intersector4Single Definitions
    ////////////////////////////////////////////////////////////////////////////////

#if defined(__AVX__)
    DEFINE_INTERSECTOR4(BVH8Line4iIntersector4,  BVHNIntersectorKSingle<8 COMMA 4 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<4 COMMA LineMiIntersectorK  <4 COMMA 4 COMMA 4 COMMA true> > >);
    DEFINE_INTERSECTOR4(BVH8Line4iMBIntersector4,BVHNIntersectorKSingle<8 COMMA 4 COMMA BVH_AN2 COMMA false COMMA ArrayIntersectorK_1<4 COMMA LineMiMBIntersectorK<4 COMMA 4 COMMA 4 COMMA true> > >);
   
    DEFINE_INTERSECTOR4(BVH8Point4iIntersector4,  BVHNIntersectorKSingle<8 COMMA 4 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<4 COMMA PointMiIntersectorK  <4 COMMA 4 COMMA 4 COMMA true> > >);
    DEFINE_INTERSECTOR4(BVH8Point4iMBIntersector4,BVHNIntersectorKSingle<8 COMMA 4 COMMA BVH_AN2 COMMA false COMMA ArrayIntersectorK_1<4 COMMA PointMiMBIntersectorK<4 COMMA 4 COMMA 4 COMMA true> > >);
   
    DEFINE_INTERSECTOR4(BVH8Bezier1vIntersector4Single_OBB, BVHNIntersectorKSingle<8 COMMA 4 COMMA BVH_AN1_UN1 COMMA false COMMA ArrayIntersectorK_1<4 COMMA Bezier1vIntersectorK<4> > >);
    DEFINE_INTERSECTOR4(BVH8Bezier1iIntersector4Single_OBB, BVHNIntersectorKSingle<8 COMMA 4 COMMA BVH_AN1_UN1 COMMA false COMMA ArrayIntersectorK_1<4 COMMA Bezier1iIntersectorK<4> > >);
    DEFINE_INTERSECTOR4(BVH8Bezier1iMBIntersector4Single_OBB,BVHNIntersectorKSingle<8 COMMA 4 COMMA BVH_AN2_UN2 COMMA false COMMA ArrayIntersectorK_1<4 COMMA Bezier1iIntersectorKMB<4> > >);

    DEFINE_INTERSECTOR4(BVH8GridAOSIntersector4, BVHNIntersectorKSingle<8 COMMA 4 COMMA BVH_AN1 COMMA true COMMA GridAOSIntersectorK<4> >);
#endif

    ////////////////////////////////////////////////////////////////////////////////
    /// BVH8Intersector8Single Definitions
    ////////////////////////////////////////////////////////////////////////////////

#if defined(__AVX__)
    DEFINE_INTERSECTOR8(BVH8Line4iIntersector8,  BVHNIntersectorKSingle<8 COMMA 8 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<8 COMMA LineMiIntersectorK  <4 COMMA 4 COMMA 8 COMMA true> > >);
    DEFINE_INTERSECTOR8(BVH8Line4iMBIntersector8,BVHNIntersectorKSingle<8 COMMA 8 COMMA BVH_AN2 COMMA false COMMA ArrayIntersectorK_1<8 COMMA LineMiMBIntersectorK<4 COMMA 4 COMMA 8 COMMA true> > >);
   
    DEFINE_INTERSECTOR8(BVH8Point4iIntersector8,  BVHNIntersectorKSingle<8 COMMA 8 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<8 COMMA PointMiIntersectorK  <4 COMMA 4 COMMA 8 COMMA true> > >);
    DEFINE_INTERSECTOR8(BVH8Point4iMBIntersector8,BVHNIntersectorKSingle<8 COMMA 8 COMMA BVH_AN2 COMMA false COMMA ArrayIntersectorK_1<8 COMMA PointMiMBIntersectorK<4 COMMA 4 COMMA 8 COMMA true> > >);
   
    DEFINE_INTERSECTOR8(BVH8Bezier1vIntersector8Single_OBB, BVHNIntersectorKSingle<8 COMMA 8 COMMA BVH_AN1_UN1 COMMA false COMMA ArrayIntersectorK_1<8 COMMA Bezier1vIntersectorK<8> > >);
    DEFINE_INTERSECTOR8(BVH8Bezier1iIntersector8Single_OBB, BVHNIntersectorKSingle<8 COMMA 8 COMMA BVH_AN1_UN1 COMMA false COMMA ArrayIntersectorK_1<8 COMMA Bezier1iIntersectorK<8> > >);
    DEFINE_INTERSECTOR8(BVH8Bezier1iMBIntersector8Single_OBB,BVHNIntersectorKSingle<8 COMMA 8 COMMA BVH_AN2_UN2 COMMA false COMMA ArrayIntersectorK_1<8 COMMA Bezier1iIntersectorKMB<8> > >);

    DEFINE_INTERSECTOR8(BVH8GridAOSIntersector8, BVHNIntersectorKSingle<8 COMMA 8 COMMA BVH_AN1 COMMA true COMMA GridAOSIntersectorK<8> >);
#endif

    ////////////////////////////////////////////////////////////////////////////////
    /// BVH8Intersector16Single Definitions
    ////////////////////////////////////////////////////////////////////////////////

#if defined(__AVX512F__)
    DEFINE_INTERSECTOR16(BVH8Line4iIntersector16,  BVHNIntersectorKSingle<8 COMMA 16 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<16 COMMA LineMiIntersectorK  <4 COMMA 4 COMMA 16 COMMA true> > >);
    DEFINE_INTERSECTOR16(BVH8Line4iMBIntersector16,BVHNIntersectorKSingle<8 COMMA 16 COMMA BVH_AN2 COMMA false COMMA ArrayIntersectorK_1<16 COMMA LineMiMBIntersectorK<4 COMMA 4 COMMA 16 COMMA true> > >);
   
    DEFINE_INTERSECTOR16(BVH8Point4iIntersector16,  BVHNIntersectorKSingle<8 COMMA 16 COMMA BVH_AN1 COMMA false COMMA ArrayIntersectorK_1<16 COMMA PointMiIntersectorK  <4 COMMA 4 COMMA 16 COMMA true> > >);
    DEFINE_INTERSECTOR16(BVH8Point4iMBIntersector16,BVHNIntersectorKSingle<8 COMMA 16 COMMA BVH_AN2 COMMA false COMMA ArrayIntersectorK_1<16 COMMA PointMiMBIntersectorK<4 COMMA 4 COMMA 16 COMMA true> > >);
   
    DEFINE_INTERSECTOR16(BVH8Bezier1vIntersector16Single_OBB, BVHNIntersectorKSingle<8 COMMA 16 COMMA BVH_AN1_UN1 COMMA false COMMA ArrayIntersectorK_1<16 COMMA Bezier1vIntersectorK<16> > >);
    DEFINE_INTERSECTOR16(BVH8Bezier1iIntersector16Single_OBB, BVHNIntersectorKSingle<8 COMMA 16 COMMA BVH_AN1_UN1 COMMA false COMMA ArrayIntersectorK_1<16 COMMA Bezier1iIntersectorK<16> > >);
    DEFINE_INTERSECTOR16(BVH8Bezier1iMBIntersector16Single_OBB,BVHNIntersectorKSingle<8 COMMA 16 COMMA BVH_AN2_UN2 COMMA false COMMA ArrayIntersectorK_1<16 COMMA Bezier1iIntersectorKMB<16> > >);

    DEFINE_INTERSECTOR16(BVH8GridAOSIntersector16, BVHNIntersectorKSingle<8 COMMA 16 COMMA BVH_AN1 COMMA true COMMA GridAOSIntersectorK<16> >);
#endif
  }
}
