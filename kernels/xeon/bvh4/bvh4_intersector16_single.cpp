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

#include "bvh4_intersector16_single.h"
#include "bvh4_intersector1.h"
#include "../geometry/intersector_iterators.h"
#include "../geometry/bezier1v_intersector.h"
#include "../geometry/bezier1i_intersector.h"
#include "../geometry/subdivpatch1_intersector1.h"
#include "../geometry/subdivpatch1cached_intersector1.h"
#include "../geometry/grid_soa_intersector1.h"

namespace embree
{
  namespace isa
  {
    template<int types, bool robust, typename PrimitiveIntersector16>
    void BVH4Intersector16Single<types,robust,PrimitiveIntersector16>::intersect(int16* valid_i, BVH4* bvh, Ray16& ray)
    {
      /* verify correct input */
      bool16 valid0 = *valid_i == -1;
#if defined(RTCORE_IGNORE_INVALID_RAYS)
      valid0 &= ray.valid();
#endif
      assert(all(valid0,ray.tnear > -FLT_MIN));

      /* load ray */
      Vec3f16 ray_org = ray.org;
      Vec3f16 ray_dir = ray.dir;
      float16 ray_tnear = ray.tnear, ray_tfar  = ray.tfar;
      const Vec3f16 rdir = rcp_safe(ray_dir);
      const Vec3f16 org(ray_org), org_rdir = org * rdir;
      ray_tnear = select(valid0,ray_tnear,float16(pos_inf));
      ray_tfar  = select(valid0,ray_tfar ,float16(neg_inf));
      const float16 inf = float16(pos_inf);
      Precalculations pre(valid0,ray);

      /* compute near/far per ray */
      Vec3i16 nearXYZ;
      nearXYZ.x = select(rdir.x >= 0.0f,int16(0*(int)sizeof(float4)),int16(1*(int)sizeof(float4)));
      nearXYZ.y = select(rdir.y >= 0.0f,int16(2*(int)sizeof(float4)),int16(3*(int)sizeof(float4)));
      nearXYZ.z = select(rdir.z >= 0.0f,int16(4*(int)sizeof(float4)),int16(5*(int)sizeof(float4)));

      /* we have no packet implementation for OBB nodes yet */
      size_t bits = movemask(valid0);
      for (size_t i=__bsf(bits); bits!=0; bits=__btc(bits,i), i=__bsf(bits)) {
	intersect1(bvh, bvh->root, i, pre, ray, ray_org, ray_dir, rdir, ray_tnear, ray_tfar, nearXYZ);
      }
      AVX_ZERO_UPPER();
    }
    
    template<int types, bool robust, typename PrimitiveIntersector16>
    void BVH4Intersector16Single<types,robust, PrimitiveIntersector16>::occluded(int16* valid_i, BVH4* bvh, Ray16& ray)
    {
      /* verify correct input */
      bool16 valid = *valid_i == -1;
#if defined(RTCORE_IGNORE_INVALID_RAYS)
      valid &= ray.valid();
#endif
      assert(all(valid,ray.tnear > -FLT_MIN));

      /* load ray */
      bool16 terminated = !valid;
      Vec3f16 ray_org = ray.org, ray_dir = ray.dir;
      float16 ray_tnear = ray.tnear, ray_tfar  = ray.tfar;
      const Vec3f16 rdir = rcp_safe(ray_dir);
      const Vec3f16 org(ray_org), org_rdir = org * rdir;
      ray_tnear = select(valid,ray_tnear,float16(pos_inf));
      ray_tfar  = select(valid,ray_tfar ,float16(neg_inf));
      const float16 inf = float16(pos_inf);
      Precalculations pre(valid,ray);

      /* compute near/far per ray */
      Vec3i16 nearXYZ;
      nearXYZ.x = select(rdir.x >= 0.0f,int16(0*(int)sizeof(float4)),int16(1*(int)sizeof(float4)));
      nearXYZ.y = select(rdir.y >= 0.0f,int16(2*(int)sizeof(float4)),int16(3*(int)sizeof(float4)));
      nearXYZ.z = select(rdir.z >= 0.0f,int16(4*(int)sizeof(float4)),int16(5*(int)sizeof(float4)));

      /* we have no packet implementation for OBB nodes yet */
      size_t bits = movemask(valid);
      for (size_t i=__bsf(bits); bits!=0; bits=__btc(bits,i), i=__bsf(bits)) {
	if (occluded1(bvh,bvh->root,i,pre,ray,ray_org,ray_dir,rdir,ray_tnear,ray_tfar,nearXYZ))
          terminated |= 1 << i;
	  //terminated[i] = -1;
      }
      store16i(valid & terminated,&ray.geomID,0);
      AVX_ZERO_UPPER();
    }

    template<typename Intersector1>
    void BVH4Intersector16FromIntersector1<Intersector1>::intersect(int16* valid_i, BVH4* bvh, Ray16& ray)
    {
      Ray rays[16];
      ray.get(rays);
      size_t bits = movemask(*valid_i == -1);
      for (size_t i=__bsf(bits); bits!=0; bits=__btc(bits,i), i=__bsf(bits)) {
	Intersector1::intersect(bvh,rays[i]);
      }
      ray.set(rays);
      AVX_ZERO_UPPER();
    }
    
    template<typename Intersector1>
    void BVH4Intersector16FromIntersector1<Intersector1>::occluded(int16* valid_i, BVH4* bvh, Ray16& ray)
    {
      Ray rays[16];
      ray.get(rays);
      size_t bits = movemask(*valid_i == -1);
      for (size_t i=__bsf(bits); bits!=0; bits=__btc(bits,i), i=__bsf(bits)) {
	Intersector1::occluded(bvh,rays[i]);
      }
      ray.set(rays);
      AVX_ZERO_UPPER();
    }
    
    DEFINE_INTERSECTOR16(BVH4Bezier1vIntersector16Single_OBB, BVH4Intersector16Single<0x101 COMMA false COMMA ArrayIntersector16_1<Bezier1vIntersectorN<Ray16> > >);
    DEFINE_INTERSECTOR16(BVH4Bezier1iIntersector16Single_OBB, BVH4Intersector16Single<0x101 COMMA false COMMA ArrayIntersector16_1<Bezier1iIntersectorN<Ray16> > >);
    DEFINE_INTERSECTOR16(BVH4Bezier1iMBIntersector16Single_OBB,BVH4Intersector16Single<0x1010 COMMA false COMMA ArrayIntersector16_1<Bezier1iIntersectorNMB<Ray16> > >);

    DEFINE_INTERSECTOR16(BVH4Subdivpatch1Intersector16, BVH4Intersector16FromIntersector1<BVH4Intersector1<0x1 COMMA true COMMA ArrayIntersector1<SubdivPatch1Intersector1 > > >);
    DEFINE_INTERSECTOR16(BVH4Subdivpatch1CachedIntersector16,BVH4Intersector16FromIntersector1<BVH4Intersector1<0x1 COMMA true COMMA SubdivPatch1CachedIntersector1> >);

    DEFINE_INTERSECTOR16(BVH4GridAOSIntersector16, BVH4Intersector16FromIntersector1<BVH4Intersector1<0x1 COMMA true COMMA GridAOSIntersector1> >);
  }
}
