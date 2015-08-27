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

#include "bvh4_intersector8_test.h"
#include "bvh4_intersector8_single.h"
#include "bvh4_intersector_node.h"

#include "../geometry/triangle4.h"
#include "../geometry/triangle4v.h"
#include "../geometry/triangle8.h"
#include "../geometry/intersector_iterators.h"
#include "../geometry/triangle_intersector_moeller.h"
#include "../geometry/triangle_intersector_pluecker.h"
#include "../geometry/subdivpatch1cached_intersector1.h"

#define SWITCH_THRESHOLD 5
#define SWITCH_DURING_DOWN_TRAVERSAL 1

namespace embree
{
    /* ----------------- */
    /* -- 0 0 0 1 1 2 -- */
    /* -- 1 2 3 2 3 3 -- */
    /* ----------------- */

    __forceinline unsigned int countBits3(const unsigned int bits, const unsigned int flip_mask, const unsigned int and_mask)
    {
      return __popcnt((bits ^ flip_mask) & and_mask);

    }
      
    __forceinline int4 cmpT(unsigned int bits)
    {
      const unsigned int count0 = countBits3(bits,0b000000,0b000111);
      const unsigned int count1 = countBits3(bits,0b000001,0b011001);
      const unsigned int count2 = countBits3(bits,0b001010,0b101010);
      const unsigned int count3 = countBits3(bits,0b110100,0b110100);

      return int4(count0,count1,count2,count3);
    }

    static int4 compareTable[64] = {
      cmpT(0*8+0), cmpT(0*8+1), cmpT(0*8+2), cmpT(0*8+3), cmpT(0*8+4), cmpT(0*8+5), cmpT(0*8+6), cmpT(0*8+7),
      cmpT(1*8+0), cmpT(1*8+1), cmpT(1*8+2), cmpT(1*8+3), cmpT(1*8+4), cmpT(1*8+5), cmpT(1*8+6), cmpT(1*8+7),
      cmpT(2*8+0), cmpT(2*8+1), cmpT(2*8+2), cmpT(2*8+3), cmpT(2*8+4), cmpT(2*8+5), cmpT(2*8+6), cmpT(2*8+7),
      cmpT(3*8+0), cmpT(3*8+1), cmpT(3*8+2), cmpT(3*8+3), cmpT(3*8+4), cmpT(3*8+5), cmpT(3*8+6), cmpT(3*8+7),
      cmpT(4*8+0), cmpT(4*8+1), cmpT(4*8+2), cmpT(4*8+3), cmpT(4*8+4), cmpT(4*8+5), cmpT(4*8+6), cmpT(4*8+7),
      cmpT(5*8+0), cmpT(5*8+1), cmpT(5*8+2), cmpT(5*8+3), cmpT(5*8+4), cmpT(5*8+5), cmpT(5*8+6), cmpT(5*8+7),
      cmpT(6*8+0), cmpT(6*8+1), cmpT(6*8+2), cmpT(6*8+3), cmpT(6*8+4), cmpT(6*8+5), cmpT(6*8+6), cmpT(6*8+7),
      cmpT(7*8+0), cmpT(7*8+1), cmpT(7*8+2), cmpT(7*8+3), cmpT(7*8+4), cmpT(7*8+5), cmpT(7*8+6), cmpT(7*8+7)
    };

    static int4 compactTable[16] = {
      /* 0b0000 */ int4(3,3,3,3),
      /* 0b0001 */ int4(0,3,3,3),
      /* 0b0010 */ int4(1,3,3,3),
      /* 0b0011 */ int4(0,1,3,3),
      /* 0b0100 */ int4(2,3,3,3),
      /* 0b0101 */ int4(0,2,3,3),
      /* 0b0110 */ int4(1,2,3,3),
      /* 0b0111 */ int4(0,1,2,3),

      /* 0b1000 */ int4(3,0,0,0),
      /* 0b1001 */ int4(0,3,1,1),
      /* 0b1010 */ int4(1,3,0,0),
      /* 0b1011 */ int4(0,1,3,2),
      /* 0b1100 */ int4(2,3,0,0),
      /* 0b1101 */ int4(0,2,3,1),
      /* 0b1110 */ int4(1,2,3,0),
      /* 0b1111 */ int4(0,1,2,3)
    };

  static int8 comparePerm0(0,0,0,1,1,2,0,0);
  static int8 comparePerm1(1,2,3,2,3,3,0,0);
  namespace isa
  {
    __forceinline size_t getCompareMask(const float4 &v)
    {
#if defined (__AVX2__)
      const float8 v8  = assign(v);
      const float8 c0  = permute(v8,comparePerm0);
      const float8 c1  = permute(v8,comparePerm1);
      PRINT(c0);
      PRINT(c1);
      const bool8 mask = c0 < c1;
      PRINT(mask);
      return movemask(mask);
#else
      return 0;
#endif
    }

    __forceinline int4 getPermuteSequence(const float4 &v)
    {
      const size_t mask = getCompareMask(v);
      assert(mask < 64);
      PRINT(compareTable[mask]);
      return compareTable[mask];
    }

    __forceinline int4 merge(const int4 &a, const int4 &b, const int imm)
    {
      return select(imm,a,b);
    }

    __forceinline int4 networkSort(const float4 &v, const bool4 &active)
    {
      const int4 or_mask = select(active,int4( step ),int4( 0xffffffff ));
      const int4 a0 = (srl(cast(v),2)) | or_mask;
      PRINT(active);
      PRINT(a0);
      PRINT(or_mask);
      PRINT(v);
      const int4 b0 = shuffle<1,0,3,2>(a0);
      const int4 c0 = umin(a0,b0);
      const int4 d0 = umax(a0,b0);
      const int4 a1 = merge(c0,d0,0b0101);
      const int4 b1 = shuffle<2,3,0,1>(a1);
      const int4 c1 = umin(a1,b1);
      const int4 d1 = umax(a1,b1);
      const int4 a2 = merge(c1,d1,0b0011);
      const int4 b2 = shuffle<0,2,1,0>(a2);
      const int4 c2 = umin(a2,b2);
      const int4 d2 = umax(a2,b2);
      const int4 a3 = merge(c2,d2,0b0010);
      return a3 & 3;
    }

    template<int types, bool robust, typename PrimitiveIntersector8>
    void BVH4Intersector8Test<types,robust,PrimitiveIntersector8>::intersect(bool8* valid_i, BVH4* bvh, Ray8& ray)
    {
      /* verify correct input */
      bool8 valid0 = *valid_i;
#if defined(RTCORE_IGNORE_INVALID_RAYS)
      valid0 &= ray.valid();
#endif
      assert(all(valid0,ray.tnear > -FLT_MIN));
      assert(!(types & BVH4::FLAG_NODE_MB) || all(valid0,ray.time >= 0.0f & ray.time <= 1.0f));
      
      /* load ray */
      Vec3f8 ray_org = ray.org;
      Vec3f8 ray_dir = ray.dir;
      float8 ray_tnear = ray.tnear, ray_tfar  = ray.tfar;
      const Vec3f8 ray_rdir = rcp_safe(ray_dir);
      ray_tnear = select(valid0,ray_tnear,float8(pos_inf));
      ray_tfar  = select(valid0,ray_tfar ,float8(neg_inf));
      const float8 inf = float8(pos_inf);
      Precalculations pre(valid0,ray);

      Vec3i8 nearXYZ;
      nearXYZ.x = select(ray_rdir.x >= 0.0f,int8(0*(int)sizeof(float4)),int8(1*(int)sizeof(float4)));
      nearXYZ.y = select(ray_rdir.y >= 0.0f,int8(2*(int)sizeof(float4)),int8(3*(int)sizeof(float4)));
      nearXYZ.z = select(ray_rdir.z >= 0.0f,int8(4*(int)sizeof(float4)),int8(5*(int)sizeof(float4)));

      const bool8 active = ray_tnear < ray_tfar;
      size_t bits = movemask(active);

      /* switch to single ray traversal */
#if 0 //!defined(__WIN32__) || defined(__X86_64__)
      /* compute near/far per ray */

      for (size_t i=__bsf(bits); bits!=0; bits=__btc(bits,i), i=__bsf(bits)) {
        BVH4Intersector8Single<types,robust,PrimitiveIntersector8>::intersect1(bvh, bvh->root, i, pre, ray, ray_org, ray_dir, ray_rdir, ray_tnear, ray_tfar, nearXYZ);
      }
#else

#if defined(__AVX2__)

      float4 t0(3,2,1,0);

      const int4 p0 = networkSort(t0, bool4(0,0,0,0) );
      const float4 t1 = permute(t0,p0);
      PRINT(t0);
      PRINT(p0);
      PRINT(t1);
      exit(0);
#endif

      StackItemT<NodeRef> stack[stackSizeSingle];  //!< stack of nodes 
      for (size_t k=__bsf(bits); bits!=0; bits=__blsr(bits), k=__bsf(bits)) 
      {
        StackItemT<NodeRef>* stackPtr = stack + 1;        //!< current stack pointer
        StackItemT<NodeRef>* stackEnd = stack + stackSizeSingle;

        stack[0].ptr  = bvh->root;
        stack[0].dist = neg_inf;

        /*! load the ray into SIMD registers */
        const Vec3f4 org(ray_org.x[k], ray_org.y[k], ray_org.z[k]);
        const Vec3f4 dir(ray_dir.x[k], ray_dir.y[k], ray_dir.z[k]);
        const Vec3f4 rdir(ray_rdir.x[k], ray_rdir.y[k], ray_rdir.z[k]);
        const Vec3f4 org_rdir(org*rdir);
        float4 ray_near(ray_tnear[k]), ray_far(ray_tfar[k]);
      
        /*! offsets to select the side that becomes the lower or upper bound */
        const size_t nearX = nearXYZ.x[k];
        const size_t nearY = nearXYZ.y[k];
        const size_t nearZ = nearXYZ.z[k];

        /* pop loop */
        while (true) pop:
        {
          /*! pop next node */
          if (unlikely(stackPtr == stack)) break;
          stackPtr--;
          NodeRef cur = NodeRef(stackPtr->ptr);
          
          /*! if popped node is too far, pop next one */
          if (unlikely(*(float*)&stackPtr->dist > ray.tfar[k]))
            continue;
        
          /* downtraversal loop */
          while (true)
          {
            /*! stop if we found a leaf node */
            if (unlikely(cur.isLeaf(types))) break;
            STAT3(normal.trav_nodes,1,1,1);

            const BVH4::Node* node = cur.node();
#if defined(__AVX2__)
            const float4 _tNearX = msub(node->lower_x, rdir.x, org_rdir.x);
            const float4 _tNearY = msub(node->lower_y, rdir.y, org_rdir.y);
            const float4 _tNearZ = msub(node->lower_z, rdir.z, org_rdir.z);
            const float4 _tFarX  = msub(node->upper_x, rdir.x, org_rdir.x);
            const float4 _tFarY  = msub(node->upper_y, rdir.y, org_rdir.y);
            const float4 _tFarZ  = msub(node->upper_z, rdir.z, org_rdir.z);

            const bool4  nactive = float4(pos_inf) == node->lower_x;
            const float4 tNearX  = mini(_tNearX,_tFarX);
            const float4 tNearY  = mini(_tNearY,_tFarY);
            const float4 tNearZ  = mini(_tNearZ,_tFarZ);
            
            const float4 tFarX   = maxi(_tNearX,_tFarX);
            const float4 tFarY   = maxi(_tNearY,_tFarY);
            const float4 tFarZ   = maxi(_tNearZ,_tFarZ);
            
            const float4 tNear = maxi(maxi(tNearX,tNearY),maxi(tNearZ,ray_near));
            const float4 tFar  = mini(mini(tFarX ,tFarY ),mini(tFarZ ,ray_far ));
            const bool4 vmask  = nactive | cast(tNear) > cast(tFar);
            
            size_t mask = movemask(vmask)^0xf;

            if (unlikely(mask == 0))
              goto pop;

            //const float4 dist = select(vmask,float4(neg_inf),tNear);
            //const float4 dist_perm = permute(dist,compactTable[mask]);

#pragma unroll
            for (size_t i=0;i<4;i++)
            {
              const unsigned int index = compactTable[mask][i];
              stackPtr[i].ptr  = node->child(index);
              stackPtr[i].dist = ((unsigned int*)&tNear)[index];
            }

            stackPtr += (ssize_t)__popcnt(mask) - 1;
            cur = stackPtr->ptr;


#else
            float4 tNear;
            size_t mask = intersect_node<robust>(cur.node(),nearX,nearY,nearZ,org,rdir,org_rdir,ray_near,ray_far,tNear); 
            /*! if no child is hit, pop next node */
            if (unlikely(mask == 0))
              goto pop;
          
            /*! one child is hit, continue with that child */
            size_t r = __bscf(mask);
            if (likely(mask == 0)) {
              cur = node->child(r); cur.prefetch(types);
              assert(cur != BVH4::emptyNode);
              continue;
            }
          
            /*! two children are hit, push far child, and continue with closer child */
            NodeRef c0 = node->child(r); c0.prefetch(types); const unsigned int d0 = ((unsigned int*)&tNear)[r];
            r = __bscf(mask);
            NodeRef c1 = node->child(r); c1.prefetch(types); const unsigned int d1 = ((unsigned int*)&tNear)[r];
            assert(c0 != BVH4::emptyNode);
            assert(c1 != BVH4::emptyNode);
            if (likely(mask == 0)) {
              assert(stackPtr < stackEnd); 
              if (d0 < d1) { stackPtr->ptr = c1; stackPtr->dist = d1; stackPtr++; cur = c0; continue; }
              else         { stackPtr->ptr = c0; stackPtr->dist = d0; stackPtr++; cur = c1; continue; }
            }
          
            /*! Here starts the slow path for 3 or 4 hit children. We push
             *  all nodes onto the stack to sort them there. */
            assert(stackPtr < stackEnd); 
            stackPtr->ptr = c0; stackPtr->dist = d0; stackPtr++;
            assert(stackPtr < stackEnd); 
            stackPtr->ptr = c1; stackPtr->dist = d1; stackPtr++;
          
            /*! three children are hit, push all onto stack and sort 3 stack items, continue with closest child */
            assert(stackPtr < stackEnd); 
            r = __bscf(mask);
            NodeRef c = node->child(r); c.prefetch(types); unsigned int d = ((unsigned int*)&tNear)[r]; stackPtr->ptr = c; stackPtr->dist = d; stackPtr++;
            assert(c != BVH4::emptyNode);
            if (likely(mask == 0)) {
              sort(stackPtr[-1],stackPtr[-2],stackPtr[-3]);
              cur = (NodeRef) stackPtr[-1].ptr; stackPtr--;
              continue;
            }
          
            /*! four children are hit, push all onto stack and sort 4 stack items, continue with closest child */
            assert(stackPtr < stackEnd); 
            r = __bscf(mask);
            c = node->child(r); c.prefetch(types); d = *(unsigned int*)&tNear[r]; stackPtr->ptr = c; stackPtr->dist = d; stackPtr++;
            assert(c != BVH4::emptyNode);
            sort(stackPtr[-1],stackPtr[-2],stackPtr[-3],stackPtr[-4]);
            cur = (NodeRef) stackPtr[-1].ptr; stackPtr--;

#endif

          }
        
          /*! this is a leaf node */
          assert(cur != BVH4::emptyNode);
          STAT3(normal.trav_leaves, 1, 1, 1);
          size_t num; Primitive* prim = (Primitive*)cur.leaf(num);

          size_t lazy_node = 0;
          PrimitiveIntersector8::intersect(pre, ray, k, prim, num, bvh->scene, lazy_node);
          ray_far = ray.tfar[k];
        }
      }

#endif
      AVX_ZERO_UPPER();
    }

    
    template<int types, bool robust, typename PrimitiveIntersector8>
    void BVH4Intersector8Test<types,robust,PrimitiveIntersector8>::occluded(bool8* valid_i, BVH4* bvh, Ray8& ray)
    {
      /* verify correct input */
      bool8 valid = *valid_i;
#if defined(RTCORE_IGNORE_INVALID_RAYS)
      valid &= ray.valid();
#endif
      assert(all(valid,ray.tnear > -FLT_MIN));
      assert(!(types & BVH4::FLAG_NODE_MB) || all(valid,ray.time >= 0.0f & ray.time <= 1.0f));

      /* load ray */
      bool8 terminated = !valid;
      Vec3f8 ray_org = ray.org, ray_dir = ray.dir;
      float8 ray_tnear = ray.tnear, ray_tfar  = ray.tfar;
      const Vec3f8 rdir = rcp_safe(ray_dir);
      const Vec3f8 org(ray_org), org_rdir = org * rdir;
      ray_tnear = select(valid,ray_tnear,float8(pos_inf));
      ray_tfar  = select(valid,ray_tfar ,float8(neg_inf));
      const float8 inf = float8(pos_inf);
      Precalculations pre(valid,ray);

      /* compute near/far per ray */
      Vec3i8 nearXYZ;
      nearXYZ.x = select(rdir.x >= 0.0f,int8(0*(int)sizeof(float4)),int8(1*(int)sizeof(float4)));
      nearXYZ.y = select(rdir.y >= 0.0f,int8(2*(int)sizeof(float4)),int8(3*(int)sizeof(float4)));
      nearXYZ.z = select(rdir.z >= 0.0f,int8(4*(int)sizeof(float4)),int8(5*(int)sizeof(float4)));

      const bool8 active = ray_tnear < ray_tfar;

        /* switch to single ray traversal */
#if !defined(__WIN32__) || defined(__X86_64__)
      size_t bits = movemask(active);
      for (size_t i=__bsf(bits); bits!=0; bits=__btc(bits,i), i=__bsf(bits)) {
        if (BVH4Intersector8Single<types,robust,PrimitiveIntersector8>::occluded1(bvh,bvh->root,i,pre,ray,ray_org,ray_dir,rdir,ray_tnear,ray_tfar,nearXYZ))
          terminated[i] = -1;
      }
#endif
                
      store8i(valid & terminated,&ray.geomID,0);
      AVX_ZERO_UPPER();
    }
    
    DEFINE_INTERSECTOR8(BVH4Triangle4Intersector8TestMoeller, BVH4Intersector8Test<0x1 COMMA false COMMA ArrayIntersector8_1<TriangleNIntersectorMMoellerTrumbore<Ray8 COMMA Triangle4 COMMA true> > >);
    DEFINE_INTERSECTOR8(BVH4Triangle4Intersector8TestMoellerNoFilter, BVH4Intersector8Test<0x1 COMMA false COMMA ArrayIntersector8_1<TriangleNIntersectorMMoellerTrumbore<Ray8 COMMA Triangle4 COMMA false> > >);
    DEFINE_INTERSECTOR8(BVH4Triangle8Intersector8TestMoeller, BVH4Intersector8Test<0x1 COMMA false COMMA ArrayIntersector8_1<TriangleNIntersectorMMoellerTrumbore<Ray8 COMMA Triangle8 COMMA true> > >);
    DEFINE_INTERSECTOR8(BVH4Triangle8Intersector8TestMoellerNoFilter, BVH4Intersector8Test<0x1 COMMA false COMMA ArrayIntersector8_1<TriangleNIntersectorMMoellerTrumbore<Ray8 COMMA Triangle8 COMMA false> > >);
    DEFINE_INTERSECTOR8(BVH4Triangle4vIntersector8TestPluecker, BVH4Intersector8Test<0x1 COMMA true COMMA ArrayIntersector8_1<TriangleNvIntersectorMPluecker<Ray8 COMMA Triangle4v COMMA true> > >);
                   

    // FIXME: add Triangle4vMB intersector
    // FIXME: add Triangle4i intersector
  }
}
