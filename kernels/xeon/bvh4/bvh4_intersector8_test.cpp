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

#if !defined(__WIN32__)

namespace embree
{

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

    static int8 compactTable64[16] = {
      /* 0b0000 */ int8(6,7,6,7,6,7,6,7),
      /* 0b0001 */ int8(0,1,6,7,6,7,6,7),
      /* 0b0010 */ int8(2,3,6,7,6,7,6,7),
      /* 0b0011 */ int8(0,1,2,3,6,7,6,7),
      /* 0b0100 */ int8(4,5,6,7,6,7,6,7),
      /* 0b0101 */ int8(0,1,4,5,6,7,6,7),
      /* 0b0110 */ int8(2,3,4,5,6,7,6,7),
      /* 0b0111 */ int8(0,1,2,3,4,5,6,7),

      /* 0b1000 */ int8(6,7,0,1,0,1,0,1),
      /* 0b1001 */ int8(0,1,6,7,2,3,2,3),
      /* 0b1010 */ int8(2,3,6,7,0,1,0,1),
      /* 0b1011 */ int8(0,1,2,3,6,7,4,5),
      /* 0b1100 */ int8(4,5,6,7,0,1,0,1),
      /* 0b1101 */ int8(0,1,4,5,6,7,2,3),
      /* 0b1110 */ int8(2,3,4,5,6,7,0,1),
      /* 0b1111 */ int8(0,1,2,3,4,5,6,7)
    };

  static int8 comparePerm0(0,0,0,1,1,2,0,0);
  static int8 comparePerm1(1,2,3,2,3,3,0,0);

  static int8 lowHighExtract(0,2,4,6,1,3,5,7);
  static int8 lowHighInsert (0,4,1,5,2,6,3,7);

  static int4 reverseCompact[5] = {
    int4(0,1,2,3),
    int4(0,1,2,3),
    int4(1,0,2,3),
    int4(2,1,0,3),
    int4(3,2,1,0)
  };

  static int4 step4( 0,1,2,3 );
  static int4 allSet4( 0xffffffff,0xffffffff,0xffffffff,0xffffffff );
  static int4 mask2( 0xfffffffc,0xfffffffc,0xfffffffc,0xfffffffc );

  static int8 step2x4( 0,1,2,3, 0,1,2,3 );
  static unsigned int mask1x8 = 0xfffffffc;
 
#undef DBG_PRINT
#define DBG_PRINT(x) 
//#define DBG_PRINT(x) PRINT(x)
  
  namespace isa
  {

 #if defined (__AVX2__)
    __forceinline int4 permute(const int4 &a, const int4 &index) {
      return _mm_castps_si128(_mm_permutevar_ps(_mm_castsi128_ps(a),index));
    }
#endif
   

    __forceinline int4 merge(const int4 &a, const int4 &b, const int imm)
    {
      return select(imm,a,b);
    }

    __forceinline float8 merge(const float8 &a, const float8 &b, const int imm)
    {
      return select(imm,a,b);
    }

    __forceinline int8 merge(const int8 &a, const int8 &b, const int imm)
    {
      return select(imm,a,b);
    }

    __forceinline unsigned int networkSort(const float4 &v, const bool4 &active, int4 &result)
    {
     // const int4 or_mask = select(active,step4,allSet4); //optimize
      const int4 vi = (cast(v) & mask2) | step4; 
      const int4 a0 = select(active,vi,int4( True ));
      const int4 b0 = shuffle<1,0,3,2>(a0);
      const int4 c0 = umin(a0,b0);
      const int4 d0 = umax(a0,b0);
      const int4 a1 = merge(c0,d0,0b0101);
      const int4 b1 = shuffle<2,3,0,1>(a1);
      const int4 c1 = umin(a1,b1);
      const unsigned int min_dist_index = extract<0>(c1) & 3;
      // 2 case border?
      const int4 d1 = umax(a1,b1);
      const int4 a2 = merge(c1,d1,0b0011);
      assert(min_dist_index < 4);
      const int4 b2 = shuffle<0,2,1,3>(a2);
      const int4 c2 = umin(a2,b2);
      const int4 d2 = umax(a2,b2);
      const int4 a3 = merge(c2,d2,0b0010);
      assert(*(unsigned int*)&a3[0] <= *(unsigned int*)&a3[1]);
      assert(*(unsigned int*)&a3[1] <= *(unsigned int*)&a3[2]);
      assert(*(unsigned int*)&a3[2] <= *(unsigned int*)&a3[3]);
      result = a3 & 3;
      return min_dist_index;
    }

    __forceinline float8 merge2x4(const float& a, const float& b)
    {
      const float8 va = broadcast(&a);
      const float8 vb = broadcast(&b);
      return merge(va,vb,0b00001111);
    }

#if 0 //

      float4 t0(3,0,1,2);

      int4 p0;
      const unsigned int index = networkSort(t0, bool4(1,0,1,1), p0);
      PRINT(index);
      
      int4 p1         = permute(p0,reverseCompact[3]);
      const float4 t1 = permute(t0,p0);
      const float4 t2 = permute(t0,p1);
      PRINT(t0);
      PRINT(p0);
      PRINT(t1);
      PRINT(p1);
      PRINT(t2);
      exit(0);
#endif
    template<int types, bool robust, typename PrimitiveIntersector8>
    void BVH4Intersector8Test<types,robust,PrimitiveIntersector8>::intersect(bool8* valid_i, BVH4* bvh, Ray8& ray)
    {
#if defined(__AVX2__)
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
      const Vec3f8 ray_org_rdir(ray_org*ray_rdir);
      const bool8 active = ray_tnear < ray_tfar;
      size_t bits = movemask(active);



      /* switch to single ray traversal */
#if 1 // !defined(__WIN32__) || defined(__X86_64__)

#if 1
      Vec3i8 nearXYZ;
      nearXYZ.x = select(ray_rdir.x >= 0.0f,int8(0*(int)sizeof(float4)),int8(1*(int)sizeof(float4)));
      nearXYZ.y = select(ray_rdir.y >= 0.0f,int8(2*(int)sizeof(float4)),int8(3*(int)sizeof(float4)));
      nearXYZ.z = select(ray_rdir.z >= 0.0f,int8(4*(int)sizeof(float4)),int8(5*(int)sizeof(float4)));

      /* compute near/far per ray */

      for (size_t i=__bsf(bits); bits!=0; bits=__btc(bits,i), i=__bsf(bits)) {
        BVH4Intersector8Single<types,robust,PrimitiveIntersector8>::intersect1(bvh, bvh->root, i, pre, ray, ray_org, ray_dir, ray_rdir, ray_tnear, ray_tfar, nearXYZ);
      }
#else

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

      NodeRef stackNode[2][stackSizeSingle];  //!< stack of nodes 
      float   stackDist[2][stackSizeSingle];  //!< stack of nodes 

      size_t m_active = bits;

      /* only a single ray active */
      const size_t numActiveRays = __popcnt(m_active);
      if (unlikely(numActiveRays <= 1))
      {
        Vec3i8 nearXYZ;
        nearXYZ.x = select(ray_rdir.x >= 0.0f,int8(0*(int)sizeof(float4)),int8(1*(int)sizeof(float4)));
        nearXYZ.y = select(ray_rdir.y >= 0.0f,int8(2*(int)sizeof(float4)),int8(3*(int)sizeof(float4)));
        nearXYZ.z = select(ray_rdir.z >= 0.0f,int8(4*(int)sizeof(float4)),int8(5*(int)sizeof(float4)));

        for (size_t i=__bsf(m_active); m_active!=0; m_active=__btc(m_active,i), i=__bsf(m_active)) 
          BVH4Intersector8Single<types,robust,PrimitiveIntersector8>::intersect1(bvh, bvh->root, i, pre, ray, ray_org, ray_dir, ray_rdir, ray_tnear, ray_tfar, nearXYZ);
        return;
      }

      for (size_t i=0;i<2;i++)
      {
        stackNode[i][0] = BVH4::invalidNode;
        stackDist[i][0] = pos_inf;
        stackNode[i][1] = bvh->root;
        stackDist[i][1] = neg_inf;
      }

      size_t rayIndex[2];
      rayIndex[0] = __bsf(m_active); m_active=__blsr(m_active);
      rayIndex[1] = __bsf(m_active); m_active=__blsr(m_active);
      assert(rayIndex[0] < 8);
      assert(rayIndex[1] < 8);

      size_t sindex[2];
      sindex[0] = 2;
      sindex[1] = 2;


      while(1)
      {
        DBG_PRINT("");
        DBG_PRINT("NEW TOP-DOWN ITERATION");
        DBG_PRINT(rayIndex[0]);
        DBG_PRINT(rayIndex[1]);
        
        const float8 ray_rdir_x     = merge2x4(ray_rdir.x[rayIndex[0]],ray_rdir.x[rayIndex[1]]);
        const float8 ray_rdir_y     = merge2x4(ray_rdir.y[rayIndex[0]],ray_rdir.y[rayIndex[1]]);
        const float8 ray_rdir_z     = merge2x4(ray_rdir.z[rayIndex[0]],ray_rdir.z[rayIndex[1]]);
        const float8 ray_org_rdir_x = merge2x4(ray_org_rdir.x[rayIndex[0]],ray_org_rdir.x[rayIndex[1]]);
        const float8 ray_org_rdir_y = merge2x4(ray_org_rdir.y[rayIndex[0]],ray_org_rdir.y[rayIndex[1]]);
        const float8 ray_org_rdir_z = merge2x4(ray_org_rdir.z[rayIndex[0]],ray_org_rdir.z[rayIndex[1]]);
        const float8 ray_near       = merge2x4(ray.tnear[rayIndex[0]],ray.tnear[rayIndex[1]]);
        const float8 ray_far        = merge2x4(ray.tfar[rayIndex[0]],ray.tfar[rayIndex[1]]);

        DBG_PRINT(ray_rdir_x);
        DBG_PRINT(ray_rdir_y);
        DBG_PRINT(ray_rdir_z);
        DBG_PRINT(ray_org_rdir_x);
        DBG_PRINT(ray_org_rdir_y);
        DBG_PRINT(ray_org_rdir_z);
        DBG_PRINT(ray_near);
        DBG_PRINT(ray_far);

        DBG_PRINT(sindex[0]);
        DBG_PRINT(sindex[1]);

#if PRINT_STACK == 1
        std::cout << "stack0 ";
        for (size_t i=0;i<sindex[0];i++)
          std::cout << stackNode[0][i] << " ";
        std::cout << std::endl;

        std::cout << "stack1 ";
        for (size_t i=0;i<sindex[1];i++)
          std::cout << stackNode[1][i] << " ";
        std::cout << std::endl;
#endif

        sindex[0]--;
        sindex[1]--;
        NodeRef cur0 = NodeRef(stackNode[0][sindex[0]]);
        NodeRef cur1 = NodeRef(stackNode[1][sindex[1]]);

        STAT3(normal.trav_stack_pop,1,1,1);

        
        while(1)
        {

          DBG_PRINT("");
          DBG_PRINT("TRAVERSAL");
          DBG_PRINT(cur0);
          DBG_PRINT(cur1);
          DBG_PRINT(sindex[0]);
          DBG_PRINT(sindex[1]);

          BVH4::NodeRef cur = cur0 | cur1;

          DBG_PRINT(cur);

          if (unlikely(cur.isLeaf(types))) break;
          STAT3(normal.trav_nodes,1,1,1);

          assert(cur0.isNode());
          assert(cur1.isNode());

          const BVH4::Node* node0 = cur0.node();
          const BVH4::Node* node1 = cur1.node();
          
          const float8 lower_x(node0->lower_x,node1->lower_x);
          const float8 lower_y(node0->lower_y,node1->lower_y);
          const float8 lower_z(node0->lower_z,node1->lower_z);

          const float8 upper_x(node0->upper_x,node1->upper_x);
          const float8 upper_y(node0->upper_y,node1->upper_y);
          const float8 upper_z(node0->upper_z,node1->upper_z);

          const float8 _tNearX = msub(lower_x, ray_rdir_x, ray_org_rdir_x);
          const float8 _tNearY = msub(lower_y, ray_rdir_y, ray_org_rdir_y);
          const float8 _tNearZ = msub(lower_z, ray_rdir_z, ray_org_rdir_z);
          const float8 _tFarX  = msub(upper_x, ray_rdir_x, ray_org_rdir_x);
          const float8 _tFarY  = msub(upper_y, ray_rdir_y, ray_org_rdir_y);
          const float8 _tFarZ  = msub(upper_z, ray_rdir_z, ray_org_rdir_z);

          const bool8  nactive = float8(pos_inf) != lower_x;

          sindex[0]--;
          cur0 = stackNode[0][sindex[0]];
          sindex[1]--;
          cur1 = stackNode[1][sindex[1]];

          const float8 tNearX  = mini(_tNearX,_tFarX);
          const float8 tNearY  = mini(_tNearY,_tFarY);
          const float8 tNearZ  = mini(_tNearZ,_tFarZ);
          
          const float8 tFarX   = maxi(_tNearX,_tFarX);
          const float8 tFarY   = maxi(_tNearY,_tFarY);
          const float8 tFarZ   = maxi(_tNearZ,_tFarZ);
            
          const float8 tNear = maxi(maxi(tNearX,tNearY),maxi(tNearZ,ray_near));
          const float8 tFar  = mini(mini(tFarX ,tFarY ),mini(tFarZ ,ray_far ));
          const bool8 vmask  = nactive & (tNear <= tFar);
          const size_t mask  = movemask(vmask);

          DBG_PRINT(mask);

          STAT(if (mask == 0) STAT3(normal.trav_hit_boxes[0],1,1,1));

          if (unlikely(mask == 0)) continue;

#if 1
          const size_t mask0 = mask & 0xf;
          const size_t mask1 = mask >> 4;
          const int8 vi = (cast(tNear) & mask1x8) | step2x4;
          const int8 a0 = select(vmask,vi,int8( True )); //optimize
          const int8 b0 = shuffle<1,0,3,2>(a0);
          const int8 c0 = umin(a0,b0);
          const int8 d0 = umax(a0,b0);
          const int8 a1 = merge(c0,d0,0b01010101);
          const int8 b1 = shuffle<2,3,0,1>(a1);
          const int8 c1 = umin(a1,b1);
          const unsigned int min_index0 = extract<0>(extract<0>(c1)) & 3;
          const unsigned int min_index1 = extract<0>(extract<1>(c1)) & 3;
          cur0 = mask0 ? node0->child(min_index0) : cur0;
          cur1 = mask1 ? node1->child(min_index1) : cur1;

          assert(cur0 != BVH4::emptyNode);
          assert(cur1 != BVH4::emptyNode);

          assert(min_index0 < 4);
          assert(min_index1 < 4);
          const int8 d1 = umax(a1,b1);
          const int8 a2 = merge(c1,d1,0b00110011);
          const int8 b2 = shuffle<0,2,1,3>(a2);
          const int8 c2 = umin(a2,b2);
          const int8 d2 = umax(a2,b2);
          const int8 a3 = merge(c2,d2,0b00100010);
          const int8 result = a3 & 3;

          if (mask0)
          {
            sindex[0]++;
            const size_t setbits = __popcnt(mask0);
            DBG_PRINT(setbits);
            const int4 rc            = reverseCompact[setbits];
            const int8 node4         = *(int8*)node0->children;
            int4 perm4i              = extract<0>(result);
            const float4 tNear4      = extract<0>(tNear);
            perm4i                   = permute(perm4i,reverseCompact[setbits]);            
            const float4 dist4_perm  = permute(tNear4,perm4i);
            const int8 lowHigh32bit0 = permute(node4,lowHighExtract);
            const int8 lowHigh32bit1 = permute4x32(lowHigh32bit0,int8(perm4i));
            const int8 node4_perm    = permute(lowHigh32bit1,lowHighInsert);

            store4f(&stackDist[0][sindex[0]],dist4_perm);
            store8i(&stackNode[0][sindex[0]],node4_perm);            
            sindex[0] += setbits - 1;
          }

          if (mask1)
          {
            sindex[1]++;
            const size_t setbits = __popcnt(mask1);
            DBG_PRINT(setbits);
            const int4 rc            = reverseCompact[setbits];
            const int8 node4         = *(int8*)node1->children;
            int4 perm4i              = extract<1>(result);
            const float4 tNear4      = extract<1>(tNear);
            perm4i                   = permute(perm4i,reverseCompact[setbits]);            
            const float4 dist4_perm  = permute(tNear4,perm4i);
            const int8 lowHigh32bit0 = permute(node4,lowHighExtract);
            const int8 lowHigh32bit1 = permute4x32(lowHigh32bit0,int8(perm4i));
            const int8 node4_perm    = permute(lowHigh32bit1,lowHighInsert);

            store4f(&stackDist[1][sindex[1]],dist4_perm);
            store8i(&stackNode[1][sindex[1]],node4_perm);            
            sindex[1] += setbits - 1;            
          }


#else
          const size_t mask0 = mask & 0xf;
          const size_t mask1 = mask >> 4;

          STAT(
            if (__popcnt(mask0) <= 1 && __popcnt(mask1) <= 1) 
            {
              STAT3(normal.trav_hit_boxes[1],1,1,1);
            }
            else
            {
              STAT3(normal.trav_hit_boxes[2],1,1,1);
            }
            );


          DBG_PRINT(mask0);
          DBG_PRINT(mask1);

          if (mask0)
          {
            sindex[0]++;
            const size_t setbits = __popcnt(mask0);
            DBG_PRINT(setbits);
            const int4 rc        = reverseCompact[setbits];
            const int8 node4     = *(int8*)node0->children;
            const float4 tNear4  = extract<0>(tNear);
            const bool4  vmask4  = extract<0>(vmask);
            int4 perm4i;
            const unsigned int min_dist_index = networkSort(tNear4,vmask4,perm4i);
            assert(min_dist_index < 4);
            cur0 = node0->child(min_dist_index);
            perm4i         = permute(perm4i,reverseCompact[setbits]);            
            const float4 dist4_perm  = permute(tNear4,perm4i);
            const int8 lowHigh32bit0 = permute(node4,lowHighExtract);
            const int8 lowHigh32bit1 = permute4x32(lowHigh32bit0,int8(perm4i));
            const int8 node4_perm    = permute(lowHigh32bit1,lowHighInsert);

            store4f(&stackDist[0][sindex[0]],dist4_perm);
            store8i(&stackNode[0][sindex[0]],node4_perm);            
            sindex[0] += setbits - 1;
          }

          if (mask1)
          {
            sindex[1]++;
            const size_t setbits = __popcnt(mask1);
            DBG_PRINT(setbits);
            const int4 rc        = reverseCompact[setbits];
            const int8 node4     = *(int8*)node1->children;
            const float4 tNear4  = extract<1>(tNear);
            const bool4  vmask4  = extract<1>(vmask);
            int4 perm4i;
            const unsigned int min_dist_index = networkSort(tNear4,vmask4,perm4i);
            assert(min_dist_index < 4);
            cur1 = node1->child(min_dist_index);
            perm4i         = permute(perm4i,reverseCompact[setbits]);            
            const float4 dist4_perm  = permute(tNear4,perm4i);
            const int8 lowHigh32bit0 = permute(node4,lowHighExtract);
            const int8 lowHigh32bit1 = permute4x32(lowHigh32bit0,int8(perm4i));
            const int8 node4_perm    = permute(lowHigh32bit1,lowHighInsert);

            store4f(&stackDist[1][sindex[1]],dist4_perm);
            store8i(&stackNode[1][sindex[1]],node4_perm);            
            sindex[1] += setbits - 1;
          }
#endif

          DBG_PRINT(sindex[0]);
          DBG_PRINT(sindex[1]);

#if PRINT_STACK == 1
          std::cout << "stack0 ";
          for (size_t i=0;i<sindex[0];i++)
            std::cout << stackNode[0][i] << " ";
          std::cout << std::endl;
          
          std::cout << "stack1 ";
          for (size_t i=0;i<sindex[1];i++)
            std::cout << stackNode[1][i] << " ";
          std::cout << std::endl;
#endif
        }

        DBG_PRINT("LEAF");

        assert(cur0 != BVH4::emptyNode);
        assert(cur1 != BVH4::emptyNode);

        DBG_PRINT(cur0);
        DBG_PRINT(cur1);

        BVH4::NodeRef current[2];
        if (unlikely(rayIndex[0] == rayIndex[1])) cur1 = BVH4::invalidNode;

        // LOOP UNTIL NON LEAF

        current[0] = cur0;
        current[1] = cur1;
        for (size_t i=0;i<2;i++)
        {          
          const BVH4::NodeRef cur = current[i];
          DBG_PRINT(cur);
          DBG_PRINT(cur.isLeaf());
          DBG_PRINT(sindex[i]);
          if (unlikely(cur.isNode())) {
            stackNode[i][sindex[i]++] = cur;
            continue;
          }
          assert(cur.isLeaf());
          if (unlikely(cur == BVH4::invalidNode)) continue;

          const size_t rindex = rayIndex[i];

          STAT3(normal.trav_leaves, 1, 1, 1);
          size_t num; Primitive* prim = (Primitive*)cur.leaf(num);

          DBG_PRINT("INTERSECTION");
          DBG_PRINT(prim->primID());
          const float old_tfar = ray.tfar[rindex];
          size_t lazy_node = 0;
          PrimitiveIntersector8::intersect(pre, ray, rindex, prim, num, bvh->scene, lazy_node);

          /*! stack compaction */
#if 1
          if (unlikely(old_tfar != ray.tfar[rindex]))
          {
            size_t new_sindex = 1;
            for (size_t s=1;s<sindex[i];s++)
              if (unlikely(stackDist[i][s] <= ray.tfar[rindex]))
              {
                stackNode[i][new_sindex] = stackNode[i][s];
                stackDist[i][new_sindex] = stackDist[i][s];
                new_sindex++;
              }
            sindex[i] = new_sindex;
          }
#endif
        }

        if (unlikely(current[0] == BVH4::invalidNode && 
                     current[1] == BVH4::invalidNode && 
              m_active == 0)) 
        {
          DBG_PRINT(rayIndex[0]);
          DBG_PRINT(rayIndex[1]);
          DBG_PRINT(current[0]);
          DBG_PRINT(current[1]);
          DBG_PRINT("ALL RAYS TERMINATED");
          break;
        }

#if PRINT_STACK == 1
        DBG_PRINT(sindex[0]);
        DBG_PRINT(sindex[1]);

          std::cout << "stack0 ";
          for (size_t i=0;i<sindex[0];i++)
            std::cout << stackNode[0][i] << " ";
          std::cout << std::endl;
          
          std::cout << "stack1 ";
          for (size_t i=0;i<sindex[1];i++)
            std::cout << stackNode[1][i] << " ";
          std::cout << std::endl;
#endif

        for (size_t i=0;i<2;i++)
          if (current[i] == BVH4::invalidNode)
          {
            if (m_active)
            {
              DBG_PRINT("REFILL");
              DBG_PRINT(i);
              rayIndex[i]     = __bsf(m_active); m_active=__blsr(m_active); 
              sindex[i]       = 2;
              stackNode[i][1] = bvh->root;
              stackDist[i][1] = neg_inf;
              DBG_PRINT(ray);

              DBG_PRINT(rayIndex[i]);
              DBG_PRINT(sindex[i]);
            }
            else
            {
              
              rayIndex[i] = rayIndex[1-i];
              sindex[i]   = sindex[1-i];
              for (size_t j=1;j<sindex[i];j++)
              {
                stackNode[i][j] = stackNode[1-i][j];
                stackDist[i][j] = stackDist[1-i][j];
              }
            }
          }
      }

      
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#endif

          //PRINT(ray);
          //exit(0);

#else

      NodeRef stackNode[stackSizeSingle];  //!< stack of nodes 
      float   stackDist[stackSizeSingle];  //!< stack of nodes 

      for (size_t k=__bsf(bits); bits!=0; bits=__blsr(bits), k=__bsf(bits)) 
      {
        size_t sindex = 2;

        stackNode[0] = BVH4::invalidNode;
        stackDist[0] = pos_inf;
        stackNode[1] = bvh->root;
        stackDist[1] = neg_inf;

        /*! load the ray into SIMD registers */
        //const Vec3f4 org(ray_org.x[k], ray_org.y[k], ray_org.z[k]);
        //const Vec3f4 dir(ray_dir.x[k], ray_dir.y[k], ray_dir.z[k]);
        const Vec3f4 rdir(ray_rdir.x[k], ray_rdir.y[k], ray_rdir.z[k]);
        const Vec3f4 org_rdir(ray_org_rdir.x[k], ray_org_rdir.y[k], ray_org_rdir.z[k]);
        float4 ray_near(ray_tnear[k]), ray_far(ray_tfar[k]);
      
        /* pop loop */
        while (true)
        {
          /*! pop next node */
          //if (unlikely(sindex == 0)) break;
          sindex--;
          NodeRef cur = NodeRef(stackNode[sindex]);
          STAT3(normal.trav_stack_pop,1,1,1);
        
          /* downtraversal loop */
          while (true)
          {
            /*! stop if we found a leaf node */
            if (unlikely(cur.isLeaf(types))) break;
            STAT3(normal.trav_nodes,1,1,1);

            const BVH4::Node* node = cur.node();
#if defined(__AVX2__)

#if 1
            const float4 rdir_x = broadcast1f(&ray_rdir.x[k]);
            const float4 rdir_y = broadcast1f(&ray_rdir.y[k]);
            const float4 rdir_z = broadcast1f(&ray_rdir.z[k]);

            const float4 org_rdir_x = broadcast1f(&ray_org_rdir.x[k]);
            const float4 org_rdir_y = broadcast1f(&ray_org_rdir.y[k]);
            const float4 org_rdir_z = broadcast1f(&ray_org_rdir.z[k]);

            const float4 _tNearX = msub(node->lower_x, rdir_x, org_rdir_x);
            const float4 _tNearY = msub(node->lower_y, rdir_y, org_rdir_y);
            const float4 _tNearZ = msub(node->lower_z, rdir_z, org_rdir_z);
            const float4 _tFarX  = msub(node->upper_x, rdir_x, org_rdir_x);
            const float4 _tFarY  = msub(node->upper_y, rdir_y, org_rdir_y);
            const float4 _tFarZ  = msub(node->upper_z, rdir_z, org_rdir_z);
#else
            const float4 _tNearX = msub(node->lower_x, rdir.x, org_rdir.x);
            const float4 _tNearY = msub(node->lower_y, rdir.y, org_rdir.y);
            const float4 _tNearZ = msub(node->lower_z, rdir.z, org_rdir.z);
            const float4 _tFarX  = msub(node->upper_x, rdir.x, org_rdir.x);
            const float4 _tFarY  = msub(node->upper_y, rdir.y, org_rdir.y);
            const float4 _tFarZ  = msub(node->upper_z, rdir.z, org_rdir.z);
#endif
            //const bool4  nactive = float4(pos_inf) == node->lower_x;
            const bool4  nactive = float4(pos_inf) != node->lower_x;

            sindex--;
            const BVH4::NodeRef stack_cur = stackNode[sindex];
            stack_cur.prefetchMem();

            const float4 tNearX  = mini(_tNearX,_tFarX);
            const float4 tNearY  = mini(_tNearY,_tFarY);
            const float4 tNearZ  = mini(_tNearZ,_tFarZ);
            
            const float4 tFarX   = maxi(_tNearX,_tFarX);
            const float4 tFarY   = maxi(_tNearY,_tFarY);
            const float4 tFarZ   = maxi(_tNearZ,_tFarZ);
            
            const float4 tNear = maxi(maxi(tNearX,tNearY),maxi(tNearZ,ray_near));
            const float4 tFar  = mini(mini(tFarX ,tFarY ),mini(tFarZ ,ray_far ));
            //const bool4 vmask  = nactive | cast(tNear) > cast(tFar);
            const bool4 vmask  = nactive & (tNear <= tFar);
            //unsigned int mask = movemask(vmask)^0xf;
            unsigned int mask = movemask(vmask);

            STAT3(normal.trav_hit_boxes[__popcnt(mask)],1,1,1);

#if 1
            cur = stack_cur;

            if (unlikely(mask == 0)) continue;

            sindex++;

            const size_t setbits = __popcnt(mask);
            const int4 rc = reverseCompact[setbits];

            //const float4 dist = select(vmask,float4(neg_inf),tNear);
            //const float4 dist_perm = permute(dist,compactTable[mask]);

            
            int8 node4 = *(int8*)node->children;

            // const int4 perm4i = compactTable[mask];
            // float4 dist4_perm = permute(tNear,perm4i);

            // const int8 perm8i = compactTable64[mask];
            // int8   node4_perm = permute(node4,perm8i);

            int4 perm4i;
            const unsigned int min_dist_index = networkSort(tNear,vmask,perm4i);

            //perm4i = compactTable[mask];
            //const unsigned int min_dist_index = extract<0>(permute(int4(step),perm4i));

            
            cur = node->child(min_dist_index);
            //cur.prefetch(types);
            perm4i         = permute(perm4i,reverseCompact[setbits]);
            
            const float4 dist4_perm = permute(tNear,perm4i);
            //dist4_perm = permute(dist4_perm,reverseCompact[setbits]);

            const int8 lowHigh32bit0 = permute(node4,lowHighExtract);
            const int8 lowHigh32bit1 = permute4x32(lowHigh32bit0,int8(perm4i));
            const int8 node4_perm    = permute(lowHigh32bit1,lowHighInsert);

            store4f(&stackDist[sindex],dist4_perm);
            store8i(&stackNode[sindex],node4_perm);

            //for (size_t i=0;i<(ssize_t)__popcnt(mask) - 1;i++)
            //  std::cout << "i " << i << " => " << stackNode[sindex+i] << " " << stackDist[sindex+i] << std::endl; 
            sindex += setbits - 1;
            //cur = stackNode[sindex];
            
#else
            
            //const int4 or_mask = select(vmask,step4,allSet4); //optimize
            const int4 vi = (cast(tNear) & mask2) | step4;
            const int4 a0 = select(vmask,vi,int4( True )); //optimize
            const int4 b0 = shuffle<1,0,3,2>(a0);
            const int4 c0 = umin(a0,b0);
            const int4 d0 = umax(a0,b0);
            const int4 a1 = merge(c0,d0,0b0101);
            const int4 b1 = shuffle<2,3,0,1>(a1);
            const int4 c1 = umin(a1,b1);
            const unsigned int min_index0 = extract<0>(c1) & 3;
            assert(min_dist_index < 4);
            cur = node->child(min_index0);
            cur = mask != 0 ? cur : stack_cur;
            const size_t hits = __popcnt(mask);
            sindex += hits;
            if (likely(hits <= 1)) continue;

            int8 node4 = *(int8*)node->children;

            const int4 d1 = umax(a1,b1);
            const int4 a2 = merge(c1,d1,0b0011);
            const int4 b2 = shuffle<0,2,1,3>(a2);
            const int4 c2 = umin(a2,b2);
            const int4 d2 = umax(a2,b2);
            const int4 a3 = merge(c2,d2,0b0010);
            assert(*(unsigned int*)&a3[0] <= *(unsigned int*)&a3[1]);
            assert(*(unsigned int*)&a3[1] <= *(unsigned int*)&a3[2]);
            assert(*(unsigned int*)&a3[2] <= *(unsigned int*)&a3[3]);
            int4 perm4i = a3 & 3;
                          
            perm4i                   = permute(perm4i,reverseCompact[hits]);            
            const float4 dist4_perm  = permute(tNear,perm4i);
            const int8 lowHigh32bit0 = permute(node4,lowHighExtract);
            const int8 lowHigh32bit1 = permute4x32(lowHigh32bit0,int8(perm4i));
            const int8 node4_perm    = permute(lowHigh32bit1,lowHighInsert);

            store4f(&stackDist[sindex-hits+1],dist4_perm);
            store8i(&stackNode[sindex-hits+1],node4_perm);
            
            
#endif



#endif

          }
        
          if (unlikely(cur == BVH4::invalidNode)) break;

          /*! this is a leaf node */
          assert(cur != BVH4::emptyNode);
          STAT3(normal.trav_leaves, 1, 1, 1);
          size_t num; Primitive* prim = (Primitive*)cur.leaf(num);

          
          size_t lazy_node = 0;
          PrimitiveIntersector8::intersect(pre, ray, k, prim, num, bvh->scene, lazy_node);

          /*! stack compaction */
          if (unlikely(any(ray_far != ray.tfar[k])))
          {
            size_t new_sindex = 1;
            for (size_t s=1;s<sindex;s++)
              if (unlikely(stackDist[s] <= ray.tfar[k]))
              {
                stackNode[new_sindex] = stackNode[s];
                stackDist[new_sindex] = stackDist[s];
                new_sindex++;
              }
            sindex = new_sindex;
          }
          ray_far = ray.tfar[k];
        }
      }

#endif
#endif
      AVX_ZERO_UPPER();
    }


#if 0
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
    
    template<int types, bool robust, typename PrimitiveIntersector8>
    void BVH4Intersector8Test<types,robust,PrimitiveIntersector8>::occluded(bool8* valid_i, BVH4* bvh, Ray8& ray)
    {
#if defined(__AVX2__)
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
      const Vec3f8 ray_rdir = rcp_safe(ray_dir);
      ray_tnear = select(valid,ray_tnear,float8(pos_inf));
      ray_tfar  = select(valid,ray_tfar ,float8(neg_inf));
      const float8 inf = float8(pos_inf);
      Precalculations pre(valid,ray);
      const Vec3f8 ray_org_rdir(ray_org*ray_rdir);



      const bool8 active = ray_tnear < ray_tfar;
      size_t m_active = movemask(active);

        /* switch to single ray traversal */
#if 1

#if 0

      /* compute near/far per ray */
      Vec3i8 nearXYZ;
      nearXYZ.x = select(ray_rdir.x >= 0.0f,int8(0*(int)sizeof(float4)),int8(1*(int)sizeof(float4)));
      nearXYZ.y = select(ray_rdir.y >= 0.0f,int8(2*(int)sizeof(float4)),int8(3*(int)sizeof(float4)));
      nearXYZ.z = select(ray_rdir.z >= 0.0f,int8(4*(int)sizeof(float4)),int8(5*(int)sizeof(float4)));

      for (size_t bits = m_active, i=__bsf(bits); bits!=0; bits=__btc(bits,i), i=__bsf(bits)) {
        if (BVH4Intersector8Single<types,robust,PrimitiveIntersector8>::occluded1(bvh,bvh->root,i,pre,ray,ray_org,ray_dir,ray_rdir,ray_tnear,ray_tfar,nearXYZ))
          terminated[i] = -1;
      }
#else

      NodeRef stackNode[8][64];  //!< stack of nodes 
      unsigned int sindex[8];
      size_t m_traversal = (bvh->root.isLeaf(types) ? 0x0 : 0xff) & m_active;
      BVH4::NodeRef cur[8];

      for (size_t bits = m_active, i=__bsf(bits); bits!=0; bits=__blsr(bits), i=__bsf(bits)) 
        {
          sindex[i]       = 1;
          stackNode[i][0] = BVH4::invalidNode;
          cur[i]          = bvh->root;
          //PRINT(i);
          //PRINT(cur[i].isLeaf(types));
          //PRINT(cur[i].isNode(types));
        }


      while(m_active)
      {
        //PRINT("");
        //PRINT("NEW ITERATION");
        //PRINT(m_active);
        //PRINT("TRAVERSAL");
        //PRINT(m_traversal);
        m_traversal &= m_active;
        size_t m_intersection = 0;

        int8 mask8( zero );
        int8 count8( zero );

        for (size_t bits = m_traversal, i=__bsf(bits); bits!=0; bits=__blsr(bits), i=__bsf(bits)) 
        {
          STAT3(shadow.trav_nodes,1,1,1);
          assert(cur[i].isNode(types));
          const BVH4::Node* node = cur[i].node();
          const float4 rdir_x = broadcast1f(&ray_rdir.x[i]);
          const float4 rdir_y = broadcast1f(&ray_rdir.y[i]);
          const float4 rdir_z = broadcast1f(&ray_rdir.z[i]);

          const float4 org_rdir_x = broadcast1f(&ray_org_rdir.x[i]);
          const float4 org_rdir_y = broadcast1f(&ray_org_rdir.y[i]);
          const float4 org_rdir_z = broadcast1f(&ray_org_rdir.z[i]);

          const float4 _tNearX = msub(node->lower_x, rdir_x, org_rdir_x);
          const float4 _tNearY = msub(node->lower_y, rdir_y, org_rdir_y);
          const float4 _tNearZ = msub(node->lower_z, rdir_z, org_rdir_z);
          const float4 _tFarX  = msub(node->upper_x, rdir_x, org_rdir_x);
          const float4 _tFarY  = msub(node->upper_y, rdir_y, org_rdir_y);
          const float4 _tFarZ  = msub(node->upper_z, rdir_z, org_rdir_z);

          const bool4  nactive = float4(pos_inf) != node->lower_x;

          const float4 tNearX  = mini(_tNearX,_tFarX);
          const float4 tNearY  = mini(_tNearY,_tFarY);
          const float4 tNearZ  = mini(_tNearZ,_tFarZ);
            
          const float4 tFarX   = maxi(_tNearX,_tFarX);
          const float4 tFarY   = maxi(_tNearY,_tFarY);
          const float4 tFarZ   = maxi(_tNearZ,_tFarZ);
            
          const float4 tNear = maxi(maxi(tNearX,tNearY),maxi(tNearZ,ray_tnear[i]));
          const float4 tFar  = mini(mini(tFarX ,tFarY ),mini(tFarZ ,ray_tfar[i] ));
          const bool4 vmask  = nactive & (tNear <= tFar);
          unsigned int mask = movemask(vmask);
          mask8[i] = mask;
          //count8[i] = __popcnt(mask);
        }

        for (size_t bits = m_traversal, i=__bsf(bits); bits!=0; bits=__blsr(bits), i=__bsf(bits)) 
        {
          //PRINT(mask);
          //PRINT(__popcnt(mask));

          unsigned int mask = mask8[i];
          const BVH4::Node* node = cur[i].node();

          assert(sindex[i] > 0);
          sindex[i]--;
          cur[i] = stackNode[i][sindex[i]];
          m_intersection &= ~((size_t)1 << i);
          m_intersection |= cur[i].isLeaf(types) ? ((size_t)1 << i) : 0;

          //cur[i].prefetch(types);
          
          if (unlikely(mask == 0)) 
          {
            //PRINT(cur[i]);
            //PRINT(cur[i].isLeaf(types));
            cur[i].prefetchMem();
            continue;
          }
          sindex[i]++;

          STAT3(shadow.trav_hit_boxes[__popcnt(mask)],1,1,1);

          const size_t setbits = __popcnt(mask);

          int8 node4 = *(int8*)node->children;

          int4 perm4i = compactTable[mask];
          const unsigned int min_dist_index = extract<0>(permute(int4(step),perm4i));
            
          cur[i] = node->child(min_dist_index);
          cur[i].prefetchMem();
          m_intersection &= ~((size_t)1 << i);
          m_intersection |= cur[i].isLeaf(types) ? ((size_t)1 << i) : 0;

          //PRINT(cur[i]);
          //PRINT(cur[i].isLeaf(types));
          //PRINT(m_intersection);

          perm4i = permute(perm4i,reverseCompact[setbits]);
            
          const int8 lowHigh32bit0 = permute(node4,lowHighExtract);
          const int8 lowHigh32bit1 = permute4x32(lowHigh32bit0,int8(perm4i));
          const int8 node4_perm    = permute(lowHigh32bit1,lowHighInsert);

          store8i(&stackNode[i][sindex[i]],node4_perm);

          sindex[i] += setbits - 1;
        }

        //PRINT("INTERSECTION");
        //PRINT(m_intersection);
        for (size_t bits = m_intersection, i=__bsf(bits); bits!=0; bits=__blsr(bits), i=__bsf(bits)) 
        {
          do 
          {
            //PRINT(i);
            //PRINT(cur[i]);
            //PRINT(cur[i].isLeaf(types));
            assert(m_active & ((size_t)1 << i));

            assert(cur[i].isLeaf(types));
            if (unlikely(cur[i] == BVH4::invalidNode))
            {
              m_active ^= (size_t)1 << i;
              //PRINT("TERMINATE");
              break;
            }

            /*! this is a leaf node */
            assert(cur[i] != BVH4::emptyNode);
            STAT3(shadow.trav_leaves, 1, 1, 1);
            size_t num; Primitive* prim = (Primitive*)cur[i].leaf(num);

            size_t lazy_node = 0;
            if (PrimitiveIntersector8::occluded(pre,ray,i,prim,num,bvh->scene,lazy_node)) {
              ray.geomID[i] = 0;
              m_active ^= (size_t)1 << i;
              //PRINT("INTERSECTION");
              break;
            }
            
            assert(sindex[i] > 0);
            sindex[i]--;
            cur[i] = stackNode[i][sindex[i]];
            //PRINT(cur[i].isLeaf(types));            
          }
          while(cur[i].isLeaf());
          if (m_active & ((size_t)1 << i))
            assert(cur[i].isNode());
          //PRINT(m_active);
        }       

        //PRINT("FINAL");
        //PRINT(m_active);
      }


#endif

#else

      NodeRef stackNode[stackSizeSingle];  //!< stack of nodes 

      for (size_t bits = m_active, k=__bsf(bits); bits!=0; bits=__blsr(bits), k=__bsf(bits)) 
      {
        size_t sindex = 2;

        stackNode[0] = BVH4::invalidNode;
        stackNode[1] = bvh->root;

        /*! load the ray into SIMD registers */
        //const Vec3f4 org(ray_org.x[k], ray_org.y[k], ray_org.z[k]);
        //const Vec3f4 dir(ray_dir.x[k], ray_dir.y[k], ray_dir.z[k]);
        const Vec3f4 rdir(ray_rdir.x[k], ray_rdir.y[k], ray_rdir.z[k]);
        const Vec3f4 org_rdir(ray_org_rdir.x[k], ray_org_rdir.y[k], ray_org_rdir.z[k]);
        const float4 ray_near(ray_tnear[k]), ray_far(ray_tfar[k]);
        
        /* pop loop */
        while (true)
        {
          /*! pop next node */
          //if (unlikely(sindex == 0)) break;
          assert(sindex < 64);
          sindex--;
          NodeRef cur = NodeRef(stackNode[sindex]);
          STAT3(shadow.trav_stack_pop,1,1,1);
        
          /* downtraversal loop */
          while (true)
          {
            /*! stop if we found a leaf node */
            if (unlikely(cur.isLeaf(types))) break;
            STAT3(shadow.trav_nodes,1,1,1);

            const BVH4::Node* node = cur.node();

#if 1
            const float4 rdir_x = broadcast1f(&ray_rdir.x[k]);
            const float4 rdir_y = broadcast1f(&ray_rdir.y[k]);
            const float4 rdir_z = broadcast1f(&ray_rdir.z[k]);

            const float4 org_rdir_x = broadcast1f(&ray_org_rdir.x[k]);
            const float4 org_rdir_y = broadcast1f(&ray_org_rdir.y[k]);
            const float4 org_rdir_z = broadcast1f(&ray_org_rdir.z[k]);

            const float4 _tNearX = msub(node->lower_x, rdir_x, org_rdir_x);
            const float4 _tNearY = msub(node->lower_y, rdir_y, org_rdir_y);
            const float4 _tNearZ = msub(node->lower_z, rdir_z, org_rdir_z);
            const float4 _tFarX  = msub(node->upper_x, rdir_x, org_rdir_x);
            const float4 _tFarY  = msub(node->upper_y, rdir_y, org_rdir_y);
            const float4 _tFarZ  = msub(node->upper_z, rdir_z, org_rdir_z);

#else
            const float4 _tNearX = msub(node->lower_x, rdir.x, org_rdir.x);
            const float4 _tNearY = msub(node->lower_y, rdir.y, org_rdir.y);
            const float4 _tNearZ = msub(node->lower_z, rdir.z, org_rdir.z);
            const float4 _tFarX  = msub(node->upper_x, rdir.x, org_rdir.x);
            const float4 _tFarY  = msub(node->upper_y, rdir.y, org_rdir.y);
            const float4 _tFarZ  = msub(node->upper_z, rdir.z, org_rdir.z);
#endif
            //const bool4  nactive = float4(pos_inf) == node->lower_x;
            const bool4  nactive = float4(pos_inf) != node->lower_x;

            sindex--;
            const BVH4::NodeRef stack_cur = stackNode[sindex];
            stack_cur.prefetchMem();

            const float4 tNearX  = mini(_tNearX,_tFarX);
            const float4 tNearY  = mini(_tNearY,_tFarY);
            const float4 tNearZ  = mini(_tNearZ,_tFarZ);
            
            const float4 tFarX   = maxi(_tNearX,_tFarX);
            const float4 tFarY   = maxi(_tNearY,_tFarY);
            const float4 tFarZ   = maxi(_tNearZ,_tFarZ);
            
            const float4 tNear = maxi(maxi(tNearX,tNearY),maxi(tNearZ,ray_near));
            const float4 tFar  = mini(mini(tFarX ,tFarY ),mini(tFarZ ,ray_far ));
            //const bool4 vmask  = nactive | cast(tNear) > cast(tFar);
            const bool4 vmask  = nactive & (tNear <= tFar);
            //unsigned int mask = movemask(vmask)^0xf;
            unsigned int mask = movemask(vmask);

            STAT3(shadow.trav_hit_boxes[__popcnt(mask)],1,1,1);

#if 1
            cur = stack_cur;

            if (unlikely(mask == 0)) continue;

            sindex++;

            const size_t setbits = __popcnt(mask);
            //const int4 rc = reverseCompact[setbits];

            //const float4 dist = select(vmask,float4(neg_inf),tNear);
            //const float4 dist_perm = permute(dist,compactTable[mask]);

            
            int8 node4 = *(int8*)node->children;

            // const int4 perm4i = compactTable[mask];
            // float4 dist4_perm = permute(tNear,perm4i);

            // const int8 perm8i = compactTable64[mask];
            // int8   node4_perm = permute(node4,perm8i);

            int4 perm4i;
            //const unsigned int min_dist_index = networkSort(tNear,vmask,perm4i);

            perm4i = compactTable[mask];
            const unsigned int min_dist_index = extract<0>(permute(int4(step),perm4i));

            
            cur = node->child(min_dist_index);
            //cur.prefetchMem();
            perm4i         = permute(perm4i,reverseCompact[setbits]);
            
            //dist4_perm = permute(dist4_perm,reverseCompact[setbits]);

            const int8 lowHigh32bit0 = permute(node4,lowHighExtract);
            const int8 lowHigh32bit1 = permute4x32(lowHigh32bit0,int8(perm4i));
            const int8 node4_perm    = permute(lowHigh32bit1,lowHighInsert);

            store8i(&stackNode[sindex],node4_perm);

            //for (size_t i=0;i<(ssize_t)__popcnt(mask) - 1;i++)
            //  std::cout << "i " << i << " => " << stackNode[sindex+i] << " " << stackDist[sindex+i] << std::endl; 
            sindex += setbits - 1;
            //cur = stackNode[sindex];
            
#else
            
            //const int4 or_mask = select(vmask,step4,allSet4); //optimize
            const int4 vi = (cast(tNear) & mask2) | step4;
            const int4 a0 = select(vmask,vi,int4( True )); //optimize
            const int4 b0 = shuffle<1,0,3,2>(a0);
            const int4 c0 = umin(a0,b0);
            const int4 d0 = umax(a0,b0);
            const int4 a1 = merge(c0,d0,0b0101);
            const int4 b1 = shuffle<2,3,0,1>(a1);
            const int4 c1 = umin(a1,b1);
            const unsigned int min_index0 = extract<0>(c1) & 3;
            assert(min_dist_index < 4);
            cur = node->child(min_index0);
            cur = mask != 0 ? cur : stack_cur;
            const size_t hits = __popcnt(mask);
            sindex += hits;
            if (likely(hits <= 1)) continue;

            int8 node4 = *(int8*)node->children;

            const int4 d1 = umax(a1,b1);
            const int4 a2 = merge(c1,d1,0b0011);
            const int4 b2 = shuffle<0,2,1,3>(a2);
            const int4 c2 = umin(a2,b2);
            const int4 d2 = umax(a2,b2);
            const int4 a3 = merge(c2,d2,0b0010);
            assert(*(unsigned int*)&a3[0] <= *(unsigned int*)&a3[1]);
            assert(*(unsigned int*)&a3[1] <= *(unsigned int*)&a3[2]);
            assert(*(unsigned int*)&a3[2] <= *(unsigned int*)&a3[3]);
            int4 perm4i = a3 & 3;
                          
            perm4i                   = permute(perm4i,reverseCompact[hits]);            
            const int8 lowHigh32bit0 = permute(node4,lowHighExtract);
            const int8 lowHigh32bit1 = permute4x32(lowHigh32bit0,int8(perm4i));
            const int8 node4_perm    = permute(lowHigh32bit1,lowHighInsert);

            store8i(&stackNode[sindex-hits+1],node4_perm);
                        
#endif




          }
        
          if (unlikely(cur == BVH4::invalidNode)) break;

          /*! this is a leaf node */
          assert(cur != BVH4::emptyNode);
          STAT3(shadow.trav_leaves, 1, 1, 1);
          size_t num; Primitive* prim = (Primitive*)cur.leaf(num);

          
          size_t lazy_node = 0;
          if (PrimitiveIntersector8::occluded(pre,ray,k,prim,num,bvh->scene,lazy_node)) {
            ray.geomID[k] = 0;
            break;
          }
        }
      }



#endif
                
      store8i(valid & terminated,&ray.geomID,0);
#endif
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

#endif
