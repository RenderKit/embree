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

#include "bvh8_intersector8_test.h"
#include "../geometry/triangle4.h"
#include "../geometry/triangle8.h"
#include "../geometry/triangle8v.h"
#include "../geometry/intersector_iterators.h"
#include "../geometry/triangle_intersector_moeller.h"
#include "../geometry/triangle_intersector_pluecker.h"
#include "../geometry/triangle_intersector_pluecker2.h"

#define DBG(x) 

#define OLD_TRAVERSAL 0
#define INTERSECTION_CHUNK_THRESHOLD 4
#undef DBG_PRINT
#define DBG_PRINT(x) 
//#define DBG_PRINT(x) PRINT(x)

#define SWITCH_THRESHOLD 2

#if !defined(__WIN32__)

namespace embree
{
  static int shiftTable[8] = {
    1 << 0,
    1 << 1,
    1 << 2,
    1 << 3,
    1 << 4,
    1 << 5,
    1 << 6,
    1 << 7
  };


  namespace isa
  { 

///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////


#if 1
    template<typename PrimitiveIntersector8>    
    void BVH8Intersector8Test<PrimitiveIntersector8>::intersect(bool8* valid_i, BVH8* bvh, Ray8& ray)
    {
#if defined(__AVX2__)
      struct __aligned(32) SharedStackItem {
        BVH8::NodeRef ref;
        size_t mask;
        float8 dist;
      };

      SharedStackItem stack[128];

      /* load ray */
      bool8 valid0 = *valid_i;
#if defined(RTCORE_IGNORE_INVALID_RAYS)
      valid0 &= ray.valid();
#endif
      assert(all(valid0,ray.tnear > -FLT_MIN));
      float8 ray_tnear = ray.tnear;
      float8 ray_tfar  = ray.tfar;

      const Vec3f8 rdir = rcp_safe(ray.dir);
      const Vec3f8 org(ray.org), org_rdir = org * rdir;
      ray_tnear = select(valid0,ray_tnear,float8(pos_inf));
      ray_tfar  = select(valid0,ray_tfar ,float8(neg_inf));
      const float8 inf = float8(pos_inf);
      Precalculations pre(valid0,ray);

      /* compute near/far per ray */
      Vec3i8 nearXYZ;
      nearXYZ.x = select(rdir.x >= 0.0f,int8(0*(int)sizeof(float8)),int8(1*(int)sizeof(float8)));
      nearXYZ.y = select(rdir.y >= 0.0f,int8(2*(int)sizeof(float8)),int8(3*(int)sizeof(float8)));
      nearXYZ.z = select(rdir.z >= 0.0f,int8(4*(int)sizeof(float8)),int8(5*(int)sizeof(float8)));

      // size_t bits = movemask(active);
      // for (size_t i=__bsf(bits); bits!=0; bits=__blsr(bits), i=__bsf(bits)) {
      //   intersect1(bvh,bvh->root,i,pre,ray,ray_org,ray_dir,rdir,ray_tnear,ray_tfar,nearXYZ);
      // }
      unsigned int m_all_active = movemask(ray_tnear < ray_tfar);

      while(m_all_active)
      {
        unsigned int rindex = __bsf(m_all_active);
        unsigned int m_octant_active = movemask( (int8(nearXYZ.x[rindex]) == nearXYZ.x) & 
                                                 (int8(nearXYZ.y[rindex]) == nearXYZ.y) & 
                                                 (int8(nearXYZ.z[rindex]) == nearXYZ.z) );

        stack[0].ref  = bvh->root;
        stack[0].mask = m_octant_active;
        stack[0].dist = min(ray_tfar,ray.tfar);
        size_t sindex = 1;

        const size_t k = __bsf(m_octant_active);

        while(sindex)
        {
          STAT3(normal.trav_stack_pop,1,1,1);

          sindex--;
          NodeRef cur      = stack[sindex].ref;        
          size_t m_current = stack[sindex].mask;
          // optimize: cull stack nodes

          if (likely(cur.isNode()))
          {
            const BVH8::Node* node = cur.node();
            //float8 curDist[8];
            int8 mask8(zero);
            //int8 rays_per_count8( zero );

            const size_t nearX = nearXYZ.x[k];
            const size_t nearY = nearXYZ.y[k];
            const size_t nearZ = nearXYZ.z[k];

            const size_t farX  = nearX ^ sizeof(float8);
            const size_t farY  = nearY ^ sizeof(float8);
            const size_t farZ  = nearZ ^ sizeof(float8);
                        
            const float8 lower_x = load8f((const char*)node+nearX);
            const float8 lower_y = load8f((const char*)node+nearY);
            const float8 lower_z = load8f((const char*)node+nearZ);

            const float8 upper_x = load8f((const char*)node+farX);
            const float8 upper_y = load8f((const char*)node+farY);
            const float8 upper_z = load8f((const char*)node+farZ);


            for (size_t bits=m_current, i=__bsf(bits); bits!=0; bits=__blsr(bits), i=__bsf(bits)) 
            {
              DBG_PRINT(i);
              STAT3(normal.trav_nodes,1,1,1);
              assert(i < 8);

              const float8 ray_rdir_x     = broadcast(&rdir.x[i]);
              const float8 ray_rdir_y     = broadcast(&rdir.y[i]);
              const float8 ray_rdir_z     = broadcast(&rdir.z[i]);
              const float8 ray_org_rdir_x = broadcast(&org_rdir.x[i]);
              const float8 ray_org_rdir_y = broadcast(&org_rdir.y[i]);
              const float8 ray_org_rdir_z = broadcast(&org_rdir.z[i]);

              const float8 tNearX = msub(lower_x, ray_rdir_x, ray_org_rdir_x);
              const float8 tNearY = msub(lower_y, ray_rdir_y, ray_org_rdir_y);
              const float8 tNearZ = msub(lower_z, ray_rdir_z, ray_org_rdir_z);
              const float8 tFarX  = msub(upper_x, ray_rdir_x, ray_org_rdir_x);
              const float8 tFarY  = msub(upper_y, ray_rdir_y, ray_org_rdir_y);
              const float8 tFarZ  = msub(upper_z, ray_rdir_z, ray_org_rdir_z);
              
              const float8 tNear   = maxi(maxi(tNearX,tNearY),maxi(tNearZ,broadcast(&ray_tnear[i])));
              const float8 tFar    = mini(mini(tFarX ,tFarY ),mini(tFarZ ,broadcast(&ray_tfar[i]) ));
              const bool8 vmask    = tNear <= tFar;
              const size_t m_mask  = movemask(vmask);
              const float8 dist    = select(vmask,tNear,inf);               
              //const size_t m_count = __popcnt(m_mask);
              //curDist[i] = dist;
              mask8[i]   = m_mask;
              //rays_per_count8[m_count] |= (unsigned int)1 << i;
            }
            size_t m_ray_hit = movemask(mask8 != int8(zero)); 
            if (unlikely(m_ray_hit == 0)) continue;

            STAT(size_t iter= 0);
            do {
              STAT(iter++);
              assert(m_ray_hit);
              const size_t index = __bsf(m_ray_hit);
              const size_t m_equal = movemask(mask8 == int8(mask8[index]));
              assert(m_equal);
              m_ray_hit &=~m_equal;
              const size_t mask = mask8[index];
              const size_t count_mask = __popcnt(mask);
              assert(mask);
              if (likely(count_mask == 1))
              {
                const size_t node_index = __bsf(mask); 
                stack[sindex].dist = ray_tfar;                
                stack[sindex].mask = m_equal;
                stack[sindex].ref  = node->child(node_index);                  
                sindex++;
              }                
              else
              {
                for (size_t bits=mask, i=__bsf(bits); bits!=0; bits=__blsr(bits), i=__bsf(bits)) 
                {
                  stack[sindex].dist = ray_tfar;                
                  stack[sindex].mask = m_equal;
                  stack[sindex].ref  = node->child(i);                  
                  sindex++;                    
                }                  
              }
            } while(m_ray_hit);            

            STAT3(normal.trav_hit_boxes[iter],1,1,1);

          }
          else
          {

            /*! this is a leaf node */
            assert(cur != BVH8::emptyNode);
            assert(m_current);

            STAT3(normal.trav_leaves,1,1,1);
            size_t num; Triangle* prim = (Triangle*) cur.leaf(num);
            size_t lazy_node = 0;

            if (unlikely(__popcnt(m_current) >= INTERSECTION_CHUNK_THRESHOLD))
            {
              // optimize mask to intersector
              PrimitiveIntersector8::intersect(bool8((int)m_current),pre,ray,prim,num,bvh->scene,lazy_node);
              ray_tfar = min(ray.tfar,ray_tfar);          
            }
            else
              for (size_t bits=m_current, i=__bsf(bits); bits!=0; bits=__blsr(bits), i=__bsf(bits)) 
              {
                PrimitiveIntersector8::intersect(pre,ray,i,prim,num,bvh->scene,lazy_node);
                ray_tfar[i] = ray.tfar[i];
              }            
          }
          
          
        }


        m_all_active &= ~m_octant_active;
      }

#endif
      AVX_ZERO_UPPER();
    }

#endif


///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////

#if 0
    template<typename PrimitiveIntersector8>    
    void BVH8Intersector8Test<PrimitiveIntersector8>::intersect(bool8* valid_i, BVH8* bvh, Ray8& ray)
    {
#if defined(__AVX2__)
      struct __aligned(16) StackItemHybrid  {

        /*! assert that the xchg function works */

        union { float dist; unsigned int dist_i; };
        unsigned int mask;
        BVH8::NodeRef ref;
        __forceinline StackItemHybrid() {
          assert(sizeof(StackItemHybrid) == 16);

        }
        __forceinline StackItemHybrid(const float dist, const unsigned int mask, const BVH8::NodeRef ref) :
          dist(dist), mask(mask), ref(ref) {}

        /*! use SSE instructions to swap stack items */
        __forceinline static void xchg(StackItemHybrid& a, StackItemHybrid& b) 
        { 
          const float4 sse_a = load4f(&a); 
          const float4 sse_b = load4f(&b);
          store4f(&a,sse_b);
          store4f(&b,sse_a);
        }

        __forceinline static void compare_xchg(int4& a, int4& b) 
        {
          //int4 a_dist = shuffle<0,0,0,0>(a);
          //int4 b_dist = shuffle<0,0,0,0>(b);
          bool4 mask  = shuffle<0,0,0,0>(b < a);
          int4 a_new  = select(mask,a,b);
          int4 b_new  = select(mask,b,a);
          a = a_new;
          b = b_new;
        }

        /*! Sort 3 stack items. */
        __forceinline static void sort3(StackItemHybrid& s1, StackItemHybrid& s2, StackItemHybrid& s3)
        {
#if 0
          if (s2.dist < s1.dist) xchg(s2,s1);
          if (s3.dist < s2.dist) xchg(s3,s2);
          if (s2.dist < s1.dist) xchg(s2,s1);
#else
          int4 a1 = *(int4*)&s1;
          int4 a2 = *(int4*)&s2;
          int4 a3 = *(int4*)&s3;
          compare_xchg(a2,a1);
          compare_xchg(a3,a2);
          compare_xchg(a2,a1);
          *(int4*)&s1 = a1;
          *(int4*)&s2 = a2;
          *(int4*)&s3 = a3;
#endif
        }
    
        /*! Sort 4 stack items. */
        __forceinline static void sort4(StackItemHybrid& s1, StackItemHybrid& s2, StackItemHybrid& s3, StackItemHybrid& s4)
        {
#if 1
          if (s2.dist < s1.dist) xchg(s2,s1);
          if (s4.dist < s3.dist) xchg(s4,s3);
          if (s3.dist < s1.dist) xchg(s3,s1);
          if (s4.dist < s2.dist) xchg(s4,s2);
          if (s3.dist < s2.dist) xchg(s3,s2);
#else
          int4 a1 = *(int4*)&s1;
          int4 a2 = *(int4*)&s2;
          int4 a3 = *(int4*)&s3;
          int4 a4 = *(int4*)&s4;
          compare_xchg(a2,a1);
          compare_xchg(a4,a3);
          compare_xchg(a3,a1);
          compare_xchg(a4,a2);
          compare_xchg(a3,a2);
          *(int4*)&s1 = a1;
          *(int4*)&s2 = a2;
          *(int4*)&s3 = a3;          
          *(int4*)&s3 = a4;          
#endif
        }


      } stack[stackSizeChunk];
      
      /* load ray */
      bool8 valid0 = *valid_i;
#if defined(RTCORE_IGNORE_INVALID_RAYS)
      valid0 &= ray.valid();
#endif
      assert(all(valid0,ray.tnear > -FLT_MIN));
      float8 ray_tnear = ray.tnear;
      float8 ray_tfar  = ray.tfar;

      const Vec3f8 rdir = rcp_safe(ray.dir);
      const Vec3f8 org(ray.org), org_rdir = org * rdir;
      ray_tnear = select(valid0,ray_tnear,float8(pos_inf));
      ray_tfar  = select(valid0,ray_tfar ,float8(neg_inf));
      const float8 inf = float8(pos_inf);
      Precalculations pre(valid0,ray);

      /* compute near/far per ray */
      Vec3i8 nearXYZ;
      nearXYZ.x = select(rdir.x >= 0.0f,int8(0*(int)sizeof(float8)),int8(1*(int)sizeof(float8)));
      nearXYZ.y = select(rdir.y >= 0.0f,int8(2*(int)sizeof(float8)),int8(3*(int)sizeof(float8)));
      nearXYZ.z = select(rdir.z >= 0.0f,int8(4*(int)sizeof(float8)),int8(5*(int)sizeof(float8)));

      // size_t bits = movemask(active);
      // for (size_t i=__bsf(bits); bits!=0; bits=__blsr(bits), i=__bsf(bits)) {
      //   intersect1(bvh,bvh->root,i,pre,ray,ray_org,ray_dir,rdir,ray_tnear,ray_tfar,nearXYZ);
      // }
      unsigned int m_root_active = movemask(ray_tnear < ray_tfar);

      while(m_root_active)
      {
        unsigned int rindex = __bsf(m_root_active);
        unsigned int m_octant_active = movemask( (int8(nearXYZ.x[rindex]) == nearXYZ.x) & 
                                                 (int8(nearXYZ.y[rindex]) == nearXYZ.y) & 
                                                 (int8(nearXYZ.z[rindex]) == nearXYZ.z) );

        m_root_active &= ~m_octant_active;

        stack[0] = StackItemHybrid(pos_inf,m_octant_active,BVH8::invalidNode);
        stack[1] = StackItemHybrid(0.0f   ,m_octant_active,bvh->root);

        size_t sindex = 2;

        while(1) pop:
        {
          STAT3(normal.trav_stack_pop,1,1,1);
          DBG_PRINT("STACK_POP");
          assert(sindex);
          sindex--;
          NodeRef cur           = stack[sindex].ref;        
          float8 curDist        = broadcast(&stack[sindex].dist);
          unsigned int m_active = stack[sindex].mask & movemask(curDist < ray_tfar);
          if (unlikely(cur == BVH8::invalidNode)) break;
          if (unlikely(m_active == 0)) continue;
        
          DBG_PRINT(cur);
          DBG_PRINT(m_active);

#if 0
          {
            size_t bits = m_active;
            if (unlikely(__popcnt(bits) <= SWITCH_THRESHOLD)) {
              for (size_t i=__bsf(bits); bits!=0; bits=__btc(bits,i), i=__bsf(bits)) {
                intersect1(bvh,cur,i,pre,ray,ray.org,ray.dir,rdir,ray_tnear,ray_tfar,nearXYZ);
              }
              ray_tfar = ray.tfar;
              continue;
            }
          }
#endif

          const size_t k = __bsf(m_active);
          const size_t nearX = nearXYZ.x[k];
          const size_t nearY = nearXYZ.y[k];
          const size_t nearZ = nearXYZ.z[k];

          while (1)
          {
            DBG_PRINT("TRAVERSAL_STEP");
            DBG_PRINT(cur);
            DBG_PRINT(m_active);
          
            assert(m_active);
            assert(__popcnt(m_active) <= __popcnt(m_octant_active));

            /* test if this is a leaf node */
            if (unlikely(cur.isLeaf())) break;
          
            int8 rays_mask_per_node(zero);
            float8 hit_dist(inf);
            const BVH8::Node* node = cur.node();

            DBG_PRINT(__popcnt(m_active));
            DBG_PRINT(rays_mask_per_node);

            STAT3(normal.trav_hit_boxes[__popcnt(m_active)],1,1,1);

            const size_t farX  = nearX ^ sizeof(float8);
            const size_t farY  = nearY ^ sizeof(float8);
            const size_t farZ  = nearZ ^ sizeof(float8);
                        
            const float8 lower_x = load8f((const char*)node+nearX);
            const float8 lower_y = load8f((const char*)node+nearY);
            const float8 lower_z = load8f((const char*)node+nearZ);

            const float8 upper_x = load8f((const char*)node+farX);
            const float8 upper_y = load8f((const char*)node+farY);
            const float8 upper_z = load8f((const char*)node+farZ);

            for (size_t bits=m_active, i=__bsf(bits); bits!=0; bits=__blsr(bits), i=__bsf(bits)) 
            {
              DBG_PRINT(i);

              STAT3(normal.trav_nodes,1,1,1);
              assert(i < 8);
              const float8 ray_rdir_x     = broadcast(&rdir.x[i]);
              const float8 ray_rdir_y     = broadcast(&rdir.y[i]);
              const float8 ray_rdir_z     = broadcast(&rdir.z[i]);
              const float8 ray_org_rdir_x = broadcast(&org_rdir.x[i]);
              const float8 ray_org_rdir_y = broadcast(&org_rdir.y[i]);
              const float8 ray_org_rdir_z = broadcast(&org_rdir.z[i]);


              const float8 _tNearX = msub(lower_x, ray_rdir_x, ray_org_rdir_x);
              const float8 _tNearY = msub(lower_y, ray_rdir_y, ray_org_rdir_y);
              const float8 _tNearZ = msub(lower_z, ray_rdir_z, ray_org_rdir_z);
              const float8 _tFarX  = msub(upper_x, ray_rdir_x, ray_org_rdir_x);
              const float8 _tFarY  = msub(upper_y, ray_rdir_y, ray_org_rdir_y);
              const float8 _tFarZ  = msub(upper_z, ray_rdir_z, ray_org_rdir_z);

              //const float8 _tNearX = msub(node->lower_x, ray_rdir_x, ray_org_rdir_x);
              //const float8 _tNearY = msub(node->lower_y, ray_rdir_y, ray_org_rdir_y);
              //const float8 _tNearZ = msub(node->lower_z, ray_rdir_z, ray_org_rdir_z);
              //const float8 _tFarX  = msub(node->upper_x, ray_rdir_x, ray_org_rdir_x);
              //const float8 _tFarY  = msub(node->upper_y, ray_rdir_y, ray_org_rdir_y);
              //const float8 _tFarZ  = msub(node->upper_z, ray_rdir_z, ray_org_rdir_z);

              //const bool8  nactive = inf != node->lower_x;

              const int8 m_node = broadcast(&shiftTable[i]);
              //const float8 tNearX  = mini(_tNearX,_tFarX);
              //const float8 tNearY  = mini(_tNearY,_tFarY);
              //const float8 tNearZ  = mini(_tNearZ,_tFarZ);
          
              //const float8 tFarX   = maxi(_tNearX,_tFarX);
              //const float8 tFarY   = maxi(_tNearY,_tFarY);
              //const float8 tFarZ   = maxi(_tNearZ,_tFarZ);
            
              //const float8 tNear = maxi(maxi(tNearX,tNearY),maxi(tNearZ,broadcast(&ray_tnear[i])));
              //const float8 tFar  = mini(mini(tFarX ,tFarY ),mini(tFarZ ,broadcast(&ray_tfar[i])));
              //const bool8 vmask  = nactive & (tNear <= tFar);
              //STAT3(normal.trav_hit_boxes[__popcnt(movemask(vmask))],1,1,1);

              const float8 tNearX  = _tNearX;
              const float8 tNearY  = _tNearY;
              const float8 tNearZ  = _tNearZ;
              
              const float8 tFarX   = _tFarX;
              const float8 tFarY   = _tFarY;
              const float8 tFarZ   = _tFarZ;
              
              const float8 tNear = maxi(maxi(tNearX,tNearY),maxi(tNearZ,broadcast(&ray_tnear[i])));
              const float8 tFar  = mini(mini(tFarX ,tFarY ),mini(tFarZ ,broadcast(&ray_tfar[i]) ));
              const bool8 vmask = tNear <= tFar;

              DBG_PRINT(vmask);

              const float8 dist = select(vmask,tNear,inf);               

              rays_mask_per_node |= m_node & int8((__m256i)vmask);
              DBG_PRINT(m_node & int8((__m256i)vmask));
              DBG_PRINT(rays_mask_per_node);

              hit_dist = mini(hit_dist,dist); //optimize
              DBG_PRINT(dist);
              DBG_PRINT(hit_dist);
            }
          
            DBG_PRINT("TRAVERSAL_SORT");
            DBG_PRINT(rays_mask_per_node);

            size_t hit_bits = movemask(rays_mask_per_node != int8(zero));

            DBG_PRINT(hit_bits);
            DBG_PRINT(__popcnt(hit_bits));

            /*! if no child is hit, pop next node */
            if (likely(hit_bits == 0))
            {
              DBG_PRINT("CASE 0");
              //STAT3(normal.trav_hit_boxes[0],1,1,1);
              goto pop;
            }

            /*! one child is hit, continue with that child */
            const size_t index0 = __bsf(hit_bits); hit_bits = __blsr(hit_bits);
            assert(rays_mask_per_node[index0]);
            assert(hit_dist[index0] != (float)pos_inf);
            assert(__popcnt(rays_mask_per_node[index0]) <= __popcnt(m_active));
            
            if (likely(hit_bits == 0)) 
            {
              DBG_PRINT("CASE 1");
              //STAT3(normal.trav_hit_boxes[1],1,1,1);
              cur = node->child(index0);
              m_active = rays_mask_per_node[index0];
              continue;            
            }

            /*! two children are hit, push far child, and continue with closer child */
            const size_t index1 = __bsf(hit_bits); hit_bits = __blsr(hit_bits);
            assert(rays_mask_per_node[index1]);
            assert(hit_dist[index1] != (float)pos_inf);
            assert(__popcnt(rays_mask_per_node[index1]) <= __popcnt(m_active));

            if (likely(hit_bits == 0)) 
            {
              DBG_PRINT("CASE 2");
              //STAT3(normal.trav_hit_boxes[2],1,1,1);

              if ( ((unsigned int*)&hit_dist)[index0] < ((unsigned int*)&hit_dist)[index1])
              {
                cur = node->child(index0);
                m_active = rays_mask_per_node[index0];
                stack[sindex++] = StackItemHybrid(hit_dist[index1],rays_mask_per_node[index1],node->child(index1));
                node->child(index1).prefetch();
                continue;            
              }
              else
              {
                cur = node->child(index1);
                m_active = rays_mask_per_node[index1];
                stack[sindex++] = StackItemHybrid(hit_dist[index0],rays_mask_per_node[index0],node->child(index0));
                node->child(index0).prefetch();
                continue;                          
              }
            }

            const size_t index2 = __bsf(hit_bits); hit_bits = __blsr(hit_bits);
            assert(rays_mask_per_node[index2]);
            assert(hit_dist[index2] != (float)pos_inf);
            assert(__popcnt(rays_mask_per_node[index2]) <= __popcnt(m_active));

            /*! three children are hit, push all onto stack and sort 3 stack items, continue with closest child */

            stack[sindex+0] = StackItemHybrid(hit_dist[index0],rays_mask_per_node[index0],node->child(index0));
            stack[sindex+1] = StackItemHybrid(hit_dist[index1],rays_mask_per_node[index1],node->child(index1));
            stack[sindex+2] = StackItemHybrid(hit_dist[index2],rays_mask_per_node[index2],node->child(index2));

            if (likely(hit_bits == 0)) 
            {
              DBG_PRINT("CASE 3");
              //STAT3(normal.trav_hit_boxes[3],1,1,1);
              StackItemHybrid::sort3(stack[sindex+0],stack[sindex+1],stack[sindex+2]);
              cur      = stack[sindex+2].ref;
              cur.prefetch();
              m_active = stack[sindex+2].mask;
              sindex += 2;
              continue;
            }          


            const size_t index3 = __bsf(hit_bits); hit_bits = __blsr(hit_bits);
            assert(rays_mask_per_node[index3]);
            assert(hit_dist[index3] != (float)pos_inf);
            assert(__popcnt(rays_mask_per_node[index3]) <= __popcnt(m_active));

            /*! four children are hit, push all onto stack and sort 4 stack items, continue with closest child */

            stack[sindex+3] = StackItemHybrid(hit_dist[index3],rays_mask_per_node[index3],node->child(index3));

            StackItemHybrid::sort4(stack[sindex+0],stack[sindex+1],stack[sindex+2],stack[sindex+3]);
            if (likely(hit_bits == 0)) 
            {
              DBG_PRINT("CASE 4");
              //STAT3(normal.trav_hit_boxes[4],1,1,1);
              cur      = stack[sindex+3].ref;
              cur.prefetch();
              m_active = stack[sindex+3].mask;
              sindex += 3;
              continue;
            }

            /*! more than four children are hit, push all onto stack */
            DBG_PRINT("CASE 4+");
            sindex += 4;
            //STAT3(normal.trav_hit_boxes[4+__popcnt(hit_bits)],1,1,1);

            for (size_t bits=hit_bits, i=__bsf(bits); bits!=0; bits=__blsr(bits), i=__bsf(bits)) 
              stack[sindex++] = StackItemHybrid(hit_dist[i],rays_mask_per_node[i],node->child(i));
            sindex--;
            cur      = stack[sindex].ref;
            cur.prefetch();

            m_active = stack[sindex].mask;          
                                
          }


          ///* return if stack is empty */
          //if (unlikely(cur == BVH8::invalidNode)) break;

          DBG_PRINT("INTERSECTION_STEP");
          DBG_PRINT(cur);
          DBG_PRINT(m_active);

          /*! this is a leaf node */
          assert(cur != BVH8::emptyNode);
          assert(m_active);

          STAT3(normal.trav_leaves,1,1,1);
          size_t num; Triangle* prim = (Triangle*) cur.leaf(num);
          size_t lazy_node = 0;

          if (__popcnt(m_active) >= INTERSECTION_CHUNK_THRESHOLD)
          {
            PrimitiveIntersector8::intersect(bool8((int)m_active),pre,ray,prim,num,bvh->scene,lazy_node);
            ray_tfar = min(ray.tfar,ray_tfar);          
          }
          else
            for (size_t bits=m_active, i=__bsf(bits); bits!=0; bits=__blsr(bits), i=__bsf(bits)) 
            {
              PrimitiveIntersector8::intersect(pre,ray,i,prim,num,bvh->scene,lazy_node);
              ray_tfar[i] = ray.tfar[i];
            }
        
        }
      }
#endif
      //PRINT(ray);
      //exit(0);
      AVX_ZERO_UPPER();
    }
#endif

#if 0
    template<typename PrimitiveIntersector8>    
    void BVH8Intersector8Test<PrimitiveIntersector8>::intersect(bool8* valid_i, BVH8* bvh, Ray8& ray)
    {
      //Context context[8];
      StackItemT<BVH8::NodeRef>  contextStack[8][stackSizeSingle];  //!< stack of nodes 
      
      /* load ray */
      bool8 valid0 = *valid_i;
#if defined(RTCORE_IGNORE_INVALID_RAYS)
      valid0 &= ray.valid();
#endif
      assert(all(valid0,ray.tnear > -FLT_MIN));
      Vec3f8 ray_org = ray.org;
      Vec3f8 ray_dir = ray.dir;
      float8 ray_tnear = ray.tnear, ray_tfar  = ray.tfar;

      const Vec3f8 rdir = rcp_safe(ray_dir);
      const Vec3f8 org(ray_org), org_rdir = org * rdir;
      ray_tnear = select(valid0,ray_tnear,float8(pos_inf));
      ray_tfar  = select(valid0,ray_tfar ,float8(neg_inf));
      const float8 inf = float8(pos_inf);
      Precalculations pre(valid0,ray);

      /* compute near/far per ray */
      Vec3i8 nearXYZ;
      nearXYZ.x = select(rdir.x >= 0.0f,int8(0*(int)sizeof(float8)),int8(1*(int)sizeof(float8)));
      nearXYZ.y = select(rdir.y >= 0.0f,int8(2*(int)sizeof(float8)),int8(3*(int)sizeof(float8)));
      nearXYZ.z = select(rdir.z >= 0.0f,int8(4*(int)sizeof(float8)),int8(5*(int)sizeof(float8)));

        /* switch to single ray traversal */
      const bool8 active = ray_tnear < ray_tfar;

#if 1 // !defined(__WIN32__) || defined(__X86_64__)
      size_t bits = movemask(active);
      for (size_t i=__bsf(bits); bits!=0; bits=__blsr(bits), i=__bsf(bits)) {
        intersect1(bvh,bvh->root,i,pre,ray,ray_org,ray_dir,rdir,ray_tnear,ray_tfar,nearXYZ);
      }
#else
      size_t g_active = movemask(active);
      size_t bits = g_active;
      for (size_t i=__bsf(bits); bits!=0; bits=__blsr(bits), i=__bsf(bits)) 
        context[i].init(bvh->root,contextStack[i]);

#define DBG_TEST(x) 

      while(g_active != 0)
      {
        size_t m_intersect = 0;
        for (size_t bits = g_active, i=__bsf(bits); bits!=0; bits=__blsr(bits), i=__bsf(bits)) 
        {
          DBG_TEST(PRINT(i));
          context[i].resetMask(); //
          BVH8::NodeRef cur = context[i].pop();
          if (unlikely(cur.isLeaf()))
          {
            const size_t m_slot = (size_t)1 << i;
            if (unlikely(cur == BVH8::invalidNode))
            {
              DBG_TEST(PRINT2("TERMINATED",i));
              g_active ^= m_slot;
              continue;
            }
            m_intersect |= m_slot;
            cur.prefetch();
          }
          else
          {
            DBG_TEST(PRINT(i));
            assert( cur != BVH8::invalidNode );
            DBG_TEST(PRINT("NODE"));
            context[i].intersectNode(cur,rdir,org_rdir,ray_tnear,ray_tfar,i);
            context[i].sortNodesDirect(cur);      
            context[i].prefetchNext();
          }
        }

        if (unlikely(g_active == 0)) break;

        for (size_t bits = m_intersect, i=__bsf(bits); bits!=0; bits=__blsr(bits), i=__bsf(bits)) 
        {
          BVH8::NodeRef cur = context[i].getCurrent();
          DBG_TEST(PRINT(i));
          assert( cur != BVH8::invalidNode );
          STAT3(normal.trav_leaves,1,1,1);
          DBG_TEST(PRINT("LEAF"));
          size_t num; 
          Triangle* prim = (Triangle*) cur.leaf(num);
          const float old_tfar = ray.tfar[i];
          size_t lazy_node = 0;
          PrimitiveIntersector8::intersect(pre,ray,i,prim,num,bvh->scene,lazy_node);
          if (unlikely(ray.tfar[i] < old_tfar))
              context[i].compactStack(ray.tfar[i]);
          
        }

#if 0
        DBG_TEST(PRINT("SORT"));
        for (size_t bits = m_traversal, i=__bsf(bits); bits!=0; bits=__btc(bits,i), i=__bsf(bits)) 
        {
          if (unlikely(context[i].hit_mask))
          {
            DBG_TEST(PRINT2("SORT",i));
            context[i].sortNodes();          
          }
          context[i].prefetchNext();
        }
#endif        
        
      }
#endif
      AVX_ZERO_UPPER();
    }
#endif

    template<typename PrimitiveIntersector8>
    __forceinline void BVH8Intersector8Test<PrimitiveIntersector8>::intersect1(const BVH8* bvh, NodeRef root, const size_t k, Precalculations& pre, Ray8& ray,const Vec3f8 &ray_org, const Vec3f8 &ray_dir, const Vec3f8 &ray_rdir, const float8 &ray_tnear, const float8 &ray_tfar, const Vec3i8& nearXYZ)
    {
#if defined (__AVX2__)
      /*! stack state */
      StackItemT<NodeRef> stack[stackSizeSingle];  //!< stack of nodes 
      StackItemT<NodeRef>* stackPtr = stack+2;        //!< current stack pointer
      //StackItemT<NodeRef>* stackEnd = stack+stackSizeSingle;
      stack[0].ptr  = BVH8::invalidNode;
      stack[0].dist = inf;
      stack[1].ptr  = root;
      stack[1].dist = neg_inf;
      
      /*! offsets to select the side that becomes the lower or upper bound */
      //const size_t nearX = nearXYZ.x[k];
      //const size_t nearY = nearXYZ.y[k];
      //const size_t nearZ = nearXYZ.z[k];

      /*! load the ray into SIMD registers */
      const Vec3f8 org (ray_org .x[k],ray_org .y[k],ray_org .z[k]);
      const Vec3f8 rdir(ray_rdir.x[k],ray_rdir.y[k],ray_rdir.z[k]);
      const Vec3f8 norg = -org;
      const Vec3f8 org_rdir(org*rdir);
      float8 rayNear(ray_tnear[k]), rayFar(ray_tfar[k]);
/* pop loop */

      while (true) //pop:
      {
        stackPtr--;
        NodeRef cur = stackPtr->ptr;

        /*! pop next node */
        //if (unlikely(stackPtr == stack)) break;
        //stackPtr--;
        
        /*! if popped node is too far, pop next one */
        //if (unlikely(*(float*)&stackPtr->dist > ray.tfar[k]))
        //continue;
        
        /* downtraversal loop */
        const float8 inactive_inf = float8(pos_inf);

        while (true)
        {
          /*! stop if we found a leaf */
          if (unlikely(cur.isLeaf())) break;
          STAT3(normal.trav_nodes,1,1,1);
          
          /*! single ray intersection with 4 boxes */
          const Node* node = cur.node();

#if OLD_TRAVERSAL == 1

          const size_t farX  = nearX ^ sizeof(float8), farY  = nearY ^ sizeof(float8), farZ  = nearZ ^ sizeof(float8);
          const float8 _tNearX = msub(load8f((const char*)node+nearX), rdir.x, org_rdir.x);
          const float8 _tNearY = msub(load8f((const char*)node+nearY), rdir.y, org_rdir.y);
          const float8 _tNearZ = msub(load8f((const char*)node+nearZ), rdir.z, org_rdir.z);
          const float8 _tFarX  = msub(load8f((const char*)node+farX ), rdir.x, org_rdir.x);
          const float8 _tFarY  = msub(load8f((const char*)node+farY ), rdir.y, org_rdir.y);
          const float8 _tFarZ  = msub(load8f((const char*)node+farZ ), rdir.z, org_rdir.z);

          const float8 tNearX  = _tNearX;
          const float8 tNearY  = _tNearY;
          const float8 tNearZ  = _tNearZ;

          const float8 tFarX   = _tFarX;
          const float8 tFarY   = _tFarY;
          const float8 tFarZ   = _tFarZ;

          const float8 tNear = maxi(maxi(tNearX,tNearY),maxi(tNearZ,rayNear));
          const float8 tFar  = mini(mini(tFarX ,tFarY ),mini(tFarZ ,rayFar ));
          const bool8 vmask = cast(tNear) > cast(tFar);
          size_t mask = movemask(vmask)^0xff;

#else
          const float8 _tNearX = msub(node->lower_x, rdir.x, org_rdir.x);
          const float8 _tNearY = msub(node->lower_y, rdir.y, org_rdir.y);
          const float8 _tNearZ = msub(node->lower_z, rdir.z, org_rdir.z);
          const float8 _tFarX  = msub(node->upper_x, rdir.x, org_rdir.x);
          const float8 _tFarY  = msub(node->upper_y, rdir.y, org_rdir.y);
          const float8 _tFarZ  = msub(node->upper_z, rdir.z, org_rdir.z);
          stackPtr--;
          const bool8  nactive = inactive_inf == node->lower_x;
          cur = stackPtr->ptr; 
          
          const float8 tNearX  = mini(_tNearX,_tFarX);
          const float8 tNearY  = mini(_tNearY,_tFarY);
          const float8 tNearZ  = mini(_tNearZ,_tFarZ);

          const float8 tFarX   = maxi(_tNearX,_tFarX);
          const float8 tFarY   = maxi(_tNearY,_tFarY);
          const float8 tFarZ   = maxi(_tNearZ,_tFarZ);

          const float8 tNear   = maxi(maxi(tNearX,tNearY),maxi(tNearZ,rayNear));
          const float8 tFar    = mini(mini(tFarX ,tFarY ),mini(tFarZ ,rayFar ));
          const bool8 vmask    = nactive | cast(tNear) > cast(tFar);

          size_t mask = movemask(vmask)^0xff;

#endif
          
          /*! if no child is hit, pop next node */
          if (unlikely(mask == 0)) continue;
            //goto pop;
          stackPtr++;

          /*! one child is hit, continue with that child */
          size_t r = __bscf(mask);
          if (likely(mask == 0)) {
            cur = node->child(r); 
            cur.prefetch();
            assert(cur != BVH8::emptyNode);
            continue;
          }
          
          /*! two children are hit, push far child, and continue with closer child */
          NodeRef c0 = node->child(r); c0.prefetch(); const unsigned int d0 = ((unsigned int*)&tNear)[r];
          r = __bscf(mask);
          NodeRef c1 = node->child(r); c1.prefetch(); const unsigned int d1 = ((unsigned int*)&tNear)[r];
          assert(c0 != BVH8::emptyNode);
          assert(c1 != BVH8::emptyNode);
          if (likely(mask == 0)) {
            if (d0 < d1) { stackPtr->ptr = c1; stackPtr->dist = d1; stackPtr++; cur = c0; continue; }
            else         { stackPtr->ptr = c0; stackPtr->dist = d0; stackPtr++; cur = c1; continue; }
          }
          
          /*! Here starts the slow path for 3 or 4 hit children. We push
           *  all nodes onto the stack to sort them there. */
          stackPtr->ptr = c0; stackPtr->dist = d0; stackPtr++;
          stackPtr->ptr = c1; stackPtr->dist = d1; stackPtr++;
          
          /*! three children are hit, push all onto stack and sort 3 stack items, continue with closest child */
          r = __bscf(mask);
          NodeRef c = node->child(r); c.prefetch(); unsigned int d = ((unsigned int*)&tNear)[r]; stackPtr->ptr = c; stackPtr->dist = d; stackPtr++;
          assert(c != BVH8::emptyNode);
          if (likely(mask == 0)) {
            sort(stackPtr[-1],stackPtr[-2],stackPtr[-3]);
            cur = (NodeRef) stackPtr[-1].ptr; stackPtr--;
            continue;
          }
          
	  /*! four children are hit, push all onto stack and sort 4 stack items, continue with closest child */
          r = __bscf(mask);
          c = node->child(r); c.prefetch(); d = *(unsigned int*)&tNear[r]; stackPtr->ptr = c; stackPtr->dist = d; stackPtr++;
	  if (likely(mask == 0)) {
	    sort(stackPtr[-1],stackPtr[-2],stackPtr[-3],stackPtr[-4]);
	    cur = (NodeRef) stackPtr[-1].ptr; stackPtr--;
	    continue;
	  }

	  /*! fallback case if more than 4 children are hit */
	  while (1)
	  {
	    r = __bscf(mask);
	    c = node->child(r); c.prefetch(); d = *(unsigned int*)&tNear[r]; stackPtr->ptr = c; stackPtr->dist = d; stackPtr++;
	    if (unlikely(mask == 0)) break;
	  }
	  
	  cur = (NodeRef) stackPtr[-1].ptr; stackPtr--;
	}
        
        /* return if stack is empty */
        if (unlikely(cur == BVH8::invalidNode)) break;

        /*! this is a leaf node */
	assert(cur != BVH8::emptyNode);
        STAT3(normal.trav_leaves,1,1,1);
        size_t num; Triangle* prim = (Triangle*) cur.leaf(num);

        const float old_tfar = ray.tfar[k];

        size_t lazy_node = 0;
        PrimitiveIntersector8::intersect(pre,ray,k,prim,num,bvh->scene,lazy_node);
#if 1
        if (unlikely(ray.tfar[k] < old_tfar))
        {
          StackItemT<NodeRef>* new_stackPtr = stack+1;        //!< current stack pointer            
          for (StackItemT<NodeRef>*s = stack+1; s != stackPtr; s++)
            if (unlikely(*(float*)&s->dist < ray.tfar[k]))
              *new_stackPtr++ = *s;
          stackPtr = new_stackPtr;
        }
#endif
        rayFar = ray.tfar[k];

      }
#endif
    }
   

    struct __aligned(64) Context 
    {
      static const size_t stackSizeSingle = 64;

      Context() {}

      __forceinline void init(BVH8::NodeRef &root,
                              StackItemT<BVH8::NodeRef> *contextStack) 
      { 
        current       = root; 
        stack         = contextStack;
        stack[0].ptr  = BVH8::invalidNode;
        stack[0].dist = inf;
        stack[1].ptr  = root;
        stack[1].dist = inf;
        stackPtr      = stack + 2;
      } 
      
      __forceinline BVH8::NodeRef pop()
      {
        stackPtr--;
        current = stackPtr->ptr;
        return current;
      }

      __forceinline BVH8::NodeRef getCurrent()
      {
        return current;
      }

      __forceinline void prefetchNext()
      {
        stackPtr[-1].ptr.prefetch();
      }
      
      __forceinline void resetMask()
      {
        hit_mask = 0;
      }

      __forceinline bool terminated()
      {
        return current == BVH8::invalidNode;
      }
      
      __forceinline void intersectNode(BVH8::NodeRef &cur,
                                       const Vec3f8 &rdir8,
                                       const Vec3f8 &org_rdir8,
                                       const float8 &tnear8,
                                       const float8 &tfar8,
                                       const size_t k)
      {

#if defined(__AVX2__)
        STAT3(normal.trav_nodes,1,1,1);
        const BVH8::Node* node = cur.node();

#if 0
        int8 index(k);
        const Vec3f8 rdir(permute(rdir8.x,index),permute(rdir8.y,index),permute(rdir8.z,index));
        const Vec3f8 org_rdir(permute(org_rdir8.x,index),permute(org_rdir8.y,index),permute(org_rdir8.z,index));
#else
        const Vec3f8 rdir(rdir8.x[k],rdir8.y[k],rdir8.z[k]);
        const Vec3f8 org_rdir(org_rdir8.x[k],org_rdir8.y[k],org_rdir8.z[k]);
#endif

        const float8 _tNearX = msub(node->lower_x, rdir.x, org_rdir.x);
        const float8 _tNearY = msub(node->lower_y, rdir.y, org_rdir.y);
        const float8 _tNearZ = msub(node->lower_z, rdir.z, org_rdir.z);
        const float8 _tFarX  = msub(node->upper_x, rdir.x, org_rdir.x);
        const float8 _tFarY  = msub(node->upper_y, rdir.y, org_rdir.y);
        const float8 _tFarZ  = msub(node->upper_z, rdir.z, org_rdir.z);

        const float8 inactive_inf = float8(pos_inf);
        const bool8  nactive      = inactive_inf == node->lower_x;

        const float8 tNearX  = mini(_tNearX,_tFarX);
        const float8 tNearY  = mini(_tNearY,_tFarY);
        const float8 tNearZ  = mini(_tNearZ,_tFarZ);

        const float8 tFarX   = maxi(_tNearX,_tFarX);
        const float8 tFarY   = maxi(_tNearY,_tFarY);
        const float8 tFarZ   = maxi(_tNearZ,_tFarZ);

        const float8 rayNear(tnear8[k]);
        const float8 rayFar(tfar8[k]);

        const float8 tNear   = maxi(maxi(tNearX,tNearY),maxi(tNearZ,rayNear));
        const float8 tFar    = mini(mini(tFarX ,tFarY ),mini(tFarZ ,rayFar ));
        const bool8 vmask    = nactive | cast(tNear) > cast(tFar);
        const size_t mask    = movemask(vmask)^0xff;
        dist        = tNear;
        hit_mask    = mask;
#endif
      }


      __forceinline void sortNodesDirect(BVH8::NodeRef &cur)
      {
        const BVH8::Node* node = cur.node();
        size_t mask = hit_mask;
        assert(current.isNode());

        if (unlikely(mask == 0)) return;

        /*! one child is hit, continue with that child */
        size_t r = __bscf(mask);
        if (likely(mask == 0)) {
          cur = node->child(r); 
          //cur.prefetch();
          assert(cur != BVH8::emptyNode);
          stackPtr->ptr = cur;
          stackPtr++;
          return;
        }
          
        /*! two children are hit, push far child, and continue with closer child */
        BVH8::NodeRef c0 = node->child(r); 
        //c0.prefetch(); 
        const unsigned int d0 = ((unsigned int*)&dist)[r];
        r = __bscf(mask);
        BVH8::NodeRef c1 = node->child(r); 
        //c1.prefetch(); 
        const unsigned int d1 = ((unsigned int*)&dist)[r];
        assert(c0 != BVH8::emptyNode);
        assert(c1 != BVH8::emptyNode);
        if (likely(mask == 0)) {
          if (d0 < d1) { stackPtr->ptr = c1; stackPtr->dist = d1; stackPtr++; stackPtr->ptr = c0; stackPtr->dist = d0; stackPtr++; return; }
          else         { stackPtr->ptr = c0; stackPtr->dist = d0; stackPtr++; stackPtr->ptr = c1; stackPtr->dist = d1; stackPtr++; return; }
        }
          
        /*! Here starts the slow path for 3 or 4 hit children. We push
         *  all nodes onto the stack to sort them there. */
        stackPtr->ptr = c0; stackPtr->dist = d0; stackPtr++;
        stackPtr->ptr = c1; stackPtr->dist = d1; stackPtr++;
        
        /*! three children are hit, push all onto stack and sort 3 stack items, continue with closest child */
        r = __bscf(mask);
          BVH8::NodeRef c = node->child(r); 
          //c.prefetch(); 
          unsigned int d = ((unsigned int*)&dist)[r]; stackPtr->ptr = c; stackPtr->dist = d; stackPtr++;
          assert(c != BVH8::emptyNode);
          if (likely(mask == 0)) {
            sort(stackPtr[-1],stackPtr[-2],stackPtr[-3]);
            return;
          }
          
	  /*! four children are hit, push all onto stack and sort 4 stack items, continue with closest child */
          r = __bscf(mask);
          c = node->child(r); 
          //c.prefetch(); 
          d = *(unsigned int*)&dist[r]; stackPtr->ptr = c; stackPtr->dist = d; stackPtr++;
	  if (likely(mask == 0)) {
	    sort(stackPtr[-1],stackPtr[-2],stackPtr[-3],stackPtr[-4]);
	    return;
	  }

	  /*! fallback case if more than 4 children are hit */
	  while (1)
	  {
	    r = __bscf(mask);
	    c = node->child(r); 
            //c.prefetch(); 
            d = *(unsigned int*)&dist[r]; stackPtr->ptr = c; stackPtr->dist = d; stackPtr++;
	    if (unlikely(mask == 0)) break;
	  }
      }

      __forceinline void sortNodes()
      {
        size_t mask = hit_mask;
          assert(current.isNode());
          const size_t hits = __popcnt(mask);
          const BVH8::Node* node = current.node();

	  while (1)
	  {
	    size_t r = __bscf(mask);
            BVH8::NodeRef c = node->child(r); 
            unsigned int d  = *(unsigned int*)&dist[r]; 
            stackPtr->ptr   = c; 
            stackPtr->dist  = d; 
            stackPtr++;
	    if (unlikely(mask == 0)) break;
	  }
          
          if (unlikely(hits >= 2))
          {
            if (likely(hits == 2))
              sort(stackPtr[-1],stackPtr[-2]);              
            else if (likely(hits == 3))
              sort(stackPtr[-1],stackPtr[-2],stackPtr[-3]);              
            else if (likely(hits == 4))
              sort(stackPtr[-1],stackPtr[-2],stackPtr[-3],stackPtr[-4]);              
          }                    
        
      }

      __forceinline void compactStack(const float max_dist)
      {
        StackItemT<BVH8::NodeRef>* new_stackPtr = stack+1;        //!< current stack pointer
        for (StackItemT<BVH8::NodeRef>*s = stack+1; s != stackPtr; s++)
          if (unlikely(*(float*)&s->dist < max_dist))
            *new_stackPtr++ = *s;
        stackPtr = new_stackPtr;        
      }


      /////////////////////////////////////////////////

      BVH8::NodeRef current;
      size_t hit_mask;
      StackItemT<BVH8::NodeRef>* stack;
      StackItemT<BVH8::NodeRef>* stackPtr;
      
      float8 dist;            
    };
    
    


    template<typename PrimitiveIntersector8>
    __forceinline bool BVH8Intersector8Test<PrimitiveIntersector8>::occluded1(const BVH8* bvh, NodeRef root, const size_t k, Precalculations& pre, Ray8& ray,const Vec3f8 &ray_org, const Vec3f8 &ray_dir, const Vec3f8 &ray_rdir, const float8 &ray_tnear, const float8 &ray_tfar, const Vec3i8& nearXYZ)
    {
#if defined (__AVX2__)

      /*! stack state */
      NodeRef stack[stackSizeSingle];  //!< stack of nodes that still need to get traversed
      NodeRef* stackPtr = stack+1;        //!< current stack pointer
      NodeRef* stackEnd = stack+stackSizeSingle;
      stack[0]  = root;
      
      /*! offsets to select the side that becomes the lower or upper bound */
      const size_t nearX = nearXYZ.x[k];
      const size_t nearY = nearXYZ.y[k];
      const size_t nearZ = nearXYZ.z[k];
      
      /*! load the ray into SIMD registers */
      const Vec3f8 org (ray_org .x[k],ray_org .y[k],ray_org .z[k]);
      const Vec3f8 rdir(ray_rdir.x[k],ray_rdir.y[k],ray_rdir.z[k]);
      const Vec3f8 norg = -org, org_rdir(org*rdir);
      const float8 rayNear(ray_tnear[k]), rayFar(ray_tfar[k]); 

      /* pop loop */
      while (true) pop:
      {
        /*! pop next node */
        if (unlikely(stackPtr == stack)) break;
        stackPtr--;
        NodeRef cur = (NodeRef) *stackPtr;
        
        /* downtraversal loop */
        while (true)
        {
          /*! stop if we found a leaf */
          if (unlikely(cur.isLeaf())) break;
          STAT3(shadow.trav_nodes,1,1,1);
          
          /*! single ray intersection with 4 boxes */
          const Node* node = cur.node();

#if OLD_TRAVERSAL == 1
          const size_t farX  = nearX ^ sizeof(float8), farY  = nearY ^ sizeof(float8), farZ  = nearZ ^ sizeof(float8);
          const float8 _tNearX = msub(load8f((const char*)node+nearX), rdir.x, org_rdir.x);
          const float8 _tNearY = msub(load8f((const char*)node+nearY), rdir.y, org_rdir.y);
          const float8 _tNearZ = msub(load8f((const char*)node+nearZ), rdir.z, org_rdir.z);
          const float8 _tFarX  = msub(load8f((const char*)node+farX ), rdir.x, org_rdir.x);
          const float8 _tFarY  = msub(load8f((const char*)node+farY ), rdir.y, org_rdir.y);
          const float8 _tFarZ  = msub(load8f((const char*)node+farZ ), rdir.z, org_rdir.z);

          const float8 tNearX  = _tNearX;
          const float8 tNearY  = _tNearY;
          const float8 tNearZ  = _tNearZ;

          const float8 tFarX   = _tFarX;
          const float8 tFarY   = _tFarY;
          const float8 tFarZ   = _tFarZ;

          const float8 tNear = maxi(maxi(tNearX,tNearY),maxi(tNearZ,rayNear));
          const float8 tFar  = mini(mini(tFarX ,tFarY ),mini(tFarZ ,rayFar ));
          const bool8 vmask = cast(tNear) > cast(tFar);
          size_t mask = movemask(vmask)^0xff;

#else

          const float8 _tNearX = msub(node->lower_x, rdir.x, org_rdir.x);
          const float8 _tNearY = msub(node->lower_y, rdir.y, org_rdir.y);
          const float8 _tNearZ = msub(node->lower_z, rdir.z, org_rdir.z);
          const float8 _tFarX  = msub(node->upper_x, rdir.x, org_rdir.x);
          const float8 _tFarY  = msub(node->upper_y, rdir.y, org_rdir.y);
          const float8 _tFarZ  = msub(node->upper_z, rdir.z, org_rdir.z);

          const bool8  nactive = float8(pos_inf) == node->lower_x;
          const float8 tNearX  = mini(_tNearX,_tFarX);
          const float8 tNearY  = mini(_tNearY,_tFarY);
          const float8 tNearZ  = mini(_tNearZ,_tFarZ);

          const float8 tFarX   = maxi(_tNearX,_tFarX);
          const float8 tFarY   = maxi(_tNearY,_tFarY);
          const float8 tFarZ   = maxi(_tNearZ,_tFarZ);

          const float8 tNear = maxi(maxi(tNearX,tNearY),maxi(tNearZ,rayNear));
          const float8 tFar  = mini(mini(tFarX ,tFarY ),mini(tFarZ ,rayFar ));
          const bool8 vmask  = nactive | cast(tNear) > cast(tFar);
          size_t mask = movemask(vmask)^0xff;

#endif
          
          /*! if no child is hit, pop next node */
          if (unlikely(mask == 0))
            goto pop;
          
          /*! one child is hit, continue with that child */
          size_t r = __bscf(mask);
          if (likely(mask == 0)) {
            cur = node->child(r); cur.prefetch(); 
            assert(cur != BVH8::emptyNode);
            continue;
          }
          
          /*! two children are hit, push far child, and continue with closer child */
          NodeRef c0 = node->child(r); c0.prefetch(); const unsigned int d0 = ((unsigned int*)&tNear)[r];
          r = __bscf(mask);
          NodeRef c1 = node->child(r); c1.prefetch(); const unsigned int d1 = ((unsigned int*)&tNear)[r];
          assert(c0 != BVH8::emptyNode);
          assert(c1 != BVH8::emptyNode);
          if (likely(mask == 0)) {
            assert(stackPtr < stackEnd);
            if (d0 < d1) { *stackPtr = c1; stackPtr++; cur = c0; continue; }
            else         { *stackPtr = c0; stackPtr++; cur = c1; continue; }
          }
          assert(stackPtr < stackEnd);
          *stackPtr = c0; stackPtr++;
          assert(stackPtr < stackEnd);
          *stackPtr = c1; stackPtr++;
          
	  /*! three children are hit */
          r = __bscf(mask);
          cur = node->child(r); cur.prefetch(); *stackPtr = cur; stackPtr++;
          if (likely(mask == 0)) {
            stackPtr--;
            continue;
          }

	  /*! process more than three children */
	  while(1)
	  {
	    r = __bscf(mask);
	    NodeRef c = node->child(r); c.prefetch(); *stackPtr = c; stackPtr++;
	    if (unlikely(mask == 0)) break;
	  }
	  cur = (NodeRef) stackPtr[-1]; stackPtr--;
        }
        
        /*! this is a leaf node */
	assert(cur != BVH8::emptyNode);
        STAT3(shadow.trav_leaves,1,1,1);
        size_t num; Triangle* prim = (Triangle*) cur.leaf(num);

        size_t lazy_node = 0;
        if (PrimitiveIntersector8::occluded(pre,ray,k,prim,num,bvh->scene,lazy_node)) {
          //ray.geomID = 0;
          //break;
	  return true;
        }

        if (unlikely(lazy_node)) {
          *stackPtr = lazy_node;
          stackPtr++;
        }
      }
#endif
      return false;
    }

     template<typename PrimitiveIntersector8>
    void BVH8Intersector8Test<PrimitiveIntersector8>::occluded(bool8* valid_i, BVH8* bvh, Ray8& ray)
    {
      /* load ray */
      bool8 valid = *valid_i;
#if defined(RTCORE_IGNORE_INVALID_RAYS)
      valid &= ray.valid();
#endif
      assert(all(valid,ray.tnear > -FLT_MIN));
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
      nearXYZ.x = select(rdir.x >= 0.0f,int8(0*(int)sizeof(float8)),int8(1*(int)sizeof(float8)));
      nearXYZ.y = select(rdir.y >= 0.0f,int8(2*(int)sizeof(float8)),int8(3*(int)sizeof(float8)));
      nearXYZ.z = select(rdir.z >= 0.0f,int8(4*(int)sizeof(float8)),int8(5*(int)sizeof(float8)));

      /* allocate stack and push root node */
      float8    stack_near[stackSizeChunk];
      NodeRef stack_node[stackSizeChunk];
      stack_node[0] = BVH8::invalidNode;
      stack_near[0] = inf;
      stack_node[1] = bvh->root;
      stack_near[1] = ray_tnear; 
      NodeRef* stackEnd = stack_node+stackSizeChunk;
      NodeRef* __restrict__ sptr_node = stack_node + 2;
      float8*    __restrict__ sptr_near = stack_near + 2;

#if !defined(__WIN32__) || defined(__X86_64__)
      const bool8 active = ray_tnear < ray_tfar;
      size_t bits = movemask(active);
      for (size_t i=__bsf(bits); bits!=0; bits=__btc(bits,i), i=__bsf(bits)) {
        if (occluded1(bvh,bvh->root,i,pre,ray,ray_org,ray_dir,rdir,ray_tnear,ray_tfar,nearXYZ))
          terminated[i] = -1;
      }
#endif
      store8i(valid & terminated,&ray.geomID,0);
      AVX_ZERO_UPPER();
    }

    DEFINE_INTERSECTOR8(BVH8Triangle4Intersector8TestMoeller,BVH8Intersector8Test<ArrayIntersector8_1<TriangleNIntersectorMMoellerTrumbore<Ray8 COMMA Triangle4 COMMA true> > >);
    DEFINE_INTERSECTOR8(BVH8Triangle4Intersector8TestMoellerNoFilter,BVH8Intersector8Test<ArrayIntersector8_1<TriangleNIntersectorMMoellerTrumbore<Ray8 COMMA Triangle4 COMMA false> > >);
    
    DEFINE_INTERSECTOR8(BVH8Triangle8Intersector8TestMoeller,BVH8Intersector8Test<ArrayIntersector8_1<TriangleNIntersectorMMoellerTrumbore<Ray8 COMMA Triangle8 COMMA true> > >);
    DEFINE_INTERSECTOR8(BVH8Triangle8Intersector8TestMoellerNoFilter,BVH8Intersector8Test<ArrayIntersector8_1<TriangleNIntersectorMMoellerTrumbore<Ray8 COMMA Triangle8 COMMA false> > >);

    DEFINE_INTERSECTOR8(BVH8Triangle8vIntersector8TestPluecker, BVH8Intersector8Test<ArrayIntersector8_1<TriangleNvIntersectorMPluecker2<Ray8 COMMA Triangle8v COMMA true> > >);
    DEFINE_INTERSECTOR8(BVH8Triangle8vIntersector8TestPlueckerNoFilter, BVH8Intersector8Test<ArrayIntersector8_1<TriangleNvIntersectorMPluecker2<Ray8 COMMA Triangle8v COMMA false> > >);

  }
}  

#endif
