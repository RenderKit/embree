// ======================================================================== //
// Copyright 2009-2017 Intel Corporation                                    //
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

#include "bvh_intersector_stream_filters.h"
#include "bvh_intersector_stream.h"

namespace embree
{
  namespace isa
  {
    static const size_t MAX_RAYS_PER_OCTANT = 8*sizeof(size_t);

    static_assert(MAX_RAYS_PER_OCTANT <= MAX_INTERNAL_STREAM_SIZE,"maximal internal stream size exceeded");

    __forceinline void RayStream::filterAOS(Scene *scene, RTCRay* _rayN, const size_t N, const size_t stride, IntersectContext* context, const bool intersect)
    {
      Ray* __restrict__ rayN = (Ray*)_rayN;

#if 1
      /* fallback to packets */
      const vintx ofs = vintx(step) * int(stride);

      for (size_t i = 0; i < N; i += VSIZEX)
      {
        const size_t n = min(N - i, size_t(VSIZEX));
        vboolx valid = vintx(step) < vintx(int(n));
        Ray* __restrict__ ray_i = (Ray*)((char*)rayN + stride * i);
        RayK<VSIZEX> ray;

#if defined(__AVX2__)
        ray.org.x  = vfloatx::gather<1>(valid, &ray_i->org.x, ofs);
        ray.org.y  = vfloatx::gather<1>(valid, &ray_i->org.y, ofs);
        ray.org.z  = vfloatx::gather<1>(valid, &ray_i->org.z, ofs);
        ray.dir.x  = vfloatx::gather<1>(valid, &ray_i->dir.x, ofs);
        ray.dir.y  = vfloatx::gather<1>(valid, &ray_i->dir.y, ofs);
        ray.dir.z  = vfloatx::gather<1>(valid, &ray_i->dir.z, ofs);
        ray.tnear  = vfloatx::gather<1>(valid, &ray_i->tnear, ofs);
        ray.tfar   = vfloatx::gather<1>(valid, &ray_i->tfar, ofs);
        ray.time   = vfloatx::gather<1>(valid, &ray_i->time, ofs);
        ray.mask   = vintx::gather<1>(valid, &ray_i->mask, ofs);
        ray.instID = vintx::gather<1>(valid, (int*)&ray_i->instID, ofs);
#else
        for (size_t k = 0; k < n; k++)
        {
          Ray* __restrict__ ray_k = (Ray*)((char*)ray_i + ofs[k]);

          ray.org.x[k] = ray_k->org.x;
          ray.org.y[k] = ray_k->org.y;
          ray.org.z[k] = ray_k->org.z;
          ray.dir.x[k] = ray_k->dir.x;
          ray.dir.y[k] = ray_k->dir.y;
          ray.dir.z[k] = ray_k->dir.z;
          ray.tnear[k] = ray_k->tnear;
          ray.tfar[k]  = ray_k->tfar;
          ray.time[k]  = ray_k->time;
          ray.mask[k]  = ray_k->mask;
          ray.instID[k] = ray_k->instID;
        }
#endif

        ray.geomID = RTC_INVALID_GEOMETRY_ID;

        /* intersect packet */
        if (intersect)
          scene->intersect(valid, ray, context);
        else
          scene->occluded (valid, ray, context);

        /* scatter hits */
#if defined(__AVX512F__)
        vintx::scatter<1>(valid, (int*)&ray_i->geomID, ofs, ray.geomID);
        if (intersect)
        {
          valid &= ray.geomID != RTC_INVALID_GEOMETRY_ID;

          vfloatx::scatter<1>(valid, &ray_i->tfar, ofs, ray.tfar);
          vfloatx::scatter<1>(valid, &ray_i->Ng.x, ofs, ray.Ng.x);
          vfloatx::scatter<1>(valid, &ray_i->Ng.y, ofs, ray.Ng.y);
          vfloatx::scatter<1>(valid, &ray_i->Ng.z, ofs, ray.Ng.z);
          vfloatx::scatter<1>(valid, &ray_i->u, ofs, ray.u);
          vfloatx::scatter<1>(valid, &ray_i->v, ofs, ray.v);
          vintx::scatter<1>(valid, (int*)&ray_i->primID, ofs, ray.primID);
          vintx::scatter<1>(valid, (int*)&ray_i->instID, ofs, ray.instID);
        }
#else
        for (size_t k = 0; k < n; k++)
        {
          Ray* __restrict__ ray_k = (Ray*)((char*)ray_i + ofs[k]);

          ray_k->geomID = ray.geomID[k];
          if (intersect && ray.geomID[k] != RTC_INVALID_GEOMETRY_ID)
          {
            ray_k->tfar   = ray.tfar[k];
            ray_k->Ng.x   = ray.Ng.x[k];
            ray_k->Ng.y   = ray.Ng.y[k];
            ray_k->Ng.z   = ray.Ng.z[k];
            ray_k->u      = ray.u[k];
            ray_k->v      = ray.v[k];
            ray_k->primID = ray.primID[k];
            ray_k->instID = ray.instID[k];
          }
        }
#endif
      }

#elif 1

      /* fallback to packets - much more complex, but not faster, so should be removed... */
      const vintx ofs = vintx(step) * int(stride);

      size_t i = 0;

      /* full packet loop */
      const size_t Nfull = N & (-size_t(VSIZEX));
      for (; i < Nfull; i += VSIZEX)
      {
        Ray* __restrict__ ray_i = (Ray*)((char*)rayN + stride * i);
        RayK<VSIZEX> ray;

        /* gather: instID */
        ray.instID = vintx::gather<1>((int*)&ray_i->instID, ofs);

#if defined(__AVX512F__)
        /* load and transpose: org.x, org.y, org.z, align0, dir.x, dir.y, dir.z, align1 */
        const vfloat8 ab0  = vfloat8::load(&((Ray*)((char*)ray_i + ofs[ 0]))->org);
        const vfloat8 ab1  = vfloat8::load(&((Ray*)((char*)ray_i + ofs[ 1]))->org);
        const vfloat8 ab2  = vfloat8::load(&((Ray*)((char*)ray_i + ofs[ 2]))->org);
        const vfloat8 ab3  = vfloat8::load(&((Ray*)((char*)ray_i + ofs[ 3]))->org);
        const vfloat8 ab4  = vfloat8::load(&((Ray*)((char*)ray_i + ofs[ 4]))->org);
        const vfloat8 ab5  = vfloat8::load(&((Ray*)((char*)ray_i + ofs[ 5]))->org);
        const vfloat8 ab6  = vfloat8::load(&((Ray*)((char*)ray_i + ofs[ 6]))->org);
        const vfloat8 ab7  = vfloat8::load(&((Ray*)((char*)ray_i + ofs[ 7]))->org);
        const vfloat8 ab8  = vfloat8::load(&((Ray*)((char*)ray_i + ofs[ 8]))->org);
        const vfloat8 ab9  = vfloat8::load(&((Ray*)((char*)ray_i + ofs[ 9]))->org);
        const vfloat8 ab10 = vfloat8::load(&((Ray*)((char*)ray_i + ofs[10]))->org);
        const vfloat8 ab11 = vfloat8::load(&((Ray*)((char*)ray_i + ofs[11]))->org);
        const vfloat8 ab12 = vfloat8::load(&((Ray*)((char*)ray_i + ofs[12]))->org);
        const vfloat8 ab13 = vfloat8::load(&((Ray*)((char*)ray_i + ofs[13]))->org);
        const vfloat8 ab14 = vfloat8::load(&((Ray*)((char*)ray_i + ofs[14]))->org);
        const vfloat8 ab15 = vfloat8::load(&((Ray*)((char*)ray_i + ofs[15]))->org);

        vfloat16 unused0, unused1;
        transpose(ab0,ab1,ab2,ab3,ab4,ab5,ab6,ab7,ab8,ab9,ab10,ab11,ab12,ab13,ab14,ab15,
                  ray.org.x, ray.org.y, ray.org.z, unused0, ray.dir.x, ray.dir.y, ray.dir.z, unused1);

        /* load and transpose: tnear, tfar, time, mask */
        const vfloat4 c0  = vfloat4::load(&((Ray*)((char*)ray_i + ofs[ 0]))->tnear);
        const vfloat4 c1  = vfloat4::load(&((Ray*)((char*)ray_i + ofs[ 1]))->tnear);
        const vfloat4 c2  = vfloat4::load(&((Ray*)((char*)ray_i + ofs[ 2]))->tnear);
        const vfloat4 c3  = vfloat4::load(&((Ray*)((char*)ray_i + ofs[ 3]))->tnear);
        const vfloat4 c4  = vfloat4::load(&((Ray*)((char*)ray_i + ofs[ 4]))->tnear);
        const vfloat4 c5  = vfloat4::load(&((Ray*)((char*)ray_i + ofs[ 5]))->tnear);
        const vfloat4 c6  = vfloat4::load(&((Ray*)((char*)ray_i + ofs[ 6]))->tnear);
        const vfloat4 c7  = vfloat4::load(&((Ray*)((char*)ray_i + ofs[ 7]))->tnear);
        const vfloat4 c8  = vfloat4::load(&((Ray*)((char*)ray_i + ofs[ 8]))->tnear);
        const vfloat4 c9  = vfloat4::load(&((Ray*)((char*)ray_i + ofs[ 9]))->tnear);
        const vfloat4 c10 = vfloat4::load(&((Ray*)((char*)ray_i + ofs[10]))->tnear);
        const vfloat4 c11 = vfloat4::load(&((Ray*)((char*)ray_i + ofs[11]))->tnear);
        const vfloat4 c12 = vfloat4::load(&((Ray*)((char*)ray_i + ofs[12]))->tnear);
        const vfloat4 c13 = vfloat4::load(&((Ray*)((char*)ray_i + ofs[13]))->tnear);
        const vfloat4 c14 = vfloat4::load(&((Ray*)((char*)ray_i + ofs[14]))->tnear);
        const vfloat4 c15 = vfloat4::load(&((Ray*)((char*)ray_i + ofs[15]))->tnear);

        vfloat16 maskf;
        transpose(c0,c1,c2,c3,c4,c5,c6,c7,c8,c9,c10,c11,c12,c13,c14,c15,
                  ray.tnear, ray.tfar, ray.time, maskf);
        ray.mask = asInt(maskf);
#elif defined(__AVX__)
        /* load and transpose: org.x, org.y, org.z, align0, dir.x, dir.y, dir.z, align1 */
        const vfloat8 ab0 = vfloat8::load(&((Ray*)((char*)ray_i + ofs[0]))->org);
        const vfloat8 ab1 = vfloat8::load(&((Ray*)((char*)ray_i + ofs[1]))->org);
        const vfloat8 ab2 = vfloat8::load(&((Ray*)((char*)ray_i + ofs[2]))->org);
        const vfloat8 ab3 = vfloat8::load(&((Ray*)((char*)ray_i + ofs[3]))->org);
        const vfloat8 ab4 = vfloat8::load(&((Ray*)((char*)ray_i + ofs[4]))->org);
        const vfloat8 ab5 = vfloat8::load(&((Ray*)((char*)ray_i + ofs[5]))->org);
        const vfloat8 ab6 = vfloat8::load(&((Ray*)((char*)ray_i + ofs[6]))->org);
        const vfloat8 ab7 = vfloat8::load(&((Ray*)((char*)ray_i + ofs[7]))->org);

        vfloat8 unused0, unused1;
        transpose(ab0,ab1,ab2,ab3,ab4,ab5,ab6,ab7, ray.org.x, ray.org.y, ray.org.z, unused0, ray.dir.x, ray.dir.y, ray.dir.z, unused1);

        /* load and transpose: tnear, tfar, time, mask */
        const vfloat4 c0 = vfloat4::load(&((Ray*)((char*)ray_i + ofs[0]))->tnear);
        const vfloat4 c1 = vfloat4::load(&((Ray*)((char*)ray_i + ofs[1]))->tnear);
        const vfloat4 c2 = vfloat4::load(&((Ray*)((char*)ray_i + ofs[2]))->tnear);
        const vfloat4 c3 = vfloat4::load(&((Ray*)((char*)ray_i + ofs[3]))->tnear);
        const vfloat4 c4 = vfloat4::load(&((Ray*)((char*)ray_i + ofs[4]))->tnear);
        const vfloat4 c5 = vfloat4::load(&((Ray*)((char*)ray_i + ofs[5]))->tnear);
        const vfloat4 c6 = vfloat4::load(&((Ray*)((char*)ray_i + ofs[6]))->tnear);
        const vfloat4 c7 = vfloat4::load(&((Ray*)((char*)ray_i + ofs[7]))->tnear);

        vfloat8 maskf;
        transpose(c0,c1,c2,c3,c4,c5,c6,c7, ray.tnear, ray.tfar, ray.time, maskf);
        ray.mask = asInt(maskf);
#else
        /* load and transpose: org.x, org.y, org.z */
        const vfloat4 a0 = vfloat4::load(&((Ray*)((char*)ray_i + ofs[0]))->org);
        const vfloat4 a1 = vfloat4::load(&((Ray*)((char*)ray_i + ofs[1]))->org);
        const vfloat4 a2 = vfloat4::load(&((Ray*)((char*)ray_i + ofs[2]))->org);
        const vfloat4 a3 = vfloat4::load(&((Ray*)((char*)ray_i + ofs[3]))->org);

        transpose(a0,a1,a2,a3, ray.org.x, ray.org.y, ray.org.z);

        /* load and transpose: dir.x, dir.y, dir.z */
        const vfloat4 b0 = vfloat4::load(&((Ray*)((char*)ray_i + ofs[0]))->dir);
        const vfloat4 b1 = vfloat4::load(&((Ray*)((char*)ray_i + ofs[1]))->dir);
        const vfloat4 b2 = vfloat4::load(&((Ray*)((char*)ray_i + ofs[2]))->dir);
        const vfloat4 b3 = vfloat4::load(&((Ray*)((char*)ray_i + ofs[3]))->dir);

        transpose(b0,b1,b2,b3, ray.dir.x, ray.dir.y, ray.dir.z);

        /* load and transpose: tnear, tfar, time, mask */
        const vfloat4 c0 = vfloat4::load(&((Ray*)((char*)ray_i + ofs[0]))->tnear);
        const vfloat4 c1 = vfloat4::load(&((Ray*)((char*)ray_i + ofs[1]))->tnear);
        const vfloat4 c2 = vfloat4::load(&((Ray*)((char*)ray_i + ofs[2]))->tnear);
        const vfloat4 c3 = vfloat4::load(&((Ray*)((char*)ray_i + ofs[3]))->tnear);

        vfloat4 maskf;
        transpose(c0,c1,c2,c3, ray.tnear, ray.tfar, ray.time, maskf);
        ray.mask = asInt(maskf);
#endif

        /* init: geomID */
        ray.geomID = RTC_INVALID_GEOMETRY_ID;

        /* intersect packet */
        if (intersect)
          scene->intersect(True, ray, context);
        else
          scene->occluded (True, ray, context);

        /* scatter hits */
#if defined(__AVX512F__)
        vintx::scatter<1>((int*)&ray_i->geomID, ofs, ray.geomID);
        if (intersect)
        {
          const vboolx valid = ray.geomID != RTC_INVALID_GEOMETRY_ID;
          vfloatx::scatter<1>(valid, &ray_i->tfar, ofs, ray.tfar);
          vfloatx::scatter<1>(valid, &ray_i->Ng.x, ofs, ray.Ng.x);
          vfloatx::scatter<1>(valid, &ray_i->Ng.y, ofs, ray.Ng.y);
          vfloatx::scatter<1>(valid, &ray_i->Ng.z, ofs, ray.Ng.z);
          vfloatx::scatter<1>(valid, &ray_i->u, ofs, ray.u);
          vfloatx::scatter<1>(valid, &ray_i->v, ofs, ray.v);
          vintx::scatter<1>(valid, (int*)&ray_i->primID, ofs, ray.primID);
          vintx::scatter<1>(valid, (int*)&ray_i->instID, ofs, ray.instID);
        }
#else
        for (size_t k = 0; k < VSIZEX; k++)
        {
          Ray* __restrict__ ray_k = (Ray*)((char*)ray_i + ofs[k]);

          ray_k->geomID = ray.geomID[k];
          if (intersect && ray.geomID[k] != RTC_INVALID_GEOMETRY_ID)
          {
            ray_k->tfar   = ray.tfar[k];
            ray_k->Ng.x   = ray.Ng.x[k];
            ray_k->Ng.y   = ray.Ng.y[k];
            ray_k->Ng.z   = ray.Ng.z[k];
            ray_k->u      = ray.u[k];
            ray_k->v      = ray.v[k];
            ray_k->primID = ray.primID[k];
            ray_k->instID = ray.instID[k];
          }
        }
#endif
      }

      /* tail packet */
      if (i < N)
      {
        const size_t Ntail = N - i;
        vboolx valid = vintx(step) < vintx(int(Ntail));

        Ray* __restrict__ ray_i = (Ray*)((char*)rayN + stride * i);
        RayK<VSIZEX> ray;

        /* gather rays */
#if defined(__AVX2__)
        ray.org.x  = vfloatx::gather<1>(valid, &ray_i->org.x, ofs);
        ray.org.y  = vfloatx::gather<1>(valid, &ray_i->org.y, ofs);
        ray.org.z  = vfloatx::gather<1>(valid, &ray_i->org.z, ofs);
        ray.dir.x  = vfloatx::gather<1>(valid, &ray_i->dir.x, ofs);
        ray.dir.y  = vfloatx::gather<1>(valid, &ray_i->dir.y, ofs);
        ray.dir.z  = vfloatx::gather<1>(valid, &ray_i->dir.z, ofs);
        ray.tnear  = vfloatx::gather<1>(valid, &ray_i->tnear, ofs);
        ray.tfar   = vfloatx::gather<1>(valid, &ray_i->tfar, ofs);
        ray.time   = vfloatx::gather<1>(valid, &ray_i->time, ofs);
        ray.mask   = vintx::gather<1>(valid, &ray_i->mask, ofs);
        ray.instID = vintx::gather<1>(valid, (int*)&ray_i->instID, ofs);
#else
        for (size_t k = 0; k < Ntail; k++)
        {
          Ray* __restrict__ ray_k = (Ray*)((char*)ray_i + ofs[k]);

          ray.org.x[k]  = ray_k->org.x;
          ray.org.y[k]  = ray_k->org.y;
          ray.org.z[k]  = ray_k->org.z;
          ray.dir.x[k]  = ray_k->dir.x;
          ray.dir.y[k]  = ray_k->dir.y;
          ray.dir.z[k]  = ray_k->dir.z;
          ray.tnear[k]  = ray_k->tnear;
          ray.tfar[k]   = ray_k->tfar;
          ray.time[k]   = ray_k->time;
          ray.mask[k]   = ray_k->mask;
          ray.instID[k] = ray_k->instID;
        }
#endif

        ray.geomID = RTC_INVALID_GEOMETRY_ID;

        /* instersect packet */
        if (intersect)
          scene->intersect(valid, ray, context);
        else
          scene->occluded (valid, ray, context);

        /* scatter hits */
#if defined(__AVX512F__)
        vintx::scatter<1>(valid, (int*)&ray_i->geomID, ofs, ray.geomID);
        if (intersect)
        {
          valid &= ray.geomID != RTC_INVALID_GEOMETRY_ID;
          vfloatx::scatter<1>(valid, &ray_i->tfar, ofs, ray.tfar);
          vfloatx::scatter<1>(valid, &ray_i->Ng.x, ofs, ray.Ng.x);
          vfloatx::scatter<1>(valid, &ray_i->Ng.y, ofs, ray.Ng.y);
          vfloatx::scatter<1>(valid, &ray_i->Ng.z, ofs, ray.Ng.z);
          vfloatx::scatter<1>(valid, &ray_i->u, ofs, ray.u);
          vfloatx::scatter<1>(valid, &ray_i->v, ofs, ray.v);
          vintx::scatter<1>(valid, (int*)&ray_i->primID, ofs, ray.primID);
          vintx::scatter<1>(valid, (int*)&ray_i->instID, ofs, ray.instID);
        }
#else
        for (size_t k = 0; k < Ntail; k++)
        {
          Ray* __restrict__ ray_k = (Ray*)((char*)ray_i + ofs[k]);

          ray_k->geomID = ray.geomID[k];
          if (intersect && ray.geomID[k] != RTC_INVALID_GEOMETRY_ID)
          {
            ray_k->tfar   = ray.tfar[k];
            ray_k->Ng.x   = ray.Ng.x[k];
            ray_k->Ng.y   = ray.Ng.y[k];
            ray_k->Ng.z   = ray.Ng.z[k];
            ray_k->u      = ray.u[k];
            ray_k->v      = ray.v[k];
            ray_k->primID = ray.primID[k];
            ray_k->instID = ray.instID[k];
          }
        }
#endif
      }

#else

      __aligned(64) Ray* octants[8][MAX_RAYS_PER_OCTANT];
      unsigned int rays_in_octant[8];

      for (size_t i=0;i<8;i++) rays_in_octant[i] = 0;
      size_t inputray_iD = 0;

      while(1)
      {
        int cur_octant = -1;
        /* sort rays into octants */
        for (;inputray_iD<N;)
        {
          Ray &ray = *(Ray*)((char*)rayN + inputray_iD * stride);
          /* skip invalid rays */
          if (unlikely(ray.tnear > ray.tfar)) { inputray_iD++; continue; }
          if (unlikely(!intersect && ray.geomID == 0)) { inputray_iD++; continue; } // ignore already occluded rays

#if defined(EMBREE_IGNORE_INVALID_RAYS)
          if (unlikely(!ray.valid())) {  inputray_iD++; continue; }
#endif

          const unsigned int octantID = movemask(vfloat4(ray.dir) < 0.0f) & 0x7;

          assert(octantID < 8);
          octants[octantID][rays_in_octant[octantID]++] = &ray;
          inputray_iD++;
          if (unlikely(rays_in_octant[octantID] == MAX_RAYS_PER_OCTANT))
          {
            cur_octant = octantID;
            break;
          }
        }
        /* need to flush rays in octant ? */
        if (unlikely(cur_octant == -1))
          for (int i=0;i<8;i++)
            if (rays_in_octant[i])
            {
              cur_octant = i;
              break;
            }

        /* all rays traced ? */
        if (unlikely(cur_octant == -1))
          break;

        
        Ray** rays = &octants[cur_octant][0];
        const size_t numOctantRays = rays_in_octant[cur_octant];

        /* special codepath for very small number of rays per octant */
        if (numOctantRays == 1)
        {
          if (intersect) scene->intersect((RTCRay&)*rays[0],context);
          else           scene->occluded ((RTCRay&)*rays[0],context);
        }        
        /* codepath for large number of rays per octant */
        else
        {
          /* incoherent ray stream code path */
          if (intersect) scene->intersectN((RTCRay**)rays,numOctantRays,context);
          else           scene->occludedN ((RTCRay**)rays,numOctantRays,context);
        }
        rays_in_octant[cur_octant] = 0;
      }
#endif
    }

    __forceinline void RayStream::filterAOP(Scene *scene, RTCRay** _rayN, const size_t N,IntersectContext* context, const bool intersect)
    {
      Ray** __restrict__ rayN = (Ray**)_rayN;
      __aligned(64) Ray* octants[8][MAX_RAYS_PER_OCTANT];
      unsigned int rays_in_octant[8];

      for (size_t i=0;i<8;i++) rays_in_octant[i] = 0;
      size_t inputray_iD = 0;

      while(1)
      {
        int cur_octant = -1;
        /* sort rays into octants */
        for (;inputray_iD<N;)
        {
          Ray &ray = *rayN[inputray_iD];
          /* skip invalid rays */
          if (unlikely(ray.tnear > ray.tfar)) { inputray_iD++; continue; }
          if (unlikely(!intersect && ray.geomID == 0)) { inputray_iD++; continue; } // ignore already occluded rays

#if defined(EMBREE_IGNORE_INVALID_RAYS)
          if (unlikely(!ray.valid())) {  inputray_iD++; continue; }
#endif

          const unsigned int octantID = movemask(vfloat4(ray.dir) < 0.0f) & 0x7;

          assert(octantID < 8);
          octants[octantID][rays_in_octant[octantID]++] = &ray;
          inputray_iD++;
          if (unlikely(rays_in_octant[octantID] == MAX_RAYS_PER_OCTANT))
          {
            cur_octant = octantID;
            break;
          }
        }
        /* need to flush rays in octant ? */
        if (unlikely(cur_octant == -1))
          for (int i=0;i<8;i++)
            if (rays_in_octant[i])
            {
              cur_octant = i;
              break;
            }

        /* all rays traced ? */
        if (unlikely(cur_octant == -1))
          break;

        
        Ray** rays = &octants[cur_octant][0];
        const size_t numOctantRays = rays_in_octant[cur_octant];

        /* special codepath for very small number of rays per octant */
        if (numOctantRays == 1)
        {
          if (intersect) scene->intersect((RTCRay&)*rays[0],context);
          else           scene->occluded ((RTCRay&)*rays[0],context);
        }        
        /* codepath for large number of rays per octant */
        else
        {
          /* incoherent ray stream code path */
          if (intersect) scene->intersectN((RTCRay**)rays,numOctantRays,context);
          else           scene->occludedN ((RTCRay**)rays,numOctantRays,context);
        }
        rays_in_octant[cur_octant] = 0;
      }
    }

    void RayStream::filterSOACoherent(Scene *scene, char* rayData, const size_t streams, const size_t stream_offset, IntersectContext* context, const bool intersect)
    {
      /* all valid accels need to have a intersectN/occludedN */
      bool chunkFallback = scene->isRobust() || !scene->accels.validIsecN();

      /* check for common octant */
      if (unlikely(!chunkFallback))
      {
        vfloatx min_x(pos_inf), max_x(neg_inf);
        vfloatx min_y(pos_inf), max_y(neg_inf);
        vfloatx min_z(pos_inf), max_z(neg_inf);
        vboolx all_active(true);
        for (size_t s=0; s<streams; s++)
        {
          const size_t offset = s*stream_offset;
          RayK<VSIZEX> &ray = *(RayK<VSIZEX>*)(rayData + offset);
          min_x = min(min_x,ray.dir.x);
          min_y = min(min_y,ray.dir.y);
          min_z = min(min_z,ray.dir.z);
          max_x = max(max_x,ray.dir.x);
          max_y = max(max_y,ray.dir.y);
          max_z = max(max_z,ray.dir.z);
          all_active &= ray.tnear <= ray.tfar;
#if defined(EMBREE_IGNORE_INVALID_RAYS)
          all_active &= ray.valid();
#endif
        }
        const bool commonOctant =
          (all(max_x < vfloatx(zero)) || all(min_x >= vfloatx(zero))) &&
          (all(max_y < vfloatx(zero)) || all(min_y >= vfloatx(zero))) &&
          (all(max_z < vfloatx(zero)) || all(min_z >= vfloatx(zero)));

        /* fallback to chunk in case of non-common octants */
        chunkFallback |= !commonOctant || !all(all_active);
      }

      /* fallback to chunk if necessary */
      if (unlikely(chunkFallback))
      {
        for (size_t s=0; s<streams; s++)
        {
          const size_t offset = s*stream_offset;
          RayK<VSIZEX> &ray = *(RayK<VSIZEX>*)(rayData + offset);
          vboolx valid = ray.tnear <= ray.tfar;
          if (intersect) scene->intersect(valid,ray,context);
          else           scene->occluded (valid,ray,context);
        }
        return;
      }

      static const size_t MAX_COHERENT_RAY_PACKETS = MAX_RAYS_PER_OCTANT / VSIZEX;
      __aligned(64) RayK<VSIZEX> *rays_ptr[MAX_RAYS_PER_OCTANT / VSIZEX];

      /* set input layout to SOA */
      context->setInputSOA(VSIZEX);
      size_t numStreams = 0;

      for (size_t s=0; s<streams; s++)
      {
        const size_t offset = s*stream_offset;
        RayK<VSIZEX> &ray = *(RayK<VSIZEX>*)(rayData + offset);
        rays_ptr[numStreams++] = &ray;
        /* trace as stream */
        if (unlikely(numStreams == MAX_COHERENT_RAY_PACKETS))
        {
          const size_t size = numStreams*VSIZEX;
          if (intersect)
            scene->intersectN((RTCRay**)rays_ptr,size,context);
          else
            scene->occludedN((RTCRay**)rays_ptr,size,context);
          numStreams = 0;
        }
      }
      /* flush remaining streams */
      if (unlikely(numStreams))
      {
        const size_t size = numStreams*VSIZEX;
        if (intersect)
          scene->intersectN((RTCRay**)rays_ptr,size,context);
        else
          scene->occludedN((RTCRay**)rays_ptr,size,context);
      }
    }

    __forceinline void RayStream::filterSOA(Scene *scene, char* rayData, const size_t N, const size_t streams, const size_t stream_offset, IntersectContext* context, const bool intersect)
    {
      /* can we use the fast path ? */
#if defined(__AVX__) && ENABLE_COHERENT_STREAM_PATH == 1 
      /* fast path for packet width == SIMD width && correct RayK alignment*/
      const size_t rayDataAlignment = (size_t)rayData        % (VSIZEX*sizeof(float));
      const size_t offsetAlignment  = (size_t)stream_offset  % (VSIZEX*sizeof(float));

      if (unlikely(isCoherent(context->user->flags) &&
                   N == VSIZEX                && 
                   !rayDataAlignment          && 
                   !offsetAlignment))
      {
        filterSOACoherent(scene, rayData, streams, stream_offset, context, intersect);
        return;
      }
#endif

      /* otherwise use stream intersector */
      RayPacket rayN(rayData,N);

      __aligned(64) Ray rays[MAX_RAYS_PER_OCTANT];
      __aligned(64) Ray *rays_ptr[MAX_RAYS_PER_OCTANT];
      
      size_t octants[8][MAX_RAYS_PER_OCTANT];
      unsigned int rays_in_octant[8];

      for (size_t i=0;i<8;i++) rays_in_octant[i] = 0;

      size_t soffset = 0;

      for (size_t s=0;s<streams;s++,soffset+=stream_offset)
      {
        // todo: use SIMD width to compute octants
        for (size_t i=0;i<N;i++)
        {
          /* global + local offset */
          const size_t offset = soffset + sizeof(float) * i;

          if (unlikely(!rayN.isValidByOffset(offset))) continue;

#if defined(EMBREE_IGNORE_INVALID_RAYS)
          __aligned(64) Ray ray = rayN.gatherByOffset(offset);
          if (unlikely(!ray.valid())) continue; 
#endif

          const size_t octantID = rayN.getOctantByOffset(offset);

          assert(octantID < 8);
          octants[octantID][rays_in_octant[octantID]++] = offset;
        
          if (unlikely(rays_in_octant[octantID] == MAX_RAYS_PER_OCTANT))
          {
            for (size_t j=0;j<MAX_RAYS_PER_OCTANT;j++)
            {
              rays_ptr[j] = &rays[j]; // rays_ptr might get reordered for occludedN
              rays[j] = rayN.gatherByOffset(octants[octantID][j]);
            }

            if (intersect)
              scene->intersectN((RTCRay**)rays_ptr,MAX_RAYS_PER_OCTANT,context);
            else
              scene->occludedN((RTCRay**)rays_ptr,MAX_RAYS_PER_OCTANT,context);

            for (size_t j=0;j<MAX_RAYS_PER_OCTANT;j++)
              rayN.scatterByOffset(octants[octantID][j],rays[j],intersect);
            
            rays_in_octant[octantID] = 0;
          }
        }        
      }

      /* flush remaining rays per octant */
      for (size_t i=0;i<8;i++)
        if (rays_in_octant[i])
        {
          for (size_t j=0;j<rays_in_octant[i];j++)
          {
            rays_ptr[j] = &rays[j]; // rays_ptr might get reordered for occludedN
            rays[j] = rayN.gatherByOffset(octants[i][j]);
          }

          if (intersect)
            scene->intersectN((RTCRay**)rays_ptr,rays_in_octant[i],context);
          else
            scene->occludedN((RTCRay**)rays_ptr,rays_in_octant[i],context);        

          for (size_t j=0;j<rays_in_octant[i];j++)
            rayN.scatterByOffset(octants[i][j],rays[j],intersect);
        }
    }

    void RayStream::filterSOPCoherent(Scene *scene, const RTCRayNp& _rayN, const size_t N, IntersectContext* context, const bool intersect)
    {
      RayPN& rayN = *(RayPN*)&_rayN;

      /* all valid accels need to have a intersectN/occludedN */
      bool chunkFallback = scene->isRobust() || !scene->accels.validIsecN();

      /* check for common octant */
      if (unlikely(!chunkFallback))
      {
        vfloatx min_x(pos_inf), max_x(neg_inf);
        vfloatx min_y(pos_inf), max_y(neg_inf);
        vfloatx min_z(pos_inf), max_z(neg_inf);
        vboolx all_active(true);
        for (size_t i = 0; i < N; i += VSIZEX)
        {
          const vintx vi = vintx(int(i)) + vintx(step);
          const vboolx valid = vi < vintx(int(N));
          const size_t offset = sizeof(float) * i;

          const Vec3vfx dir = rayN.gatherDirByOffset(valid, offset);

          min_x = min(min_x, dir.x);
          min_y = min(min_y, dir.y);
          min_z = min(min_z, dir.z);
          max_x = max(max_x, dir.x);
          max_y = max(max_y, dir.y);
          max_z = max(max_z, dir.z);

          vboolx active = rayN.isValidByOffset(valid, offset);
#if defined(EMBREE_IGNORE_INVALID_RAYS)
          __aligned(64) Ray ray = rayN.gatherByOffset(offset);
          active &= ray.valid();
#endif
          all_active = select(valid, all_active & active, all_active);
        }
        const bool commonOctant =
          (all(max_x < vfloatx(zero)) || all(min_x >= vfloatx(zero))) &&
          (all(max_y < vfloatx(zero)) || all(min_y >= vfloatx(zero))) &&
          (all(max_z < vfloatx(zero)) || all(min_z >= vfloatx(zero)));

        /* fallback to chunk in case of non-common octants */
        chunkFallback |= !commonOctant || !all(all_active);
      }

      /* fallback to chunk if necessary */
      if (unlikely(chunkFallback))
      {
        for (size_t i = 0; i < N; i += VSIZEX)
        {
          const vintx vi = vintx(int(i)) + vintx(step);
          vboolx valid = vi < vintx(int(N));
          const size_t offset = sizeof(float) * i;

          RayK<VSIZEX> ray = rayN.gatherByOffset<VSIZEX>(valid, offset);
          valid &= ray.tnear <= ray.tfar;
          if (intersect)
            scene->intersect(valid, ray, context);
          else
            scene->occluded (valid, ray, context);
          rayN.scatterByOffset<VSIZEX>(valid, offset, ray, intersect);
        }
        return;
      }

      static const size_t MAX_COHERENT_RAY_PACKETS = MAX_RAYS_PER_OCTANT / VSIZEX;
      __aligned(64) RayK<VSIZEX> rays[MAX_COHERENT_RAY_PACKETS];
      __aligned(64) RayK<VSIZEX>* rays_ptr[MAX_COHERENT_RAY_PACKETS];

      /* set input layout to SOA */
      context->setInputSOA(VSIZEX);

      for (size_t i = 0; i < N; i += MAX_COHERENT_RAY_PACKETS * VSIZEX)
      {
        const size_t size = min(N-i, MAX_COHERENT_RAY_PACKETS * VSIZEX);

        /* convert from SOP to SOA */
        for (size_t j = 0; j < size; j += VSIZEX)
        {
          const vintx vi = vintx(int(i+j)) + vintx(step);
          const vboolx valid = vi < vintx(int(N));
          const size_t offset = sizeof(float) * (i+j);
          const size_t packetID = j / VSIZEX;

          rays[packetID] = rayN.gatherByOffset(valid, offset);
          rays_ptr[packetID] = &rays[packetID]; // rays_ptr might get reordered for occludedN
        }

        /* trace as stream */
        if (intersect)
          scene->intersectN((RTCRay**)rays_ptr, size, context);
        else
          scene->occludedN((RTCRay**)rays_ptr, size, context);

        /* convert from SOA to SOP */
        for (size_t j = 0; j < size; j += VSIZEX)
        {
          const vintx vi = vintx(int(i+j)) + vintx(step);
          const vboolx valid = vi < vintx(int(N));
          const size_t offset = sizeof(float) * (i+j);
          const size_t packetID = j / VSIZEX;

          rayN.scatterByOffset(valid, offset, rays[packetID], intersect);
        }
      }
    }

    void RayStream::filterSOP(Scene *scene, const RTCRayNp& _rayN, const size_t N, IntersectContext* context, const bool intersect)
    {
      /* use fast path for coherent ray mode */
#if defined(__AVX__) && ENABLE_COHERENT_STREAM_PATH == 1
      if (unlikely(isCoherent(context->user->flags)))
      {
        filterSOPCoherent(scene, _rayN, N, context, intersect);
        return;
      }
#endif
      
      /* otherwise use stream intersector */
      RayPN& rayN = *(RayPN*)&_rayN;

#if 1

      /* fallback to packets */
      for (size_t i = 0; i < N; i += VSIZEX)
      {
        const vintx vi = vintx(int(i)) + vintx(step);
        vboolx valid = vi < vintx(int(N));
        const size_t offset = sizeof(float) * i;

        RayK<VSIZEX> ray = rayN.gatherByOffset<VSIZEX>(valid, offset);
        valid &= ray.tnear <= ray.tfar;
        if (intersect)
          scene->intersect(valid, ray, context);
        else
          scene->occluded (valid, ray, context);
        rayN.scatterByOffset<VSIZEX>(valid, offset, ray, intersect);
      }

#else

      size_t rayStartIndex = 0;

      __aligned(64) Ray rays[MAX_RAYS_PER_OCTANT];
      __aligned(64) Ray *rays_ptr[MAX_RAYS_PER_OCTANT];

      size_t octants[8][MAX_RAYS_PER_OCTANT];
      unsigned int rays_in_octant[8];

      for (size_t i=0;i<8;i++) rays_in_octant[i] = 0;

      {
        // todo: use SIMD width to compute octants
        for (size_t i=rayStartIndex;i<N;i++)
        {
          /* global + local offset */
          const size_t offset = sizeof(float) * i;

          if (unlikely(!rayN.isValidByOffset(offset))) continue;

#if defined(EMBREE_IGNORE_INVALID_RAYS)
          __aligned(64) Ray ray = rayN.gatherByOffset(offset);
          if (unlikely(!ray.valid())) continue; 
#endif

          const size_t octantID = rayN.getOctantByOffset(offset);

          assert(octantID < 8);
          octants[octantID][rays_in_octant[octantID]++] = offset;
        
          if (unlikely(rays_in_octant[octantID] == MAX_RAYS_PER_OCTANT))
          {
            for (size_t j=0;j<MAX_RAYS_PER_OCTANT;j++)
            {
              rays_ptr[j] = &rays[j]; // rays_ptr might get reordered for occludedN
              rays[j] = rayN.gatherByOffset(octants[octantID][j]);
            }

            if (intersect)
              scene->intersectN((RTCRay**)rays_ptr,MAX_RAYS_PER_OCTANT,context);
            else
              scene->occludedN((RTCRay**)rays_ptr,MAX_RAYS_PER_OCTANT,context);

            for (size_t j=0;j<MAX_RAYS_PER_OCTANT;j++)
              rayN.scatterByOffset(octants[octantID][j],rays[j],intersect);
            
            rays_in_octant[octantID] = 0;
          }
        }        
      }

      /* flush remaining rays per octant */
      for (size_t i=0;i<8;i++)
        if (rays_in_octant[i])
        {
          for (size_t j=0;j<rays_in_octant[i];j++)
          {
            rays_ptr[j] = &rays[j]; // rays_ptr might get reordered for occludedN
            rays[j] = rayN.gatherByOffset(octants[i][j]);
          }

          if (intersect)
            scene->intersectN((RTCRay**)rays_ptr,rays_in_octant[i],context);
          else
            scene->occludedN((RTCRay**)rays_ptr,rays_in_octant[i],context);        

          for (size_t j=0;j<rays_in_octant[i];j++)
            rayN.scatterByOffset(octants[i][j],rays[j],intersect);
        }

#endif

    }

    RayStreamFilterFuncs rayStreamFilterFuncs() {
      return RayStreamFilterFuncs(RayStream::filterAOS,RayStream::filterAOP,RayStream::filterSOA,RayStream::filterSOP);
    }
  };
};
