// ======================================================================== //
// Copyright 2009-2016 Intel Corporation                                    //
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

#include "raystreams.h"
#include "../../common/scene.h"

namespace embree
{
  static const size_t MAX_RAYS_PER_OCTANT = 8*sizeof(size_t);

  namespace isa
  {


    void RayStream::filterAOS(Scene *scene, RTCRay* _rayN, const size_t N, const size_t stride, const size_t flags, const bool intersect)
    {
      /* fast path for very small ray packets */
      if (likely(N == 1)) {
        if (intersect) scene->intersect(*_rayN);
        else           scene->occluded (*_rayN);
        return;
      }
#if 0
      Ray* __restrict__ rayN = (Ray*)_rayN;
      __aligned(64) Ray* octants[MAX_RAYS_PER_OCTANT];
      unsigned int rays_in_octant = 0;
      
      size_t inputRayID = 0;
      while(1)
      {
        /* sort rays into octants */
        for (;inputRayID<N;)
        {
          Ray &ray = *(Ray*)((char*)rayN + inputRayID * stride);
          /* skip invalid rays */
          if (unlikely(ray.tnear > ray.tfar)) { inputRayID++; continue; }
          octants[rays_in_octant++] = &ray;
          inputRayID++;
          if (unlikely(rays_in_octant == MAX_RAYS_PER_OCTANT))
            break;
        }
        if (rays_in_octant == 0) 
          break;
        
        if (intersect)
          scene->intersectN((RTCRay**)&octants[0],rays_in_octant,flags);
        else
          scene->occludedN((RTCRay**)&octants[0],rays_in_octant,flags);

        rays_in_octant = 0;
      }

#else
      Ray* __restrict__ rayN = (Ray*)_rayN;
      __aligned(64) Ray* octants[8][MAX_RAYS_PER_OCTANT];
      unsigned int rays_in_octant[8];

      for (size_t i=0;i<8;i++) rays_in_octant[i] = 0;
      size_t inputRayID = 0;
      while(1)
      {
        int cur_octant = -1;
        /* sort rays into octants */
        for (;inputRayID<N;)
        {
          Ray &ray = *(Ray*)((char*)rayN + inputRayID * stride);
          /* skip invalid rays */
          if (unlikely(ray.tnear > ray.tfar)) { inputRayID++; continue; }
#if defined(__MIC__)
          const unsigned int octantID = \
            (ray.dir.x < 0.0f ? 1 : 0) |
            (ray.dir.y < 0.0f ? 2 : 0) |
            (ray.dir.z < 0.0f ? 4 : 0);
#else
          const unsigned int octantID = movemask(vfloat4(ray.dir) < 0.0f) & 0x7;
#endif
          assert(octantID < 8);
          octants[octantID][rays_in_octant[octantID]++] = &ray;
          inputRayID++;
          if (unlikely(rays_in_octant[octantID] == MAX_RAYS_PER_OCTANT))
          {
            cur_octant = octantID;
            break;
          }
        }
        /* need to flush rays in octant ? */
        if (unlikely(cur_octant == -1))
          for (size_t i=0;i<8;i++)
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

        if (intersect)
          scene->intersectN((RTCRay**)rays,numOctantRays,flags);
        else
          scene->occludedN((RTCRay**)rays,numOctantRays,flags);

        rays_in_octant[cur_octant] = 0;

      }
#endif
    }


    void RayStream::filterSOA(Scene *scene, RTCRaySOA& _rayN, const size_t N, const size_t streams, const size_t stream_offset, const size_t flags, const bool intersect)
    {
      __aligned(64) Ray rays[MAX_RAYS_PER_OCTANT];
      __aligned(64) Ray *rays_ptr[MAX_RAYS_PER_OCTANT];

      for (size_t i=0;i<MAX_RAYS_PER_OCTANT;i++)
        rays_ptr[i] = &rays[i];

      size_t octants[8][MAX_RAYS_PER_OCTANT];
      unsigned int rays_in_octant[8];

      for (size_t i=0;i<8;i++) rays_in_octant[i] = 0;

      size_t soffset = 0;
      RaySOA &rayN = *(RaySOA*)&_rayN;

      for (size_t s=0;s<streams;s++,soffset+=stream_offset)
      {
        // todo: use SIMD width to compute octants
        for (size_t i=0;i<N;i++)
        {
          /* global + local offset */
          const size_t offset = soffset + sizeof(float) * i;

          if (unlikely(!rayN.isValidByOffset(offset))) continue;

          const size_t octantID = rayN.getOctantByOffset(offset);

          assert(octantID < 8);
          octants[octantID][rays_in_octant[octantID]++] = offset;
        
          if (unlikely(rays_in_octant[octantID] == MAX_RAYS_PER_OCTANT))
          {
            for (size_t j=0;j<MAX_RAYS_PER_OCTANT;j++)
            {
              rays[j] = rayN.gatherByOffset(octants[octantID][j]);
              assert(rays[j].valid());
            }

            if (intersect)
              scene->intersectN((RTCRay**)rays_ptr,MAX_RAYS_PER_OCTANT,flags);
            else
              scene->occludedN((RTCRay**)rays_ptr,MAX_RAYS_PER_OCTANT,flags);

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
            rays[j] = rayN.gatherByOffset(octants[i][j]);
            //if (!rays[j].valid()) PRINT(rays[j]);
            assert(rays[j].valid());
          }

          if (intersect)
            scene->intersectN((RTCRay**)rays_ptr,rays_in_octant[i],flags);
          else
            scene->occludedN((RTCRay**)rays_ptr,rays_in_octant[i],flags);        

          for (size_t j=0;j<rays_in_octant[i];j++)
            rayN.scatterByOffset(octants[i][j],rays[j],intersect);

        }
    }

    RayStreamFilterFuncs rayStreamFilters(RayStream::filterAOS,RayStream::filterSOA);

  };
};
