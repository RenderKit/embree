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
    MAYBE_UNUSED static const size_t MAX_PACKET_STREAM_SIZE = MAX_INTERNAL_STREAM_SIZE / VSIZEX;

    __forceinline void RayStreamFilter::filterAOS(Scene* scene, RTCRay* _rayN, size_t N, size_t stride, IntersectContext* context, bool intersect)
    {
      RayStreamAOS rayN(_rayN);

      /* use fast path for coherent ray mode */
      if (unlikely(isCoherent(context->user->flags)))
      {
        __aligned(64) RayK<VSIZEX> rays[MAX_PACKET_STREAM_SIZE];
        __aligned(64) RayK<VSIZEX>* rayPtrs[MAX_PACKET_STREAM_SIZE];

        for (size_t i = 0; i < N; i += MAX_INTERNAL_STREAM_SIZE)
        {
          const size_t size = min(N - i, MAX_INTERNAL_STREAM_SIZE);

          /* convert from AOS to SOA */
          for (size_t j = 0; j < size; j += VSIZEX)
          {
            const vintx vij = vintx(int(i+j)) + vintx(step);
            const vboolx valid = vij < vintx(int(N));
            const vintx offset = vij * int(stride);
            const size_t packetIndex = j / VSIZEX;

            RayK<VSIZEX> ray = rayN.getRayByOffset(valid, offset);
            ray.tnear() = select(valid, ray.tnear(), zero);
            ray.tfar()  = select(valid, ray.tfar(),  neg_inf);

            rays[packetIndex] = ray;
            rayPtrs[packetIndex] = &rays[packetIndex]; // rayPtrs might get reordered for occludedN
          }

          /* trace stream */
          if (intersect)
            scene->intersectors.intersectN(rayPtrs, size, context);
          else
            scene->intersectors.occludedN(rayPtrs, size, context);

          /* convert from SOA to AOS */
          for (size_t j = 0; j < size; j += VSIZEX)
          {
            const vintx vij = vintx(int(i+j)) + vintx(step);
            const vboolx valid = vij < vintx(int(N));
            const vintx offset = vij * int(stride);
            const size_t packetIndex = j / VSIZEX;
            rayN.setHitByOffset(valid, offset, rays[packetIndex], intersect);
          }
        }
      }
      else if (unlikely(!intersect)) /* octant sorting for occlusion rays */
      {
#define MAX_RAYS_PER_OCTANT 32
        /* reintroduced SOA path for occlusion rays */
        __aligned(64) unsigned int octants[8][MAX_RAYS_PER_OCTANT];
        __aligned(64) RayK<VSIZEX> rays[MAX_PACKET_STREAM_SIZE];
        __aligned(64) RayK<VSIZEX>* rayPtrs[MAX_PACKET_STREAM_SIZE];

        unsigned int rays_in_octant[8];        
        for (size_t i=0;i<8;i++) rays_in_octant[i] = 0;
        size_t inputRayID = 0;

        while(1)
        {
          int cur_octant = -1;
          /* sort rays into octants */
          for (;inputRayID<N;)
          {
            Ray &ray = *(Ray*)((char*)_rayN + inputRayID * stride);
            /* skip invalid rays */
            if (unlikely(ray.tnear() > ray.tfar() || ray.geomID == 0)) { inputRayID++; continue; } // ignore invalid or already occluded rays
#if defined(EMBREE_IGNORE_INVALID_RAYS)
            if (unlikely(!ray.valid())) {  inputRayID++; continue; }
#endif

            const unsigned int octantID = movemask(vfloat4(Vec3fa(ray.dir)) < 0.0f) & 0x7;

            assert(octantID < 8);
            octants[octantID][rays_in_octant[octantID]++] = (unsigned int)inputRayID;
            inputRayID++;
            if (unlikely(rays_in_octant[octantID] == MAX_RAYS_PER_OCTANT))
            {
              cur_octant = octantID;
              break;
            }
          }
          /* need to flush rays in octant ? */
          if (unlikely(cur_octant == -1))
            for (unsigned int i=0;i<8;i++)
              if (rays_in_octant[i])
              {
                cur_octant = i;
                break;
              }

          /* all rays traced ? */
          if (unlikely(cur_octant == -1))
            break;
        
          unsigned int* const rayIDs = &octants[cur_octant][0];
          const unsigned int numOctantRays = rays_in_octant[cur_octant];
          assert(numOctantRays);

          /* could get optmized because we know all packets except the last is fully populated */
          for (unsigned int j = 0; j < numOctantRays; j+= VSIZEX)
          {
            const vintx vi = vintx(int(j)) + vintx(step);
            const vboolx valid = vi < vintx(int(numOctantRays));
            const vintx IDs = select(valid,*(vintx*)&rayIDs[j],vintx(zero));
            const vintx offset = IDs * int(stride);
            RayK<VSIZEX> &ray = rays[j/VSIZEX];
            rayPtrs[j/VSIZEX] = &ray;
            ray = rayN.getRayByOffset(vboolx(true), offset);
            ray.tnear() = select(valid, ray.tnear(), zero);
            ray.tfar()  = select(valid, ray.tfar(),  neg_inf);            
          }

#if 0     
          /* for testing */
          for (unsigned int j = 0; j < numOctantRays; j+= VSIZEX)
          {
            const vintx IDs = *(vintx*)&rayIDs[j];
            const vintx vi = vintx(int(j)) + vintx(step);
            const vboolx valid = vi < vintx(int(numOctantRays));
            scene->intersectors.occluded(valid, rays[j/VSIZEX], context);
          }
#else
          scene->intersectors.occludedN(rayPtrs, numOctantRays, context);          
#endif

          for (unsigned int j = 0; j < numOctantRays; j+= VSIZEX)
          {
            const vintx vi = vintx(int(j)) + vintx(step);
            const vboolx valid = vi < vintx(int(numOctantRays));
            const vintx IDs = select(valid,*(vintx*)&rayIDs[j],vintx(zero));
            const vintx offset = IDs * int(stride);
            rayN.setHitByOffset(valid, offset, rays[j/VSIZEX], intersect);
          }

          /* special codepath for very small number of rays per octant */
          // if (numOctantRays == 1)
          // {
          //   else           scene->occluded ((RTCRay&)*rays[0],context);
          // }        
          /* codepath for large number of rays per octant */
          /* incoherent ray stream code path */
          
          rays_in_octant[cur_octant] = 0;
        }

      }
      else
      {
        /* fallback to packets */
        for (size_t i=0; i<N; i+=VSIZEX)
        {
          const vintx vi = vintx(int(i)) + vintx(step);
          vboolx valid = vi < vintx(int(N));
          const vintx offset = vi * int(stride);

          RayK<VSIZEX> ray = rayN.getRayByOffset(valid, offset);
          valid &= ray.tnear() <= ray.tfar();

          if (intersect)
            scene->intersectors.intersect(valid, ray, context);
          else
            scene->intersectors.occluded(valid, ray, context);

          rayN.setHitByOffset(valid, offset, ray, intersect);
        }
      }
    }

    __forceinline void RayStreamFilter::filterAOP(Scene* scene, RTCRay** _rayN, size_t N, IntersectContext* context, bool intersect)
    {
      RayStreamAOP rayN(_rayN);

      /* use fast path for coherent ray mode */
      if (unlikely(isCoherent(context->user->flags)))
      {
        __aligned(64) RayK<VSIZEX> rays[MAX_PACKET_STREAM_SIZE];
        __aligned(64) RayK<VSIZEX>* rayPtrs[MAX_PACKET_STREAM_SIZE];

        for (size_t i = 0; i < N; i += MAX_INTERNAL_STREAM_SIZE)
        {
          const size_t size = min(N - i, MAX_INTERNAL_STREAM_SIZE);

          /* convert from AOP to SOA */
          for (size_t j = 0; j < size; j += VSIZEX)
          {
            const vintx vij = vintx(int(i+j)) + vintx(step);
            const vboolx valid = vij < vintx(int(N));
            const size_t packetIndex = j / VSIZEX;

            RayK<VSIZEX> ray = rayN.getRayByIndex(valid, i+j);
            ray.tnear() = select(valid, ray.tnear(), zero);
            ray.tfar()  = select(valid, ray.tfar(),  neg_inf);

            rays[packetIndex] = ray;
            rayPtrs[packetIndex] = &rays[packetIndex]; // rayPtrs might get reordered for occludedN
          }

          /* trace stream */
          if (intersect)
            scene->intersectors.intersectN(rayPtrs, size, context);
          else
            scene->intersectors.occludedN(rayPtrs, size, context);

          /* convert from SOA to AOP */
          for (size_t j = 0; j < size; j += VSIZEX)
          {
            const vintx vij = vintx(int(i+j)) + vintx(step);
            const vboolx valid = vij < vintx(int(N));
            const size_t packetIndex = j / VSIZEX;

            rayN.setHitByIndex(valid, i+j, rays[packetIndex], intersect);
          }
        }
      }
      else
      {
        /* fallback to packets */
        for (size_t i = 0; i < N; i += VSIZEX)
        {
          const vintx vi = vintx(int(i)) + vintx(step);
          vboolx valid = vi < vintx(int(N));

          RayK<VSIZEX> ray = rayN.getRayByIndex(valid, i);
          valid &= ray.tnear() <= ray.tfar();

          if (intersect)
            scene->intersectors.intersect(valid, ray, context);
          else
            scene->intersectors.occluded(valid, ray, context);

          rayN.setHitByIndex(valid, i, ray, intersect);
        }
      }
    }

    __forceinline void RayStreamFilter::filterSOA(Scene* scene, char* rayData, size_t N, size_t numPackets, size_t stride, IntersectContext* context, bool intersect)
    {
      const size_t rayDataAlignment = (size_t)rayData % (VSIZEX*sizeof(float));
      const size_t offsetAlignment  = (size_t)stride  % (VSIZEX*sizeof(float));

      /* fast path for packets with the correct width and data alignment */
      if (likely(N == VSIZEX &&
                 !rayDataAlignment &&
                 !offsetAlignment))
      {
        if (unlikely(isCoherent(context->user->flags)))
        {
          __aligned(64) RayK<VSIZEX>* rayPtrs[MAX_INTERNAL_STREAM_SIZE / VSIZEX];

          size_t packetIndex = 0;
          for (size_t i = 0; i < numPackets; i++)
          {
            const size_t offset = i * stride;
            RayK<VSIZEX>& ray = *(RayK<VSIZEX>*)(rayData + offset);
            rayPtrs[packetIndex++] = &ray;

            /* trace as stream */
            if (unlikely(packetIndex == MAX_PACKET_STREAM_SIZE))
            {
              const size_t size = packetIndex*VSIZEX;
              if (intersect)
                scene->intersectors.intersectN(rayPtrs, size, context);
              else
                scene->intersectors.occludedN(rayPtrs, size, context);
              packetIndex = 0;
            }
          }

          /* flush remaining packets */
          if (unlikely(packetIndex > 0))
          {
            const size_t size = packetIndex*VSIZEX;
            if (intersect)
              scene->intersectors.intersectN(rayPtrs, size, context);
            else
              scene->intersectors.occludedN(rayPtrs, size, context);
          }
        }
        else
        {
          /* fallback to packets */
          for (size_t i = 0; i < numPackets; i++)
          {
            const size_t offset = i * stride;
            RayK<VSIZEX>& ray = *(RayK<VSIZEX>*)(rayData + offset);
            const vboolx valid = ray.tnear() <= ray.tfar();

            if (intersect)
              scene->intersectors.intersect(valid, ray, context);
            else
              scene->intersectors.occluded(valid, ray, context);
          }
        }
      }
      else
      {
        /* fallback to packets for arbitrary packet size and alignment */
        for (size_t i = 0; i < numPackets; i++)
        {
          const size_t offsetN = i * stride;
          RayPacketSOA rayN(rayData + offsetN, N);

          for (size_t j = 0; j < N; j += VSIZEX)
          {
            const size_t offset = j * sizeof(float);
            vboolx valid = (vintx(int(j)) + vintx(step)) < vintx(int(N));

            RayK<VSIZEX> ray = rayN.getRayByOffset(valid, offset);
            valid &= ray.tnear() <= ray.tfar();

            if (intersect)
              scene->intersectors.intersect(valid, ray, context);
            else
              scene->intersectors.occluded(valid, ray, context);

            rayN.setHitByOffset(valid, offset, ray, intersect);
          }
        }
      }
    }

    void RayStreamFilter::filterSOP(Scene* scene, const RTCRayNp& _rayN, size_t N, IntersectContext* context, bool intersect)
    { 
      RayStreamSOP& rayN = *(RayStreamSOP*)&_rayN;

      /* use fast path for coherent ray mode */
      if (unlikely(isCoherent(context->user->flags)))
      {
        __aligned(64) RayK<VSIZEX> rays[MAX_PACKET_STREAM_SIZE];
        __aligned(64) RayK<VSIZEX>* rayPtrs[MAX_PACKET_STREAM_SIZE];

        for (size_t i = 0; i < N; i += MAX_INTERNAL_STREAM_SIZE)
        {
          const size_t size = min(N - i, MAX_INTERNAL_STREAM_SIZE);

          /* convert from SOP to SOA */
          for (size_t j = 0; j < size; j += VSIZEX)
          {
            const vintx vij = vintx(int(i+j)) + vintx(step);
            const vboolx valid = vij < vintx(int(N));
            const size_t offset = (i+j) * sizeof(float);
            const size_t packetIndex = j / VSIZEX;

            RayK<VSIZEX> ray = rayN.getRayByOffset(valid, offset);
            ray.tnear() = select(valid, ray.tnear(), zero);
            ray.tfar()  = select(valid, ray.tfar(),  neg_inf);

            rays[packetIndex] = ray;
            rayPtrs[packetIndex] = &rays[packetIndex]; // rayPtrs might get reordered for occludedN
          }

          /* trace stream */
          if (intersect)
            scene->intersectors.intersectN(rayPtrs, size, context);
          else
            scene->intersectors.occludedN(rayPtrs, size, context);

          /* convert from SOA to SOP */
          for (size_t j = 0; j < size; j += VSIZEX)
          {
            const vintx vij = vintx(int(i+j)) + vintx(step);
            const vboolx valid = vij < vintx(int(N));
            const size_t offset = (i+j) * sizeof(float);
            const size_t packetIndex = j / VSIZEX;

            rayN.setHitByOffset(valid, offset, rays[packetIndex], intersect);
          }
        }
      }
      else
      {
        /* fallback to packets */
        for (size_t i = 0; i < N; i += VSIZEX)
        {
          const vintx vi = vintx(int(i)) + vintx(step);
          vboolx valid = vi < vintx(int(N));
          const size_t offset = i * sizeof(float);

          RayK<VSIZEX> ray = rayN.getRayByOffset(valid, offset);
          valid &= ray.tnear() <= ray.tfar();

          if (intersect)
            scene->intersectors.intersect(valid, ray, context);
          else
            scene->intersectors.occluded(valid, ray, context);

          rayN.setHitByOffset(valid, offset, ray, intersect);
        }
      }
    }

    RayStreamFilterFuncs rayStreamFilterFuncs() {
      return RayStreamFilterFuncs(RayStreamFilter::filterAOS, RayStreamFilter::filterAOP, RayStreamFilter::filterSOA, RayStreamFilter::filterSOP);
    }
  };
};
