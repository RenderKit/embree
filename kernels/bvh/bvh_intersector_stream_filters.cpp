// ======================================================================== //
// Copyright 2009-2018 Intel Corporation                                    //
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
    MAYBE_UNUSED static const size_t MAX_INTERNAL_PACKET_STREAM_SIZE = MAX_INTERNAL_STREAM_SIZE / VSIZEX;

    template<bool intersect>
    __forceinline void RayStreamFilter::filterAOS(Scene* scene, void* _rayN, size_t N, size_t stride, IntersectContext* context)
    {
      RayStreamAOS rayN(_rayN);

      /* use fast path for coherent ray mode */
      if (unlikely(isCoherent(context->user->flags)))
      {
        __aligned(64) RayTypeK<VSIZEX, intersect> rays[MAX_INTERNAL_PACKET_STREAM_SIZE];
        __aligned(64) RayTypeK<VSIZEX, intersect>* rayPtrs[MAX_INTERNAL_PACKET_STREAM_SIZE];

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

            RayTypeK<VSIZEX, intersect> ray = rayN.getRayByOffset(valid, offset);
            ray.tnear() = select(valid, ray.tnear(), zero);
            ray.tfar()  = select(valid, ray.tfar(),  neg_inf);

            rays[packetIndex] = ray;
            rayPtrs[packetIndex] = &rays[packetIndex]; // rayPtrs might get reordered for occludedN
          }

          /* trace stream */
          scene->intersectors.intersectN(rayPtrs, size, context);

          /* convert from SOA to AOS */
          for (size_t j = 0; j < size; j += VSIZEX)
          {
            const vintx vij = vintx(int(i+j)) + vintx(step);
            const vboolx valid = vij < vintx(int(N));
            const vintx offset = vij * int(stride);
            const size_t packetIndex = j / VSIZEX;
            rayN.setHitByOffset(valid, offset, rays[packetIndex]);
          }
        }
      }
      else if (unlikely(!intersect))
      {
        /* octant sorting for occlusion rays */
        __aligned(64) unsigned int octants[8][MAX_INTERNAL_STREAM_SIZE];
        __aligned(64) RayK<VSIZEX> rays[MAX_INTERNAL_PACKET_STREAM_SIZE];
        __aligned(64) RayK<VSIZEX>* rayPtrs[MAX_INTERNAL_PACKET_STREAM_SIZE];

        unsigned int raysInOctant[8];
        for (unsigned int i = 0; i < 8; i++)
          raysInOctant[i] = 0;
        size_t inputRayID = 0;

        for (;;)
        {
          int curOctant = -1;

          /* sort rays into octants */
          for (; inputRayID < N;)
          {
            const Ray& ray = rayN.getRayByOffset(inputRayID * stride);

            /* skip invalid rays */
            if (unlikely(ray.tnear() > ray.tfar() || ray.tfar() < 0.0f)) { inputRayID++; continue; } // ignore invalid or already occluded rays
#if defined(EMBREE_IGNORE_INVALID_RAYS)
            if (unlikely(!ray.valid())) { inputRayID++; continue; }
#endif

            const unsigned int octantID = movemask(vfloat4(Vec3fa(ray.dir)) < 0.0f) & 0x7;

            assert(octantID < 8);
            octants[octantID][raysInOctant[octantID]++] = (unsigned int)inputRayID;
            inputRayID++;
            if (unlikely(raysInOctant[octantID] == MAX_INTERNAL_STREAM_SIZE))
            {
              curOctant = octantID;
              break;
            }
          }

          /* need to flush rays in octant? */
          if (unlikely(curOctant == -1))
          {
            for (unsigned int i = 0; i < 8; i++)
              if (raysInOctant[i]) { curOctant = i; break; }
          }

          /* all rays traced? */
          if (unlikely(curOctant == -1))
            break;
        
          unsigned int* const rayIDs = &octants[curOctant][0];
          const unsigned int numOctantRays = raysInOctant[curOctant];
          assert(numOctantRays);

          for (unsigned int j = 0; j < numOctantRays; j += VSIZEX)
          {
            const vintx vi = vintx(int(j)) + vintx(step);
            const vboolx valid = vi < vintx(int(numOctantRays));
            const vintx offset = *(vintx*)&rayIDs[j] * int(stride);
            RayK<VSIZEX>& ray = rays[j/VSIZEX];
            rayPtrs[j/VSIZEX] = &ray;
            ray = rayN.getRayByOffset(valid, offset);
            ray.tnear() = select(valid, ray.tnear(), zero);
            ray.tfar()  = select(valid, ray.tfar(),  neg_inf);
          }

          scene->intersectors.occludedN(rayPtrs, numOctantRays, context);

          for (unsigned int j = 0; j < numOctantRays; j += VSIZEX)
          {
            const vintx vi = vintx(int(j)) + vintx(step);
            const vboolx valid = vi < vintx(int(numOctantRays));
            const vintx offset = *(vintx*)&rayIDs[j] * int(stride);
            rayN.setHitByOffset(valid, offset, rays[j/VSIZEX]);
          }

          raysInOctant[curOctant] = 0;
        }
      }
      else
      {
        /* fallback to packets */
        for (size_t i = 0; i < N; i += VSIZEX)
        {
          const vintx vi = vintx(int(i)) + vintx(step);
          vboolx valid = vi < vintx(int(N));
          const vintx offset = vi * int(stride);

          RayTypeK<VSIZEX, intersect> ray = rayN.getRayByOffset(valid, offset);
          valid &= ray.tnear() <= ray.tfar();

          scene->intersectors.intersect(valid, ray, context);

          rayN.setHitByOffset(valid, offset, ray);
        }
      }
    }

    template<bool intersect>
    __forceinline void RayStreamFilter::filterAOP(Scene* scene, void** _rayN, size_t N, IntersectContext* context)
    {
      RayStreamAOP rayN(_rayN);

      /* use fast path for coherent ray mode */
      if (unlikely(isCoherent(context->user->flags)))
      {
        __aligned(64) RayTypeK<VSIZEX, intersect> rays[MAX_INTERNAL_PACKET_STREAM_SIZE];
        __aligned(64) RayTypeK<VSIZEX, intersect>* rayPtrs[MAX_INTERNAL_PACKET_STREAM_SIZE];

        for (size_t i = 0; i < N; i += MAX_INTERNAL_STREAM_SIZE)
        {
          const size_t size = min(N - i, MAX_INTERNAL_STREAM_SIZE);

          /* convert from AOP to SOA */
          for (size_t j = 0; j < size; j += VSIZEX)
          {
            const vintx vij = vintx(int(i+j)) + vintx(step);
            const vboolx valid = vij < vintx(int(N));
            const size_t packetIndex = j / VSIZEX;

            RayTypeK<VSIZEX, intersect> ray = rayN.getRayByIndex(valid, vij);
            ray.tnear() = select(valid, ray.tnear(), zero);
            ray.tfar()  = select(valid, ray.tfar(),  neg_inf);

            rays[packetIndex] = ray;
            rayPtrs[packetIndex] = &rays[packetIndex]; // rayPtrs might get reordered for occludedN
          }

          /* trace stream */
          scene->intersectors.intersectN(rayPtrs, size, context);

          /* convert from SOA to AOP */
          for (size_t j = 0; j < size; j += VSIZEX)
          {
            const vintx vij = vintx(int(i+j)) + vintx(step);
            const vboolx valid = vij < vintx(int(N));
            const size_t packetIndex = j / VSIZEX;

            rayN.setHitByIndex(valid, vij, rays[packetIndex]);
          }
        }
      }
      else if (unlikely(!intersect))
      {
        /* octant sorting for occlusion rays */
        __aligned(64) unsigned int octants[8][MAX_INTERNAL_STREAM_SIZE];
        __aligned(64) RayK<VSIZEX> rays[MAX_INTERNAL_PACKET_STREAM_SIZE];
        __aligned(64) RayK<VSIZEX>* rayPtrs[MAX_INTERNAL_PACKET_STREAM_SIZE];

        unsigned int raysInOctant[8];
        for (unsigned int i = 0; i < 8; i++)
          raysInOctant[i] = 0;
        size_t inputRayID = 0;

        for (;;)
        {
          int curOctant = -1;

          /* sort rays into octants */
          for (; inputRayID < N;)
          {
            const Ray& ray = rayN.getRayByIndex(inputRayID);

            /* skip invalid rays */
            if (unlikely(ray.tnear() > ray.tfar() || ray.tfar() < 0.0f)) { inputRayID++; continue; } // ignore invalid or already occluded rays
#if defined(EMBREE_IGNORE_INVALID_RAYS)
            if (unlikely(!ray.valid())) { inputRayID++; continue; }
#endif

            const unsigned int octantID = movemask(vfloat4(ray.dir) < 0.0f) & 0x7;

            assert(octantID < 8);
            octants[octantID][raysInOctant[octantID]++] = (unsigned int)inputRayID;
            inputRayID++;
            if (unlikely(raysInOctant[octantID] == MAX_INTERNAL_STREAM_SIZE))
            {
              curOctant = octantID;
              break;
            }
          }

          /* need to flush rays in octant? */
          if (unlikely(curOctant == -1))
          {
            for (unsigned int i = 0; i < 8; i++)
              if (raysInOctant[i]) { curOctant = i; break; }
          }

          /* all rays traced? */
          if (unlikely(curOctant == -1))
            break;

          unsigned int* const rayIDs = &octants[curOctant][0];
          const unsigned int numOctantRays = raysInOctant[curOctant];
          assert(numOctantRays);

          for (unsigned int j = 0; j < numOctantRays; j += VSIZEX)
          {
            const vintx vi = vintx(int(j)) + vintx(step);
            const vboolx valid = vi < vintx(int(numOctantRays));
            const vintx index = *(vintx*)&rayIDs[j];
            RayK<VSIZEX>& ray = rays[j/VSIZEX];
            rayPtrs[j/VSIZEX] = &ray;
            ray = rayN.getRayByIndex(valid, index);
            ray.tnear() = select(valid, ray.tnear(), zero);
            ray.tfar()  = select(valid, ray.tfar(),  neg_inf);
          }

          scene->intersectors.occludedN(rayPtrs, numOctantRays, context);

          for (unsigned int j = 0; j < numOctantRays; j += VSIZEX)
          {
            const vintx vi = vintx(int(j)) + vintx(step);
            const vboolx valid = vi < vintx(int(numOctantRays));
            const vintx index = *(vintx*)&rayIDs[j];
            rayN.setHitByIndex(valid, index, rays[j/VSIZEX]);
          }

          raysInOctant[curOctant] = 0;
        }
      }
      else
      {
        /* fallback to packets */
        for (size_t i = 0; i < N; i += VSIZEX)
        {
          const vintx vi = vintx(int(i)) + vintx(step);
          vboolx valid = vi < vintx(int(N));

          RayTypeK<VSIZEX, intersect> ray = rayN.getRayByIndex(valid, vi);
          valid &= ray.tnear() <= ray.tfar();

          scene->intersectors.intersect(valid, ray, context);

          rayN.setHitByIndex(valid, vi, ray);
        }
      }
    }

    template<bool intersect>
    __forceinline void RayStreamFilter::filterSOA(Scene* scene, char* rayData, size_t N, size_t numPackets, size_t stride, IntersectContext* context)
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
          __aligned(64) RayTypeK<VSIZEX, intersect>* rayPtrs[MAX_INTERNAL_STREAM_SIZE / VSIZEX];

          size_t packetIndex = 0;
          for (size_t i = 0; i < numPackets; i++)
          {
            const size_t offset = i * stride;
            RayTypeK<VSIZEX, intersect>& ray = *(RayTypeK<VSIZEX, intersect>*)(rayData + offset);
            rayPtrs[packetIndex++] = &ray;

            /* trace as stream */
            if (unlikely(packetIndex == MAX_INTERNAL_PACKET_STREAM_SIZE))
            {
              const size_t size = packetIndex*VSIZEX;
              scene->intersectors.intersectN(rayPtrs, size, context);
              packetIndex = 0;
            }
          }

          /* flush remaining packets */
          if (unlikely(packetIndex > 0))
          {
            const size_t size = packetIndex*VSIZEX;
            scene->intersectors.intersectN(rayPtrs, size, context);
          }
        }
        else if (unlikely(!intersect))
        {
          /* octant sorting for occlusion rays */
          RayStreamSOA rayN(rayData, VSIZEX);

          __aligned(64) unsigned int octants[8][MAX_INTERNAL_STREAM_SIZE];
          __aligned(64) RayK<VSIZEX> rays[MAX_INTERNAL_PACKET_STREAM_SIZE];
          __aligned(64) RayK<VSIZEX>* rayPtrs[MAX_INTERNAL_PACKET_STREAM_SIZE];

          unsigned int raysInOctant[8];
          for (unsigned int i = 0; i < 8; i++)
            raysInOctant[i] = 0;
          size_t inputRayID = 0;

          for (;;)
          {
            int curOctant = -1;

            /* sort rays into octants */
            for (; inputRayID < N*numPackets;)
            {
              const size_t offset = (inputRayID / VSIZEX) * stride + (inputRayID % VSIZEX) * sizeof(float);

              /* skip invalid rays */
              if (unlikely(!rayN.isValidByOffset(offset))) { inputRayID++; continue; } // ignore invalid or already occluded rays
  #if defined(EMBREE_IGNORE_INVALID_RAYS)
              __aligned(64) Ray ray = rayN.getRayByOffset(offset);
              if (unlikely(!ray.valid())) { inputRayID++; continue; }
  #endif

              const unsigned int octantID = (unsigned int)rayN.getOctantByOffset(offset);

              assert(octantID < 8);
              octants[octantID][raysInOctant[octantID]++] = (unsigned int)offset;
              inputRayID++;
              if (unlikely(raysInOctant[octantID] == MAX_INTERNAL_STREAM_SIZE))
              {
                curOctant = octantID;
                break;
              }
            }

            /* need to flush rays in octant? */
            if (unlikely(curOctant == -1))
            {
              for (unsigned int i = 0; i < 8; i++)
                if (raysInOctant[i]) { curOctant = i; break; }
            }

            /* all rays traced? */
            if (unlikely(curOctant == -1))
              break;

            unsigned int* const rayOffsets = &octants[curOctant][0];
            const unsigned int numOctantRays = raysInOctant[curOctant];
            assert(numOctantRays);

            for (unsigned int j = 0; j < numOctantRays; j += VSIZEX)
            {
              const vintx vi = vintx(int(j)) + vintx(step);
              const vboolx valid = vi < vintx(int(numOctantRays));
              const vintx offset = *(vintx*)&rayOffsets[j];
              RayK<VSIZEX>& ray = rays[j/VSIZEX];
              rayPtrs[j/VSIZEX] = &ray;
              ray = rayN.getRayByOffset(valid, offset);
              ray.tnear() = select(valid, ray.tnear(), zero);
              ray.tfar()  = select(valid, ray.tfar(),  neg_inf);
            }

            scene->intersectors.occludedN(rayPtrs, numOctantRays, context);

            for (unsigned int j = 0; j < numOctantRays; j += VSIZEX)
            {
              const vintx vi = vintx(int(j)) + vintx(step);
              const vboolx valid = vi < vintx(int(numOctantRays));
              const vintx offset = *(vintx*)&rayOffsets[j];
              rayN.setHitByOffset(valid, offset, rays[j/VSIZEX]);
            }
            raysInOctant[curOctant] = 0;
          }
        }
        else
        {
          /* fallback to packets */
          for (size_t i = 0; i < numPackets; i++)
          {
            const size_t offset = i * stride;
            RayTypeK<VSIZEX, intersect>& ray = *(RayTypeK<VSIZEX, intersect>*)(rayData + offset);
            const vboolx valid = ray.tnear() <= ray.tfar();

            scene->intersectors.intersect(valid, ray, context);
          }
        }
      }
      else
      {
        /* fallback to packets for arbitrary packet size and alignment */
        for (size_t i = 0; i < numPackets; i++)
        {
          const size_t offsetN = i * stride;
          RayStreamSOA rayN(rayData + offsetN, N);

          for (size_t j = 0; j < N; j += VSIZEX)
          {
            const size_t offset = j * sizeof(float);
            vboolx valid = (vintx(int(j)) + vintx(step)) < vintx(int(N));

            RayTypeK<VSIZEX, intersect> ray = rayN.getRayByOffset(valid, offset);
            valid &= ray.tnear() <= ray.tfar();

            scene->intersectors.intersect(valid, ray, context);

            rayN.setHitByOffset(valid, offset, ray);
          }
        }
      }
    }

    template<bool intersect>
    __forceinline void RayStreamFilter::filterSOP(Scene* scene, const void* _rayN, size_t N, IntersectContext* context)
    { 
      RayStreamSOP& rayN = *(RayStreamSOP*)_rayN;

      /* use fast path for coherent ray mode */
      if (unlikely(isCoherent(context->user->flags)))
      {
        __aligned(64) RayTypeK<VSIZEX, intersect> rays[MAX_INTERNAL_PACKET_STREAM_SIZE];
        __aligned(64) RayTypeK<VSIZEX, intersect>* rayPtrs[MAX_INTERNAL_PACKET_STREAM_SIZE];

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

            RayTypeK<VSIZEX, intersect> ray = rayN.getRayByOffset(valid, offset);
            ray.tnear() = select(valid, ray.tnear(), zero);
            ray.tfar()  = select(valid, ray.tfar(),  neg_inf);

            rays[packetIndex] = ray;
            rayPtrs[packetIndex] = &rays[packetIndex]; // rayPtrs might get reordered for occludedN
          }

          /* trace stream */
          scene->intersectors.intersectN(rayPtrs, size, context);

          /* convert from SOA to SOP */
          for (size_t j = 0; j < size; j += VSIZEX)
          {
            const vintx vij = vintx(int(i+j)) + vintx(step);
            const vboolx valid = vij < vintx(int(N));
            const size_t offset = (i+j) * sizeof(float);
            const size_t packetIndex = j / VSIZEX;

            rayN.setHitByOffset(valid, offset, rays[packetIndex]);
          }
        }
      }
      else if (unlikely(!intersect))
      {
        /* octant sorting for occlusion rays */
        __aligned(64) unsigned int octants[8][MAX_INTERNAL_STREAM_SIZE];
        __aligned(64) RayK<VSIZEX> rays[MAX_INTERNAL_PACKET_STREAM_SIZE];
        __aligned(64) RayK<VSIZEX>* rayPtrs[MAX_INTERNAL_PACKET_STREAM_SIZE];

        unsigned int raysInOctant[8];
        for (unsigned int i = 0; i < 8; i++)
          raysInOctant[i] = 0;
        size_t inputRayID = 0;

        for (;;)
        {
          int curOctant = -1;

          /* sort rays into octants */
          for (; inputRayID < N;)
          {
            const size_t offset = inputRayID * sizeof(float);
            /* skip invalid rays */
            if (unlikely(!rayN.isValidByOffset(offset))) { inputRayID++; continue; } // ignore invalid or already occluded rays
#if defined(EMBREE_IGNORE_INVALID_RAYS)
            __aligned(64) Ray ray = rayN.getRayByOffset(offset);
            if (unlikely(!ray.valid())) { inputRayID++; continue; }
#endif

            const unsigned int octantID = (unsigned int)rayN.getOctantByOffset(offset);

            assert(octantID < 8);
            octants[octantID][raysInOctant[octantID]++] = (unsigned int)offset;
            inputRayID++;
            if (unlikely(raysInOctant[octantID] == MAX_INTERNAL_STREAM_SIZE))
            {
              curOctant = octantID;
              break;
            }
          }

          /* need to flush rays in octant? */
          if (unlikely(curOctant == -1))
          {
            for (unsigned int i = 0; i < 8; i++)
              if (raysInOctant[i]) { curOctant = i; break; }
          }

          /* all rays traced? */
          if (unlikely(curOctant == -1))
            break;

          unsigned int* const rayOffsets = &octants[curOctant][0];
          const unsigned int numOctantRays = raysInOctant[curOctant];
          assert(numOctantRays);

          for (unsigned int j = 0; j < numOctantRays; j += VSIZEX)
          {
            const vintx vi = vintx(int(j)) + vintx(step);
            const vboolx valid = vi < vintx(int(numOctantRays));
            const vintx offset = *(vintx*)&rayOffsets[j];
            RayK<VSIZEX>& ray = rays[j/VSIZEX];
            rayPtrs[j/VSIZEX] = &ray;
            ray = rayN.getRayByOffset(valid, offset);
            ray.tnear() = select(valid, ray.tnear(), zero);
            ray.tfar()  = select(valid, ray.tfar(),  neg_inf);
          }

          scene->intersectors.occludedN(rayPtrs, numOctantRays, context);

          for (unsigned int j = 0; j < numOctantRays; j += VSIZEX)
          {
            const vintx vi = vintx(int(j)) + vintx(step);
            const vboolx valid = vi < vintx(int(numOctantRays));
            const vintx offset = *(vintx*)&rayOffsets[j];
            rayN.setHitByOffset(valid, offset, rays[j/VSIZEX]);
          }

          raysInOctant[curOctant] = 0;
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

          RayTypeK<VSIZEX, intersect> ray = rayN.getRayByOffset(valid, offset);
          valid &= ray.tnear() <= ray.tfar();

          scene->intersectors.intersect(valid, ray, context);

          rayN.setHitByOffset(valid, offset, ray);
        }
      }
    }


    void RayStreamFilter::intersectAOS(Scene* scene, RTCRayHit* _rayN, size_t N, size_t stride, IntersectContext* context) {
      filterAOS<true>(scene, _rayN, N, stride, context);
    }

    void RayStreamFilter::occludedAOS(Scene* scene, RTCRay* _rayN, size_t N, size_t stride, IntersectContext* context) {
      filterAOS<false>(scene, _rayN, N, stride, context);
    }

    void RayStreamFilter::intersectAOP(Scene* scene, RTCRayHit** _rayN, size_t N, IntersectContext* context) {
      filterAOP<true>(scene, (void**)_rayN, N, context);
    }

    void RayStreamFilter::occludedAOP(Scene* scene, RTCRay** _rayN, size_t N, IntersectContext* context) {
      filterAOP<false>(scene, (void**)_rayN, N, context);
    }

    void RayStreamFilter::intersectSOA(Scene* scene, char* rayData, size_t N, size_t numPackets, size_t stride, IntersectContext* context) {
      filterSOA<true>(scene, rayData, N, numPackets, stride, context);
    }

    void RayStreamFilter::occludedSOA(Scene* scene, char* rayData, size_t N, size_t numPackets, size_t stride, IntersectContext* context) {
      filterSOA<false>(scene, rayData, N, numPackets, stride, context);
    }

    void RayStreamFilter::intersectSOP(Scene* scene, const RTCRayHitNp* _rayN, size_t N, IntersectContext* context) {
      filterSOP<true>(scene, _rayN, N, context);
    }

    void RayStreamFilter::occludedSOP(Scene* scene, const RTCRayNp* _rayN, size_t N, IntersectContext* context) {
      filterSOP<false>(scene, _rayN, N, context);
    }


    RayStreamFilterFuncs rayStreamFilterFuncs() {
      return RayStreamFilterFuncs(RayStreamFilter::intersectAOS, RayStreamFilter::intersectAOP, RayStreamFilter::intersectSOA, RayStreamFilter::intersectSOP,
                                  RayStreamFilter::occludedAOS,  RayStreamFilter::occludedAOP,  RayStreamFilter::occludedSOA,  RayStreamFilter::occludedSOP);
    }
  };
};
