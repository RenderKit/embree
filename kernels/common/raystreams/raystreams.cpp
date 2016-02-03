#include "raystreams.h"

namespace embree
{
  namespace isa
  {
#define MAX_RAYS_PER_OCTANT 32

    void RayStream::intersectAOS(Scene *scene, Ray* rayN, const size_t N, const size_t stride, const size_t flags)
    {
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

          const unsigned int octantID = \
            (ray.dir.x < 0.0f ? 1 : 0) + 
            (ray.dir.y < 0.0f ? 2 : 0) + 
            (ray.dir.z < 0.0f ? 4 : 0);

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

        //scene->intersectN(rays,numOctantRays,flags);

        rays_in_octant[cur_octant] = 0;

      }
    }


  };
};
