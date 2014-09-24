// ======================================================================== //
// Copyright 2009-2014 Intel Corporation                                    //
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

#pragma once

#include "common/default.h"
#include "common/scene_subdivision.h"

namespace embree
{
  namespace isa
  {
    template<typename T>
    class Array2D
    {
    public:
      Array2D () : array(NULL), size_x(0), size_y(0) {}
      ~Array2D () { delete[] array; }

      void init(size_t width, size_t height) {
        size_x = width; size_y = height;
        delete[] array; array = new T[width*height];
      }

      __forceinline size_t width() const { 
        return size_x;
      }

      __forceinline size_t height() const {
        return size_y;
      }

      __forceinline Vec3fa& operator() (size_t x, size_t y) {
        return array[y*width+x];
      }

    private:
      T* array;
      size_t size_x, size_y;
    };

    class __aligned(64) IrregularSubdividedCatmullClarkPatch
    {
      struct Ring
      {
        Vec3fa vtx;
        Vec3fa ring[2*MAX_VALENCE]; //!< two vertices per face
        unsigned int N;             //!< number of vertices
        
        __forceinline void subdivide (Ring& dest) const
        {
          dest.N = N;
          for (size_t i=1; i<N; i+=2) F += dest.ring[i];
          for (size_t i=0; i<N; i+=2) R += dest.ring[i];

          /* compute face points */
          Vec3fa F(zero);
          for (size_t i=1; i<N; i+=2)
            const Vec3fa f = 0.25f*(vtx+ring[i-1]+ring[i]+ring[i+1]); F += f;
            dest.ring[i] = f;

          /* compute edge points */
          Vec3fa R(zero);
          const Vec3fa r = 0.5f*(vtx+ring[0]); R += r;
          const Vec3fa f = 0.5f*(dst.ring[N-1] + dst.ring[1]);
          dest.ring[0] = 0.5f*(f+r); 
          for (size_t i=1; i<N; i+=2) {
            const Vec3fa r = 0.5f*(vtx+ring[i]); R += r;
            const Vec3fa f = 0.5f*(dst.ring[i-1] + dst.ring[i+1]);
            dest.ring[i] = 0.5f*(f+r); 
          }

          /* compute new point */
          const size_t valence = N/2;
          F /= (float)valence;
          R /= (float)valence; 
          dest.vtx = (F + 2.0f * R + (float)(valence-3)*vtx) / valence;
        }
    };

    public:
      Ring ring00,ring01,ring10,ring11;
      Array2D<Vec3fa> points;

      __forceinline Vec3fa& face (size_t x, size_t y) { return points(2*x,2*y); }
      __forceinline Vec3fa& vedge(size_t x, size_t y) { return points(2*x-1,2*y); }
      __forceinline Vec3fa& hedge(size_t x, size_t y) { return points(2*x,2*y-1); }
      __forceinline Vec3fa& point(size_t x, size_t y) { return points(2*x-1,2*y-1); }

      IrregularSubdividedCatmullClarkPatch (Vec3fa patch[3][3])
      {
        init(3,3);
        for (size_t y=0; y<3; y++)
          for (size_t x=0; x<3; x++)
            points(x,y) = patch[y][x];
      }

      void init(size_t width, size_t height) {
        points.init(width,height);
      }
      
      __forceinline void subdivide(IrregularSubdividedCatmullClarkPatch& out) const
      {
        /* greate properly sized output patch */
        const size_t width  = points.width();
        const size_t height = points.height();
        out.init(2*width-3, 2*height-3);

        /* subdivide corner rings first */
        Ring ring00b; ring00.subdivide(ring00b); ring00 = ring00b;
        Ring ring01b; ring01.subdivide(ring01b); ring01 = ring01b;
        Ring ring10b; ring10.subdivide(ring10b); ring10 = ring10b;
        Ring ring11b; ring11.subdivide(ring11b); ring11 = ring11b;

        /* calculate face points */
        for (size_t y=0; y<height-1; y++) {
          for (size_t x=0; x<width-1; x++) {
            out.face(x,y) = 0.25f*(get(x+0,y+0) + get(x+0,y+1) + get(x+1,y+0) + get(x+1,y+1));
          }
        }

        /* calculate vertical edge centers */
        for (size_t y=0; y<height-1; y++) {
          for (size_t x=1; x<width-1; x++) {
            out.vedge(x,y) = 0.5f*(get(x,y)+get(x,y+1));
          }
        }

        /* calculate horizontal edge centers */
        for (size_t y=1; y<height-1; y++) {
          for (size_t x=0; x<width-1; x++) {
            out.hedge(x,y) = 0.5f*(get(x,y)+get(x+1,y));
          }
        }

        /* calculate base points */
        for (size_t y=1; y<height-1; y++) {
          for (size_t x=1; x<width-1; x++) {
            const Vec3fa F = 0.25f*(face(x-1,y-1)+face(x-1,y)+face(x,y-1)+face(x,y));
            const Vec3fa R = 0.25f*(hedge(x-1,y)+hedge(x,y)+vedge(x,y-1)+vedge(x,y));
            const Vec3fa P = get(x,y);
            out.point(x,y) = 0.25*F + 0.5*R + 0.25*P;
          }
        }

        /* calculate vertical edge points */
        for (size_t y=0; y<height-1; y++) {
          for (size_t x=1; x<width-1; x++) {
            out.vedge(x,y) = 0.5f*(vedge(x,y) + (face(x-1,y)+face(x,y)));
          }
        }

        /* calculate horizontal edge points */
        for (size_t y=1; y<height-1; y++) {
          for (size_t x=0; x<width-1; x++) {
            out.hedge(x,y) = 0.25f*(hedge(x,y) + (face(x,y-1)+face(x,y)));
          }
        }

        /* copy invalid corner points from corner rings */
        out.point(0  ,0  ) = ring00.vtx;
        out.point(0  ,N-1) = ring01.vtx;
        out.point(N-1,0  ) = ring10.vtx;
        out.point(N-1,N-1) = ring11.vtx;

        out.vedge(  1,  0) = ring00.first();
        out.vedge(  1,N-1) = ring01.last();
        out.vedge(N-1,N-1) = ring11.first();
        out.vedge(N-1,  0) = ring10.last();

        out.hedge(  0,  1) = ring00.last();
        out.hedge(  0,N-1) = ring01.first();
        out.hedge(N-1,N-1) = ring11.last();
        out.hedge(N-1,  1) = ring10.first();
      }
    };
  }
}
