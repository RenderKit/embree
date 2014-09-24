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
    class __aligned(64) IrregularSubdividedCatmullClarkPatch
    {
    public:
      //CatmullClark1Ring ring[4];
      Array<Vec3fa> points;

      __forceinline Vec3fa& face (size_t x, size_t y) { return points(2*x,2*y); }
      __forceinline Vec3fa& vedge(size_t x, size_t y) { return points(2*x-1,2*y); }
      __forceinline Vec3fa& hedge(size_t x, size_t y) { return points(2*x,2*y-1); }
      __forceinline Vec3fa& point(size_t x, size_t y) { return points(2*x-1,2*y-1); }
      
      __forceinline void subdivide(IrregularSubdividedCatmullClarkPatch& out) const
      {
        /* greate properly sized output patch */
        const size_t width = points.width();
        const size_t height = points.height();
        out.init(2*width-3, 2*height-3);

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
      }
    };
  }
}
