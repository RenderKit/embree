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
  //namespace isa
  //{
    class __aligned(64) IrregularSubdividedCatmullClarkPatch
    {
    public:

      IrregularSubdividedCatmullClarkPatch (const SubdivMesh::HalfEdge* h, const Vec3fa* const vertices, size_t level)
        : K(1)
      {
        size_t M = (1<<level)+1;
        v.init(M,M,Vec3fa(nan));
        
        ring00.init(h,vertices); h = h->next();
        ring10.init(h,vertices); h = h->next();
        ring11.init(h,vertices); h = h->next();
        ring01.init(h,vertices); h = h->next();
        edgeT.init(M,ring00,ring10);
        edgeR.init(M,ring10,ring11);
        edgeB.init(M,ring11,ring01);
        edgeL.init(M,ring01,ring00);
        init();
        
        for (size_t l=0; l<level; l++) {
          subdivide();
        }
      }

      __forceinline size_t size() const { return K+1; }
      __forceinline Vec3fa& get(size_t x, size_t y) { assert(x<=K); assert(y<=K); return v(x,y); }

      void subdivide_points()
      {
        size_t K0 = K;
        size_t K1 = 2*K;

        assert(2*K+1 <= v.width());
        assert(2*K+1 <= v.height());

        for (ssize_t y=K; y>=0; y--) {
          for (ssize_t x=K; x>=0; x--) {
            v(2*x+0,2*y+0) = v(x,y);
          }
        }
        for (ssize_t y=0; y<K; y++) {
          for (ssize_t x=0; x<K; x++) {
            v(2*x+1,2*y+0) = Vec3fa(nan);
            v(2*x+0,2*y+1) = Vec3fa(nan);
            v(2*x+1,2*y+1) = Vec3fa(nan);
          }
        }

        for (ssize_t x=K-1; x>=0; x--) 
        {
          const Vec3fa c00 = v(2*x+0,0);
          const Vec3fa c10 = v(2*x+2,0);
          const Vec3fa c01 = v(2*x+0,2);
          const Vec3fa c11 = v(2*x+2,2);
          v(2*x+1,1) = 0.25f*(c00+c01+c10+c11);
          v(2*x+2,1) = 0.50f*(c10+c11);
        }
        
        for (size_t y=1; y<K; y++)
        {
          //Vec3fa v20 = zero;
          Vec3fa c20 = zero;
          Vec3fa v21 = zero;
          Vec3fa v22 = zero;

          //Vec3fa v10 = v(2*K,0);
          Vec3fa v11 = v(2*K,2*y);
          Vec3fa v12 = v(2*K,2*y+2);

          for (ssize_t x=K-1; x>=0; x--) 
          {
            /* load next column */
            //Vec3fa v00 = v(2*x,2*y-2);
            Vec3fa v01 = v(2*x,2*y+0);
            Vec3fa v02 = v(2*x,2*y+2);
            
            /* calculate face points and edge centers */
            const Vec3fa c00 = v(2*x+1,2*y-1);
            const Vec3fa c10 = v(2*x+2,2*y-1);
            //const Vec3fa c20 = v(2*x+3,2*y-1);
            const Vec3fa c01 = 0.50f*(v01+v11);
            const Vec3fa c21 = 0.50f*(v11+v21);
            const Vec3fa c02 = 0.25f*(v01+v11+v02+v12);
            const Vec3fa c12 = 0.50f*(v11+v12);
            const Vec3fa c22 = 0.25f*(v11+v21+v12+v22);

            /* store face points and edge point at 2*x+1 */
            //v(2*x+1,2*y-1) = c00;
            v(2*x+1,2*y+0) = 0.5f*(c01+0.5f*(c00+c02));
            v(2*x+1,2*y+1) = c02;

            /* store face points and edge point at 2*x+2 */
            const Vec3fa F = 0.25f*(c00+c20+c02+c22);
            const Vec3fa R = 0.25f*(c10+c01+c21+c12);
            const Vec3fa P = v11;
            v(2*x+2,2*y-1) = 0.5f*(c10+0.5f*(c00+c20));
            v(2*x+2,2*y+0) = 0.25*F + 0.5*R + 0.25*P;
            v(2*x+2,2*y+1) = c12;
            
            /* propagate points to next iteration */
            //v20 = v10; v10 = v00;
            v21 = v11; v11 = v01;
            v22 = v12; v12 = v02;
            c20 = c00;
          }        
        }

        for (ssize_t x=1; x<K; x++) 
          v(2*x,2*K-1) = 0.5f*(v(2*x,2*K-1)+0.5f*(v(2*x-1,2*K-1)+v(2*x+1,2*K-1)));

        K=2*K;
        init();
      }

      void init()
      {
        for (size_t i=0; i<(K+1); i++)
        {
          v(i,0) = edgeT.v(i,1);
          v(K,i) = edgeR.v(i,1);
          v(i,K) = edgeB.v(K-i,1);
          v(0,i) = edgeL.v(K-i,1);
        }
      }

      void subdivide()
      {
        CatmullClark1Ring ring00a; ring00.update(ring00a); ring00 = ring00a;
        CatmullClark1Ring ring01a; ring01.update(ring01a); ring01 = ring01a;
        CatmullClark1Ring ring10a; ring10.update(ring10a); ring10 = ring10a;
        CatmullClark1Ring ring11a; ring11.update(ring11a); ring11 = ring11a;
        edgeT.subdivide(ring00,ring10);
        edgeR.subdivide(ring10,ring11);
        edgeB.subdivide(ring11,ring01);
        edgeL.subdivide(ring01,ring00);
        subdivide_points();
      }

      friend __forceinline std::ostream &operator<<(std::ostream& out, const IrregularSubdividedCatmullClarkPatch& patch)
      {
        size_t N = patch.K+1;
        out << "ring00 = " << patch.ring00 << std::endl;
        out << "ring01 = " << patch.ring01 << std::endl;
        out << "ring10 = " << patch.ring10 << std::endl;
        out << "ring11 = " << patch.ring11 << std::endl;
        out << "edgeT  = " << patch.edgeT << std::endl;
        out << "edgeR  = " << patch.edgeR << std::endl;
        out << "edgeB  = " << patch.edgeB << std::endl;
        out << "edgeL  = " << patch.edgeL << std::endl;

        out << std::endl;
        out << "              ";
        for (size_t x=0; x<N; x++) out << std::setw(10) << patch.edgeT.v(x,0).x << " ";
        out << std::endl;
        out << std::endl;
        for (size_t y=0; y<N; y++) {
          out << std::setw(10) << patch.edgeL.v(N-1-y,0).x << "    ";
          for (size_t x=0; x<N; x++) out << std::setw(10) << patch.v(x,y).x << " ";
          out << std::setw(10) << "    " << patch.edgeR.v(y,0).x;
          out << std::endl;
        }
        out << std::endl;
        out << "              ";
        for (size_t x=0; x<N; x++) out << std::setw(10) << patch.edgeB.v(N-1-x,0).x << " ";
        out << std::endl;
        return out;
      } 

    private:
      size_t K;
      CatmullClark1Ring ring00,ring01,ring10,ring11;
      CatmullClark1Edge edgeT, edgeB, edgeL, edgeR; 
      Array2D<Vec3fa> v;
    };
//}
}
