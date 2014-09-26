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
#include <iomanip>

namespace embree
{
  //namespace isa
  //{
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

      void init(size_t width, size_t height, const T& v) 
      {
        size_x = width; size_y = height;
        delete[] array; array = new T[width*height];
        for (size_t y=0; y<height; y++)
          for (size_t x=0; x<width; x++)
            array[y*size_x+x] = v;
      }

      __forceinline size_t width() const { 
        return size_x;
      }

      __forceinline size_t height() const {
        return size_y;
      }

      __forceinline Vec3fa& operator() (size_t x, size_t y) {
        assert(x<size_x);
        assert(y<size_y);
        return array[y*size_x+x];
      }

      __forceinline const Vec3fa& operator() (size_t x, size_t y) const {
        assert(x<size_x);
        assert(y<size_y);
        return array[y*size_x+x];
      }

    private:
      T* array;
      size_t size_x, size_y;
    };

    class CatmullClark1Edge
    {
    public:
      void init(size_t N, CatmullClark1Ring& r0, CatmullClark1Ring& r1)
      {
        v.init(N,3);
        this->N = 2;
        init(r0,r1);
      }

      __forceinline void subdivide(CatmullClark1Ring& r0, CatmullClark1Ring& r1) 
      {
        assert(2*N-1 < v.width());
        size_t N0 = N;
        size_t N1 = 2*N-1;
        
        Vec3fa v20 = zero;
        Vec3fa v21 = zero;
        Vec3fa v22 = zero;

        Vec3fa v10 = v(N-1,0);
        Vec3fa v11 = v(N-1,1);
        Vec3fa v12 = v(N-1,2);

        for (ssize_t x=N-2; x>=0; x--) 
        {
          PRINT(x);

          /* load next column */
          Vec3fa v00 = v(x,0);
          Vec3fa v01 = v(x,1);
          Vec3fa v02 = v(x,2);

          /* calculate face points and edge centers */
          const Vec3fa c00 = 0.25f*(v00+v10+v01+v11);
          const Vec3fa c10 = 0.50f*(v10+v11);
          const Vec3fa c20 = 0.25f*(v10+v20+v11+v21);
          const Vec3fa c01 = 0.50f*(v01+v11);
          const Vec3fa c21 = 0.50f*(v11+v21);
          const Vec3fa c02 = 0.25f*(v01+v11+v02+v12);
          const Vec3fa c12 = 0.50f*(v11+v12);
          const Vec3fa c22 = 0.25f*(v11+v21+v12+v22);
          PRINT(c00.z);
          PRINT(c10.z);
          PRINT(c20.z);
          PRINT(c01.z);
          PRINT(c21.z);
          PRINT(c02.z);
          PRINT(c12.z);
          PRINT(c22.z);

          /* store face points and edge point at 2*x+1 */
          v(2*x+1,0) = c00;
          v(2*x+1,1) = 0.5f*(c01+0.5f*(c00+c02));
          v(2*x+1,2) = c02;
          
          /* store face points and edge point at 2*x+2 */
          const Vec3fa F = 0.25f*(c00+c20+c02+c22);
          const Vec3fa R = 0.25f*(c10+c01+c21+c12);
          const Vec3fa P = v11;
          v(2*x+2,0) = 0.5f*(c10+0.5f*(c00+c20));
          v(2*x+2,1) = 0.25*F + 0.5*R + 0.25*P;
          v(2*x+2,2) = 0.5f*(c12+0.5f*(c02+c22));

          /* propagate points to next iteration */
          v20 = v10; v10 = v00;
          v21 = v11; v11 = v01;
          v22 = v12; v12 = v02;
        }        
        N = N1; PRINT(*this); N = N0;
        
        N=2*N-1;
        init(r0,r1);
      }

      __forceinline void init(CatmullClark1Ring& ring0, CatmullClark1Ring& ring1) 
      {
        v(0,0) = ring0.first();
        v(0,1) = ring0.vtx;
        v(0,2) = ring0.end();
        v(N-1,0) = ring1.last();
        v(N-1,1) = ring1.vtx;
        v(N-1,2) = ring1.begin();
      }

      friend __forceinline std::ostream &operator<<(std::ostream& out, const CatmullClark1Edge& edge)
      {
        out << std::endl;
        for (size_t y=0; y<3; y++) {
          for (size_t x=0; x<edge.N; x++) {
            //out << patch.v(x,y) << " ";
            out << std::setw(10) << edge.v(x,y).z << " ";
          }
          out << std::endl;
        }
        return out;
      } 

    public:
      Array2D<Vec3fa> v;
      size_t N;
    };

    class __aligned(64) IrregularSubdividedCatmullClarkPatch2
    {
    public:

      IrregularSubdividedCatmullClarkPatch2 (const SubdivMesh::HalfEdge* h, const Vec3fa* const vertices, size_t level)
        : N(2)
      {
        size_t M = (1<<level)+3;
        v.init(M,M,Vec3fa(nan));
        
        ring00.init(h,vertices); h = h->next();
        ring01.init(h,vertices); h = h->next();
        ring11.init(h,vertices); h = h->next();
        ring10.init(h,vertices); h = h->next();
        edgeT.init(M,ring00,ring10);
        edgeR.init(M,ring10,ring11);
        edgeB.init(M,ring11,ring01);
        edgeL.init(M,ring01,ring00);
        init();
        
        std::cout << *this << std::endl;
        for (size_t l=0; l<level; l++) {
          subdivide();
          PRINT(l+1);
          std::cout << *this << std::endl;
        }
      }

      __forceinline size_t size() const { return N; }
      __forceinline Vec3fa& get(size_t x, size_t y) { return v(x,y); }

      void subdivide_points()
      {
        assert(2*N < v.width());
        assert(2*N < v.height());

        for (size_t y=1; y<N; y++)
        {
          //Vec3fa v20 = zero;
          Vec3fa v21 = zero;
          Vec3fa v22 = zero;

          //Vec3fa v10 = v(N-1,0);
          Vec3fa v11 = v(N-1,1);
          Vec3fa v12 = v(N-1,2);

          for (ssize_t x=N-2; x>=0; x--) 
          {
            /* load next column */
            //Vec3fa v00 = v(x,y-1);
            Vec3fa v01 = v(x,y+0);
            Vec3fa v02 = v(x,y+1);
            
            /* calculate face points and edge centers */
            const Vec3fa c00 = v(2*x+1,2*y-1);
            const Vec3fa c10 = v(2*x+2,2*y-1);
            const Vec3fa c20 = v(2*x+3,2*y-1);
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
          }        
        }

        N=2*N-1;
        init();
      }

      void init()
      {
        for (size_t i=0; i<N; i++)
        {
          v(i,0) = edgeT.v(i,1);
          v(N-1,i) = edgeR.v(i,1);
          v(i,N-1) = edgeB.v(N-1-i,1);
          v(0,i) = edgeL.v(N-1-i,1);
        }
      }

      void subdivide()
      {
        PING;
        CatmullClark1Ring ring00a; ring00.update(ring00a); ring00 = ring00a;
        CatmullClark1Ring ring01a; ring01.update(ring01a); ring01 = ring01a;
        CatmullClark1Ring ring10a; ring10.update(ring10a); ring10 = ring10a;
        CatmullClark1Ring ring11a; ring11.update(ring11a); ring11 = ring11a;
        PING;
        PRINT(edgeT);
        edgeT.subdivide(ring00,ring10);
        PRINT(edgeT);

        edgeR.subdivide(ring10,ring11);
        edgeB.subdivide(ring11,ring01);
        edgeL.subdivide(ring01,ring00);
        subdivide_points();
      }

      friend __forceinline std::ostream &operator<<(std::ostream& out, const IrregularSubdividedCatmullClarkPatch2& patch)
      {
        size_t N = patch.N;
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
        for (size_t x=0; x<N; x++) out << std::setw(10) << patch.edgeT.v(x,0).z << " ";
        out << std::endl;
        out << std::endl;
        for (size_t y=0; y<N; y++) {
          out << std::setw(10) << patch.edgeL.v(N-1-y,0).z << "    ";
          for (size_t x=0; x<N; x++) out << std::setw(10) << patch.v(x,y).z << " ";
          out << std::setw(10) << "    " << patch.edgeR.v(y,0).z;
          out << std::endl;
        }
        out << std::endl;
        out << "              ";
        for (size_t x=0; x<N; x++) out << std::setw(10) << patch.edgeB.v(N-1-x,0).z << " ";
        out << std::endl;
        return out;
      } 

    private:
      size_t N;
      CatmullClark1Ring ring00,ring01,ring10,ring11;
      CatmullClark1Edge edgeT, edgeB, edgeL, edgeR; 
      Array2D<Vec3fa> v;
    };

    class __aligned(64) IrregularSubdividedCatmullClarkPatch
    {
      typedef CatmullClark1Ring Ring;

    public:
      Ring ring00,ring01,ring10,ring11;
      Array2D<Vec3fa> points;

      __forceinline Vec3fa& face (size_t x, size_t y) { return points(2*x,2*y); }
      __forceinline Vec3fa& vedge(size_t x, size_t y) { return points(2*x-1,2*y); }
      __forceinline Vec3fa& hedge(size_t x, size_t y) { return points(2*x,2*y-1); }
      __forceinline Vec3fa& point(size_t x, size_t y) { return points(2*x-1,2*y-1); }
      __forceinline       Vec3fa& get  (size_t x, size_t y)       { return points(x,y); }
      __forceinline const Vec3fa& get  (size_t x, size_t y) const { return points(x,y); }

      IrregularSubdividedCatmullClarkPatch () {}

      IrregularSubdividedCatmullClarkPatch (size_t width, size_t height) 
      {
        points.init(width,height);
      }

      void init (size_t width, size_t height) {
        points.init(width,height);
      }

      IrregularSubdividedCatmullClarkPatch (const SubdivMesh::HalfEdge* h, const Vec3fa *const vertices)
      {
        points.init(4,4);
        ring00.init(h,vertices); h = h->next();
        ring01.init(h,vertices); h = h->next();
        ring11.init(h,vertices); h = h->next();
        ring10.init(h,vertices); h = h->next();
        handle_corners();
      }

      void handle_corners()
      {
        size_t w = points.width();
        size_t h = points.height();
        
        points(0,0) = Vec3fa(zero);
        points(1,1) = ring00.vtx;
        points(1,0) = ring00.first();
        points(0,1) = ring00.last();

        points(0,h-1) = Vec3fa(zero);
        points(1,h-2) = ring01.vtx;
        points(1,h-1) = ring01.last();
        points(0,h-2) = ring01.first();

        points(w-1,h-1) = Vec3fa(zero);
        points(w-2,h-2) = ring11.vtx;
        points(w-2,h-1) = ring11.first();
        points(w-1,h-2) = ring11.last();

        points(w-1,0) = Vec3fa(zero);
        points(w-2,1) = ring10.vtx;
        points(w-2,0) = ring10.last();
        points(w-1,1) = ring10.first();
      }
      
      __forceinline void subdivide(IrregularSubdividedCatmullClarkPatch* out)
      {
        /* greate properly sized output patch */
        const size_t width  = points.width();
        const size_t height = points.height();
        out->init(2*width-3, 2*height-3);

        /* subdivide corner rings first */
        ring00.update(out->ring00);
        ring01.update(out->ring01);
        ring10.update(out->ring10);
        ring11.update(out->ring11);

        /* calculate face points */
        for (size_t y=0; y<height-1; y++) {
          for (size_t x=0; x<width-1; x++) {
            out->face(x,y) = 0.25f*(get(x+0,y+0) + get(x+0,y+1) + get(x+1,y+0) + get(x+1,y+1));
          }
        }

        /* calculate vertical edge centers */
        for (size_t y=0; y<height-1; y++) {
          for (size_t x=1; x<width-1; x++) {
            out->vedge(x,y) = 0.5f*(get(x,y)+get(x,y+1));
          }
        }

        /* calculate horizontal edge centers */
        for (size_t y=1; y<height-1; y++) {
          for (size_t x=0; x<width-1; x++) {
            out->hedge(x,y) = 0.5f*(get(x,y)+get(x+1,y));
          }
        }

        /* calculate base points */
        for (size_t y=1; y<height-1; y++) {
          for (size_t x=1; x<width-1; x++) {
            const Vec3fa F = 0.25f*(out->face(x-1,y-1)+out->face(x-1,y)+out->face(x,y-1)+out->face(x,y));
            const Vec3fa R = 0.25f*(out->hedge(x-1,y)+out->hedge(x,y)+out->vedge(x,y-1)+out->vedge(x,y));
            const Vec3fa P = get(x,y);
            out->point(x,y) = 0.25*F + 0.5*R + 0.25*P;
          }
        }

        /* calculate vertical edge points */
        for (size_t y=0; y<height-1; y++) {
          for (size_t x=1; x<width-1; x++) {
            out->vedge(x,y) = 0.5f*(out->vedge(x,y) + 0.5f*(out->face(x-1,y)+out->face(x,y)));
          }
        }

        /* calculate horizontal edge points */
        for (size_t y=1; y<height-1; y++) {
          for (size_t x=0; x<width-1; x++) {
            out->hedge(x,y) = 0.5f*(out->hedge(x,y) + 0.5f*(out->face(x,y-1)+out->face(x,y)));
          }
        }

        /* copy invalid corner points from corner rings */
        out->handle_corners();
      }

      friend __forceinline std::ostream &operator<<(std::ostream &o, const IrregularSubdividedCatmullClarkPatch& patch)
      {
        o << "ring00 = " << patch.ring00 << std::endl;
        o << "ring01 = " << patch.ring01 << std::endl;
        o << "ring10 = " << patch.ring10 << std::endl;
        o << "ring11 = " << patch.ring11 << std::endl;
        for (size_t y=0; y<patch.points.height(); y++) {
          for (size_t x=0; x<patch.points.width(); x++) {
            o << patch.points(x,y) << " ";
            //o << std::setw(10) << patch.points(x,y).z << " ";
          }
          o << std::endl;
        }
        return o;
      } 
    };
//}
}
