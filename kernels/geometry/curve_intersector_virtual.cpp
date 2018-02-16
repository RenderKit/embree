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
 
#include "curve_intersector_virtual.h"
#include "intersector_epilog.h"

#include "../subdiv/bezier_curve.h"
#include "../subdiv/bspline_curve.h"

#include "curveNi_intersector.h"
#include "curveNv_intersector.h"
#include "curveNi_mb_intersector.h"

#include "curve_intersector_distance.h"
#include "curve_intersector_ribbon.h"
#include "curve_intersector_oriented.h"
#include "curve_intersector_sweep.h"

namespace embree
{
  namespace isa
  {
    template<typename Curve3fa, int N>
    static VirtualCurvePrimitive::Intersectors RibbonNiIntersectors()
    {
      VirtualCurvePrimitive::Intersectors intersectors;
      intersectors.intersect1 = (VirtualCurvePrimitive::Intersect1Ty)&CurveNiIntersector1<N>::template intersect_t<Ribbon1Intersector1<Curve3fa>, Intersect1EpilogMU<VSIZEX,true> >;
      intersectors.occluded1  = (VirtualCurvePrimitive::Occluded1Ty) &CurveNiIntersector1<N>::template occluded_t <Ribbon1Intersector1<Curve3fa>, Occluded1EpilogMU<VSIZEX,true> >;
      intersectors.intersect4 = (VirtualCurvePrimitive::Intersect4Ty)&CurveNiIntersectorK<N,4>::template intersect_t<Ribbon1IntersectorK<Curve3fa,4>, Intersect1KEpilogMU<VSIZEX,4,true> >;
      intersectors.occluded4  = (VirtualCurvePrimitive::Occluded4Ty) &CurveNiIntersectorK<N,4>::template occluded_t <Ribbon1IntersectorK<Curve3fa,4>, Occluded1KEpilogMU<VSIZEX,4,true> >;
#if defined(__AVX__)
      intersectors.intersect8 = (VirtualCurvePrimitive::Intersect8Ty)&CurveNiIntersectorK<N,8>::template intersect_t<Ribbon1IntersectorK<Curve3fa,8>, Intersect1KEpilogMU<VSIZEX,8,true> >;
      intersectors.occluded8  = (VirtualCurvePrimitive::Occluded8Ty) &CurveNiIntersectorK<N,8>::template occluded_t <Ribbon1IntersectorK<Curve3fa,8>, Occluded1KEpilogMU<VSIZEX,8,true> >;
#endif
#if defined(__AVX512F__)
      intersectors.intersect16 = (VirtualCurvePrimitive::Intersect16Ty)&CurveNiIntersectorK<N,16>::template intersect_t<Ribbon1IntersectorK<Curve3fa,16>, Intersect1KEpilogMU<VSIZEX,16,true> >;
      intersectors.occluded16  = (VirtualCurvePrimitive::Occluded16Ty) &CurveNiIntersectorK<N,16>::template occluded_t <Ribbon1IntersectorK<Curve3fa,16>, Occluded1KEpilogMU<VSIZEX,16,true> >;
#endif
      return intersectors;
    }

    template<typename Curve3fa, int N>
    static VirtualCurvePrimitive::Intersectors RibbonNvIntersectors()
    {
      VirtualCurvePrimitive::Intersectors intersectors;
      intersectors.intersect1 = (VirtualCurvePrimitive::Intersect1Ty)&CurveNvIntersector1<N>::template intersect_t<Ribbon1Intersector1<Curve3fa>, Intersect1EpilogMU<VSIZEX,true> >;
      intersectors.occluded1  = (VirtualCurvePrimitive::Occluded1Ty) &CurveNvIntersector1<N>::template occluded_t <Ribbon1Intersector1<Curve3fa>, Occluded1EpilogMU<VSIZEX,true> >;
      intersectors.intersect4 = (VirtualCurvePrimitive::Intersect4Ty)&CurveNvIntersectorK<N,4>::template intersect_t<Ribbon1IntersectorK<Curve3fa,4>, Intersect1KEpilogMU<VSIZEX,4,true> >;
      intersectors.occluded4  = (VirtualCurvePrimitive::Occluded4Ty) &CurveNvIntersectorK<N,4>::template occluded_t <Ribbon1IntersectorK<Curve3fa,4>, Occluded1KEpilogMU<VSIZEX,4,true> >;
#if defined(__AVX__)
      intersectors.intersect8 = (VirtualCurvePrimitive::Intersect8Ty)&CurveNvIntersectorK<N,8>::template intersect_t<Ribbon1IntersectorK<Curve3fa,8>, Intersect1KEpilogMU<VSIZEX,8,true> >;
      intersectors.occluded8  = (VirtualCurvePrimitive::Occluded8Ty) &CurveNvIntersectorK<N,8>::template occluded_t <Ribbon1IntersectorK<Curve3fa,8>, Occluded1KEpilogMU<VSIZEX,8,true> >;
#endif
#if defined(__AVX512F__)
      intersectors.intersect16 = (VirtualCurvePrimitive::Intersect16Ty)&CurveNvIntersectorK<N,16>::template intersect_t<Ribbon1IntersectorK<Curve3fa,16>, Intersect1KEpilogMU<VSIZEX,16,true> >;
      intersectors.occluded16  = (VirtualCurvePrimitive::Occluded16Ty) &CurveNvIntersectorK<N,16>::template occluded_t <Ribbon1IntersectorK<Curve3fa,16>, Occluded1KEpilogMU<VSIZEX,16,true> >;
#endif
      return intersectors;
    }

    template<typename Curve3fa, int N>
    static VirtualCurvePrimitive::Intersectors RibbonNiMBIntersectors()
    {
      VirtualCurvePrimitive::Intersectors intersectors;
      intersectors.intersect1 = (VirtualCurvePrimitive::Intersect1Ty)&CurveNiMBIntersector1<N>::template intersect_t<Ribbon1Intersector1<Curve3fa>, Intersect1EpilogMU<VSIZEX,true> >;
      intersectors.occluded1  = (VirtualCurvePrimitive::Occluded1Ty) &CurveNiMBIntersector1<N>::template occluded_t <Ribbon1Intersector1<Curve3fa>, Occluded1EpilogMU<VSIZEX,true> >;
      intersectors.intersect4 = (VirtualCurvePrimitive::Intersect4Ty)&CurveNiMBIntersectorK<N,4>::template intersect_t<Ribbon1IntersectorK<Curve3fa,4>, Intersect1KEpilogMU<VSIZEX,4,true> >;
      intersectors.occluded4  = (VirtualCurvePrimitive::Occluded4Ty) &CurveNiMBIntersectorK<N,4>::template occluded_t <Ribbon1IntersectorK<Curve3fa,4>, Occluded1KEpilogMU<VSIZEX,4,true> >;
#if defined(__AVX__)
      intersectors.intersect8 = (VirtualCurvePrimitive::Intersect8Ty)&CurveNiMBIntersectorK<N,8>::template intersect_t<Ribbon1IntersectorK<Curve3fa,8>, Intersect1KEpilogMU<VSIZEX,8,true> >;
      intersectors.occluded8  = (VirtualCurvePrimitive::Occluded8Ty) &CurveNiMBIntersectorK<N,8>::template occluded_t <Ribbon1IntersectorK<Curve3fa,8>, Occluded1KEpilogMU<VSIZEX,8,true> >;
#endif
#if defined(__AVX512F__)
      intersectors.intersect16 = (VirtualCurvePrimitive::Intersect16Ty)&CurveNiMBIntersectorK<N,16>::template intersect_t<Ribbon1IntersectorK<Curve3fa,16>, Intersect1KEpilogMU<VSIZEX,16,true> >;
      intersectors.occluded16  = (VirtualCurvePrimitive::Occluded16Ty) &CurveNiMBIntersectorK<N,16>::template occluded_t <Ribbon1IntersectorK<Curve3fa,16>, Occluded1KEpilogMU<VSIZEX,16,true> >;
#endif
      return intersectors;
    }

    template<typename Curve3fa, int N>
    static VirtualCurvePrimitive::Intersectors CurveNiIntersectors()
    {
      VirtualCurvePrimitive::Intersectors intersectors;
      intersectors.intersect1 = (VirtualCurvePrimitive::Intersect1Ty)&CurveNiIntersector1<N>::template intersect_t<BezierCurve1Intersector1<Curve3fa>, Intersect1Epilog1<true> >;
      intersectors.occluded1  = (VirtualCurvePrimitive::Occluded1Ty) &CurveNiIntersector1<N>::template occluded_t <BezierCurve1Intersector1<Curve3fa>, Occluded1Epilog1<true> >;
      intersectors.intersect4 = (VirtualCurvePrimitive::Intersect4Ty)&CurveNiIntersectorK<N,4>::template intersect_t<BezierCurve1IntersectorK<Curve3fa,4>, Intersect1KEpilog1<4,true> >;
      intersectors.occluded4  = (VirtualCurvePrimitive::Occluded4Ty) &CurveNiIntersectorK<N,4>::template occluded_t <BezierCurve1IntersectorK<Curve3fa,4>, Occluded1KEpilog1<4,true> >;
#if defined(__AVX__)
      intersectors.intersect8 = (VirtualCurvePrimitive::Intersect8Ty)&CurveNiIntersectorK<N,8>::template intersect_t<BezierCurve1IntersectorK<Curve3fa,8>, Intersect1KEpilog1<8,true> >;
      intersectors.occluded8  = (VirtualCurvePrimitive::Occluded8Ty) &CurveNiIntersectorK<N,8>::template occluded_t <BezierCurve1IntersectorK<Curve3fa,8>, Occluded1KEpilog1<8,true> >;
#endif
#if defined(__AVX512F__)
      intersectors.intersect16 = (VirtualCurvePrimitive::Intersect16Ty)&CurveNiIntersectorK<N,16>::template intersect_t<BezierCurve1IntersectorK<Curve3fa,16>, Intersect1KEpilog1<16,true> >;
      intersectors.occluded16  = (VirtualCurvePrimitive::Occluded16Ty) &CurveNiIntersectorK<N,16>::template occluded_t <BezierCurve1IntersectorK<Curve3fa,16>, Occluded1KEpilog1<16,true> >;
#endif
      return intersectors;
    }

    template<typename Curve3fa, int N>
    static VirtualCurvePrimitive::Intersectors CurveNvIntersectors()
    {
      VirtualCurvePrimitive::Intersectors intersectors;
      intersectors.intersect1 = (VirtualCurvePrimitive::Intersect1Ty)&CurveNvIntersector1<N>::template intersect_t<BezierCurve1Intersector1<Curve3fa>, Intersect1Epilog1<true> >;
      intersectors.occluded1  = (VirtualCurvePrimitive::Occluded1Ty) &CurveNvIntersector1<N>::template occluded_t <BezierCurve1Intersector1<Curve3fa>, Occluded1Epilog1<true> >;
      intersectors.intersect4 = (VirtualCurvePrimitive::Intersect4Ty)&CurveNvIntersectorK<N,4>::template intersect_t<BezierCurve1IntersectorK<Curve3fa,4>, Intersect1KEpilog1<4,true> >;
      intersectors.occluded4  = (VirtualCurvePrimitive::Occluded4Ty) &CurveNvIntersectorK<N,4>::template occluded_t <BezierCurve1IntersectorK<Curve3fa,4>, Occluded1KEpilog1<4,true> >;
#if defined(__AVX__)
      intersectors.intersect8 = (VirtualCurvePrimitive::Intersect8Ty)&CurveNvIntersectorK<N,8>::template intersect_t<BezierCurve1IntersectorK<Curve3fa,8>, Intersect1KEpilog1<8,true> >;
      intersectors.occluded8  = (VirtualCurvePrimitive::Occluded8Ty) &CurveNvIntersectorK<N,8>::template occluded_t <BezierCurve1IntersectorK<Curve3fa,8>, Occluded1KEpilog1<8,true> >;
#endif
#if defined(__AVX512F__)
      intersectors.intersect16 = (VirtualCurvePrimitive::Intersect16Ty)&CurveNvIntersectorK<N,16>::template intersect_t<BezierCurve1IntersectorK<Curve3fa,16>, Intersect1KEpilog1<16,true> >;
      intersectors.occluded16  = (VirtualCurvePrimitive::Occluded16Ty) &CurveNvIntersectorK<N,16>::template occluded_t <BezierCurve1IntersectorK<Curve3fa,16>, Occluded1KEpilog1<16,true> >;
#endif
      return intersectors;
    }

    template<typename Curve3fa, int N>
    static VirtualCurvePrimitive::Intersectors CurveNiMBIntersectors()
    {
      VirtualCurvePrimitive::Intersectors intersectors;
      intersectors.intersect1 = (VirtualCurvePrimitive::Intersect1Ty)&CurveNiMBIntersector1<N>::template intersect_t<BezierCurve1Intersector1<Curve3fa>, Intersect1Epilog1<true> >;
      intersectors.occluded1  = (VirtualCurvePrimitive::Occluded1Ty) &CurveNiMBIntersector1<N>::template occluded_t <BezierCurve1Intersector1<Curve3fa>, Occluded1Epilog1<true> >;
      intersectors.intersect4 = (VirtualCurvePrimitive::Intersect4Ty)&CurveNiMBIntersectorK<N,4>::template intersect_t<BezierCurve1IntersectorK<Curve3fa,4>, Intersect1KEpilog1<4,true> >;
      intersectors.occluded4  = (VirtualCurvePrimitive::Occluded4Ty) &CurveNiMBIntersectorK<N,4>::template occluded_t <BezierCurve1IntersectorK<Curve3fa,4>, Occluded1KEpilog1<4,true> >;
#if defined(__AVX__)
      intersectors.intersect8 = (VirtualCurvePrimitive::Intersect8Ty)&CurveNiMBIntersectorK<N,8>::template intersect_t<BezierCurve1IntersectorK<Curve3fa,8>, Intersect1KEpilog1<8,true> >;
      intersectors.occluded8  = (VirtualCurvePrimitive::Occluded8Ty) &CurveNiMBIntersectorK<N,8>::template occluded_t <BezierCurve1IntersectorK<Curve3fa,8>, Occluded1KEpilog1<8,true> >;
#endif
#if defined(__AVX512F__)
      intersectors.intersect16 = (VirtualCurvePrimitive::Intersect16Ty)&CurveNiMBIntersectorK<N,16>::template intersect_t<BezierCurve1IntersectorK<Curve3fa,16>, Intersect1KEpilog1<16,true> >;
      intersectors.occluded16  = (VirtualCurvePrimitive::Occluded16Ty) &CurveNiMBIntersectorK<N,16>::template occluded_t <BezierCurve1IntersectorK<Curve3fa,16>, Occluded1KEpilog1<16,true> >;
#endif
      return intersectors;
    }

    template<typename Curve3fa, int N>
    static VirtualCurvePrimitive::Intersectors OrientedCurveNiIntersectors()
    {
      VirtualCurvePrimitive::Intersectors intersectors;
      intersectors.intersect1 = (VirtualCurvePrimitive::Intersect1Ty)&CurveNiIntersector1<N>::template intersect_t2<OrientedBezierCurve1Intersector1, Intersect1Epilog1<true> >;
      intersectors.occluded1  = (VirtualCurvePrimitive::Occluded1Ty) &CurveNiIntersector1<N>::template occluded_t2<OrientedBezierCurve1Intersector1, Occluded1Epilog1<true> >;
      //intersectors.intersect4 = (VirtualCurvePrimitive::Intersect4Ty)&CurveNiIntersectorK<N,4>::template intersect_t<OrientedBezierCurve1IntersectorK<Curve3fa,4>, Intersect1KEpilog1<4,true> >;
      //intersectors.occluded4  = (VirtualCurvePrimitive::Occluded4Ty) &CurveNiIntersectorK<N,4>::template occluded_t <OrientedBezierCurve1IntersectorK<Curve3fa,4>, Occluded1KEpilog1<4,true> >;
#if defined(__AVX__)
      //intersectors.intersect8 = (VirtualCurvePrimitive::Intersect8Ty)&CurveNiIntersectorK<N,8>::template intersect_t<OrientedBezierCurve1IntersectorK<Curve3fa,8>, Intersect1KEpilog1<8,true> >;
      //intersectors.occluded8  = (VirtualCurvePrimitive::Occluded8Ty) &CurveNiIntersectorK<N,8>::template occluded_t <OrientedBezierCurve1IntersectorK<Curve3fa,8>, Occluded1KEpilog1<8,true> >;
#endif
#if defined(__AVX512F__)
      //intersectors.intersect16 = (VirtualCurvePrimitive::Intersect16Ty)&CurveNiIntersectorK<N,16>::template intersect_t<OrientedBezierCurve1IntersectorK<Curve3fa,16>, Intersect1KEpilog1<16,true> >;
      //intersectors.occluded16  = (VirtualCurvePrimitive::Occluded16Ty) &CurveNiIntersectorK<N,16>::template occluded_t <OrientedBezierCurve1IntersectorK<Curve3fa,16>, Occluded1KEpilog1<16,true> >;
#endif
      return intersectors;
    }

    VirtualCurvePrimitive* VirtualCurvePrimitiveIntersector4i()
    {
      static VirtualCurvePrimitive prim;
      prim.vtbl[Geometry::GTY_ROUND_BEZIER_CURVE] = CurveNiIntersectors <BezierCurve3fa,4>();
      prim.vtbl[Geometry::GTY_FLAT_BEZIER_CURVE ] = RibbonNiIntersectors<BezierCurve3fa,4>();
      prim.vtbl[Geometry::GTY_ORIENTED_BEZIER_CURVE] = OrientedCurveNiIntersectors<BezierCurve3fa,4>();
      prim.vtbl[Geometry::GTY_ROUND_BSPLINE_CURVE] = CurveNiIntersectors <BSplineCurve3fa,4>();
      prim.vtbl[Geometry::GTY_FLAT_BSPLINE_CURVE ] = RibbonNiIntersectors<BSplineCurve3fa,4>();
      return &prim;
    }

    VirtualCurvePrimitive* VirtualCurvePrimitiveIntersector4v()
    {
      static VirtualCurvePrimitive prim;
      prim.vtbl[Geometry::GTY_ROUND_BEZIER_CURVE] = CurveNvIntersectors <BezierCurve3fa,4>();
      prim.vtbl[Geometry::GTY_FLAT_BEZIER_CURVE ] = RibbonNvIntersectors<BezierCurve3fa,4>();
      prim.vtbl[Geometry::GTY_ROUND_BSPLINE_CURVE] = CurveNvIntersectors <BSplineCurve3fa,4>();
      prim.vtbl[Geometry::GTY_FLAT_BSPLINE_CURVE ] = RibbonNvIntersectors<BSplineCurve3fa,4>();
      return &prim;
    }

    VirtualCurvePrimitive* VirtualCurvePrimitiveIntersector4iMB()
    {
      static VirtualCurvePrimitive prim;
      prim.vtbl[Geometry::GTY_ROUND_BEZIER_CURVE] = CurveNiMBIntersectors <BezierCurve3fa,4>();
      prim.vtbl[Geometry::GTY_FLAT_BEZIER_CURVE ] = RibbonNiMBIntersectors<BezierCurve3fa,4>();
      prim.vtbl[Geometry::GTY_ROUND_BSPLINE_CURVE] = CurveNiMBIntersectors <BSplineCurve3fa,4>();
      prim.vtbl[Geometry::GTY_FLAT_BSPLINE_CURVE ] = RibbonNiMBIntersectors<BSplineCurve3fa,4>();
      return &prim;
    }

#if defined (__AVX__)
    
    VirtualCurvePrimitive* VirtualCurvePrimitiveIntersector8i()
    {
      static VirtualCurvePrimitive prim;
      prim.vtbl[Geometry::GTY_ROUND_BEZIER_CURVE] = CurveNiIntersectors <BezierCurve3fa,8>();
      prim.vtbl[Geometry::GTY_FLAT_BEZIER_CURVE ] = RibbonNiIntersectors<BezierCurve3fa,8>();
      prim.vtbl[Geometry::GTY_ORIENTED_BEZIER_CURVE] = OrientedCurveNiIntersectors<BezierCurve3fa,8>();
      prim.vtbl[Geometry::GTY_ROUND_BSPLINE_CURVE] = CurveNiIntersectors <BSplineCurve3fa,8>();
      prim.vtbl[Geometry::GTY_FLAT_BSPLINE_CURVE ] = RibbonNiIntersectors<BSplineCurve3fa,8>();
      return &prim;
    }

    VirtualCurvePrimitive* VirtualCurvePrimitiveIntersector8v()
    {
      static VirtualCurvePrimitive prim;
      prim.vtbl[Geometry::GTY_ROUND_BEZIER_CURVE] = CurveNvIntersectors <BezierCurve3fa,8>();
      prim.vtbl[Geometry::GTY_FLAT_BEZIER_CURVE ] = RibbonNvIntersectors<BezierCurve3fa,8>();
      prim.vtbl[Geometry::GTY_ROUND_BSPLINE_CURVE] = CurveNvIntersectors <BSplineCurve3fa,8>();
      prim.vtbl[Geometry::GTY_FLAT_BSPLINE_CURVE ] = RibbonNvIntersectors<BSplineCurve3fa,8>();
      return &prim;
    }
    
    VirtualCurvePrimitive* VirtualCurvePrimitiveIntersector8iMB()
    {
      static VirtualCurvePrimitive prim;
      prim.vtbl[Geometry::GTY_ROUND_BEZIER_CURVE] = CurveNiMBIntersectors <BezierCurve3fa,8>();
      prim.vtbl[Geometry::GTY_FLAT_BEZIER_CURVE ] = RibbonNiMBIntersectors<BezierCurve3fa,8>();
      prim.vtbl[Geometry::GTY_ROUND_BSPLINE_CURVE] = CurveNiMBIntersectors <BSplineCurve3fa,8>();
      prim.vtbl[Geometry::GTY_FLAT_BSPLINE_CURVE ] = RibbonNiMBIntersectors<BSplineCurve3fa,8>();
      return &prim;
    }
  
#endif
  }
}
