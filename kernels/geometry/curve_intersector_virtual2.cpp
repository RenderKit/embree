// Copyright 2009-2020 Intel Corporation
// SPDX-License-Identifier: Apache-2.0
 
#include "curve_intersector_virtual.h"
#include "intersector_epilog.h"

#include "../subdiv/bezier_curve.h"
#include "../subdiv/bspline_curve.h"
#include "../subdiv/hermite_curve.h"
#include "../subdiv/catmullrom_curve.h"

#include "spherei_intersector.h"
#include "disci_intersector.h"

#include "linei_intersector.h"

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
#if defined (__AVX__)
    
    VirtualCurveIntersector* VirtualCurveIntersector8i()
    {
      static VirtualCurveIntersector function_local_static_prim;
      function_local_static_prim.vtbl[Geometry::GTY_SPHERE_POINT] = SphereNiIntersectors<8>();
      function_local_static_prim.vtbl[Geometry::GTY_DISC_POINT] = DiscNiIntersectors<8>();
      function_local_static_prim.vtbl[Geometry::GTY_ORIENTED_DISC_POINT] = OrientedDiscNiIntersectors<8>();
      function_local_static_prim.vtbl[Geometry::GTY_ROUND_LINEAR_CURVE ] = LinearConeNiIntersectors<8>();
      function_local_static_prim.vtbl[Geometry::GTY_FLAT_LINEAR_CURVE ] = LinearRibbonNiIntersectors<8>();
      function_local_static_prim.vtbl[Geometry::GTY_ROUND_BEZIER_CURVE] = CurveNiIntersectors <BezierCurve3fa,8>();
      function_local_static_prim.vtbl[Geometry::GTY_FLAT_BEZIER_CURVE ] = RibbonNiIntersectors<BezierCurve3fa,8>();
      function_local_static_prim.vtbl[Geometry::GTY_ORIENTED_BEZIER_CURVE] = OrientedCurveNiIntersectors<BezierCurve3fa,8>();
      function_local_static_prim.vtbl[Geometry::GTY_ROUND_BSPLINE_CURVE] = CurveNiIntersectors <BSplineCurve3fa,8>();
      function_local_static_prim.vtbl[Geometry::GTY_FLAT_BSPLINE_CURVE ] = RibbonNiIntersectors<BSplineCurve3fa,8>();
      function_local_static_prim.vtbl[Geometry::GTY_ORIENTED_BSPLINE_CURVE] = OrientedCurveNiIntersectors<BSplineCurve3fa,8>();
      function_local_static_prim.vtbl[Geometry::GTY_ROUND_HERMITE_CURVE] = HermiteCurveNiIntersectors <HermiteCurve3fa,8>();
      function_local_static_prim.vtbl[Geometry::GTY_FLAT_HERMITE_CURVE ] = HermiteRibbonNiIntersectors<HermiteCurve3fa,8>();
      function_local_static_prim.vtbl[Geometry::GTY_ORIENTED_HERMITE_CURVE] = HermiteOrientedCurveNiIntersectors<HermiteCurve3fa,8>();
      function_local_static_prim.vtbl[Geometry::GTY_ROUND_CATMULL_ROM_CURVE] = CurveNiIntersectors <CatmullRomCurve3fa,8>();
      function_local_static_prim.vtbl[Geometry::GTY_FLAT_CATMULL_ROM_CURVE ] = RibbonNiIntersectors<CatmullRomCurve3fa,8>();
      function_local_static_prim.vtbl[Geometry::GTY_ORIENTED_CATMULL_ROM_CURVE] = OrientedCurveNiIntersectors<CatmullRomCurve3fa,8>();
      return &function_local_static_prim;
    }

    VirtualCurveIntersector* VirtualCurveIntersector8v()
    {
      static VirtualCurveIntersector function_local_static_prim;
      function_local_static_prim.vtbl[Geometry::GTY_SPHERE_POINT] = SphereNiIntersectors<8>();
      function_local_static_prim.vtbl[Geometry::GTY_DISC_POINT] = DiscNiIntersectors<8>();
      function_local_static_prim.vtbl[Geometry::GTY_ORIENTED_DISC_POINT] = OrientedDiscNiIntersectors<8>();
      function_local_static_prim.vtbl[Geometry::GTY_ROUND_LINEAR_CURVE ] = LinearConeNiIntersectors<8>();
      function_local_static_prim.vtbl[Geometry::GTY_FLAT_LINEAR_CURVE ] = LinearRibbonNiIntersectors<8>();
      function_local_static_prim.vtbl[Geometry::GTY_ROUND_BEZIER_CURVE] = CurveNvIntersectors <BezierCurve3fa,8>();
      function_local_static_prim.vtbl[Geometry::GTY_FLAT_BEZIER_CURVE ] = RibbonNvIntersectors<BezierCurve3fa,8>();
      function_local_static_prim.vtbl[Geometry::GTY_ORIENTED_BEZIER_CURVE] = OrientedCurveNiIntersectors<BezierCurve3fa,8>();
      function_local_static_prim.vtbl[Geometry::GTY_ROUND_BSPLINE_CURVE] = CurveNvIntersectors <BSplineCurve3fa,8>();
      function_local_static_prim.vtbl[Geometry::GTY_FLAT_BSPLINE_CURVE ] = RibbonNvIntersectors<BSplineCurve3fa,8>();
      function_local_static_prim.vtbl[Geometry::GTY_ORIENTED_BSPLINE_CURVE] = OrientedCurveNiIntersectors<BSplineCurve3fa,8>();
      function_local_static_prim.vtbl[Geometry::GTY_ROUND_HERMITE_CURVE] = HermiteCurveNiIntersectors <HermiteCurve3fa,8>();
      function_local_static_prim.vtbl[Geometry::GTY_FLAT_HERMITE_CURVE ] = HermiteRibbonNiIntersectors<HermiteCurve3fa,8>();
      function_local_static_prim.vtbl[Geometry::GTY_ORIENTED_HERMITE_CURVE] = HermiteOrientedCurveNiIntersectors<HermiteCurve3fa,8>();
      function_local_static_prim.vtbl[Geometry::GTY_ROUND_CATMULL_ROM_CURVE] = CurveNiIntersectors <CatmullRomCurve3fa,8>();
      function_local_static_prim.vtbl[Geometry::GTY_FLAT_CATMULL_ROM_CURVE ] = RibbonNiIntersectors<CatmullRomCurve3fa,8>();
      function_local_static_prim.vtbl[Geometry::GTY_ORIENTED_CATMULL_ROM_CURVE] = OrientedCurveNiIntersectors<CatmullRomCurve3fa,8>();
      return &function_local_static_prim;
    }
    
    VirtualCurveIntersector* VirtualCurveIntersector8iMB()
    {
      static VirtualCurveIntersector function_local_static_prim;
      function_local_static_prim.vtbl[Geometry::GTY_SPHERE_POINT] = SphereNiMBIntersectors<8>();
      function_local_static_prim.vtbl[Geometry::GTY_DISC_POINT] = DiscNiMBIntersectors<8>();
      function_local_static_prim.vtbl[Geometry::GTY_ORIENTED_DISC_POINT] = OrientedDiscNiMBIntersectors<8>();
      function_local_static_prim.vtbl[Geometry::GTY_ROUND_LINEAR_CURVE ] = LinearConeNiMBIntersectors<8>();
      function_local_static_prim.vtbl[Geometry::GTY_FLAT_LINEAR_CURVE ] = LinearRibbonNiMBIntersectors<8>();
      function_local_static_prim.vtbl[Geometry::GTY_ROUND_BEZIER_CURVE] = CurveNiMBIntersectors <BezierCurve3fa,8>();
      function_local_static_prim.vtbl[Geometry::GTY_FLAT_BEZIER_CURVE ] = RibbonNiMBIntersectors<BezierCurve3fa,8>();
      function_local_static_prim.vtbl[Geometry::GTY_ORIENTED_BEZIER_CURVE] = OrientedCurveNiMBIntersectors<BezierCurve3fa,8>();
      function_local_static_prim.vtbl[Geometry::GTY_ROUND_BSPLINE_CURVE] = CurveNiMBIntersectors <BSplineCurve3fa,8>();
      function_local_static_prim.vtbl[Geometry::GTY_FLAT_BSPLINE_CURVE ] = RibbonNiMBIntersectors<BSplineCurve3fa,8>();
      function_local_static_prim.vtbl[Geometry::GTY_ORIENTED_BSPLINE_CURVE] = OrientedCurveNiMBIntersectors<BSplineCurve3fa,8>();
      function_local_static_prim.vtbl[Geometry::GTY_ROUND_HERMITE_CURVE] = HermiteCurveNiMBIntersectors <HermiteCurve3fa,8>();
      function_local_static_prim.vtbl[Geometry::GTY_FLAT_HERMITE_CURVE ] = HermiteRibbonNiMBIntersectors<HermiteCurve3fa,8>();
      function_local_static_prim.vtbl[Geometry::GTY_ORIENTED_HERMITE_CURVE] = HermiteOrientedCurveNiMBIntersectors<HermiteCurve3fa,8>();
      function_local_static_prim.vtbl[Geometry::GTY_ROUND_CATMULL_ROM_CURVE] = CurveNiMBIntersectors <CatmullRomCurve3fa,8>();
      function_local_static_prim.vtbl[Geometry::GTY_FLAT_CATMULL_ROM_CURVE ] = RibbonNiMBIntersectors<CatmullRomCurve3fa,8>();
      function_local_static_prim.vtbl[Geometry::GTY_ORIENTED_CATMULL_ROM_CURVE] = OrientedCurveNiMBIntersectors<CatmullRomCurve3fa,8>();
      return &function_local_static_prim;
    }
  
#endif
  }
}
