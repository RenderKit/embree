// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0
 
#include "curve_intersector_virtual.h"
#include "intersector_epilog.h"

#include "kernels/subdiv/bezier_curve.h"
#include "kernels/subdiv/bspline_curve.h"
#include "kernels/subdiv/hermite_curve.h"
#include "kernels/subdiv/catmullrom_curve.h"

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
      static VirtualCurveIntersector function_local_static_prim = []()
      {
        VirtualCurveIntersector intersector;
        intersector.vtbl[Geometry::GTY_SPHERE_POINT] = SphereNiIntersectors<8>();
        intersector.vtbl[Geometry::GTY_DISC_POINT] = DiscNiIntersectors<8>();
        intersector.vtbl[Geometry::GTY_ORIENTED_DISC_POINT] = OrientedDiscNiIntersectors<8>();
        intersector.vtbl[Geometry::GTY_CONE_LINEAR_CURVE ] = LinearConeNiIntersectors<8>();
        intersector.vtbl[Geometry::GTY_ROUND_LINEAR_CURVE ] = LinearRoundConeNiIntersectors<8>();
        intersector.vtbl[Geometry::GTY_FLAT_LINEAR_CURVE ] = LinearRibbonNiIntersectors<8>();
        intersector.vtbl[Geometry::GTY_ROUND_BEZIER_CURVE] = CurveNiIntersectors <BezierCurveT,8>();
        intersector.vtbl[Geometry::GTY_FLAT_BEZIER_CURVE ] = RibbonNiIntersectors<BezierCurveT,8>();
        intersector.vtbl[Geometry::GTY_ORIENTED_BEZIER_CURVE] = OrientedCurveNiIntersectors<BezierCurveT,8>();
        intersector.vtbl[Geometry::GTY_ROUND_BSPLINE_CURVE] = CurveNiIntersectors <BSplineCurveT,8>();
        intersector.vtbl[Geometry::GTY_FLAT_BSPLINE_CURVE ] = RibbonNiIntersectors<BSplineCurveT,8>();
        intersector.vtbl[Geometry::GTY_ORIENTED_BSPLINE_CURVE] = OrientedCurveNiIntersectors<BSplineCurveT,8>();
        intersector.vtbl[Geometry::GTY_ROUND_HERMITE_CURVE] = HermiteCurveNiIntersectors <HermiteCurveT,8>();
        intersector.vtbl[Geometry::GTY_FLAT_HERMITE_CURVE ] = HermiteRibbonNiIntersectors<HermiteCurveT,8>();
        intersector.vtbl[Geometry::GTY_ORIENTED_HERMITE_CURVE] = HermiteOrientedCurveNiIntersectors<HermiteCurveT,8>();
        intersector.vtbl[Geometry::GTY_ROUND_CATMULL_ROM_CURVE] = CurveNiIntersectors <CatmullRomCurveT,8>();
        intersector.vtbl[Geometry::GTY_FLAT_CATMULL_ROM_CURVE ] = RibbonNiIntersectors<CatmullRomCurveT,8>();
        intersector.vtbl[Geometry::GTY_ORIENTED_CATMULL_ROM_CURVE] = OrientedCurveNiIntersectors<CatmullRomCurveT,8>();
        return intersector;
      }();
      return &function_local_static_prim;
    }
  
#endif
  }
}
