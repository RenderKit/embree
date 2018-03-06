% RTC_GEOMETRY_TYPE_*_CURVE(3) | Embree Ray Tracing Kernels 3

#### NAME

    RTC_GEOMETRY_TYPE_ROUND_BEZIER_CURVE -
      Bézier curve geometry type using a sweep surface

    RTC_GEOMETRY_TYPE_ROUND_BSPLINE_CURVE -
      B-spline curve geometry type using a sweep surface
    
    RTC_GEOMETRY_TYPE_FLAT_LINEAR_CURVE -
      linear curve geometry type

    RTC_GEOMETRY_TYPE_FLAT_BEZIER_CURVE -
      Bézier curve geometry type using a ribbon approximation

    RTC_GEOMETRY_TYPE_FLAT_BSPLINE_CURVE - 
      B-spline curve geometry type using a ribbon approximation

#### SYNOPSIS

    #include <embree3/rtcore.h>

    rtcNewGeometry(device, RTC_GEOMETRY_TYPE_ROUND_BEZIER_CURVE);
    rtcNewGeometry(device, RTC_GEOMETRY_TYPE_ROUND_BSPLINE_CURVE);
    rtcNewGeometry(device, RTC_GEOMETRY_TYPE_FLAT_LINEAR_CURVE);
    rtcNewGeometry(device, RTC_GEOMETRY_TYPE_FLAT_BEZIER_CURVE);
    rtcNewGeometry(device, RTC_GEOMETRY_TYPE_FLAT_BSPLINE_CURVE);

#### DESCRIPTION

Curves with per vertex radii are supported with linear, cubic Bézier,
and cubic B-spline bases. Such curve geometries are created by passing
`RTC_GEOMETRY_TYPE_FLAT_LINEAR_CURVE`,
`RTC_GEOMETRY_TYPE_FLAT_BEZIER_CURVE`,
`RTC_GEOMETRY_TYPE_FLAT_BSPLINE_CURVE`,
`RTC_GEOMETRY_TYPE_ROUND_BEZIER_CURVE`, or
`RTC_GEOMETRY_TYPE_ROUND_BSPLINE_CURVE`, to the `rtcNewGeometry`
function. The curve indices can be specified through an index buffer
(`RTC_BUFFER_TYPE_INDEX`) and the curve vertices through a vertex
buffer (`RTC_BUFFER_TYPE_VERTEX`). See `rtcSetGeometryBuffer` and
`rtcSetSharedGeometryBuffer` for more details on how to set
buffers.

The index buffer contains an array of 32-bit indices (`RTC_FORMAT_UINT`
format), each pointing to the ID of the first control vertex. The
vertex buffer stores each control vertex in the form of a single
precision position and radius stored in `x`, `y`, `z`, `r` order in
memory (`RTC_FORMAT_FLOAT4` format). The number of vertices is inferred
from the size of this buffer. The radii may be smaller than zero for the
B-Spline basis, but the interpolated radii should always be greater or
equal to zero.

The `RTC_GEOMETRY_TYPE_FLAT_*` flat mode is a fast mode designed to
render distant hair. In this mode the curve is rendered as a connected
sequence of ray facing quads. Individual quads are considered to have
subpixel size, and zooming onto the curve might show geometric
artifacts. The number of quads to subdivide into can be specified
through the `rtcSetGeometryTessellationRate` function. By default the
tessellation rate is 4.

In the `RTC_GEOMETRY_TYPE_ROUND_*` round mode, a real geometric
surface is rendered for the curve, which is more expensive but allows
closeup views. For the Bézier and B-spline bases, this mode renders a
sweep surface by sweeping a varying radius circle tangential along the
curve. As a limitation, the radius of the curve has to be smaller than
the curvature radius of the curve at each location on the curve. The
round mode is currently not supported for the linear basis.

The intersection with the curve segment stores the parametric hit
location along the curve segment as u-coordinate (range 0 to +1).

For Bézier and B-spline curves, the v-coordinate is set to the
normalized distance in the range -1 to +1. For the linear basis and in
round mode the v-coordinate is set to zero.

In flat mode, the geometry normal `Ng` is set to the tangent of the
curve at the hit location. In round mode, the geometry normal `Ng` is
set to the non-normalized geometric normal of the surface.

For multi-segment motion blur, the number of time steps must be first
specified using the `rtcSetGeometryTimeStepCount` call. Then a vertex
buffer for each time step can be set using different buffer slots, and
all these buffers must have the same stride and size.

Also see tutorials [Hair] and [Curves] for examples of how to create and
use curve geometries.

#### EXIT STATUS

On failure `NULL` is returned and an error code is set that can be
queried using `rtcDeviceGetError`.

#### SEE ALSO

[rtcNewGeometry]
