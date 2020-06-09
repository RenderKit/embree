% rtcSetGeometryMaxRadiusScale(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcSetGeometryMaxRadiusScale - assigns a maximal curve radius scale factor for min-width feature

#### SYNOPSIS

    #include <embree3/rtcore.h>

    void rtcSetGeometryMaxRadiusScale(RTCGeometry geometry, float maxRadiusScale);

#### DESCRIPTION

The `rtcSetMaxGeometryScale` function specifies a maximal scaling
factor for curve radii used by the min-width feature.

The min-width feature can increase the radius of curves and points, in
order to reduce aliasing and improve render times. The feature is
disabled by default and has to get enabled using the
EMBREE_MIN_WIDTH cmake option.

To use the feature, one has to specify a maximal curve radius scaling
factor using the [rtcSetGeometryMaxRadiusScale] function. This factor
should be a small number (e.g. 4) as the constructed BVH bounds get
increased in order to bound the curve in the worst case of maximal
radii.

One also has to set the minWidthDistanceFactor in the
RTCIntersectContext when tracing a ray. This factor controls the
target radius size of a curve or point at some distance away of the
ray origin.

For each control point p with radius r of a curve or point primitive,
the primitive intersectors first calculate a target radius r' as:

    r' = length(p-ray_org) * minWidthDistanceFactor

Typically the minWidthDistanceFactor is set by the application such
that the target radius projects to the width of half a pixel (thus
primitive diameter is pixel sized).

The target radius r' is then clamped against the minimal bound r and
maximal bound maxRadiusScale*r to obtain the final radius r'':

    r'' = max(r, min(r', maxRadiusScale*r))

Thus curves or points close to the camera are rendered with a normal
radii r, and curves or points far from the camera are not enlarged too
much, as this would be very expensive to render.

When `rtcSetGeometryMaxRadiusScale` function is not invoked for a
curve or point geometry (or if the maximal scaling factor is set to
1.0), then the curve or point geometry renders normally, with radii
not modified by the min-width feature.

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[rtcInitIntersectContext]

