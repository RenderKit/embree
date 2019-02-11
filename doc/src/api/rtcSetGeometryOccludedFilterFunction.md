% rtcSetGeometryOccludedFilterFunction(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcSetGeometryOccludedFilterFunction - sets the occlusion filter
      for the geometry

#### SYNOPSIS

    #include <embree3/rtcore.h>

    void rtcSetGeometryOccludedFilterFunction(
      RTCGeometry geometry,
      RTCFilterFunctionN filter
    );

#### DESCRIPTION

The `rtcSetGeometryOccludedFilterFunction` function registers an
occlusion filter callback function (`filter` argument) for the
specified geometry (`geometry` argument).

Only a single callback function can be registered per geometry, and
further invocations overwrite the previously set callback function.
Passing `NULL` as function pointer disables the registered callback
function.

The registered intersection filter function is invoked for every hit
encountered during the `rtcOccluded`-type ray queries and can accept
or reject that hit. The feature can be used to define a silhouette for
a primitive and reject hits that are outside the silhouette. E.g. a
tree leaf could be modeled with an alpha texture that decides whether
hit points lie inside or outside the leaf.

Please see the description of the
`rtcSetGeometryIntersectFilterFunction` for a description of the
filter callback function.

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[rtcSetGeometryIntersectFilterFunction]
