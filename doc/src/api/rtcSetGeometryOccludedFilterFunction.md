% rtcSetGeometryOccludedFilterFunction(3) | Embree Ray Tracing Kernels 4

#### NAME

    rtcSetGeometryOccludedFilterFunction - sets the occlusion filter
      for the geometry

#### SYNOPSIS

    #include <embree4/rtcore.h>

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

The registered occlusion filter function is invoked for every hit
encountered during the `rtcOccluded`-type ray queries and can accept
or reject that hit. The feature can be used to define a silhouette for
a primitive and reject hits that are outside the silhouette. E.g. a
tree leaf could be modeled with an alpha texture that decides whether
hit points lie inside or outside the leaf.

Please see the description of the
`rtcSetGeometryIntersectFilterFunction` for a description of the
filter callback function.

The `rtcOccluded`-type functions terminate traversal when a hit got
committed. As filter functions can only set the `tfar` distance of the
ray for a committed hit, the occlusion filter cannot influence the
`tfar` value of subsequent traversal, as that directly ends. For that
reason `rtcOccluded` and occlusion filters cannot get used to gather
the next n-hits, and `rtcIntersect` and intersection filters should
get used instead.

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[rtcSetGeometryIntersectFilterFunction]
