% rtcFilterIntersection(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcFilterIntersection - invokes the intersection filter function

#### SYNOPSIS

    #include <embree3/rtcore.h>

    void rtcFilterIntersection(
      const struct RTCIntersectFunctionNArguments* args,
      const struct RTCFilterFunctionNArguments* filterArgs
    );

#### DESCRIPTION

The `rtcFilterIntersection` function can be called inside an
`RTCIntersectFunctionN` callback function to invoke the intersection
filter registered to the geometry and stored inside the context.
For this an `RTCFilterFunctionNArguments` structure must be created
(see `rtcSetGeometryIntersectFilterFunction`) which basically consists
of a valid mask, a hit packet to filter, the corresponding ray packet,
and the packet size. After the invocation of `rtcFilterIntersection`,
only rays that are still valid (valid mask set to -1) should update a
hit.

#### EXIT STATUS

For performance reasons this function does not do any error checks,
thus will not set any error flags on failure.

#### SEE ALSO

[rtcFilterOcclusion], [rtcSetGeometryIntersectFunction]
