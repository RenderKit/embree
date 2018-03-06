% rtcFilterOcclusion(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcFilterOcclusion - invokes the occlusion filter function

#### SYNOPSIS

    #include <embree3/rtcore.h>

    void rtcFilterOcclusion(
      const struct RTCOccludedFunctionNArguments* args,
      const struct RTCFilterFunctionNArguments* filterArgs
    );

#### DESCRIPTION

The `rtcFilterOcclusion` function can be called inside an
`RTCOccludedFunctionN` callback function to invoke the occlusion filter
registered to the geometry and stored inside the context. For this an
`RTCFilterFunctionNArguments` structure must be created (see
`rtcSetGeometryIntersectFilterFunction`) which basically consists of a
valid mask, a hit packet to filter, the corresponding ray packet, and
the packet size. After the invocation of `rtcFilterOcclusion` only rays
that are still valid (valid mask set to -1) should signal an occlusion.

#### EXIT STATUS

For performance reasons this function does not do any error checks,
thus will not set any error flags on failure.

#### SEE ALSO

[rtcFilterIntersection], [rtcSetGeometryOccludedFunction]
