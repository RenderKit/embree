% rtcInvokeOccludedFilterFromGeometry(3) | Embree Ray Tracing Kernels 4

#### NAME

    rtcInvokeOccludedFilterFromGeometry - invokes the occlusion
      filter function from the geometry

#### SYNOPSIS

    #include <embree4/rtcore.h>

    void rtcInvokeOccludedFilterFromGeometry(
      const struct RTCOccludedFunctionNArguments* args,
      const struct RTCFilterFunctionNArguments* filterArgs
    );

#### DESCRIPTION

The `rtcInvokeOccludedFilterFromGeometry` function can be called
inside an `RTCOccludedFunctionN` user geometry callback function to
invoke the occlusion filter registered to the geometry. For this an
`RTCFilterFunctionNArguments` structure must be created (see
`rtcSetGeometryIntersectFilterFunction`) which basically consists of a
valid mask, a hit packet to filter, the corresponding ray packet, and
the packet size. After the invocation of
`rtcInvokeOccludedFilterFromGeometry` only rays that are still valid
(valid mask set to -1) should signal an occlusion.

#### EXIT STATUS

For performance reasons this function does not do any error checks,
thus will not set any error flags on failure.

#### SEE ALSO

[rtcInvokeIntersectFilterFromGeometry], [rtcSetGeometryOccludedFunction]
