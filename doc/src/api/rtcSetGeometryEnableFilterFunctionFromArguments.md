% rtcSetGeometryEnableFilterFunctionFromArguments(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcSetGeometryEnableFilterFunctionFromArguments - enables
      argument filter functions for the geometry

#### SYNOPSIS

    #include <embree4/rtcore.h>

    void rtcSetGeometryEnableFilterFunctionFromArguments(
       RTCGeometry geometry, bool enable);

#### DESCRIPTION

This function enables invokation the filter function passed through
`RTCIntersectArguments` or `RTCOccludedArguments` to the intersect and
occluded queries. If enable is true the argument filter function
invokation is enabled for the geometry or disabled otherwise.

The argument filter function invokation can also get enforced for each
geometry by using the
`RTC_INTERSECT_CONTEXT_FLAG_INVOKE_ARGUMENT_FILTER` flag that can get
passed to `rtcIntersect` and `rtcOccluded` functions. See Section
[rtcInitIntersectArguments] and [rtcInitOccludedArguments] for more
details.

In order to use the argument filter function for some scene, that
feature additionally has to get enabled using the
`RTC_SCENE_FLAG_FILTER_FUNCTION_IN_ARGUMENTS` scene flag. See Section
[rtcSetSceneFlags] for more details.

#### EXIT STATUS

On failure an error code is set that can get queried using
`rtcGetDeviceError`.

#### SEE ALSO

[rtcInitIntersectArguments], [rtcInitOccludedArguments], [rtcSetSceneFlags]
