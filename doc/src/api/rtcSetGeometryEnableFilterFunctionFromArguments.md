% rtcSetGeometryEnableFilterFunctionFromArguments(3) | Embree Ray Tracing Kernels 4

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
invokation is enabled for the geometry or disabled otherwise. By
default the invokation of the argument filter function is disabled for
some geometry.

The argument filter function invokation can also get enforced for each
geometry by using the
`RTC_RAY_QUERY_FLAG_INVOKE_ARGUMENT_FILTER` ray query flag that can get
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
