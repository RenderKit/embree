% rtcSetGeometryInstancedScene(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcSetGeometryInstancedScene - sets the instanced scene of
      an instance geometry

#### SYNOPSIS

    #include <embree3/rtcore.h>

    void rtcSetGeometryInstancedScene(
      RTCGeometry geometry,
      RTCScene scene
    );

#### DESCRIPTION

The `rtcSetGeometryInstancedScene` function sets the instanced scene
(`scene` argument) of the specified instance geometry (`geometry`
argument).

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[RTC_GEOMETRY_TYPE_INSTANCE], [rtcSetGeometryTransform]
