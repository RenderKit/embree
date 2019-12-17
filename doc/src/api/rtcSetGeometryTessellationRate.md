% rtcSetGeometryTessellationRate(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcSetGeometryTessellationRate - sets the tessellation rate of the
      geometry

#### SYNOPSIS

    #include <embree3/rtcore.h>

    void rtcSetGeometryTessellationRate(
      RTCGeometry geometry,
      float tessellationRate
    );

#### DESCRIPTION

The `rtcSetGeometryTessellationRate` function sets the tessellation
rate (`tessellationRate` argument) for the specified geometry
(`geometry` argument). The tessellation rate can only be set for flat
curves and subdivision geometries. For curves, the tessellation rate
specifies the number of ray-facing quads per curve segment. For
subdivision surfaces, the tessellation rate specifies the number of
quads along each edge.

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[RTC_GEOMETRY_TYPE_CURVE], [RTC_GEOMETRY_TYPE_SUBDIVISION]
