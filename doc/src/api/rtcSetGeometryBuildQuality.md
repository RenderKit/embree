% rtcSetGeometryBuildQuality(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcSetGeometryBuildQuality - sets the build quality for the geometry

#### SYNOPSIS

    #include <embree3/rtcore.h>

    void rtcSetGeometryBuildQuality(
      RTCGeometry geometry,
      enum RTCBuildQuality quality
    );

#### DESCRIPTION

The `rtcSetGeometryBuildQuality` function sets the build quality
(`quality` argument) for the specified geometry (`geometry`
argument). The per-geometry build quality is only a hint and may be
ignored. Embree currently uses the per-geometry build quality when the
scene build quality is set to `RTC_BUILD_QUALITY_LOW`. In this mode a
two-level acceleration structure is build, and geometries build a
separate acceleration structure using the geometry build quality. The
per-geometry build quality can be one of:

+ `RTC_BUILD_QUALITY_LOW`: Creates lower quality data structures,
  e.g. for dynamic scenes.

+ `RTC_BUILD_QUALITY_MEDIUM`: Default build quality for most
  usages. Gives a good compromise between build and render
  performance.

+ `RTC_BUILD_QUALITY_HIGH`: Creates higher quality data structures for
  final-frame rendering. Enables a spatial split builder for certain
  primitive types.

+ `RTC_BUILD_QUALITY_REFIT`: Uses a BVH refitting approach when
  changing only the vertex buffer.

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[rtcSetSceneBuildQuality]
