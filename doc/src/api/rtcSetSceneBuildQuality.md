% rtcSetSceneBuildQuality(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcSetSceneBuildQuality - sets the build quality for
      the scene

#### SYNOPSIS

    #include <embree3/rtcore.h>

    void rtcSetSceneBuildQuality(
      RTCScene scene,
      enum RTCBuildQuality quality
    );

#### DESCRIPTION

The `rtcSetSceneBuildQuality` function sets the build quality
(`quality` argument) for the specified scene (`scene` argument).
Possible values for the build quality are:

+ `RTC_BUILD_QUALITY_LOW`: Create lower quality data structures,
  e.g. for dynamic scenes. A two-level spatial index structure is
  built when enabling this mode, which supports fast partial scene
  updates, and allows for setting a per-geometry build quality through
  the `rtcSetGeometryBuildQuality` function.

+ `RTC_BUILD_QUALITY_MEDIUM`: Default build quality for most usages.
  Gives a good compromise between build and render performance.

+ `RTC_BUILD_QUALITY_HIGH`: Create higher quality data structures for
  final-frame rendering. For certain geometry types this enables a
  spatial split BVH.

Selecting a higher build quality results in better rendering
performance but slower scene commit times. The default build quality
for a scene is `RTC_BUILD_QUALITY_MEDIUM`.

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[rtcSetGeometryBuildQuality]
