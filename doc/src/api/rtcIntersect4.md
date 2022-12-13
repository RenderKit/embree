% rtcIntersect4/8/16(3) | Embree Ray Tracing Kernels 4

#### NAME

    rtcIntersect4/8/16 - finds the closest hits for a ray packet

#### SYNOPSIS

    #include <embree4/rtcore.h>

    void rtcIntersect4(
      const int* valid,
      RTCScene scene,
      struct RTCRayHit4* rayhit,
      struct RTCIntersectArguments* args = NULL
    );

    void rtcIntersect8(
      const int* valid,
      RTCScene scene,
      struct RTCRayHit8* rayhit,
      struct RTCIntersectArguments* args = NULL
    );

    void rtcIntersect16(
      const int* valid,
      RTCScene scene,
      struct RTCRayHit16* rayhit,
      struct RTCIntersectArguments* args = NULL
    );

#### DESCRIPTION

The `rtcIntersect4/8/16` functions finds the closest hits for a ray
packet of size 4, 8, or 16 (`rayhit` argument) with the scene (`scene`
argument). The ray/hit input contains a ray packet and hit packet. The
passed optional arguments struct (`args` argument) are used to pass
additional arguments for advanced features. See Section
[rtcIntersect1] for more details and a description of how to set up
and trace rays.

A ray valid mask must be provided (`valid` argument) which stores
one 32-bit integer (`-1` means valid and `0` invalid) per ray in the
packet. Only active rays are processed, and hit data of inactive rays
is not changed.

``` {include=src/api/inc/raypointer.md}
```

For `rtcIntersect4` the ray packet must be aligned to 16 bytes, for
`rtcIntersect8` the alignment must be 32 bytes, and for
`rtcIntersect16` the alignment must be 64 bytes.

The `rtcIntersect4`, `rtcIntersect8` and `rtcIntersect16` functions
may change the ray packet size and ray order when calling back into
filter functions or user geometry callbacks. Under some
conditions the application can assume packets to stay intakt, which
can determined by querying the
`RTC_DEVICE_PROPERTY_NATIVE_RAY4_SUPPORTED`,
`RTC_DEVICE_PROPERTY_NATIVE_RAY8_SUPPORTED`,
`RTC_DEVICE_PROPERTY_NATIVE_RAY16_SUPPORTED` properties through the
`rtcGetDeviceProperty` function. See [rtcGetDeviceProperty] for more
information.

#### EXIT STATUS

For performance reasons this function does not do any error checks,
thus will not set any error flags on failure.

#### SEE ALSO

[rtcIntersect1], [rtcOccluded4/8/16], [rtcInitIntersectArguments]
