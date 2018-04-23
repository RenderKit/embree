% rtcIntersect4/8/16(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcIntersect4/8/16 - finds the closest hits for a ray packet

#### SYNOPSIS

    #include <embree3/rtcore.h>

    void rtcIntersect4(
      const int* valid,
      RTCScene scene,
      struct RTCIntersectContext* context,
      struct RTCRayHit4* rayhit
    );

    void rtcIntersect8(
      const int* valid,
      RTCScene scene,
      struct RTCIntersectContext* context,
      struct RTCRayHit8* rayhit
    );

    void rtcIntersect16(
      const int* valid,
      RTCScene scene,
      struct RTCIntersectContext* context,
      struct RTCRayHit16* rayhit
    );

#### DESCRIPTION

The `rtcIntersect4/8/16` functions finds the closest hits for a ray
packet of size 4, 8, or 16 (`rayhit` argument) with the scene (`scene`
argument). The ray/hit input contains a ray packet and hit packet. See
Section [rtcIntersect1] for a description of how to set up and trace
rays.

A ray valid mask must be provided (`valid` argument) which stores
one 32-bit integer (`-1` means valid and `0` invalid) per ray in the
packet. Only active rays are processed, and hit data of inactive rays
is not changed.

``` {include=src/api/inc/context.md}
```

``` {include=src/api/inc/raypointer.md}
```

The implementation of these functions is guaranteed to invoke callback
functions always with the same ray packet size and ordering of rays as
specified initially.

For `rtcIntersect4` the ray packet must be aligned to 16 bytes, for
`rtcIntersect8` the alignment must be 32 bytes, and for
`rtcIntersect16` the alignment must be 64 bytes.

The `rtcIntersect4`, `rtcIntersect8` and `rtcIntersect16` functions
may change the ray packet size and ray order when calling back into
intersect filter functions or user geometry callbacks. Under some
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

[rtcOccluded4/8/16]
