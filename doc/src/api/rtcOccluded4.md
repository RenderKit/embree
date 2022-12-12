% rtcOccluded4/8/16(3) | Embree Ray Tracing Kernels 4

#### NAME

    rtcOccluded4/8/16 - finds any hits for a ray packet

#### SYNOPSIS

    #include <embree4/rtcore.h>

    void rtcOccluded4(
      const int* valid,
      RTCScene scene,
      struct RTCRay4* ray,
      struct RTCOccludedArguments* args = NULL
    );

    void rtcOccluded8(
      const int* valid,
      RTCScene scene,
      struct RTCRay8* ray,
      struct RTCOccludedArguments* args = NULL
    );

    void rtcOccluded16(
      const int* valid,
      RTCScene scene,
      struct RTCRay16* ray,
      struct RTCOccludedArguments* args = NULL
    );

#### DESCRIPTION

The `rtcOccluded4/8/16` functions checks for each active ray of the
ray packet of size 4, 8, or 16 (`ray` argument) whether there is any
hit with the scene (`scene` argument). The passed optional arguments
struct (`args` argument) can get used for advanced use cases, see
section [rtcInitOccludedArguments] for more details. See Section
[rtcOccluded1] for more details and a description of how to set up and
trace occlusion rays.

A ray valid mask must be provided (`valid` argument) which stores
one 32-bit integer (`-1` means valid and `0` invalid) per ray in the
packet. Only active rays are processed, and hit data of inactive rays
is not changed.

``` {include=src/api/inc/raypointer.md}
```

For `rtcOccluded4` the ray packet must be aligned to 16 bytes, for
`rtcOccluded8` the alignment must be 32 bytes, and for `rtcOccluded16`
the alignment must be 64 bytes.

The `rtcOccluded4`, `rtcOccluded8` and `rtcOccluded16` functions
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

[rtcOccluded1], [rtcIntersect4/8/16], [rtcInitOccludedArguments]
