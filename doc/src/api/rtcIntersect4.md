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

Further, `rtcIntersect4` is only available when Embree was compiled
with `EMBREE_RAY_PACKETS` turned on in CMake (which is the default).
Further, the `rtcIntersect8` function works only if Embree was
compiled with `AVX` or `AVX2`, and if the CPU it is running on
supports the enabled ISA. Similar, the `rtcIntersect16` function
works only if Embree was compiled with `AVX512SKX` or `AVX512KNL`
and the CPU it is running on supports the enabled ISA. For this reason,
use these functions only after checking whether they are available by
using an `rtcGetDeviceParameter` query.

#### EXIT STATUS

For performance reasons this function does not do any error checks,
thus will not set any error flags on failure.

#### SEE ALSO

[rtcOccluded4/8/16]
