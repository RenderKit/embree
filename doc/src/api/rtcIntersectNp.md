% rtcIntersectNp(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcIntersectNp - finds the closest hits for a SOA ray stream of
      size N

#### SYNOPSIS

    #include <embree3/rtcore.h>

    void rtcIntersectNp(
      RTCScene scene,
      struct RTCIntersectContext* context,
      struct RTCRayHitNp* rayhit,
      unsigned int N
    );

#### DESCRIPTION

The `rtcIntersectNp` function finds the closest hits for a SOA ray
stream (`rays` argument) of size `N` (basically a large ray packet)
with the scene (`scene` argument). The `rayhit` argument points to two
structures of pointers with one pointer for each ray and hit component.
Each of these pointers points to an array with the ray or hit component
data for each ray or hit. This way the individual components of the SOA
ray stream do not need to be stored sequentially in memory, which makes
it possible to have large varying size ray packets in SOA layout. See
Section [rtcIntersect1] for a description of how to set up and trace
rays.

``` {include=src/api/inc/context.md}
```

``` {include=src/api/inc/reorder.md}
```

A ray in a ray stream is considered inactive if its `tnear` value is
larger than its `tfar` value.

The stream size `N` can be an arbitrary positive integer including 0.
Each ray component array must be aligned to 16 bytes.

#### EXIT STATUS

For performance reasons this function does not do any error checks,
thus will not set any error flags on failure.

#### SEE ALSO

[rtcOccludedNp]
