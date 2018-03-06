% rtcIntersectNM(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcIntersectNM - finds the closest hits for a stream of M
      ray packets of size N

#### SYNOPSIS

    #include <embree3/rtcore.h>

    void rtcIntersectNM(
      RTCScene scene,
      struct RTCIntersectContext* context,
      struct RTCRayHitN* rayhit,
      unsigned int N,
      unsigned int M,
      size_t byteStride
    );

#### DESCRIPTION

The `rtcIntersectNM` function finds the closest hits for a stream of
`M` ray packets (`rayhit` argument) of size `N` with the scene
(`scene` argument). The `rays` argument points to an array of ray and
hit packets with specified byte stride (`byteStride` argument) between the
ray/hit packets. See Section [rtcIntersect1] for a description of how
to set up and trace rays.

``` {include=src/api/inc/context.md}
```

``` {include=src/api/inc/reorder.md}
```

A ray in a ray stream is considered inactive if its `tnear` value is
larger than its `tfar` value.

The packet size `N` must be larger than 0, and the stream size `M`
can be an arbitrary positive integer including 0. Each ray must be
aligned to 16 bytes.

#### EXIT STATUS

For performance reasons this function does not do any error checks,
thus will not set any error flags on failure.

#### SEE ALSO

[rtcOccludedNM]
