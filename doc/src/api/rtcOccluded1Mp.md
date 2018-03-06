% rtcOccluded1Mp(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcOccluded1Mp - find any hits for a stream of M pointers to
      single rays

#### SYNOPSIS

    #include <embree3/rtcore.h>

    void rtcOccluded1M(
      RTCScene scene,
      struct RTCIntersectContext* context,
      struct RTCRay** ray,
      unsigned int M
    );

#### DESCRIPTION

The `rtcOccluded1Mp` function checks whether there are any hits for a
stream of `M` single rays (`ray` argument) with the scene (`scene`
argument). The `ray` argument points to an array of pointers to
rays. Section [rtcOccluded1] for a description of how to set up and
trace a occlusion rays.

``` {include=src/api/inc/context.md}
```

``` {include=src/api/inc/reorder.md}
```

A ray in a ray stream is considered inactive if its `tnear` value is
larger than its `tfar` value.

The stream size `M` can be an arbitrary positive integer including 0.
Each ray must be aligned to 16 bytes.

#### EXIT STATUS

For performance reasons this function does not do any error checks,
thus will not set any error flags on failure.

#### SEE ALSO

[rtcIntersect1Mp]
